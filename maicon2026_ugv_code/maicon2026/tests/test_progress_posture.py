"""Recovery policy boundaries; these do not assert simulated match success."""
import json
import math

import pytest

from maicon26.config import Config
from maicon26.mission.posture import Posture, PostureController, blocker_fire_allowed
from maicon26.nav.progress import ProgressMonitor


def _enabled():
    cfg = Config()
    cfg.progress.enabled = True
    return cfg


def test_ladder_thresholds_actions_once_and_progress_reset():
    p = ProgressMonitor(_enabled(), "B2")
    p.update(0, "face42", 2.0)
    for at, stage, action in ((3, 1, "replan"), (6, 2, "engage"), (9, 3, "backoff")):
        before = p.update(at - .01, "face42", 2.0, physically_blocked=True)
        assert before.stage == stage - 1
        hit = p.update(at, "face42", 2.0, physically_blocked=True)
        assert hit.stage == stage and hit.actions == (action,)
        assert p.update(at + .01, "face42", 2.0, physically_blocked=True).actions == ()
    # Three small reductions accumulate; increased/replanned path length does
    # not reset the anchor and cannot manufacture progress while going backward.
    assert not p.update(10, "face42", 1.94).progressed
    assert not p.update(10.5, "face42", 2.20).progressed
    assert not p.update(11, "face42", 1.90).progressed
    reset = p.update(11.5, "face42", 1.85)
    assert reset.progressed and reset.stage == 0 and reset.stalled_s == 0
    assert p.update(14.5, "face42", 1.85).actions == ("replan",)


def test_goal_abandon_once_cooldown_and_second_failure_excludes_match():
    p = ProgressMonitor(_enabled(), "B1")
    p.update(0, "g", 1.0)
    s = p.update(13, "g", 1.0)
    assert s.stage == 4 and s.actions == ("replan", "engage", "abandon")
    assert not s.goal_available and p.failures["g"] == 1
    for t in (14, 20, 42):
        assert p.update(t, "g", 1.0).actions == ()
    assert p.eligible("g", 43)
    p.update(43, "other", 1.0)
    p.update(44, "g", 1.0)
    assert p.update(57, "g", 1.0).actions[-1] == "abandon"
    assert p.failures["g"] == 2 and not p.eligible("g", 10_000)
    p.reset(58)
    assert not p.eligible("g", 10_000)


def test_two_second_goal_churn_still_reaches_all_recovery_thresholds():
    p = ProgressMonitor(_enabled())
    seen=[]
    for t in range(14):
        # Artificially shorter paths belong to different goals and cannot
        # manufacture progress. Every other tick changes the selected goal.
        goal="g"+str(t//2)
        state=p.update(t,goal,100.-10.*(t//2),physically_blocked=True)
        seen.extend((t,action) for action in state.actions)
        assert not state.progressed
    assert seen==[(3,"replan"),(6,"engage"),(9,"backoff"),(13,"abandon")]
    assert p.failures["g6"]==1 and not p.eligible("g6",20.)
    fresh=p.update(14,"new_eligible_goal",2.)
    assert fresh.stage==0 and fresh.stalled_s==0 and fresh.reason=="new_attempt"


def test_empty_selection_and_new_goal_cannot_hide_an_active_stall():
    p = ProgressMonitor(_enabled())
    p.update(0,"g",2.)
    assert p.update(2,None,None).stalled_s==2.
    assert p.update(3,"other",1.).actions==("replan",)
    abandoned=p.update(13,None,None)
    assert abandoned.actions[-1]=="abandon" and p.failures["other"]==1


def test_real_progress_during_goal_change_and_explicit_task_boundary_reset():
    p = ProgressMonitor(_enabled())
    p.update(0,"first",2.)
    p.update(6,"first",2.)
    new_face=p.update(7,"second",1.,face_count=1)
    assert new_face.progressed and new_face.reason=="new_face"
    assert new_face.stage==0 and new_face.stalled_s==0
    p.update(10,"second",1.,face_count=1)
    p.reset(10.5)  # Explicit unrelated mission/task boundary.
    unrelated=p.update(11,"third",4.,face_count=1)
    assert not unrelated.progressed and unrelated.stalled_s==0 and unrelated.stage==0


def test_deferred_obstruction_can_trigger_backoff_after_threshold():
    p = ProgressMonitor(_enabled())
    p.update(0, "g", 1.0)
    assert "backoff" not in p.update(9, "g", 1.0).actions
    assert p.update(10, "g", 1.0, physically_blocked=True).actions == ("backoff",)
    assert not p.update(11, "g", 1.0, physically_blocked=True).actions


def test_normal_wait_suspends_stall_without_erasing_it():
    p = ProgressMonitor(_enabled())
    p.update(0, "g", 2.0)
    assert p.update(2, "g", 2.0).stalled_s == 2
    assert p.update(3, "g", 2.0, paused=True).stalled_s == 2
    assert p.update(20, "g", 2.0, paused=True).stalled_s == 2
    assert p.update(21, "g", 2.0).stalled_s == 2
    assert p.update(22, "g", 2.0).actions == ("replan",)


@pytest.mark.parametrize("progress", [dict(face_count=1), dict(shot_count=1), dict(arrived=True)])
def test_observed_task_progress_resets_stall(progress):
    p = ProgressMonitor(_enabled())
    p.update(0, "g", 2.0)
    p.update(8, "g", 2.0)
    s = p.update(9, "g", 2.0, **progress)
    assert s.progressed and s.stage == 0 and s.stalled_s == 0


def test_no_path_is_stalled_not_progress_and_backwards_clock_rejected():
    p = ProgressMonitor(_enabled())
    p.update(0, "g", None)
    assert p.update(6, "g", math.inf).actions == ("replan", "engage")
    assert not p.update(6.1, "g", 4.0).progressed
    assert p.update(6.2, "g", 3.84).progressed
    with pytest.raises(ValueError):
        p.update(1, "g", 3.0)


def test_fixed_slot_jitter_independent_of_call_order_and_different_between_slots():
    cfg = _enabled()
    waits = []
    for slot in ("B1", "B2", "B3", "R1", "R2", "R3"):
        p = ProgressMonitor(cfg, slot)
        first = p.jittered(1.0, "blocked")
        p.jittered(99, "unrelated")
        assert .8 <= first <= 1.2
        assert first == p.jittered(1.0, "blocked") == ProgressMonitor(cfg, slot).jittered(1.0, "blocked")
        waits.append(first)
    assert len(set(waits)) == 6


def test_direction_oscillation_across_stops_locks_for_commit_interval():
    p = ProgressMonitor(_enabled())
    for at, direction in ((0, 1), (1, 0), (2, -1), (3, 1)):
        assert p.note_decision(at, "AVOID", 1, direction) == 0
    assert p.note_decision(4, "AVOID", 1, -1) == 8
    assert p.note_decision(5, "AVOID", 1, 1) == 8


def test_decision_oscillation_locks_and_zero_does_not_count_direction():
    p = ProgressMonitor(_enabled())
    for at in range(4):
        assert p.note_decision(at, "AVOID" if at % 2 else "ENGAGE_OK", 0) == 0
    assert p.note_decision(4, "ENGAGE_OK", 0) == 8


def test_threat_rises_immediately_and_lowers_one_step_per_calm_window():
    cfg = _enabled()
    # Isolate the three-second level hysteresis here; the default combined
    # decision-change budget is covered by the dedicated churn test below.
    cfg.progress.max_decision_flips = 99
    c = PostureController(cfg)
    assert c.update(0, exposure=0).level == 0
    assert c.update(.01, exposure=3.1, hits=2).level == 3
    assert c.update(1, exposure=0, hits=0).level == 3
    assert c.update(3.99, exposure=0, hits=0).level == 3
    assert c.update(4, exposure=0, hits=0).level == 2
    assert c.update(6.99, exposure=0, hits=0).level == 2
    assert c.update(7, exposure=0, hits=0).level == 1
    assert c.update(10, exposure=0, hits=0).level == 0


def test_noncalm_tick_resets_level_down_timer_and_hit_floor_is_retained():
    c = PostureController(_enabled())
    c.update(0, exposure=3, hits=2)
    c.update(1, exposure=0, hits=2)
    c.update(3.9, exposure=3, hits=2)
    assert c.update(4, exposure=0, hits=2).level == 3
    assert c.update(7, exposure=0, hits=2).level == 2
    assert c.update(100, exposure=0, hits=2).level == 2


def test_level_down_up_churn_is_committed_but_new_threat_escalates_immediately():
    c = PostureController(_enabled())
    assert c.update(0, exposure=1).level == 1
    c.update(.1, exposure=0)
    assert c.update(3.1, exposure=0).level == 0
    assert c.update(3.2, exposure=1).level == 1
    c.update(3.3, exposure=0)
    held = c.update(6.3, exposure=0)
    assert held.level == 1 and held.lock_until == pytest.approx(10.3)
    raised = c.update(6.31, exposure=3, hits=2)
    assert raised.level == 3
    assert any("threat_rise" in reason for reason in raised.transitions)


def test_first_hit_changes_cost_only_and_l3_needs_second_hit():
    c = PostureController(_enabled())
    s = c.update(0, exposure=10, hits=1, visible_threat=True)
    assert s.level == 2 and s.posture == Posture.AVOID
    assert not s.allow_evade and not s.cap_exposed_dwell
    s = c.update(1, exposure=10, hits=2, visible_threat=True)
    assert s.level == 3 and s.allow_evade and s.cap_exposed_dwell


@pytest.mark.parametrize("reason", [dict(in_cm=True), dict(stall_stage=2), dict(invulnerable=True), dict(in_goal=True)])
def test_engage_reasons_cap_risk_disable_evade_and_apply_exposed_dwell(reason):
    c = PostureController(_enabled())
    s = c.update(0, exposure=3, hits=2, visible_threat=True, **reason)
    assert s.posture == Posture.ENGAGE_OK and s.risk_weight == .3
    assert not s.allow_evade and s.cap_exposed_dwell


def test_engage_commit_time_or_net_distance_and_minimum_dwell():
    c = PostureController(_enabled())
    c.update(0, in_cm=True, xy=(0, 0))
    assert c.update(1, xy=(.7, 0)).posture == Posture.ENGAGE_OK  # min dwell
    assert c.update(3, xy=(0, 0)).posture == Posture.ENGAGE_OK   # traveled out/back is not gain
    assert c.update(3.1, xy=(.61, 0)).posture == Posture.AVOID
    c = PostureController(_enabled())
    c.update(0, in_cm=True)
    assert c.update(3.99).posture == Posture.ENGAGE_OK
    assert c.update(4).posture == Posture.AVOID


def test_three_hits_immediately_override_commit_and_locks():
    c = PostureController(_enabled())
    c.update(0, in_cm=True, hits=2)
    s = c.update(.1, in_cm=True, hits=3, lock_until=100)
    assert s.posture == Posture.AVOID and s.reason == "hit_limit"
    assert c.update(101, in_cm=True, hits=3).posture == Posture.AVOID


@pytest.mark.parametrize("exception", [dict(invulnerable=True), dict(in_goal=True, goal_distance=1.0)])
def test_three_hit_exceptions_keep_engage(exception):
    c = PostureController(_enabled())
    c.update(0, in_cm=True, hits=2)
    assert c.update(.1, in_cm=True, hits=3, **exception).posture == Posture.ENGAGE_OK


def test_posture_prevents_fourth_ordinary_flip_within_ten_seconds():
    cfg = _enabled()
    cfg.posture.min_dwell_s = 0
    cfg.posture.commit_s = .5
    c = PostureController(cfg)
    c.update(0, in_cm=True)
    c.update(1)
    c.update(2, in_cm=True)
    s = c.update(3)
    assert s.posture == Posture.ENGAGE_OK and s.reason == "oscillation_commit"
    assert s.lock_until == 3.5


def test_fire_blocker_gate_denies_unknown_allies_reserve_and_lock():
    cfg = _enabled()
    kwargs = dict(now=100, posture=Posture.ENGAGE_OK, stall_stage=2, affiliation="enemy",
                  confidence=.8, distance_m=1., dome_visible=True, friendly_in_line=False,
                  weapon_locked=False, ammo=4, m2_complete=False)
    assert not blocker_fire_allowed(cfg, **kwargs).allowed
    cfg.engage.fire_on_blocker = True
    assert blocker_fire_allowed(cfg, **kwargs).allowed
    for change in (dict(affiliation="unknown"), dict(affiliation="ally"), dict(confidence=.79),
                   dict(ammo=3), dict(weapon_locked=True), dict(friendly_in_line=True),
                   dict(dome_visible=False), dict(last_target_shot=90.01), dict(actually_blocking=False),
                   dict(distance_m=math.nan), dict(confidence=math.nan), dict(stall_stage=1)):
        assert not blocker_fire_allowed(cfg, **dict(kwargs, **change)).allowed, change
    assert blocker_fire_allowed(cfg, **dict(kwargs, ammo=1, m2_complete=True, last_target_shot=90)).allowed
    assert cfg.mission.engage_ugv is False


def test_new_config_roundtrips_and_v2_switch_does_not_modify_baseline(tmp_path):
    cfg = _enabled()
    path = tmp_path / "config.json"
    path.write_text(json.dumps(cfg.to_dict()), encoding="utf8")
    loaded = Config.load(str(path))
    assert loaded.threat.lidar_crosstalk.enabled is False
    assert loaded.avoid.evade_min_hits == loaded.risk.dwell_cap_from_hits == 2
    loaded.progress.enabled = False
    p = ProgressMonitor(loaded)
    p.update(0, "g", 1.0)
    assert p.update(100, "g", 1.0).stage == 0
