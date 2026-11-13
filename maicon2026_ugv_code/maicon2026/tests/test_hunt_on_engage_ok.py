"""자세가 ENGAGE_OK(교전 허용)이면 HUNT 도 고를 수 있게 하는 스위치(mission.hunt_on_engage_ok).

- 코드 기본은 꺼짐: 기존 동작(engage_ugv=False 면 HUNT 없음)이 그대로다.
- 실기 설정(config/waveshare_real.json)은 켠다. engage_ugv 는 여전히 False — 평소에는 사냥하지 않는다.
- 자세는 progress.enabled 일 때만 계산되므로, 꺼져 있으면 남아 있는 옛 자세로 켜지지 않는다.
시뮬 없이 _arbitrate 를 가짜 객체로 부른다(test_practice_mode_models.py 와 같은 방식).
"""
from pathlib import Path
from types import SimpleNamespace as NS

from maicon26.config import Config
from maicon26.launch_profile import load_run_config
from maicon26.mission.manager import Agent, Mode, hunt_allowed
from maicon26.mission.posture import Posture, PostureState
from maicon26.types import Pose2D, RobotStatus

ROOT = Path(__file__).resolve().parents[1]
NOW = 10.0


def _cfg(*, engage_ugv=False, hunt_on_engage_ok=True, progress=True):
    cfg = Config()
    cfg.mission.engage_ugv = engage_ugv
    cfg.mission.hunt_on_engage_ok = hunt_on_engage_ok
    cfg.progress.enabled = progress
    return cfg


def _me(cfg, posture=Posture.ENGAGE_OK, enemies=(), faces=(), reserve=0, locked=False, defer_hunt=False):
    return NS(cfg=cfg, posture_state=None if posture is None else PostureState(posture=posture, engage_reasons=("cm",)),
              locks=NS(weapon_locked=lambda st, now: locked), stack=NS(pose=Pose2D(0.0, 0.0, 0.0)),
              _hunt_candidates=lambda now: list(enemies), _target_faces=lambda: list(faces), _reserve=lambda: reserve,
              _coverage_defers_m2=lambda: False, _coverage_defers_m4=lambda: defer_hunt,
              _strike_block_until=0.0, _m2_defer_s=0.0)


NEAR = NS(x=0.8, y=0.0, last_cam_t=NOW - 0.2)        # 1.6 m 안, 카메라로 방금 봄
FAR_SEEN = NS(x=2.2, y=0.0, last_cam_t=NOW - 0.5)    # 1.6 m 밖, 2 s 안에 카메라로 봄
FULL = RobotStatus(ammo=8)


def _arb(me, st=FULL):
    return Agent._arbitrate(me, NOW, st)


def test_code_default_is_off_and_real_config_turns_it_on():
    assert Config().mission.hunt_on_engage_ok is False
    assert Config().mission.engage_ugv is False
    real, _applied = load_run_config(ROOT / "config" / "waveshare_real.json")
    assert real.mission.hunt_on_engage_ok is True
    assert real.mission.engage_ugv is False                     # 평소(AVOID)에는 여전히 사냥하지 않는다
    assert real.progress.enabled is True                        # 자세가 계산되는 설정이어야 스위치가 의미가 있다


def test_engage_ok_enables_hunt_for_a_near_seen_enemy():
    assert _arb(_me(_cfg(), enemies=(NEAR,))) == Mode.HUNT


def test_coverage_budget_prevents_both_near_and_far_reentry_without_disabling_m2():
    for enemy in (NEAR, FAR_SEEN):
        assert _arb(_me(_cfg(), enemies=(enemy,), defer_hunt=True)) == Mode.EXPLORE
        assert _arb(_me(_cfg(), enemies=(enemy,), faces=(object(),), defer_hunt=True)) == Mode.STRIKE


def test_engage_ok_near_enemy_still_preempts_strike_like_engage_ugv_does():
    assert _arb(_me(_cfg(), enemies=(NEAR,), faces=(object(),))) == Mode.HUNT


def test_engage_ok_far_enemy_does_not_preempt_m2_strike():
    assert _arb(_me(_cfg(), enemies=(FAR_SEEN,), faces=(object(),))) == Mode.STRIKE
    assert _arb(_me(_cfg(), enemies=(FAR_SEEN,))) == Mode.HUNT  # 지정표적 면이 없으면 교전


def test_other_postures_do_not_enable_hunt():
    for posture in (Posture.AVOID, Posture.COVER, None):
        assert _arb(_me(_cfg(), posture=posture, enemies=(NEAR,))) == Mode.EXPLORE, posture


def test_switch_off_keeps_existing_behaviour_even_in_engage_ok():
    assert _arb(_me(_cfg(hunt_on_engage_ok=False), enemies=(NEAR,))) == Mode.EXPLORE


def test_stale_posture_is_ignored_when_progress_is_disabled():
    assert _arb(_me(_cfg(progress=False), enemies=(NEAR,))) == Mode.EXPLORE


def test_engage_ugv_still_enables_hunt_regardless_of_posture():
    for posture in (Posture.AVOID, Posture.COVER, None):
        assert _arb(_me(_cfg(engage_ugv=True, hunt_on_engage_ok=False, progress=False), posture=posture,
                        enemies=(NEAR,))) == Mode.HUNT


def test_existing_hunt_gates_still_apply_in_engage_ok():
    assert _arb(_me(_cfg(), enemies=(NEAR,), locked=True)) == Mode.EXPLORE          # 피격 직후 무기 잠금
    assert _arb(_me(_cfg(), enemies=(NEAR,), reserve=3), RobotStatus(ammo=3)) == Mode.EXPLORE   # M2 예비 탄 3발
    assert _arb(_me(_cfg(), enemies=())) == Mode.EXPLORE                            # 후보 없음
    stale_far = NS(x=3.0, y=0.0, last_cam_t=NOW - 5.0)                              # 멀고 카메라로 오래 못 봄
    assert _arb(_me(_cfg(), enemies=(stale_far,))) == Mode.EXPLORE


def test_hunt_allowed_accepts_enum_or_plain_string_posture():
    cfg = _cfg()
    assert hunt_allowed(NS(cfg=cfg, posture_state=NS(posture="ENGAGE_OK")), cfg.mission)
    assert not hunt_allowed(NS(cfg=cfg, posture_state=NS(posture="AVOID")), cfg.mission)
    assert not hunt_allowed(NS(cfg=cfg), cfg.mission)                               # posture_state 가 아직 없음
