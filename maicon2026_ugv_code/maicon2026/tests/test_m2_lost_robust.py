"""M2 조준 중 표적을 놓쳤을 때(mission.m2_lost_robust): 영구 포기 대신 그 면을 기억해 제외하고, 보이는 다른 면으로 갈아탄다.
배치·좌표를 가정하지 않는다 — 관측된 면 위치만으로 판단한다."""
import math
from types import SimpleNamespace

import pytest

from maicon26.config import Config
from maicon26.mission.manager import Mode
from maicon26.types import Pose2D
from maicon26.validate import sanitize_config
from test_m2_target_scope import agent_fixture, face_at


def robust_agent(**over):
    a = agent_fixture()
    a.cfg.mission.m2_lost_robust = True
    for k, v in over.items():
        setattr(a.cfg.mission, k, v)
    a._m2_bad = []
    return a


def test_defaults_are_off_and_real_config_turns_it_on():
    mc = Config().mission
    assert mc.m2_lost_robust is False and (mc.m2_lost_cooldown_s, mc.m2_lost_max, mc.m2_evidence_w) == (6.0, 3, 0.0)
    real = Config.load("config/waveshare_real.json")
    assert real.mission.m2_lost_robust is True and real.mission.m2_evidence_w > 0


@pytest.mark.parametrize("name,bad", [("m2_lost_cooldown_s", 0.0), ("m2_lost_max", 0), ("m2_lost_max", 2.5), ("m2_evidence_w", 1.0)])
def test_out_of_range_lost_keys_fall_back_to_the_default(name, bad):
    cfg = Config()
    default = getattr(cfg.mission, name)
    setattr(cfg.mission, name, bad)
    sanitize_config(cfg)
    assert getattr(cfg.mission, name) == default


def test_lost_face_is_set_aside_with_growing_cooldown_then_for_good():
    a = robust_agent(m2_lost_cooldown_s=5.0, m2_lost_max=3)
    f = face_at(4.5, 2.0)
    assert a._m2_register_lost(100.0, f) == pytest.approx(105.0)
    assert a._m2_register_lost(120.0, f) == pytest.approx(130.0)               # 두 번째는 두 배
    assert a._m2_register_lost(150.0, f) == math.inf                           # m2_lost_max 번째: 경기 끝까지
    assert a.stats["m2_lost_n"] == 3
    assert a._m2_face_banned(f, 1e9)


def test_banned_face_leaves_the_target_list_until_the_cooldown_ends_but_other_faces_stay():
    a = robust_agent()
    ghost, real = face_at(4.5, 2.0), face_at(4.9, 3.4)
    a.omap.faces = [ghost, real]
    assert a._target_faces() == [ghost, real]
    a._m2_register_lost(12.0, ghost)                                           # 제외 끝 18 s (now=12)
    assert a._target_faces() == [real]
    a.robot = SimpleNamespace(now=lambda: 19.0)
    assert a._target_faces() == [ghost, real]                                  # 쿨다운이 끝나면 다시 후보
    assert not a.target_failed


def test_ban_matches_by_observed_position_not_identity_so_a_remerged_ghost_stays_banned():
    a = robust_agent()
    a._m2_register_lost(12.0, face_at(4.5, 2.0))
    assert a._m2_face_banned(face_at(4.6, 2.1), 13.0)                          # 같은 면이 조금 옮겨 병합됨
    assert not a._m2_face_banned(face_at(4.5, 2.9), 13.0)                      # 다른 면


def test_all_target_faces_banned_does_not_declare_the_led_confirmed_done():
    a = robust_agent()
    f = face_at(4.5, 2.0)
    a.omap.faces = [f]
    a._m2_register_lost(12.0, f)
    assert a._target_faces() == [] and not a.target_done                       # '점등되어 live 가 없는 것'과 '제외되어 없는 것'을 구분


def test_evidence_weight_prefers_a_well_observed_face_over_a_slightly_closer_ghost():
    a = robust_agent(m2_evidence_w=0.08)
    pose = Pose2D(3.0, 2.0, 0.0)
    ghost, solid = face_at(4.0, 2.0), face_at(4.5, 2.0)
    ghost.n, solid.n = 2, 12
    assert a._m2_pick_face([ghost, solid], pose) is solid
    a.cfg.mission.m2_evidence_w = 0.0
    assert a._m2_pick_face([ghost, solid], pose) is ghost                      # 기본(0)은 예전처럼 가장 가까운 면


def test_visible_other_face_is_adopted_when_the_current_one_is_not_seen():
    a = robust_agent()
    cur, other = face_at(4.5, 2.0), face_at(4.9, 3.4)
    a.cam = SimpleNamespace(cx=320.0)
    a._dets = [SimpleNamespace(cls="COMM_FACILITY", conf=0.9, cx=320.0)]
    a._target_detection_matches = lambda d, g: g is other
    got = a._m2_adopt([cur, other], cur, "COMM_FACILITY")
    assert got is not None and got[0] is other
    a._target_detection_matches = lambda d, g: False
    assert a._m2_adopt([cur, other], cur, "COMM_FACILITY") is None


def strike_agent(robust):
    a = robust_agent()
    a.cfg.mission.m2_lost_robust = robust
    a.mode = Mode.STRIKE
    cur = face_at(4.5, 2.0)
    a.omap.faces = [cur]
    a.stack = SimpleNamespace(pose=Pose2D(3.6, 2.0, 0.0), clear_goal=lambda: None)
    a.laser = SimpleNamespace(shots=1, firing=lambda: False, track=lambda *x: None)
    a.robot = SimpleNamespace(now=lambda: 50.0, get_pan_tilt=lambda: (0.0, 0.0))
    a._dets = []
    a.cam = SimpleNamespace(cx=320.0)
    a.aim = SimpleNamespace(reset=lambda: None)
    a._apply = lambda *x, **k: None
    a._log = lambda *x, **k: None
    a._m2_defer_s = 0.0
    modes = []
    a._set_mode = lambda now, m, why: modes.append((m, why))
    a._strike.clear()
    a._strike.update(face=cur, phase="aim", fails=2, shots0=1, lost_since=40.0, t0=30.0, defer0=0.0, t_aim0=40.0)
    return a, cur, modes


def test_third_lost_aim_sets_the_face_aside_instead_of_failing_the_mission():
    a, cur, modes = strike_agent(True)
    a._mode_strike(50.0, 0.1, 50.0)
    assert not a.target_failed
    assert modes and modes[-1][0] == Mode.EXPLORE and "set aside" in modes[-1][1]
    assert a._m2_face_banned(cur, 51.0) and not a._m2_face_banned(cur, 50.0 + 6.0 + 0.1)


def test_legacy_third_lost_aim_still_fails_permanently_when_the_option_is_off():
    a, cur, modes = strike_agent(False)
    a._mode_strike(50.0, 0.1, 50.0)
    assert a.target_failed and modes[-1] == (Mode.EXPLORE, "M2 lost")


# ====================================================================== 자세 이력이 늦을 때의 외삽 (perception.pose_extrap_max_s)
def hist_agent(ex):
    from collections import deque
    a = agent_fixture()
    a.cfg.perception.pose_extrap_max_s = ex
    a.stats = {"pose_at_clamped": 0}
    a._log = lambda *x, **k: None
    a.stack = SimpleNamespace(pose=Pose2D(0.0, 0.0, 0.0), clear_goal=lambda: None)
    # 0.2 m/s 로 +x, 0.5 rad/s 로 회전하며 0.05 s 간격 이력
    a._hist = deque(((0.05 * i, 0.2 * 0.05 * i, 0.0, 0.5 * 0.05 * i) for i in range(10)), maxlen=120)
    return a


def test_pose_after_the_history_end_is_extrapolated_with_the_recent_velocity_when_enabled():
    a = hist_agent(0.7)
    p = a.pose_at(0.45 + 0.4)                                                   # 이력 끝(0.45 s)보다 0.4 s 늦은 촬영 시각
    assert p.x == pytest.approx(0.2 * 0.85, abs=1e-6) and p.th == pytest.approx(0.5 * 0.85, abs=1e-6)
    assert a.stats["pose_at_extrapolated"] == 1 and a.stats["pose_at_clamped"] == 0


def test_extrapolation_is_off_by_default_and_bounded_in_time():
    a = hist_agent(0.0)
    assert a.pose_at(0.85).x == pytest.approx(0.2 * 0.45, abs=1e-6)             # 예전: 가장자리 자세
    b = hist_agent(0.3)
    assert b.pose_at(0.45 + 0.6).x == pytest.approx(0.2 * 0.45, abs=1e-6)       # 외삽 한계(0.3 s)를 넘으면 가장자리 자세


def test_extrapolated_speed_is_capped_to_what_the_robot_can_do():
    a = hist_agent(0.7)
    a._hist.clear()
    a._hist.extend([(0.0, 0.0, 0.0, 0.0), (0.1, 1.0, 0.0, 0.0)])                # 10 m/s — 오도메트리 튐
    p = a.pose_at(0.1 + 0.5)
    assert p.x - 1.0 <= 1.5 * a.cfg.robot.v_max * 0.5 + 1e-9
