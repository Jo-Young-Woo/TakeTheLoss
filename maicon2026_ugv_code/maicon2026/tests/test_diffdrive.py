"""차동(스키드 스티어) 구동 변환 계층 (hal/diffdrive.py) 검증."""

import pytest

from maicon26.calib.generate import gen_config_overlay
from maicon26.calib.profile import Profile
from maicon26.config import Config, RobotCfg
from maicon26.hal.diffdrive import DiffDrive
from maicon26.sim.drive import SimDrive
from maicon26.sim.run import build_match


def _dd(**kw):
    return DiffDrive(RobotCfg(**kw))


# ====================================================================== 순수 운동학
def test_twist_wheels_roundtrip_and_signs():
    d = _dd(track_width=0.20)
    vl, vr = d.wheels_from_twist(0.3, 1.0)
    assert vl == pytest.approx(0.2) and vr == pytest.approx(0.4)      # 반시계(w>0) → 오른쪽 바퀴가 더 빠르다
    assert d.twist_from_wheels(vl, vr) == pytest.approx((0.3, 1.0))
    assert d.wheels_from_twist(0.0, 2.0) == pytest.approx((-0.2, 0.2))  # 제자리 회전: 반대 방향
    wl, wr = _dd(track_width=0.20, wheel_radius=0.05).wheel_rates(0.3, 0.0)
    assert (wl, wr) == pytest.approx((6.0, 6.0))                       # v / r


def test_defaults_are_neutral():
    d = _dd()
    assert d.compensate(0.3, 1.2) == (0.3, 1.2)
    v, w, cut = d.limit(0.1, 0.5)
    assert (v, w, cut) == (0.1, 0.5, False)


def test_limit_scales_both_to_keep_curvature():
    d = _dd(track_width=0.20, v_max=0.45)
    v, w, cut = d.limit(0.45, 1.0)                                      # 바깥 바퀴 0.55 > 0.45
    assert cut and max(abs(x) for x in d.wheels_from_twist(v, w)) == pytest.approx(0.45)
    assert w / v == pytest.approx(1.0 / 0.45)                           # 곡률(w/v) 유지
    v, w, cut = d.limit(0.0, 6.0)                                       # 제자리 회전도 바퀴 한계 안으로
    assert cut and w == pytest.approx(4.5)
    assert _dd(track_width=0.20, v_max=0.45, wheel_limit=False).limit(0.45, 1.0) == (0.45, 1.0, False)


def test_wheel_v_max_overrides_v_max():
    d = _dd(track_width=0.20, v_max=0.45, wheel_v_max=0.30)
    assert d.wheel_max == 0.30
    v, w, cut = d.limit(0.3, 0.0)
    assert not cut
    v, w, cut = d.limit(0.4, 0.0)
    assert cut and v == pytest.approx(0.30)


def test_compensation_inverts_the_sim_model():
    # 시뮬(sim/world.py): w_실제 = w_명령·(1 − pivot_loss) + lr_bias·v   →  보상한 명령은 원하는 w 를 만든다
    d = _dd(yaw_eff=0.9, lr_bias=0.05)
    v, wc = d.compensate(0.3, 0.5)
    assert 0.9 * wc + 0.05 * v == pytest.approx(0.5)
    assert d.compensate(0.0, 1.0)[1] == pytest.approx(1.0 / 0.9)


# ====================================================================== 시뮬 로봇에서: 보상이 실제로 슬립·편차를 지운다
def _drive(compensate, v, w, seconds, yaw_eff=0.90, lr_bias=0.05):
    m = build_match(seed=1, cfg=Config(), teams=["B2"])
    u = m.robots[0].u
    u.drive = SimDrive(lr_bias=lr_bias, pivot_loss=1.0 - yaw_eff)       # 로봇의 '숨은' 특성
    dd = DiffDrive(RobotCfg(yaw_eff=yaw_eff if compensate else 1.0, lr_bias=lr_bias if compensate else 0.0))
    dth, steps = 0.0, int(seconds / 0.05)
    for _ in range(steps):
        u.cmd_v, u.cmd_w = dd.command(v, w)
        m.world.step(0.05)
        dth += u.w_true * 0.05
    return dth


def test_straight_line_heading_drift_is_removed():
    raw = _drive(False, 0.30, 0.0, 2.5)
    fixed = _drive(True, 0.30, 0.0, 2.5)
    assert abs(raw) > 0.02                                               # 보상 없이는 휜다(0.05 rad/m × ~0.7 m)
    assert abs(fixed) < 0.2 * abs(raw) and abs(fixed) < 0.006


def test_in_place_rotation_reaches_the_commanded_rate():
    ideal = 1.0 * 3.0 - 0.5 * 1.0 ** 2 / 8.0                            # 각가속 제한(8 rad/s²) 램프 손실 반영
    raw = _drive(False, 0.0, 1.0, 3.0)
    fixed = _drive(True, 0.0, 1.0, 3.0)
    assert abs(raw - ideal) > 0.2                                        # 보상 없이는 10 % 덜 돈다
    assert abs(fixed - ideal) < 0.03


# ====================================================================== Agent 연결
def test_agent_respects_wheel_limit_and_reports_it():
    cfg = Config()
    cfg.mission.spin_w, cfg.mission.tour_nx, cfg.mission.tour_ny = 1.8, 3, 3
    cfg.robot.wheel_v_max = 0.20                                        # 바퀴가 느린 로봇
    cfg.robot.track_width = 0.20
    m = build_match(seed=1, cfg=cfg, teams=["B2"])
    a, u = m.agents[0], m.robots[0].u
    peak = 0.0
    for _ in range(int(25 / 0.05)):
        a.tick()
        vl, vr = a.diff.wheels_from_twist(u.cmd_v, u.cmd_w)
        peak = max(peak, abs(vl), abs(vr))
        m.world.step(0.05)
    assert peak <= 0.20 + 1e-6
    assert a.stats["wheel_limited"] > 0 and a.stats["multi_output"] == 0


def test_calibrated_overlay_carries_drive_parameters():
    ov = gen_config_overlay(Profile())["robot"]
    assert {"track_width", "wheel_radius", "yaw_eff", "lr_bias"} <= set(ov)
    cfg = Config()
    from maicon26.config import _merge
    _merge(cfg, {"robot": ov})                                           # 생성한 값이 Config 에 그대로 들어간다
    assert cfg.robot.track_width == ov["track_width"] and cfg.robot.yaw_eff == ov["yaw_eff"]
