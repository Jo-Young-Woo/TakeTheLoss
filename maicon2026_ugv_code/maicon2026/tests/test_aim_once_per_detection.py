"""조준 루프: 같은 검출의 영상 오차를 틱마다 다시 더하지 않는다(aim.once_per_detection).
이전 문제: manager._mode_strike 가 20 Hz 로 self._dets(최대 0.8 s 유지)를 AimController.update 에 넣어,
`pan_cmd = 현재 팬 + 0.9 × 오차` 를 새 검출 하나에 평균 7번가량 더했다 → 넘치고 되돌아오는 진동.
시뮬을 쓰지 않는다(가짜 로봇·가짜 검출기·가짜 항법 스택)."""
import inspect
import math
from unittest.mock import MagicMock

import pytest

from hw_fakes import make_robot
from maicon26.config import Config
from maicon26.link.base import ServerLink
from maicon26.mission import manager as manager_mod
from maicon26.mission.aim import AimController
from maicon26.mission.manager import Agent
from maicon26.perception.detector import Detector
from maicon26.types import Detection, Pose2D


class NullLink(ServerLink):
    def poll(self):
        return []

    def submit(self, kind, payload):
        return True


class NoDet(Detector):
    def detect(self, frame):
        return []


def target(du=60.0, dv=0.0):
    """화면 중심(320, 240)에서 오른쪽으로 du px 떨어진 표적 — 오른쪽이면 오차가 음(팬을 오른쪽으로 돌려야 한다)."""
    cx, cy = 320.0 + du, 240.0 + dv
    return Detection("TANK", 0.9, (cx - 20, cy - 20, cx + 20, cy + 20), 0.0, None, {})


def make_agent(**aim_over):
    cfg = Config()
    cfg.nav.backend = "builtin"
    cfg.aim.require_sensor = False
    for k, v in aim_over.items():
        setattr(cfg.aim, k, v)
    robot, tr, clock = make_robot("practice", "idle", cfg=cfg)
    agent = Agent(robot, NullLink(), cfg=cfg, detector=NoDet())
    agent.team, agent.slot = "BLUE", 2
    agent.stack = MagicMock()
    agent.stack.pose = Pose2D(2.0, 1.0, 0.0)
    return agent


def set_ctx(agent, stamp, pan_cap=0.0, tilt_cap=0.0, th=0.0, t_cap=1.0):
    agent._det_consumed = stamp
    agent._det_ctx = (t_cap, Pose2D(2.0, 1.0, th), pan_cap, tilt_cap)


# ---------------------------------------------------------------- 핵심: 같은 검출은 한 번만 적용
def test_same_detection_gives_the_same_command_every_tick():
    agent = make_agent()
    set_ctx(agent, stamp=5.0)
    det = target()
    outs = [agent._aim_step(det, 0.0, 0.0, 5.0 + 0.05 * i, 1.0) for i in range(20)]
    pans = {round(o[0], 9) for o in outs}
    assert len(pans) == 1, pans
    assert agent.stats["aim_updates"] == 1 and agent.stats["aim_held_ticks"] == 19


def test_old_behaviour_overshoots_and_new_behaviour_converges_when_the_gimbal_follows_commands():
    """짐벌이 명령을 그대로 따라간다고 하고 같은 검출을 7틱 동안 쓴다. 예전: 오차를 7번 더해 목표를 크게 넘는다."""
    cam = AimController(Config()).cam
    err_deg = math.degrees(math.atan2(60.0, cam.f))                  # 표적이 오른쪽 → 팬은 −err 만큼
    one_step = 0.9 * err_deg

    old = make_agent(once_per_detection=False)
    set_ctx(old, stamp=5.0)
    pan = 0.0
    for i in range(7):
        pan, _, _ = old._aim_step(target(), pan, 0.0, 5.0 + 0.05 * i, 1.0)
    assert abs(pan) > 3.0 * one_step                                  # 오차를 반복해서 더했다(진동의 원인)

    new = make_agent()
    set_ctx(new, stamp=5.0)
    pan = 0.0
    for i in range(7):
        pan, _, _ = new._aim_step(target(), pan, 0.0, 5.0 + 0.05 * i, 1.0)
    assert abs(pan) == pytest.approx(one_step, rel=0.02)               # 한 번만 적용 — 목표 방향 한 걸음
    assert pan < 0


def test_uses_pan_and_tilt_at_capture_not_the_current_ones():
    agent = make_agent()
    set_ctx(agent, stamp=5.0, pan_cap=10.0, tilt_cap=-2.0)
    base = make_agent()
    set_ctx(base, stamp=5.0, pan_cap=0.0, tilt_cap=0.0)
    p_cur, t_cur, _ = agent._aim_step(target(), 40.0, 9.0, 5.0, 1.0)     # 지금 팬·틸트는 전혀 다르다
    p0, t0, _ = base._aim_step(target(), 0.0, 0.0, 5.0, 1.0)
    assert p_cur - p0 == pytest.approx(10.0, abs=1e-6)
    assert t_cur - t0 == pytest.approx(-2.0, abs=1e-6)


def test_new_detection_recomputes():
    agent = make_agent()
    set_ctx(agent, stamp=5.0)
    p1, _, _ = agent._aim_step(target(60.0), 0.0, 0.0, 5.0, 1.0)
    set_ctx(agent, stamp=5.4)
    p2, _, _ = agent._aim_step(target(10.0), 0.0, 0.0, 5.4, 1.0)
    assert agent.stats["aim_updates"] == 2
    assert abs(p2) < abs(p1)                                          # 오차가 줄어든 새 검출을 반영


def test_aim_reset_forces_a_fresh_computation_even_for_the_same_frame():
    agent = make_agent()
    set_ctx(agent, stamp=5.0)
    agent._aim_step(target(), 0.0, 0.0, 5.0, 1.0)
    agent.aim.reset()                                                  # 새 표적 면 등
    agent._aim_step(target(), 0.0, 0.0, 5.05, 1.0)
    assert agent.stats["aim_updates"] == 2


def test_body_rotation_after_capture_is_removed_from_the_pan_command():
    agent = make_agent()
    set_ctx(agent, stamp=5.0, th=0.0)
    p0, _, _ = agent._aim_step(target(), 0.0, 0.0, 5.0, 1.0)
    agent.stack.pose = Pose2D(2.0, 1.0, math.radians(8.0))             # 찍은 뒤 차체가 왼쪽으로 8° 돌았다
    p1, _, _ = agent._aim_step(target(), 0.0, 0.0, 5.1, 1.0)
    assert p1 == pytest.approx(p0 - 8.0, abs=1e-6)
    assert agent.stats["aim_updates"] == 1                             # 재계산이 아니라 보정


def test_pan_command_stays_inside_the_pan_limit():
    agent = make_agent()
    set_ctx(agent, stamp=5.0, th=0.0)
    agent.stack.pose = Pose2D(2.0, 1.0, math.radians(120.0))
    p, _, _ = agent._aim_step(target(), 0.0, 0.0, 5.0, 1.0)
    assert abs(p) <= agent.cfg.robot.pan_limit + 1e-9


def test_body_rate_follows_the_current_pan():
    agent = make_agent()
    set_ctx(agent, stamp=5.0, pan_cap=0.0)
    _, _, res = agent._aim_step(target(), 12.0, 0.0, 5.0, 1.0)
    assert res.body_w == pytest.approx(agent.aim.body_rate(12.0))
    assert res.body_w != agent.aim.body_rate(0.0)


def test_without_capture_context_it_falls_back_to_the_per_tick_update():
    agent = make_agent()
    agent._det_ctx = None
    p1, _, _ = agent._aim_step(target(), 0.0, 0.0, 5.0, 1.0)
    p2, _, _ = agent._aim_step(target(), p1, 0.0, 5.05, 1.0)
    assert abs(p2) > abs(p1) * 1.5                                      # 예전 동작(누적)
    assert "aim_updates" not in agent.stats


def test_strike_and_hunt_both_use_the_shared_step():
    for fn in (Agent._mode_strike, Agent._mode_hunt):
        src = inspect.getsource(fn)
        assert "self.aim.update(" not in src and "self._aim_step(" in src


# ---------------------------------------------------------------- AimController 쪽
def test_default_update_is_unchanged_when_t_obs_is_omitted():
    a, b = AimController(Config()), AimController(Config())
    cfg_det = target(30.0)
    assert a.update(cfg_det, 3.0, 1.0, 2.0, 1.2) == b.update(cfg_det, 3.0, 1.0, 2.0, 1.2, t_obs=None)


def test_velocity_lead_uses_capture_times_not_compute_times():
    """두 검출이 0.4 s 간격으로 찍혔는데 같은 틱에 처리돼도(now 간격 0.01 s) 속도는 0.4 s 기준이다."""
    cfg = Config()
    fast, slow = AimController(cfg), AimController(cfg)
    d1 = Detection("UGV", 0.9, (300, 220, 340, 260), 0.0, "RED", {})
    d2 = Detection("UGV", 0.9, (340, 220, 380, 260), 0.0, "RED", {})
    fast.update(d1, 0.0, 0.0, 5.00, 1.5, t_obs=1.0)
    fast.update(d2, 0.0, 0.0, 5.01, 1.5, t_obs=1.4)
    slow.update(d1, 0.0, 0.0, 5.00, 1.5)
    slow.update(d2, 0.0, 0.0, 5.01, 1.5)
    assert abs(fast._vel_u.y) < abs(slow._vel_u.y) / 10.0                # 0.4 s 동안 40 px = 100 px/s vs 0.01 s 동안 40 px = 4000 px/s


def test_generation_counts_resets():
    a = AimController(Config())
    g = a.generation
    a.reset()
    assert a.generation == g + 1


def test_default_config_enables_the_fix_and_it_can_be_turned_off():
    assert Config().aim.once_per_detection is True
    assert manager_mod.Agent._aim_step is not None
