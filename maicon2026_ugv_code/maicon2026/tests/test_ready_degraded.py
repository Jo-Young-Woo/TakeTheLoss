"""1장 — /ready 막힘 제거와 저하 모드. rclpy·시뮬 없이 가짜 robot·stack·시계·transport 로 본다.

규칙: 필수(state·match·match_mode)가 없으면 지금처럼 TimeoutError, 주행(scan·odom·imu·Nav2·TF)은 no_drive,
영상은 no_vision, 짐벌·발사 서비스는 no_fire, 깊이·OAK·OLED 는 경고만. 모두 갖춰지면 기존과 같다.
"""
from types import SimpleNamespace as NS

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.degrade import DegradeController
from maicon26.hal.wingbot import Transport, WingbotRobot
from maicon26.link.base import ServerLink
from maicon26.nav.degraded import NoDriveStack
from maicon26.nav.stack import NavOut
from maicon26.perception.detector import Detector
from maicon26.preflight import (ReadyState, classify_readiness, classify_reason, degraded_oled_text, wait_ready)
from hw_fakes import Clock, FakeTransport, make_robot as _make_robot
from test_real_runtime import runtime  # noqa: F401  (run_real 수명주기 가짜 환경)


# ====================================================================== 분류
@pytest.mark.parametrize("name,kind", [
    ("state", "required"), ("match", "required"), ("match_mode", "required"),
    ("scan", "drive"), ("odom", "drive"), ("imu", "drive"),
    ("Nav2 액션 서버 미준비", "drive"), ("map→base TF 미수신", "drive"),
    ("multiple_publishers:/cmd_vel", "drive"), ("transport", "drive"),
    ("처음_보는_이름", "drive"),                                  # 모르는 이름은 보수적으로 주행 갈래
    ("image", "vision"), ("camera_info", "vision"), ("camera_calibration", "vision"),
    ("service:gimbal", "fire"), ("service:fire", "fire"), ("gimbal_feedback", "fire"),
    ("depth", "optional"), ("depth: CameraInfo frame mismatch", "optional"), ("topic:oled", "optional"), ("oak_rgb", "optional"),
])
def test_every_missing_name_has_one_lane(name, kind):
    assert classify_reason(name) == kind


def test_classify_readiness_maps_lanes_to_flags():
    s = classify_readiness(["scan", "image", "service:fire", "depth"])
    assert s.degraded and s.flags == {"no_drive", "no_vision", "no_fire"}
    assert s.optional_missing == ("depth",) and not s.required_missing
    assert classify_readiness([]) == ReadyState()
    only_depth = classify_readiness(["depth"])
    assert only_depth.degraded and only_depth.flags == frozenset()


def test_oled_text_fits_21_chars():
    assert degraded_oled_text({"no_drive"}) == "DEGRADED NO_DRIVE"
    assert degraded_oled_text(set()) == ""
    multi = degraded_oled_text({"no_drive", "no_vision", "no_fire"})
    assert multi.startswith("DEG ") and len(multi) <= 21


# ====================================================================== wait_ready
def test_scan_never_arrives_gives_no_drive_after_the_timeout():
    c = Clock()
    state = wait_ready(NS(readiness=lambda: (False, ["scan"])), NS(), 1.0, clock=c, sleep=c.sleep)
    assert state.degraded and state.flags == {"no_drive"} and state.reasons == ("scan",)
    assert c.t >= 1.0                                              # 제한시간을 다 기다린 뒤에 돌아온다


@pytest.mark.parametrize("required", ["state", "match", "match_mode"])
def test_missing_required_item_still_raises(required):
    c = Clock()
    with pytest.raises(TimeoutError, match=required):
        wait_ready(NS(readiness=lambda: (False, ["scan", required])), NS(), .5, clock=c, sleep=c.sleep)


def test_everything_ready_returns_immediately_exactly_as_before():
    c, polled = Clock(), []

    def readiness():
        polled.append(1)
        return True, []
    state = wait_ready(NS(readiness=readiness), NS(), 60.0, clock=c, sleep=c.sleep)
    assert state == ReadyState() and not state.degraded
    assert len(polled) == 1 and c.t == 0.0 and c.calls == 1        # 확인 1번, 시계 1번(deadline 계산) — 기존과 같다


def test_ready_before_the_deadline_is_not_degraded():
    c, n = Clock(), [0]

    def readiness():
        n[0] += 1
        return (n[0] >= 4), ([] if n[0] >= 4 else ["scan"])
    state = wait_ready(NS(readiness=readiness), NS(), 60.0, clock=c, sleep=c.sleep)
    assert not state.degraded and c.t < 1.0


def test_only_depth_missing_does_not_block():
    c = Clock()
    state = wait_ready(NS(readiness=lambda: (False, ["depth"]), stats={}), NS(), .3, clock=c, sleep=c.sleep)
    assert state.degraded and state.flags == frozenset() and state.optional_missing == ("depth",)


def test_image_and_services_map_to_their_own_flags():
    c = Clock()
    s = wait_ready(NS(readiness=lambda: (False, ["image", "service:gimbal"])), NS(), .2, clock=c, sleep=c.sleep)
    assert s.flags == {"no_vision", "no_fire"}


def test_required_missing_is_reported_every_five_seconds_not_more_often():
    c, lines = Clock(), []
    with pytest.raises(TimeoutError):
        wait_ready(NS(readiness=lambda: (False, ["state"])), NS(), 12.0, clock=c, sleep=c.sleep, report=lines.append)
    assert 2 <= len(lines) <= 3 and all("state" in line for line in lines)      # t=0, 5, 10


# ====================================================================== HAL 가드 (플래그마다 한 곳)
def make_robot(flags=()):
    return _make_robot("practice", "idle", flags)                   # 연습 모드: 제어 허용


def test_no_drive_never_publishes_a_nonzero_command():
    robot, tr, clock = make_robot({"no_drive"})
    for k in range(50):
        clock.t += 0.1
        robot.set_velocity(0.3, 1.0)
        robot.pump()
    assert tr.cmds and all(c == (0.0, 0.0) for c in tr.cmds)
    assert robot.stats["cmd_zero_no_drive"] > 0


def test_without_the_flag_the_command_goes_out_as_before():
    robot, tr, clock = make_robot(())
    clock.t += 0.1
    robot.set_velocity(0.3, 0.0)
    robot.pump()
    assert (0.3, 0.0) in tr.cmds


def test_flag_set_after_motion_cuts_the_running_command_at_once():
    robot, tr, clock = make_robot(())
    robot.set_velocity(0.3, 0.0)
    robot.pump()
    assert tr.cmds[-1] == (0.3, 0.0)
    robot.set_degraded({"no_drive"})
    assert tr.cmds[-1] == (0.0, 0.0)


def test_no_fire_never_calls_gimbal_or_fire_services():
    robot, tr, clock = make_robot({"no_fire"})
    clock.t += 1.0
    robot.set_pan_tilt(10.0, 5.0)
    robot.pump()
    assert robot.laser_fire() is False
    assert tr.gimbal == [] and tr.fire == []
    robot.set_degraded(())                                          # 저하가 풀리면 서비스를 다시 부른다
    clock.t += 1.0
    robot.set_pan_tilt(10.0, 5.0)
    robot.pump()
    assert tr.gimbal


def test_set_degraded_exposes_flags():
    robot, _, _ = make_robot({"no_vision"})
    assert robot.is_degraded("no_vision") and not robot.is_degraded("no_drive")
    assert robot.degraded_flags() == {"no_vision"}


# ====================================================================== 항법 대체물
class InnerStack:
    def __init__(self):
        self.goals, self.cleared, self.steps, self.closed = [], 0, 0, False
        self.initialized = True
        self.grid = object()
        self.pose = NS(x=1, y=2, th=0)
        self.client = NS(available=lambda: True, lookup_map_to_base=lambda: (0, 0, 0))
        self.replans = self.recoveries = 0

    def set_goal(self, goal, tol=None, speed_scale=1.0):
        self.goals.append(goal)

    def clear_goal(self):
        self.cleared += 1

    def step(self, scan, now, dt):
        self.steps += 1
        return NavOut(0.2, 0.0, "moving", 1.0)

    def limit(self, scan, v, w, now):
        return v, w

    def update(self, odom, scan, now):
        pass

    def start(self, scan, odom, guess):
        return odom

    def set_hint(self, *a, **k):
        pass

    def set_obstacle_layer(self, mask):
        pass

    def set_keepout_layer(self, mask):
        return False

    def spread(self):
        return 0.0

    def diagnostics(self):
        return {}

    def close(self):
        self.closed = True


def test_no_drive_stack_refuses_goals_and_reports_unreachable_at_once():
    inner = InnerStack()
    st = NoDriveStack(inner)
    st.set_goal((1.0, 2.0))
    out = st.step(None, 0.0, 0.1)
    assert inner.goals == [] and inner.steps == 0                   # Nav2 목표를 보내지 않는다
    assert (out.status, out.v, out.w) == ("blocked", 0.0, 0.0)
    assert st.limit(None, 0.5, 1.0, 0.0) == (0.0, 0.0)
    assert st.client is inner.client and st.pose is inner.pose and st.initialized
    st.close()
    assert inner.closed


def test_no_drive_stack_passes_through_after_recovery():
    inner = InnerStack()
    st = NoDriveStack(inner)
    st.set_no_drive(False)
    st.set_goal((1.0, 2.0))
    assert inner.goals == [(1.0, 2.0)] and st.step(None, 0.0, 0.1).status == "moving"


# ====================================================================== 시작 전 재확인
def test_lidar_coming_back_before_start_restores_normal_mode():
    c = Clock()
    present = {"scan": False}
    robot = NS(readiness=lambda: ((present["scan"]), [] if present["scan"] else ["scan"]), stats={},
               set_degraded=lambda f: setattr(robot, "flags", frozenset(f)), flags=frozenset(), show=lambda t: None)
    inner = InnerStack()
    stack = NoDriveStack(inner)
    lines = []
    ctl = DegradeController(robot, stack, classify_readiness(["scan"]), clock=c, report=lines.append)
    assert robot.flags == {"no_drive"} and stack.enabled
    c.t = 0.5
    ctl.tick(started=False)                                         # 1 s 간격 전에는 다시 보지 않는다
    present["scan"] = True
    ctl.tick(started=False)
    assert ctl.rechecks <= 1
    c.t = 1.6
    ctl.tick(started=False)
    assert robot.flags == frozenset() and not stack.enabled         # 정상 모드로 출발
    assert any("플래그 변경" in m for m in lines)


def test_flags_are_frozen_at_match_start():
    c = Clock()
    present = {"scan": False}
    robot = NS(readiness=lambda: ((present["scan"]), [] if present["scan"] else ["scan"]), stats={},
               set_degraded=lambda f: setattr(robot, "flags", frozenset(f)), flags=frozenset())
    stack = NoDriveStack(InnerStack())
    ctl = DegradeController(robot, stack, classify_readiness(["scan"]), clock=c, report=lambda m: None)
    ctl.tick(started=True)                                          # 경기 시작 → 고정
    present["scan"] = True
    for k in range(10):
        c.t += 2.0
        ctl.tick(started=True)
    assert ctl.frozen and robot.flags == {"no_drive"} and stack.enabled      # 경기 중 복귀는 범위 밖


def test_degraded_state_is_printed_every_five_seconds():
    c, lines = Clock(), []
    robot = NS(readiness=lambda: (False, ["scan"]), stats={}, set_degraded=lambda f: None)
    ctl = DegradeController(robot, NoDriveStack(InnerStack()), classify_readiness(["scan"]), clock=c, report=lines.append)
    for k in range(120):                                            # 12 s, 0.1 s 틱
        c.t += 0.1
        ctl.tick(started=False)
    reports = [m for m in lines if "누락 항목" in m]
    assert 2 <= len(reports) <= 3


def test_controller_never_raises():
    c = Clock()
    robot = NS(readiness=lambda: (_ for _ in ()).throw(RuntimeError("boom")), stats={}, set_degraded=lambda f: None)
    ctl = DegradeController(robot, NoDriveStack(InnerStack()), classify_readiness(["scan"]), clock=c, report=lambda m: None)
    c.t = 5.0
    ctl.tick(started=False)


# ====================================================================== Agent 의 no_vision / OLED
class CountingDetector(Detector):
    def __init__(self):
        self.calls = 0

    def detect(self, frame):
        self.calls += 1
        return []


class NullLink(ServerLink):
    def poll(self):
        return []

    def submit(self, kind, payload):
        return True


def make_agent(flags):
    from maicon26.mission.manager import Agent
    cfg = Config()
    cfg.nav.backend = "builtin"
    tr = FakeTransport()
    robot = WingbotRobot(cfg, transport=tr)
    robot.on_match("practice", "idle")
    robot.set_degraded(flags)
    robot.on_image(np.zeros((480, 640, 3), np.uint8), 0.0)
    det = CountingDetector()
    agent = Agent(robot, NullLink(), cfg=cfg, detector=det)
    from maicon26.perception.tracks import UgvTracker
    agent.tracker = UgvTracker(cfg, "BLUE")                         # _sense 가 정체성을 정할 때 만드는 것
    return agent, robot, det


def test_no_vision_never_calls_the_detector():
    agent, robot, det = make_agent({"no_vision"})
    for _ in range(5):
        agent._ingest_vision(1.0)
    assert det.calls == 0


def test_with_vision_the_detector_is_called_as_before():
    agent, robot, det = make_agent(())
    agent._ingest_vision(1.0)
    assert det.calls == 1


def test_degraded_m3_exception_skips_only_that_submission():
    agent, robot, det = make_agent({"no_drive"})
    agent.t_start = 0.0

    def boom(now, t):
        raise RuntimeError("no evidence yet")
    agent._submit_m3_if_due = boom
    agent._submit_m3_checked(1.0, 1.0)                              # 예외가 밖으로 나오지 않는다
    assert agent.stats["m3_degraded_errors"] == 1
    normal, _, _ = make_agent(())
    normal._submit_m3_if_due = boom
    with pytest.raises(RuntimeError):                               # 정상 모드는 기존과 같다(감추지 않는다)
        normal._submit_m3_checked(1.0, 1.0)


def test_oled_status_line_shows_the_degraded_flag():
    agent, robot, det = make_agent({"no_drive"})
    agent._show_status(1.0, robot.status())
    assert "DEGRADED NO_DRIVE" in robot.screen.want


# ====================================================================== run_real
def ready_state(flags, reasons):
    return ReadyState(degraded=True, flags=frozenset(flags), reasons=tuple(reasons))


def test_degraded_state_still_sends_ready_and_wraps_the_stack(runtime, monkeypatch):
    seen = {"timeouts": [], "agent": {}}
    state = ready_state({"no_drive"}, ["scan"])

    def fake_wait(robot, stack, timeout, **kw):
        seen["timeouts"].append(timeout)
        return state

    class DegRobot(runtime.Robot):
        def set_degraded(self, flags):
            runtime.events.append(("degraded", tuple(sorted(flags))))

    def agent(*a, **kw):
        seen["agent"] = kw
        runtime.events.append("agent")
    monkeypatch.setattr(runtime.mod, "wait_ready", fake_wait)
    monkeypatch.setattr(runtime.mod, "RosWingbot", DegRobot)
    monkeypatch.setattr(runtime.mod, "Agent", agent)
    runtime.mod.main()
    e = runtime.events
    assert "handshake" in e and "runner" in e                      # 저하여도 ready_handshake 가 불린다
    assert ("degraded", ("no_drive",)) in e
    assert isinstance(seen["agent"]["stack"], NoDriveStack)
    assert seen["agent"]["degrade"].flags == {"no_drive"}
    assert seen["timeouts"][0] == 60.0 and seen["timeouts"][1] <= 1.0   # 핸드셰이크 재확인은 짧게(제한시간을 또 쓰지 않는다)
    assert "close-stack" in e                                      # 래퍼를 통해 안쪽 스택도 닫힌다


def test_normal_state_leaves_the_agent_call_unchanged(runtime, monkeypatch):
    seen = {}
    monkeypatch.setattr(runtime.mod, "wait_ready", lambda *a, **k: ReadyState())
    monkeypatch.setattr(runtime.mod, "Agent", lambda *a, **kw: seen.update(kw))
    runtime.mod.main()
    assert "degrade" not in seen and not isinstance(seen["stack"], NoDriveStack)


def test_no_vision_skips_the_first_real_image_inference(runtime, monkeypatch):
    monkeypatch.setattr(runtime.mod, "wait_ready", lambda *a, **k: ready_state({"no_vision"}, ["image"]))
    runtime.mod.main()
    assert "warmup" not in runtime.events and "handshake" in runtime.events


def test_preflight_only_exits_nonzero_when_degraded_and_zero_otherwise(runtime, monkeypatch, capsys):
    runtime.argv[1:] = ["--preflight-only"]
    monkeypatch.setattr(runtime.mod, "wait_ready", lambda *a, **k: ready_state({"no_drive"}, ["scan"]))
    assert runtime.mod.main() == 2
    assert "scan" in capsys.readouterr().out and "handshake" not in runtime.events
    monkeypatch.setattr(runtime.mod, "wait_ready", lambda *a, **k: ReadyState())
    assert not runtime.mod.main()
