"""IMU 충돌 대응: HAL(IMU → ImpactDetector) → 매니저(정지·검증·후속 행동) → ImpactResponder 분류.
ROS·시뮬 없이 가짜 로봇·스택·시계로 돈다. 임계값은 [가정] — 현장에서 실제 충돌로 다시 맞춰야 한다."""
import math
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.mission.impact import ImpactCtx, ImpactResponder
from maicon26.mission.manager import Agent, Mode
from maicon26.nav.grid import OccupancyGrid
from maicon26.nav.pose_guard import ImpactEvent, PoseGuard
from maicon26.types import Pose2D, RobotStatus, Scan

from hw_fakes import make_robot


def scan_with(front=None, side=None, default=3.0):
    """360 광선 스캔. front(m)·side(m)가 있으면 그 방향(±20°)을 그 거리로 막는다."""
    ang = np.deg2rad(np.arange(-180, 180, 1.0))
    r = np.full(len(ang), default)
    if front is not None:
        r[np.abs(ang) <= math.radians(20)] = front
    if side is not None:
        r[np.abs(ang - math.pi / 2) <= math.radians(20)] = side
    return Scan(-math.pi, math.radians(1.0), r, stamp=1.0, range_max=8.0)


def ev(t=10.0, dg=0.8, bearing=0.0, gyro=0.0, src="accel"):
    return ImpactEvent(t, dg, bearing, gyro, src)


def cfg_on():
    c = Config()
    c.impact.enable = True
    return c


# ====================================================================== 분류·행동 (순수 계산)
def test_front_hit_on_a_visible_wall_while_driving_forward_backs_off():
    ir = ImpactResponder(cfg_on().impact)
    ctx = ImpactCtx(Pose2D(2, 2, 0), v_cmd=0.3, scan=scan_with(front=0.25))
    p = ir.handle(10.0, [ev()], ctx)
    assert (p.sector, p.contact, p.moving, p.action) == ("front", "visible", "forward", "backoff") and p.bump_xy is None
    assert p.hold_s == pytest.approx(cfg_on().impact.hold_s) and p.severity == "minor"
    assert ir.hold_active(10.2) and not ir.hold_active(10.5)


def test_front_hit_with_nothing_in_lidar_is_recorded_as_an_invisible_obstacle_ahead():
    ir = ImpactResponder(cfg_on().impact)
    p = ir.handle(10.0, [ev()], ImpactCtx(Pose2D(1.0, 2.0, math.pi / 2), v_cmd=0.3, scan=scan_with()))
    assert p.contact == "invisible" and p.action == "backoff"
    assert p.bump_xy == pytest.approx((1.0, 2.22), abs=1e-6) and p.bump_th == pytest.approx(math.pi / 2)


def test_bump_direction_follows_the_contact_bearing_not_the_heading():
    ir = ImpactResponder(cfg_on().impact)
    p = ir.handle(10.0, [ev(bearing=math.pi / 2)], ImpactCtx(Pose2D(1.0, 1.0, 0.0), w_cmd=0.9, scan=scan_with()))
    assert p.sector == "left" and p.contact == "invisible" and p.action == "replan"
    assert p.bump_xy == pytest.approx((1.0, 1.22), abs=1e-6)


def test_hit_while_standing_still_is_not_recorded_and_only_replans():
    ir = ImpactResponder(cfg_on().impact)
    p = ir.handle(10.0, [ev()], ImpactCtx(Pose2D(2, 2, 0), scan=scan_with()))
    assert (p.contact, p.moving, p.action, p.bump_xy) == ("unknown", "still", "replan", None)


def test_unknown_direction_never_records_or_backs_off():
    ir = ImpactResponder(cfg_on().impact)
    p = ir.handle(10.0, [ev(bearing=None, gyro=500.0, dg=0.0, src="gyro")], ImpactCtx(Pose2D(2, 2, 0), v_cmd=0.3, scan=scan_with()))
    assert p.sector == "unknown" and p.contact == "unknown" and p.action == "replan" and p.severity == "major" and p.hold_s == pytest.approx(0.8)


def test_ugv_next_to_us_makes_it_a_ugv_contact_not_a_bump():
    ir = ImpactResponder(cfg_on().impact)
    ctx = ImpactCtx(Pose2D(2, 2, 0), v_cmd=0.3, scan=scan_with(), ugv_xy=[(2.5, 2.05)])
    p = ir.handle(10.0, [ev()], ctx)
    assert p.contact == "ugv" and p.action == "backoff" and p.bump_xy is None
    far = ir.handle(20.0, [ev(t=20.0)], ImpactCtx(Pose2D(2, 2, 0), v_cmd=0.3, scan=scan_with(), ugv_xy=[(2.0, 3.5), (0.5, 2.0)]))
    assert far.contact == "invisible"                                  # 앞쪽이 아닌 곳·먼 곳의 UGV 는 이 접촉의 원인이 아니다


def test_rear_and_side_hits_do_not_back_off():
    ir = ImpactResponder(cfg_on().impact)
    rear = ir.handle(10.0, [ev(bearing=math.pi)], ImpactCtx(Pose2D(2, 2, 0), v_cmd=-0.1, scan=scan_with()))
    assert rear.sector == "rear" and rear.action == "replan"
    side = ir.handle(20.0, [ev(t=20.0, bearing=-math.pi / 2)], ImpactCtx(Pose2D(2, 2, 0), v_cmd=0.3, scan=scan_with(side=None)))
    assert side.sector == "right" and side.action == "replan"


def test_goal_phase_minor_hit_replans_but_major_still_backs_off():
    ir = ImpactResponder(cfg_on().impact)
    ctx = ImpactCtx(Pose2D(2, 2, 0), v_cmd=0.3, scan=scan_with(front=0.25), mode="GOAL")
    assert ir.handle(10.0, [ev()], ctx).action == "replan"
    assert ir.handle(30.0, [ev(t=30.0, dg=1.5)], ctx).action == "backoff"


def test_backoff_can_be_switched_off():
    c = cfg_on()
    c.impact.backoff = False
    p = ImpactResponder(c.impact).handle(10.0, [ev()], ImpactCtx(Pose2D(2, 2, 0), v_cmd=0.3, scan=scan_with(front=0.2)))
    assert p.action == "replan"


def test_storm_holds_longer_slows_down_and_expires():
    c = cfg_on()
    ir = ImpactResponder(c.impact)
    ctx = ImpactCtx(Pose2D(2, 2, 0), v_cmd=0.3, scan=scan_with(front=0.2))
    for t in (10.0, 12.0):
        assert not ir.handle(t, [ev(t=t)], ctx).storm
    p = ir.handle(14.0, [ev(t=14.0)], ctx)
    assert p.storm and p.hold_s == pytest.approx(c.impact.storm_hold_s)
    assert ir.speed_scale(15.0) == pytest.approx(0.6) and ir.speed_scale(14.0 + c.impact.storm_window_s + 0.1) == 1.0
    assert ir.stats["storm"] == 1


def test_not_running_only_records_and_does_not_hold():
    ir = ImpactResponder(cfg_on().impact)
    assert ir.handle(10.0, [ev()], ImpactCtx(Pose2D(2, 2, 0), scan=scan_with()), match_running=False) is None
    assert not ir.hold_active(10.1) and ir.stats["events"] == 1 and ir.stats["ignored_no_start"] == 1
    assert ir.skip_scan(10.1, 0.0, 0.0)                                  # 그래도 직후 스캔은 격자에 쌓지 않는다


def test_hold_waits_for_verification_up_to_the_timeout():
    c = cfg_on()
    ir = ImpactResponder(c.impact)
    ir.handle(10.0, [ev()], ImpactCtx(Pose2D(2, 2, 0), scan=scan_with()))
    assert ir.hold_active(10.6, verifying=True) and not ir.hold_active(10.6, verifying=False)
    assert not ir.hold_active(10.0 + c.impact.verify_timeout_s + 0.1, verifying=True)       # 검증이 끝나지 않아도 한도가 지나면 재개


def test_followup_is_handed_out_once_and_major_wins():
    ir = ImpactResponder(cfg_on().impact)
    ctx = ImpactCtx(Pose2D(2, 2, 0), v_cmd=0.3, scan=scan_with(front=0.2))
    ir.handle(10.0, [ev(dg=0.6), ev(t=10.05, dg=1.6)], ctx)
    f = ir.followup()
    assert f is not None and f.severity == "major" and ir.followup() is None


def test_skip_scan_after_impact_and_when_tilted():
    c = cfg_on()
    ir = ImpactResponder(c.impact)
    assert not ir.skip_scan(5.0, 1.0, 2.0)
    assert ir.skip_scan(5.0, 0.0, c.impact.tilt_scan_skip_deg + 1.0) and ir.skip_scan(5.0, None, None) is False
    ir.handle(10.0, [ev()], ImpactCtx(Pose2D(2, 2, 0), scan=scan_with()))
    assert ir.skip_scan(10.2, 0.0, 0.0) and not ir.skip_scan(10.0 + c.impact.scan_skip_s + 0.05, 0.0, 0.0)


# ====================================================================== HAL: IMU → 감지기
def imu_stream(robot, clock, seconds, hz=100.0, ax=0.0, ay=0.0, g=1.0, kick=None, roll=0.0):
    """kick = (시각, ax_g, ay_g) 한 표본에 충격을 넣는다."""
    n = int(seconds * hz)
    for i in range(n):
        clock.t += 1.0 / hz
        a, b, gg = ax, ay, g
        if kick is not None and abs(clock.t - kick[0]) < 0.5 / hz:
            a, b, gg = kick[1], kick[2], math.sqrt(kick[1] ** 2 + kick[2] ** 2 + 1.0)
        robot.on_imu(roll, 0.0, 0.0, gg, a, b, 0.0)


def test_hal_off_by_default_has_no_detector_and_keeps_old_signature():
    robot, _, clock = make_robot()
    assert robot._impact_det is None and robot.pop_impacts() == []
    robot.on_imu(1.0, 2.0, 3.0, 1.0)                                      # 예전 4개 인자 호출이 그대로 된다
    assert robot.status().accel_g == pytest.approx(1.0)


def test_hal_detects_a_frontal_hit_and_pops_it_once():
    robot, _, clock = make_robot(cfg=cfg_on())
    imu_stream(robot, clock, 2.0)
    assert robot.pop_impacts() == []
    imu_stream(robot, clock, 0.5, kick=(clock.t + 0.2, -1.4, 0.0))       # 차체가 앞에서 부딪히면 가속도는 뒤(-x)로 쏠린다
    imu_stream(robot, clock, 0.5)
    got = robot.pop_impacts()
    assert len(got) == 1 and got[0].sector == "front" and got[0].severity == "major"
    assert robot.pop_impacts() == [] and robot.stats["impact_events"] == 1


def test_hal_does_not_judge_impacts_while_tilted_past_the_stop_limit():
    robot, _, clock = make_robot(cfg=cfg_on())
    imu_stream(robot, clock, 2.0)
    tilt = robot.cfg.hal.tilt_stop_deg + 5.0
    imu_stream(robot, clock, 0.4, kick=(clock.t + 0.2, -1.4, 0.0), roll=tilt)
    assert robot.pop_impacts() == []


# ====================================================================== 매니저
CODE = "Impact88"


@pytest.fixture
def make_agent(monkeypatch):
    from maicon26.link.async_link import AsyncServerLink
    from maicon26.link.http import HttpServerLink
    from maicon26.sim.mockserver import FakeClock, MockServer
    monkeypatch.setattr(threading.Thread, "start", lambda _: None)
    made = []

    def build(cfg=None, events=None, scan=None, guard=True, pose=Pose2D(3.0, 1.5, 0.0)):
        cfg = cfg or Config()
        cfg.team, cfg.slot = "BLUE", 2
        cfg.hal.restart_state_path = None
        clock = FakeClock(10.)
        mock = MockServer({CODE: "B-2"}, clock=clock)
        mock.set_phase("running")
        link = HttpServerLink("http://fixture.invalid", CODE, clock=clock, transport=mock)
        link.me()
        link._started_bg = True
        transport = AsyncServerLink(link)
        queue = list(events or [])
        robot = SimpleNamespace(get_scan=lambda: scan, get_frame=lambda: None, get_depth=lambda: None,
                                get_odom=lambda: Pose2D(), now=lambda: clock() + 90.,
                                pop_impacts=lambda: [queue.pop(0) for _ in range(len(queue))],
                                status=lambda: RobotStatus(roll_deg=0.0, pitch_deg=0.0))
        grid = OccupancyGrid(cfg.arena)
        calls = SimpleNamespace(clear=0, layers=[])
        g = PoseGuard(cfg.pose_guard, cfg.arena) if guard else None
        nav = SimpleNamespace(initialized=True, pose=pose, grid=grid, update=lambda *a: None,
                              clear_goal=lambda: setattr(calls, "clear", calls.clear + 1),
                              set_obstacle_layer=lambda m: calls.layers.append(m), guard=g)
        agent = Agent(robot, transport, cfg=cfg, stack=nav)
        agent.tracker = SimpleNamespace(tracks=[], update_uav=lambda *a, **k: None)
        agent.t_start = 0.0
        agent._scan = scan                                                 # _sense 가 매 틱 채우는 최신 스캔
        agent.sw.fresh = lambda now: True                                  # LiDAR 는 신선하다고 본다(이 시험의 대상이 아니다)
        agent._test_logs = []
        agent._log = lambda now, msg: agent._test_logs.append((now, msg))
        made.append(agent)
        return SimpleNamespace(agent=agent, cfg=cfg, robot=robot, nav=nav, calls=calls, queue=queue, guard=g)

    yield build
    for a in made:
        try:
            a.close_deadlock()
        except Exception:                                                  # noqa: BLE001
            pass


def st():
    return RobotStatus(roll_deg=0.0, pitch_deg=0.0)


def test_manager_without_features_has_no_responder_and_never_reports_impact(make_agent):
    h = make_agent(Config(), guard=False)
    a = h.agent
    assert a._impact is None
    a._impact_update(100.0)
    assert a._unsafe_reason(100.0, st()) is None and a._speed_guard_scale(100.0) == 1.0


def test_event_stops_the_robot_then_resumes_after_hold_and_verification(make_agent):
    cfg = Config()
    cfg.impact.enable = True
    cfg.pose_guard.enable = True
    h = make_agent(cfg, events=[ev(t=100.0)], scan=scan_with(front=0.25))
    a = h.agent
    a._v_cmd = 0.3
    now = 100.05
    a._impact_update(now)
    assert any("IMPACT" in m for _, m in a._test_logs) and a.stats["impacts"] == 1
    assert h.guard.verifying                                              # 충격 직전 기준 오프셋을 굳히고 검증을 시작했다
    assert a._unsafe_reason(now, st()) == "impact"
    assert a._unsafe_reason(100.6, st()) == "impact"                      # 정지 시간은 지났지만 검증 중
    assert a._unsafe_reason(100.0 + cfg.impact.verify_timeout_s + 0.2, st()) is None


def test_hold_ends_when_verification_finishes_early(make_agent):
    cfg = Config()
    cfg.impact.enable = True
    cfg.pose_guard.enable = True
    h = make_agent(cfg, events=[ev(t=100.0)], scan=scan_with())
    a = h.agent
    a._impact_update(100.05)
    h.guard._v = None                                                       # 검증이 끝났다
    assert a._unsafe_reason(100.2, st()) == "impact"                       # 정지 시간 안
    assert a._unsafe_reason(100.6, st()) is None


def test_invisible_front_hit_records_a_bump_and_updates_the_obstacle_layer(make_agent):
    cfg = Config()
    cfg.impact.enable = True
    h = make_agent(cfg, events=[ev(t=100.0)], scan=scan_with(), guard=False, pose=Pose2D(3.0, 1.5, 0.0))
    a = h.agent
    a._v_cmd = 0.3
    n0 = a.stats["bump_events"]
    a._impact_update(100.05)
    assert a.stats["bump_events"] == n0 + 1 and len(a._bumps) == 1 and h.calls.layers
    assert a._bumps[0][1:3] == pytest.approx((3.22, 1.5))


def test_visible_front_hit_does_not_record_a_bump(make_agent):
    cfg = Config()
    cfg.impact.enable = True
    h = make_agent(cfg, events=[ev(t=100.0)], scan=scan_with(front=0.2), guard=False)
    a = h.agent
    a._v_cmd = 0.3
    a._impact_update(100.05)
    assert a._bumps == [] and a._unsafe_reason(100.1, st()) == "impact"


def test_followup_backs_off_after_a_front_hit_and_replans_otherwise(make_agent):
    cfg = Config()
    cfg.impact.enable = True
    h = make_agent(cfg, events=[ev(t=100.0)], scan=scan_with(front=0.2), guard=False)
    a = h.agent
    a._v_cmd = 0.3
    a._impact_update(100.05)
    a.mode = Mode.EXPLORE
    a._impact_followup(101.0)
    assert h.calls.clear >= 1 and a.mode == Mode.RECOVER and a.stats["recoveries"] == 1
    n = h.calls.clear
    a._impact_followup(101.1)                                              # 후속은 한 번만
    assert h.calls.clear == n and a.stats["recoveries"] == 1

    h2 = make_agent(cfg, events=[ev(t=100.0, bearing=math.pi / 2)], scan=scan_with(), guard=False)
    h2.agent._w_cmd = 0.9
    h2.agent._impact_update(100.05)
    h2.agent.mode = Mode.EXPLORE
    h2.agent._impact_followup(101.0)
    assert h2.calls.clear == 1 and h2.agent.mode == Mode.EXPLORE and h2.agent.stats["recoveries"] == 0


def test_events_before_the_match_start_do_not_stop_the_robot(make_agent):
    cfg = Config()
    cfg.impact.enable = True
    cfg.pose_guard.enable = True
    h = make_agent(cfg, events=[ev(t=100.0)], scan=scan_with())
    h.agent.t_start = None
    h.agent._impact_update(100.05)
    assert h.agent._unsafe_reason(100.1, st()) is None
    assert h.guard.verifying                                              # 위치 검증은 시작 전에도 한다
    assert any("경기 시작 전" in m for _, m in h.agent._test_logs)


def test_virtual_impact_from_a_heading_step_holds_the_robot(make_agent):
    cfg = Config()
    cfg.pose_guard.enable = True
    h = make_agent(cfg, scan=scan_with())
    a, g = h.agent, h.guard
    for i in range(30):                                                    # 안정된 기준 오프셋을 배운다
        g.process(50.0 + 0.1 * i, Pose2D(3.0, 1.5, 0.2), Pose2D(0, 0, 0.2), None)
    assert g.step_detected(53.0) is None
    for i in range(6):                                                     # 정지 상태에서 SLAM 헤딩만 8° 계단
        g.process(53.0 + 0.1 * i, Pose2D(3.0, 1.5, 0.2 + math.radians(8)), Pose2D(0, 0, 0.2), None)
    a._impact_update(54.0)
    assert any("IMPACT" in m and "yaw_step" in m for _, m in a._test_logs)
    assert a._unsafe_reason(54.1, st()) == "impact"


def test_pose_lost_stops_and_requires_new_scans(make_agent):
    cfg = Config()
    cfg.pose_guard.enable = True
    h = make_agent(cfg, scan=scan_with())
    h.guard.state = "lost"
    assert h.agent._unsafe_reason(100.0, st()) == "pose_lost"
    h.guard.state = "ok"
    assert h.agent._unsafe_reason(100.0, st()) is None


def test_speed_scale_combines_guard_degradation_and_storm(make_agent):
    cfg = Config()
    cfg.impact.enable = True
    cfg.pose_guard.enable = True
    h = make_agent(cfg, scan=scan_with())
    a = h.agent
    assert a._speed_guard_scale(100.0) == 1.0
    h.guard.state = "degraded"
    assert a._speed_guard_scale(100.0) == pytest.approx(cfg.pose_guard.degraded_speed_scale)
    h.guard.state = "ok"
    a._impact._slow_until = 200.0
    assert a._speed_guard_scale(100.0) == pytest.approx(0.6)
    h.guard.state = "degraded"
    assert a._speed_guard_scale(100.0) == pytest.approx(min(0.6, cfg.pose_guard.degraded_speed_scale))


def test_uav_map_pose_subtracts_the_uav_alignment_and_reports_state(make_agent):
    cfg = Config()
    cfg.pose_guard.enable = True
    h = make_agent(cfg, scan=scan_with(), pose=Pose2D(3.0, 1.5, 0.3))
    a = h.agent
    assert a.uav_map_pose() == {"x": 3.0, "y": 1.5, "th": 0.3, "aligned": False, "state": "ok"}
    a.coverage.uav_alignment_report = {"dx": 0.12, "dy": -0.05, "matches": 3, "rms": 0.02, "t": 1.0}
    assert a.uav_map_pose()["x"] == pytest.approx(2.88) and a.uav_map_pose()["y"] == pytest.approx(1.55)
    h.guard.state = "degraded"
    assert a.uav_map_pose()["state"] == "degraded"


def test_pose_guard_tick_reports_map_link_and_warns_on_alignment_change(make_agent):
    cfg = Config()
    cfg.pose_guard.enable = True
    h = make_agent(cfg, scan=scan_with())
    a, g = h.agent, h.guard
    g.process(50.0, Pose2D(3.0, 1.5, 0.0), Pose2D(0, 0, 0.0), None)
    a.coverage.uav_alignment_report = {"dx": 0.02, "dy": 0.0, "matches": 3, "rms": 0.02, "t": 1.0}
    a._pose_guard_tick(60.0, g)
    ml = a.stats["map_link"]
    assert ml["uav_alignment"]["dx"] == 0.02 and ml["pose_state"] == "ok" and ml["uav_map_pose"]["aligned"]
    assert not any("정렬이" in m for _, m in a._test_logs)
    a.coverage.uav_alignment_report = {"dx": 0.20, "dy": 0.0, "matches": 3, "rms": 0.02, "t": 2.0}
    a._pose_guard_tick(62.0, g)
    assert any("UAV↔LiDAR 정렬이" in m for _, m in a._test_logs)
