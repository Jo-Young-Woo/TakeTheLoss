"""Real HAL readiness, service races and sensor-frame contracts (no ROS required)."""
import math
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest

from maicon26.hal.ros_mil import RosMilTransport, scan_in_base
from maicon26.hal.odom_tf import pose_to_transform
from test_mil_hal import _robot
from test_odom_tf import _odom


def test_unknown_match_never_moves_or_calls_services():
    r, tr, clk = _robot()
    r.set_velocity(0.3, 0.2)
    r.set_pan_tilt(20, 0)
    r.pump()
    assert tr.cmds == [(0, 0)]
    assert not tr.gimbal_calls and not r.laser_fire() and not tr.fire_calls


def test_preflight_practice_does_not_center_gimbal_without_explicit_command():
    r, tr, clk = _robot()
    r.on_match("practice", "idle")
    r.pump()
    assert tr.cmds == [(0, 0)] and not tr.gimbal_calls and not tr.fire_calls


def test_preflight_rejects_other_final_velocity_publishers_only():
    import threading
    from maicon26.config import Config
    tr = object.__new__(RosMilTransport)
    tr.cfg, tr._lock = Config(), threading.Lock()
    counts = {"/cmd_vel": 1, "/cmd_vel_nav2": 2}
    tr.node = NS(count_publishers=lambda topic: counts[topic])
    tr._cli_gimbal = tr._cli_fire = NS(service_is_ready=lambda: True)
    tr._pub_oled = object()
    assert tr.readiness() == (True, [])
    counts["/cmd_vel"] = 2
    assert tr.readiness() == (False, ["multiple_publishers:/cmd_vel"])
    tr.cfg.hal.cmd_topic = "/robot/cmd_vel"
    counts["/robot/cmd_vel"] = 2
    assert tr.readiness() == (False, ["multiple_publishers:/robot/cmd_vel"])


class _PauseAfterUnlock:
    """Inject pause after checking control but before service dispatch."""
    def __init__(self, robot, unlocks):
        self.lock, self.robot, self.left = robot._lock, robot, unlocks

    def __enter__(self):
        self.lock.acquire()
        return self

    def __exit__(self, *args):
        self.lock.release()
        self.left -= 1
        if self.left == 0:
            self.robot.on_match("mission", "pause")


@pytest.mark.parametrize("action,unlocks", [("fire", 1), ("gimbal", 2)])
def test_pause_between_control_check_and_dispatch_prevents_service(action, unlocks):
    r, tr, clk = _robot(fire_wait_s=0.001)
    r.on_match("mission", "start")
    r._lock = _PauseAfterUnlock(r, unlocks)
    if action == "fire":
        assert not r.laser_fire()
    else:
        r.set_pan_tilt(20, 0)
    assert r.status().paused
    assert not tr.fire_calls and not tr.gimbal_calls


@pytest.mark.parametrize("action", ["fire", "gimbal"])
def test_service_enqueue_is_serialized_with_pause(action):
    r, tr, clk = _robot(fire_wait_s=0.001)
    r.on_match("mission", "start")
    entered, release, pause_started, pause_done = (threading.Event() for _ in range(4))
    original = getattr(tr, "call_" + action)

    def enqueue(*args):
        entered.set()
        assert release.wait(2)
        original(*args)  # synchronous callback must reenter RLock safely

    setattr(tr, "call_" + action, enqueue)
    send = threading.Thread(target=r.laser_fire if action == "fire" else lambda: r.set_pan_tilt(20, 0))

    def pause():
        pause_started.set()
        r.on_match("mission", "pause")
        pause_done.set()

    stop = threading.Thread(target=pause)
    send.start()
    try:
        assert entered.wait(2)
        stop.start()
        assert pause_started.wait(2)
        assert not pause_done.wait(0.05)
    finally:
        release.set()
        send.join(2)
        if stop.ident is not None:
            stop.join(2)
    assert not send.is_alive() and not stop.is_alive()
    assert pause_done.is_set() and r.status().paused
    assert not r.status().laser_on


def test_readiness_requires_received_fresh_samples_not_default_pose():
    r, tr, clk = _robot()
    assert r.get_odom().x == 0
    assert "odom" in r.readiness()[1]
    r.on_state(None, None, None)
    assert "state" in r.readiness()[1]
    r.on_match("mission", "standby")
    r.on_state(4, 4, 90)
    r.on_odom(0, 0, 0)
    r.on_scan(-math.pi, math.pi / 2, [1, 1, 1, 1], 0, 8)
    r.on_image(np.zeros((2, 2, 3), dtype=np.uint8), 0)
    r.on_imu(0, 0, 0, 1)
    assert r.readiness() == (True, [])
    assert r.status().control is False  # standby != permission to drive
    clk.t += 1.2
    assert "odom" in r.readiness()[1] and "match" not in r.readiness()[1]
    r.on_odom(float("nan"), 0, 0)
    assert "odom" in r.readiness()[1]


def test_fire_timeout_cannot_send_duplicate_or_report_success():
    r, tr, clk = _robot(fire_wait_s=0.001)
    r.on_match("mission", "start")
    tr.defer = True
    assert not r.laser_fire()
    assert not r.laser_fire() and len(tr.fire_calls) == 1
    assert r.status().ammo == 8
    tr.flush()
    assert r.status().ammo == 7 and r.status().laser_on


def test_late_fire_after_pause_does_not_reenable_emission():
    r, tr, clk = _robot(fire_wait_s=0.001)
    r.on_match("mission", "start")
    tr.defer = True
    r.laser_fire()
    r.on_match("mission", "pause")
    tr.flush()
    assert r.status().ammo == 7  # confirmed request consumed one round
    assert not r.status().laser_on and not r.status().control


def test_late_gimbal_reply_cannot_overwrite_newer_request():
    r, tr, clk = _robot()
    r.on_match("mission", "start")
    tr.defer = True
    r.set_pan_tilt(10, 0)
    old = tr._deferred.pop()
    clk.t += 1.1
    r.set_pan_tilt(30, 0)
    tr.flush()
    old()
    assert r.gimbal.target == (30, 0)
    assert r.stats["gimbal_ok"] == 1


def test_pause_freezes_servo_and_ignores_late_ack():
    r, tr, clk = _robot()
    r.on_match("mission", "start")
    r.set_pan_tilt(40, 0)
    clk.t += 0.1
    r.on_match("mission", "pause")
    p = r.get_pan_tilt()
    clk.t += 3
    assert r.get_pan_tilt() == p


def test_scan_offset_and_yaw_are_applied_without_filling_unknown_rays():
    a, inc, out, rmax = scan_in_base(0, math.pi / 2, [1, float("inf"), float("nan"), 0.01], .1, 8, .2, .1, math.pi / 2)
    valid = np.where(np.isfinite(out))[0]
    assert len(valid) == 1
    assert out[valid[0]] == pytest.approx(math.hypot(.2, 1.1))
    assert a + valid[0] * inc == pytest.approx(math.pi / 2)
    assert rmax == pytest.approx(8 + math.hypot(.2, .1))


def test_scan_rebin_keeps_nearest_when_two_endpoints_share_direction():
    _, _, out, _ = scan_in_base(0, math.pi / 4, [1, 1], .1, 8, 10, 0, 0)
    assert np.isfinite(out).sum() == 1
    assert out[np.isfinite(out)][0] < 11


def test_odom_bridge_refuses_frame_relabel_without_transform():
    m = _odom()
    m.header.frame_id = "odom"
    m.child_frame_id = "base_footprint"
    with pytest.raises(ValueError, match="frames differ"):
        pose_to_transform(m, "odom", "base_link")
