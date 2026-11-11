"""ROS-free checks for the Fortress transport; no running simulator is touched."""
import math
from pathlib import Path
import re
import sys
from types import ModuleType, SimpleNamespace as NS
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.hal import gazebo
from maicon26.hal.gazebo_model import BASE_FRAME, LIDAR_FRAME, LIDAR_RPY, LIDAR_XYZ, lidar_frame
from scripts.generate_gazebo_world import robot_model
from scripts.run_gazebo_match import ready_step, simulation_delta

ROOT = Path(__file__).resolve().parents[1]


class Node:
    def __init__(self, name):
        self.name, self.subscriptions, self.timers, self.sent = name, [], [], []

    def create_subscription(self, kind, topic, callback, qos):
        self.subscriptions.append((topic, callback, qos))

    def create_publisher(self, *args):
        return NS(publish=lambda msg: self.sent.append((msg.linear.x, msg.angular.z)))

    def create_timer(self, period, callback):
        self.timers.append((period, callback))
        return callback


@pytest.fixture
def robot_factory(monkeypatch):
    wall = NS(value=10.0)
    qos = object()
    modules = {
        "geometry_msgs.msg": {"Twist": lambda: NS(linear=NS(x=0.), angular=NS(z=0.))},
        "nav_msgs.msg": {"Odometry": object},
        "rclpy.node": {"Node": Node},
        "rclpy.qos": {"qos_profile_sensor_data": qos},
        "rosgraph_msgs.msg": {"Clock": object},
        "sensor_msgs.msg": {"LaserScan": object},
    }
    for name, attrs in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(gazebo, "ensure_ros", lambda: None)
    monkeypatch.setattr(gazebo, "add_node", lambda node: None)
    monkeypatch.setattr(gazebo, "time", NS(monotonic=lambda: wall.value))
    return lambda prefix="": gazebo.GazeboRobot(prefix), wall, qos


def stamp(t):
    sec = math.floor(t)
    return NS(sec=sec, nanosec=round((t-sec) * 1e9))


def scan(t, frame=LIDAR_FRAME):
    return NS(header=NS(stamp=stamp(t), frame_id=frame),
              angle_min=0., angle_increment=math.pi / 180.,
              range_min=.05, range_max=8., ranges=[1.])


def odom(t):
    return NS(header=NS(stamp=stamp(t)), pose=NS(pose=NS(
        position=NS(x=1., y=2.), orientation=NS(x=0., y=0., z=0., w=1.))))


def receive(robot, t=1., *, scan_stamp=None, odom_stamp=None):
    robot._on_clock(NS(clock=stamp(t)))
    robot._on_scan(scan(t if scan_stamp is None else scan_stamp, robot._lidar_frame))
    robot._on_odom(odom(t if odom_stamp is None else odom_stamp))


def test_qos_accepts_best_effort_bridge_and_watchdog_runs_without_sim_time(robot_factory):
    make, _, qos = robot_factory
    robot = make()
    assert {topic for topic, _, _ in robot.node.subscriptions} == {"/clock", "/scan", "/odom"}
    assert all(profile is qos for _, _, profile in robot.node.subscriptions)
    assert robot.node.timers == [(.1, robot._watchdog)]


def test_ready_requires_all_current_inputs(robot_factory):
    make, _, _ = robot_factory
    robot = make()
    assert not robot.ready()
    robot._on_clock(NS(clock=stamp(1.)))
    robot._on_scan(scan(1.))
    assert not robot.ready()
    robot._on_odom(odom(1.))
    assert robot.ready()


def test_optional_arena_localization_quality_inhibits_drive_when_missing_invalid_or_stale(robot_factory, monkeypatch):
    import json
    _, wall, _ = robot_factory
    module = ModuleType("std_msgs.msg")
    module.String = object
    monkeypatch.setitem(sys.modules, "std_msgs.msg", module)
    robot = gazebo.GazeboRobot(require_arena_localization=True)
    receive(robot)
    assert not robot.ready()
    robot.set_velocity(.2, 0.)
    assert robot.last_cmd == (0., 0.)
    robot._on_localization_quality(NS(data=json.dumps({"valid": True, "stamp": 1., "status": "tracking"})))
    assert robot.ready()
    robot.set_velocity(.2, 0.)
    assert robot.last_cmd == (.2, 0.)
    robot._on_localization_quality(NS(data=json.dumps({"valid": False, "stamp": 1., "status": "ambiguous"})))
    robot._watchdog()
    assert robot.last_cmd == (0., 0.) and not robot.ready()
    robot._on_localization_quality(NS(data=json.dumps({"valid": True, "stamp": 1.})))
    wall.value += .6
    receive(robot, 1.6)
    assert not robot.ready()


def test_wall_staleness_stops_existing_velocity_and_hides_old_scan(robot_factory):
    make, wall, _ = robot_factory
    robot = make()
    receive(robot)
    robot.set_velocity(.2, .1)
    assert robot.node.sent[-1] == (.2, .1)
    wall.value += 2.1
    robot._watchdog()
    assert not robot.ready() and robot.get_scan() is None
    assert robot.node.sent[-1] == (0., 0.)
    robot.set_velocity(.4, .2)
    assert robot.node.sent[-1] == (0., 0.)
    receive(robot, 1.1)
    assert robot.ready()


def test_clock_subscription_itself_must_be_recent(robot_factory):
    make, wall, _ = robot_factory
    robot = make()
    receive(robot)
    wall.value += 2.1
    robot._on_scan(scan(1.))
    robot._on_odom(odom(1.))
    assert not robot.ready()


@pytest.mark.parametrize("scan_stamp,odom_stamp", [(1., 2.), (2., 1.), (3., 2.), (2., 3.)])
def test_replayed_or_future_sensor_headers_do_not_refresh_readiness(robot_factory, scan_stamp, odom_stamp):
    make, _, _ = robot_factory
    robot = make()
    receive(robot, 2., scan_stamp=scan_stamp, odom_stamp=odom_stamp)
    assert not robot.ready()


def test_world_reset_latches_failure_and_discards_old_epoch(robot_factory):
    make, _, _ = robot_factory
    robot = make()
    receive(robot, 2.)
    robot._frame = object()
    robot._frame_t = 2.
    receive(robot, .2)
    assert robot.now() == pytest.approx(.2)
    assert robot._frame is None
    assert not robot.ready() and robot.get_scan() is None
    receive(robot, 3.)
    assert not robot.ready()  # New HAL / Agent / referee run is required.


def test_lidar_origin_is_transformed_once_for_internal_scan(robot_factory):
    make, _, _ = robot_factory
    robot = make()
    receive(robot)
    internal = robot.get_scan()
    assert internal.min_in_sector(0., .05) == pytest.approx(1.039815)
    assert np.isfinite(internal.ranges).sum() == 1  # No synthetic free space behind the UGV.


def test_legacy_sdf_frame_is_rejected_with_migration_message(robot_factory):
    make, _, _ = robot_factory
    robot = make()
    receive(robot)
    robot._on_scan(scan(1., "base_link"))
    assert not robot.ready()
    assert "regenerate" in robot.scan_error
    assert robot.get_scan() is None


def test_other_robot_scan_frame_and_topics_are_scoped(robot_factory):
    make, _, _ = robot_factory
    robot = make("R1")
    receive(robot)
    assert robot.ready()
    assert robot._lidar_frame == "R1/gazebo_lidar"
    assert {topic for topic, _, _ in robot.node.subscriptions} == {"/clock", "/R1/scan", "/R1/odom"}


def test_invalid_scan_does_not_escape_ros_callback(robot_factory):
    make, _, _ = robot_factory
    robot = make()
    receive(robot)
    invalid = scan(1.)
    invalid.angle_increment = 0.
    robot._on_scan(invalid)
    assert not robot.ready()
    assert "increment" in robot.scan_error


def test_pause_has_zero_referee_delta_and_stops_commands():
    stops = []
    robot = NS(now=lambda: 5., ready=lambda: True, stop=lambda: stops.append("stop"))
    previous = 5.
    elapsed = 0.
    for _ in range(100):
        previous, delta = ready_step({"B2": robot}, previous)
        elapsed += delta
    assert elapsed == 0. and len(stops) == 100
    robot.now = lambda: 5.1
    assert ready_step({"B2": robot}, previous) == pytest.approx((5.1, .1))


@pytest.mark.parametrize("current", [4., float("nan"), float("inf")])
def test_reset_or_invalid_clock_stops_and_raises(current):
    stops = []
    robot = NS(now=lambda: current, ready=lambda: True, stop=lambda: stops.append("stop"))
    with pytest.raises(RuntimeError, match="clock"):
        ready_step({"B2": robot}, 5.)
    assert stops == ["stop"]


def test_sensor_loss_during_running_match_stops_and_fails():
    stops = []
    robot = NS(now=lambda: 5.1, ready=lambda: False, stop=lambda: stops.append("stop"))
    with pytest.raises(RuntimeError, match="stale"):
        ready_step({"B2": robot}, 5.)
    assert stops == ["stop"]
    assert simulation_delta(5., 5.05) == pytest.approx(.05)


@pytest.mark.parametrize("prefix", ["", "/R1"])
def test_generated_scan_frame_matches_its_actual_sdf_pose(prefix):
    model = robot_model(ET.Element("world"), Config(), 0., 0., 0., topic_prefix=prefix)
    sensor = model.find("link[@name='base_link']/sensor[@name='laser']")
    assert sensor.findtext("ignition_frame_id") == lidar_frame(prefix)
    assert tuple(map(float, sensor.findtext("pose").split())) == (*LIDAR_XYZ, *LIDAR_RPY)
    assert LIDAR_XYZ == (.039815, 0., .16)


def test_demo_static_tf_matches_generator_and_is_cleaned_up():
    script = (ROOT / "scripts/gazebo_nav_demo.sh").read_text(encoding="utf-8")
    xyzrpy = tuple(float(re.search(rf"--{key} ([\d.\-]+)", script)[1])
                  for key in ("x", "y", "z", "roll", "pitch", "yaw"))
    assert xyzrpy == (*LIDAR_XYZ, *LIDAR_RPY)
    assert f"--frame-id {BASE_FRAME} --child-frame-id {LIDAR_FRAME}" in script
    assert "robot_state_publisher.launch" not in script
    assert 'kill -INT -- "-$NAV_PID" "-$LIDAR_TF_PID"' in script
    assert 'kill -KILL -- "-$NAV_PID" "-$LIDAR_TF_PID"' in script
