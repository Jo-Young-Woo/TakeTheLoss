"""Sensor evidence, not service goals, determines calibrated image geometry."""
import copy
from collections import deque
import math
from pathlib import Path
from types import SimpleNamespace as NS
import sys

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.hal.wingbot import WingbotRobot, Transport
from maicon26.hal.wingbot_calibration import configure_wingbot_simulation, WingbotCameraCalibration

ROOT = Path(__file__).resolve().parents[1]


def configured():
    cfg = Config()
    profile = configure_wingbot_simulation(cfg, ROOT/"ros2/wingbot_description/config/frames.yaml")
    return cfg, profile, WingbotCameraCalibration(profile)


def info(profile, **kw):
    return NS(**dict(dict(width=640, height=480, header=NS(frame_id=profile["optical_frame"]),
                         k=profile["output_K"], d=[-.25, 0., 0., 0., 0.], distortion_model="plumb_bob",
                         p=[999., 0., 10., 0., 0., 999., 10., 0., 0., 0., 1., 0.]), **kw))


def joints(profile, stamp=1., pan=.2, tilt=.1):
    return NS(header=NS(stamp=NS(sec=int(stamp), nanosec=round((stamp-int(stamp))*1e9))),
              name=["ugv_B2::"+profile["pan_joint"], profile["tilt_joint"]], position=[pan, tilt])


def tf(xyz, pan=0., tilt=0.):
    # Rz(pan)*Ry(-tilt): internal left+/up+ lens axis.
    q = [math.sin(pan/2)*math.sin(tilt/2), -math.cos(pan/2)*math.sin(tilt/2),
         math.sin(pan/2)*math.cos(tilt/2), math.cos(pan/2)*math.cos(tilt/2)]
    return NS(translation=NS(**dict(zip("xyz", xyz))), rotation=NS(**dict(zip("xyzw", q))))


def test_simulation_profile_is_explicit_and_does_not_mutate_model_files():
    paths = [ROOT/"ros2/wingbot_description/config"/name for name in ("wingbot_mods.yaml", "frames.yaml")]
    before = [p.read_bytes() for p in paths]
    cfg, p, _ = configured()
    assert Config().camera.calibration_profile is None
    assert cfg.camera.hfov_deg == pytest.approx(math.degrees(2.))
    assert cfg.robot.cam_height == pytest.approx(.2035)
    assert p["lidar_scan_plane_xyz"] == pytest.approx([.0402746383, 0., .15], abs=1e-6)
    assert cfg.rig.base_z + cfg.rig.lidar_xyz[2] == pytest.approx(.15)
    assert cfg.aim.laser_vertical_m == pytest.approx(.0465)
    assert p["laser_from_camera_xyz"] == pytest.approx([-.009, 0., .0465])
    assert p["physical_calibration_verified"] is False
    assert [p.read_bytes() for p in paths] == before


def test_fortress_ogre2_advertised_distortion_is_not_applied_to_pinhole_pixels():
    _, p, cal = configured()
    cal.camera_info(info(p))
    image = np.random.default_rng(10).integers(0, 256, (480, 640, 3), dtype=np.uint8)
    assert np.array_equal(cal.image(image, p["optical_frame"]), image)
    assert cal.info["advertised_D"][0] == -.25
    assert cal.info["effective_D"] == [0.]*5
    assert cal.info["raw_K"] == p["output_K"]  # ignores deliberately different P


def test_actual_camera_info_principal_point_maps_into_fixed_agent_intrinsics():
    _, p, cal = configured()
    k = p["output_K"].copy()
    k[2] -= 1.
    cal.camera_info(info(p, k=k))
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[240, 319] = 255
    out = cal.image(image, p["optical_frame"])
    assert tuple(np.unravel_index(np.argmax(out[:, :, 0]), out.shape[:2])) == (240, 320)
    assert p["output_K"][2] == 320  # cached Agent models remain valid


@pytest.mark.parametrize("change", [dict(width=320), dict(header=NS(frame_id="wrong")),
    dict(k=[0.]*9), dict(k=[float("nan")]*9), dict(d=[-.1, 0, 0, 0, 0]),
    dict(distortion_model="equidistant")])
def test_invalid_camera_info_invalidates_previously_accepted_state(change):
    _, p, cal = configured()
    cal.camera_info(info(p))
    with pytest.raises(ValueError):
        cal.camera_info(info(p, **change))
    with pytest.raises(ValueError, match="CameraInfo"):
        cal.image(np.zeros((480, 640, 3), np.uint8), p["optical_frame"])


def test_image_requires_matching_info_resolution_and_frame():
    _, p, cal = configured()
    with pytest.raises(ValueError):
        cal.image(np.zeros((480, 640, 3), np.uint8), p["optical_frame"])
    cal.camera_info(info(p))
    for image, frame in [(np.zeros((240, 320, 3), np.uint8), p["optical_frame"]),
                         (np.zeros((480, 640, 3), np.uint8), "camera_link")]:
        with pytest.raises(ValueError):
            cal.image(image, frame)


def test_measured_joint_direction_interpolation_and_capture_time_tf_agree():
    _, p, cal = configured()
    cal.joint_state(joints(p, 1., .1, .02))
    cal.joint_state(joints(p, 1.1, .3, .18))
    angles, meta = cal.measured_pose(1.05, tf([.057, 0, .21], -.2, .1), tf([-.009, 0, .0465]))
    assert angles == pytest.approx(np.degrees([-.2, .1]))
    assert meta["camera_xyz_base"] == [.057, 0, .21]
    assert meta["laser_from_camera_xyz"] == [-.009, 0, .0465]


def test_stale_feedback_and_inconsistent_tf_fail_instead_of_using_command_goal():
    _, p, cal = configured()
    cal.joint_state(joints(p, 1., .2, .1))
    with pytest.raises(ValueError, match="stale"):
        cal.measured_pose(1.5, tf([.057, 0, .2035], -.2, .1), tf([-.009, 0, .0465]))
    with pytest.raises(ValueError, match="disagree"):
        cal.measured_pose(1., tf([.057, 0, .2035], .2, .1), tf([-.009, 0, .0465]))
    with pytest.raises(ValueError, match="boresight"):
        cal.measured_pose(1., tf([.057, 0, .2035], -.2, .1), tf([0., 0, 0.]))


def test_joint_reset_discards_old_time_epoch():
    _, p, cal = configured()
    cal.joint_state(joints(p, 5.))
    cal.joint_state(joints(p, 1.))
    assert len(cal.joints) == 1
    with pytest.raises(ValueError, match="stale"):
        cal.joints_at(5.)


def test_robot_frame_and_live_angles_use_measurement_without_changing_goal():
    cfg, _, _ = configured()
    robot = WingbotRobot(cfg, Transport(), clock=lambda: 10.)
    robot.gimbal.command(0., 40., 15.)
    robot.on_gimbal_feedback(-11., 3.)
    assert robot.get_pan_tilt() == (-11., 3.)
    assert robot.gimbal.target == (40., 15.)
    image = np.zeros((480, 640, 3), np.uint8)
    robot.on_image(image, 1.)
    assert robot.get_frame() is None
    robot.on_image(image, 1., pan_tilt=(-10., 2.), meta={"test": "actual"})
    assert (robot.get_frame().pan, robot.get_frame().tilt) == (-10., 2.)
    assert robot.get_frame().meta == {"test": "actual"}


def test_legacy_hal_still_uses_command_estimate_without_opt_in():
    robot = WingbotRobot(Config(), Transport(), clock=lambda: 10.)
    robot.on_image(np.zeros((480, 640, 3), np.uint8), 1.)
    assert robot.get_frame() is not None
    assert robot.get_frame().meta == {}


def test_other_scope_cannot_reuse_simulator_distortion_exception():
    _, p, _ = configured()
    other = copy.deepcopy(p)
    other["scope"] = "physical_camera"
    with pytest.raises(ValueError):
        WingbotCameraCalibration(other)


def test_transport_retries_exact_image_stamp_without_latest_tf_substitution(monkeypatch):
    from maicon26.hal.ros_mil import RosMilTransport
    from maicon26.hal import ros_mil
    cfg, p, cal = configured()
    cal.camera_info(info(p))
    cal.joint_state(joints(p, 1., .2, .1))
    transport = RosMilTransport.__new__(RosMilTransport)
    robot = WingbotRobot(cfg, Transport(), clock=lambda: 10.)
    transport.robot, transport.cfg, transport._camera_calibration = robot, cfg, cal
    msg = NS(header=NS(stamp=NS(sec=1, nanosec=0), frame_id=p["optical_frame"]))
    image = np.zeros((480, 640, 3), np.uint8)
    transport._pending_camera = deque([(msg, image, 10.)], maxlen=4)
    calls, available = [], [False]
    def lookup(target, source, stamp):
        calls.append((target, source, stamp))
        if not available[0]:
            raise RuntimeError("extrapolation into future")
        return NS(transform=tf([.057, 0., .2035], -.2, .1) if target == "base_footprint" else tf([-.009, 0., .0465]))
    transport._tf = NS(lookup_transform=lookup)
    monkeypatch.setitem(sys.modules, "rclpy.time", NS(Time=NS(from_msg=lambda stamp: (stamp.sec, stamp.nanosec))))
    monkeypatch.setattr(ros_mil.time, "monotonic", lambda: 10.1)
    transport._retry_camera()
    assert len(transport._pending_camera) == 1 and robot.get_frame() is None
    available[0] = True
    transport._retry_camera()
    assert not transport._pending_camera
    assert robot.get_frame().pan == pytest.approx(-math.degrees(.2))
    assert robot.get_frame().meta["sensor_stamp"] == 1.
    assert all(stamp == (1, 0) for _, _, stamp in calls)
    assert "camera_calibration_error" not in robot.stats


def test_expired_tf_wait_drops_image_and_does_not_retain_previous_frame(monkeypatch):
    from maicon26.hal.ros_mil import RosMilTransport
    from maicon26.hal import ros_mil
    cfg, p, cal = configured()
    transport = RosMilTransport.__new__(RosMilTransport)
    robot = WingbotRobot(cfg, Transport(), clock=lambda: 10.)
    robot.on_image(np.zeros((480, 640, 3), np.uint8), 1., pan_tilt=(0, 0))
    transport.robot, transport.cfg, transport._camera_calibration = robot, cfg, cal
    msg = NS(header=NS(stamp=NS(sec=1, nanosec=0), frame_id=p["optical_frame"]))
    transport._pending_camera = deque([(msg, np.zeros((480, 640, 3), np.uint8), 10.)], maxlen=4)
    def missing(*args):
        raise RuntimeError("missing image-time TF")
    transport._tf = NS(lookup_transform=missing)
    monkeypatch.setitem(sys.modules, "rclpy.time", NS(Time=NS(from_msg=lambda stamp: (stamp.sec, stamp.nanosec))))
    monkeypatch.setattr(ros_mil.time, "monotonic", lambda: 11.)
    transport._retry_camera()
    assert not transport._pending_camera and robot.get_frame() is None
    assert robot.stats["camera_tf_dropped"] == 1
