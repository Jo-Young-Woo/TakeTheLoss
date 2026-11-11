"""OAK depth must agree with its CameraInfo and calibrated optical TF (no ROS needed)."""
import math
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.hal.depth_io import camera_info_K, decode_depth_image, validate_depth_transform
from maicon26.hal.ros_mil import RosMilTransport
from maicon26.hal.wingbot import Transport, WingbotRobot


OPTICAL = "measured_depth_optical_frame"


def _header(frame=OPTICAL, sec=10):
    return NS(frame_id=frame, stamp=NS(sec=sec, nanosec=0))


def _image(width=3, height=2, frame=OPTICAL):
    return NS(width=width, height=height, header=_header(frame), encoding="16UC1", is_bigendian=0,
              step=width * 2, data=np.full((height, width), 1000, dtype="<u2").tobytes())


def _info(width=3, height=2, frame=OPTICAL):
    return NS(width=width, height=height, header=_header(frame), p=[],
              k=np.array([120., 0, 1, 0, 121, .5, 0, 0, 1]))


class _Transport(Transport):
    def __init__(self):
        self.cmds = []

    def publish_cmd(self, v, w):
        self.cmds.append((v, w))


def _robot(strict=True):
    cfg = Config()
    cfg.depth.require_calibration = strict
    cfg.depth.calibration_verified = True
    cfg.depth.optical_frame = OPTICAL
    clock, tr = [0.0], _Transport()
    robot = WingbotRobot(cfg, tr, clock=lambda: clock[0])
    clock[0] = 10.0
    robot.on_match("mission", "standby")
    robot._received.update({k: 10.0 for k in ("scan", "odom", "image", "imu", "state")})
    return robot, tr, clock


def _transform(cfg, base="base_footprint", quat=(-.5, .5, -.5, .5)):
    x, y, z = cfg.rig.depth_xyz
    return NS(translation=NS(x=x, y=y, z=z + (cfg.rig.base_z if base == "base_footprint" else 0)),
              rotation=NS(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))


def test_strict_depth_needs_info_and_verified_tf_before_readiness():
    r, _, _ = _robot()
    assert r.readiness() == (False, ["depth"])
    r.on_depth(_image(), tf_validated=True)
    assert r.get_depth() is None and "CameraInfo" in r.stats["depth_error"]
    r.on_depth_info(_info())
    r.on_depth(_image())
    assert r.get_depth() is None and "TF" in r.stats["depth_error"]
    r.on_depth(_image(), tf_validated=True)
    assert r.readiness() == (True, [])
    assert r.get_depth().K[4] == 121 and np.all(r.get_depth().depth == 1)


@pytest.mark.parametrize("width,height,frame", [(4, 2, OPTICAL), (3, 3, OPTICAL), (3, 2, "rgb_optical")])
def test_strict_depth_rejects_dimensions_or_frame_mismatch(width, height, frame):
    r, _, _ = _robot()
    r.on_depth_info(_info())
    r.on_depth(_image(), tf_validated=True)
    r.on_depth(_image(width, height, frame), tf_validated=True)
    assert r.get_depth() is None and "depth" not in r._received
    assert "do not match" in r.stats["depth_error"]


@pytest.mark.parametrize("bad", ["fy", "nan", "width", "frame", "p_nan"])
def test_invalid_info_clears_old_intrinsics_and_depth(bad):
    r, _, _ = _robot()
    r.on_depth_info(_info())
    r.on_depth(_image(), tf_validated=True)
    info = _info()
    if bad == "fy":
        info.k[4] = 0
    elif bad == "nan":
        info.k[2] = float("nan")
    elif bad == "width":
        info.width = 0
    elif bad == "frame":
        info.header.frame_id = "rgb_optical"
    else:
        info.p = [float("nan")] * 12
    r.on_depth_info(info)
    assert r._depth_K is None and r.get_depth() is None
    r.on_depth(_image(), tf_validated=True)
    assert r.get_depth() is None and not r.readiness()[0]


def test_strict_unverified_calibration_or_zero_stamp_is_rejected():
    r, _, _ = _robot()
    r.on_depth_info(_info())
    r.cfg.depth.calibration_verified = False
    r.on_depth(_image(), tf_validated=True)
    assert "not verified" in r.stats["depth_error"]
    r.cfg.depth.calibration_verified = True
    image = _image()
    image.header.stamp.sec = 0
    r.on_depth(image, tf_validated=True)
    assert r.get_depth() is None and "timestamp" in r.stats["depth_error"]


def test_camera_info_repeat_preserves_frame_but_changed_calibration_invalidates_it():
    r, _, _ = _robot()
    r.on_depth_info(_info())
    r.on_depth(_image(), tf_validated=True)
    previous = r.get_depth()
    r.on_depth_info(_info())
    assert r.get_depth() is previous
    info = _info()
    info.k[0] += 3
    r.on_depth_info(info)
    assert r.get_depth() is None


def test_permissive_depth_still_allows_fov_fallback_but_drops_invalid_old_K():
    r, _, _ = _robot(strict=False)
    r.on_depth_info(_info())
    invalid = _info()
    invalid.k[4] = 0
    r.on_depth_info(invalid)
    r.on_depth(_image())
    assert r.get_depth() is not None and r.get_depth().K is None
    assert r.readiness() == (True, [])


def test_strict_depth_loss_stops_final_velocity_and_invalid_info_stops_immediately():
    r, tr, clock = _robot()
    r.on_match("practice", "idle")
    r._publish_cmd(.2, .3)
    assert tr.cmds[-1] == (0, 0)
    r.on_depth_info(_info())
    r.on_depth(_image(), tf_validated=True)
    r._publish_cmd(.2, .3)
    assert tr.cmds[-1] == (.2, .3)
    clock[0] += 1.01
    r._publish_cmd(.2, .3)
    assert tr.cmds[-1] == (0, 0) and "depth" in r.readiness()[1]
    image = _image()
    image.header.stamp.sec = 11
    r.on_depth(image, tf_validated=True)
    r._publish_cmd(.2, .3)
    assert tr.cmds[-1] == (.2, .3)
    r.on_depth_info(_info(frame="wrong_frame"))
    assert tr.cmds[-1] == (0, 0) and r.get_depth() is None


def test_optional_or_disabled_depth_does_not_lock_drive():
    for strict, enabled in [(False, True), (True, False)]:
        r, tr, _ = _robot(strict)
        r.cfg.depth.enable = enabled
        r.on_match("practice", "idle")
        r._publish_cmd(.2, .3)
        assert tr.cmds[-1] == (.2, .3) and r.readiness()[0]


def _blind_frame(value, sec=11):
    image = _image()
    image.encoding, image.step = "32FC1", image.width * 4
    image.data = np.full((image.height, image.width), value, dtype="<f4").tobytes()
    image.header.stamp.sec = sec
    return image


@pytest.mark.parametrize("value", [0., float("nan"), float("inf"), .1])
def test_strict_depth_without_any_usable_pixel_revokes_readiness_and_blocks_only_forward(value):
    """영상은 오는데 쓸 수 있는 픽셀이 없다(벽에 코를 박음·렌즈 가림): 전진만 막고 후진·회전은 허용해 갇히지 않게 한다."""
    r, tr, clock = _robot()
    r.on_match("practice", "idle")
    r.on_depth_info(_info())
    r.on_depth(_image(), tf_validated=True)
    r.set_velocity(.2, .3)                                           # Agent 가 준 명령(pump 가 이것을 반복 발행한다)
    r._publish_cmd(.2, .3)
    assert tr.cmds[-1] == (.2, .3)
    clock[0] = 11
    r.on_depth(_blind_frame(value), tf_validated=True)
    assert r.get_depth() is None and "depth" not in r._received
    assert "depth" in r.readiness()[1] and "no valid pixels" in r.stats["depth_error"]
    assert tr.cmds[-1] == (0, .3)                                    # 전진 중이었다면 바로 끊되 회전은 그대로
    r._publish_cmd(-.5, 1.0)
    assert tr.cmds[-1] == (-r.cfg.depth.blind_reverse_mps, 1.0)      # 후진은 낮은 속도로 허용
    r._publish_cmd(.3, 0.0)
    assert tr.cmds[-1] == (0, 0.0)


def test_blind_but_alive_expires_into_full_stop():
    r, tr, clock = _robot()
    r.on_match("practice", "idle")
    r.on_depth_info(_info())
    clock[0] = 11
    r.on_depth(_blind_frame(0.), tf_validated=True)
    clock[0] = 13                                                    # 새 영상이 없다 = 카메라 사망 → 전면 정지
    r._publish_cmd(-.1, .5)
    assert tr.cmds[-1] == (0, 0)


def test_far_only_depth_is_valid_free_space_not_blind():
    """경기장에서 먼 벽만 보이면(전부 max_range 밖) '앞이 비었다'는 정보다. 로봇이 서면 안 된다."""
    r, tr, clock = _robot()
    r.on_match("practice", "idle")
    r.on_depth_info(_info())
    clock[0] = 11
    r.on_depth(_blind_frame(6.0), tf_validated=True)
    assert r.get_depth() is not None and r.readiness()[0]
    r._publish_cmd(.3, 0.0)
    assert tr.cmds[-1] == (.3, 0.0)


def test_strict_depth_accepts_one_usable_pixel_without_inventing_coverage_threshold():
    r, _, _ = _robot()
    r.on_depth_info(_info())
    image = _image()
    values = np.zeros((image.height, image.width), dtype="<u2")
    values[0, 0] = 1000
    image.data = values.tobytes()
    r.on_depth(image, tf_validated=True)
    assert r.readiness()[0] and r.get_depth() is not None


def test_permissive_all_invalid_depth_is_still_optional():
    r, tr, _ = _robot(strict=False)
    r.on_match("practice", "idle")
    image = _image()
    image.data = bytes(len(image.data))
    r.on_depth(image)
    r._publish_cmd(.2, .3)
    assert r.get_depth() is not None and not np.any(r.get_depth().depth)
    assert r.readiness()[0] and tr.cmds[-1] == (.2, .3)


def test_replayed_depth_cannot_refresh_readiness_or_keep_drive_alive():
    r, tr, clock = _robot()
    r.on_match("practice", "idle")
    r.on_depth_info(_info())
    r.on_depth(_image(), tf_validated=True)
    clock[0] += 1.01
    r.on_depth(_image(), tf_validated=True)
    assert r._received["depth"] == 10 and not r._depth_fresh(r.now())
    r._publish_cmd(.2, .3)
    assert tr.cmds[-1] == (0, 0) and r.stats["depth_repeated"] == 1
    image = _image()
    image.header.stamp.sec = 9
    r.on_depth(image, tf_validated=True)
    assert r.get_depth() is None and "backwards" in r.stats["depth_error"]


@pytest.mark.parametrize("bad", ["dimensions", "step", "truncated", "scale"])
def test_invalid_depth_layout_and_units_raise_clear_errors(bad):
    msg, scale = _image(), .001
    if bad == "dimensions":
        msg.height = 0
    elif bad == "step":
        msg.step = 1
    elif bad == "truncated":
        msg.data = msg.data[:-1]
    else:
        scale = float("nan")
    with pytest.raises(ValueError):
        decode_depth_image(msg, scale)


@pytest.mark.parametrize("base", ["base_footprint", "base_link"])
def test_tf_includes_optical_rotation_and_correct_base_height(base):
    cfg = Config()
    assert validate_depth_transform(_transform(cfg, base), cfg.rig, base, .02, 2) == (0, 0)
    wrong = _transform(cfg, base, quat=(0, 0, 0, 1))
    with pytest.raises(ValueError, match="differs"):
        validate_depth_transform(wrong, cfg.rig, base, .02, 2)
    wrong = _transform(cfg, "base_link" if base == "base_footprint" else "base_footprint")
    with pytest.raises(ValueError, match="differs"):
        validate_depth_transform(wrong, cfg.rig, base, .02, 2)


def test_tf_checks_calibrated_rpy_and_declared_tolerances():
    cfg = Config()
    cfg.rig.depth_rpy = (0, 0, math.radians(3))
    with pytest.raises(ValueError, match="3.000 deg"):
        validate_depth_transform(_transform(cfg), cfg.rig, "base_footprint", .02, 2)
    distance, angle = validate_depth_transform(_transform(cfg), cfg.rig, "base_footprint", .02, 4)
    assert distance == 0 and angle == pytest.approx(3)
    tf = _transform(cfg)
    tf.translation.x += .03
    with pytest.raises(ValueError, match="0.0300 m"):
        validate_depth_transform(tf, cfg.rig, "base_footprint", .02, 4)


@pytest.mark.parametrize("quat", [(0, 0, 0, 0), (0, 0, 0, 2), (0, 0, 0, float("nan"))])
def test_tf_rejects_invalid_quaternion(quat):
    cfg = Config()
    with pytest.raises(ValueError, match="quaternion"):
        validate_depth_transform(_transform(cfg, quat=quat), cfg.rig, "base_footprint", .02, 2)


def test_tf_rejects_nonfinite_translation_unknown_base_and_bad_tolerance():
    cfg = Config()
    tf = _transform(cfg)
    tf.translation.z = float("inf")
    with pytest.raises(ValueError, match="non-finite"):
        validate_depth_transform(tf, cfg.rig, "base_footprint", .02, 2)
    with pytest.raises(ValueError, match="only base"):
        validate_depth_transform(_transform(cfg), cfg.rig, "chassis", .02, 2)
    with pytest.raises(ValueError, match="tolerances"):
        validate_depth_transform(_transform(cfg), cfg.rig, "base_link", float("nan"), 2)


def test_ros_depth_callback_uses_sensor_time_tf_and_revokes_old_frame(monkeypatch):
    r, tr, _ = _robot()
    r.on_depth_info(_info())
    r.on_match("practice", "idle")
    transport = object.__new__(RosMilTransport)
    transport.robot, transport.cfg = r, r.cfg
    calls, tf = [], _transform(r.cfg)

    def lookup(base, frame, stamp):
        calls.append((base, frame, stamp))
        return NS(transform=tf)

    monkeypatch.setitem(sys.modules, "rclpy.time", NS(Time=NS(from_msg=lambda stamp: (stamp.sec, stamp.nanosec))))
    transport._tf = NS(lookup_transform=lookup)
    r.cfg.nav2.base_frame = "base_footprint"
    transport._on_depth(_image())
    assert calls == [("base_footprint", OPTICAL, (10, 0))] and r.readiness()[0]
    r._publish_cmd(.2, 0)
    tf.translation.x += .2
    transport._on_depth(_image())
    assert r.get_depth() is None and "differs" in r.stats["depth_tf_error"]
    assert tr.cmds[-1] == (0, 0)
    transport._tf = NS(lookup_transform=lambda *args: (_ for _ in ()).throw(RuntimeError("TF unavailable")))
    transport._on_depth(_image())
    assert "TF unavailable" in r.stats["depth_tf_error"]


def test_camera_info_numpy_arrays_supported():
    info = _info()
    info.p = np.zeros(12)
    K, w, h = camera_info_K(info)
    assert K[0] == 120 and (w, h) == (3, 2)
