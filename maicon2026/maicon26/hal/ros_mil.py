"""rclpy 전송 계층 — WINGBOT-UGV ROS2 API(2차 사전교육 PDF p9~18). 로봇(Jetson Docker, ROS2 Humble)에서만 import 된다. **미검증.**

표준 메시지(Twist, Odometry, Image, Imu, LaserScan)는 직접 import 한다. 자료에 패키지 이름이 없는 커스텀 타입
(/mil/state, /mil/match 의 메시지, /mil/oled 의 OledLine, /mil/gimbal 의 Gimbal, /mil/fire 의 Fire)은 **실행 중인 토픽/서비스에서 타입 이름을 알아내** import 한다
(ros2 topic list -t / ros2 service list -t 와 같은 정보). 안 되면 config.hal.type_override 에 'pkg/msg/Name' 을 적는다.

QoS (PDF p11·15): /mil/state·/mil/match = depth 1 + TRANSIENT_LOCAL(마지막 상태를 바로 받음), /odom·영상·IMU = qos_profile_sensor_data.
(예전 템플릿은 /odom 을 기본 RELIABLE 로 구독했다 — 센서 QoS(BEST_EFFORT) 발행자와는 연결이 안 된다.)
"""
from __future__ import annotations

import os
import threading
import math
import time
from collections import deque
from typing import Any, Callable, Dict, Optional

import numpy as np

from .wingbot import Transport, WingbotRobot
from .depth_io import camera_info_K, validate_depth_transform
from ..validate import check_image_message


def lidar_mount_from_transform(tf):
    """geometry_msgs/TransformStamped(base ← LiDAR 프레임) → (x, y, yaw[rad]). LiDAR 평면이 수평이 아니면 ValueError.
    Agent 의 _on_scan 과 ros2/scripts/scan_self_mask.py 가 같은 장착 변환을 쓰도록 분리했다."""
    from .mil_api import quat_to_rpy_deg
    p, q = tf.transform.translation, tf.transform.rotation
    roll, pitch, yaw_deg = quat_to_rpy_deg(q.x, q.y, q.z, q.w)
    if abs(roll) > 1.0 or abs(pitch) > 1.0:
        raise ValueError("LiDAR plane is not horizontal in base frame")
    return p.x, p.y, math.radians(yaw_deg)


def scan_rays_in_base(angle_min, angle_inc, ranges, range_min, range_max, x, y, yaw):
    """LaserScan 광선마다 끝점을 차체(base) 원점 기준 극좌표로 옮긴다 → (ok, theta, rr).

    ok 는 쓸 수 있는 광선(유한, range_min..range_max, 0 초과), theta 는 (-π, π] 차체 각도(전방 0, 왼쪽 +), rr 는 차체 원점에서의 거리.
    theta·rr 는 ok 인 광선만 담는다(길이 = ok.sum()). scan_in_base 와 ros2/scripts/scan_self_mask.py 가 같은 변환을 쓰도록 분리했다.
    """
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(len(r)) * angle_inc + yaw
    ok = np.isfinite(r) & (r >= max(range_min, 0.0)) & (r > 0) & (r <= range_max)
    px, py = x + r[ok] * np.cos(a[ok]), y + r[ok] * np.sin(a[ok])
    return ok, np.arctan2(py, px), np.hypot(px, py)


def scan_in_base(angle_min, angle_inc, ranges, range_min, range_max, x, y, yaw):
    """Transform measured endpoints and conservatively rebin around base origin.

    Unobserved/invalid rays remain NaN; this never invents a clear max-range ray.
    Several endpoints in a bin keep the closest. Raw ROS /scan is unchanged.
    """
    if not math.isfinite(angle_inc) or angle_inc == 0:
        raise ValueError("invalid scan angular increment")
    ratio = 2 * math.pi / abs(angle_inc)
    # float32 증분 때문에 720발 스캔이 720.00001 로 나온다 → ceil 이면 721칸이 되어 칸이 1개씩 밀리고 NaN 틈이 생긴다.
    n = max(1, int(round(ratio)) if abs(ratio - round(ratio)) < 0.01 else int(math.ceil(ratio)))
    inc = 2 * math.pi / n
    _, th, rr = scan_rays_in_base(angle_min, angle_inc, ranges, range_min, range_max, x, y, yaw)
    ix = np.rint((th + math.pi) / inc).astype(int) % n
    out = np.full(n, np.inf)
    np.minimum.at(out, ix, rr)
    out[~np.isfinite(out)] = np.nan
    return -math.pi, inc, out, range_max + math.hypot(x, y)


def image_to_bgr(m) -> np.ndarray:
    """sensor_msgs/Image → BGR (H,W,3) uint8. cv_bridge 없이 numpy 로 변환한다(p17: 640×480 BGR)."""
    h, w, enc = int(m.height), int(m.width), str(m.encoding).lower()
    buf = np.frombuffer(bytes(m.data) if not isinstance(m.data, (bytes, bytearray, np.ndarray)) else m.data, dtype=np.uint8)
    step = int(getattr(m, "step", 0)) or (len(buf) // max(h, 1))
    if enc in ("bgr8", "rgb8"):
        a = buf[: h * step].reshape(h, step)[:, : w * 3].reshape(h, w, 3)
        return a if enc == "bgr8" else a[:, :, ::-1].copy()
    if enc in ("bgra8", "rgba8"):
        a = buf[: h * step].reshape(h, step)[:, : w * 4].reshape(h, w, 4)[:, :, :3]
        return a.copy() if enc == "bgra8" else a[:, :, ::-1].copy()
    if enc in ("mono8", "8uc1"):
        g = buf[: h * step].reshape(h, step)[:, :w]
        return np.repeat(g[:, :, None], 3, axis=2)
    raise ValueError(f"지원하지 않는 영상 인코딩: {m.encoding}")


_ENC_CHANNELS = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4, "mono8": 1, "8uc1": 1}


def image_message_problem(m, expect_w: Optional[int] = None, expect_h: Optional[int] = None) -> Optional[str]:
    """V4: sensor_msgs/Image 의 모양 검사(변환하기 전에). 정상이면 None, 아니면 이유. 모르는 인코딩은 image_to_bgr 가 거절하므로 여기서는 통과시킨다.
    예외를 던지지 않는다."""
    try:
        channels = _ENC_CHANNELS.get(str(m.encoding).lower())
        if channels is None:
            return None
        h = int(m.height)
        n = len(m.data)
        step = int(getattr(m, "step", 0)) or (n // max(h, 1))
        return check_image_message(h, m.width, step, channels, n, expect_w, expect_h)
    except Exception:                                    # noqa: BLE001 — 이상한 메시지는 그냥 '버린다'
        return "bad_message"


def _text(res: Any) -> Optional[str]:
    """서비스 응답에서 ok/locked/fired… 문자열을 꺼낸다(필드 이름이 자료에 없다: message 가 유력)."""
    for name in ("message", "result", "status", "msg", "state"):
        v = getattr(res, name, None)
        if isinstance(v, str) and v:
            return v.strip().lower()
    return None


class RosMilTransport(Transport):
    def __init__(self, robot: WingbotRobot) -> None:
        from geometry_msgs.msg import Twist  # type: ignore
        from nav_msgs.msg import Odometry  # type: ignore
        from sensor_msgs.msg import Image, Imu, LaserScan  # type: ignore
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data  # type: ignore
        from ..nav.ros_runtime import add_node, ensure_ros

        self.robot, self.cfg = robot, robot.cfg
        h = self.cfg.hal
        rclpy, _ = ensure_ros()                         # rclpy 초기화·실행기는 SLAM/Nav2 클라이언트와 공유 (init 이중 호출 방지)
        namespace = getattr(h, "ros_namespace", "").rstrip("/")
        if namespace:
            self.node = rclpy.create_node("maicon26_agent_"+namespace.rsplit("/",1)[-1],
                namespace=namespace, cli_args=["--ros-args","-r","/tf:="+namespace+"/tf",
                                               "-r","/tf_static:="+namespace+"/tf_static"])
        else:
            self.node = rclpy.create_node("maicon26_agent")
        from tf2_ros import Buffer, TransformListener
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self.node, spin_thread=False)
        self._Twist = Twist
        self._lock = threading.Lock()
        self._types: Dict[str, Any] = {}
        self._ready: Dict[str, bool] = {k: False for k in ("state", "match", "oled", "gimbal", "fire")}
        self._pub_oled = self._cli_gimbal = self._cli_fire = None
        self._closed = False
        self._scan_receipt_seq = 0
        self._qos_latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)
        self._qos_sensor = qos_profile_sensor_data
        self._camera_calibration = None
        self.imu_odometry = None
        if getattr(h,"imu_heading_fusion",False):
            from .imu_odometry import RelativeImuOdometry
            self.imu_odometry = RelativeImuOdometry(h.imu_heading_max_age_s)
        self._pending_camera = deque(maxlen=4)
        if self.cfg.camera.calibration_profile:
            from .wingbot_calibration import WingbotCameraCalibration
            from sensor_msgs.msg import CameraInfo, JointState
            self._camera_calibration = WingbotCameraCalibration(self.cfg.camera.calibration_profile)
            profile = self.cfg.camera.calibration_profile
            self.node.create_subscription(CameraInfo, profile["info_topic"], self._safe(self._on_camera_info), self._qos_sensor)
            self.node.create_subscription(JointState, profile["joint_topic"], self._safe(self._on_camera_joints), self._qos_sensor)
            self.node.create_timer(.02, self._safe(self._retry_camera))

        # 구동: /cmd_vel 단독 발행자. Nav2 는 /cmd_vel_nav2 로 리맵되어 있다(여러 노드가 동시에 보내면 명령이 충돌한다, p15)
        self._pub_cmd = self.node.create_publisher(Twist, h.cmd_topic, 10)
        self.node.create_subscription(Odometry, h.odom_topic, self._safe(self._on_odom), self._qos_sensor)
        self.node.create_subscription(LaserScan, h.scan_topic, self._safe(self._on_scan), self._qos_sensor)
        self.node.create_subscription(Image, h.image_topic, self._safe(self._on_image), self._qos_sensor)
        self.node.create_subscription(Imu, h.imu_topic, self._safe(self._on_imu), self._qos_sensor)
        dc = self.cfg.depth
        if dc.enable:                                    # 깊이 카메라(OAK): /image_raw 가 아니다(p17). 토픽 이름은 config.depth.*
            from sensor_msgs.msg import CameraInfo  # type: ignore
            self.node.create_subscription(Image, dc.topic, self._safe(self._on_depth), self._qos_sensor)
            self.node.create_subscription(CameraInfo, dc.info_topic, self._safe(robot.on_depth_info), self._qos_sensor)
        self._oak_last_decode = -1e9
        self._oak_pwr = {"cli": {}, "pend": None}        # oak.power_gating: Trigger 클라이언트(on/off)와 진행 중인 호출 (켜는지, future, 시각)
        self._oak_topic_timer = None
        od = self.cfg.oak_det
        if od.enable:                                    # 6장: OAK 컬러 보조 탐지 — 구독만 한다. 디스크에는 쓰지 않는다
            self._subscribe_oak()
        if od.enable or os.environ.get("MAICON_OAK", "").strip() == "1":
            self._oak_topic_timer = self.node.create_timer(max(float(self.cfg.oak.check_topics_after_s), 0.5), self._check_oak_topics)
        self._discover()                                 # 커스텀 타입: 이미 떠 있으면 바로, 아니면 1 s 마다 재시도
        self.node.create_timer(1.0, self._discover)
        self.node.create_timer(1.0 / max(h.cmd_hz, 1.0), self._tick)
        add_node(self.node)

    # ------------------------------------------------------------------ OAK 컬러 (6장)
    def _subscribe_oak(self) -> None:
        from sensor_msgs.msg import CameraInfo, CompressedImage, Image  # type: ignore
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # type: ignore
        oc = self.cfg.oak
        self._oak_compressed = str(oc.rgb_topic).rstrip("/").endswith("/compressed")
        latest_only = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)   # 센서 데이터 QoS, 큐 깊이 1
        self.node.create_subscription(CompressedImage if self._oak_compressed else Image, oc.rgb_topic,
                                      self._safe(self._on_oak_rgb), latest_only)
        self.node.create_subscription(CameraInfo, oc.rgb_info_topic, self._safe(self._on_oak_info), self._qos_sensor)
        if self.cfg.oak.power_gating:                    # 평소엔 꺼 두고 식별 단계에서만 켠다(드라이버의 start/stop 서비스, std_srvs/Trigger)
            from std_srvs.srv import Trigger  # type: ignore
            self._oak_pwr["Trigger"] = Trigger
            self._oak_pwr["cli"] = {True: self.node.create_client(Trigger, oc.start_srv),
                                    False: self.node.create_client(Trigger, oc.stop_srv)}

    def _on_oak_rgb(self, m) -> None:
        """최신 프레임만: 디코딩은 oak_det.max_hz 의 2배 이하로만 한다(디코딩도 CPU 를 쓴다). 디스크 쓰기 없음."""
        now = self.robot.now()
        if now - self._oak_last_decode < 1.0 / (2.0 * max(float(self.cfg.oak_det.max_hz), 0.1)):
            return
        self._oak_last_decode = now
        from .oak_io import oak_msg_to_bgr
        if not self._oak_compressed:                     # V8: 원본 영상은 변환 전에 모양을 본다(압축 영상은 디코더가 거른다). 크기는 on_oak_frame 이 CameraInfo 와 비교한다
            why = image_message_problem(m)
            if why is not None:
                self.robot.vlog.bad("oak_rgb", why)
                return
        try:
            img = oak_msg_to_bgr(m, self._oak_compressed)
        except ValueError:
            self.robot.stats["oak_decode_error"] = self.robot.stats.get("oak_decode_error", 0) + 1
            self.robot.vlog.bad("oak_rgb", "decode_error")
            return
        self.robot.on_oak_frame(img, m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)

    def call_oak_power(self, on: bool) -> str:
        """OAK 켜기/끄기(Trigger) 비동기 호출. 막지 않는다: 첫 호출은 요청을 보내고 'pending', 같은 인자로 다시 부르면 응답을 본다.
        서비스가 안 보이면 'none'(OAK 를 켜 둔 채로 쓴다). 응답이 5 s 안에 없으면 'fail' 로 두고 다음 호출이 새로 보낸다."""
        st = self._oak_pwr
        cli = st["cli"].get(bool(on)) if st else None
        if cli is None:
            return "none"
        pend = st["pend"]
        now = self.robot.now()
        if pend is not None and pend[0] == bool(on):
            fut = pend[1]
            if not fut.done():
                if now - pend[2] > 5.0:
                    st["pend"] = None
                    return "fail"
                return "pending"
            st["pend"] = None
            try:
                res = fut.result()
                return "ok" if bool(getattr(res, "success", False)) else "fail"
            except Exception:                            # noqa: BLE001
                return "fail"
        if not cli.service_is_ready():
            return "none"
        st["pend"] = (bool(on), cli.call_async(st["Trigger"].Request()), now)
        return "pending"

    def _on_oak_info(self, m) -> None:
        try:
            K, w, h = camera_info_K(m)
        except (ValueError, TypeError, OverflowError) as e:   # V8: 쓸 수 없는 CameraInfo → 이전 값도 지우고 OAK 관측을 끈다(예외는 밖으로 나가지 않는다)
            self.robot.drop_oak_info("camera_info_unusable", str(e)[:60])
            return
        self.robot.on_oak_info(K, w, h)

    def _check_oak_topics(self) -> None:
        """oak.check_topics_after_s 뒤 한 번: 설정한 OAK 토픽이 ROS 그래프에 있는지 본다. 없으면 oak 이 든 토픽 목록을 경고로 낸다(막지 않는다)."""
        if self._oak_topic_timer is not None:
            self._oak_topic_timer.cancel()
        from ..oak_launch import check_topics, wanted_topics
        try:
            graph = self.node.get_topic_names_and_types()
            missing = check_topics(graph, wanted_topics(self.cfg, include_rgb=True),
                                   report=lambda msg: print(msg, flush=True))
            self.robot.stats["oak_topics_missing"] = list(missing)
        except Exception as e:                           # pragma: no cover — 그래프 조회 실패도 경고뿐
            self.robot.stats["oak_topic_check_error"] = repr(e)

    # ------------------------------------------------------------------ 도우미
    def _safe(self, fn: Callable) -> Callable:
        """콜백 예외가 실행기 스레드를 죽이지 않게 감싼다(실행기가 죽으면 모든 센서·명령이 멈춘다)."""
        def wrapped(*a, **k):
            try:
                return fn(*a, **k)
            except Exception as e:                       # pragma: no cover
                self.robot.stats["cb_errors"] = self.robot.stats.get("cb_errors", 0) + 1
                self.robot.stats["cb_last_error"] = f"{fn.__name__}: {e!r}"
                vlog = getattr(self.robot, "vlog", None)
                if vlog is not None:                     # 9장: 콜백 예외도 이유로 센다(같은 이유의 경고는 5 s 에 한 줄)
                    vlog.bad(getattr(fn, "__name__", "callback"), "exception", f"{type(e).__name__}: {e}"[:80])
        return wrapped

    def _tick(self) -> None:
        try:
            self.robot.pump()
        except Exception as e:                           # pragma: no cover
            self.robot.stats["pump_error"] = repr(e)

    def _type_of(self, key: str, name: str, services: bool = False):
        """key 의 타입 문자열(override 우선)을 클래스로. 아직 없으면 None."""
        ov = self.cfg.hal.type_override.get(key)
        tstr = ov
        if tstr is None:
            pairs = dict(self.node.get_service_names_and_types() if services else self.node.get_topic_names_and_types())
            tl = pairs.get(name)
            tstr = tl[0] if tl else None
        if not tstr:
            return None
        try:
            if services:
                from rosidl_runtime_py.utilities import get_service  # type: ignore
                return get_service(tstr)
            from rosidl_runtime_py.utilities import get_message  # type: ignore
            return get_message(tstr)
        except Exception:
            return None

    def _discover(self) -> None:
        if self._closed:
            return
        h = self.cfg.hal
        with self._lock:
            for key, name, cb in (("state", h.state_topic, self._on_state), ("match", h.match_topic, self._on_match)):
                if not self._ready[key]:
                    cls = self._type_of(key, name)
                    if cls is not None:
                        self.node.create_subscription(cls, name, self._safe(cb), self._qos_latched)
                        self._ready[key] = True
            if not self._ready["oled"]:
                cls = self._type_of("oled", h.oled_topic)
                if cls is not None:
                    self._pub_oled, self._oled_cls, self._ready["oled"] = self.node.create_publisher(cls, h.oled_topic, 10), cls, True
            if not self._ready["gimbal"]:
                cls = self._type_of("gimbal", h.gimbal_srv, services=True)
                if cls is not None:
                    self._cli_gimbal, self._gimbal_cls, self._ready["gimbal"] = self.node.create_client(cls, h.gimbal_srv), cls, True
            if not self._ready["fire"]:
                cls = self._type_of("fire", h.fire_srv, services=True)
                if cls is not None:
                    self._cli_fire, self._fire_cls, self._ready["fire"] = self.node.create_client(cls, h.fire_srv), cls, True
        self.robot.stats["mil_ready"] = dict(self._ready)

    # ------------------------------------------------------------------ 발행
    def readiness(self):
        missing = []
        calibration = getattr(self, "_camera_calibration", None)
        if calibration is not None:
            if calibration.info is None:
                missing.append("camera_info")
            if self.robot.stats.get("camera_calibration_error"):
                missing.append("camera_calibration")
            measured = self.robot._measured_gimbal
            if measured is None or self.robot.now()-measured[2] > calibration.profile["feedback_max_age_s"]:
                missing.append("gimbal_feedback")
        with self._lock:
            # PDF p15: only this HAL may publish the final actuator command.
            # Nav2 controller/behavior publishers on its intermediate topic do
            # not conflict; inspect only hal.cmd_topic, not cmd_vel_nav2.
            if self.node.count_publishers(self.cfg.hal.cmd_topic) > 1:
                missing.append("multiple_publishers:" + self.cfg.hal.cmd_topic)
            for key in ("gimbal", "fire"):
                cli = getattr(self, "_cli_" + key)
                if cli is None or not cli.service_is_ready():
                    missing.append("service:" + key)
            if self._pub_oled is None:
                missing.append("topic:oled")
        return not missing, missing

    def publish_cmd(self, v: float, w: float) -> None:
        t = self._Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self._pub_cmd.publish(t)

    def publish_oled(self, line: int, text: str) -> None:
        if self._pub_oled is None:
            return
        self._pub_oled.publish(self._oled_cls(line=int(line), text=str(text)))

    def call_gimbal(self, pan_unit, tilt_unit, speed, acc, cb) -> None:
        cli = self._cli_gimbal
        if cli is None or not cli.service_is_ready():
            cb(None)
            return
        req = self._gimbal_cls.Request()
        req.pan, req.tilt, req.speed, req.acc = float(pan_unit), float(tilt_unit), float(speed), float(acc)
        fut = cli.call_async(req)                        # 기다리지 않는다(제어 루프 step ≤ 0.1 s). 응답은 실행기 스레드에서 콜백
        fut.add_done_callback(lambda f: cb(_text(f.result()) if f.exception() is None and f.result() is not None else None))

    def call_fire(self, cb) -> None:
        cli = self._cli_fire
        if cli is None or not cli.service_is_ready():
            cb(None, None)
            return
        fut = cli.call_async(self._fire_cls.Request())

        def done(f) -> None:
            if f.exception() is not None or f.result() is None:
                cb(None, None)
                return
            r = f.result()
            left = getattr(r, "left_ms", None)
            try:
                left = float(left) if left is not None else None
            except (TypeError, ValueError, OverflowError):
                pass                                     # 이상한 값은 그대로 넘겨 FireModel(V1)이 버린다 — 응답 자체는 처리한다
            cb(_text(r), left)
        fut.add_done_callback(done)

    # ------------------------------------------------------------------ 수신
    def _on_depth(self, m) -> None:
        dc = self.cfg.depth
        if not dc.require_calibration:
            self.robot.on_depth(m)
            return
        try:
            if not dc.calibration_verified or not dc.optical_frame:
                raise ValueError("depth calibration is not verified")
            if m.header.frame_id != dc.optical_frame:
                raise ValueError("depth Image frame does not match depth.optical_frame")
            stamp = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
            if not math.isfinite(stamp) or stamp <= 0:
                raise ValueError("depth Image needs a nonzero sensor timestamp for TF")
            from rclpy.time import Time
            tf = self._tf.lookup_transform(self.cfg.nav2.base_frame, dc.optical_frame, Time.from_msg(m.header.stamp))
            distance, angle = validate_depth_transform(tf.transform, self.cfg.rig, self.cfg.nav2.base_frame,
                                                       dc.tf_tolerance_m, dc.tf_tolerance_deg)
        except Exception as e:
            # Missing/incorrect TF must not retain a previously accepted image.
            self.robot.stats["depth_tf_error"] = str(e)
            self.robot.reject_depth("depth TF: " + str(e))
            return
        self.robot.stats.pop("depth_tf_error", None)
        self.robot.stats["depth_tf_translation_m"] = distance
        self.robot.stats["depth_tf_rotation_deg"] = angle
        self.robot.on_depth(m, tf_validated=True)

    def _on_odom(self, m) -> None:
        import math
        q = m.pose.pose.orientation
        th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        base, child = self.cfg.nav2.base_frame, m.child_frame_id
        if not child:
            self.robot.stats["odom_tf_error"] = "empty odometry child_frame_id"
            return
        if child != base:
            from rclpy.time import Time
            from .mil_api import quat_to_rpy_deg
            try:
                tf = self._tf.lookup_transform(child, base, Time.from_msg(m.header.stamp))
                p, rot = tf.transform.translation, tf.transform.rotation
                _, _, yaw = quat_to_rpy_deg(rot.x, rot.y, rot.z, rot.w)
                x, y = x + math.cos(th) * p.x - math.sin(th) * p.y, y + math.sin(th) * p.x + math.cos(th) * p.y
                th += math.radians(yaw)
            except Exception as e:
                self.robot.stats["odom_tf_error"] = str(e)
                return
        self.robot.stats.pop("odom_tf_error", None)
        fusion = getattr(self,"imu_odometry",None)
        if fusion is not None:
            from ..types import Pose2D
            stamp = m.header.stamp.sec + 1e-9*m.header.stamp.nanosec
            fused = fusion.update_wheel(stamp if stamp>0 else None,Pose2D(x,y,th))
            self.robot.stats["imu_odometry"] = fusion.diagnostics()
            if fused is None:
                return
            x,y,th = fused.x,fused.y,fused.th
        self.robot.on_odom(x, y, th)

    def _on_scan(self, m) -> None:
        receipt_wall_s = time.perf_counter()
        self._scan_receipt_seq += 1
        receipt_seq = self._scan_receipt_seq
        self.robot.stats["scan_callbacks_received"] = receipt_seq
        hdr = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        base, source = self.cfg.nav2.base_frame, m.header.frame_id
        if not source:
            self.robot.stats["scan_tf_error"] = "empty scan frame_id"
            return
        x = y = yaw = 0.0
        if source != base:
            from rclpy.time import Time
            try:
                tf = self._tf.lookup_transform(base, source, Time.from_msg(m.header.stamp))
                x, y, yaw = lidar_mount_from_transform(tf)
            except Exception as e:
                self.robot.stats["scan_tf_error"] = str(e)
                return
        amin, inc, ranges, rmax = scan_in_base(m.angle_min, m.angle_increment, m.ranges,
                                              m.range_min, m.range_max, x, y, yaw)
        self.robot.stats.pop("scan_tf_error", None)
        self.robot.on_scan(amin, inc, ranges, hdr, rmax,
                           receipt_wall_s=receipt_wall_s, receipt_seq=receipt_seq)

    def _on_image(self, m) -> None:
        # 10 FPS 이하만 변환(변환 자체가 CPU 를 쓴다)
        if self.robot.now() - self.robot._last_img_t < 1.0 / max(self.cfg.hal.cam_max_fps, 1e-3) - 1e-3:
            self.robot.stats["frames_dropped"] += 1
            return
        # V4: 변환하기 전에 모양을 본다 — 크기가 설정(camera.width×height)과 다르면 K 가 조용히 틀리므로 버린다
        why = image_message_problem(m, self.cfg.camera.width, self.cfg.camera.height)
        if why is not None:
            self.robot.vlog.bad("image", why, f"{getattr(m, 'width', '?')}x{getattr(m, 'height', '?')}")
            return
        try:
            img = image_to_bgr(m)
        except ValueError:
            self.robot.stats["image_encoding_error"] = str(m.encoding)
            self.robot.vlog.bad("image", "encoding")
            return
        if getattr(self, "_camera_calibration", None) is not None:
            # Keep a bounded original frame queue while TF catches up. Never
            # replace its stamp with latest TF or block the shared ROS executor.
            if len(self._pending_camera) == self._pending_camera.maxlen:
                self.robot.stats["camera_queue_dropped"] = self.robot.stats.get("camera_queue_dropped", 0)+1
            self._pending_camera.append((m, img, time.monotonic()))
            self._retry_camera()
            return
        self.robot.on_image(img, m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)

    def _on_camera_info(self, msg) -> None:
        try:
            self._camera_calibration.camera_info(msg)
        except ValueError as error:
            self.robot.stats["camera_calibration_error"] = str(error)

    def _on_camera_joints(self, msg) -> None:
        try:
            angles = self._camera_calibration.joint_state(msg)
            self.robot.on_gimbal_feedback(*angles)
        except ValueError as error:
            self.robot.stats["camera_joint_error"] = str(error)
            return
        self.robot.stats.pop("camera_joint_error", None)

    def _retry_camera(self) -> None:
        if not self._pending_camera:
            return
        msg, image, received = self._pending_camera[0]
        calibration = self._camera_calibration
        p = calibration.profile
        try:
            from rclpy.time import Time
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            if stamp <= 0:
                raise ValueError("camera image requires nonzero sensor timestamp")
            tm = Time.from_msg(msg.header.stamp)
            # Height is relative to ground; metadata retains the complete actual
            # translation/rotation for downstream audits, including lever arm.
            camera = self._tf.lookup_transform("base_footprint", p["camera_frame"], tm)
            laser = self._tf.lookup_transform(p["camera_frame"], p["laser_frame"], tm)
            angles, measured = calibration.measured_pose(stamp, camera.transform, laser.transform)
            output = calibration.image(image, msg.header.frame_id)
        except Exception as error:
            self.robot.stats["camera_calibration_error"] = str(error)
            if time.monotonic()-received > p["tf_wait_wall_s"]:
                self._pending_camera.popleft()
                self.robot.stats["camera_tf_dropped"] = self.robot.stats.get("camera_tf_dropped", 0)+1
                self.robot._frame = None
            return
        self._pending_camera.popleft()
        self.robot.stats.pop("camera_calibration_error", None)
        metadata = dict(measured, calibration_scope="simulation_only", sensor_stamp=stamp,
                        optical_frame=p["optical_frame"], camera_info=dict(calibration.info),
                        distortion_status=p["distortion_status"])
        self.cfg.robot.cam_height = measured["camera_xyz_base"][2]
        self.robot.stats["camera_calibration"] = metadata
        self.robot.on_image(output, stamp, pan_tilt=angles, meta=metadata)

    def _on_imu(self, m) -> None:
        from .mil_api import accel_g, quat_to_rpy_deg
        q, a = m.orientation, m.linear_acceleration
        r, p, y = quat_to_rpy_deg(q.x, q.y, q.z, q.w)
        gz = getattr(m, "angular_velocity", None)
        gz_dps = math.degrees(gz.z) if gz is not None and math.isfinite(gz.z) else None
        # 충돌 방향용 가속도(차체 x 앞·y 왼쪽, g). IMU 가 차체에 yaw 로 돌려 달렸으면 imu_mount_yaw_deg 로 맞춘다(기본 0 = 같은 방향).
        my = math.radians(float(getattr(self.cfg.hal, "imu_mount_yaw_deg", 0.0)))
        gx, gy = a.x / 9.80665, a.y / 9.80665
        ax, ay = gx * math.cos(my) - gy * math.sin(my), gx * math.sin(my) + gy * math.cos(my)
        self.robot.on_imu(r, p, y, accel_g(a.x, a.y, a.z), ax, ay, gz_dps)
        fusion = getattr(self,"imu_odometry",None)
        if fusion is not None:
            stamp = m.header.stamp.sec + 1e-9*m.header.stamp.nanosec
            covariance = list(getattr(m,"orientation_covariance",()))
            norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
            valid = (math.isfinite(norm) and abs(norm-1.)<.02 and len(covariance)==9 and
                     all(math.isfinite(value) for value in covariance) and
                     covariance[0]>=0 and covariance[-1]>=0)
            reason = "" if valid else "invalid orientation quaternion or covariance"
            if valid:
                source,base = m.header.frame_id,self.cfg.nav2.base_frame
                try:
                    if not source:
                        raise ValueError("empty IMU frame")
                    if source != base and getattr(self,"_imu_mount_pair",None)!=(base,source):
                        from rclpy.time import Time
                        # This verifies the rigid sensor mounting, not a world
                        # pose at exposure time. Exact-stamp lookup can fail
                        # merely because ROS TF delivery trails the IMU. The
                        # measured yaw association below still uses its source
                        # timestamp and the strict 150ms maximum age.
                        transform = self._tf.lookup_transform(base,source,Time())
                        rotation = transform.transform.rotation
                        mount_norm = math.sqrt(sum(value*value for value in
                            (rotation.x,rotation.y,rotation.z,rotation.w)))
                        if not math.isfinite(mount_norm) or abs(mount_norm-1.)>=.02:
                            raise ValueError("Invalid IMU mounting transform quaternion")
                        mount_r,mount_p,_ = quat_to_rpy_deg(rotation.x,rotation.y,rotation.z,rotation.w)
                        if max(abs(mount_r),abs(mount_p))>1.:
                            raise ValueError("IMU yaw fusion requires a verified level mounting")
                        self._imu_mount_pair=(base,source)
                        self.robot.stats["imu_heading_mount"]={"base":base,"sensor":source,
                            "roll_deg":mount_r,"pitch_deg":mount_p,"verified_rigid_mount":True}
                except Exception as error:
                    valid=False;reason="IMU frame validation: "+type(error).__name__
            accepted = fusion.add_imu(stamp,math.radians(y),valid=valid,reason=reason)
            self.robot.stats["imu_odometry"] = fusion.diagnostics()
            if accepted:
                self.robot.stats.pop("imu_heading_error",None)
            else:
                self.robot.stats["imu_heading_error"]=reason or "Invalid or out-of-order IMU timestamp"

    @staticmethod
    def _get(m, *names):
        for n in names:
            v = getattr(m, n, None)
            if v is not None:
                return v
        return None

    def _on_state(self, m) -> None:
        self.robot.on_state(self._get(m, "hp"), self._get(m, "hp_max"), self._get(m, "battery_pct", "battery"))
        # mode/cmd 가 /mil/state 에 같이 들어 있는 구현도 받는다 (p10 표는 둘을 한 줄에 적었다)
        mode, cmd = self._get(m, "mode"), self._get(m, "cmd")
        if mode is not None or cmd is not None:
            self.robot.on_match(str(mode or ""), str(cmd or ""))

    def _on_match(self, m) -> None:
        self.robot.on_match(str(self._get(m, "mode") or ""), str(self._get(m, "cmd") or ""))

    def close(self) -> None:
        self._closed = True
        try:
            self.publish_cmd(0.0, 0.0)
            self.node.destroy_node()
        except Exception:
            pass
