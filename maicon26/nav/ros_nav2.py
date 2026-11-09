"""Nav2Client의 ROS2 Humble 구현 — !! 로봇(Jetson Docker)에서만 동작, 이 PC에서는 실행해 보지 못했다 !!

필요한 것: rclpy, tf2_ros, nav_msgs, geometry_msgs, nav2_msgs, action_msgs (ROS2 Humble 기본 + nav2).
연결 대상은 ros2/launch/maicon_nav.launch.py 가 띄운다:
  SLAM  →  /map (nav_msgs/OccupancyGrid), TF map→odom
  Nav2  →  액션 /navigate_to_pose, 속도 /cmd_vel_nav2 (geometry_msgs/Twist)
  로봇 드라이버 →  TF odom→base_link, /scan
이름은 config.nav2.* 로 바꿀 수 있다. 실기 토픽·프레임명이 다르면 거기만 고친다.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Optional, Tuple

import numpy as np

from ..config import Config
from .cmd_cache import CmdCache
from .nav2_stack import MapData, Nav2Client
from .ros_runtime import add_node, ensure_ros

log = logging.getLogger("maicon26.nav2")


class RosNav2Client(Nav2Client):
    RECOVERY_FLAG_S = 1.5            # 피드백의 복구 횟수가 늘면 이 시간 동안 recovering 으로 본다

    def __init__(self, cfg: Config) -> None:
        self.c2 = cfg.nav2
        rclpy, _ = ensure_ros()
        from rclpy.action import ActionClient                       # type: ignore
        from rclpy.node import Node                                 # type: ignore
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # type: ignore
        from geometry_msgs.msg import Twist                         # type: ignore
        from nav2_msgs.action import NavigateToPose                 # type: ignore
        from nav_msgs.msg import OccupancyGrid                      # type: ignore
        import tf2_ros                                              # type: ignore

        self._rclpy = rclpy
        self._NavigateToPose = NavigateToPose
        from rclpy.parameter import Parameter                       # type: ignore
        self.node = Node("maicon26_nav2_client",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, self.c2.use_sim_time)])
        self.tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node, spin_thread=False)

        map_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(OccupancyGrid, self.c2.map_topic, self._on_map, map_qos)
        self.node.create_subscription(Twist, self.c2.cmd_topic, self._on_cmd, 10)
        self._ac = ActionClient(self.node, NavigateToPose, self.c2.action_name)

        self._lock = threading.Lock()
        self._map: Optional[MapData] = None
        self._map_count = 0
        self._cc = CmdCache(self.c2.cmd_timeout)                    # 목표 전환 시 이전 목표의 속도를 버리는 게이트 (R06)
        self._state = "none"
        self._handle: Any = None
        self._seq = 0
        self._last_recoveries = 0
        self._recovering_until = 0.0
        add_node(self.node)
        if not self._ac.wait_for_server(timeout_sec=self.c2.server_wait_s):
            log.warning("Nav2 액션 서버(%s)가 %.0f s 안에 안 보인다 — ros2 launch maicon_nav 가 떠 있는지 확인",
                        self.c2.action_name, self.c2.server_wait_s)

    # ------------------------------------------------------------------ 구독 콜백 (실행기 스레드)
    def _on_map(self, m: Any) -> None:
        q = m.info.origin.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if abs(yaw) > 0.05:
            log.warning("지도 원점에 회전(%.2f rad)이 있다 — Nav2Stack 은 0 으로 가정한다", yaw)
        data = np.asarray(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
        with self._lock:
            self._map_count += 1             # header.stamp 는 0 이거나 안 바뀌는 퍼블리셔가 있어 수신 횟수로 구분한다
            self._map = MapData(data, float(m.info.resolution), float(m.info.origin.position.x),
                                float(m.info.origin.position.y), float(self._map_count))

    def _on_cmd(self, t: Any) -> None:
        self._cc.put(float(t.linear.x), float(t.angular.z), time.monotonic())

    # ------------------------------------------------------------------ Nav2Client
    def lookup_map_to_base(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(self.c2.map_frame, self.c2.base_frame, self._rclpy.time.Time())
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (float(t.x), float(t.y), yaw)

    def latest_map(self) -> Optional[MapData]:
        with self._lock:
            return self._map

    def latest_cmd(self, now: float) -> Optional[Tuple[float, float]]:
        return self._cc.get(time.monotonic())

    def cmd_stats(self) -> Tuple[int, int]:
        """(채택한 Nav2 속도 수, 목표 전환 게이트가 버린 속도 수) — 진단용."""
        return self._cc.accepted, self._cc.dropped

    def send_goal(self, x: float, y: float, yaw: float) -> None:
        if not self._ac.server_is_ready():
            log.warning("Nav2 액션 서버 미준비 — 목표 전송 보류")
            with self._lock:
                self._state = "aborted"
            return
        from geometry_msgs.msg import PoseStamped                   # type: ignore
        goal = self._NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.c2.map_frame
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        goal.pose = ps
        self._cc.invalidate()                                        # 이전 목표의 속도는 버린다. 새 목표가 수락되면 문을 연다
        with self._lock:
            self._seq += 1
            seq = self._seq
            self._state = "active"
            self._last_recoveries = 0
        fut = self._ac.send_goal_async(goal, feedback_callback=lambda fb, s=seq: self._on_feedback(s, fb))
        fut.add_done_callback(lambda f, s=seq: self._on_goal_response(s, f))

    def _on_goal_response(self, seq: int, fut: Any) -> None:
        handle = fut.result()
        with self._lock:
            if seq != self._seq:                               # 이미 대체/취소된 목표
                stale = True
            else:
                stale = False
                if not handle.accepted:
                    self._state = "rejected"
                else:
                    self._handle = handle
        if stale:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if handle.accepted:
            self._cc.open_gate()
            handle.get_result_async().add_done_callback(lambda f, s=seq: self._on_result(s, f))

    def _on_result(self, seq: int, fut: Any) -> None:
        from action_msgs.msg import GoalStatus                      # type: ignore
        status = fut.result().status
        name = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
                GoalStatus.STATUS_CANCELED: "canceled"}.get(status, "aborted")
        with self._lock:
            if seq == self._seq:
                self._state = name

    def _on_feedback(self, seq: int, fb: Any) -> None:
        n = int(fb.feedback.number_of_recoveries)
        with self._lock:
            if seq == self._seq and n > self._last_recoveries:
                self._last_recoveries = n
                self._recovering_until = time.monotonic() + self.RECOVERY_FLAG_S

    def cancel(self) -> None:
        self._cc.invalidate()                                        # 취소 직후 도착하는 속도 방지
        with self._lock:
            h = self._handle
            self._handle = None
            self._seq += 1                                      # 늦게 오는 결과 무시
            self._state = "none"
        if h is not None:
            h.cancel_goal_async()

    def goal_state(self) -> str:
        with self._lock:
            return self._state

    def available(self) -> bool:
        return bool(self._ac.server_is_ready())

    def recovering(self) -> bool:
        with self._lock:
            return time.monotonic() < self._recovering_until

    def set_arena_walls(self, pts_map: np.ndarray) -> None:
        """경기장 둘레 점을 1 Hz 로 PointCloud2(map 프레임)로 재발행 → Nav2 전역 비용지도의 virtual_walls 마킹 소스."""
        try:
            from sensor_msgs.msg import PointCloud2                 # type: ignore
            from sensor_msgs_py import point_cloud2                 # type: ignore
        except Exception as e:                                       # pragma: no cover
            log.warning("sensor_msgs_py 없음 — 외벽 점군 발행 생략 (%s). LiDAR가 보는 벽만으로 계획한다.", e)
            return
        pts3 = [(float(x), float(y), 0.1) for x, y in pts_map]
        pub = self.node.create_publisher(PointCloud2, self.c2.walls_topic, 1)

        def _tick() -> None:
            from std_msgs.msg import Header                         # type: ignore
            h = Header()
            h.frame_id = self.c2.map_frame
            h.stamp = self.node.get_clock().now().to_msg()
            pub.publish(point_cloud2.create_cloud_xyz32(h, pts3))

        self._walls_timer = self.node.create_timer(1.0, _tick)
        _tick()

    def set_depth_obstacles(self, pts_map: np.ndarray, height: float = 0.1) -> None:
        """3D 지도의 'LiDAR 가 못 보는' 장애물 셀을 PointCloud2(map 프레임)로 2 Hz 재발행 → Nav2 비용지도 depth_obstacles 마킹 소스.
        마킹 전용이라 사라진 물체의 표시는 LiDAR 클리어링이 못 지운다 — 낮은 물체는 정적이라 감수한다(그래서 UGV 같은 움직이는 물체는 보내지 않는다)."""
        with self._lock:
            self._depth_pts = [(float(x), float(y), float(height)) for x, y in np.asarray(pts_map).reshape(-1, 2)]
        if getattr(self, "_depth_pub", None) is None:
            try:
                from sensor_msgs.msg import PointCloud2             # type: ignore
                from sensor_msgs_py import point_cloud2             # type: ignore
            except Exception as e:                                   # pragma: no cover
                log.warning("sensor_msgs_py 없음 — 깊이 장애물 점군 발행 생략 (%s)", e)
                self._depth_pub = False
                return
            self._depth_pub = self.node.create_publisher(PointCloud2, self.c2.depth_obstacles_topic, 1)
            self._pc2 = point_cloud2

            def _tick() -> None:
                from std_msgs.msg import Header                     # type: ignore
                h = Header()
                h.frame_id = self.c2.map_frame
                h.stamp = self.node.get_clock().now().to_msg()
                with self._lock:
                    pts = list(self._depth_pts)
                self._depth_pub.publish(self._pc2.create_cloud_xyz32(h, pts))

            self._depth_timer = self.node.create_timer(0.5, _tick)

    def set_keepout_mask(self, mask: MapData) -> bool:
        """Replace transient policy masks; never insert fake sensor obstacles.

        Humble KeepoutFilter consumes CostmapFilterInfo(type=0, base=0,
        multiplier=1) and a reliable transient-local OccupancyGrid. A zero mask
        clears the preceding match rather than leaving marking-only ghosts.
        """
        from nav_msgs.msg import OccupancyGrid                     # type: ignore
        from nav2_msgs.msg import CostmapFilterInfo                 # type: ignore
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # type: ignore
        if not hasattr(self, "_keepout_mask_pub"):
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._keepout_mask_pub = self.node.create_publisher(OccupancyGrid, self.c2.keepout_mask_topic, qos)
            self._keepout_info_pub = self.node.create_publisher(CostmapFilterInfo, self.c2.keepout_info_topic, qos)
        grid = OccupancyGrid()
        grid.header.frame_id = self.c2.map_frame
        grid.header.stamp = self.node.get_clock().now().to_msg()
        grid.info.resolution = float(mask.resolution)
        grid.info.height, grid.info.width = map(int, mask.data.shape)
        grid.info.origin.position.x, grid.info.origin.position.y = float(mask.origin_x), float(mask.origin_y)
        grid.info.origin.orientation.w = 1.
        grid.data = np.asarray(mask.data, dtype=np.int8).reshape(-1).tolist()
        self._keepout_mask_pub.publish(grid)
        if not getattr(self, "_keepout_info_sent", False):
            info = CostmapFilterInfo()
            info.header = grid.header
            info.type, info.base, info.multiplier = 0, 0., 1.
            info.filter_mask_topic = self.c2.keepout_mask_topic
            self._keepout_info_pub.publish(info)
            self._keepout_info_sent = True
        return self._keepout_mask_pub.get_subscription_count() > 0

    def shutdown(self) -> None:
        try:
            self.node.destroy_node()
        except Exception:
            pass
