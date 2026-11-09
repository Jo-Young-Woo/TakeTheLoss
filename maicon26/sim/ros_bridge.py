"""2D 시뮬레이터 → ROS2 브리지. 실제 SLAM + Nav2 / 보정 시험을 시뮬 로봇으로 돌려 보기 위한 것(WSL Ubuntu 22.04 + Humble).

시뮬이 ROS 센서처럼 발행하는 것:  /clock(시뮬 시각, sim_clock=True 일 때), /scan(LaserScan), /odom + TF odom→base_link
                                  (드리프트 있는 오도메트리), /imu(자이로 z), /ground_truth(참 자세)
시뮬이 ROS 에서 받는 것:        /cmd_vel (최종 속도), /sim/teleport(PoseStamped: 시험 시작 자세로 순간이동),
                                  /sim/add_box(Point: 로봇 기준 위치에 박스 추가 — 회피 시험용)
Nav2 의 출력 /cmd_vel_nav2 는 로봇에 연결하지 않는다 — 실기와 같은 구조(Agent 가 중재)를 그대로 시험한다.

Nav2 용: 모든 ROS 노드를 use_sim_time:=true 로 (sim_clock=True, wall_stamps=False).
보정 시험용: 벽시계 기준 (sim_clock=False, wall_stamps=True) — RosIO 가 노드 시계로 표본 시각을 찍는다.
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np

from ..nav.ros_runtime import add_node, ensure_ros
from .robot import SimRobot
from .world import World

T0 = 100.0           # 발행 시각 오프셋: 시각 0 은 TF/Cartographer 에서 '최신'을 뜻하는 특수값이라 피한다


def _stamp(t: float) -> Any:
    from builtin_interfaces.msg import Time        # type: ignore
    tt = T0 + t
    return Time(sec=int(tt), nanosec=int((tt - int(tt)) * 1e9))


class RosSimBridge:
    def __init__(self, world: World, robot: SimRobot, sim_clock: bool = True, wall_stamps: bool = False,
                 redirect_agent_cmd: bool = True, laser_frame: str = "base_link") -> None:
        rclpy, _ = ensure_ros()
        from geometry_msgs.msg import Point, PoseStamped, Twist, TransformStamped      # type: ignore
        from nav_msgs.msg import Odometry                           # type: ignore
        from rclpy.node import Node                                 # type: ignore
        from rosgraph_msgs.msg import Clock                         # type: ignore
        from sensor_msgs.msg import Imu, LaserScan                  # type: ignore
        from tf2_ros import TransformBroadcaster                    # type: ignore

        self.world, self.robot, self.u = world, robot, robot.u
        self.sim_clock, self.wall, self.laser_frame = sim_clock, wall_stamps, laser_frame
        self.node = Node("maicon26_sim_bridge")
        self._Clock, self._Scan, self._Odom, self._Twist, self._TFS, self._Imu = Clock, LaserScan, Odometry, Twist, TransformStamped, Imu
        self.pub_clock = self.node.create_publisher(Clock, "/clock", 10) if sim_clock else None
        self.pub_scan = self.node.create_publisher(LaserScan, "/scan", 10)
        self.pub_odom = self.node.create_publisher(Odometry, "/odom", 10)
        self.pub_imu = self.node.create_publisher(Imu, "/imu", 10)
        self.pub_truth = self.node.create_publisher(Odometry, "/ground_truth", 10)
        self.pub_cmd = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.node.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.node.create_subscription(PoseStamped, "/sim/teleport", self._on_teleport, 10)
        self.node.create_subscription(Point, "/sim/add_box", self._on_add_box, 10)
        self.tf = TransformBroadcaster(self.node)
        self.pub_depth = self.pub_depth_info = None            # 깊이 카메라는 처음 프레임이 있을 때 만든다
        self._depth_stamp = -1.0
        self._scan_stamp = -1.0
        self.n_cmd = 0
        add_node(self.node)
        if redirect_agent_cmd:
            # Agent 의 set_velocity 를 ROS(/cmd_vel)로 돌린다: Agent → /cmd_vel → 브리지 → 시뮬 로봇 (실기의 드라이버 구독과 같은 경로)
            robot.set_velocity = self._send_cmd                         # type: ignore[assignment]

    # ------------------------------------------------------------------ 시각
    def _st(self, t: float) -> Any:
        return self.node.get_clock().now().to_msg() if self.wall else _stamp(t)

    # ------------------------------------------------------------------ 수신
    def _send_cmd(self, v: float, w: float) -> None:
        m = self._Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.pub_cmd.publish(m)

    def _on_cmd(self, m: Any) -> None:
        self.u.cmd_v, self.u.cmd_w = float(m.linear.x), float(m.angular.z)
        self.n_cmd += 1

    @staticmethod
    def _yaw(q: Any) -> float:
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def _on_teleport(self, m: Any) -> None:
        u = self.u                                                       # 오도메트리는 그대로 둔다(실기에서 로봇을 옮겨도 오도메트리는 안 리셋)
        u.cmd_v = u.cmd_w = 0.0
        u.v = u.w = 0.0
        u.pose.x, u.pose.y, u.pose.th = m.pose.position.x, m.pose.position.y, self._yaw(m.pose.orientation)
        u._cmd_q.clear()
        u._w_hist.clear()

    def _on_add_box(self, m: Any) -> None:
        p = self.u.pose
        c, s = math.cos(p.th), math.sin(p.th)
        self.world.add_blank_cube(p.x + c * m.x - s * m.y, p.y + s * m.x + c * m.y)

    # ------------------------------------------------------------------ 센서 발행 (한 틱에 한 번)
    def publish(self) -> None:
        t = self.world.t
        st = self._st(t)
        if self.pub_clock is not None:
            clk = self._Clock()
            clk.clock = _stamp(t)
            self.pub_clock.publish(clk)

        o = self.u.odom
        tf = self._TFS()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = st, "odom", "base_link"
        tf.transform.translation.x, tf.transform.translation.y = float(o.x), float(o.y)
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(o.th / 2), math.cos(o.th / 2)
        self.tf.sendTransform(tf)
        od = self._Odom()
        od.header.stamp, od.header.frame_id, od.child_frame_id = st, "odom", "base_link"
        od.pose.pose.position.x, od.pose.pose.position.y = float(o.x), float(o.y)
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(o.th / 2), math.cos(o.th / 2)
        od.twist.twist.linear.x, od.twist.twist.angular.z = float(self.u.v), float(self.u.w)
        self.pub_odom.publish(od)

        p = self.u.pose
        gt = self._Odom()
        gt.header.stamp, gt.header.frame_id, gt.child_frame_id = st, "world", "base_link"
        gt.pose.pose.position.x, gt.pose.pose.position.y = float(p.x), float(p.y)
        gt.pose.pose.orientation.z, gt.pose.pose.orientation.w = math.sin(p.th / 2), math.cos(p.th / 2)
        self.pub_truth.publish(gt)

        im = self._Imu()
        im.header.stamp, im.header.frame_id = st, "imu_link"
        im.orientation_covariance[0] = -1.0                              # 방향 추정 없음
        im.angular_velocity.z = float(self.u.imu_gyro_z())
        self.pub_imu.publish(im)

        self._publish_depth()
        sc = self.robot.get_scan()
        if sc is not None and sc.stamp != self._scan_stamp:
            self._scan_stamp = sc.stamp
            m = self._Scan()
            m.header.stamp, m.header.frame_id = self._st(sc.stamp), self.laser_frame
            n = len(sc.ranges)
            m.angle_min, m.angle_increment = float(sc.angle_min), float(sc.angle_inc)
            m.angle_max = float(sc.angle_min + (n - 1) * sc.angle_inc)
            m.scan_time, m.time_increment = 0.1, 0.0
            m.range_min, m.range_max = 0.05, float(sc.range_max)
            m.ranges = [float(x) for x in np.asarray(sc.ranges, dtype=np.float32)]
            self.pub_scan.publish(m)

    def _publish_depth(self) -> None:
        """깊이 영상(32FC1, m) + camera_info + TF base_link→depth_optical(촬영 시점의 팬/틸트 포함). 실기의 OAK 드라이버와 같은 모양이다."""
        df = self.robot.get_depth()
        if df is None or df.stamp == self._depth_stamp:
            return
        self._depth_stamp = df.stamp
        if self.pub_depth is None:
            from sensor_msgs.msg import CameraInfo, Image          # type: ignore
            dc = self.world.cfg.depth
            self._Image, self._CamInfo = Image, CameraInfo
            self.pub_depth = self.node.create_publisher(Image, dc.topic, 2)
            self.pub_depth_info = self.node.create_publisher(CameraInfo, dc.info_topic, 2)
        from ..geom3d import RigModel
        st = self._st(df.stamp)
        m = self._Image()
        m.header.stamp, m.header.frame_id = st, "depth_optical"
        m.height, m.width, m.encoding, m.is_bigendian = df.height, df.width, "32FC1", 0
        m.step = df.width * 4
        m.data = np.ascontiguousarray(df.depth, dtype="<f4").tobytes()
        self.pub_depth.publish(m)
        ci = self._CamInfo()
        ci.header = m.header
        ci.height, ci.width = df.height, df.width
        K = df.K or tuple(self.world.cfg.depth.intrinsics(df.width, df.height).K())
        ci.k = [float(x) for x in K]
        ci.p = [K[0], 0.0, K[2], 0.0, 0.0, K[4], K[5], 0.0, 0.0, 0.0, 1.0, 0.0]
        ci.d = [0.0] * 5
        ci.distortion_model = "plumb_bob"
        self.pub_depth_info.publish(ci)
        rig = RigModel(self.world.cfg.rig)
        T = rig.optical_in_world(0.0, 0.0, 0.0, df.pan, df.tilt, "depth")      # base_link 을 원점으로 둔 변환(높이는 base_z 만큼 빼서)
        R = T.R
        tf = self._TFS()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = st, "base_link", "depth_optical"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = float(T.t[0]), float(T.t[1]), float(T.t[2] - rig.base_z)
        qw = math.sqrt(max(1.0 + R[0, 0] + R[1, 1] + R[2, 2], 0.0)) / 2
        qw = max(qw, 1e-9)
        tf.transform.rotation.w = qw
        tf.transform.rotation.x = float((R[2, 1] - R[1, 2]) / (4 * qw))
        tf.transform.rotation.y = float((R[0, 2] - R[2, 0]) / (4 * qw))
        tf.transform.rotation.z = float((R[1, 0] - R[0, 1]) / (4 * qw))
        self.tf.sendTransform(tf)

    def close(self) -> None:
        try:
            self.node.destroy_node()
        except Exception:
            pass
