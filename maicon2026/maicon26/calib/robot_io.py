"""보정 시험이 로봇을 다루는 최소 인터페이스. 구현 두 가지:
  SimIO  2D 시뮬(참값 있음, ROS 불필요) — 분석 코드가 숨은 정답을 복원하는지 시험하는 데 쓴다.
  RosIO  실기/ROS 브리지 — /cmd_vel, /odom, /imu, /scan 구독, (선택) /ground_truth 또는 수기 측정.
"""
from __future__ import annotations

import json
import math
import sys
import threading
import time
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from ..types import Scan

NAN = float("nan")


class RobotIO(ABC):
    name = "io"
    dt = 0.02                                   # 제어/기록 주기 (50 Hz)

    @abstractmethod
    def set_cmd(self, v: float, w: float) -> None: ...

    @abstractmethod
    def tick(self) -> Dict[str, float]:
        """한 주기 진행(시뮬은 한 스텝, ROS 는 주기만큼 대기) 후 최신 센서 표본."""

    @abstractmethod
    def place(self, x: float, y: float, th: float, prompt: str = "") -> None:
        """시험 시작 자세. 시뮬은 순간이동, 실기는 '로봇을 옮기고 Enter'."""

    def has_truth(self) -> bool:
        return False

    def end_truth(self, kind: str) -> Optional[Dict[str, float]]:
        """시험 끝의 수기/참값 측정. None 이면 없음."""
        return None

    @abstractmethod
    def scans(self, n: int) -> List[Scan]: ...

    def info(self) -> Dict[str, Any]:
        return {"io": self.name}

    # ---- 장애물 회피(항법 스택 포함) 시험용: 선택 기능
    def avoid_supported(self) -> bool:
        return False

    def avoid_setup(self, forward: float, lateral: float) -> None: ...

    def avoid_start(self, forward: float, lateral: float) -> None: ...

    def avoid_poll(self) -> Tuple[str, int]:
        """(상태: running|succeeded|failed, 복구 횟수)"""
        return "failed", 0

    def avoid_stop(self) -> None: ...

    def close(self) -> None: ...


# ====================================================================== 시뮬
class SimIO(RobotIO):
    name = "sim"

    def __init__(self, drive=None, robot: Optional[Dict[str, float]] = None, surface: str = "default",
                 seed: int = 0, dt: float = 0.02) -> None:
        from ..config import Config
        from ..sim.drive import SimDrive
        from ..sim.world import World
        cfg = Config()
        cfg.mission.t_total = 1e9                     # 시험 중 경기 종료로 정지하지 않게
        for k, v in (robot or {}).items():
            setattr(cfg.robot, k, v)
        self.cfg, self.dt = cfg, dt
        self.world = World(cfg, seed=seed)
        self.world.cubes.clear()
        self.u = self.world.add_ugv("BLUE", 2, cfg)
        self.u.drive = drive or SimDrive()
        self.u.drive.surface = surface
        self._nav = None
        self._goal: Optional[Tuple[float, float]] = None
        self._scan_t = -1.0
        self._scan: Optional[Scan] = None

    def set_cmd(self, v: float, w: float) -> None:
        self.u.cmd_v, self.u.cmd_w = float(v), float(w)

    def has_truth(self) -> bool:
        return True

    def tick(self) -> Dict[str, float]:
        self.world.step(self.dt)
        p, o = self.u.pose, self.u.odom
        return {"t": self.world.t, "odom_x": o.x, "odom_y": o.y, "odom_th": o.th, "gyro_z": self.u.imu_gyro_z(),
                "truth_x": p.x, "truth_y": p.y, "truth_th": p.th, "scan_lat": 0.0,
                "odom_t": self.world.t, "truth_t": self.world.t, "imu_t": self.world.t,
                "scan_min": self.scan_min() if self._nav is not None else NAN}

    def place(self, x: float, y: float, th: float, prompt: str = "") -> None:
        u = self.u
        u.cmd_v = u.cmd_w = 0.0
        u.v = u.w = 0.0
        u.pose.x, u.pose.y, u.pose.th = x, y, th
        u.odom.x = u.odom.y = u.odom.th = 0.0
        u._cmd_q.clear()
        u._w_hist.clear()
        for _ in range(int(0.3 / self.dt)):            # 정지 안정화
            self.world.step(self.dt)

    def scans(self, n: int) -> List[Scan]:
        out = []
        for _ in range(n):
            self.world.step(self.dt)
            out.append(self.world.lidar(self.u))
        return out

    def info(self) -> Dict[str, Any]:
        return {"io": "sim", "drive_truth": self.u.drive.to_dict()}

    def end_truth(self, kind: str) -> Optional[Dict[str, float]]:
        p = self.u.pose
        return {"x": p.x, "y": p.y, "th": p.th}

    # ---- 회피 시험: 내장 항법기(A*)로 대신 (Nav2 없이 경로 위 박스를 돌아가는지 본다)
    def avoid_supported(self) -> bool:
        return True

    def avoid_setup(self, forward: float, lateral: float) -> None:
        p = self.u.pose
        c, s = math.cos(p.th), math.sin(p.th)
        mx, my = p.x + c * forward / 2 - s * lateral / 2, p.y + s * forward / 2 + c * lateral / 2
        self.world.add_blank_cube(mx, my)

    def avoid_start(self, forward: float, lateral: float) -> None:
        from ..nav.navigator import Navigator
        p = self.u.pose
        c, s = math.cos(p.th), math.sin(p.th)
        self._goal = (p.x + c * forward - s * lateral, p.y + s * forward + c * lateral)
        self._nav = Navigator(self.cfg)
        self._nav.set_goal(self._goal, 0.12)

    def avoid_poll(self) -> Tuple[str, int]:
        t = self.world.t
        if self._scan is None or t - self._scan_t >= 0.1:
            self._scan, self._scan_t = self.world.lidar(self.u), t
            self._nav.update_map(self._scan, self.u.pose)
        out = self._nav.step(self.u.pose, self._scan, t, self.dt)
        self.set_cmd(out.v, out.w)
        st = {"reached": "succeeded", "blocked": "failed"}.get(out.status, "running")
        return st, self._nav.recoveries

    def avoid_stop(self) -> None:
        self.set_cmd(0.0, 0.0)
        self._nav = None

    def scan_min(self) -> float:
        sc = self._scan or self.world.lidar(self.u)
        v = sc.valid()
        return float(sc.ranges[v].min()) if v.any() else NAN


# ====================================================================== ROS
class RosIO(RobotIO):
    name = "ros"

    def __init__(self, cmd_topic: str = "/cmd_vel", odom_topic: str = "/odom", imu_topic: str = "/imu",
                 scan_topic: str = "/scan", truth_topic: Optional[str] = None, use_sim_time: bool = False,
                 manual_truth: Optional[str] = None, interactive: bool = True, dt: float = 0.02,
                 nav2_cfg=None, teleport_topic: Optional[str] = None) -> None:
        from ..nav.ros_runtime import add_node, ensure_ros
        rclpy, _ = ensure_ros()
        from geometry_msgs.msg import Twist                         # type: ignore
        from nav_msgs.msg import Odometry                           # type: ignore
        from rclpy.node import Node                                 # type: ignore
        from rclpy.parameter import Parameter                       # type: ignore
        from sensor_msgs.msg import Imu, LaserScan                  # type: ignore
        self.dt = dt
        self._rclpy = rclpy
        self.node = Node("maicon26_calib_io", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)])
        self._Twist = Twist
        self.pub = self.node.create_publisher(Twist, cmd_topic, 10)
        self._lock = threading.Lock()
        self._odom = self._imu = self._truth = None
        self._odom_t = self._truth_t = self._imu_t = NAN
        self._scan: Optional[Any] = None
        self._scan_lat = NAN
        self.node.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.node.create_subscription(Imu, imu_topic, self._on_imu, 50)
        self.node.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self._truth_topic = truth_topic
        if truth_topic:
            self.node.create_subscription(Odometry, truth_topic, self._on_truth, 20)
        add_node(self.node)
        self._manual = {}
        if manual_truth:
            with open(manual_truth, "r", encoding="utf-8") as f:
                self._manual = json.load(f)
        self.interactive = interactive
        self._cmd = (0.0, 0.0)
        self._t_next = time.monotonic()
        self._origin_odom: Optional[Tuple[float, float, float]] = None   # place() 시점의 오도메트리(상대화)
        self._origin_truth: Optional[Tuple[float, float, float]] = None
        self.nav2 = None
        self._nav2_cfg = nav2_cfg
        self._pub_tp = None                                  # 시뮬 브리지 전용: 시험 시작 자세로 순간이동 / 박스 추가
        self._pub_box = None
        if teleport_topic:
            from geometry_msgs.msg import Point, PoseStamped        # type: ignore
            self._PoseStamped, self._Point = PoseStamped, Point
            self._pub_tp = self.node.create_publisher(PoseStamped, teleport_topic, 1)
            self._pub_box = self.node.create_publisher(Point, "/sim/add_box", 1)
            time.sleep(1.0)                                  # 발행자-구독자 연결 대기

    # ---- 콜백
    @staticmethod
    def _yaw(q) -> float:
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _hdr(m) -> float:
        return m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec

    def _on_odom(self, m) -> None:
        with self._lock:
            self._odom = (m.pose.pose.position.x, m.pose.pose.position.y, self._yaw(m.pose.pose.orientation))
            self._odom_t = self._hdr(m)

    def _on_truth(self, m) -> None:
        with self._lock:
            self._truth = (m.pose.pose.position.x, m.pose.pose.position.y, self._yaw(m.pose.pose.orientation))
            self._truth_t = self._hdr(m)

    def _on_imu(self, m) -> None:
        with self._lock:
            self._imu = float(m.angular_velocity.z)
            self._imu_t = self._hdr(m)

    def _on_scan(self, m) -> None:
        stamp = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        with self._lock:
            self._scan = m
            self._scan_lat = self._now() - stamp

    # ---- RobotIO
    def set_cmd(self, v: float, w: float) -> None:
        self._cmd = (float(v), float(w))
        m = self._Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.pub.publish(m)

    def has_truth(self) -> bool:
        return self._truth_topic is not None

    def tick(self) -> Dict[str, float]:
        self.set_cmd(*self._cmd)                                   # 명령 재발행(드라이버 워치독 대비)
        self._t_next += self.dt
        slack = self._t_next - time.monotonic()
        if slack > 0:
            time.sleep(slack)
        else:
            self._t_next = time.monotonic()
        with self._lock:
            o, im, tr, lat = self._odom, self._imu, self._truth, self._scan_lat
            sc = self._scan
            ot, tt, it = self._odom_t, self._truth_t, self._imu_t
        s: Dict[str, float] = {"t": self._now(), "gyro_z": im if im is not None else NAN, "scan_lat": lat,
                                 "odom_t": ot, "truth_t": tt, "imu_t": it}
        def rel(p, p0):                                      # 시험 시작 자세 기준(시작 방향이 +x)으로 변환
            dx, dy = p[0] - p0[0], p[1] - p0[1]
            c, sn = math.cos(p0[2]), math.sin(p0[2])
            return c * dx + sn * dy, -sn * dx + c * dy, p[2] - p0[2]
        if o is not None:
            s.update(zip(("odom_x", "odom_y", "odom_th"), rel(o, self._origin_odom or o)))
        if tr is not None:
            s.update(zip(("truth_x", "truth_y", "truth_th"), rel(tr, self._origin_truth or tr)))
        if sc is not None:
            r = np.asarray(sc.ranges, dtype=float)
            r = r[np.isfinite(r) & (r > max(sc.range_min, 0.02))]
            s["scan_min"] = float(r.min()) if len(r) else NAN
        return s

    def place(self, x: float, y: float, th: float, prompt: str = "") -> None:
        self.set_cmd(0.0, 0.0)
        if self._pub_tp is not None:                          # 시뮬: 자동으로 시작 자세에 놓는다
            m = self._PoseStamped()
            m.pose.position.x, m.pose.position.y = float(x), float(y)
            m.pose.orientation.z, m.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
            self._pub_tp.publish(m)
            time.sleep(0.8)
        elif prompt and self.interactive and sys.stdin.isatty():
            input(f"\n[준비] {prompt}\n  → 로봇을 놓고 정지한 뒤 Enter: ")
        time.sleep(0.5)
        with self._lock:                                           # 이후 표본은 이 자세 기준 상대값
            self._origin_odom = self._odom
            self._origin_truth = self._truth

    def end_truth(self, kind: str) -> Optional[Dict[str, float]]:
        if self._truth is not None:
            return None                                            # 연속 참값이 이미 기록됨
        if kind in self._manual:
            return self._manual[kind]
        if self.interactive and sys.stdin.isatty():
            print(f"\n[수기 측정] '{kind}' 시험 종료 자세 (시작 자세 기준, 줄자/각도기). 모르면 Enter(건너뜀)")
            out = {}
            for key, label in (("distance_m", "전진 거리 m"), ("lateral_m", "좌(+)/우(-) 편차 m"), ("heading_deg", "회전각 deg(반시계+)")):
                v = input(f"  {label}: ").strip()
                if v:
                    out[key] = float(v)
            return out or None
        return None

    def scans(self, n: int) -> List[Scan]:
        out, last = [], None
        t_end = time.monotonic() + 15.0
        while len(out) < n and time.monotonic() < t_end:
            time.sleep(0.02)
            with self._lock:
                m = self._scan
            if m is None or m is last:
                continue
            last = m
            out.append(Scan(angle_min=m.angle_min, angle_inc=m.angle_increment,
                            ranges=np.asarray(m.ranges, dtype=float), stamp=m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec,
                            range_max=float(m.range_max)))
        return out

    def info(self) -> Dict[str, Any]:
        return {"io": "ros", "truth_topic": self._truth_topic}

    # ---- 회피: Nav2 액션으로 (로봇 시작 자세 기준 상대 목표, map 프레임은 SLAM 시작 자세가 원점이라는 가정)
    def avoid_supported(self) -> bool:
        return True

    def avoid_setup(self, forward: float, lateral: float) -> None:
        if self._pub_box is not None:
            p = self._Point()
            p.x, p.y = forward / 2, lateral / 2
            self._pub_box.publish(p)
            time.sleep(0.5)
        elif self.interactive and sys.stdin.isatty():
            input(f"\n[준비] 로봇 정면 {forward/2:.2f} m 지점(경로 한가운데)에 박스(40 cm)를 놓고 Enter: ")

    def avoid_start(self, forward: float, lateral: float) -> None:
        from ..config import Config
        from ..nav.ros_nav2 import RosNav2Client
        cfg = self._nav2_cfg or Config()
        self.nav2 = RosNav2Client(cfg)
        m = self.nav2.lookup_map_to_base()
        if m is None:
            raise RuntimeError("TF map→base_link 없음 — SLAM 이 떠 있어야 한다")
        c, s = math.cos(m[2]), math.sin(m[2])
        self.nav2.send_goal(m[0] + c * forward - s * lateral, m[1] + s * forward + c * lateral, m[2])

    def avoid_poll(self) -> Tuple[str, int]:
        cmd = self.nav2.latest_cmd(0.0)                          # Agent 대신 Nav2 속도를 로봇(/cmd_vel)으로 중계
        self.set_cmd(*(cmd if cmd is not None else (0.0, 0.0)))
        st = self.nav2.goal_state()
        return {"succeeded": "succeeded", "active": "running", "none": "running"}.get(st, "failed"), int(self.nav2.recovering())

    def avoid_stop(self) -> None:
        if self.nav2 is not None:
            self.nav2.cancel()
            self.nav2.shutdown()
            self.nav2 = None
        self.set_cmd(0.0, 0.0)

    def close(self) -> None:
        self.set_cmd(0.0, 0.0)
        try:
            self.node.destroy_node()
        except Exception:
            pass
