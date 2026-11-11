"""/odom → TF odom→base_link 변환 노드. 로봇이 /odom 만 내고 TF 를 안 낼 때(2차 사전교육 PDF p15 에는 /odom 만 있다) SLAM·Nav2 가 돌게 한다.

  - 정적 TF(base_link→laser/imu/camera…)는 이 노드의 일이 아니다 → ros2/launch/robot_description.launch.py(URDF).
  - 다른 곳이 이미 odom→base_link 를 내고 있으면(로봇 드라이버) 중복 발행하면 TF 가 튄다 → 시작할 때 probe_s 동안 듣고,
    이미 있으면 쉰다(auto). 주기적으로 다시 확인해, 나중에 드라이버 TF 가 끊기면 이어받는다. --force 면 확인 없이 발행.
    한번 발행을 시작한 뒤 다른 발행자의 복귀를 판별하지 못하므로 EKF 구성에서는 이 브리지를 실행하지 않는다.
  - Odometry 프레임이 인자와 다르면 재명명하지 않고 거부한다. 좌표 변환 없이 프레임 이름만 바꾸면 안 된다.

실행: python3 scripts/odom_tf_bridge.py [--odom-frame odom --base-frame base_link] [--force] [--use-sim-time]
"""
from __future__ import annotations

import argparse
import math
from typing import Any, Dict, Optional


def pose_to_transform(odom_msg: Any, parent: str, child: str) -> Dict[str, Any]:
    """nav_msgs/Odometry(또는 같은 모양의 객체) → TransformStamped 에 넣을 값. rclpy 없이 시험된다.
    stamp 가 0 이면 stamp=None(호출자가 노드 시계로 채운다)."""
    if ((getattr(odom_msg.header, "frame_id", "") or parent) != parent or
            (getattr(odom_msg, "child_frame_id", "") or child) != child):
        raise ValueError("Odometry frames differ from requested TF; explicit transform required")
    p, q = odom_msg.pose.pose.position, odom_msg.pose.pose.orientation
    if not all(math.isfinite(float(v)) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
        raise ValueError("Odometry pose is not finite")     # NaN 이 TF 트리에 들어가면 SLAM·Nav2 가 같이 망가진다
    st = odom_msg.header.stamp
    sec, nsec = int(st.sec), int(st.nanosec)
    n = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) ** 0.5
    if n < 1e-6:                                       # 방향이 비어 있다(0,0,0,0) → 항등. 안 그러면 tf2 가 NaN/오류를 낸다
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    else:
        qx, qy, qz, qw = q.x / n, q.y / n, q.z / n, q.w / n
    return {"parent": parent, "child": child, "stamp": None if (sec == 0 and nsec == 0) else (sec, nsec),
            "t": (float(p.x), float(p.y), float(p.z)), "q": (qx, qy, qz, qw)}


def run(odom_topic: str = "/odom", parent: str = "odom", child: str = "base_link", force: bool = False,
        probe_s: float = 2.0, recheck_s: float = 3.0, use_sim_time: bool = False, spin_for: Optional[float] = None) -> Dict[str, Any]:  # pragma: no cover - rclpy
    import time

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.duration import Duration
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformBroadcaster, TransformListener

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("odom_tf_bridge", parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)])
    buf = Buffer()
    TransformListener(buf, node)
    br = TransformBroadcaster(node)
    st: Dict[str, Any] = {"active": force, "sent": 0, "seen": 0, "warned": False, "foreign_until": 0.0}
    t0 = time.monotonic()

    def foreign_exists() -> bool:
        """우리가 아닌 발행자가 odom→base_link 를 내고 있는가: 우리가 쉬는 동안 최근 변환이 들어오는지로 본다."""
        try:
            tf = buf.lookup_transform(parent, child, Time(), Duration(seconds=0.0))
        except Exception:
            return False
        age = (node.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        return age < 1.0

    def decide() -> None:
        if force:
            return
        if time.monotonic() - t0 < probe_s:
            return
        ext = foreign_exists() if not st["active"] else False
        if not st["active"] and not ext:
            st["active"] = True
            node.get_logger().info(f"TF {parent}→{child} 발행자가 없다 → 이 노드가 /odom 으로 발행한다")
        elif st["active"] is False and ext and not st["warned"]:
            st["warned"] = True
            node.get_logger().info(f"TF {parent}→{child} 를 이미 다른 노드가 낸다 → 쉰다")

    def on_odom(m) -> None:
        st["seen"] += 1
        if (m.header.frame_id and m.header.frame_id != parent) or (m.child_frame_id and m.child_frame_id != child):
            if not st.get("frame_warn"):
                st["frame_warn"] = True
                node.get_logger().warn(f"/odom 프레임 불일치({m.header.frame_id}→{m.child_frame_id}); TF 발행 거부")
            return
        decide()
        if not st["active"]:
            return
        try:
            d = pose_to_transform(m, parent, child)
        except ValueError:
            st["bad"] = st.get("bad", 0) + 1                # 유한하지 않은 /odom 표본은 건너뛴다(발행하지 않음)
            if st["bad"] == 1:
                node.get_logger().warn("/odom 위치·방향이 유한하지 않다; 해당 표본의 TF 발행을 건너뛴다")
            return
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = d["parent"], d["child"]
        if d["stamp"] is None:
            t.header.stamp = node.get_clock().now().to_msg()
        else:
            t.header.stamp.sec, t.header.stamp.nanosec = d["stamp"]
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = d["t"]
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = d["q"]
        br.sendTransform(t)
        st["sent"] += 1

    node.create_subscription(Odometry, odom_topic, on_odom, qos_profile_sensor_data)
    node.create_timer(recheck_s, decide)
    try:
        end = None if spin_for is None else time.monotonic() + spin_for
        while rclpy.ok() and (end is None or time.monotonic() < end):
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    return st


def resolve_base_frame(explicit: Optional[str], config_path: Optional[str]) -> str:
    """--base-frame 이 우선, 없으면 --config 의 nav2.base_frame, 둘 다 없으면 base_link. 실기 설정(base_footprint)과 어긋나 발행이 거부되는 것을 막는다."""
    if explicit:
        return explicit
    if config_path:
        from ..config import Config
        return Config.load(config_path).nav2.base_frame
    return "base_link"


def main(argv=None) -> int:  # pragma: no cover
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--odom-topic", default="/odom")
    ap.add_argument("--odom-frame", default="odom")
    ap.add_argument("--base-frame", default=None,
                    help="기본 base_link. --config 가 있으면 그 설정의 nav2.base_frame(실기는 base_footprint)")
    ap.add_argument("--config", default=None, help="Agent 설정 JSON(예: config/waveshare_real.json) — base 프레임을 거기서 읽는다")
    ap.add_argument("--force", action="store_true", help="다른 발행자 확인 없이 항상 발행")
    ap.add_argument("--probe-s", type=float, default=2.0)
    ap.add_argument("--use-sim-time", action="store_true")
    a = ap.parse_args(argv)
    base = resolve_base_frame(a.base_frame, a.config)
    run(a.odom_topic, a.odom_frame, base, a.force, a.probe_s, use_sim_time=a.use_sim_time)
    return 0
