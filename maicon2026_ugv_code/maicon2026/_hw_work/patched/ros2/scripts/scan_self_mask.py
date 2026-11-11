#!/usr/bin/env python3
"""LiDAR 자기 가림 필터 노드: /scan → /scan_filtered  (Cartographer·slam_toolbox·Nav2 가 쓰는 스캔용).

Agent 는 원본 /scan 을 받아 자기 SelfMask(nav/selfmask.py)를 쓴다. 그런데 SLAM 과 Nav2 비용지도는 원본 /scan 을 읽으므로 팬틸트 기둥·안테나·
브래킷에서 오는 반사가 차체에 붙은 가짜 장애물이 된다. 이 노드는 **같은 SelfMask.apply** 로 같은 광선을 지워 /scan_filtered 로 내보낸다.
Agent 는 계속 원본 /scan 을 쓴다 — 마스크가 두 번 적용되는 일이 없다.

  - 각도 기준: 차체(base) 각도(전방 0, 왼쪽 +). LiDAR 장착 변환(x, y, yaw)은 TF base←LiDAR 에서 읽고, 광선 끝점을 차체 원점 기준 극좌표로
    옮기는 계산은 hal/ros_mil.py 의 scan_rays_in_base(= Agent 의 scan_in_base 가 쓰는 함수)를 그대로 부른다. 가까움 판정의 거리도 차체 원점 기준이다.
  - 지운 광선은 NaN(정보 없음). inf 를 쓰면 Nav2 가 그 방향의 장애물을 지우는 광선으로 쓸 수 있다.
  - header, 각도 범위·증분, range_min/max, QoS(센서 데이터)는 입력과 같다. 광선 수와 순서도 같다(광선을 재배열하지 않는다).
  - TF 를 못 읽으면 그 스캔은 내보내지 않는다(Agent 의 _on_scan 과 같은 원칙: 원본을 몰래 통과시키면 가짜 장애물이 그대로 들어간다).

  python3 ros2/scripts/scan_self_mask.py --sectors "170:15,-170:15" --mask-range 0.35 --min-range 0 --base-frame base_footprint
  --sectors 는 '중심deg:반폭deg' 쉼표 목록(= lidar.self_mask_sectors). 보통은 ros2/launch/maicon_waveshare.launch.py 의 scan_self_mask 인자로 켠다.

rclpy 는 main() 안에서만 import 한다 → 순수 함수(parse_sectors, mask_ranges, filtered_fields)는 ROS 없이 시험된다.
"""
import argparse
import math
import os
import sys
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from maicon26.config import LidarCfg                     # noqa: E402
from maicon26.hal.ros_mil import scan_rays_in_base       # noqa: E402  (rclpy 는 이 모듈 안에서 늦게 import 된다)
from maicon26.nav.selfmask import SelfMask               # noqa: E402
from maicon26.types import Scan                          # noqa: E402


def parse_sectors(text: str) -> Tuple[Tuple[float, float], ...]:
    """'170:15,-170:15' → ((170.0, 15.0), (-170.0, 15.0)). 빈 문자열은 ()."""
    out = []
    for part in str(text or "").replace(";", ",").split(","):
        part = part.strip()
        if not part:
            continue
        try:
            c, h = (float(v) for v in part.split(":"))
        except ValueError:
            raise ValueError(f"scan_self_mask 구간 형식은 '중심deg:반폭deg' (받은 값: {part!r})") from None
        if not (math.isfinite(c) and math.isfinite(h)) or not (0.0 < h <= 180.0):
            raise ValueError(f"scan_self_mask 구간 값이 올바르지 않다: {part!r} (반폭은 0 초과 180 이하)")
        out.append((c, h))
    return tuple(out)


@dataclass
class _Rays(Scan):
    """광선마다 각도가 제각각인 Scan — SelfMask.apply 가 scan.angles 만 쓰므로 그 값을 광선별 차체 각도로 바꿔 끼운다."""
    theta: Optional[np.ndarray] = None

    @property
    def angles(self) -> np.ndarray:
        return self.theta


def mask_ranges(angle_min, angle_inc, ranges, range_min, range_max, x, y, yaw, mask: SelfMask) -> np.ndarray:
    """원본 LaserScan 거리 배열 → 같은 길이의 배열. 마스크에 걸린 광선만 NaN, 나머지는 입력 그대로(NaN·inf 포함)."""
    r = np.asarray(ranges, dtype=float)
    out = r.copy()
    if not mask.active or r.size == 0:
        return out
    ok, theta, rr = scan_rays_in_base(angle_min, angle_inc, r, range_min, range_max, x, y, yaw)
    if not ok.any():
        return out
    probe = _Rays(0.0, 1.0, rr, theta=theta)
    cleared = np.isnan(mask.apply(probe).ranges)              # rr 는 모두 유한 → NaN 이면 마스크가 지운 광선
    out[np.flatnonzero(ok)[cleared]] = np.nan
    return out


def filtered_fields(msg, mount: Tuple[float, float, float], mask: SelfMask) -> dict:
    """LaserScan 메시지 → 같은 필드의 dict(ranges 만 필터됨). header·각도·range_min/max·시간 필드·intensities 는 입력 그대로."""
    x, y, yaw = mount
    fields = {name: getattr(msg, name) for name in ("header", "angle_min", "angle_max", "angle_increment", "time_increment",
                                                    "scan_time", "range_min", "range_max", "intensities")}
    fields["ranges"] = mask_ranges(msg.angle_min, msg.angle_increment, msg.ranges, msg.range_min, msg.range_max,
                                   x, y, yaw, mask).astype(np.float32).tolist()
    return fields


def build_mask(sectors, mask_range: float, min_range: float) -> SelfMask:
    return SelfMask(LidarCfg(self_mask_sectors=tuple(sectors), self_mask_range=float(mask_range), min_range=float(min_range)))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="LiDAR 자기 가림 필터: /scan → /scan_filtered")
    ap.add_argument("--input", default="/scan")
    ap.add_argument("--output", default="/scan_filtered")
    ap.add_argument("--base-frame", default="base_footprint")
    ap.add_argument("--sectors", default="", help="'중심deg:반폭deg' 쉼표 목록 (= lidar.self_mask_sectors)")
    ap.add_argument("--mask-range", type=float, default=0.35, help="구간 안에서 이보다 가까운 반사만 지운다 [m]")
    ap.add_argument("--min-range", type=float, default=0.0, help="모든 방향에서 이보다 가까운 반사를 지운다 [m] (0 = 끔)")
    ap.add_argument("--use-sim-time", action="store_true")
    args = ap.parse_args(argv)
    mask = build_mask(parse_sectors(args.sectors), args.mask_range, args.min_range)
    if not mask.active:
        print("[scan_self_mask] 마스크가 비어 있어 아무것도 하지 않고 끝낸다 (SLAM·Nav2 는 원본 스캔을 쓴다)", flush=True)
        return 0

    import rclpy                                                     # ROS 가 없는 환경의 시험이 이 파일을 import 할 수 있게 여기서만 import
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener
    from maicon26.hal.ros_mil import lidar_mount_from_transform

    rclpy.init()
    node = rclpy.create_node("scan_self_mask", parameter_overrides=[Parameter("use_sim_time", value=bool(args.use_sim_time))])
    buf = Buffer()
    TransformListener(buf, node)
    pub = node.create_publisher(LaserScan, args.output, qos_profile_sensor_data)
    state = {"in": 0, "out": 0, "tf_fail": 0, "masked": 0, "log_t": 0.0}

    def on_scan(m) -> None:
        state["in"] += 1
        source = m.header.frame_id
        if not source:
            return
        x = y = yaw = 0.0
        if source != args.base_frame:
            try:
                x, y, yaw = lidar_mount_from_transform(buf.lookup_transform(args.base_frame, source, Time()))
            except Exception as e:                                   # 정적 TF 가 아직 없거나 LiDAR 평면이 수평이 아니다
                state["tf_fail"] += 1
                now = node.get_clock().now().nanoseconds * 1e-9
                if now - state["log_t"] > 5.0:
                    state["log_t"] = now
                    node.get_logger().warning(f"{args.base_frame} ← {source} 변환을 못 읽어 스캔을 내보내지 않는다: {e}")
                return
        f = filtered_fields(m, (x, y, yaw), mask)
        out = LaserScan()
        for name, value in f.items():
            setattr(out, name, value)
        state["masked"] = int(np.isnan(np.asarray(f["ranges"])).sum() - np.isnan(np.asarray(m.ranges, dtype=float)).sum())
        pub.publish(out)
        state["out"] += 1
        now = node.get_clock().now().nanoseconds * 1e-9
        if now - state["log_t"] > 30.0:
            state["log_t"] = now
            node.get_logger().info(f"in={state['in']} out={state['out']} tf_fail={state['tf_fail']} 마지막 스캔에서 지운 광선={state['masked']}")

    node.create_subscription(LaserScan, args.input, on_scan, qos_profile_sensor_data)
    node.get_logger().info(f"{args.input} → {args.output} 구간={mask.cfg.self_mask_sectors} 범위={args.mask_range} min_range={args.min_range}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
