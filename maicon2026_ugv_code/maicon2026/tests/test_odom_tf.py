"""/odom → TF 변환(순수 부분). rclpy 를 쓰는 발행·중복 회피는 WSL 에서 실제 ROS2 로 확인했다(docs/2차멘토링_API_반영.md)."""
import math
from types import SimpleNamespace as NS

from maicon26.hal.odom_tf import pose_to_transform


def _odom(x=1.0, y=-2.0, z=0.0, q=(0.0, 0.0, 0.0, 1.0), sec=5, nsec=7):
    return NS(header=NS(stamp=NS(sec=sec, nanosec=nsec)),
              pose=NS(pose=NS(position=NS(x=x, y=y, z=z), orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3]))))


def test_pose_maps_to_transform():
    th = 0.8
    d = pose_to_transform(_odom(q=(0, 0, math.sin(th / 2), math.cos(th / 2))), "odom", "base_link")
    assert (d["parent"], d["child"]) == ("odom", "base_link")
    assert d["t"] == (1.0, -2.0, 0.0) and d["stamp"] == (5, 7)
    assert abs(d["q"][2] - math.sin(th / 2)) < 1e-9 and abs(d["q"][3] - math.cos(th / 2)) < 1e-9


def test_empty_quaternion_becomes_identity_and_zero_stamp_is_none():
    d = pose_to_transform(_odom(q=(0, 0, 0, 0), sec=0, nsec=0), "odom", "base_link")
    assert d["q"] == (0.0, 0.0, 0.0, 1.0)                           # (0,0,0,0) 은 tf2 가 거부한다
    assert d["stamp"] is None                                       # 호출자가 노드 시계로 채운다


def test_base_frame_resolution_follows_real_config():
    from pathlib import Path
    from maicon26.hal.odom_tf import resolve_base_frame
    real = str(Path(__file__).resolve().parents[1] / "config" / "waveshare_real.json")
    assert resolve_base_frame(None, None) == "base_link"
    assert resolve_base_frame(None, real) == "base_footprint"       # 실기 설정과 같은 프레임
    assert resolve_base_frame("base_link", real) == "base_link"     # 명시한 값이 우선


def test_non_finite_pose_is_rejected():
    for bad in (_odom(x=float("nan")), _odom(y=float("inf")), _odom(q=(0, 0, float("nan"), 1.0))):
        try:
            pose_to_transform(bad, "odom", "base_link")
        except ValueError:
            continue
        raise AssertionError("NaN/inf pose must not become a TF")


def test_unnormalized_quaternion_is_normalized():
    d = pose_to_transform(_odom(q=(0, 0, 2.0, 2.0)), "odom", "base_link")
    assert abs(sum(c * c for c in d["q"]) - 1.0) < 1e-12
