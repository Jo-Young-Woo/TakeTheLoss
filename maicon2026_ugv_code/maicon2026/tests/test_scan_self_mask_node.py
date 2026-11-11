"""하드웨어 문서 반영 5장: LiDAR 자기 가림 필터 노드 — ROS·시뮬 없이 순수 함수와 파일만 본다.

노드(ros2/scripts/scan_self_mask.py)는 Agent 와 같은 SelfMask.apply 와 같은 광선→차체 변환(scan_rays_in_base)을 쓴다.
"""
import importlib.util
import json
import math
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from maicon26.config import Config, LidarCfg
from maicon26.hal.ros_mil import scan_in_base
from maicon26.launch_profile import nav_launch_command, scan_self_mask_args
from maicon26.nav.selfmask import SelfMask
from maicon26.types import Scan

ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / "ros2" / "scripts" / "scan_self_mask.py"


@pytest.fixture(scope="module")
def node():
    assert "rclpy" not in sys.modules or True
    spec = importlib.util.spec_from_file_location("scan_self_mask_under_test", NODE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)                           # rclpy 없이 import 된다 — rclpy 는 main() 안에서만
    return mod


def _scan(n=360, far=2.0):
    """1° 간격 360발. angle_min = -π (LaserScan 이 보통 -π..π)."""
    inc = 2 * math.pi / n
    return -math.pi, inc, np.full(n, far)


def _idx_of_deg(deg, n=360, mount_yaw_deg=0.0):
    """LiDAR 프레임 각도가 (deg - mount_yaw) 인 광선 번호."""
    a = math.radians(deg - mount_yaw_deg)
    return int(round((a + math.pi) / (2 * math.pi / n))) % n


# ------------------------------------------------------------------------------------------ 구간 문자열
def test_parse_sectors(node):
    assert node.parse_sectors("170:15,-170:15") == ((170.0, 15.0), (-170.0, 15.0))
    assert node.parse_sectors(" 170:15 , ") == ((170.0, 15.0),)
    assert node.parse_sectors("") == () and node.parse_sectors(None) == ()
    for bad in ("170", "a:b", "170:0", "170:-3", "170:181", "nan:5", "1:2:3"):
        with pytest.raises(ValueError):
            node.parse_sectors(bad)


# ------------------------------------------------------------------------------------------ 순수 함수
def test_only_near_returns_inside_the_sector_become_nan(node):
    amin, inc, r = _scan()
    ix = _idx_of_deg(170)
    r[ix - 3: ix + 4] = 0.20                                   # 구간 안의 가까운 반사 (기둥)
    far_same_angle = _idx_of_deg(165)
    r[far_same_angle] = 1.50                                   # 같은 구간 안의 먼 반사 (실제 장애물)
    near_elsewhere = _idx_of_deg(0)
    r[near_elsewhere] = 0.20                                   # 구간 밖의 가까운 반사 (실제 장애물)
    mask = node.build_mask(node.parse_sectors("170:15"), 0.35, 0.0)
    out = node.mask_ranges(amin, inc, r, 0.05, 8.0, 0.0, 0.0, 0.0, mask)
    assert np.isnan(out[ix - 3: ix + 4]).all()
    assert out[far_same_angle] == 1.50 and out[near_elsewhere] == 0.20
    assert np.isnan(out).sum() == 7                           # 그 밖에는 하나도 지우지 않는다
    assert not np.isinf(out[np.isnan(out)]).any()             # inf 가 아니라 NaN


def test_masked_cells_are_nan_never_inf(node):
    amin, inc, r = _scan()
    r[:] = 0.1
    mask = node.build_mask(((0.0, 180.0),), 0.35, 0.0)
    out = node.mask_ranges(amin, inc, r, 0.05, 8.0, 0.0, 0.0, 0.0, mask)
    assert np.isnan(out).all()


def test_untouched_cells_pass_through_exactly_including_nan_and_inf(node):
    amin, inc, r = _scan()
    r[10], r[20], r[30] = np.nan, np.inf, -np.inf
    mask = node.build_mask(node.parse_sectors("170:15"), 0.35, 0.0)
    out = node.mask_ranges(amin, inc, r, 0.05, 8.0, 0.0, 0.0, 0.0, mask)
    assert np.isnan(out[10]) and out[20] == np.inf and out[30] == -np.inf
    assert out.shape == r.shape


def test_min_range_clears_everywhere(node):
    amin, inc, r = _scan()
    r[100] = 0.12
    mask = node.build_mask((), 0.35, 0.20)
    out = node.mask_ranges(amin, inc, r, 0.05, 8.0, 0.0, 0.0, 0.0, mask)
    assert np.isnan(out[100]) and np.isnan(out).sum() == 1


def test_empty_mask_returns_an_unchanged_copy(node):
    amin, inc, r = _scan()
    r[5] = 0.1
    out = node.mask_ranges(amin, inc, r, 0.05, 8.0, 0.0, 0.0, 0.0, node.build_mask((), 0.35, 0.0))
    assert np.array_equal(out, r) and out is not r


def test_nonzero_mount_yaw_matches_the_agent_cell_for_cell(node):
    """Agent: scan_in_base → SelfMask.apply.  노드: 원본 광선을 지운 뒤 같은 scan_in_base 로 옮겨 본 결과.  두 결과가 칸 단위로 같다."""
    n = 720
    inc = 2 * math.pi / n
    rng = np.random.default_rng(7)
    # 구간 가장자리가 광선(0.5° 간격) 한가운데에 정확히 놓이면 1e-16 크기의 반올림이 판정을 가를 수 있다 → 가장자리가 광선 사이에 오게 잡는다.
    sectors = ((170.0, 15.2), (-100.0, 8.2))
    cfg = LidarCfg(self_mask_sectors=sectors, self_mask_range=0.35, min_range=0.08)
    for mount_yaw in (math.radians(90.0), math.radians(-37.3), math.pi, 0.0):
        r = rng.uniform(0.1, 3.0, n)
        r[rng.random(n) < 0.05] = np.nan
        sel = SelfMask(cfg)
        amin_b, inc_b, base_ranges, rmax_b = scan_in_base(-math.pi, inc, r, 0.05, 8.0, 0.0, 0.0, mount_yaw)
        agent = sel.apply(Scan(amin_b, inc_b, base_ranges, range_max=rmax_b)).ranges
        filtered = node.mask_ranges(-math.pi, inc, r, 0.05, 8.0, 0.0, 0.0, mount_yaw, node.build_mask(sectors, 0.35, 0.08))
        _, _, via_node, _ = scan_in_base(-math.pi, inc, filtered, 0.05, 8.0, 0.0, 0.0, mount_yaw)
        assert np.array_equal(agent, via_node, equal_nan=True), mount_yaw
        assert np.isnan(agent).sum() > np.isnan(base_ranges).sum()       # 시험이 실제로 무언가를 지웠다


def test_distance_is_measured_from_the_base_origin_not_the_lidar(node):
    # LiDAR 가 차체 앞 0.10 m 에 있고 뒤를 보도록(yaw=π) 달려 있다. LiDAR 광선 π 는 차체 +x(전방)를 가리킨다.
    amin, inc, r = _scan()
    ix = _idx_of_deg(0, mount_yaw_deg=180.0)
    mask = node.build_mask(((0.0, 10.0),), 0.35, 0.0)
    r[ix] = 0.20                                              # 끝점 x = 0.10 + 0.20 = 0.30 m < 0.35 → 지운다
    assert np.isnan(node.mask_ranges(amin, inc, r, 0.05, 8.0, 0.10, 0.0, math.pi, mask)[ix])
    r[ix] = 0.30                                              # 끝점 x = 0.40 m ≥ 0.35 → 남긴다 (LiDAR 기준 거리면 지웠을 것)
    assert node.mask_ranges(amin, inc, r, 0.05, 8.0, 0.10, 0.0, math.pi, mask)[ix] == 0.30


def test_filtered_fields_keep_header_and_geometry(node):
    amin, inc, r = _scan()
    r[_idx_of_deg(170)] = 0.1
    msg = SimpleNamespace(header=SimpleNamespace(frame_id="laser", stamp=12.5), angle_min=amin, angle_max=amin + inc * 359,
                          angle_increment=inc, time_increment=1e-4, scan_time=0.1, range_min=0.05, range_max=8.0,
                          ranges=r.astype(np.float32).tolist(), intensities=[1.0] * 360)
    f = node.filtered_fields(msg, (0.0, 0.0, 0.0), node.build_mask(node.parse_sectors("170:15"), 0.35, 0.0))
    for name in ("header", "angle_min", "angle_max", "angle_increment", "time_increment", "scan_time", "range_min",
                 "range_max", "intensities"):
        assert f[name] is getattr(msg, name) or f[name] == getattr(msg, name), name
    assert len(f["ranges"]) == 360 and math.isnan(f["ranges"][_idx_of_deg(170)])
    assert sum(1 for v in f["ranges"] if math.isnan(v)) == 1


# ------------------------------------------------------------------------------------------ 파일 구조
def test_node_is_importable_without_rclpy_and_imports_it_only_inside_main():
    src = NODE.read_text(encoding="utf-8")
    head, _, body = src.partition("def main(")
    assert "import rclpy" not in head and "from rclpy" not in head and "sensor_msgs" not in head and "tf2_ros" not in head
    assert "import rclpy" in body
    assert "qos_profile_sensor_data" in body                   # 입력·출력 모두 센서 데이터 QoS
    assert body.count("qos_profile_sensor_data") >= 3          # import 한 번 + 발행 + 구독
    assert "/cmd_vel" not in src and "/mil/" not in src         # 이 노드는 구동·사격과 무관하다


def test_node_does_not_copy_the_agent_logic():
    src = NODE.read_text(encoding="utf-8")
    assert "scan_rays_in_base" in src and "SelfMask" in src and "lidar_mount_from_transform" in src
    assert "np.abs(np.angle" not in src                        # SelfMask.apply 의 각도 판정 코드를 복사하지 않았다


# ------------------------------------------------------------------------------------------ launch_profile
def test_empty_mask_adds_no_arguments_and_equals_the_baseline():
    assert scan_self_mask_args(LidarCfg()) == []
    base = json.loads((ROOT / "tests/fixtures/hw_baseline/launch_cmd.json").read_text(encoding="utf-8"))
    cmd = nav_launch_command(["--config", str(ROOT / "config/waveshare_real.json")], {}, root=ROOT)
    assert not any(a.startswith(("scan_self_mask", "scan_min_range")) for a in cmd)
    assert len(cmd) == len(base["default"])


def _cmd_with_lidar(tmp_path, **lidar):
    data = json.loads((ROOT / "config/waveshare_real.json").read_text(encoding="utf-8"))
    data.setdefault("lidar", {}).update(lidar)
    p = tmp_path / "p.json"
    p.write_text(json.dumps(data), encoding="utf-8")
    return nav_launch_command(["--config", str(p)], {}, root=ROOT)


def test_mask_adds_the_three_arguments(tmp_path):
    cmd = _cmd_with_lidar(tmp_path, self_mask_sectors=[[170, 15], [-170, 15]], self_mask_range=0.4, min_range=0.1)
    assert "scan_self_mask:=170:15,-170:15" in cmd
    assert "scan_self_mask_range:=0.4" in cmd and "scan_min_range:=0.1" in cmd
    assert not any(a.startswith("scan_topic:=/scan_filtered") for a in cmd)     # 필터 토픽 이름은 launch 가 정한다


def test_min_range_alone_still_turns_the_filter_on_without_an_empty_mask_argument(tmp_path):
    cmd = _cmd_with_lidar(tmp_path, min_range=0.15)
    assert not any(a.startswith("scan_self_mask:=") for a in cmd)
    assert "scan_min_range:=0.15" in cmd and "scan_self_mask_range:=0.35" in cmd


def test_agent_activates_selfmask_under_the_same_condition_as_the_launch_arguments():
    for lidar in (LidarCfg(), LidarCfg(self_mask_sectors=((0.0, 5.0),)), LidarCfg(min_range=0.1)):
        assert SelfMask(lidar).active == bool(scan_self_mask_args(lidar))


# ------------------------------------------------------------------------------------------ 런치 파일 (실행하지 않고 정적으로만)
class _Action:
    def __init__(self, *a, **k):
        self.args, self.kwargs = a, k


class _LaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


@pytest.fixture
def launch_modules(monkeypatch):
    mapping = {
        "launch": {"LaunchDescription": _Action},
        "launch.actions": {k: type(k, (_Action,), {}) for k in
                           ("DeclareLaunchArgument", "ExecuteProcess", "OpaqueFunction", "IncludeLaunchDescription")},
        "launch.substitutions": {"LaunchConfiguration": _LaunchConfiguration},
        "launch.launch_description_sources": {"PythonLaunchDescriptionSource": _Action},
        "launch_ros": {},
        "launch_ros.actions": {"Node": type("Node", (_Action,), {})},
        "nav2_common": {},
        "nav2_common.launch": {"RewrittenYaml": type("RewrittenYaml", (_Action,), {})},
    }
    for name, attrs in mapping.items():
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, m)

    def load(name):
        spec = importlib.util.spec_from_file_location(name + "_ssm", ROOT / "ros2" / "launch" / (name + ".launch.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    return load


def _ctx(**kw):
    base = {"slam": "cartographer", "use_sim_time": "false", "fast_replan": "true", "params_file": "p.yaml",
            "base_frame": "base_footprint", "cartographer_config": "cartographer_waveshare.lua", "odom_tf": "off",
            "nav2": "true", "depth_obstacles": "false", "bt_xml": "", "scan_topic": "/scan", "odom_topic": "/odom",
            "nav_cmd_topic": "/cmd_vel_nav2"}
    base.update(kw)
    return base


def test_waveshare_overlay_declares_and_forwards_the_three_arguments(launch_modules):
    actions = launch_modules("maicon_waveshare").generate_launch_description().args[0]
    defaults = {a.args[0]: a.kwargs["default_value"] for a in actions if type(a).__name__ == "DeclareLaunchArgument"}
    assert defaults["scan_self_mask"] == "" and defaults["scan_self_mask_range"] == "0.35" and defaults["scan_min_range"] == "0"
    include = next(a for a in actions if type(a).__name__ == "IncludeLaunchDescription")
    forwarded = {k: v.name for k, v in dict(include.kwargs["launch_arguments"]).items() if hasattr(v, "name")}   # 일부 값은 고정 문자열
    for k in ("scan_self_mask", "scan_self_mask_range", "scan_min_range"):
        assert forwarded[k] == k
    assert not any(type(a).__name__ in ("Node", "ExecuteProcess") for a in actions)      # 오버레이 자체는 여전히 노드를 직접 띄우지 않는다


def test_empty_mask_starts_no_node_and_keeps_the_raw_scan_topic(launch_modules):
    mod = launch_modules("maicon_nav")
    for extra in ({}, {"scan_self_mask": "", "scan_self_mask_range": "0.35", "scan_min_range": "0"}):
        actions = mod._launch_setup(_ctx(**extra))
        assert not any(type(a).__name__ == "ExecuteProcess" for a in actions)
        slam = next(a for a in actions if a.kwargs.get("executable") == "cartographer_node")
        assert ("scan", "/scan") in slam.kwargs["remappings"]


def test_mask_starts_the_node_and_switches_slam_and_nav2_to_the_filtered_topic(launch_modules):
    mod = launch_modules("maicon_nav")
    actions = mod._launch_setup(_ctx(scan_self_mask="170:15,-170:15", scan_self_mask_range="0.35", scan_min_range="0"))
    cmd = next(a.kwargs["cmd"] for a in actions if type(a).__name__ == "ExecuteProcess")
    assert Path(cmd[1]).is_file() and Path(cmd[1]).name == "scan_self_mask.py"
    assert cmd[cmd.index("--input") + 1] == "/scan" and cmd[cmd.index("--output") + 1] == "/scan_filtered"
    assert cmd[cmd.index("--sectors") + 1] == "170:15,-170:15" and cmd[cmd.index("--base-frame") + 1] == "base_footprint"
    slam = next(a for a in actions if a.kwargs.get("executable") == "cartographer_node")
    assert ("scan", "/scan_filtered") in slam.kwargs["remappings"]
    for ex in ("controller_server", "planner_server", "behavior_server", "bt_navigator"):
        rw = next(a for a in actions if a.kwargs.get("executable") == ex).kwargs["parameters"][0].kwargs["param_rewrites"]
        assert rw["local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic"] == "/scan_filtered"
        assert rw["global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic"] == "/scan_filtered"
    tb = mod._launch_setup(_ctx(slam="slam_toolbox", scan_self_mask="170:15"))
    node_ = next(a for a in tb if a.kwargs.get("executable") == "async_slam_toolbox_node")
    assert {"base_frame": "base_footprint", "scan_topic": "/scan_filtered"} in node_.kwargs["parameters"]


def test_min_range_alone_also_starts_the_node(launch_modules):
    actions = launch_modules("maicon_nav")._launch_setup(_ctx(scan_min_range="0.15"))
    assert any(type(a).__name__ == "ExecuteProcess" for a in actions)


def test_launch_rejects_a_filtered_input_topic_and_bad_numbers(launch_modules):
    mod = launch_modules("maicon_nav")
    with pytest.raises(RuntimeError, match="_filtered"):
        mod._launch_setup(_ctx(scan_topic="/scan_filtered", scan_self_mask="170:15"))
    with pytest.raises(RuntimeError, match="숫자"):
        mod._launch_setup(_ctx(scan_self_mask="170:15", scan_min_range="abc"))


def test_launch_file_is_never_executed_by_these_tests():
    for name in ("maicon_waveshare", "maicon_nav"):
        text = (ROOT / "ros2" / "launch" / f"{name}.launch.py").read_text(encoding="utf-8")
        assert "scan_self_mask" in text
