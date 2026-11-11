import json
from pathlib import Path

import pytest

from maicon26.launch_profile import load_run_config, nav_launch_command

ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "config/waveshare_real.json"


def profile(tmp_path, **sections):
    data = json.loads(PROFILE.read_text(encoding="utf-8"))
    for key, value in sections.items():
        data.setdefault(key, {}).update(value)
    p = tmp_path / "profile.json"
    p.write_text(json.dumps(data), encoding="utf-8")
    return p


def test_default_child_matches_hardware_agent():
    cmd = nav_launch_command([], {})
    assert "base_frame:=base_footprint" in cmd
    assert "depth_obstacles:=false" in cmd
    assert "nav_cmd_topic:=/cmd_vel_nav2" in cmd
    assert not any("ugv_bringup" in arg for arg in cmd)


def test_cli_and_overrides_select_same_topics_for_child_and_agent(tmp_path):
    p = profile(tmp_path, hal={"scan_topic": "/filtered_scan", "odom_topic": "/filtered_odom"})
    ov = tmp_path / "overrides.json"
    ov.write_text(json.dumps({"values": {"nav2.cmd_topic": "/team_nav_cmd"}}))
    argv = ["--config", str(PROFILE), "--config", str(p), "--overrides", str(ov), "--nav", "nav2"]
    cmd = nav_launch_command(argv, {})
    cfg, applied = load_run_config(p, ov)
    assert "scan_topic:=" + cfg.hal.scan_topic in cmd
    assert "odom_topic:=" + cfg.hal.odom_topic in cmd
    assert "nav_cmd_topic:=" + cfg.nav2.cmd_topic in cmd
    assert "nav2.cmd_topic" in applied


def test_builtin_and_help_do_not_start_an_unused_nav2():
    assert nav_launch_command(["--nav", "builtin"], {}) is None
    assert nav_launch_command(["--help"], {}) is None


def test_calibrated_depth_enabled_in_both_layers_only_when_complete(tmp_path):
    # 실기에서 OAK 깊이는 장애물 항법에 쓰지 않는다(2D LiDAR 만). depth.nav_allowed=true 로 명시했을 때만 두 층이 함께 켜진다.
    p = profile(tmp_path, depth={"enable": True, "calibration_verified": True, "optical_frame": "depth_optical", "nav_allowed": True},
                map3d={"enable": True}, nav2={"publish_depth_obstacles": True})
    assert "depth_obstacles:=true" in nav_launch_command(["--config", str(p)], {})
    p = profile(tmp_path, depth={"enable": True, "calibration_verified": True, "optical_frame": "depth_optical"},
                map3d={"enable": True}, nav2={"publish_depth_obstacles": True})
    assert "depth_obstacles:=false" in nav_launch_command(["--config", str(p)], {})
    # 규칙 변경(프롬프트 1장-6): 깊이 설정이 덜 됐다고 ValueError 로 시작을 막지 않는다. 경고 후 depth.enable=False 로 바꾸므로
    # Nav2 쪽에도 깊이 장애물이 켜지지 않는다(두 층 모두 꺼짐) — '완성됐을 때만 켜진다'는 이 시험의 요점은 그대로다.
    p = profile(tmp_path, depth={"enable": True}, map3d={"enable": True}, nav2={"publish_depth_obstacles": True})
    assert "depth_obstacles:=false" in nav_launch_command(["--config", str(p)], {})


def test_custom_map_graph_is_not_silently_launched_with_default_names(tmp_path):
    p = profile(tmp_path, nav2={"map_topic": "/other_map"})
    with pytest.raises(ValueError, match="지도/액션"):
        nav_launch_command(["--config", str(p)], {})


def test_params_path_with_spaces_is_one_argument_and_never_shell_code(tmp_path):
    p = tmp_path / "nav params.yaml"
    p.write_text("{}")
    cmd = nav_launch_command([], {"MAICON_NAV_PARAMS": str(p), "MAICON_SLAM": "slam_toolbox"})
    assert "params_file:=" + str(p.resolve()) in cmd
    assert "slam:=slam_toolbox" in cmd
    with pytest.raises(FileNotFoundError):
        nav_launch_command([], {"MAICON_NAV_BT": str(tmp_path / "missing.xml")})
    with pytest.raises(ValueError, match="MAICON_SLAM"):
        nav_launch_command([], {"MAICON_SLAM": "unknown"})
