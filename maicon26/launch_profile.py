"""Use the same runtime profile for the Agent and its managed SLAM/Nav2 child."""
from __future__ import annotations

import argparse
from pathlib import Path

from .config import Config
from .preflight import validate_real_config

ROOT = Path(__file__).resolve().parents[1]


def load_run_config(path, overrides=None, nav="nav2", *, root=ROOT):
    cfg = Config.load(str(Path(path).expanduser()))
    applied = []
    if overrides:
        from .field import overrides as ov
        p = Path(overrides).expanduser()
        if not p.is_absolute():
            p = Path(root) / p
        applied = ov.apply_to_config(cfg, ov.load(str(p)))
    cfg.nav.backend = nav
    return cfg, applied


def nav_launch_command(argv, env, *, root=ROOT):
    """Build argv without shell interpolation; CLI settings take precedence.

    Only parses shared profile flags. The real entry point validates the full
    mission command line. Returning None means no managed ROS child is needed.
    """
    if "--help" in argv or "-h" in argv:
        return None
    root = Path(root)
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--config", default=str(root / "config/waveshare_real.json"))
    parser.add_argument("--overrides", nargs="?", const="config/field_overrides.json")
    parser.add_argument("--nav", choices=("nav2", "builtin"), default="nav2")
    args, _ = parser.parse_known_args(argv)
    cfg, _ = load_run_config(args.config, args.overrides, args.nav, root=root)
    validate_real_config(cfg)
    if cfg.nav.backend != "nav2":
        return None
    # These endpoints belong to the current overlay. Reject silent mismatch;
    # custom graph layouts must be launched explicitly outside team_node.
    n = cfg.nav2
    if (n.map_frame != "map" or n.map_topic != "/map" or
            n.action_name not in ("navigate_to_pose", "/navigate_to_pose") or
            n.walls_topic != "/arena_walls_cloud" or
            n.depth_obstacles_topic != "/depth_obstacles_cloud"):
        raise ValueError("자동 launch가 지원하지 않는 지도/액션/점군 이름입니다. 별도 launch와 설정을 일치시키세요")
    slam = env.get("MAICON_SLAM", "cartographer")
    if slam not in ("cartographer", "slam_toolbox", "none"):
        raise ValueError("MAICON_SLAM은 cartographer | slam_toolbox | none")
    depth = cfg.depth.enable and cfg.map3d.enable and cfg.map3d.use_for_nav and n.publish_depth_obstacles
    command = ["ros2", "launch", str(root / "ros2/launch/maicon_waveshare.launch.py"),
               "slam:=" + slam, "base_frame:=" + n.base_frame,
               "scan_topic:=" + cfg.hal.scan_topic, "odom_topic:=" + cfg.hal.odom_topic,
               "nav_cmd_topic:=" + n.cmd_topic, "depth_obstacles:=" + str(bool(depth)).lower()]
    for key, arg in (("MAICON_NAV_PARAMS", "params_file"), ("MAICON_NAV_BT", "bt_xml")):
        value = env.get(key)
        if value:
            p = Path(value).expanduser()
            if not p.is_absolute():
                p = root / p
            if not p.is_file():
                raise FileNotFoundError(f"{key} 파일이 없습니다: {p}")
            command.append(arg + ":=" + str(p.resolve()))
    return command
