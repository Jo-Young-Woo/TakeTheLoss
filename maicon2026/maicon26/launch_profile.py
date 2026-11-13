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


def scan_self_mask_args(lidar):
    """SLAM·Nav2 용 LiDAR 자기 가림 필터 launch 인자. Agent 의 SelfMask 가 켜질 조건(구간이 있거나 min_range > 0)과 같을 때만 붙이고,
    마스크가 비어 있으면 아무것도 붙이지 않는다(기존 launch 명령과 같다)."""
    if not lidar.self_mask_sectors and not lidar.min_range > 0.0:
        return []
    out = []
    if lidar.self_mask_sectors:                      # min_range 만 쓰는 설정이면 빈 값 인자(scan_self_mask:=)는 붙이지 않는다
        out.append("scan_self_mask:=" + ",".join(f"{c:g}:{h:g}" for c, h in lidar.self_mask_sectors))
    return out + ["scan_self_mask_range:=" + f"{lidar.self_mask_range:g}", "scan_min_range:=" + f"{lidar.min_range:g}"]


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
    command += scan_self_mask_args(cfg.lidar)
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


def oak_launch_spec(argv, env, *, root=ROOT):
    """MAICON_OAK=1 이면 (명령, 자식 환경), 아니면 None. 설정(--config/--overrides)은 nav_launch_command 와 같은 규칙으로 읽는다.
    실패(파일 없음·잘못된 oak.profile)는 호출자가 경고만 하고 넘어가도록 예외를 그대로 올린다."""
    from . import oak_launch
    if not oak_launch.oak_enabled(env):
        return None
    root = Path(root)
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--config", default=str(root / "config/waveshare_real.json"))
    parser.add_argument("--overrides", nargs="?", const="config/field_overrides.json")
    args, _ = parser.parse_known_args(argv)
    cfg, _ = load_run_config(args.config, args.overrides, "nav2", root=root)
    validate_real_config(cfg)                        # 깊이가 꺼지면(보정 미확인 등) oak.profile 도 rgb_lowpower 로 — 실행기가 보는 설정과 같은 판정
    return oak_launch.launch_command(cfg.oak, root), oak_launch.child_env(env)
