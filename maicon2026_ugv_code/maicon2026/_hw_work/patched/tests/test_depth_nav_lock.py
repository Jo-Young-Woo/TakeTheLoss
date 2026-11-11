"""실기에서는 OAK 깊이를 장애물 항법에 쓰지 않는다(preflight.lock_depth_navigation). 2D LiDAR 가 항법 장애물 입력의 전부다. 시뮬을 쓰지 않는다."""
from pathlib import Path

import pytest

from maicon26.config import Config
from maicon26.launch_profile import nav_launch_command
from maicon26.preflight import lock_depth_navigation, validate_real_config

ROOT = Path(__file__).resolve().parents[1]


def real():
    cfg = Config.load(str(ROOT / "config/waveshare_real.json"))
    cfg.depth.enable, cfg.depth.calibration_verified, cfg.depth.optical_frame = True, True, "depth_optical"
    cfg.map3d.enable = cfg.map3d.use_for_nav = cfg.map3d.guard_enable = True
    cfg.nav2.publish_depth_obstacles = True
    return cfg


def test_real_start_turns_off_every_depth_navigation_switch_but_keeps_depth_ranging(caplog):
    cfg = real()
    with caplog.at_level("WARNING"):
        validate_real_config(cfg)
    assert not cfg.map3d.use_for_nav and not cfg.map3d.guard_enable and not cfg.nav2.publish_depth_obstacles
    assert cfg.depth.enable and cfg.map3d.enable and cfg.map3d.use_range         # 거리 측정·발사 전 3D 확인은 항법이 아니므로 그대로
    assert "depth.nav_allowed" in caplog.text


def test_nav_allowed_true_is_the_only_way_to_keep_depth_in_navigation():
    cfg = real()
    cfg.depth.nav_allowed = True
    validate_real_config(cfg)
    assert cfg.map3d.use_for_nav and cfg.map3d.guard_enable and cfg.nav2.publish_depth_obstacles


def test_lock_reports_what_it_changed_and_is_idempotent():
    cfg = real()
    assert sorted(lock_depth_navigation(cfg)) == ["map3d.guard_enable", "map3d.use_for_nav", "nav2.publish_depth_obstacles"]
    assert lock_depth_navigation(cfg) == []


def test_lock_never_raises_on_odd_configs():
    class Frozen:
        def __init__(self):
            object.__setattr__(self, "use_for_nav", True)

        def __setattr__(self, k, v):
            raise AttributeError("frozen")

    cfg = real()
    cfg.map3d = Frozen()
    assert lock_depth_navigation(cfg) == ["nav2.publish_depth_obstacles"]          # 못 바꾸는 항목은 건너뛰고 예외 없이 지나간다


def test_oak_rgbd_profile_falls_back_to_rgb_lowpower_when_depth_is_off(caplog):
    cfg = Config.load(str(ROOT / "config/waveshare_real.json"))
    cfg.oak.profile = "rgbd"
    with caplog.at_level("WARNING"):
        validate_real_config(cfg)
    assert cfg.oak.profile == "rgb_lowpower" and "rgbd" in caplog.text
    cfg = real()
    cfg.oak.profile = "rgbd"
    validate_real_config(cfg)
    assert cfg.oak.profile == "rgbd"                                              # 깊이를 거리 측정에 쓰면 rgbd 유지


def test_default_real_config_has_no_depth_obstacles_in_the_nav_launch():
    assert "depth_obstacles:=false" in nav_launch_command([], {})


def test_sim_defaults_are_not_touched_by_the_real_lock():
    cfg = Config()
    assert cfg.depth.nav_allowed is False
    assert cfg.map3d.use_for_nav is True and cfg.map3d.guard_enable is True       # 시뮬 기본값은 그대로(실기 시작 때만 잠근다)
