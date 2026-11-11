"""실기 설정에서 OAK 깊이(피아식별·거리 측정용)의 시작 판정: 보정을 확인하지 않았으면 깊이를 끄고 OAK 드라이버도 저전력 프로파일로 띄운다.
확인했으면 깊이를 켠 채 rgbd 프로파일로 띄운다. 어느 쪽이든 구동은 깊이에 묶이지 않는다. ROS·시뮬 없이 설정만 본다."""
import logging
from pathlib import Path

from maicon26.config import Config
from maicon26.launch_profile import load_run_config, oak_launch_spec
from maicon26.preflight import validate_real_config

ROOT = Path(__file__).resolve().parents[1]
REAL = ROOT / "config" / "waveshare_real.json"


def _cfg():
    cfg, _ = load_run_config(str(REAL), None, "nav2", root=ROOT)
    return cfg


def test_unverified_depth_is_turned_off_at_start_and_ranging_falls_back_to_the_old_order(caplog):
    cfg = _cfg()
    assert cfg.depth.enable and cfg.oak.profile == "rgbd"
    with caplog.at_level(logging.WARNING):
        validate_real_config(cfg)
    assert cfg.depth.enable is False and cfg.oak.profile == "rgb_lowpower"
    assert "depth.enable=False" in caplog.text
    assert cfg.oak_det.enable and cfg.oak_det.ugv_only and cfg.oak_det.depth_range      # 검출·UGV 추적은 그대로. 깊이 프레임이 없으면 크기·LiDAR 순서로 잰다
    assert cfg.map3d.enable is False and cfg.nav2.publish_depth_obstacles is False


def test_verified_depth_stays_on_with_the_rgbd_profile_and_never_feeds_navigation():
    cfg = _cfg()
    cfg.depth.calibration_verified, cfg.depth.optical_frame = True, "oak_rgb_camera_optical_frame"
    validate_real_config(cfg)
    assert cfg.depth.enable and cfg.depth.require_calibration and cfg.depth.gate_drive is False
    assert cfg.oak.profile == "rgbd"
    assert not cfg.map3d.use_for_nav and not cfg.map3d.guard_enable and not cfg.nav2.publish_depth_obstacles and not cfg.depth.nav_allowed


def test_oak_driver_launch_uses_the_same_profile_the_runner_will_use():
    spec = oak_launch_spec([], {"MAICON_OAK": "1"}, root=ROOT)
    cmd, _ = spec
    assert cmd[-1].endswith("rgb_lowpower.yaml")                    # 보정 미확인 → 깊이 꺼짐 → 스테레오 연산을 아낀다
    assert oak_launch_spec([], {}, root=ROOT) is None               # MAICON_OAK 없으면 드라이버를 띄우지 않는다


def test_code_defaults_are_untouched_by_the_real_depth_policy():
    d = Config()
    assert d.depth.gate_drive is True and d.oak.profile == "rgb_lowpower"
