"""10장 설정 키: 새 키가 기본값·로드·병합·알 수 없는 키 거부·실기 설정 불변·launch 인자에서 문서대로 동작하는지. 시뮬을 쓰지 않는다."""
import json
from pathlib import Path

import pytest

from maicon26.config import Config
from maicon26.launch_profile import load_run_config, scan_self_mask_args
from maicon26.oak_launch import oak_enabled

ROOT = Path(__file__).resolve().parents[1]

# (섹션, 키, 문서의 기본값) — 10장 표 그대로
DEFAULTS = [
    ("camera", "model", "pinhole"), ("camera", "D", None), ("camera", "usable_hfov_deg", None),
    ("aim", "gimbal_settle_s", 0.0), ("aim", "settle_move_deg", 1.0),
    ("laser", "practice_empty_backoff_s", 10.0),
    ("oak", "launch_cmd", ("ros2", "launch", "depthai_ros_driver", "camera.launch.py")), ("oak", "launch_args", ()),
    ("oak", "profile", "rgb_lowpower"),
    ("oak", "rgb_topic", "/oak/rgb/image_raw"), ("oak", "rgb_info_topic", "/oak/rgb/camera_info"),
    ("oak", "check_topics_after_s", 8.0),
    ("oak_det", "enable", False), ("oak_det", "conf", 0.35), ("oak_det", "max_hz", 3.0), ("oak_det", "pause_in_strike", True),
    ("oak_det", "weight", 0.5), ("oak_det", "submit_alone", False), ("oak_det", "save_max", 8),
    ("oak_det", "save_every_s", 2.0), ("oak_det", "cleanup_every_s", 30.0), ("oak_det", "save_dir", None),
    ("rig", "oak_rgb_rpy", (0.0, 0.0, 0.0)),
    ("hal", "fire_left_ms_max", 15000.0), ("hal", "frozen_frames", 20), ("hal", "dark_frames", 30),
    ("perception", "min_box_px", 4.0),
    ("link", "max_command_chars", 500),
    # LiDAR 면 지도·OAK 식별 단계·깊이 항법 잠금(코드 기본은 모두 꺼짐 — 실기 설정 파일에서만 켠다)
    ("explore", "uav_face_registry", False), ("explore", "redetect_resolved", True), ("avoid", "friend_track", False), ("avoid", "friend_lost_s", 4.0),
    ("avoid", "friend_gate_m", 0.5), ("explore", "lidar_faces", False), ("explore", "lidar_face_min_len_m", 0.40), ("explore", "lidar_ugv_exclude_m", 0.30),
    ("oak_det", "on_demand", False), ("oak_det", "identify_fallback", False), ("oak_det", "identify_submit", False),
    ("oak_det", "identify_dist_m", 0.8), ("depth", "nav_allowed", False),
    # OAK(Depth 카메라) 우선 식별 + 전원 제어(코드 기본은 모두 꺼짐 — 실기 설정도 일단 꺼 둔다)
    ("oak_det", "identify_primary", False), ("oak_det", "identify_edge_margin_px", 8), ("oak_det", "identify_crop_frames", 2),
    ("oak_det", "identify_fail_limit", 2), ("oak_det", "identify_noview_s", 6.0), ("oak_det", "fit_hfov_deg", 69.0),
    ("oak_det", "fit_vfov_deg", 42.0), ("oak_det", "fit_usable", 0.92),
    ("oak", "power_gating", False), ("oak", "start_srv", "/oak/start_camera"), ("oak", "stop_srv", "/oak/stop_camera"),
    ("oak", "wake_dist_m", 2.0), ("oak", "off_delay_s", 4.0), ("oak", "start_timeout_s", 8.0), ("oak", "power_retry_s", 2.0),
    # 충돌 대응·지도 연결·OAK 깊이 거리/피아식별·UAV 지도 우선순위(코드 기본은 모두 꺼짐 — 실기 설정 파일에서만 켠다)
    ("oak_det", "ugv_only", False), ("oak_det", "depth_range", False), ("oak_det", "depth_max_dt_s", 0.35), ("oak_det", "iff_max_range_m", 2.2),
    ("depth", "gate_drive", True), ("hal", "imu_mount_yaw_deg", 0.0),
    ("impact", "enable", False), ("impact", "trigger_dg", 0.45), ("impact", "backoff", True),
    ("pose_guard", "enable", False), ("pose_guard", "link_check", True), ("pose_guard", "link_trim", False), ("pose_guard", "fallback_deg", 5.0),
    ("pose_guard", "reanchor", False), ("pose_guard", "reanchor_use_uav_boxes", False), ("pose_guard", "reanchor_max_m", 0.6),
    ("pose_guard", "reanchor_confirm_n", 3),
    ("explore", "m1_priority_enemy_first", False), ("explore", "uav_census", False), ("explore", "m1_cm_defer", 0.3),
]

# 문서의 기본이 '다른 키와 같다/창 없음' 으로 적힌 키
SPECIAL = [("oak_det", "vote_window", None), ("rig", "oak_rgb_xyz", None)]


@pytest.mark.parametrize("section,key,default", DEFAULTS)
def test_every_new_key_exists_with_the_documented_default(section, key, default):
    assert getattr(getattr(Config(), section), key) == default


def test_keys_documented_as_following_another_default_do_follow_it():
    cfg = Config()
    assert cfg.oak_det.vote_window is None                                  # 창 없음 = 팬틸트 쪽과 같은 규칙(perception.min_votes 만)
    assert cfg.rig.oak_rgb_xyz == cfg.rig.depth_xyz                         # rig.depth_xyz 와 같다


NON_DEFAULT = {
    "camera": {"model": "fisheye", "D": [0.1, -0.02, 0.0, 0.0], "usable_hfov_deg": 120.0,
               "K": [300.0, 0.0, 320.0, 0.0, 300.0, 240.0, 0.0, 0.0, 1.0]},
    "aim": {"gimbal_settle_s": 0.4, "settle_move_deg": 2.5},
    "laser": {"practice_empty_backoff_s": 4.0},
    "oak": {"launch_cmd": ["ros2", "run", "x", "y"], "launch_args": ["camera_model:=OAK-D-LITE"], "profile": "rgbd",
            "rgb_topic": "/cam/rgb", "rgb_info_topic": "/cam/info", "check_topics_after_s": 3.0},
    "oak_det": {"enable": True, "conf": 0.5, "max_hz": 2.0, "pause_in_strike": False, "vote_window": 8, "weight": 0.7,
                "submit_alone": True, "save_max": 4, "save_every_s": 1.0, "cleanup_every_s": 10.0, "save_dir": "/tmp/x"},
    "rig": {"oak_rgb_xyz": [0.07, 0.0, 0.03], "oak_rgb_rpy": [0.0, 0.1, 0.0]},
    "hal": {"fire_left_ms_max": 12000.0, "frozen_frames": 10, "dark_frames": 15},
    "perception": {"min_box_px": 6.0},
    "link": {"max_command_chars": 300},
}


def test_every_new_key_can_be_loaded_from_a_config_file_and_lists_become_tuples(tmp_path):
    p = tmp_path / "c.json"
    p.write_text(json.dumps(NON_DEFAULT), encoding="utf-8")
    cfg = Config.load(str(p))
    for section, keys in NON_DEFAULT.items():
        for key, want in keys.items():
            got = getattr(getattr(cfg, section), key)
            got = tuple(got) if isinstance(got, list) else got               # 기본이 None 인 Optional 키(camera.D·K)는 로더가 리스트를 그대로 둔다 — 쓰는 쪽이 tuple 로 바꾼다
            assert got == (tuple(want) if isinstance(want, list) else want), (section, key)
    assert isinstance(cfg.oak.launch_args, tuple) and isinstance(cfg.rig.oak_rgb_xyz, tuple)     # 기본이 tuple 인 키는 tuple 로 바뀐다
    from maicon26.perception.camera import CameraModel
    cm = CameraModel(cfg.camera)                                            # 파일에서 읽은 K·D(리스트)로도 어안 모델이 선다
    assert cm.model == "fisheye" and cm.D == (0.1, -0.02, 0.0, 0.0) and cm.usable_hfov_deg == 120.0


def test_new_keys_merge_over_the_real_config_and_leave_the_rest_alone(tmp_path):
    ov = tmp_path / "ov.json"
    ov.write_text(json.dumps({"oak_det": {"enable": True, "max_hz": 2.0}, "aim": {"gimbal_settle_s": 0.3}}), encoding="utf-8")
    base, _ = load_run_config(str(ROOT / "config" / "waveshare_real.json"), None, "nav2", root=ROOT)
    merged = Config.load(str(ov))
    assert merged.oak_det.enable and merged.oak_det.max_hz == 2.0 and merged.aim.gimbal_settle_s == 0.3
    assert merged.oak_det.conf == base.oak_det.conf and merged.laser.ammo == base.laser.ammo      # 건드리지 않은 값


def test_impact_pose_guard_depth_and_census_keys_load_and_round_trip(tmp_path):
    raw = {"impact": {"enable": True, "trigger_dg": 0.6}, "pose_guard": {"enable": True, "fallback_deg": 4.0, "link_trim": True},
           "depth": {"gate_drive": False}, "oak_det": {"enable": True, "ugv_only": True, "depth_range": True, "ugv_width_m": [0.1, 0.5]},
           "explore": {"m1_priority_enemy_first": True, "uav_census": True, "m1_enemy_budget_s": 90.0}, "hal": {"imu_mount_yaw_deg": 90.0}}
    p = tmp_path / "new.json"
    p.write_text(json.dumps(raw), encoding="utf-8")
    cfg = Config.load(str(p))
    assert cfg.impact.enable and cfg.impact.trigger_dg == 0.6 and cfg.pose_guard.link_trim and cfg.pose_guard.fallback_deg == 4.0
    assert cfg.depth.gate_drive is False and cfg.oak_det.ugv_only and cfg.oak_det.ugv_width_m == (0.1, 0.5)
    assert cfg.explore.m1_priority_enemy_first and cfg.explore.uav_census and cfg.explore.m1_enemy_budget_s == 90.0 and cfg.hal.imu_mount_yaw_deg == 90.0
    q = tmp_path / "rt2.json"
    q.write_text(json.dumps(cfg.to_dict(), default=list), encoding="utf-8")
    back = Config.load(str(q))
    assert back.impact == cfg.impact and back.pose_guard == cfg.pose_guard and back.oak_det.ugv_width_m == (0.1, 0.5)


def test_config_round_trips_through_to_dict_with_the_new_keys(tmp_path):
    cfg = Config.load(None)
    cfg.camera.model, cfg.oak_det.max_hz, cfg.link.max_command_chars = "fisheye", 2.5, 321
    p = tmp_path / "rt.json"
    p.write_text(json.dumps(cfg.to_dict(), default=list), encoding="utf-8")
    back = Config.load(str(p))
    assert (back.camera.model, back.oak_det.max_hz, back.link.max_command_chars) == ("fisheye", 2.5, 321)
    assert back.oak_det.save_max == cfg.oak_det.save_max and back.hal.frozen_frames == cfg.hal.frozen_frames


@pytest.mark.parametrize("section,key", [(s, k) for s, k, _ in DEFAULTS] + [(s, k) for s, k, _ in SPECIAL])
def test_unknown_keys_next_to_the_new_ones_are_rejected_not_ignored(tmp_path, section, key):
    p = tmp_path / "bad.json"
    p.write_text(json.dumps({section: {key + "_typo": 1}}), encoding="utf-8")
    with pytest.raises(KeyError, match="알 수 없는 설정 키"):
        Config.load(str(p))


def test_unknown_section_is_rejected_too(tmp_path):
    p = tmp_path / "bad.json"
    p.write_text(json.dumps({"oak_detect": {"enable": True}}), encoding="utf-8")
    with pytest.raises(KeyError):
        Config.load(str(p))


def test_the_real_robot_config_keeps_every_default_this_work_must_not_change():
    cfg, _ = load_run_config(str(ROOT / "config" / "waveshare_real.json"), None, "nav2", root=ROOT)
    assert cfg.camera.model == "pinhole" and cfg.camera.D is None and cfg.camera.usable_hfov_deg is None
    assert cfg.aim.gimbal_settle_s == 0.0
    # Depth 카메라(OAK)는 상대 UGV 거리·피아식별에만 쓴다 — 면 식별은 팬틸트가 맡는다(식별 폴백·OAK 우선·식별 제출·단독 제출·전원 제어는 꺼 둔다)
    assert cfg.oak_det.enable is True and cfg.oak_det.ugv_only is True and cfg.oak_det.depth_range is True
    assert cfg.oak_det.on_demand is False and cfg.oak_det.submit_alone is False
    assert cfg.oak_det.identify_primary is False and cfg.oak_det.identify_fallback is False and cfg.oak_det.identify_submit is False
    assert cfg.oak.power_gating is False
    default = Config()
    assert cfg.lidar.self_mask_sectors == default.lidar.self_mask_sectors      # 자기 가림 구간은 현장 측정 전까지 바꾸지 않는다
    # 실기 UAV 상자 검출기 + M1 구역 순회: 실기 설정에서만 켠다(기본 Config 는 꺼짐). 보정 파일이 없으면 검출기만 꺼진 채 시작한다
    assert cfg.uav_boxes.enable is True and cfg.uav_boxes.camera == "main"
    assert cfg.uav_boxes.calibration == "calib/uav_main_calibration.json"
    assert cfg.explore.m1_sector_sweep is True
    assert default.uav_boxes.enable is False and default.uav_boxes.calibration is None and default.explore.m1_sector_sweep is False


def test_real_config_uav_boxes_pass_the_config_sanity_check():
    from maicon26.validate import sanitize_config
    cfg, _ = load_run_config(str(ROOT / "config" / "waveshare_real.json"), None, "nav2", root=ROOT)
    assert sanitize_config(cfg) == []


def test_oak_driver_is_off_unless_the_environment_variable_says_one(monkeypatch):
    assert oak_enabled({}) is False and oak_enabled({"MAICON_OAK": ""}) is False and oak_enabled({"MAICON_OAK": "0"}) is False
    assert oak_enabled({"MAICON_OAK": "1"}) is True and oak_enabled({"MAICON_OAK": " 1 "}) is True


def test_scan_mask_launch_arguments_are_absent_by_default_and_follow_the_config():
    cfg = Config()
    assert cfg.lidar.self_mask_range == 0.35 and cfg.lidar.min_range == 0.0
    if not cfg.lidar.self_mask_sectors:
        assert scan_self_mask_args(cfg.lidar) == []                         # 빈 값 / 0 — 기존 launch 명령과 같다
    cfg.lidar.self_mask_sectors = ((3.14, 0.4),)
    cfg.lidar.min_range = 0.12
    assert scan_self_mask_args(cfg.lidar) == ["scan_self_mask:=3.14:0.4", "scan_self_mask_range:=0.35", "scan_min_range:=0.12"]
    cfg.lidar.self_mask_sectors = ()
    assert scan_self_mask_args(cfg.lidar) == ["scan_self_mask_range:=0.35", "scan_min_range:=0.12"]   # min_range 만이어도 마스크는 켜진다


def test_every_registered_tunable_is_a_real_config_key():
    from maicon26.field.tunables import TUNABLES
    cfg = Config()
    missing = []
    for t in TUNABLES:
        obj = cfg
        try:
            for part in t.path.split("."):
                obj = getattr(obj, part)
        except AttributeError:
            missing.append(t.path)
    assert missing == []
