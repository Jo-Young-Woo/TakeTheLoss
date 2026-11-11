"""실기 설정(config/waveshare_real.json)의 정책: progress 사용 · IMU 헤딩 융합 · 아레나 6.12×5.00 고정 · 깊이는 항법에 쓰지 않음.
시뮬을 쓰지 않는다(설정 파일과 소스 구문만 본다)."""
import ast
import json
from pathlib import Path

import pytest

from maicon26.config import Config
from maicon26.launch_profile import load_run_config

ROOT = Path(__file__).resolve().parents[1]
REAL = ROOT / "config" / "waveshare_real.json"


def real_cfg():
    cfg, _ = load_run_config(str(REAL), None, "nav2", root=ROOT)
    return cfg


def test_progress_and_imu_heading_fusion_are_on_for_the_real_robot_but_off_in_code_defaults():
    cfg, d = real_cfg(), Config()
    assert cfg.progress.enabled is True and cfg.hal.imu_heading_fusion is True
    assert d.progress.enabled is False and d.hal.imu_heading_fusion is False        # 코드 기본값은 그대로(시뮬 A/B 대조군)


def test_arena_is_pinned_to_the_confirmed_6_12_by_5_00():
    cfg = real_cfg()
    assert (cfg.arena.W, cfg.arena.H) == (6.12, 5.00)
    assert (Config().arena.W, Config().arena.H) == (6.12, 5.00)


def test_run_real_does_not_default_to_the_6_00_by_4_80_arena_spec():
    """--arena-spec 은 명시했을 때만 쓴다. (사용자 확인: 경기장은 6.12×5.00 — spec 의 6.00×4.80 을 기본으로 적용하지 않는다.)"""
    tree = ast.parse((ROOT / "scripts" / "run_real.py").read_text(encoding="utf-8-sig"))
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, "attr", "") == "add_argument"
             and n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value == "--arena-spec"]
    assert len(calls) == 1 and not any(k.arg in ("default", "required") for k in calls[0].keywords)


def test_the_real_config_file_only_sets_known_keys_and_keeps_depth_out_of_navigation():
    cfg = real_cfg()
    raw = json.loads(REAL.read_text(encoding="utf-8"))
    assert {"progress", "arena"} <= set(raw) and raw["hal"]["imu_heading_fusion"] is True
    # 깊이(OAK)는 피아식별·상대 UGV 거리 측정에만 쓴다 — 항법 장애물·3D 지도에는 쓰지 않는다(장애물 입력은 2D LiDAR)
    assert cfg.depth.enable is True and cfg.depth.require_calibration is True and cfg.depth.gate_drive is False
    assert cfg.map3d.enable is False and cfg.nav2.publish_depth_obstacles is False and cfg.depth.nav_allowed is False
    assert cfg.depth.calibration_verified is False and cfg.depth.optical_frame == ""        # 실측값 — 운영자가 TF 를 확인한 뒤 채운다(안 채우면 시작 때 depth 가 꺼지고 거리는 LiDAR·크기로)
    assert cfg.nav.backend == "nav2"


def test_stage_policy_default_is_the_enemy_middle_door_in_the_real_config():
    cfg = real_cfg()
    assert cfg.mission.stage_policy == "enemy_middle" and cfg.mission.stage_point_override is None


def test_real_config_uses_the_oak_for_ugv_range_and_iff_only_and_code_defaults_stay_off():
    """OAK(Depth 카메라)는 상대 UGV 거리 측정·피아식별에만 쓴다(oak_det.ugv_only + depth_range). 면 식별은 계속 팬틸트가 맡는다 —
    식별 폴백·OAK 우선 식별·식별 제출·전원 제어·OAK 단독 제출은 모두 꺼 둔다."""
    cfg, d = real_cfg(), Config()
    assert cfg.explore.lidar_faces is True
    assert (cfg.oak_det.enable, cfg.oak_det.ugv_only, cfg.oak_det.depth_range) == (True, True, True)
    assert (cfg.oak_det.on_demand, cfg.oak_det.identify_fallback, cfg.oak_det.identify_submit) == (False,) * 3
    assert cfg.oak_det.identify_primary is False and cfg.oak_det.submit_alone is False and cfg.oak.power_gating is False
    assert cfg.oak.profile == "rgbd"                                    # 깊이(컬러 정렬)를 내는 프로파일. 깊이가 꺼지면 시작 때 rgb_lowpower 로 내려간다
    assert d.explore.lidar_faces is False and d.oak_det.enable is False
    assert (d.oak_det.ugv_only, d.oak_det.depth_range) == (False, False)
    assert (d.oak_det.on_demand, d.oak_det.identify_fallback, d.oak_det.identify_submit) == (False,) * 3
    assert d.oak_det.identify_primary is False and d.oak.power_gating is False
    assert d.depth.nav_allowed is False and cfg.depth.nav_allowed is False and d.depth.gate_drive is True


def d_structures_default_is_fixed():
    return Config().pose_guard.structures == "fixed"


def test_real_config_turns_on_impact_pose_guard_and_the_enemy_first_uav_census_but_code_defaults_stay_off():
    cfg, d = real_cfg(), Config()
    assert cfg.impact.enable is True and cfg.pose_guard.enable is True
    assert cfg.pose_guard.link_trim is False and cfg.pose_guard.link_check is True      # 지도 정렬을 스스로 고치는 기능은 켜지 않는다 — 점검만
    assert cfg.pose_guard.structures == "uav" and d_structures_default_is_fixed()                 # 구조물은 매 경기 달라 저장된 13개를 쓰지 않는다 — UAV 가 시작 때 찍은 상자만
    assert cfg.pose_guard.reanchor is False                                              # 앵커 재설정도 연습 시간에 도면 잔차·복구량을 확인한 뒤에 켠다
    assert (cfg.explore.m1_priority_enemy_first, cfg.explore.uav_census) == (True, True)
    assert (d.impact.enable, d.pose_guard.enable, d.pose_guard.link_trim, d.pose_guard.reanchor) == (False, False, False, False)
    assert (d.explore.m1_priority_enemy_first, d.explore.uav_census) == (False, False)
    assert d.hal.imu_mount_yaw_deg == 0.0 and cfg.hal.imu_mount_yaw_deg == 0.0           # IMU 장착 방향은 현장에서 확인할 때까지 가정


def test_real_config_turns_on_the_uav_face_registry_and_friend_tracking_but_code_defaults_stay_off():
    cfg, d = real_cfg(), Config()
    assert (cfg.explore.uav_face_registry, cfg.explore.redetect_resolved, cfg.avoid.friend_track) == (True, False, True)
    assert (d.explore.uav_face_registry, d.explore.redetect_resolved, d.avoid.friend_track) == (False, True, False)
