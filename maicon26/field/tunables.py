"""튜너블 레지스트리: 코드 안의 숫자 중 '현장에서 확인·갱신해야 하는 것'과 그 근거를 한곳에 모은다.

상태(status)
  PDF        규정/사양서(PDF)에 있는 값. 현장에서 달라지면 사양서 쪽이 틀린 것이다.
  ASSUMED    근거 없는 가정. 현장 측정 전에는 믿지 말 것.
  SIM_TUNED  시뮬레이션에서 돌려 보고 정한 값. 시뮬 가정(잡음 모델 등)에 맞춰져 있어서 실기에서 다시 확인해야 한다. basis 에 무엇을 봤는지 적었다.
  MEASURED   현장 실험으로 갱신된 값(= config/field_overrides.json 의 provenance 에 기록이 있는 것). 레지스트리 표에는 쓰지 않는다 — 오버라이드가 생기면 자동으로 바뀐다.
exp 는 확인할 실험 ID: F01..F10(maicon26/field/experiments.py), `calib:<시험>`(maicon26/calib/ 기존 보정 도구), `MANUAL`(실험 코드 없이 줄자·눈·ros2 topic list 로 확인).
값 자체는 여기에 복사하지 않는다 — Config 의 기본값이 단일 출처이고, 표에는 현재값을 읽어서 보여 준다.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional

PDF, ASSUMED, SIM_TUNED, MEASURED = "PDF", "ASSUMED", "SIM_TUNED", "MEASURED"


@dataclass(frozen=True)
class Tunable:
    path: str                # Config 의 점 경로: "map3d.nav_thr"
    status: str
    exp: str
    unit: str = ""
    basis: str = ""          # 지금 값을 어떻게 정했나(시뮬이면 무엇을 봤나)
    how: str = ""            # 현장에서 어떻게 확인·갱신하나
    pass_if: str = ""        # 합격 기준 / 갱신 규칙
    prio: int = 2            # 1 = 먼저(틀리면 임무 실패), 2 = 보통, 3 = 나중


def _t(path, status, exp, unit="", basis="", how="", pass_if="", prio=2) -> Tunable:
    return Tunable(path, status, exp, unit, basis, how, pass_if, prio)


TUNABLES: List[Tunable] = [
    # ---------------------------------------------------------------- 깊이 카메라 설치·내부 파라미터
    _t("rig.depth_xyz", ASSUMED, "F09", "m (base_link 기준)", "(0.10, 0, 0.10) 가정: 팬/틸트 '아래' 차체 고정이라는 것만 확인됨(사용자). 위치는 모른다",
       "줄자로 base_link(차체 중심·차축 높이)에서 깊이 렌즈까지 x(앞), y(왼쪽), z(위). 그 뒤 F04 로 x 를 교차 확인", "F04 잔차 |중앙값| < 0.02 m", 1),
    _t("rig.depth_rpy", ASSUMED, "F02", "rad", "(0,0,0) 가정", "평평한 빈 바닥에서 F02(바닥 평면)로 roll·pitch. yaw 는 벽 정면에서 영상 좌우 대칭으로 눈 확인",
       "바닥 인라이어 ≥ 40 %, RMS < 10 mm", 1),
    _t("rig.base_z", ASSUMED, "F09", "m", "바퀴 반경 0.04 가정", "바닥~base_link(차축 높이) 줄자", "", 2),
    _t("rig.pan_xyz", ASSUMED, "F09", "m", "(0,0,0.20) 가정", "base_link→팬 축 위치 줄자", "", 2),
    _t("rig.camera_xyz", ASSUMED, "F09", "m (tilt_link 기준)", "(0.05,0,0.03) 가정", "팬/틸트 축에서 RGB 렌즈까지 줄자", "", 2),
    _t("rig.camera_rpy", ASSUMED, "MANUAL", "rad", "(0,0,0) 가정", "틸트 0° 일 때 RGB 광축이 수평인지 수평계로 확인", "", 3),
    _t("rig.lidar_xyz", ASSUMED, "calib:lidar_square", "m", "(0,0,0.20) 가정", "calib_run.py lidar_square 로 x, y, yaw; z 는 줄자", "", 2),
    _t("depth.hfov_deg", ASSUMED, "MANUAL", "deg", "OAK-D 계열 72° 가정(실제 모델 미확인)", "depthai 가 내는 camera_info 의 K 로 대체(K 가 있으면 hfov 는 안 쓴다)", "", 2),
    _t("depth.K", ASSUMED, "MANUAL", "px", "None(= hfov 로 근사)", "ros2 topic echo /oak/stereo/camera_info → k 9개와 해상도(K_width/K_height)", "", 2),
    _t("depth.unit_scale", ASSUMED, "MANUAL", "m/단위", "0.001(16UC1 = mm) 가정", "ros2 topic echo --once depth topic 의 encoding 확인; F03 에서 줄자와 크기가 맞는지로 교차 확인", "F03 편향 < 3 cm", 1),
    _t("depth.topic", ASSUMED, "MANUAL", "", "/oak/stereo/image_raw (depthai-ros 기본 가정)", "ros2 topic list | grep -i stereo", "", 1),
    _t("depth.info_topic", ASSUMED, "MANUAL", "", "/oak/stereo/camera_info 가정", "ros2 topic list", "", 2),
    _t("depth.min_range", ASSUMED, "F03", "m", "0.30 가정(스테레오 최소 거리)", "F03: 가까운 거리에서 유효율·편향이 괜찮은 최소 거리", "유효율 ≥ 80 %, |편향| < 3 cm", 2),
    _t("depth.max_range", ASSUMED, "F03", "m", "3.5 가정(오차 ∝ 거리²)", "F03: 편향 < 3 cm, 표준편차 < 2 cm 인 최대 거리", "", 2),
    _t("depth.min_latency_s", ASSUMED, "F01", "s", "0(미측정). 같은 시계면 쓰이지 않는다(hal/clock.py)", "F01 로 시계 모드 확인. 다른 시계일 때만 의미: 로봇을 일정 속도로 돌리며 정지 표적 방위 오차가 0 이 되는 값을 찾는다", "", 3),
    _t("depth.sim_noise_k", SIM_TUNED, "F03", "m⁻¹", "시뮬 깊이 잡음 모델 σ=k·z² 의 k=0.004(스테레오 일반론에서 가정). 시뮬 충실도용 값이라 로봇 동작에는 직접 안 쓰인다",
       "F03 이 실제 σ/z² 를 제안", "", 3),
    _t("depth.sim_dropout", SIM_TUNED, "F03", "비율", "시뮬 무효 픽셀 3 % 가정", "F03 의 유효율(1−valid)", "", 3),
    _t("depth.sim_flying_pixels", SIM_TUNED, "F06", "확률", "시뮬 가장자리 번짐 30 % 가정 — 이 허상 때문에 3D 층 필터(nav_thr 등)를 정했다", "F06 에서 빈 환경의 허상 셀 수로 간접 확인", "", 2),
    # ---------------------------------------------------------------- 3D 지도·사용
    _t("map3d.nav_thr", SIM_TUNED, "F06", "로그오즈", "1.8(약 3회 관측). 시뮬에서 허상 0 이 되도록 정함(가장자리 번짐 30 % 가정)", "F06: 빈 환경에서 3D 층 셀 수. >0 이면 올림, F05 에서 낮은 블록이 안 잡히면 내림", "F06 셀 ≤ 3 & F05 감지", 1),
    _t("map3d.nav_min_neighbors", SIM_TUNED, "F06", "개", "3(면/덩어리만 통과, 광선 방향 한 줄 허상 제거). 시뮬에서 정함", "F05/F06 와 함께 조정", "", 2),
    _t("map3d.blind_grow_cells", SIM_TUNED, "F06", "칸(5 cm)", "3: 박스 둘레 15 cm 번짐을 'LiDAR 가 보는 곳'으로 처리. 시뮬 잡음에 맞춤", "F06", "", 2),
    _t("map3d.blind_floor_layers", SIM_TUNED, "F05", "층(5 cm)", "2: 바닥에서 올라온 5~15 cm 물체만. 박스 윗부분 잔재를 거르려고 정함", "F05 에서 실제 낮은 물체 높이를 바꿔 가며", "", 2),
    _t("map3d.nav_z_lo", SIM_TUNED, "F05", "m", "0.06(바닥 잡음 위)", "F05: 바닥이 장애물로 찍히면 올림", "", 2),
    _t("map3d.stride", SIM_TUNED, "F05", "px", "2 + 프레임마다 위치 순환: 12 cm 물체가 영상에서 몇 줄밖에 안 돼서 3→2 로 바꿨다(시뮬 해상도 96x72 기준)", "실기 해상도(예 640x400)에서는 같은 물체가 더 많은 줄이므로 키울 수 있다 — F05 로 확인", "", 3),
    _t("map3d.integrate_hz", SIM_TUNED, "F05", "Hz", "5(시뮬 깊이 주기와 같게)", "Jetson CPU 부하 보고 조정(깊이 프레임 처리 시간 측정)", "처리 < 50 ms", 2),
    _t("map3d.guard_dist", ASSUMED, "F05", "m", "0.22(LiDAR stop_dist 0.17 보다 약간 큼)", "F05: 실제 정지거리(calib brake)보다 커야 한다", "정지거리 + 여유", 1),
    _t("map3d.guard_slow", ASSUMED, "F05", "m", "0.50", "F05: 감지 거리보다 작아야 의미가 있다", "감지 최초 거리 > guard_slow", 2),
    _t("map3d.guard_half_width", ASSUMED, "MANUAL", "m", "0.15(차체 폭 0.23 의 절반 + 여유)", "실제 폭 줄자", "", 3),
    _t("map3d.beam_margin_m", ASSUMED, "MANUAL", "m", "0.10(표적 면 자체는 사선 막힘에서 제외)", "표적 앞 박스 모서리가 사선을 가리는 상황을 눈으로 확인", "", 3),
    _t("map3d.beam_timeout_s", ASSUMED, "MANUAL", "s", "3(허상으로 보고 통과)", "실기에서 사선 막힘이 오래 가는 오탐이 있는지 로그(stats beam_override)", "", 3),
    _t("map3d.target_window_z", ASSUMED, "MANUAL", "m", "(0.22, 0.46): PDF p12 사진에서 짐작한 흰 원판 높이 0.30~0.37 둘레", "표적 면 흰 원판 중심 높이를 줄자로 재서 ±0.08 로", "", 2),
    _t("map3d.ugv_receiver_z", ASSUMED, "MANUAL", "m", "(0.04, 0.32): PDF p21 사진에서 짐작", "상대 UGV 수광 돔 높이 줄자", "", 2),
    _t("map3d.smog_max_range", ASSUMED, "F08", "m", "1.5: 연기 속 스테레오 열화를 가정", "스모그 장치가 켜졌을 때 F08 로 깊이를 저장해 유효율·잡음 확인", "", 3),
    _t("map3d.sync_max_dt", ASSUMED, "F01", "s", "0.25", "F01 이 카메라/깊이 프레임 간격에서 제안", "", 3),
    _t("map3d.lidar_check_bias_m", ASSUMED, "F04", "m", "0.08(중앙값 잔차 경고)", "F04 정상 설치 잔차 분포의 3배 정도", "", 3),
    _t("map3d.bump_window_s", SIM_TUNED, "F07", "s", "1.5: 시뮬에서 정체를 1.5 s 안에 알아챔", "F07", "", 1),
    _t("map3d.bump_v_min", SIM_TUNED, "F07", "m/s", "0.08", "F07 + 실제 최소 구동 속도(calib speed_steps)", "", 2),
    _t("map3d.bump_ratio", SIM_TUNED, "F07", "비율", "0.3(실제/기대 이동). 시뮬의 SLAM 자세 오차 ~1 cm 에 맞춤", "F07: 자유 주행 비율 최솟값과 막힘 비율 최댓값 사이", "분리 마진 ≥ 2배", 1),
    _t("map3d.bump_lidar_clear_m", ASSUMED, "F07", "m", "0.5", "F07", "", 3),
    _t("map3d.bump_ahead_m", ASSUMED, "F07", "m", "0.22(차체 반경 + 여유)", "F07: 접촉 때 범퍼에서 장애물까지 실제 거리", "", 3),
    _t("map3d.bump_ttl_s", ASSUMED, "MANUAL", "s", "90", "경기 길이 300 s 에 맞춘 값 — 장애물이 움직이는지에 따라", "", 3),
    # ---------------------------------------------------------------- 센서 주기·신선도
    _t("mission.lidar_stale_s", ASSUMED, "F01", "s", "0.6(10 Hz 기준 6주기 가정)", "F01 의 LiDAR 간격 p95 에서 제안", "", 1),
    _t("mission.vision_stale_s", ASSUMED, "F01", "s", "0.8", "F01 카메라 간격 p95 + 검출기 지연(YOLO 추론 시간)", "", 1),
    _t("camera.min_latency_s", ASSUMED, "F01", "s", "0(미측정). 같은 시계면 쓰이지 않는다", "F01 로 시계 모드 확인", "", 3),
    _t("lidar.min_latency_s", ASSUMED, "F01", "s", "0(미측정)", "F01", "", 3),
    # ---------------------------------------------------------------- 레이저·조준
    _t("aim.laser_lateral_m", ASSUMED, "F10", "m (왼쪽 +)", "0(미측정): 레이저 발광부가 카메라보다 옆으로 떨어진 거리", "F10(두 거리에서 점 위치) 또는 줄자", "F10 두 값 일관", 1),
    _t("aim.laser_vertical_m", ASSUMED, "F10", "m (위 +)", "0(미측정)", "F10", "", 1),
    _t("aim.laser_offset_pan_deg", ASSUMED, "F10", "deg", "0(미측정): 카메라 축 대비 레이저 축 수평 오프셋", "F10", "", 1),
    _t("aim.laser_offset_tilt_deg", ASSUMED, "F10", "deg", "0(미측정)", "F10", "", 1),
    _t("aim.tol_deg", SIM_TUNED, "MANUAL", "deg", "1.2: 시뮬 서보·검출 지터 기준", "실기 서보 정착 오차·검출 지터로 재조정(서브게임 반복)", "", 2),
    _t("aim.tol_deg_far", SIM_TUNED, "MANUAL", "deg", "0.7", "〃", "", 2),
    _t("aim.sensor_radius_frac", ASSUMED, "MANUAL", "비율", "0.6(센서 반경의 60 %)", "수광센서 크기·유효 입사각 확인 후", "", 2),
    _t("aim.kp_pan", SIM_TUNED, "MANUAL", "", "0.9: 시뮬 키네매틱 서보 기준", "실기 서보 응답(과도/진동)을 보며", "진동 없이 3 틱 안 정착", 2),
    _t("aim.kp_tilt", SIM_TUNED, "MANUAL", "", "0.9", "〃", "", 2),
    _t("aim.aim_point_target", ASSUMED, "MANUAL", "bbox 비율", "(0.5, 0.16): 센서를 못 찾을 때의 대체 추측", "실제 YOLO 라벨링 방식(면 전체 vs 물체)에 따라", "", 2),
    _t("laser.max_range", ASSUMED, "MANUAL", "m", "2.5(수광 유효거리 가정)", "표적에 수광되는 최대 거리를 거리별로 쏴 보며", "", 1),
    _t("laser.min_range", ASSUMED, "MANUAL", "m", "0.35", "〃", "", 2),
    _t("mission.strike_range", SIM_TUNED, "MANUAL", "m", "1.10(시뮬 서브게임에서)", "laser.max_range 와 서보 안정성에 맞춰", "", 2),
    # ---------------------------------------------------------------- 로봇·항법 (기존 보정 도구)
    _t("robot.v_max", ASSUMED, "calib:speed_steps", "m/s", "0.45 가정", "calib_run.py speed_steps → calib_fit.py → calib_generate.py", "", 1),
    _t("robot.w_max", ASSUMED, "calib:turn_steps", "rad/s", "2.2 가정", "calib_run.py turn_steps", "", 2),
    _t("robot.a_max", ASSUMED, "calib:speed_steps", "m/s²", "1.2 가정", "calib_run.py speed_steps / brake", "", 2),
    _t("robot.length", PDF, "F09", "m", "0.25 [PDF p21] 250 mm — 줄자로 실제 확인(안테나·바퀴 돌출 포함 여부)", "차체 최대 길이(돌출부 포함)를 줄자로", "PDF 값과 ±5 mm 이내", 1),
    _t("robot.width", PDF, "F09", "m", "0.23 [PDF p21] 230 mm — 바퀴 바깥 폭 포함 여부 확인", "차체 최대 폭(바퀴 포함)을 줄자로", "PDF 값과 ±5 mm 이내", 1),
    _t("robot.radius", ASSUMED, "F09", "m", "0.1698 = 외접 반경 √(0.125²+0.115²) (length/width 에서 파생). 직진 통과 기준이 아니다", "length/width 를 바꾸면 Config.apply_budget() 이 다시 계산", "", 2),
    _t("lidar_err.range_bias_m", ASSUMED, "MANUAL", "m", "0.005 [가정 — PDF 에 LiDAR 정확도 없음]", "정지 평면 벽 앞 0.3/1/2 m 에서 줄자 거리와 LiDAR 평균 차이", "", 1),
    _t("lidar_err.range_sigma_m", ASSUMED, "MANUAL", "m", "0.008 [가정]", "같은 측정의 표준편차", "", 1),
    _t("lidar_err.angle_sigma_deg", ASSUMED, "MANUAL", "deg", "0.3 [가정]", "직각 모서리/기둥의 방위 반복 측정", "", 2),
    _t("lidar_err.mount_xy_m", ASSUMED, "calib:lidar_square", "m", "0.005 [가정]", "calib_run.py lidar_square 잔차", "", 2),
    _t("lidar_err.mount_yaw_deg", ASSUMED, "calib:lidar_square", "deg", "0.5 [가정]", "〃", "", 2),
    _t("lidar_err.latency_s", ASSUMED, "F01", "s", "0.10 [가정]", "스캔 stamp 와 수신 시각 차, 회전 중 방위 오차", "", 1),
    _t("lidar_err.pose_sigma_m", ASSUMED, "MANUAL", "m", "0.010 [가정; 시뮬 단독 주행 중앙값 약 1.6 cm]", "SLAM 위치와 줄자 위치 비교(정지 표적 N회)", "", 1),
    _t("lidar_err.dropout_p", ASSUMED, "MANUAL", "비율", "0.02 [가정] — 거리 여유로 메우지 않고 지속성 필터로 다룬다", "정지 장면에서 무효 빔 비율", "", 2),
    _t("lidar_err.outlier_p", ASSUMED, "MANUAL", "비율", "0.005 [가정]", "정지 장면에서 이웃과 크게 어긋난 빔 비율", "", 2),
    _t("nav.rotate_clear", ASSUMED, "F09", "m", "0.219 = 외접 0.1698 + 계획 여유 0.049 (파생). 40 cm 통로(반폭 0.20)에서는 제자리 회전 불가", "통로 안에서 실제로 돌 수 있는 최소 폭", "", 1),
    _t("nav.side_lim", ASSUMED, "calib:avoid", "m", "0.178 = 반폭 0.115 + 정지 여유 0.067 (파생)", "40 cm 통로 중앙(0.20)에서 측면 보정이 작동하지 않는지", "", 2),
    _t("nav.stop_dist", ASSUMED, "calib:brake", "m", "0.192 = 앞 반길이 0.125 + 정지 여유 0.067 (센서 오차+지연; 파생). 제동거리는 slow_dist 감속 구간이 맡는다", "벽 앞 정지 위치(차체 앞 끝과 벽 사이)를 줄자로", "", 1),
    _t("robot.track_width", ASSUMED, "calib:rotate", "m", "0.20", "좌우 바퀴 중심 간격 줄자", "", 2),
    _t("robot.wheel_radius", ASSUMED, "F09", "m", "0.04", "줄자", "", 3),
    _t("robot.yaw_eff", ASSUMED, "calib:rotate", "비율", "1.0", "calib_run.py rotate", "", 2),
    _t("robot.lr_bias", ASSUMED, "calib:straight", "rad/m", "0", "calib_run.py straight", "", 2),
    _t("robot.pan_limit", PDF, "MANUAL", "deg", "±50 (PDF: 수평 100°)", "서보 실제 가동 범위 확인", "", 3),
    _t("robot.tilt_limit", PDF, "MANUAL", "deg", "±15 (PDF: 수직 30°, 중심 가정)", "서보 영점(수평)이 맞는지", "", 2),
    _t("robot.cam_height", ASSUMED, "F09", "m", "0.25(시뮬 거리 추정용)", "바닥~RGB 렌즈 줄자(rig 와 일치해야 한다)", "", 3),
    _t("camera.hfov_deg", ASSUMED, "calib:camera", "deg", "110(160° 렌즈의 보정 후 유효 화각 가정)", "체커보드 보정(calib_fit.py --camera-images)", "RMS < 0.5 px", 1),
    _t("camera.K", ASSUMED, "calib:camera", "px", "None(= hfov 로 근사)", "체커보드 보정", "", 1),
    _t("nav.inflate_hard", ASSUMED, "calib:avoid", "m", "0.164 = 반폭 0.115 + 계획 여유 0.049 (파생; 직진 통과 기준). 경로 확정은 직사각형 검증이 따로 한다. 예전 0.17 + 완화(0.8·0.6배)는 폐기", "40 cm 통로 직진·비스듬한 진입 실주행, calib_run.py avoid", "", 1),
    _t("nav.v_cruise", ASSUMED, "calib:speed_steps", "m/s", "0.38", "calib 에서 유도", "", 2),
    _t("nav2.map_frame", ASSUMED, "MANUAL", "", "map", "ros2 run tf2_tools view_frames", "", 2),
    _t("nav2.base_frame", ASSUMED, "MANUAL", "", "base_link", "〃", "", 2),
    # ---------------------------------------------------------------- 이벤트·인식
    _t("event.smog_on_ratio", SIM_TUNED, "F08", "비율", "0.04: 시뮬 스모그(허상 반사 16 %·30 % 가 8~20 cm)에서 정함. 실제 연기의 LiDAR 반사 분포는 모른다", "스모그 장치가 켜진 상태로 F08(LiDAR 저장) → 고립 반사 비율 분포 확인", "", 2),
    _t("event.smog_off_ratio", SIM_TUNED, "F08", "비율", "0.015", "〃", "", 2),
    _t("event.smog_speed_scale", ASSUMED, "MANUAL", "배", "0.8", "스모그 중 실제 시야에 맞춰", "", 3),
    _t("event.smog_fire_conf", ASSUMED, "MANUAL", "", "0.60", "스모그 중 검출 신뢰도 분포로", "", 3),
    _t("event.tower_ready_color", ASSUMED, "F08", "", "yellow(규정에 색 없음)", "3색 타워 실물 사진(F08)", "", 2),
    _t("event.tower_running_color", ASSUMED, "F08", "", "green", "〃", "", 2),
    _t("event.tower_finished_color", ASSUMED, "F08", "", "red", "〃", "", 2),
    _t("event.ugv_led_range_m", ASSUMED, "F08", "m", "1.8(후면 LED 판독 거리)", "상대 UGV 뒤 LED 를 거리별로 찍어(F08) 판독 가능 거리 확인", "", 3),
    _t("perception.conf_min", ASSUMED, "MANUAL", "", "0.40", "학습한 YOLO 의 PR 곡선", "", 2),
    _t("perception.ugv_conf_min", ASSUMED, "MANUAL", "", "0.50", "〃", "", 2),
    _t("perception.max_obj_range", ASSUMED, "MANUAL", "m", "2.2(먼 객체 오분류 방지)", "거리별 인식 정확도", "", 2),
    # ---------------------------------------------------------------- 경기장·임무 타이밍
    _t("arena.W", ASSUMED, "F09", "m", "6.12(도면 근사)", "경기장 실측", "", 1),
    _t("arena.H", ASSUMED, "F09", "m", "5.00(도면 근사)", "경기장 실측", "", 1),
    _t("arena.base_depth", ASSUMED, "F09", "m", "0.80(도면 근사)", "거점 영역 깊이 실측", "", 1),
    _t("arena.base_height", ASSUMED, "F09", "m", "0.90(도면 근사; 규격서는 1200x400)", "거점 영역 폭 실측", "", 1),
    _t("arena.common_half", ASSUMED, "F09", "m", "0.615(도면 근사)", "COMMON 띠 반폭 실측", "", 2),
    _t("arena.box_size", PDF, "F09", "m", "0.40", "박스 실측", "", 2),
    _t("arena.wall_h", ASSUMED, "F09", "m", "0.40", "외벽 높이 실측", "", 2),
    _t("arena.base_cy", ASSUMED, "F09", "m", "(4.16, 2.50, 0.83) 도면 근사", "거점 1·2·3 중심 y 실측", "", 1),
    _t("mission.prestage_at", ASSUMED, "MANUAL", "s", "222(M5 사전 이동 시작)", "주행 시간 실측에 맞춰", "", 2),
    _t("mission.ugv_range", SIM_TUNED, "MANUAL", "m", "1.40(M4 교전 거리)", "M4 는 기본 OFF", "", 3),
    # ---------------------------------------------------------------- 규정 값(PDF) — 현장 사양이 다르면 규정집이 우선
    _t("laser.ammo", PDF, "MANUAL", "발", "8(PDF 탄창)", "로봇 펌웨어 탄창 수와 일치하는지", "", 2),
    _t("laser.shot_s", PDF, "MANUAL", "s", "5(1발 5초 발광)", "펌웨어가 5 s 를 직접 하는지(mentoring 질문 6)", "", 2),
    _t("laser.cooldown_s", PDF, "MANUAL", "s", "5(재발사 대기)", "〃", "", 2),
    _t("laser.dwell_s", PDF, "MANUAL", "s", "2(2초 연속 수광 = 유효)", "표적 LED 가 켜지는 조건 확인", "", 1),
    _t("laser.disabled_s", PDF, "MANUAL", "s", "10(피격 후 정지)", "규정집", "", 3),
    _t("mission.t_total", PDF, "MANUAL", "s", "300(경기 5분)", "규정집", "", 3),
    _t("mission.t_base_cmd", PDF, "MANUAL", "s", "240(종료 1분 전 목표거점 명령)", "규정집", "", 3),
    _t("mission.t_report", PDF, "MANUAL", "s", "30(종료 후 M6 제출 기한)", "규정집", "", 3),
    # ---------------------------------------------------------------- LiDAR 자기 가림
    _t("lidar.self_mask_sectors", ASSUMED, "F08", "(deg, deg)", "()(기본 끔 — 장착 위치를 모른다)", "F08 로 LiDAR 를 저장해 제자리에서 0.3 m 안에 늘 같은 방위에 나오는 반사(팬틸트 기둥·안테나)를 찾는다. 로그의 lidar_suspects 도 참고",
       "자기 가림 방위가 없어야 한다", 1),
    _t("lidar.self_mask_range", ASSUMED, "F08", "m", "0.35", "자기 가림 반사 거리", "", 3),
    _t("lidar.min_range", ASSUMED, "F08", "m", "0(끔)", "몸체 안쪽 반사가 있으면 그 거리로", "", 3),
    # ---------------------------------------------------------------- 서보·복구·항법 타이밍 (시뮬 튜닝)
    _t("aim.hold_before_fire", SIM_TUNED, "MANUAL", "s", "0.20: 정렬이 이만큼 안정되면 발사", "실기 서보 정착 시간에 맞춰", "", 2),
    _t("aim.lost_timeout", SIM_TUNED, "MANUAL", "s", "0.8: 이 시간 안 보이면 추적 해제", "검출 주기·끊김에 맞춰", "", 3),
    _t("aim.body_kp", SIM_TUNED, "MANUAL", "", "1.6: 팬 → 차체 회전", "실기 차체 회전 응답", "", 3),
    _t("aim.sensor_min_tol_deg", ASSUMED, "MANUAL", "deg", "0.30: 팬 1단위 = 0.5° 라 양자화 오차가 ±0.25° [PDF p14] (틸트 0.15°)", "팬 1단위 이동으로 센서 원이 화면에서 몇 픽셀 움직이는지", "", 3),
    _t("mission.aim_budget_s", SIM_TUNED, "MANUAL", "s", "25: 조준 단계 시간 예산", "실기 조준 소요 시간 분포로", "", 3),
    _t("mission.recover_after_s", SIM_TUNED, "MANUAL", "s", "4: 항법이 이 시간 넘게 막히면 RECOVER", "", "", 3),
    _t("mission.recover_back_s", SIM_TUNED, "MANUAL", "s", "0.7 후진 시간(≈ 7 cm @0.1 m/s)", "실기 후진 거리·후방 여유", "", 3),
    _t("mission.spin_w", SIM_TUNED, "MANUAL", "rad/s", "1.1(시점 도착 후 360° 회전 속도)", "회전 중 검출 블러·미끄러짐", "", 3),
    _t("nav.slow_dist", ASSUMED, "calib:brake", "m", "0.45", "속도별 정지거리에서 유도", "", 2),
    _t("nav.lookahead", SIM_TUNED, "calib:avoid", "m", "0.38: 경로 추종 선행 거리", "실기 추종 진동 여부", "", 3),
    _t("nav.v_slow", SIM_TUNED, "calib:speed_steps", "m/s", "0.18", "", "", 3),
    _t("nav.stuck_s", SIM_TUNED, "calib:avoid", "s", "3: 내장 항법기 끼임 판정 시간", "", "", 3),
    _t("nav.stuck_move", SIM_TUNED, "calib:avoid", "m", "0.05", "", "", 3),
    # ---------------------------------------------------------------- ROS 이름 (틀리면 아무것도 안 온다)
    _t("nav2.map_topic", ASSUMED, "MANUAL", "", "/map", "ros2 topic list", "", 1),
    _t("nav2.cmd_topic", ASSUMED, "MANUAL", "", "/cmd_vel_nav2(Nav2 출력; 드라이버가 구독하는 /cmd_vel 과 분리)", "ros2 topic list, 드라이버 구독 확인", "", 1),
    _t("nav2.action_name", ASSUMED, "MANUAL", "", "navigate_to_pose", "ros2 action list", "", 2),
    # ---------------------------------------------------------------- 이벤트 타이밍·임계 (시뮬/가정)
    _t("event.smog_near_m", SIM_TUNED, "F08", "m", "1.2: 이보다 가까운 고립 반사만 본다", "스모그 장치 켠 LiDAR(F08)", "", 3),
    _t("event.smog_on_s", SIM_TUNED, "F08", "s", "0.8", "〃", "", 3),
    _t("event.smog_off_s", SIM_TUNED, "F08", "s", "3.0", "〃", "", 3),
    _t("event.speckle_nbr_m", SIM_TUNED, "F08", "m", "0.25: 이웃 빔 거리차 기준", "정상 장면의 고립 반사 수(0~2 가정) 확인", "", 3),
    _t("event.tower_conf_min", ASSUMED, "F08", "", "0.6", "타워 LED 검출 신뢰도", "", 3),
    _t("event.tower_min_t", ASSUMED, "MANUAL", "s", "60: 시작 후 이 시간 전에는 타워로 정지 안 함", "", "", 3),
    _t("event.uav_off_s", ASSUMED, "MANUAL", "s", "8: 새 프레임이 이만큼 없으면 드론 중단", "UAV 영상 주기 변화 실제 값(1→2→5 s)", "", 3),
    # ---------------------------------------------------------------- M3 온라인 탐색 (explore.*)  — 시뮬 비교로 정한 값. 가중치는 측정값이 아니다
    _t("explore.strategy", SIM_TUNED, "MANUAL", "", "online(시뮬 6시드에서 이동거리 -7 %·충돌 감소, 재현율은 고정 순회보다 약 5 %p 낮음). lattice 로 되돌리면 기존 5x5 순회",
       "현장 5분 주행 2~3회로 lattice 와 online 의 M3 정답 대조(면 단위)", "online 재현율 ≥ lattice", 1),
    _t("explore.m3_counting", SIM_TUNED, "MANUAL", "", "ledger(박스 면 1개=객체 1개). omap 은 위치 병합이라 같은 면을 두 번 세는 오류가 시뮬에서 났다",
       "현장 객체 개수와 제출 개수 대조(중복·누락)", "", 1),
    _t("explore.view_max_incidence_deg", ASSUMED, "MANUAL", "deg", "70: 면을 이 입사각보다 비스듬히 보면 '본 것'으로 안 친다", "실제 카메라로 면 사진: 몇 도까지 객체 종류를 알아보는가", "분류 정확도 ≥ 90 %", 2),
    _t("explore.nbv_dists", ASSUMED, "MANUAL", "m", "(0.45,0.6,0.8,1.0,1.4): 면 앞 시점 거리 후보. perception.max_obj_range(2.2) 안", "실기 YOLO 가 객체를 안정적으로 분류하는 최소·최대 거리", "", 2),
    _t("explore.w_threat", SIM_TUNED, "MANUAL", "", "0.8: 효용 = 정보×(1−min(0.9,w·위험))/시간. 시뮬에서 정한 임의 가중치", "회피 시나리오 비교 곡선으로 조정", "", 3),
    _t("explore.deadline_margin_s", SIM_TUNED, "MANUAL", "s", "3: prestage_at 앞 여유", "실기 이동 시간 분산을 보고", "", 2),
    _t("explore.stage_margin_s", SIM_TUNED, "MANUAL", "s", "4: M5 명령(t_base_cmd) 앞 여유", "〃", "", 2),
    _t("explore.spin_clear_m", ASSUMED, "MANUAL", "m", "0.32: 제자리 360° 회전에 필요한 몸 둘레 여유. 스키드 조향 밀림은 시뮬 값", "실기 제자리 회전 시 쓸리는 반경", "", 2),
    _t("mission.strike_max_incidence_deg", ASSUMED, "MANUAL", "deg", "40: 지정표적 사격 입사각 한계(창 ±3.5 cm 라 비스듬하면 빗나간다)", "실기 표적 창 크기·수광 각도", "", 1),
    _t("mission.fire_range_slack", ASSUMED, "MANUAL", "m", "0.15: strike_range+이 값 이하에서만 발사", "실기 서보 지터에서 맞는 최대 거리", "", 2),
    # ---------------------------------------------------------------- 적 UGV 회피 (avoid.*) — 대부분 가정. 실측이 필요한 것은 prio 1
    _t("avoid.enable", SIM_TUNED, "MANUAL", "", "True. 끄면 기존 동작(위협을 모르는 항법)", "현장에서 상대 UGV 가 있을 때 켜고 끈 5분 주행 비교", "피격↓·M5 시각 +10 s 이내", 1),
    _t("avoid.r_aware", SIM_TUNED, "MANUAL", "m", "3.0", "시뮬 스윕. 실기 LiDAR 인지거리·정지거리로", "", 2),
    _t("avoid.r_evade", SIM_TUNED, "MANUAL", "m", "1.6", "〃", "", 2),
    _t("avoid.r_close", SIM_TUNED, "MANUAL", "m", "0.8", "〃", "", 2),
    _t("avoid.r_core", ASSUMED, "MANUAL", "m", "2.2(= laser.max_range − 0.3): 사정거리 안·시야가 트이면 거리와 무관하게 맞는다는 시뮬 규칙", "수광센서 유효거리와 거리별 피격 확률 실측", "", 1),
    _t("avoid.r_laser_extra", ASSUMED, "MANUAL", "m", "0.3", "수광센서 유효거리 실측", "", 2),
    _t("avoid.r_body", ASSUMED, "MANUAL", "m", "0.13(시뮬 UGV 반경)", "상대 UGV 실물 치수", "", 2),
    _t("avoid.p_cons", SIM_TUNED, "MANUAL", "", "0.7: 팀 미확인 UGV 의 최소 위험 가중", "카메라 팀 판정 정확도(안테나 색)를 현장 사진으로 측정한 뒤 조정", "", 1),
    _t("avoid.sigma_cam", ASSUMED, "MANUAL", "m", "0.08", "정지 상대 UGV 반복 측정", "", 2),
    _t("avoid.sigma_uav", ASSUMED, "MANUAL", "m", "0.15(시뮬은 0.04). 실기 UAV 영상 검출기는 아직 없다", "UAV 영상 검출기가 생긴 뒤 측정", "", 2),
    _t("avoid.sigma_lidar", ASSUMED, "MANUAL", "m", "0.06", "정지 상대 UGV 를 LiDAR 로 반복 측정", "", 2),
    _t("avoid.lidar_blob_min", ASSUMED, "MANUAL", "m", "0.12", "상대 UGV LiDAR 군집 폭(F12 제안): 거리별 호 길이", "", 1),
    _t("avoid.lidar_blob_max", ASSUMED, "MANUAL", "m", "0.36", "〃 (박스 면 0.4 m 와 구별되는 상한)", "", 1),
    _t("avoid.lidar_blob_min_speed", SIM_TUNED, "MANUAL", "m/s", "0.15: 새로 드러난 정적 박스의 자세 오차성 겉보기 이동을 거르려고 정함(시뮬 오탐 120 s 에 0~2회)", "실기 LiDAR·SLAM 지터에서 정적 장면 오탐률 측정", "정적 장면 오탐 0", 1),
    _t("avoid.w_risk", SIM_TUNED, "MANUAL", "", "3.0: 위험 비용 가중. 시뮬에서 피격↔시간 균형", "현장 비교로 조정", "", 2),
    _t("avoid.w_close", SIM_TUNED, "MANUAL", "", "1.5", "〃", "", 3),
    _t("avoid.exposure_limit_s", ASSUMED, "MANUAL", "s", "1.2(= 0.6×dwell_s 2 s [PDF]): 적이 이미 조준 중일 수 있다는 마진", "상대 조준 지연 실측", "", 1),
    _t("avoid.v_enemy_max", ASSUMED, "MANUAL", "m/s", "0.45(= robot.v_max 가정)", "상대 속도 관찰", "", 2),
    _t("avoid.risk_extra_inflate_m", SIM_TUNED, "MANUAL", "m", "0.07: 위험 인지 경로가 벽을 타지 않게. 시뮬에서 벽 따라가다 LiDAR 제한에 걸려 M5 를 놓친 뒤 정함", "실기 LiDAR 제한 거리", "", 2),
    _t("avoid.reverse_speed", ASSUMED, "calib:brake", "m/s", "0.12: 근거리 후진 속도", "실제 후진 정지거리", "", 2),
    _t("avoid.t_margin_s", SIM_TUNED, "MANUAL", "s", "8: M5 시간 여유 계산 상수", "실기 이동 시간 분산", "", 2),
    _t("avoid.fresh_s", SIM_TUNED, "MANUAL", "s", "1.6: 소유권(EVADE/CLOSE)을 거는 관측의 최대 나이. UAV 지연 2 s 에서 오래된 위치로 움직이면 기준선보다 맞았다(시뮬)", "실기 UAV 영상 지연·주기 확인 후", "", 2),
    _t("avoid.cap_s", SIM_TUNED, "MANUAL", "s", "25(60 s 창): 회피가 구동을 쥐는 시간 상한 — 임무 지연 방지", "현장 비교", "", 2),
    _t("avoid.strike_cooldown_s", SIM_TUNED, "MANUAL", "s", "3: 회피 직후 지정표적 재접근 쿨다운", "", "", 3),
    _t("avoid.m2_defer_max_s", SIM_TUNED, "MANUAL", "s", "30: 회피로 M2 가 미뤄진 합계 상한", "", "", 3),
    # ---------------------------------------------------------------- 로봇 ROS2 API (2차 사전교육 PDF p9~18) — hal/wingbot.py, hal/ros_mil.py
    _t("hal.scan_topic", ASSUMED, "MANUAL", "", "/scan: LiDAR(D500) 토픽은 API 자료에 없다", "ros2 topic list -t | grep -i scan", "", 1),
    _t("hal.type_override", ASSUMED, "MANUAL", "", "{}: /mil/* 메시지·서비스 타입은 실행 중인 토픽/서비스에서 자동 탐색(패키지 이름을 모른다)",
       "ros2 topic list -t / ros2 service list -t 에서 /mil/state·/mil/match·/mil/oled·/mil/gimbal·/mil/fire 타입 확인. 자동 탐색이 실패하면 'pkg/msg/Name' 으로 직접 적는다",
       "stats['mil_ready'] 5개 모두 True", 1),
    _t("hal.pan_sign", ASSUMED, "MANUAL", "", "−1: 내부 팬은 왼쪽 +, API 는 오른쪽 + [PDF p14·15]", "연습 모드에서 pan=+50 을 보내 실제로 오른쪽으로 도는지, angular.z=+1 이 왼쪽 회전인지 눈으로 확인", "방향 일치", 1),
    _t("hal.tilt_sign", ASSUMED, "MANUAL", "", "+1: 위 +", "tilt=+50 이 위로 가는지", "방향 일치", 1),
    _t("hal.gimbal_latency_s", ASSUMED, "MANUAL", "s", "0.05: 팬틸트 명령 접수→움직임 시작(위치 추정용). 위치 피드백이 없어 모델로 추정한다", "고속 카메라/영상으로 명령 후 화면이 움직이기 시작하는 시간", "촬영 시각 팬각 오차 < 0.5°", 2),
    _t("hal.gimbal_speed", ASSUMED, "MANUAL", "deg/s", "150: 서비스 speed(1~270). 추정 위치 모델도 같은 값으로 움직인다고 본다", "pan 0→+50 이동 시간을 재서 실제 속도 확인", "실측/명령 속도 ±15 %", 2),
    _t("hal.gimbal_acc", PDF, "MANUAL", "", "20 (권장값) [PDF p14]", "", "", 3),
    _t("hal.cmd_stale_s", ASSUMED, "MANUAL", "s", "0.3: Agent 가 이 시간 새 속도를 안 주면 HAL 이 0 을 낸다(PDF: 0.5 s 끊기면 로봇이 자동 정지)", "제어 루프 최대 지연 로그(tick_ms_max)", "", 2),
    _t("hal.locked_hold_s", ASSUMED, "MANUAL", "s", "1.0: 제어 허용 중 fire/gimbal 이 locked 를 주면 이 시간 피격 잠금으로 본다", "피격 후 locked 응답이 실제 몇 초 동안 오는지", "", 3),
    _t("hal.drive_pause_s", ASSUMED, "MANUAL", "s", "10: 누적 피격이 hp_max 에 이르면 주행 정지 시간(예전 코드 규칙을 옮김; 2차 자료엔 없다)", "룰북/실기 확인: HP 0 에서 주행이 정지하는가", "", 2),
    _t("hal.tilt_stop_deg", ASSUMED, "MANUAL", "deg", "25: |roll|·|pitch| 가 넘으면 정지 [PDF p17: 기울기·충격 감지 시 정지]", "경기장 경사·박스 넘기 때 IMU 값 기록", "정상 주행에서 오동작 0", 2),
    _t("hal.impact_g", ASSUMED, "MANUAL", "g", "3.0: 충격 이벤트 기록 한계(정지는 하지 않음)", "벽/박스 접촉 때 가속도 크기 기록", "", 3),
    _t("hal.freeze_clock_on_pause", ASSUMED, "MANUAL", "", "True: cmd=pause 동안 경기 시계를 멈춘다. 서버가 일시정지 시간을 빼 주는지 문서에 없다", "주최 측에 확인", "", 2),
    _t("hal.battery_low_pct", ASSUMED, "MANUAL", "%", "20: 이 아래면 깊이 영상 처리 중단(배터리 교체 불가, 약 40분 [PDF p6])", "연속 경기 후 실제 배터리 소모", "", 3),
    _t("mission.m3_first_s", ASSUMED, "MANUAL", "s", "60: M3 첫 제출. 서버 문서 7단계: 마지막 제출을 채점하고 5:00 이후는 409 → 미리·자주 보내라", "", "", 2),
    _t("mission.m3_resubmit_s", ASSUMED, "MANUAL", "s", "15: M3 갱신 제출 간격(집계가 바뀔 때만)", "", "", 3),
]

_BY_PATH: Dict[str, Tunable] = {t.path: t for t in TUNABLES}


def get(path: str) -> Optional[Tunable]:
    return _BY_PATH.get(path)


def resolve(cfg: Any, path: str) -> Any:
    """Config 에서 점 경로의 현재값을 읽는다."""
    cur = cfg
    for k in path.split("."):
        cur = getattr(cur, k)
    return cur


def status_of(path: str, provenance: Dict[str, Any]) -> str:
    """오버라이드(실측 갱신)가 있으면 MEASURED, 없으면 레지스트리 상태, 레지스트리에 없으면 'UNLISTED'."""
    if path in provenance:
        return MEASURED
    t = _BY_PATH.get(path)
    return t.status if t else "UNLISTED"


def summary(cfg: Any, provenance: Dict[str, Any]) -> Dict[str, int]:
    out: Dict[str, int] = {PDF: 0, ASSUMED: 0, SIM_TUNED: 0, MEASURED: 0}
    for t in TUNABLES:
        out[status_of(t.path, provenance)] += 1
    return out
