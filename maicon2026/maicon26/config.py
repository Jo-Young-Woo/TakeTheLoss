"""모든 튜닝 값은 여기 한 곳에. 숫자를 코드에 흩뿌리지 않는다.

기본값 출처 표기:
  [PDF]  = 1차 사전교육 자료에 명시된 규칙/사양
  [가정] = 자료에 없어서 둔 합리적 가정 → 사전 테스트 경기장에서 반드시 실측·교정
덮어쓰기: Config.load("my.yaml") 또는 .json (PyYAML이 없으면 json만).
"""
from __future__ import annotations

import dataclasses
import json
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

from .arena import ArenaGeometry


@dataclass
class RobotCfg:
    # ---- 차체 외형 [PDF p21]: 250(L) × 230(W) × 320(H) mm. 방향이 있는 직사각형이 기준 외곽이다(maicon26/footprint.py).
    length: float = 0.25           # m [PDF p21] 진행 방향 길이
    width: float = 0.23            # m [PDF p21]
    height: float = 0.32           # m [PDF p21]
    radius: float = 0.1698         # m 외접 반경 √(0.125²+0.115²) — 원이 꼭 필요한 곳(레거시 코드)용. 직진 통과 기준이 아니다
    collision_model: str = "rect"  # 시뮬 충돌: "rect"(직사각형, 기본) | "circle"(레거시: radius_legacy 원) — 변경 전후 비교용
    radius_legacy: float = 0.14    # collision_model="circle" 일 때의 반경 (변경 전 값)
    v_max: float = 0.45            # m/s [가정] 6WD 메탈기어모터 — 실측 후 조정
    w_max: float = 2.2             # rad/s [가정]
    a_max: float = 1.2             # m/s² [가정] 가감속 제한(슬립 방지)
    pan_limit: float = 50.0        # deg [PDF] Horizontal 100도 → ±50
    tilt_limit: float = 15.0       # deg [PDF] Vertical 30도 → ±15(중심 가정)
    pan_rate: float = 150.0        # deg/s [가정] ST3215 서보 속도 제한
    cam_height: float = 0.25       # m [가정] 바닥~카메라 (시뮬/거리추정용)
    # ---- 차동(스키드 스티어) 구동 — hal/diffdrive.py. 6WD 이지만 조향 서보 언급이 없어 좌우 바퀴 속도 차로 돈다고 가정 [가정]
    track_width: float = 0.20      # m [가정] 좌우 바퀴 중심 간격 (calib: model.wheels.track). 실측 필수
    wheel_radius: float = 0.04     # m [가정] 바퀴 반경 (바퀴 각속도 환산용)
    wheel_v_max: float = 0.0       # m/s 바퀴 1개 최대 선속도. 0 이면 v_max (직진 최고속도 = 두 바퀴가 한계일 때)
    yaw_eff: float = 1.0           # 지령 각속도 대비 실제 각속도 비(스키드 슬립). 보정 drive.pivot.yaw_efficiency
    lr_bias: float = 0.0           # rad/m 직진 시 좌우 편차(+ = 왼쪽으로 휨). 보정 drive.lr_bias_rad_per_m
    wheel_limit: bool = True       # 바퀴 속도 한계 적용(곡률 유지 비례 축소). 끄면 v, w 를 따로 제한하던 기존 동작


@dataclass
class HalCfg:
    """실기 ROS2 API(2차 사전교육 자료 p9~18) 연결 설정. [PDF] = 자료에 명시, [가정] = 자료에 없어 둔 값(현장 확인).
    pan/tilt 는 각도가 아니라 위치값이다(−100~100): 1단위 = 팬 0.5° / 틸트 0.15° → 팬 ±50°, 틸트 ±15°(수평 100° / 수직 30°와 일치)."""
    # ---- 토픽/서비스 이름 [PDF p10]
    cmd_topic: str = "/cmd_vel"
    odom_topic: str = "/odom"
    image_topic: str = "/image_raw"
    imu_topic: str = "/imu/data"
    imu_heading_fusion: bool = False   # Explicit candidate opt-in; relative yaw only, validated level mounting.
    imu_heading_max_age_s: float = 0.15
    state_topic: str = "/mil/state"
    match_topic: str = "/mil/match"
    oled_topic: str = "/mil/oled"
    gimbal_srv: str = "/mil/gimbal"
    fire_srv: str = "/mil/fire"
    scan_topic: str = "/scan"          # [가정] LiDAR(D500) 토픽은 자료에 없다 → ros2 topic list 로 확인
    # 메시지·서비스 타입은 실행 중인 토픽/서비스에서 알아낸다(패키지 이름을 모른다). 안 되면 여기에 'pkg/msg/Name' 으로 직접 적는다
    ros_namespace: str = ""  # Isolated simulation API and TF.
    type_override: Dict[str, str] = field(default_factory=dict)   # 키: state | match | oled | gimbal | fire
    # ---- 속도 한계·주기 [PDF p15]
    v_cap: float = 0.6                 # m/s linear.x ±0.6
    w_cap: float = 3.0                 # rad/s angular.z ±3
    cmd_hz: float = 10.0               # 속도 명령 반복 주기 (자료: 약 10 Hz, 0.5 s 끊기면 자동 정지)
    cmd_stale_s: float = 0.3           # [가정] Agent 가 이 시간 넘게 새 명령을 안 주면(멈춤·예외) HAL 이 0 을 낸다
    # ---- 팬틸트 [PDF p14] / 서보 동작 [가정]
    pan_deg_per_unit: float = 0.5
    tilt_deg_per_unit: float = 0.15
    gimbal_unit_max: float = 100.0
    pan_sign: float = -1.0             # 내부(왼쪽 +) → API(오른쪽 +). 현장 방향 확인 시 틀리면 +1
    tilt_sign: float = 1.0             # 위 +, 같은 방향
    gimbal_speed: float = 150.0        # deg/s (1~270). 추적 응답과 서보 부담의 절충 [가정] (robot.pan_rate 와 같은 값)
    gimbal_acc: float = 20.0           # 1~254, 자료 권장 20 [PDF p14]
    gimbal_latency_s: float = 0.05     # [가정] 명령 접수→움직임 시작 지연(추정 위치용). 실측 필요
    gimbal_min_interval_s: float = 0.05    # 같은 목표 변경을 이 간격보다 자주 서비스로 보내지 않는다
    gimbal_timeout_s: float = 1.0      # 응답이 이 시간 넘게 없으면 다음 호출을 허용
    locked_retry_s: float = 0.5        # locked 응답 뒤 재시도 간격
    locked_hold_s: float = 1.0         # 발사/팬틸트가 locked 를 돌려주면(제어 허용 중) 이 시간 동안 피격 잠금으로 본다 [가정]
    # ---- 레이저 [PDF p16]
    fire_wait_s: float = 0.06          # 응답 대기 상한. 시간 초과는 성공이 아니며 중복 요청은 발사 주기 동안 제한
    drive_pause_s: float = 10.0        # [가정] 누적 피격이 hp_max 에 이르면 주행 정지 시간 (1차 자료 규칙을 옮긴 것)
    # ---- 카메라/IMU [PDF p17]
    cam_max_fps: float = 10.0          # 10 FPS 이하 권장
    tilt_stop_deg: float = 25.0        # [가정] |roll| 또는 |pitch| 가 이 값을 넘으면 정지(자료: 기울기·충격 감지 시 Twist() 즉시 정지)
    tilt_clear_deg: float = 15.0       #   이 값 아래로 내려와야 해제(히스테리시스)
    impact_g: float = 3.0              # [가정] 가속도 크기가 이 값을 넘으면 충격 이벤트로 기록(정지는 기울기일 때만)
    imu_mount_yaw_deg: float = 0.0     # [실측] IMU 의 x 축이 차체 앞(+x)에서 왼쪽으로 몇 도 돌아 달렸나(충돌 방향 계산용. 기본 0 = 같은 방향)
    # ---- 들어오는 데이터 유효성(maicon26/validate.py, 프롬프트 9장)
    fire_left_ms_max: float = 15000.0  # /mil/fire left_ms 허용 최대(= 규칙 10 s + 여유). 유한하고 0..이 값일 때만 쓴다(V1)
    frozen_frames: int = 20            # [가정] 같은 영상이 이만큼 이어지면 camera_frozen(10 fps 에서 2 s) — 정지 장면 1분으로 오탐 0 확인(V4)
    dark_frames: int = 30              # 평균 밝기 <5 또는 >250 이 이만큼 이어지면 경고만(버리지 않는다)
    # ---- OLED [PDF p13]
    oled_cols: int = 21
    oled_lines: int = 4
    oled_min_interval_s: float = 0.5
    oled_refresh_s: float = 8.0        # 10 s 뒤 기본 화면으로 돌아가므로 그 전에 다시 쓴다
    oled_period_s: float = 1.0         # Agent 가 표시 내용을 갱신하는 주기
    # ---- 경기 단계 반응
    freeze_clock_on_pause: bool = True     # [가정] cmd=pause 동안 경기 시계(t_start)를 멈춘다 — 서버가 일시정지 시간을 빼 주는지 HTTP 문서로 확인
    match_cmd_stop_backup: bool = True     # cmd=report/stop 이면 서버 STOP 이 없어도 정지 절차를 시작(서버 STOP 유실 보조)
    battery_low_pct: float = 20.0          # 이 아래면 경고를 남기고 깊이 3D 지도를 끈다(자료: 배터리 교체 불가 — Depth 카메라 등 전력 절약)
    step_budget_s: float = 0.1             # [PDF p18] team_node step() 은 0.1 s 이내. 넘으면 로그·통계
    restart_state_path: str = ""           # 팀 노드 자동 재실행(경기당 최대 3회) 시 경기 시작 시각을 복원할 파일. 빈 값 = 복원 안 함


@dataclass
class CameraCfg:
    width: int = 640
    height: int = 480
    hfov_deg: float = 110.0        # [가정] 160° 광각 렌즈 중 왜곡 보정 후 사용하는 유효 화각
    min_latency_s: float = 0.0     # 카메라 촬영→수신 최소 지연(s) [미측정]. stamp 가 다른 시계일 때만 쓴다(hal/clock.py): 알면 방위 오차가 사라진다
    # 체커보드 보정값(calib/camera.py → profile → config/robot_calibrated.json). 있으면 hfov 대신 fx, fy, cx, cy 를 쓴다
    # (주점이 영상 중심에서 벗어난 만큼의 방위 오차, fx≠fy 가 사라진다). 왜곡 보정(언디스토션)을 거친 영상 기준이다. [미보정]
    K: Optional[Tuple[float, ...]] = None      # (fx,0,cx, 0,fy,cy, 0,0,1)
    # Explicit simulation-only runtime contract; real camera calibration stays separate.
    calibration_profile: Optional[Dict[str, Any]] = None
    # ---- 어안 모델(7장). 팬틸트 카메라는 OV5640 160° 어안이다(하드웨어 PDF p45). 기본은 핀홀이라 기존 동작이 그대로다.
    model: str = "pinhole"                     # "pinhole" | "fisheye". fisheye 인데 K 나 D(4개)가 없으면 경고 후 핀홀(hfov)로 돌아간다
    D: Optional[Tuple[float, ...]] = None      # 어안 왜곡 계수 k1..k4 (OpenCV fisheye, 640×480 보정, OpenCV 4.x). [미보정]
    usable_hfov_deg: Optional[float] = None    # 커버리지에 쓰는 수평 화각(None = 모델 전체). 가장자리 검출률을 보고 정한다 [미측정]


@dataclass
class RigCfg:
    """차체→팬→틸트→센서 기구학(= URDF/TF). calib/robot_profile.yaml 의 model.pan_tilt.xyz, sensors.*.xyz/rpy 와 같은 값이다.
    단위 m / rad. [가정] 모두 CAD·줄자로 실측해서 프로파일에 넣을 것 — 깊이 지도는 이 값이 틀리면 그만큼 어긋나 쌓인다."""
    base_z: float = 0.04                                   # 바닥(base_footprint)→base_link = 바퀴 반경
    pan_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.20)       # base_link→pan_link
    camera_xyz: Tuple[float, float, float] = (0.05, 0.0, 0.03)   # tilt_link→camera_link (RGB)
    camera_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    # 깊이 카메라(OAK Stereo)는 2축 팬/틸트 '아래'에 붙은 고정 센서(사용자 확인): base_link→depth_link. 팬/틸트와 같이 움직이지 않는다.
    # 위치는 몰라 가정: 차체 앞쪽 위(앞 끝 0.125 m 안쪽, 팬틸트 0.20 m 보다 낮게) — 실측 필수
    depth_xyz: Tuple[float, float, float] = (0.10, 0.0, 0.10)
    depth_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    lidar_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.20)     # base_link→laser
    # OAK-D Lite 컬러 카메라(보조 탐지, 6장). 기본은 depth_xyz 와 같은 값 — URDF 3d_camera_link(0.065, 0, 0.022)와 다르니 줄자로 잰다 [실측 필요]
    oak_rgb_xyz: Tuple[float, float, float] = (0.10, 0.0, 0.10)
    oak_rgb_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class DepthCfg:
    """깊이 카메라(OAK Stereo, PDF p20~21 'RGB-D'). 팬/틸트 아래 차체에 고정(전방 고정 시야) — calib/robot_profile.yaml sensors.depth, RigCfg.depth_xyz."""
    enable: bool = True
    width: int = 96                # 시뮬 해상도. 실기 strict 모드는 Image와 CameraInfo의 실제 해상도 일치를 요구한다
    height: int = 72
    hfov_deg: float = 72.0         # [가정] OAK-D 계열 스테레오 수평 화각. K 가 있으면 쓰지 않는다
    K: Optional[Tuple[float, ...]] = None      # (fx,0,cx, 0,fy,cy, 0,0,1) — 깊이 camera_info 에서. 해상도는 K_width x K_height 기준
    K_width: int = 0               # 시뮬/오프라인 K 해상도(0이면 width x height). 실기는 수신 CameraInfo 기준
    K_height: int = 0
    unit_scale: float = 0.001      # 16UC1 값 × 이 값 = m (OAK 기본은 mm). 32FC1 은 이미 m 이라 무시한다
    min_range: float = 0.30        # m [가정] 스테레오 최소 거리(이보다 가까운 값은 믿지 않는다)
    max_range: float = 3.5         # m [가정] 이 거리까지만 쓴다(스테레오 오차 ∝ 거리², 경기장 최대 대각선은 7.9 m)
    min_latency_s: float = 0.0     # 촬영→수신 최소 지연(s) [미측정]. stamp 가 다른 시계일 때만(hal/clock.py)
    topic: str = "/oak/stereo/image_raw"          # [가정] depthai-ros 기본 이름 — 실기에서 확인
    info_topic: str = "/oak/stereo/camera_info"
    # 실기는 프레임/CameraInfo/TF 검증을 통과한 보정만 사용한다. 시뮬레이션의 가정 K/HFOV는 별도 허용.
    require_calibration: bool = False
    calibration_verified: bool = False  # 실제 장착값을 rig.depth_* / base_z에 반영한 뒤에만 설정
    optical_frame: str = ""             # 실제 Image/CameraInfo header.frame_id. 이름을 추측하지 않는다
    tf_tolerance_m: float = 0.02         # 설정된 설치 위치와 ROS TF 사이 허용 차이
    tf_tolerance_deg: float = 2.0        # 설정된 광학 축과 ROS TF 사이 허용 각도
    nav_allowed: bool = False            # 실기에서 깊이를 장애물 항법(Nav2 장애물 입력·3D 층·전방 깊이 정지)에 쓰도록 허용할지. False 면 실기 시작 때 해당 스위치를 끈다 — 실기 항법의 장애물 입력은 2D LiDAR 뿐
    blind_reverse_mps: float = 0.2       # 영상은 오는데 쓸 수 있는 픽셀이 없을 때(벽에 코를 박음·렌즈 가림) 허용하는 후진 속도. 전진은 막고 회전은 허용
    gate_drive: bool = True              # 엄격 모드(require_calibration)에서 깊이 영상이 끊기면 구동·준비 판정을 막을지. False 면 깊이는 선택 센서 — 피아식별·거리 측정에만 쓰고(항법 장애물은 2D LiDAR) 영상이 끊겨도 로봇은 멈추지 않고 준비 완료 판정에서도 빠진다
    # ---- 시뮬 전용
    sim_hz: float = 5.0            # 0 이면 시뮬 로봇에 깊이 카메라가 없다
    sim_noise_k: float = 0.004     # 깊이 표준편차 = k·z² (스테레오 특성; 3 m 에서 약 3.6 cm) [가정]
    sim_dropout: float = 0.03      # 무작위로 무효가 되는 픽셀 비율
    sim_flying_pixels: float = 0.30   # 깊이 불연속(물체 가장자리) 픽셀이 앞/뒤 사이 값으로 번지는 확률(스테레오의 '날아다니는 점')
    sim_mount_err_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # '숨은 정답': 깊이 카메라 실제 설치와 RigCfg 의 차이
    sim_mount_err_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def intrinsics(self, width: Optional[int] = None, height: Optional[int] = None):
        """주어진(없으면 설정) 해상도의 내부 파라미터. K 가 있으면 K 해상도에서 비례 축소한다."""
        from .geom3d import Intrinsics
        w, h = int(width or self.width), int(height or self.height)
        if self.K:
            base = Intrinsics.from_K(self.K, self.K_width or self.width, self.K_height or self.height)
            return base if (w, h) == (base.width, base.height) else base.scaled(w, h)
        return Intrinsics.from_hfov(w, h, self.hfov_deg)


@dataclass
class Map3DCfg:
    """3D 복셀 지도(perception/voxel_map.py)와 임무에서의 사용(mission/manager.py). 깊이 영상 + 자세(오도메트리·SLAM) + 카메라 자세로 쌓는다."""
    enable: bool = True
    res: float = 0.05              # m (2D 점유격자와 같은 격자라 서로 겹쳐 쓸 수 있다)
    z_max: float = 0.80            # m 이 높이 위는 지도에 넣지 않는다(경기장 벽·박스 0.4 m, UGV 0.32 m)
    floor_z: float = 0.05          # m 이 높이 아래 점은 바닥으로 보고 장애물로 쓰지 않는다(공간은 비운다)
    integrate_hz: float = 5.0      # 지도 갱신 주기 상한(깊이 프레임은 더 자주 와도 이 주기로만 쌓는다)
    stride: int = 2                # 깊이 영상을 이 간격으로 솎아 광선을 쏜다(프레임마다 위치를 돌려 stride² 프레임이면 전 픽셀 사용)
    max_free_range: float = 3.0    # m 광선을 따라 '비움'을 칠하는 최대 거리
    hit: float = 0.8               # 로그오즈 증분(점유 관측) — 2회 관측이면 occ_thr 를 넘는다
    free: float = -0.3
    l_min: float = -2.0
    l_max: float = 4.0
    occ_thr: float = 1.0
    # ---- 임무 사용
    use_for_nav: bool = True       # 로봇 키 높이대에 있는 점유를 2D 점유격자(내장 A*·Nav2 폴백)와 Nav2 점군에 더한다
    nav_z_lo: float = 0.06         # m 차체가 부딪히는 높이대(바닥 잡음보다 위 ~ 마스트 끝)
    nav_z_hi: float = 0.34
    nav_thr: float = 1.8           # 항법에 쓰는 점유 임계(로그오즈, 약 3회 관측) — 스테레오 허상(날아다니는 점)이 길을 막지 않게 2D 보다 엄격
    nav_min_neighbors: int = 3     # 이웃(6방향) 복셀이 이 수 이상 점유여야 쓴다 — 면/덩어리는 통과하고 광선 방향으로 늘어선 허상(스테레오 번짐)은 거른다
    blind_grow_cells: int = 3      # 'LiDAR 가 보는 곳' 마스크를 이 칸 수만큼 부풀린다(박스·벽 둘레로 번진 잡음 제거)
    blind_floor_layers: int = 2    # 'LiDAR 가 못 보는 장애물'은 바닥 바로 위 이 층 수(z_lo~)에 점유가 있는 것만(≈ 높이 5~15 cm 의 바닥 물체)
    nav_only_invisible: bool = True    # True: 'LiDAR 평면에 안 걸리는' 장애물만 더한다(박스·UGV·벽은 LiDAR 격자에 맡김). False: 3D 점유 전부
    lidar_plane_half: float = 0.05     # m LiDAR 평면 두께(±): 이 높이대에 점유가 있으면 LiDAR 가 보는 것으로 본다
    guard_enable: bool = True      # 전방 깊이 안전 정지: LiDAR 평면 밖의 장애물이 코앞에 있으면 감속·정지
    guard_dist: float = 0.22       # m 이 안에 있으면 전진 금지(LiDAR 안전 정지 stop_dist 보다 약간 큼 — 중복이 아니라 평면 밖 보강)
    guard_slow: float = 0.50       # m 이 안에서는 감속
    guard_half_width: float = 0.15
    use_range: bool = True         # LiDAR 가 거리를 못 주는 검출에 깊이로 거리를 준다(표적·UGV 거리 → 시차 보정·사거리 판단)
    beam_check: bool = True        # 발사 전: 레이저 발광부→수광센서 선분에 3D 지도상 장애물이 있으면 보류
    beam_margin_m: float = 0.10    # 센서 쪽 이 거리는 표적 자신의 면이라 검사에서 뺀다
    beam_min_blocked: int = 1      # 선분이 지나는 '표면 복셀'(이웃 nav_min_neighbors 개 이상 점유)이 이 수 이상이면 막힘 — 깊이는 앞면 한 겹만 보인다
    beam_timeout_s: float = 3.0    # 막힘이 이 시간 이어지면 허상으로 보고 통과시킨다(허상 때문에 M2 를 통째로 잃지 않게). 통계에 센다
    height_check: bool = True      # 수광센서의 깊이 기반 높이가 그 종류의 기대 높이대 밖이면 오검출로 보고 발사 보류
    target_window_z: Tuple[float, float] = (0.22, 0.46)   # 지정표적의 하얀 원판 높이대 [PDF p12 사진에서 짐작: 약 0.30~0.37]
    ugv_receiver_z: Tuple[float, float] = (0.04, 0.32)    # 상대 UGV 수광 돔 높이대 [PDF p21 사진에서 짐작]
    # ---- 접촉(정체) 감지: 깊이는 전방 고정 시야·최소거리 0.30 m 라 옆으로 스치거나 가까이서 마주한 낮은 물체를 못 본다. 그때는 '앞으로 가라는데
    #      SLAM 자세가 안 변한다'(+LiDAR 앞이 비었다)로 접촉을 알아채고, 그 자리를 장애물로 기억해 계획에 알린 뒤 RECOVER 한다. 깊이가 없어도 동작한다.
    bump_enable: bool = True
    bump_window_s: float = 1.5     # 이 시간 내내 전진 명령(bump_v_min 이상)이었는데
    bump_v_min: float = 0.08       # m/s
    bump_ratio: float = 0.3        # 실제 이동이 기대 이동(∫v dt)의 이 비율 미만이면 정체
    bump_lidar_clear_m: float = 0.5    # 앞 ±25° 의 LiDAR 거리가 이보다 가까우면 LiDAR 가 보는 장애물이라 기존 로직(안전 제한·RECOVER)에 맡긴다
    bump_ahead_m: float = 0.22     # 접촉 장애물을 차체 중심 앞 이 거리에 표시(폭 0.30 m × 깊이 0.15 m)
    bump_ttl_s: float = 90.0       # 기억 유지 시간(그 뒤엔 다시 시도할 수 있게 지움 — 움직이는 물체였을 수도 있다)
    smog_max_range: float = 1.5    # m 스모그 중에는 스테레오가 흐려져 이 거리 안만 쓴다
    sync_max_dt: float = 0.25      # s RGB 프레임과 깊이 프레임 시각 차가 이 안일 때만 짝지어 쓴다
    lidar_check: bool = True       # 깊이↔LiDAR 정합 감시(카메라 자세 오차 조기 발견)
    lidar_check_bias_m: float = 0.08   # 중앙값 잔차가 이를 넘으면 경고


@dataclass
class LaserCfg:

    ammo: int = 8                  # [PDF] 탄창 8발
    shot_s: float = 5.0            # [PDF] 1회 발사 5초 발광
    cooldown_s: float = 5.0        # [PDF] 재발사 대기 5초
    dwell_s: float = 2.0           # [PDF] 2초 이상 연속 수광 = 유효타격
    disabled_s: float = 10.0       # [PDF] 피격 후 10초간 레이저·틸트 정지
    # 연습 모드에서는 탄약이 자동 회복된다(p70). 추정 탄약으로 사격을 막지 않고, empty 응답이 오면 이 시간 뒤에 다시 시도한다 [미측정]
    practice_empty_backoff_s: float = 10.0
    max_range: float = 2.5         # m [가정] 수광센서 유효거리 — 실측 필수
    min_range: float = 0.35        # m [가정]
    # 시뮬 전용 '숨은 정답': 레이저 발광부가 카메라에서 떨어진 위치(m). aim.laser_lateral_m/vertical_m 이 이 값을 보정해야 맞는다
    sim_origin_lateral_m: float = 0.0
    sim_origin_vertical_m: float = 0.0


@dataclass
class AimCfg:
    kp_pan: float = 0.9            # 이미지 오차(각도) → 팬 증분 비율 (1.0 = 한 틱에 오차 전부 보정)
    kp_tilt: float = 0.9
    body_kp: float = 1.6           # 팬 각도 → 차체 회전 속도(rad/s per rad): 팬을 0으로 되돌리는 방향
    tol_deg: float = 1.2           # 발사 허용 조준 오차(수평)
    tol_deg_far: float = 0.7       # 먼 표적일수록 엄격
    hold_before_fire: float = 0.20  # s: 오차가 tol 안에서 이만큼 안정되면 발사
    # /mil/gimbal 의 ok 는 도착이 아니라 접수다(p72). settle_move_deg 보다 크게 움직인 마지막 짐벌 명령 뒤로 이 시간이 지나기 전에는 발사하지 않는다.
    # 0.0 = 기존과 같음(hold_before_fire 와 AND). 현장에서 0 / 0.3 / 0.5 를 비교해 정한다 — waveshare_real.json 에는 아직 넣지 않았다.
    gimbal_settle_s: float = 0.0
    settle_move_deg: float = 1.0
    lost_timeout: float = 0.8      # s: 이 시간 보이지 않으면 추적 해제(필터·직전 겨냥점 초기화)
    target_jump_px: float = 120.0  # px: 직전 겨냥점에서 이보다 멀리 뛰면 다른 대상으로 보고 추적을 새로 시작
    # bbox 안에서 '수광부'가 있는 위치(상대좌표 fx, fy). [가정] 현장 사진으로 교정
    aim_point_target: Tuple[float, float] = (0.5, 0.16)   # 지정표적: 면 상단 중앙의 흰 원판 (PDF p12 사진 기준; 센서를 못 찾을 때만 쓰는 대체)
    aim_point_ugv: Tuple[float, float] = (0.5, 0.78)      # 상대 UGV: y(하단)만 사용, x는 좌/우 센서 쪽(aim.py)
    # ---- 수광센서(원)를 직접 겨냥한다 (perception/receiver.py). 위의 aim_point_* 는 센서를 못 찾을 때의 대체 추측이다
    require_sensor: bool = True         # 센서를 찾은 상태에서만 발사. 못 찾으면 sensor_fallback_s 뒤에 추측 지점으로 발사 허용
    sensor_fallback_s: float = 2.0
    sensor_radius_frac: float = 0.6     # 허용 오차 = 센서 반경의 이 비율(빔이 원 안에 머물게). 반경을 모르면 쓰지 않는다
    sensor_min_tol_deg: float = 0.30    # 위 허용 오차의 하한. [PDF p14] 팬 1단위 = 0.5° → 양자화 오차 ±0.25° 라 이보다 작게 둘 수 없다(틸트는 0.15°)
    laser_lateral_m: float = 0.0        # 레이저 발광부가 카메라보다 왼쪽으로 떨어진 거리(m, 왼쪽 +). 시차 보정 — 실측 필수 [미측정]
    laser_vertical_m: float = 0.0       # 레이저 발광부가 카메라보다 위로 떨어진 거리(m, 위 +). 〃
    laser_offset_pan_deg: float = 0.0   # 카메라 축 대비 레이저 축 수평 오프셋 [가정] 현장 교정
    laser_offset_tilt_deg: float = 0.0  # 〃 수직 오프셋(레이저가 카메라 위에 있으면 +)
    # 같은 검출(프레임)의 영상 오차를 틱마다 다시 더하지 않는다. 오차는 그 검출을 찍은 순간의 팬틸트 기준으로 한 번만 쓰고,
    # 다음 검출이 올 때까지 같은 명령을 유지한다(그 사이 차체가 돈 만큼만 보정). False = 예전 동작(검출이 남아 있는 동안 매 틱 갱신).
    once_per_detection: bool = True


@dataclass
class LidarErrCfg:
    """LiDAR 불확실도 예산(**가정값** — PDF 에 LiDAR 정확도 수치가 없다). 실측 전에는 assumed 로만 기록하고 실측값으로 표시하지 않는다.
    값은 calib/robot_profile.yaml 의 sensors.lidar.error 에서 오고(calib/generate.py), footprint.py 의 ClearanceBudget 이 계층별 여유로 환산한다.
    sim_* 은 시뮬이 쓰는 숨은 진짜 오차이며 위의 가정 예산과 별개다(예산이 실제 오차를 덮는지 시험하려고 따로 둔다)."""
    range_bias_m: float = 0.005       # 거리 편향 (+ = 멀게 읽음)
    range_sigma_m: float = 0.008      # 거리 잡음 1σ (현 시뮬 잡음 0.008 m 와 같은 값을 가정으로 채택)
    angle_sigma_deg: float = 0.3      # 빔 각도 오차 1σ
    mount_xy_m: float = 0.005         # 장착 위치 오차(캘리브레이션 잔차)
    mount_yaw_deg: float = 0.5        # 장착 yaw 오차
    latency_s: float = 0.10           # 측정→수신 지연
    pose_sigma_m: float = 0.010       # 자기 위치추정 오차 1σ (시뮬 단독 주행 중앙값 약 1.6 cm — README)
    dropout_p: float = 0.02           # 누락 확률 (거리 여유로 메우지 않고 지속성 필터로 다룬다)
    outlier_p: float = 0.005          # 이상치 확률
    k_sigma: float = 2.0              # 확률 오차를 여유로 바꿀 때의 포괄 계수
    ref_range_m: float = 0.5          # 각도 오차를 옆 방향 오차로 바꿀 기준 거리
    status: str = "assumed"           # "assumed" | "measured" — 실측으로 갱신하면 generate.py 가 measured 로 바꾼다
    source: Dict[str, str] = field(default_factory=lambda: {k: "assumed (PDF 에 없음; 현장 정지 표적 반복 측정 필요)" for k in
                                                           ("range_bias_m", "range_sigma_m", "angle_sigma_deg", "mount_xy_m", "mount_yaw_deg",
                                                            "latency_s", "pose_sigma_m")})
    # ---- 시뮬 숨은 진짜 오차 (기본값은 이전 시뮬 동작과 같다: 편향 0, σ 0.008, 누락 1 %)
    sim_range_bias_m: float = 0.0
    sim_range_sigma_m: float = 0.008
    sim_range_sigma_per_m: float = 0.0   # 거리 비례 잡음 계수 (σ = sigma + per_m·r)
    sim_angle_sigma_deg: float = 0.0
    sim_dropout_p: float = 0.01
    sim_outlier_p: float = 0.0
    sim_latency_s: float = 0.0
    sim_pose_sigma_m: float = 0.0        # 시뮬 odom 에 얹는 위치 오차(스캔마다 가우시안) — 위치추정 오차 시험용


def _default_budget():
    from .footprint import budget_from
    return budget_from(RobotCfg(), LidarErrCfg(), 0.38)


_B = _default_budget()


@dataclass
class NavCfg:
    res: float = 0.05              # 격자 해상도 m
    # inflate_hard / rotate_clear / side_lim / stop_dist 는 footprint.py 의 ClearanceBudget 에서 *파생*한 값이다(기본값 = 파생값).
    # 직접 바꾸면 Config.consistency() 가 어긋남을 알려 준다. 근거 표: ClearanceBudget.table()
    inflate_hard: float = round(_B.plan_inflate_straight, 3)   # A* 하드 인플레이션 = 반폭 0.115 + 계획 여유 (직진 통과 기준). 경로 확정은 직사각형 검증(footprint_check)
    rotate_clear: float = round(_B.plan_inflate_rotate, 3)     # 제자리 회전에 필요한 자유 반경 = 외접 0.1698 + 계획 여유
    side_lim: float = round(_B.stop_side, 3)                   # 안전 제한의 측면 접근 보정 시작 거리 = 반폭 + 정지 여유
    footprint_check: bool = True   # 경로 확정 전 직사각형 검증(좁은 구간 방향·회전 공간). False 면 레거시(중심점 경로만)
    relax_inflation: bool = False  # True 면 레거시: 경로를 못 찾으면 인플레이션을 0.8·0.6 배로 줄여 다시 찾는다 — 차체보다 작은 여유라 기본 금지
    inflate_soft: float = 0.34     # 이 거리까지는 비용 가산(중앙 선호)
    soft_weight: float = 4.0
    lookahead: float = 0.38
    v_cruise: float = 0.38         # m/s
    v_slow: float = 0.18
    stop_dist: float = round(_B.stop_front, 3)   # 전방 이 거리(센서→장애물) 안 → 정지/후진. = 앞 반길이 0.125 + 정지 여유(센서 오차+지연)
    slow_dist: float = 0.45
    replan_s: float = 0.2
    goal_tol: float = 0.12
    stuck_s: float = 3.0
    stuck_move: float = 0.05
    backend: str = "builtin"       # "builtin"(내장 SLAM+A*) | "nav2"(ROS2 SLAM + Nav2) | "nav2-fake"(시뮬 시험용 Nav2 대역)
    replan_on_change: bool = True  # 경로 근처에 새 장애물이 생기면 주기(replan_s)를 기다리지 않고 즉시 재계획
    slam: bool = False            # 내장 백엔드에서 만든 지도(박스)를 위치추정 우도에도 쓸지. 시뮬 실측: 외벽만 쓸 때 오차 중앙값
                                   # 0.4 cm, 켜면 2 cm(자기강화). 벽이 박스에 많이 가려지는 배치에서만 켜 볼 것. 실기는 backend="nav2".


@dataclass
class Nav2Cfg:
    """backend="nav2" 용 ROS2 연결 설정. 이름은 ros2/ 폴더의 launch·yaml과 일치해야 한다. [가정] 실기 확인 필요."""
    action_name: str = "navigate_to_pose"
    allow_builtin_fallback: bool = True  # False: stop/report instead of silently changing a Nav2 trial's backend.
    map_topic: str = "/map"                # SLAM(Cartographer/slam_toolbox)이 내는 nav_msgs/OccupancyGrid
    cmd_topic: str = "/cmd_vel_nav2"       # Nav2가 내는 속도(/cmd_vel 이 아님 — Agent가 안전 제한 후 대신 보낸다)
    map_frame: str = "map"
    base_frame: str = "base_link"
    cmd_timeout: float = 0.5               # s: 이보다 오래된 Nav2 속도는 0으로 본다
    occ_thresh: int = 65                   # 지도 값 ≥ 이 값 → 점유
    free_thresh: int = 25                  # 0 ≤ 값 ≤ 이 값 → 자유
    map_refresh_s: float = 1.0             # 경기장 좌표 격자로 다시 투영하는 주기
    max_resend: int = 2                    # Nav2가 '성공'했지만 목표 허용오차 밖일 때 다시 보내는 횟수
    progress_timeout_s: float = 8.0       # active/recovery가 계속되며 실제 병진이 없을 때 기존 안전 A* 폴백
    progress_min_m: float = 0.08          # 목표 전환과 무관하게 이 거리 이상 이동해야 무진행 타이머 리셋
    server_wait_s: float = 5.0             # 시작 시 액션 서버를 기다리는 시간
    grid_source: str = "scan"              # Agent 용 점유격자(박스 탐색)를 무엇으로 만들까: "scan"(LiDAR 로그오즈, 박스가 옮겨지면 빨리 갱신)
                                           # | "slam_map"(SLAM /map 투영 — 서브맵 누적이라 옮겨진 박스가 한동안 남을 수 있음)
    publish_walls: bool = True             # 경기장 외벽을 Nav2 전역 비용지도에 마킹 전용 점군으로 알려줄지
    walls_topic: str = "/arena_walls_cloud"
    publish_depth_obstacles: bool = True   # 깊이 3D 지도에서 'LiDAR 가 못 보는' 장애물(낮은 물체 등)을 Nav2 비용지도에 마킹 전용 점군으로 알릴지
    depth_obstacles_topic: str = "/depth_obstacles_cloud"
    keepout_mask_topic: str = "/own_base_keepout_mask"
    keepout_info_topic: str = "/own_base_keepout_info"
    use_sim_time: bool = False             # ROS 시뮬 연동(sim/ros_bridge.py)에서 /clock 을 따를 때 True
    start_pose_override: Optional[Tuple[float, float, float]] = None  # Gazebo 처럼 출발 자세를 정확히 아는 시험에서만 사용


@dataclass
class PerceptionCfg:
    conf_min: float = 0.40
    ugv_conf_min: float = 0.50
    object_merge_dist: float = 0.22   # m: 같은 면의 같은 객체로 합치는 거리
    min_votes: int = 4
    pose_extrap_max_s: float = 0.0    # 센서 촬영 시각이 자세 이력 끝보다 이만큼(s) 이내로 앞서면 마지막 속도로 외삽한다(틱이 느려 이력이 늦을 때 자세를 옛 값에 묶어 광선이 틀어지는 것을 막는다). 0=끔(가장자리 자세로 대체)
    min_box_px: float = 4.0           # 검출 상자 너비·높이가 이보다 작으면 버린다(maicon26/validate.py V5)
    max_obj_range: float = 2.2        # m: 이보다 먼 객체 인식은 집계에 넣지 않음(오분류 방지)
    target_classes: Tuple[str, ...] = ("GUARD_POST", "RADAR_FACILITY", "COMM_FACILITY")
    object_classes: Tuple[str, ...] = ("AMMO_BOX", "DRUM", "MORTAR", "TRUCK", "TANK", "MISSILE")
    # 비인식용 박스 4종(도시 나무·차단벽·전진지·건물, 이름은 멘토링 전사 기준 — 정확한 이름 미확인). 세지 않고 무시한다(통계만).
    nonobject_classes: Tuple[str, ...] = ("TREE", "BARRIER", "POSITION", "BUILDING")
    # 시뮬 전용: 지정표적 검출 bbox 가 무엇을 덮는가. "face"=면 전체(수광 원판이 bbox 안 상단 중앙), "object"=사진 속 시설 물체만
    # (PDF p12 사진에서 흰 원은 물체 위 하늘에 있어 bbox 밖). 실제 YOLO 라벨링 방식은 아직 모른다 → 센서를 영상에서 직접 찾는다.
    sim_target_box: str = "face"


@dataclass
class LinkCfg:
    """서버 링크 입력 정리(9장 V6)."""
    max_command_chars: int = 500       # tactical_command 최대 길이. 제어문자를 빼고 이 길이로 자른 뒤 LLM 에 넘긴다(잘랐으면 경고)


@dataclass
class OakCfg:
    """OAK-D Lite 드라이버 기동·토픽(6-1, 6-2장). 환경변수 MAICON_OAK=1 일 때만 team_node 가 띄운다. 값은 모두 [실기 확인]."""
    launch_cmd: Tuple[str, ...] = ("ros2", "launch", "depthai_ros_driver", "camera.launch.py")   # [실기 확인] 패키지 이름
    launch_args: Tuple[str, ...] = ()                       # 예: ("camera_model:=OAK-D-LITE",)  [실기 확인]
    params_file_arg: str = "params_file"                     # 파라미터 파일 런치 인자 이름 → "params_file:=<경로>"  [실기 확인]
    profile: str = "rgb_lowpower"                            # config/oak/<profile>.yaml (rgb_lowpower | rgbd)
    rgb_topic: str = "/oak/rgb/image_raw"                    # [실기 확인] ros2 topic list | grep oak
    rgb_info_topic: str = "/oak/rgb/camera_info"             # [실기 확인]
    check_topics_after_s: float = 8.0                        # 기동 뒤 이 시간이 지나면 토픽이 있는지 본다(없으면 oak 토픽 목록을 경고로)
    start_grace_s: float = 5.0                               # 이 안에 프로세스가 끝나 버리면 경고만 하고 계속한다
    # --- 기본 꺼짐: OAK 를 평소엔 멈춰 두고 식별 단계에서만 켠다(드라이버의 start/stop 서비스, std_srvs/Trigger). 서비스가 안 보이면 켜 둔 채로 쓴다. 기본 꺼짐(실기 설정에서 켠다)
    power_gating: bool = False
    start_srv: str = "/oak/start_camera"                     # [실기 확인] ros2 service list | grep oak
    stop_srv: str = "/oak/stop_camera"                       # [실기 확인]
    wake_dist_m: float = 2.0                                 # m 식별 지점까지 이 거리 안으로 들어오면 켠다(켜지는 시간을 이동 중에 번다)
    off_delay_s: float = 4.0                                 # s 식별이 끝난 뒤 이 시간 안에 다음 식별 지점이 없으면 끈다(켰다 껐다 반복을 막는다)
    start_timeout_s: float = 8.0                             # s 켠 뒤 이 시간이 지나도 새 프레임이 없으면 카메라 문제로 본다
    power_retry_s: float = 2.0                               # s 켜기·끄기 명령이 확인되지 않으면 이 간격으로 다시 보낸다


@dataclass
class OakDetCfg:
    """OAK 컬러 영상으로 하는 보조 탐지(6-3~6-5장). 기본은 꺼짐. 조준·사격에는 절대 쓰지 않는다."""
    enable: bool = False
    conf: float = 0.35                                       # OAK 검출 신뢰도 기준 [미측정: 현장에서 팬틸트와 비교]
    max_hz: float = 3.0                                      # OAK 검출 최대 빈도
    pause_in_strike: bool = True                             # STRIKE(조준·사격) 중에는 OAK 를 처리하지 않는다
    vote_window: Optional[int] = None                        # 투표 창(프레임). None = 팬틸트 쪽과 같은 규칙(창 없음, perception.min_votes 만) [비교 후보: 8 / 16]
    weight: float = 0.5                                      # 두 카메라 관측을 합칠 때 OAK 관측의 투표 가중치
    submit_alone: bool = False                               # OAK 로만 본 트랙을 M3 에 낼지
    save_max: int = 8                                        # 확인용 링 버퍼 최대 장수(0..10, 0 = 저장 안 함)
    save_every_s: float = 2.0                                # 저장 최소 간격
    cleanup_every_s: float = 30.0                            # 주기 정리 간격
    save_dir: Optional[str] = None                           # None = 실행 로그 폴더(저장소 밖) 아래 oak_samples
    # --- 식별 단계(미식별 면을 OAK RGB + YOLO 로 다시 본다). 모두 기본 꺼짐 — 실기 설정에서 켠다.
    on_demand: bool = False                                  # OAK 검출(YOLO)을 식별 단계에서만 돌린다(그 밖에는 프레임을 검출기에 넣지 않는다). 드라이버 노드는 계속 떠 있다
    identify_fallback: bool = False                          # 팬틸트로 못 푼 면 앞으로 가서 OAK 로 식별하는 후보('identify')를 탐색에 넣는다
    identify_submit: bool = False                            # 식별 단계에서 OAK 로 본 면은 팬틸트 관측 없이도 M3 로 제출(submit_alone 과 별개로, 식별 단계 관측분만)
    identify_min_attempts: int = 2                           # 팬틸트가 그 면에 이만큼 가 보고도 못 풀었을 때부터 OAK 식별 대상
    identify_max_attempts: int = 2                           # 같은 면을 OAK 로 식별하러 가는 최대 횟수
    identify_dist_m: float = 0.8                             # m 면 앞에서 떨어질 거리(OAK RGB 의 시야에 면 하나가 들어오는 거리) [현장 확인]
    identify_max_range_m: float = 1.3                        # m 가시성 검사 최대 거리(OAK 는 팬틸트보다 해상도가 낮다고 보고 짧게)
    identify_dwell_s: float = 2.5                            # s 면을 향해 정렬한 채 머무는 최소 시간
    identify_min_frames: int = 4                             # 식별을 끝내려면 이 시간 동안 처리된 OAK 프레임이 이만큼 필요(없으면 timeout)
    identify_timeout_s: float = 7.0                          # s 이 시간이 지나면 프레임이 모자라도 이번 시도를 끝낸다
    identify_weight: float = 1.0                             # 식별 단계의 OAK 관측 투표 가중치(평소 weight 0.5)
    identify_gain: float = 0.9                               # 식별 후보 하나의 정보 가치(면 1개 = 1.0)
    # --- 처음부터 OAK(Depth 카메라)로 식별: 팬틸트는 OAK 가 못 보는 면에서만 쓴다. 기본 꺼짐 — 실기 설정에서 켠다.
    identify_primary: bool = False                           # 면 식별을 OAK 가 먼저 한다. 팬틸트 카메라의 M3 면 관측·득표는 '폴백으로 넘어온 면'에서만 받는다(OAK 를 못 쓰면 전부 팬틸트)
    identify_edge_margin_px: int = 8                         # px 검출 상자가 영상 가장자리에서 이 안이면 '잘렸다'(ObjectMap 은 6 px 안을 득표에서 뺀다)
    identify_crop_frames: int = 2                            # 한 번의 식별에서 잘린 검출만 이만큼 보이고 온전한 검출이 없으면 폴백(cropped)
    identify_fail_limit: int = 2                             # OAK 가 프레임을 못 준 식별이 이만큼 이어지면 OAK 를 포기하고 모든 면을 팬틸트로 본다
    identify_noview_s: float = 6.0                           # s OAK 로 볼 수 있는 자리가 없는데 팬틸트로는 볼 수 있는 면이 이만큼 이어지면 폴백(no_oak_view)
    fit_hfov_deg: float = 69.0                               # deg OAK RGB 가로 화각(K 를 받기 전·면 전체가 시야에 들어오는 최소 거리 계산용) [실기 확인]
    fit_vfov_deg: float = 42.0                               # deg OAK RGB 세로 화각(16:9 스트림 기준; 4:3 이면 54) [실기 확인 — 스트림 해상도]
    fit_usable: float = 0.92                                 # 면 그림이 시야의 이 비율 안에 들어와야 '온전히 보인다'
    # --- OAK Depth 로 상대 UGV 까지 거리·피아식별. 기본 꺼짐 — 실기 설정에서 켠다. 조준·사격에는 쓰지 않는다.
    ugv_only: bool = False                                   # OAK 검출 중 UGV 만 쓴다(트랙·피아식별). 면 득표(물체 지도)에는 아무것도 넣지 않는다
    depth_range: bool = False                                # UGV 상자 안의 깊이로 거리를 잰다(LiDAR·크기 추정보다 먼저). 깊이 프레임이 없으면 기존 순서 그대로
    depth_max_dt_s: float = 0.35                             # s 검출 프레임과 깊이 프레임의 촬영 시각 차 상한(UGV 가 천천히 움직이므로 느슨하게)
    depth_roi_frac: float = 0.50                             # 상자 가운데 이 비율(가로·세로)만 본다 — 가장자리는 배경이 섞인다
    depth_min_pixels: int = 12                               # 이보다 적은 유효 깊이 픽셀이면 쓰지 않는다
    depth_fg_mass: float = 0.25                              # 가장 가까운 깊이 덩어리가 유효 픽셀의 이 비율 이상이면 그것을 UGV 로 본다(뒤 배경 제외)
    depth_bin_m: float = 0.05                                # m 깊이 히스토그램 간격
    depth_agree_tol: float = 0.45                            # 깊이 거리와 크기 추정이 이 비율 넘게 다르면 깊이를 버린다(상자에 다른 물체·바닥이 들어옴)
    depth_aligned_to_rgb: bool = True                        # 깊이가 컬러에 정렬돼 있다(config/oak/rgbd.yaml: i_align_depth). False 면 깊이 거리를 쓰지 않는다
    ugv_width_m: Tuple[float, float] = (0.08, 0.70)          # m 깊이로 잰 상자 폭의 허용 범위(UGV 약 0.2~0.4 m). 영상 가장자리에 걸치지 않았는데 벗어나면 UGV 가 아니다
    iff_max_range_m: float = 2.2                             # m 이보다 먼 UGV 는 안테나 색(피아)을 믿지 않는다 — 깊이로 거리를 잴 수 있을 때만 적용
    iff_weight_boost: float = 1.5                            # 깊이로 거리를 확인한 가까운 UGV 의 피아 득표 가중(1 = 없음)


@dataclass
class ImpactCfg:
    """IMU 충돌 감지와 행동(mission/impact.py). 기본 꺼짐 — 실기 설정에서 켠다. 값은 모두 [가정, 현장 측정 필요]: 바닥에서 벽에 살살 부딪혀 trigger_dg 를 정한다."""
    enable: bool = False
    trigger_dg: float = 0.45           # g 기준선에서 이만큼 벗어난 가속도를 충돌로 본다(바닥 진동 잡음의 5배 문턱과 큰 쪽을 쓴다)
    major_dg: float = 1.2              # g 이 이상 또는 yaw 각속도 major_gyro_dps 이상이면 큰 충격
    gyro_dps: float = 180.0            # deg/s yaw 각속도가 이 이상이면 가속도가 약해도 충격(자이로를 줄 때만)
    major_gyro_dps: float = 400.0
    noise_k: float = 5.0               # 문턱 = max(trigger_dg, noise_k × 최근 평균 편차)
    refractory_s: float = 0.35         # s 이 안의 후속 표본은 같은 충돌
    hold_s: float = 0.40               # s 충돌 직후 정지(작은 충격 최소 정지)
    hold_major_s: float = 0.80         # s 큰 충격 최소 정지
    verify_timeout_s: float = 2.0      # s 충격 후 LiDAR 틀어짐 검증을 기다리는 최대 시간(그 뒤엔 검증 못 한 채 계속)
    backoff: bool = True               # 앞에서 부딪혔고 주행 중이었으면 RECOVER(후진 후 트인 쪽으로 회전)로 빠져나간다
    contact_lidar_m: float = 0.35      # m 앞 LiDAR 가 이보다 가까운 물체를 보고 있으면 '보이는 장애물과 접촉'(보이지 않으면 기록해 둔다 — 낮은 물체)
    ugv_contact_m: float = 0.90        # m 접촉 방향 이 거리 안에 UGV 트랙이 있으면 UGV 와의 접촉으로 본다(후진·회전은 하되 적 쪽으로 돌지 않는다)
    storm_n: int = 3                   # 이 횟수의 충돌이
    storm_window_s: float = 10.0       # s 이 시간 안에 오면 '연속 충돌' — 정지를 늘리고 목표·경로를 다시 정한다
    storm_hold_s: float = 1.5          # s 연속 충돌일 때 정지 시간
    scan_skip_s: float = 0.35          # s 충돌 직후 이 시간의 스캔은 Agent 격자에 쌓지 않는다(차체 흔들림으로 바닥·천장을 훑는다)
    tilt_scan_skip_deg: float = 8.0    # deg |roll|·|pitch| 가 이보다 크면 그 스캔은 격자에 쌓지 않는다
    vib_filter: bool = False           # 바퀴·바닥 진동 거르기: 덩어리의 알짜 충격량(부호 일치도)이 낮으면 충돌로 보지 않는다(Gazebo 에서 진동 0.98 g 가 충돌로 오인된 문제). 켜려면 IMU 에 ax·ay 가 있어야 한다
    vib_window_s: float = 0.12         # s 덩어리가 시작된 뒤 이 시간 동안 수평 가속도를 모아 판정한다 — 충돌 정지가 이만큼 늦어진다
    vib_min_unipolar: float = 0.45     # |Σ a·dt| / Σ|a|·dt 가 이 값 미만이면 진동. 한쪽 펄스(충돌)는 높고 번갈아 바뀌는 진동은 0 에 가깝다 [가정, 실측 필요]
    vib_min_samples: int = 4           # 이 표본 수 미만이면 판단하지 않고 그대로 충돌로 낸다(IMU 가 느린 경우)


@dataclass
class PoseGuardCfg:
    """자세 감시(nav/pose_guard.py): LiDAR 틀어짐·SLAM 도약·경기장(UAV) 좌표 연결 점검. 기본 꺼짐 — 실기 설정에서 켠다. 값은 [가정, 현장 측정 필요]."""
    enable: bool = False
    # ---- LiDAR 장착 yaw 틀어짐
    shift_detect_deg: float = 2.5      # deg 충격 전후 (SLAM 헤딩 − 차체 헤딩) 가 이만큼 달라지면 틀어짐으로 본다
    shift_confirm_scans: int = 8       # 충격 후 정지 상태에서 모아야 하는 표본(스캔) 수
    shift_max_deg: float = 40.0        # deg 이보다 크면 보정하지 않는다(SLAM 이 길을 잃었거나 IMU 오류 가능) → 내장 A* 로 간다
    shift_compensate: bool = True      # 확정된 틀어짐을 자세 헤딩·스캔 각도에서 보정한다(False 면 감지·보고만)
    fallback_deg: float = 5.0          # deg 틀어짐이 이 이상이면 Nav2(TF 를 못 고친다) 대신 내장 A* 로 간다
    pre_window_s: float = 6.0          # s 충격 직전 기준 오프셋을 모으는 시간 창
    pre_std_deg: float = 1.5           # deg 기준 오프셋 표본의 허용 표준편차(넘으면 기준 없음)
    post_std_deg: float = 2.0          # deg 충격 후 표본의 허용 표준편차(넘으면 더 모으거나 포기)
    settle_s: float = 0.25             # s 충격 직후 이 시간은 표본을 모으지 않는다(차체가 흔들린다)
    quiet_rot_dps: float = 7.0         # deg/s 이보다 느리게 도는 동안의 표본만 오프셋에 쓴다(SLAM·오도메트리 시각 차 오차가 헤딩 오차로 바뀌지 않게)
    quiet_lin: float = 0.05            # m/s 충격 후 검증 표본은 이보다 느릴 때만(정지)
    verify_timeout_s: float = 2.0      # s 충격 후 검증 최대 시간
    step_trigger_deg: float = 6.0      # deg 충격 센서가 못 잡았어도 정지 중 오프셋이 이만큼 갑자기 바뀌면 가상 충격으로 본다
    # ---- 추측항법·도약
    dead_reckon_s: float = 3.0         # s TF 가 끊기거나 도약을 못 믿는 동안 IMU/바퀴 추측항법으로 이어 가는 최대 시간(넘으면 lost)
    jump_m: float = 0.45               # m 한 틱에 예측(이전 자세 + 오도메트리 변화)에서 이만큼 벗어난 SLAM 자세는 바로 믿지 않는다
    jump_deg: float = 20.0             # deg 같은 의미의 헤딩 기준
    jump_confirm_n: int = 4            # 도약 뒤 새 자리가 이만큼의 연속 표본에서 유지돼야 받아들인다
    # ---- 경기장(UAV) 좌표 연결 점검
    link_check: bool = True            # 벽·고정 구조물(+UAV 상자)에 LiDAR 끝점이 맞는지 주기 점검(보고·복구용). enable 가 켜져 있을 때만
    link_period_s: float = 1.0         # s 점검 주기
    link_rot_dps: float = 20.0         # deg/s 이보다 빨리 도는 동안은 점검하지 않는다
    link_lin: float = 0.25             # m/s 이보다 빠르면 점검하지 않는다
    link_search_xy: float = 0.15       # m 오프셋 탐색 범위
    link_search_deg: float = 6.0       # deg
    structures: str = "fixed"          # 적합에 쓰는 구조물 모델. "fixed"=저장된 고정 구조물 13개(시뮬 배치) · "uav"=외벽만 + 경기 시작 때 UAV 가 찍은 상자 윤곽(정렬 전 원좌표). 실제 경기는 구조물이 매번 달라 "uav"
    structures_min_boxes: int = 4      # structures="uav": 이 개수 이상 UAV 상자 윤곽이 들어오기 전에는 기준선을 만들지 않는다(벽만으로는 대칭·한 벽 시점을 못 가른다)
    link_baseline_min: float = 0.30    # 시작 때 벽·구조물에 맞는 비율이 이 이상이어야 기준선으로 삼는다(미만이면 'poor_start' 로 보고 — 도면 치수·시작 자세를 점검)
    link_lost_ratio: float = 0.12      # 이 아래면 연결 불량(bad)
    link_drop: float = 0.20            # 기준선보다 이만큼 나빠지면 연결이 약해진 것(weak)
    link_trim: bool = False            # 연결이 크게 나빠졌을 때 소폭 복구 보정(앵커 이동)을 한다. 평상시 절대 보정은 하지 않는다
    link_trim_gain: float = 0.12       # 보정 후 점수 회복 폭의 하한
    link_trim_max_m: float = 0.20      # m 누적 보정 한도
    link_trim_max_deg: float = 8.0     # deg
    link_degrades: bool = True         # 연결이 weak/bad 이면 상태를 degraded 로(속도 제한)
    degraded_speed_scale: float = 0.75 # degraded 일 때 전진 속도 배율
    # ---- 앵커 재설정(reanchor): SLAM 지도 → 경기장 좌표 앵커를 충돌·도약·연결 불량 뒤에 벽·고정 구조물로 다시 맞춘다. 기본 꺼짐.
    # SLAM 지도는 그대로 두고 '경기장 = 앵커 ∘ 지도' 의 앵커만 바꾼다(격자·벽·keepout 은 새 앵커로 다시 만든다). 시작 때의 기준선(도면 잔차)을 기준으로 상대 보정한다.
    reanchor: bool = False
    reanchor_on_impact: bool = True    # 충격 검증이 끝난 뒤(큰 충격이거나 틀어짐 확정·한도 초과·검증 실패)
    reanchor_on_jump: bool = True      # SLAM 도약을 받아들인 뒤
    reanchor_on_link: bool = True      # 연결이 weak/bad 로 reanchor_link_n 번 연속 점검될 때
    reanchor_link_n: int = 3
    reanchor_use_uav_boxes: bool = False  # 적합에 UAV 상자 윤곽(정렬 전 원좌표)도 쓴다. 기본은 외벽·고정 구조물만 — 상자 위치는 UAV 보정 오차·윗면 투영 오차가 있다
    reanchor_plan_spread_m: float = 0.06  # 자리마다 잰 도면 잔차의 퍼짐(m)이 이보다 크면 도면(구조물 좌표)을 믿을 수 없다고 보고 재설정하지 않는다(0 = 검사 끔)
    reanchor_require_plan_check: bool = False  # True 면 서로 1.5 m 이상 떨어진 자리 4 곳 이상에서 도면 잔차를 잰 뒤에만(그 전에는 도면을 믿을 근거가 없다) 재설정한다
    reanchor_ugv_exclude: bool = True  # 알려진 UGV(트래커·움직이는 LiDAR 덩어리) 주변 끝점을 적합에서 뺀다 — 구조물(외벽·고정 구조물)만으로 앵커를 맞춘다
    reanchor_ugv_radius_m: float = 0.30  # m UGV 한 대가 가리는 최소 반경(트래커가 준 반경이 더 크면 그것)
    reanchor_ugv_margin_m: float = 0.45  # m 추가 여유 — UGV 위치는 UAV 틀이고 스캔 끝점은 (어긋났을 수 있는) 앵커로 옮긴 것이라 그만큼 어긋난다
    reanchor_search_xy: float = 0.50   # m 기준선 오프셋 둘레 탐색 반경
    reanchor_search_deg: float = 20.0  # deg
    reanchor_min_ratio: float = 0.40   # 새 자리에서 벽·구조물에 설명되는 끝점 비율 하한
    reanchor_min_gain: float = 0.10    # 현재 앵커 대비 점수(0..1) 향상 하한
    reanchor_ambiguity: float = 0.06   # 다른 후보(0.12 m·4° 이상 떨어진 것)와의 점수 차 하한 — 미만이면 모호(대칭·반복 구조)로 거부
    reanchor_min_move_m: float = 0.04  # 이보다 작은 보정은 하지 않는다(잡음 따라가기 방지)
    reanchor_min_move_deg: float = 1.5
    reanchor_max_m: float = 0.60       # 한 번 보정 한도 — 넘으면 거부하고 보고만(SLAM 이 길을 잃었거나 지도가 망가졌을 수 있다)
    reanchor_max_deg: float = 25.0
    reanchor_total_m: float = 1.20     # 누적 한도
    reanchor_total_deg: float = 40.0
    reanchor_confirm_n: int = 3        # 정지 상태에서 서로 일치하는 적합이 이만큼 연속돼야 적용
    reanchor_ptp_m: float = 0.05       # 일치 판정(최대−최소)
    reanchor_ptp_deg: float = 2.0
    reanchor_window_s: float = 12.0    # 트리거 뒤 이 시간 안에 정지 표본을 얻지 못하면 포기한다
    reanchor_hold_s: float = 3.0       # 표본을 모으는 동안 멈춰 서 있는 최대 시간(주행 중에는 모으지 않는다)
    reanchor_cooldown_s: float = 15.0  # 적용 뒤 다음 시도까지 최소 간격
    reanchor_max_count: int = 4        # 한 경기 최대 적용 횟수
    reanchor_verify_n: int = 3         # 적용 뒤 확인 표본 수 — 맞음 비율이 나빠지면 앞 앵커로 되돌린다
    reanchor_verify_drop: float = 0.05 # 확인 때 맞음 비율이 적용 전 기대값보다 이만큼 이상 낮으면 원복


@dataclass
class UavBoxesCfg:
    """실기 UAV 상자 윗면 검출기(perception/uav_boxes_real.py). 실영상으로 검증한 값이 아니다 — 연습 시간에 scripts/uav_boxes_tool.py 로 맞춘다.
    enable 이 켜져 있어도 보정 파일(calibration)이 없거나 쓸 수 없으면 경고만 내고 검출기를 만들지 않는다(기존 동작 그대로)."""
    enable: bool = False
    calibration: Optional[str] = None  # UAV 보정 JSON(scripts/calibrate_uav_grid.py · calibrate_uav.py 결과). 저장소 루트 기준 상대 경로도 된다
    camera: str = "main"               # 이 보정이 맞는 카메라(서버 X-Camera). 다른 카메라 프레임은 쓰지 않는다
    top_color: str = "black"           # 윗면 색: black(규정 검정) | grey | both
    v_black_max: float = 45.0          # 검정 윗면 밝기(V) 상한 — 바닥 밝기에 비례한 문턱이 이보다 크면 이 값으로 자른다 [가정 — 실영상 확인]
    v_black_rel: float = 0.45          # 검정 문턱 = 이 비율 × 바닥 밝기 중앙값(조명이 어두우면 문턱도 내려간다)
    v_black_min: float = 12.0          # 검정 문턱의 하한(JPEG 잡음 위)
    min_floor_contrast: float = 1.6    # 바닥 밝기 / 검정 문턱이 이보다 작으면 윗면과 바닥을 가를 수 없다고 보고 그 프레임을 버린다
    grey_s_max: float = 60.0           # 회색 윗면(top_color=grey|both): 채도 상한
    grey_v_min: float = 75.0
    grey_v_max: float = 155.0
    ppm: int = 128                     # 윗면 편평화 해상도 [화소/m]
    fill_min: float = 0.82             # 0.4 m 칸에서 윗면 색 화소 비율의 하한(잡음·가림·0.05 m 보정 오차 허용; 0.7 이하는 옆면 번짐이 가짜 칸을 만든다)
    min_conf: float = 0.6              # 이보다 낮은 신뢰도의 칸은 내지 않는다(coverage.ingest_uav_boxes 문턱과 같다)
    min_top_m: float = 0.28            # 이보다 작은 덩어리(UGV·표식)는 윗면이 아니다
    max_extent_m: float = 3.2          # 이보다 큰 덩어리(벽·그림자)는 버린다
    tile_slack: float = 0.25           # 덩어리 크기 / 0.4 m 의 올림 여유(옆면 번짐이 칸을 하나 더 만들지 않게)
    extra_free: float = 0.15           # 칸으로 설명되지 않는 덩어리 면적 비율이 이만큼까지는 신뢰도를 깎지 않는다(그 위로는 0.5 배로 깎는다)
    edge_margin_m: float = 0.30        # 벽에서 이 거리 안의 중심은 상자가 아니다(레이아웃 상자는 벽에서 0.4 m 이상 떨어진다 — 벽 센서 상자·중앙 패드 제외)
    wall_band: bool = True             # 벽 안쪽 면이 윗면 평면에서 안쪽으로 번져 보이는 띠를 윗면 후보에서 뺀다
    wall_band_extra_m: float = 0.03
    min_mean_v: float = 8.0            # 프레임 평균 밝기가 이보다 낮으면 어두운 프레임
    max_mean_v: float = 245.0
    max_boxes: int = 40                # 한 프레임에 이보다 많으면 오검출로 보고 통째로 버린다
    ignore_rects: Tuple[float, ...] = ()   # 무시 구역(고정 시설 등): x0 y0 x1 y1 네 수씩 묶어 아레나 좌표로. 중심이 이 안(여유 ignore_margin_m)이면 상자로 내지 않는다
    ignore_margin_m: float = 0.10


@dataclass
class EventCfg:
    """본선 규정의 이벤트 요소(PDF p25 '미션 이벤트 장치', p14·p22, 멘토링 11/4) 처리 설정. 상태전이표 6장.
    [가정] 표시는 규정에 값이 없어 정한 것이다 — 현장·11/6 API 에서 확인할 것."""
    # ---- E3 스모그(포그머신): 중앙 측면 1곳에서 분출, 서버가 ON/OFF·시간·주기 제어. LiDAR 에 고립된 가까운 반사(연기 입자)가 생긴다
    smog_enable: bool = True
    smog_near_m: float = 1.2           # m: 이보다 가까운 반사만 본다
    smog_on_ratio: float = 0.04        # 전체 빔 중 '고립된 가까운 반사'(이웃 빔과 거리가 안 맞는 것)의 비율이 이 이상이면 스모그 의심(≈14/360 빔)
    smog_off_ratio: float = 0.015      #   이 이하(≈5빔)로 내려가야 해제(히스테리시스). 정상 장면의 고립 반사는 0~2개
    smog_on_s: float = 0.8             # 의심이 이 시간 이어져야 켠다(다른 UGV 안테나 같은 순간 반사 무시)
    smog_off_s: float = 3.0            # 해제 조건이 이 시간 이어져야 끈다
    speckle_nbr_m: float = 0.25        # 이웃 빔(±2)과 거리 차가 이보다 작으면 같은 면의 점으로 본다
    smog_speed_scale: float = 0.8      # 스모그 중 주행 속도 배율
    smog_fire_conf: float = 0.60       # 스모그 중 발사에 필요한 검출 신뢰도(평소 perception.conf_min)
    smog_obs_range_m: float = 1.0      # 스모그 중 객체(M3) 집계에 쓰는 최대 거리(먼 것은 가려져 오분류 위험)
    smog_contrast_drop: float = 0.55   # (실영상) 영상 대비가 기준의 이 비율 아래로 떨어지면 스모그 증거로 본다
    # ---- E5 지정표적 시그널 LED(유효 타격 시 빨간색 점등): 한 프레임 오검출로 M2 를 끝내지 않도록 연속 확인
    led_confirm_frames: int = 3
    # ---- E8 UGV 후면 LED(피격 횟수 색 + 8칸 탄창): M6 의 생존·잔탄 증거. 뒤에서 가까이 볼 때만 읽힌다
    ugv_led_range_m: float = 1.8
    ugv_led_conf_min: float = 0.5
    # ---- E2 3색 타워 LED(경기 준비·진행·완료): 서버 STOP 이 유실됐을 때의 보조. 색은 규정에 없다 [가정]
    tower_stop_backup: bool = True
    tower_ready_color: str = "yellow"
    tower_running_color: str = "green"
    tower_finished_color: str = "red"
    tower_conf_min: float = 0.6
    tower_confirm_s: float = 1.5
    tower_min_t: float = 60.0          # 시작 후 이 시간 전에는 타워로 정지하지 않는다
    tower_requires_link_down: bool = True   # 서버 연결이 정상이면 STOP 메시지를 기다린다(오검출로 경기 중 멈추지 않게)
    # ---- E4 드론(UAV) 영상 주기 변화: 상대 M2 유효타격 1개→2 s, 2개→5 s, 3개→중단
    uav_off_s: float = 8.0             # 새 프레임이 이 시간 넘게 없으면 중단으로 본다
    # ---- E6 지정표적 오타격(다른 팀 표적을 치면 그 팀에 가점) 방지
    wrong_target_guard: bool = True


@dataclass
class ExploreCfg:
    """M3 온라인 탐색(mission/explore_online.py, perception/coverage.py). 값은 시뮬에서 정한 초깃값이다 — 실측값처럼 읽지 말 것.
    tunables.py 의 SIM_TUNED/ASSUMED 표 참고."""
    strategy: str = "online"           # "online"(프런티어 + 미확인 면 NBV) | "lattice"(기존: 5x5 고정 순회 + 박스 4개 이상이면 면 방문)
    sector_finish_bonus: float = 0.0   # 채점 섹터에서 남은 면이 적을수록 그 면의 가치에 곱할 가산(0=끔). M3 는 섹터의 종류·개수가 모두 맞아야 점수라 '거의 다 본 섹터'를 먼저 끝낸다
    sector_finish_max_todo: int = 3    # 남은 면(pending·viewed·uncertain)이 이 개수 이하인 섹터만 가산한다(1개 남으면 가산 전부, 이 개수면 1/개수)
    sector_finish_min_done: float = 0.5  # 이미 끝난 면(식별·빈 면·가림·도달불가)의 비율이 이 이상일 때만 — 막 시작한 작은 섹터를 앞당기지 않는다
    m3_dedupe: bool = False            # M3 제출 직전 섹터별 객체 목록에서 같은 면의 중복(장부 상자가 둘로 갈라진 경우 등)을 합친다
    m3_dedupe_dist_m: float = 0.25     # 같은 섹터·비슷한 면 방향에서 면 중심이 이 거리 안이면 같은 면 — 이웃 상자 면은 0.4 m 떨어져 있어 0.4 미만이어야 한다
    m3_counting: str = "ledger"        # M3 제출 집계: "ledger"(박스 면 장부 — 같은 면 한 번만, 박스 중심 섹터) | "omap"(기존 ObjectMap 위치 합치기)
    complete_m3_faces: bool = False   # PDF four-photo cube protocol; enabled by the fixed Gazebo trial
    exhaustive_scoring_faces: bool = False  # Observe all exposed enemy/CM sides; never infer a picture from geometry.
    coverage_sector_commit_s: float = 28.0  # Finish a local sweep, then service another unfinished sector.
    coverage_m2_budget_s: float = 30.0  # Total M2 time before resuming unfinished face coverage; zero disables.
    coverage_m4_budget_s: float = 12.0  # Total HUNT time while scoring faces remain unfinished; zero disables.
    coverage_goal_reserve_s: float = 25.0  # Keep the final interval for M5 travel/capture; unresolved faces use earlier time.
    m3_missing_face_weight: float = 4.0
    m3_observe_s: float = 2.0
    seen_res: float = 0.10             # m LiDAR '관측함' 지도 해상도
    seen_range_m: float = 4.0          # m 이 거리까지의 반사·자유공간만 '봤다'로 인정(그보다 먼 곳은 신뢰 낮음)
    box_refresh_s: float = 1.0         # 점유격자에서 박스를 다시 뽑는 주기
    uav_layout_timeout_s: float = 12.0 # 마지막 실제 배치 영상 이후 이 시간이 지나면 LiDAR·지상 카메라 탐색으로 전환
    view_max_incidence_deg: float = 70.0   # [가정] 면 법선과 시선이 이 각도 안일 때만 '그 면을 봤다'(비스듬한 면은 분류가 흔들림)
    view_min_gap_m: float = 0.15       # 같은 면의 '독립된 관측'으로 세는 최소 이동·회전·시간 간격(15 Hz 프레임을 따로 세지 않는다)
    view_min_gap_deg: float = 12.0
    view_min_gap_s: float = 0.5
    empty_min_views: int = 3           # 이만큼 독립 관측했는데 검출이 없으면 '빈 면'(객체 없음)으로 확정 — 단 서로 다른 자리 2곳 이상
    empty_min_poses: int = 2
    confirm_min_margin: float = 0.25   # 분류 확정에 필요한 1·2위 득표 차이 비율
    contradict_free_frac: float = 0.6  # 박스 증거 셀 중 L<0.3(자유로 재관측) 비율이 이 이상이고 클러스터가 사라지면 '이동함'
    attempts_giveup: int = 3           # 같은 면에 도착했는데도 못 본 횟수 → 포기
    spin_clear_m: float = 0.32       # m: 제자리 360° 회전에 필요한 몸 둘레 여유(점유 칸이 이 안에 있으면 한 방향 관측으로 대체) [시뮬 가정 — 실기 확인]
    nbv_dists: Tuple[float, ...] = (0.30, 0.35, 0.45, 0.6, 0.8, 1.0, 1.4) # normal offset; oblique views still require radial range >= .4 m
    nbv_lateral: Tuple[float, ...] = (-0.6, -0.3, 0.0, 0.3, 0.6)   # 벽에 붙은 면은 옆에서 비스듬히(입사각 view_max_incidence_deg 안) 본다
    w_frontier: float = 0.01           # 미관측 셀 1개(0.01 m²)의 정보 가치(면 1개 = 1.0)
    frontier_min_cells: int = 6
    frontier_rays: int = 36
    frontier_radius: float = 3.0       # m 시점에서 미관측 면적을 세는 반경
    travel_factor: float = 1.15        # [가정] A* 길이 → 실제 Nav2 주행 길이 보정
    plan_speed_scale: float = 0.9      # 이동 시간 = 길이 / (nav.v_cruise × 이 값)
    observe_look_s: float = 1.2        # 한 방향을 보는 관측 시간(s)
    time_overhead_s: float = 1.0       # 효용 분모의 고정 시간
    min_gain: float = 0.45             # 이보다 정보가 적은 후보는 선택하지 않는다
    switch_ratio: float = 1.3          # 현재 목표를 버리고 옮기려면 새 후보 효용이 이 배수 이상
    deadline_margin_s: float = 3.0     # 사전 이동(prestage_at) 전에 끝내야 하는 여유
    stage_margin_s: float = 4.0        # t_base_cmd 까지 스테이징 지점에 닿아야 하는 여유
    max_eval: int = 8                  # A* 로 이동시간을 정확히 재는 후보 수(직선거리 사전 순위)
    tabu_s: float = 20.0               # 막히거나 시간 초과한 시점을 다시 고르지 않는 시간
    goal_timeout_s: float = 30.0
    min_goal_commit_s: float = 6.0    # 새 경로를 따라갈 최소 시간; 2초마다 목표가 바뀌며 정지하는 현상 방지
    coverage_debt_s: float = 35.0     # 마지막 지상 관측 이후 섹터 탐색 가치가 1만큼 증가하는 시간
    coverage_debt_max: float = 2.5    # 거리/위험 비용을 유지하면서 미방문 섹터의 굶주림을 막는 상한
    sector_priority: bool = False      # Legacy option; Agent M3 always uses the team's four scoring sectors, independently of M1.
    sector_bonus: float = 0.25         # Explicit standalone explorer preference only; never narrows Agent M3 to M1 sectors.
    w_threat: float = 0.8              # 시점·경로 위험(risk·초)이 효용을 깎는 정도(1 − min(0.9, w·R))
    # --- 2D LiDAR 면 지도(perception/lidar_faces.py). 기본 꺼짐 — 실기 설정(config/waveshare_real.json)에서 켠다.
    lidar_faces: bool = False          # 장애물 군집을 'LiDAR 로 찾은 직선 면(≥40 cm) + 닫힌 도형 검사'로 만든다(이어 붙은 정육면체 L·T·줄 모양 허용)
    lidar_face_min_len_m: float = 0.40 # m 면으로 기록할 최소 직선 길이(0.9 배까지 허용 — 셀 양자화)
    lidar_face_min_L: float = 1.5      # 점유 셀로 인정하는 log-odds 하한(L_THR 0.7 보다 높게: 두 번 이상 맞은 셀만)
    lidar_ugv_exclude_m: float = 0.30  # m UGV 로 알려진 위치(트래커·움직이는 덩어리·카메라 검출) 주변은 면 기록에서 뺀다 [UGV 궤도는 LiDAR 평면 아래라 크기로 구분하지 않는다]
    lidar_closure_min_cells: int = 4   # 닫힌 도형의 안쪽 칸 수 하한(0.05 m 칸 기준)
    lidar_scout_standoff_m: float = 0.65   # m 닫히지 않은 그룹의 가려진 쪽을 보러 갈 때 그 지점에서 떨어질 거리
    lidar_open_bonus: float = 0.5      # 닫히지 않은 그룹의 면에 더하는 가치(뒷면 정찰 유도)
    lidar_scout_max_attempts: int = 2  # 같은 정찰 지점에 가 보는 최대 횟수(그래도 안 닫히면 포기)
    # --- UAV 영상으로 처음 받은 배치의 모든 면을 '면 기록부'에 올리고, 식별한 면은 다시 보러 가지 않는다. 기본 꺼짐 — 실기 설정에서 켠다.
    uav_face_registry: bool = False    # 기록부를 둔다(박스를 받아들일 때 4면 모두 등록, 확정·빈 면 기록, 상자가 사라졌다 다시 생겨도 이어 받는다)
    m1_sector_sweep: bool = False      # M1 이 지정한 탐색 섹터의 모든 면을 확인·분류해 면 기록부에 식별로 올린다(M3 채점 구역 밖이어도). 기본 꺼짐 — 실기 설정에서 켠다
    m1_assigned_only: bool = False     # Restrict exploration to the two M1 sectors; scoring rules stay unchanged.
    m1_sector_weight: float = 1.6      # M1 지정 섹터 면의 가치 배수(채점 구역 면보다 먼저 보게)
    # --- M1 지정 구역 우선순위: 적 섹터를 먼저, CM 은 그 다음. 기본 꺼짐(= 위의 단일 가중 그대로) — 실기 설정에서 켠다.
    m1_priority_enemy_first: bool = False   # True: M1 지정 구역 안에서 적 섹터 면 → CM 면 순서로 본다(적 섹터가 끝나기 전 CM 면은 m1_cm_defer 배로 깎는다)
    m1_enemy_weight: float = 2.4       # M1 지정 '적 섹터' 면의 가치 배수(m1_priority_enemy_first 일 때 m1_sector_weight 대신)
    m1_cm_weight: float = 1.6          # M1 지정 CM 면의 가치 배수(적 섹터를 끝낸 뒤·시간 상한 뒤의 값)
    m1_cm_defer: float = 0.30          # 적 섹터 면이 남아 있는 동안 CM 면 가치에 곱하는 값(0..1). 0 에 가까울수록 엄격한 순서, 1 이면 순서 없음
    m1_enemy_budget_s: float = 150.0   # s M1 수신 뒤 이 시간이 지나면 적 섹터가 안 끝났어도 CM 을 풀어 준다(못 푸는 면에 경기를 다 쓰지 않게). 0 = 상한 없음
    # --- UAV 초기 영상 장부: 처음 받아들인 UAV 배치의 모든 면을 '미탐지'로 올려 두고, 식별되면 '탐지'로 바꾼다(보고용). 기본 꺼짐.
    uav_census: bool = False
    uav_census_window_s: float = 4.0   # s 첫 UAV 배치를 받아들인 때부터 이 시간 안에 올라온 면을 '초기 영상의 면'으로 센다(첫 프레임이 일부만 잡았을 때를 위해)
    # --- UAV 장애물 지도: 첫 영상으로 만든 지도를 끝까지 쓰고(uav_map_freeze), 맞닿은 면은 장부에서 빼며(uav_outer_faces), UGV 는 지도에서 뺀다(uav_exclude_ugv). 기본 꺼짐 — 실기 설정에서 켠다.
    uav_map_freeze: bool = False       # True: 첫 배치를 받아들인 뒤 uav_map_freeze_s 안에 모은 상자를 '그 경기의 장애물 지도'로 굳힌다 — 영상이 끊겨도 계획에서 빠지지 않고, 이후 프레임이 상자를 더하거나 옮기지 않는다(LiDAR 가 빈 공간임을 반복 확인한 상자만 지운다)
    uav_map_freeze_s: float = 4.0      # s 지도를 모으는 시간(첫 받아들임부터). 그 뒤 상자가 uav_map_freeze_min_boxes 개 이상이면 굳힌다(모자라면 계속 모은다)
    uav_map_freeze_min_boxes: int = 1
    uav_outer_faces: bool = False      # True: 이웃 상자와 맞닿은 면(서로 가린 면)·벽에 붙은 면은 면 장부·기록부(census 포함)에 올리지 않는다 — 바깥으로 드러난 최외곽 면만 센다
    uav_adjacent_tol_m: float = 0.14   # m 이웃 상자와 '맞닿았다'고 보는 위치 허용 오차(중심 간격이 상자 한 변 ± 이 값, 옆 어긋남도 이 값 이내)
    uav_cover_frac: float = 0.2        # 이웃 상자가 면 길이의 이 비율 이상을 맞대고 있으면 그 면을 안쪽 면으로 본다(반 칸 엇갈린 면은 0.5 라 빠지고, 살짝 스친 0.2 미만은 남는다)
    uav_exclude_ugv: bool = False      # True: UAV 영상에서 UGV 로 검출된 위치 주변의 상자 후보는 장애물 지도·정렬·면 장부에서 뺀다(UGV 윗면이 상자로 잡히는 것을 막는다)
    uav_ugv_exclude_m: float = 0.25    # m UGV 검출 중심과 상자 후보 중심이 이 거리 안이면 UGV 로 본다(맞닿은 상자는 중심이 0.325 m 라 남는다)
    redetect_resolved: bool = True     # True(기존): 확정·빈 면도 '이동 의심(suspect)'이면 재검증 가치 0.4 를 준다. False: 식별한 면은 다시 보지 않는다(교차 불일치 recheck 만 예외)


@dataclass
class AvoidCfg:
    """적 UGV 회피(perception/threat_board.py, mission/threat_field.py, mission/avoid.py). 초깃값은 시뮬용 — 실측 전에는 가정이다."""
    enable: bool = True
    evade_min_hits: int = 2          # 시간 소모 회피/엄폐는 두 번째 피격부터. 첫 피격부터 켜는 변경은 A/B 검증 필요.
    r_aware: float = 3.0
    r_evade: float = 1.6
    r_close: float = 0.8
    exit_margin_m: float = 0.6         # 해제 문턱 = 진입 문턱 + 이 값
    exit_hold_s: float = 1.5
    min_hold_s: float = 1.0
    r_core: float = 2.2                # 위험 1.0 인 반경(m) = laser.max_range − 0.3. 사정거리 안·시야가 트이면 거리와 무관하게 맞는다는 시뮬 규칙 [가정 — 실측 필요]
    r_laser_extra: float = 0.3         # 위험 반경 = laser.max_range + 이 값
    p_cons: float = 0.7                # 팀 미확인 UGV 의 최소 위험 가중
    friend_w: float = 0.15
    enemy_p: float = 0.85
    friend_p: float = 0.10
    v_enemy_max: float = 0.45          # [가정] = robot.v_max
    r_body: float = 0.13               # m 적 차체 반경(시뮬 UGV_R) — 실물 치수로 교체
    sigma_lidar: float = 0.06
    sigma_cam: float = 0.08
    sigma_uav: float = 0.15            # [가정] 시뮬 UAV 는 0.04 지만 실기는 모른다
    k_sigma: float = 1.5
    hold_track_s: float = 2.0
    ghost_s: float = 6.0
    layers: int = 4
    layer_dt: float = 0.75
    w_risk: float = 3.0
    w_close: float = 1.5
    exposure_limit_s: float = 1.2      # 0.6 × laser.dwell_s(PDF 2 s)
    carrot_m: float = 1.0              # 경유지를 경로 위 이만큼 앞에 둔다
    retreat_min_m: float = 0.6
    retreat_max_m: float = 3.0
    breadcrumb_step_m: float = 0.25
    breadcrumb_max: int = 40
    hold_max_s: float = 3.0
    commit_min_s: float = 1.5
    reverse_max_m: float = 0.5
    t_margin_s: float = 8.0            # M5 시간 여유 계산 상수
    t_soft_s: float = 25.0
    dash_dist_m: float = 1.0
    dash_exposure_s: float = 1.2
    m2_defer_max_s: float = 30.0
    lidar_blob_min: float = 0.12       # LiDAR 동적 군집: 크기 범위(m)와 지속 스캔 수
    lidar_blob_max: float = 0.36
    lidar_blob_scans: int = 3
    lidar_blob_min_speed: float = 0.15  # m/s: 이보다 느린 겉보기 이동은 움직이는 물체로 치지 않는다(느린 적은 카메라/UAV 가 맡는다)
    use_blobs: bool = True             # LiDAR 동적 군집을 위협 입력으로 쓴다(꺼도 카메라/UAV 로 동작)
    use_visibility: bool = True        # 위험장에 '박스가 가리는 시야'를 쓴다(끄면 거리만)
    r_close_cost: float = 1.2          # m: 근접 압박 비용 반경
    ttc_evade_s: float = 2.5
    ttc_aware_s: float = 6.0
    exit_hold_evade_s: float = 2.0
    exit_hold_close_s: float = 1.0
    close_exit_margin_m: float = 0.3
    strike_cooldown_s: float = 3.0     # 회피가 끝난 뒤 지정표적 재접근까지 쉬는 시간
    reentry_max: int = 4               # EVADE 진입이 이 창 안에 이만큼 넘으면 위험 비용만 쓰는 AWARE 로 격하
    reentry_window_s: float = 30.0
    cap_s: float = 25.0                # 회피가 구동을 쥔 시간이 이 창에서 이만큼 넘으면 일정 시간 격하(임무 지연 상한)
    cap_window_s: float = 60.0
    downgrade_s: float = 15.0
    plan_period_s: float = 0.2
    ring_radii: Tuple[float, ...] = (1.0, 1.5, 2.0)
    ring_n: int = 16
    max_verify: int = 4                # 후퇴 후보 중 위험 A* 로 확인하는 상위 개수
    reverse_speed: float = 0.12        # m/s: 근거리 후진 속도 [가정 — 실제 정지거리로 교체]
    hold_front_min_d: float = 0.6
    risk_hit: float = 0.5              # 이 이상이면 '맞을 수 있는 칸'으로 보고 노출 시간을 센다
    fresh_s: float = 1.6               # s: EVADE/CLOSE(소유권)는 이보다 최근 관측에만 건다 — UAV 지연 등 오래된 위치로 움직이면 오히려 맞았다(시뮬)
    aware_speed_scale: float = 0.85    # AWARE 이상에서 임무 이동 속도 배율
    risk_extra_inflate_m: float = 0.07  # 위험 인지 경로의 하드 인플레이션 가산(m): 벽을 타고 가는 경로 방지 [시뮬 가정]
    route_keep_ratio: float = 1.25     # 새 위험 인지 경로가 옛 경로(남은 부분)보다 이 배율 넘게 싸지 않으면 옛 경로 유지(경로 뒤집힘 방지)
    route_keep_abs: float = 0.4        # m 상당의 절대 여유
    carrot_hold_m: float = 0.45        # 경유지를 이 거리 안에 들어와야 다음 경유지로 바꾼다(Nav2 목표 재전송 폭주 방지)
    field_period_s: float = 0.25       # 위협장 재생성 주기
    # --- 아군 UGV 추적(perception/friend_map.py). 기본 꺼짐 — 실기 설정에서 켠다. 놓치면 '식별 안 됨'으로 남기고 다시 이어 붙이지 않는다.
    friend_track: bool = False         # UAV 영상으로 출발 때부터 아군 위치를 이어 추적하며 피아식별에 쓴다(꺼짐: 옛 방식 — 시작 3 s 거점 근접 ident 를 위치 연속성으로 계속 이어 씀)
    friend_birth_s: float = 3.0        # s 첫 UAV 프레임 뒤 이 시간 안에 자기 거점 근처(friend_birth_radius_m)에서 보인 UGV 에만 신원을 붙인다(UAV 주기가 느리면 1.5 주기까지 늘어난다)
    friend_birth_radius_m: float = 0.75
    friend_lost_s: float = 4.0         # s 이 시간 동안 같은 아군이 확인되지 않으면 놓친 것(UAV 주기의 2.5 배보다는 길게 잡는다)
    friend_gate_m: float = 0.5         # m 연속 추적 기본 문턱(여기에 friend_max_speed × 경과 시간이 더해진다)
    friend_max_speed: float = 0.6      # m/s [가정] robot.v_max 0.45 + UAV 위치 오차 여유
    friend_ambiguity_m: float = 0.35   # m 다른 후보가 가장 가까운 후보보다 이 거리 안에 있으면 어느 쪽인지 모른다 → 놓친 것으로 본다


@dataclass
class LidarCrosstalkCfg:
    enabled: bool = False           # 실기 감지율/오탐/선행 시간 검증 전에는 사용하지 않는다.


@dataclass
class ThreatCfg:
    level_up: Tuple[float, ...] = (1.0, 2.0, 3.0)
    level_down_s: float = 3.0
    lidar_min_width_m: float = 0.12
    lidar_max_width_m: float = 0.40
    lidar_confirm_scans: int = 1
    lidar_static_margin_m: float = 0.06
    lidar_stale_s: float = 1.0
    # Candidate fast path only. Assumed margins; validate with real pose/range noise.
    lidar_birth_require_prior_interior: bool = True
    lidar_birth_guard_m: float = 0.12
    lidar_birth_support_radius_m: float = 0.05
    lidar_birth_min_free_fraction: float = 0.75  # centre + at least 7/9 cells at 5 cm resolution
    lidar_crosstalk: LidarCrosstalkCfg = field(default_factory=LidarCrosstalkCfg)


@dataclass
class RiskCfg:
    unknown_space_cost: float = 3.0  # Finite preference for observed free space; unknown remains traversable.
    w: Dict[str, float] = field(default_factory=lambda: {"L0": 0.5, "L1": 1.5, "L2": 4.0, "L3": 10.0, "engage": 0.3})
    # 서빈님 비용표 미제공: 아래 값은 시작 가정이며 현장 비용표와 A/B로 교체한다.
    cost_table: Dict[str, float] = field(default_factory=lambda: {"exposure": 1.0, "hit_history": 2.0, "bottleneck": 1.5, "closing": 0.5, "ally_fire_lane": 1.0})
    team_weight: Dict[str, float] = field(default_factory=lambda: {"enemy": 1.0, "unknown": 0.7, "ally": 0.3})
    range_m: float = 3.0
    update_hz: float = 5.0
    threaded: bool = True  # Fixed-step 2D A/B selects False for reproducibility.
    max_exposed_dwell_s: float = 1.5
    dwell_cap_from_hits: int = 2


@dataclass
class PostureCfg:
    commit_s: float = 4.0
    commit_dist_m: float = 0.6
    min_dwell_s: float = 3.0
    engage_in_cm: bool = True
    engage_in_goal: bool = True
    engage_time_pressure_at: Optional[float] = None
    engage_exit_hits: int = 3
    goal_exception_m: float = 1.0


@dataclass
class ProgressCfg:
    enabled: bool = False            # Candidate opt-in until A/B acceptance. False: 기존 v2 전략을 유지하는 A/B 대조군.
    stall_replan_s: float = 3.0
    stall_engage_s: float = 6.0
    stall_backoff_s: float = 9.0
    stall_abandon_s: float = 13.0
    min_gain_m: float = 0.15
    retry_after_s: float = 30.0
    max_goal_failures: int = 2
    jitter: float = 0.2
    oscillation_window_s: float = 10.0
    max_decision_flips: int = 3
    max_direction_reversals: int = 2


@dataclass
class EngageCfg:
    min_clearance_m: float = 0.30     # 상대 차체 가장자리와 우리 차체 가장자리 사이.
    fire_on_blocker: bool = False    # 실측/시뮬 A/B 근거가 생기기 전에는 끔.
    m2_ammo_reserve: int = 3
    enemy_confidence: float = 0.8
    blocker_min_range_m: float = 0.6
    blocker_max_range_m: float = 1.5
    target_cooldown_s: float = 10.0


@dataclass
class LidarCfg:
    """LiDAR 자기 가림(팬틸트 기둥·안테나·카메라 브래킷이 스캔 평면에 들어오는 것) 처리. 장착 위치를 몰라 기본은 끔 [미측정].
    스캔 각도는 차체 기준(전방 0, 왼쪽 +). 마스크는 Agent 가 받은 스캔에만 적용된다 — ROS 쪽(Cartographer/Nav2 비용지도)은 원본 /scan 을 쓰므로
    드라이버나 laser_filters 에서 같은 각도를 따로 걸러야 한다(ros2/README.md)."""
    self_mask_sectors: Tuple[Tuple[float, float], ...] = ()   # ((중심 deg, 반폭 deg), …) — 이 각도 안의 '가까운' 반사를 무시
    self_mask_range: float = 0.35      # m: 마스크 각도 안에서 이보다 가까운 반사만 지운다(그보다 먼 건 실제 장애물로 둔다)
    min_range: float = 0.0             # m: 모든 방향에서 이보다 가까운 반사를 지운다(0 이면 끔; 몸체 안쪽 반사용)
    min_latency_s: float = 0.0         # LiDAR 측정→수신 최소 지연(s) [미측정]. stamp 가 다른 시계일 때만 쓴다(hal/clock.py)
    suspect_range: float = 0.30        # m: 이보다 가까운 반사가
    suspect_s: float = 8.0             #   이 시간 이상, 로봇이 이 각도(rad)만큼 돌아도 같은 방향·거리에 남아 있으면
    suspect_rot: float = 1.0           #   차체 일부(자기 가림)로 의심하고 경고한다(마스크 각도 후보를 로그에 남김)


@dataclass
class LlmCfg:
    # Organizer endpoint/request format is unannounced. No generic HTTP client
    # is invented: the team's module:function adapter owns that contract.
    endpoint: Optional[str] = None       # MAICON_LLM_API_URL may supply the announced endpoint.
    api_key_env: str = "MAICON_LLM_API_KEY"  # Environment variable name, never store the key in saved config.
    adapter_factory: Optional[str] = None  # Existing complete(prompt, timeout) factory interface.


@dataclass
class MissionCfg:
    decision_period_s: float = 0.05  # 행동·자세·관측 목표 재판단. 센서/조준/안전/STOP 주기와 분리.
    t_total: float = 300.0         # [PDF] 경기 5분
    t_base_cmd: float = 240.0      # [PDF] 경기 종료 1분 전 목표거점 명령
    forbid_own_bases_at: float = 225.0  # [요청 전략/G12] S9.7의 상대 점령 오발동 방지; 공식 진입금지 시각은 아님. 음수는 명시적 비활성.
    own_base_guard_horizon_s: float = 0.5  # [가정] 명령 유지/정지 예측 시간; 실제 제동시간으로 현장 교정.
    own_base_guard_step_m: float = 0.01   # [수치 검사] 회전 모서리 포함 궤적 샘플 이동 상한.
    t_report: float = 30.0         # [PDF] 종료 후 30초 안에 M6 제출
    prestage_at: float = 222.0     # [가정] 이 시각부터 상대 진영 중앙으로 사전 이동
    stage_point_override: Optional[Tuple[float, float]] = None  # 고정 Gazebo 배치에서 장애물을 피한 대기점
    stage_policy: str = "enemy_middle"  # 대기점: enemy_middle = 적 가운데 거점(BLUE 면 R-2, RED 면 B-2)의 경기장 쪽 문 앞 | legacy = (W-2.0, H/2) — 옛 값은 고정 구조물 안에 들어간다
    stage_standoff_m: float = 0.30   # [가정] 적 가운데 거점 경계에서 경기장 쪽으로 떨어진 거리(m). 앞 장애물과의 여유는 현장 배치로 확인(0.22 m 이상이어야 돌 수 있다)
    goal_entry_creep_s: float = 0.0  # Gazebo 광전센서 경계 앞 위치오차 보정용 저속 진입(실기 기본 끔)
    m5_dwell_s: float = 0.0         # [잠정 API 2026-11-04] 현재는 센서가 가려지는 순간; 체류시간 확정 전 슬롯만 유지.
    strike_range: float = 1.10     # m: 지정표적을 쏠 때 접근 목표 거리
    strike_max_incidence_deg: float = 40.0  # 지정표적 면의 법선과 로봇 방향 사이 각 한계 — 넘으면 정면 지점으로 재접근 [가정 — 실기 확인]
    fire_range_slack: float = 0.15  # m: 지정표적 사격은 거리가 strike_range+이 값 이하일 때만(창 ±3.5 cm 의 허용 각도가 거리에 반비례) [시뮬 가정 — 실기 확인]
    ugv_range: float = 1.40        # m: 상대 UGV 교전 목표 거리
    m2_robust_abandon: bool = False   # 지정표적 접근 포기를 '회피 시간을 뺀 12 s + 막힘이 이어질 때'로, 영구 포기가 아니라 쿨다운 뒤 재시도(m2_unreach_max 번째에 영구)로 바꾼다
    m2_nav_budget_s: float = 12.0     # 접근 시작 뒤 이 시간(회피로 쓴 시간 제외)이 지나야 포기 후보
    m2_blocked_s: float = 4.0         # 이동이 '막힘'으로 이만큼 이어져야 포기 후보 — 한 틱 막힘으로 포기하지 않는다
    m2_unreach_max: int = 3           # 접근 포기가 이 횟수에 이르면 영구 포기(target_failed)
    m2_lost_robust: bool = False      # 조준 중 표적을 3번 놓쳐도 영구 포기하지 않는다: 그 면을 '나쁜 면'으로 기억해 쿨다운 동안 제외하고, 다른 면이 보이면 갈아탄다
    m2_lost_cooldown_s: float = 6.0   # 놓친 면을 다시 시도하기까지의 기본 대기(놓친 횟수 배로 늘어난다)
    m2_lost_max: int = 3              # 같은 면을 이만큼 놓치면 그 면은 경기 끝까지 제외(유령 면)
    m2_evidence_w: float = 0.0        # 표적 면 고르기: 거리에서 (관측 횟수 x 이 값[m], 최대 15회)를 뺀다 — 근거가 많은 면을 우선
    max_shots_per_target: int = 3
    friendly_fire_guard_deg: float = 6.0   # 발사선 ±이 각도 안에 아군/불명 UGV가 있으면 발사 금지
    engage_ugv: bool = False       # 상태 전이표: 기본 전략은 M2 우선·적 UGV 회피·M5 진입 → M4 교전(HUNT)은 기본 OFF
    hunt_on_engage_ok: bool = False  # 자세가 ENGAGE_OK(교전 허용: CM·정체·무적·GOAL 등)인 동안에는 engage_ugv 가 꺼져 있어도 HUNT 를 허용한다.
                                     # 자세는 progress.enabled 일 때만 계산된다. 코드 기본은 끔(기존 동작), 실기 설정(waveshare_real.json)에서 켠다
    # ---- 상태 전이표(MAICON2026_상태전이표_경쟁상태반영.html) 구현 파라미터. [가정] 표시는 실측 후 조정
    local_stop_grace_s: float = 1.0   # 시작 후 t_total + 이 값이 지나도 STOP이 없으면 로컬 안전 정지(통신 유실 대비)
    lidar_stale_s: float = 0.6        # [가정] LiDAR 신선도 한계(정상 10 Hz 기준 약 6주기). 실측 주기·지연 후 정한다
    safe_resume_scans: int = 2        # SAFE_STOP 복귀에 필요한 '정지 이후 새' LiDAR 표본 수
    vision_stale_s: float = 0.8       # [가정] 검출 결과가 이보다 오래되면 발사 금지·검출 폐기
    hit_grace_s: float = 0.6          # 피격 이벤트 직후 하드웨어 상태 갱신 전까지 보수적으로 잠금을 거는 시간
    aim_budget_s: float = 25.0        # STRIKE 조준 단계 시간 예산(초과하면 재접근/포기)
    recover_after_s: float = 4.0      # 항법이 이 시간 넘게 'blocked'면 RECOVER
    recover_back_s: float = 0.7       # RECOVER: 후방이 비었으면 후진하는 시간
    recover_turn_s: float = 1.4       # RECOVER: 트인 쪽으로 제자리 회전하는 시간
    recover_max: int = 3              # recover_window_s 안에서 허용하는 RECOVER 횟수(초과하면 SAFE_STOP)
    recover_window_s: float = 40.0
    recover_hold_s: float = 5.0       # 'recover_exhausted' SAFE_STOP 유지 시간
    submit_retry_s: float = 1.0       # 제출 실패(거부)·응답 유실 때 재시도 간격
    submit_ack_timeout_s: float = 3.0 # 제출 응답을 이 시간 넘게 못 받으면 유실로 보고 재전송
    submit_max_tries: int = 0         # M1/M3 최대 시도. 0 = STOP 전까지 무제한(점수를 통째로 잃지 않게). M6 는 보고 기한까지 계속
    m3_first_s: float = 10.0          # Early full replacement snapshot; subsequent observations update all scoring sectors.
    m3_resubmit_s: float = 5.0        # Changed complete snapshots, never only newly found sectors/classes.
    m3_late_at_s: float = 225.0       # Increase changed-snapshot frequency before either camp can end via M5.
    m3_late_gap_s: float = 1.0        # Seconds between changed full replacements after m3_late_at_s.
    m3_final_flush_s: float = 20.0    # Force a complete snapshot this far before t_total, even if counts are unchanged.
    m3_deadline_margin_s: float = 10.0 # Stop sends/retries early: server RECEIPT must precede 300 s, not merely request creation.
    m3_at_stop: bool = False          # Deprecated compatibility field; M3 is always prohibited after STOP.
    report_llm_wait_s: float = 8.0    # M6 첫 규칙 보고는 즉시; 아직 보고 중일 때 받아들일 LLM 정정의 최대 지연
    m1_llm_timeout: float = 6.0
    m1_submit_after: float = 1.0   # s: 규칙파서 결과를 이 시간 안에 먼저 제출(LLM이 늦으면)
    spin_w: float = 1.1            # rad/s 시점 도착 후 360° 회전 속도(0이면 회전 안 함)
    tour_nx: int = 5               # 탐색 시점 격자 열 수
    tour_ny: int = 5


@dataclass
class Config:
    team: Optional[str] = None     # 'BLUE' | 'RED' | None(서버에서 받음)
    slot: Optional[int] = None     # 1..3 시작 거점 번호 | None(서버/추정)
    arena: ArenaGeometry = field(default_factory=ArenaGeometry)
    robot: RobotCfg = field(default_factory=RobotCfg)
    hal: HalCfg = field(default_factory=HalCfg)
    camera: CameraCfg = field(default_factory=CameraCfg)
    rig: RigCfg = field(default_factory=RigCfg)
    depth: DepthCfg = field(default_factory=DepthCfg)
    map3d: Map3DCfg = field(default_factory=Map3DCfg)
    laser: LaserCfg = field(default_factory=LaserCfg)
    aim: AimCfg = field(default_factory=AimCfg)
    nav: NavCfg = field(default_factory=NavCfg)
    nav2: Nav2Cfg = field(default_factory=Nav2Cfg)
    perception: PerceptionCfg = field(default_factory=PerceptionCfg)
    explore: ExploreCfg = field(default_factory=ExploreCfg)
    avoid: AvoidCfg = field(default_factory=AvoidCfg)
    threat: ThreatCfg = field(default_factory=ThreatCfg)
    posture: PostureCfg = field(default_factory=PostureCfg)
    risk: RiskCfg = field(default_factory=RiskCfg)
    progress: ProgressCfg = field(default_factory=ProgressCfg)
    engage: EngageCfg = field(default_factory=EngageCfg)
    lidar: LidarCfg = field(default_factory=LidarCfg)
    lidar_err: LidarErrCfg = field(default_factory=LidarErrCfg)
    event: EventCfg = field(default_factory=EventCfg)
    mission: MissionCfg = field(default_factory=MissionCfg)
    llm: LlmCfg = field(default_factory=LlmCfg)
    link: LinkCfg = field(default_factory=LinkCfg)
    oak: OakCfg = field(default_factory=OakCfg)
    oak_det: OakDetCfg = field(default_factory=OakDetCfg)
    uav_boxes: UavBoxesCfg = field(default_factory=UavBoxesCfg)
    impact: ImpactCfg = field(default_factory=ImpactCfg)
    pose_guard: PoseGuardCfg = field(default_factory=PoseGuardCfg)
    loop_hz: float = 20.0

    # ------------------------------------------------------------ 로드/저장
    @classmethod
    def load(cls, path: Optional[str] = None) -> "Config":
        cfg = cls()
        if not path:
            return cfg
        with open(path, "r", encoding="utf-8") as f:
            txt = f.read()
        if path.endswith((".yaml", ".yml")):
            import yaml  # type: ignore
            data = yaml.safe_load(txt) or {}
        else:
            data = json.loads(txt)
        _merge(cfg, data)
        return cfg

    def to_dict(self) -> Dict[str, Any]:
        return dataclasses.asdict(self)

    # ------------------------------------------------------------ 차체 외곽·여유거리 (footprint.py)
    def budget(self):
        """현재 robot/lidar_err 값에서 계산한 ClearanceBudget."""
        from .footprint import budget_from
        return budget_from(self.robot, self.lidar_err, self.nav.v_cruise)

    def consistency(self, tol: float = 0.002) -> list:
        """파생값(nav.inflate_hard / rotate_clear / side_lim / stop_dist)이 예산과 어긋난 항목 [(이름, 현재, 기대)]. 비면 일관."""
        if self.robot.collision_model == "circle":
            return []                                    # 레거시 비교 모드는 일부러 옛 값을 쓴다
        b = self.budget()
        want = {"nav.inflate_hard": b.plan_inflate_straight, "nav.rotate_clear": b.plan_inflate_rotate,
                "nav.side_lim": b.stop_side, "nav.stop_dist": b.stop_front}
        out = []
        for k, v in want.items():
            sec, name = k.split(".")
            cur = getattr(getattr(self, sec), name)
            if abs(cur - v) > tol:
                out.append((k, cur, round(v, 4)))
        return out

    def api_issues(self) -> list:
        """실기 API 한계(PDF p14·15)를 넘는 설정 [(항목, 현재, 한계)]. 비면 정상. 넘으면 HAL 이 잘라 보내므로 계획과 실제가 어긋난다."""
        h, r = self.hal, self.robot
        out = []
        if r.v_max > h.v_cap + 1e-9:
            out.append(("robot.v_max", r.v_max, h.v_cap))
        if r.w_max > h.w_cap + 1e-9:
            out.append(("robot.w_max", r.w_max, h.w_cap))
        if r.pan_limit > h.gimbal_unit_max * h.pan_deg_per_unit + 1e-9:
            out.append(("robot.pan_limit", r.pan_limit, h.gimbal_unit_max * h.pan_deg_per_unit))
        if r.tilt_limit > h.gimbal_unit_max * h.tilt_deg_per_unit + 1e-9:
            out.append(("robot.tilt_limit", r.tilt_limit, h.gimbal_unit_max * h.tilt_deg_per_unit))
        if not 1.0 <= h.gimbal_speed <= 270.0:
            out.append(("hal.gimbal_speed", h.gimbal_speed, "1~270"))
        if not 1.0 <= h.gimbal_acc <= 254.0:
            out.append(("hal.gimbal_acc", h.gimbal_acc, "1~254"))
        if h.cmd_hz < 2.0:
            out.append(("hal.cmd_hz", h.cmd_hz, "0.5 s 끊기면 자동 정지 → 최소 2 Hz, 권장 10 Hz"))
        return out

    def apply_budget(self) -> None:
        """robot/lidar_err 를 바꾼 뒤 파생값을 다시 채운다(현장 실측 입력 → 일관 갱신)."""
        b = self.budget()
        self.nav.inflate_hard = round(b.plan_inflate_straight, 3)
        self.nav.rotate_clear = round(b.plan_inflate_rotate, 3)
        self.nav.side_lim = round(b.stop_side, 3)
        self.nav.stop_dist = round(b.stop_front, 3)
        self.robot.radius = round(b.fp.circumradius, 4)

    def use_legacy_circle(self) -> "Config":
        """변경 전 동작(비교용): 시뮬 충돌 원 0.14 m, A* 인플레이션 0.17 + 실패 시 0.8·0.6 배 완화, 정지 0.17, 측면 0.19, 직사각형 검증 없음."""
        self.robot.collision_model = "circle"
        self.robot.radius = self.robot.radius_legacy
        self.nav.inflate_hard, self.nav.stop_dist, self.nav.side_lim = 0.17, 0.17, 0.19
        self.nav.relax_inflation, self.nav.footprint_check = True, False
        return self


def _merge(obj: Any, data: Dict[str, Any]) -> None:
    for k, v in data.items():
        if not hasattr(obj, k):
            raise KeyError(f"알 수 없는 설정 키: {k}")
        cur = getattr(obj, k)
        if dataclasses.is_dataclass(cur) and isinstance(v, dict):
            _merge(cur, v)
        else:
            if isinstance(cur, tuple) and isinstance(v, list):
                v = tuple(v)
            setattr(obj, k, v)




