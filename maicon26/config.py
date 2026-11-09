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
    blind_reverse_mps: float = 0.2       # 영상은 오는데 쓸 수 있는 픽셀이 없을 때(벽에 코를 박음·렌즈 가림) 허용하는 후진 속도. 전진은 막고 회전은 허용
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
    replan_s: float = 1.0
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
    max_obj_range: float = 2.2        # m: 이보다 먼 객체 인식은 집계에 넣지 않음(오분류 방지)
    target_classes: Tuple[str, ...] = ("GUARD_POST", "RADAR_FACILITY", "COMM_FACILITY")
    object_classes: Tuple[str, ...] = ("AMMO_BOX", "DRUM", "MORTAR", "TRUCK", "TANK", "MISSILE")
    # 비인식용 박스 4종(도시 나무·차단벽·전진지·건물, 이름은 멘토링 전사 기준 — 정확한 이름 미확인). 세지 않고 무시한다(통계만).
    nonobject_classes: Tuple[str, ...] = ("TREE", "BARRIER", "POSITION", "BUILDING")
    # 시뮬 전용: 지정표적 검출 bbox 가 무엇을 덮는가. "face"=면 전체(수광 원판이 bbox 안 상단 중앙), "object"=사진 속 시설 물체만
    # (PDF p12 사진에서 흰 원은 물체 위 하늘에 있어 bbox 밖). 실제 YOLO 라벨링 방식은 아직 모른다 → 센서를 영상에서 직접 찾는다.
    sim_target_box: str = "face"


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
    m3_counting: str = "ledger"        # M3 제출 집계: "ledger"(박스 면 장부 — 같은 면 한 번만, 박스 중심 섹터) | "omap"(기존 ObjectMap 위치 합치기)
    complete_m3_faces: bool = False   # PDF four-photo cube protocol; enabled by the fixed Gazebo trial
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
    plan_period_s: float = 0.5
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
    t_total: float = 300.0         # [PDF] 경기 5분
    t_base_cmd: float = 240.0      # [PDF] 경기 종료 1분 전 목표거점 명령
    forbid_own_bases_at: float = 225.0  # [요청 전략/G12] S9.7의 상대 점령 오발동 방지; 공식 진입금지 시각은 아님. 음수는 명시적 비활성.
    own_base_guard_horizon_s: float = 0.5  # [가정] 명령 유지/정지 예측 시간; 실제 제동시간으로 현장 교정.
    own_base_guard_step_m: float = 0.01   # [수치 검사] 회전 모서리 포함 궤적 샘플 이동 상한.
    t_report: float = 30.0         # [PDF] 종료 후 30초 안에 M6 제출
    prestage_at: float = 222.0     # [가정] 이 시각부터 상대 진영 중앙으로 사전 이동
    stage_point_override: Optional[Tuple[float, float]] = None  # 고정 Gazebo 배치에서 장애물을 피한 대기점
    goal_entry_creep_s: float = 0.0  # Gazebo 광전센서 경계 앞 위치오차 보정용 저속 진입(실기 기본 끔)
    m5_dwell_s: float = 0.0         # [잠정 API 2026-11-04] 현재는 센서가 가려지는 순간; 체류시간 확정 전 슬롯만 유지.
    strike_range: float = 1.10     # m: 지정표적을 쏠 때 접근 목표 거리
    strike_max_incidence_deg: float = 40.0  # 지정표적 면의 법선과 로봇 방향 사이 각 한계 — 넘으면 정면 지점으로 재접근 [가정 — 실기 확인]
    fire_range_slack: float = 0.15  # m: 지정표적 사격은 거리가 strike_range+이 값 이하일 때만(창 ±3.5 cm 의 허용 각도가 거리에 반비례) [시뮬 가정 — 실기 확인]
    ugv_range: float = 1.40        # m: 상대 UGV 교전 목표 거리
    max_shots_per_target: int = 3
    friendly_fire_guard_deg: float = 6.0   # 발사선 ±이 각도 안에 아군/불명 UGV가 있으면 발사 금지
    engage_ugv: bool = False       # 상태 전이표: 기본 전략은 M2 우선·적 UGV 회피·M5 진입 → M4 교전(HUNT)은 기본 OFF
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




