# 로봇 보정 (calib/)

> **상태: 도구와 절차는 시뮬 로봇으로 검증했다. 실제 WINGBOT에서 측정한 값은 아직 없다.**
> `robot_profile.yaml`의 값은 PDF에 있는 것(`pdf`)과 가정(`assumed`)뿐이다. 실기 측정 후 `scripts/calib_fit.py`가 `measured`로 바꾼다.
> 남은 가정 목록: `python scripts/calib_report.py`

## 산출물 5종과 위치
| 산출물 | 들어가는 값 | 파일 | 만드는 법 |
|---|---|---|---|
| **로봇 모델** (URDF, 시뮬 설정) | 크기, 질량, 무게중심, 관성, 바퀴 위치·반경, 충돌 형상, 마찰 | `ros2/urdf/wingbot_ugv.urdf`, `config/sim_drive.json`, `ros2/launch/robot_description.launch.py` | `calib_generate.py` |
| **구동 보정표** | 최대 속도·가속도, 회전 성능, 정지거리, 좌우 편차, 주행면별 슬립·마찰 | `calib/outputs/drive_calibration.yaml` / `.md` | `calib_run.py` → `calib_fit.py` → `calib_generate.py` |
| **센서 보정 파일** | LiDAR·카메라·IMU 장착 TF, 카메라 내부 파라미터, 오도메트리 오차, 센서 지연 | `calib/outputs/sensor_calibration.yaml`, `ros2/config/camera_info.yaml` | 위 + 카메라는 `calib_fit.py --camera-images` |
| **항법 설정** | footprint, 장애물 여유거리, 경로 추종·복구 파라미터 | `ros2/config/nav2_params.calibrated.yaml`, `ros2/behavior_trees/navigate_replan_fast.calibrated.xml`, `config/robot_calibrated.json` | `calib_generate.py` |
| **측정 기록** | 직진·제자리 회전·급정지·장애물 회피 시험 데이터 + rosbag | `calib/records/<날짜_io>/` (`*.csv`, `*.json`, `manifest.json`, `bag_*/`) | `calib_run.py` |

단일 출처는 `calib/robot_profile.yaml`(값마다 출처 표기). 생성 파일은 직접 고치지 않는다.

## 절차
```bash
# 0) 현재 가정 목록
python scripts/calib_report.py

# 1) 측정 (로봇 드라이버가 /cmd_vel /odom /imu /scan 을 내는 ROS2 환경, 주행면마다 반복)
python3 scripts/calib_run.py --io ros --surface tile   --experiments static speed_steps turn_steps straight rotate brake avoid
python3 scripts/calib_run.py --io ros --surface carpet --experiments straight brake speed_steps
python3 scripts/calib_run.py --io ros --surface tile --experiments lidar_square --d-front 1.20 --d-left 0.80
#    참값: 모캡이 있으면 --truth-topic /ground_truth (nav_msgs/Odometry). 없으면 시험마다 줄자·각도기 값을 입력하거나 --manual-truth 파일.
#    rosbag 은 시험마다 calib/records/<run>/bag_<시험>/ 에 자동 기록 (--no-bag 로 끔)

# 2) 분석 → 프로파일 갱신(원본은 .bak) + 보고서 calib/reports/
python scripts/calib_fit.py --records calib/records/<run> --ref-surface tile
python scripts/calib_fit.py --camera-images "calib/camera/*.png" --board 9x6 --square 0.025 --camera-model fisheye    # OpenCV 4.x
python scripts/calib_fit.py --mu-pull tile=6.1 carpet=9.4          # 견인 시험(스프링저울 N) → 마찰계수

# 3) 산출물 생성 → 적용
python scripts/calib_generate.py
ros2 launch ros2/launch/maicon_nav.launch.py params_file:=$PWD/ros2/config/nav2_params.calibrated.yaml \
     bt_xml:=$PWD/ros2/behavior_trees/navigate_replan_fast.calibrated.xml
python3 scripts/run_real.py ... --config config/robot_calibrated.json
```

## 시험과 알아내는 값 (`maicon26/calib/experiments.py`)
| 시험 | 필요한 공간 | 알아내는 값 |
|---|---|---|
| `static` | 정지 20 s | 오도메트리 잡음, 오도메트리·IMU 자이로 편향 |
| `speed_steps` | 직선 3.5 m | 최대 속도(전/후진), 가속도, 명령→움직임 지연, 상승시간, 오도메트리 지연 |
| `turn_steps` | 1×1 m | 최대 각속도, 각가속도, IMU 지연·스케일 |
| `straight` | 직선 3 m | 직진 좌우 편차(rad/m), 오도메트리 선형 스케일, 주행면별 슬립 |
| `rotate` | 1×1 m | 제자리 회전 효율, 오도메트리 회전 스케일, 회전 중 밀림(원 궤적 반경), 정지 후 오버슈트 |
| `brake` | 직선 3 m | 제동 감속도, **속도별 정지거리 표**, 제동 지연, 마찰계수 하한(= 제동 감속 ÷ g) |
| `lidar_square` | 벽 모서리 | LiDAR 장착 x, y, yaw (정면·왼쪽 두 벽까지 줄자 거리 필요) |
| `avoid` | 직선 3 m + 박스 | 경로 위 박스 회피: 성공, 시간, **최소 이격**, 복구 횟수 (실기는 Nav2 필요) |

## 항법 설정이 보정표에서 유도되는 방식 (`generate.py: derive_nav`)
- 주행 속도 = `min(0.75·v_max, 정지거리 ≤ 0.25 m 가 되는 최대 속도)` — 정지거리 표(없으면 `지연·v + v²/2a` 모델)를 쓴다. 제동이 약하면 자동으로 느려진다.
- LiDAR 정면 안전 정지 거리 = 내접반경 + 그 속도의 정지거리 + 0.02 m.
- 회전 접근 속도 = `0.7·w_max`, 최대 각가속도 = 측정값, 복구 spin 속도·가속은 `w_max`·`alpha_max` 에 비례.
- footprint 는 **직사각형 `model.size`(250×230 mm, PDF) + 계획 여유**(`sensors.lidar.error.*` 의 LiDAR 오차·위치추정 오차에서 유도, 전부 assumed). 예전 원형 0.12 는 폐기. 근거: `docs/차체외곽_LiDAR오차_경기장유효성.md`.
- LiDAR 정면 정지 거리 = 앞 반길이 + 정지 여유(센서 오차 + 지연×속도). 제동거리는 `slow_dist` 감속 구간이 맡는다.
- 주행면별 값은 표에만 기록한다. Nav2 는 한 가지 값만 쓰므로 가장 불리한(느린) 면을 기준으로 하려면 프로파일의 `drive.*` 를 그 면 값으로 두고 생성한다.

## 검증 (시뮬; 실기 검증이 아님)
`sim/drive.py`의 숨은 정답(지연 0.12 s, 제동 1.9 m/s², 편차 0.025 rad/m, 오도메트리 스케일 1.03, LiDAR 오프셋 등)을 측정→분석이 복원하는지 본다.
- 직접 시뮬 기록(참값 연속): `tests/test_calib.py`
- **ROS 경로 전체**(WSL Humble): `bash scripts/calib_sim_ros_demo.sh` — 시뮬 로봇을 ROS 토픽으로 내고 `calib_run.py --io ros` 가 `/cmd_vel` 로 구동하며 `ros2 bag record` 로 기록 → `calib/records/demo_sim_ros/`
  - 같은 기록에서 `python scripts/calib_validate_sim.py --records calib/records/demo_sim_ros` → `truth_comparison.md` (18/18 항목이 허용 범위 안)
  - 허용 오차는 결과를 본 뒤 정했다(특히 0.3 s 미만 과도인 제동 감속·각가속도는 ±15 %). **정지거리 표는 직접 측정이라 정확하고, 항법 설정은 표를 우선 쓴다.**
- 발견·수정한 문제: ROS 로 읽은 참값 표본의 약 40 %가 중복(발행 지터) → 그대로 미분하면 속도가 18 % 틀렸다. 메시지 header 시각으로 재구성하도록 고쳤다(`odom_t/truth_t/imu_t` 열).
- 카메라 내부 파라미터: 합성 어안 데이터에서 fx/fy 0.2 % 이내 복원(OpenCV 4.5.4, WSL). **OpenCV 5.0.0(Windows pip)의 `cv2.fisheye.calibrate`는 수렴하지 않았다** → 4.x 사용.
- URDF: `check_urdf` 통과, `robot_state_publisher` 로 TF 확인(`base_link→laser` 등 프로파일 값과 일치). xacro 패키지가 없어 순수 URDF 로 생성했다.

## 한계 (정직하게)
- 실기 측정값이 없다. `robot_profile.yaml` 의 바퀴 반경·위치, 무게중심, 섀시 높이, 센서 장착 위치, 카메라 내부 파라미터 등 60개가 `assumed`.
- 관성은 직육면체 근사(질량·외형 치수). 정밀하게는 이중 진자 시험이나 CAD 값을 `model.inertia_override` 에 넣는다.
- 마찰계수는 제동 감속에서 구한 **하한값**이다. 정확한 값은 견인 시험(`--mu-pull`).
- 팬/틸트는 URDF 에서 고정(0°) 조인트다. 카메라 TF 는 팬/틸트 각도가 0°일 때만 맞다.
- 회피 시험은 시뮬에서 내장 A*, 실기에서 Nav2 액션을 쓴다(실기 Nav2 경로는 이 PC에서 박스를 놓는 시험을 하지 못했다).
- 가속도·지연 추정은 표본 주기(20 ms)·발행 지터로 ±1~2 표본(±20~40 ms) 오차가 있다.
