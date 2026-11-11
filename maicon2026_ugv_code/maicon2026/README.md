# MAICON 2026 UGV 임무 코드 (maicon26)

처음 읽는 사람은 [저장소 첫 화면](../../README.md)에서 설치·실행 순서와 폴더 지도를 확인하세요. [문서 목차](docs/README.md)와 [후보 02 결과 안내](results/README.md)도 있습니다. 아래 내용은 개발 과정의 상세 설명과 시점별 검증 기록을 보존한 것입니다.

> **후보 02 삭제 (2026-11-11).** 동결본 `runtime/selected_candidate02`와 선택 프로필 `config/active_candidate.json`을 지웠다. Gazebo 실행기는 이제 개발 코드 `maicon26/`를 그대로 실행한다(과거 후보 02 수치는 해당 시점 동결본의 기록이며 현재 코드의 결과가 아니다). 삭제 직전 사본은 `_hw_work/candidate02_runtime_backup.tar.gz`에 있다. WSL에서 `bash scripts/run_selected_gazebo.sh`로 실행한다. [선택 근거](results/SELECTION.md)와 [후속 Gazebo 4경기](results/candidate02_four_runs/RESULTS.md)를 구분해 읽어야 한다.

> 이전 배포본 검증 기록(2026-11-09): 당시 독립 ZIP 추출본에서 **1,352개 통과·실패0·skip0**, pyflakes 경고0, API 추적97/97을 확인했습니다. 현재 후보 02에 대한 새 전체 회귀 결과는 아닙니다. [검증 원본](docs/RELEASE_VALIDATION.json)을 보존했고, 중복 배포 ZIP은 후속 정리에서 삭제했습니다. [실제 정리 기록](results/cleanup_manifest.json)을 참고하세요.

> 2026-11-09 새 경기장: 제공 YAML 기준 **6.00×4.80m MAICON Gazebo 경기장**을 구성했습니다. [실행 안내](ros2/maicon_arena_gz/README.md)와 [실제 검증 범위](docs/ARENA_ACCEPTANCE.md)를 참고하세요. GPU 화면·CPU 센서 분리 실행, 표적/M3 근접면 방향, 여섯 거점 빔, 60초 성능을 확인했습니다. 최종 전체 회귀와 배포 ZIP 내부 검증은 진행 중이며 아래 이전 시뮬레이션 기록과 구분합니다.

> 2026-11-09 Gazebo 개선: [UAV 다점 격자 좌표 보정](docs/uav_calibration.md), [3인칭 녹화](docs/gazebo_recording.md). 고정 경기장 전용 `SLAM=arena`는 알려진 출발 거점과 기존 벽·구조물, 실제 LiDAR/바퀴 관측으로 위치를 추정하며 랜덤 M3 위치는 입력하지 않습니다. M3는 지상에서 사진을 확인한 박스의 네 면을 추적하고, 생성 시 각 면의 촬영 공간과 진입 경로를 검사합니다. 인식 분류는 아직 모의 검출이며 실제 YOLO 정확도를 검증한 결과는 아닙니다.

> 2026-11-07: [Waveshare 위키 반영 내역](docs/Waveshare_위키_반영_2026-11-07.md). Agent/ROS 설정 동기화, 실기 깊이 보정 검증, M1·M6용 Ollama 어댑터, Gazebo 센서·시계 처리를 개선했습니다.

> 2026-11-06 실기 통합 개정: [실기 재구현 가이드](docs/실기_재구현_가이드.md)를 먼저 읽으세요. Waveshare `ros2-humble-develop` 기반 실행은 `config/waveshare_real.json`과 `maicon_waveshare.launch.py`를 사용합니다. `run_real.py --preflight-only`로 준비 상태를 확인할 수 있습니다. 기존 시뮬레이션 설정은 유지했습니다.

WINGBOT-UGV로 본선 임무 M1~M6와 사전 서브게임을 수행하는 코드. **로봇·서버 실물에서는 아직 실행해 보지 못했다**(시뮬레이터 + 모의 서버/FakeTransport 시험).
2차 사전교육 자료(로봇 ROS2 API)와 「참가팀 단계별 API 안내」(경기 서버 HTTP API)를 반영했다 — `docs/2차멘토링_API_반영.md`.
`hal/wingbot.py`(+`hal/ros_mil.py`)와 `link/http.py`는 그 규격대로 구현돼 있고, 규격에 없는 값(LiDAR 토픽, `/mil/*` 타입 이름, 일부 판정 규칙)은 `[가정]`으로 표시했다.

## 빠른 시작
```bash
pip install numpy pytest          # 실기/학습 시 ultralytics, opencv-python, pyyaml 추가
python scripts/check_command.py "찰리와 델타 구역의 레이더 시설을 타격하고 군수보급소로 진입하라"
python scripts/run_sim.py --seed 3            # 3 vs 3 전체 경기 (약 30~40 s)
python scripts/run_sim.py --seed 1 --solo B2  # 한 대만
python scripts/run_subgame.py --n 10          # 사전 서브게임(뒤돌아 시작)
python scripts/run_sim.py --seed 2 --solo B2 --shuffle 60 150   # 경기 중 60 s, 150 s에 구조물(박스) 재배치
python scripts/run_sim.py --seed 2 --solo R2 --nav nav2-fake    # Nav2Stack 경계(앵커링·map 프레임)를 시뮬 대역으로 검증
python scripts/run_sim.py --solo B2 --seed 2 --low-barrier --map3d-out out.ply   # 깊이 3D 지도: LiDAR 에 안 보이는 낮은 장벽 회피 + 3D 지도를 .ply 로 저장 (--no-depth 로 끄고 비교)
python -m pytest -q tests                      # 전체 회귀 검사(약 25분); Windows에서는 먼저 PYTHONUTF8=1 환경변수 설정
python scripts/field_exp.py list               # 대회장 현장 실험 F01~F10 (스켈레톤; 시뮬 드라이런만 확인) — 아래 "현장 실험" 절, 체크리스트 field/FIELD_CHECKLIST.md
python scripts/run_real.py --preflight-only    # (실기) 센서·TF·서비스 준비 점검. 공식 실행은 프로세스당 한 경기
python scripts/calib_report.py                 # 아직 가정인 로봇 파라미터 목록 (실기에서 측정할 것)
bash scripts/calib_sim_ros_demo.sh             # (WSL) 시뮬 로봇을 ROS 로 내보내 보정 시험·rosbag 기록 전 과정 리허설
# WSL Ubuntu 22.04 + ROS2 Humble: 시뮬 로봇을 ROS 센서로 연결해 실제 SLAM+Nav2로 주행 (약 4~5분, 실시간)
bash scripts/ros_sim_demo.sh cartographer --robot B2 --seed 2 --shuffle 60 150
```

## 구조
| 경로 | 역할 |
|---|---|
| `hal/base.py` | `Robot` 추상 인터페이스 (임무 코드는 이것만 본다) |
| `hal/wingbot.py`, `hal/mil_api.py`, `hal/ros_mil.py` | **실기 어댑터(2차 자료 ROS2 API)**: `/cmd_vel` 10 Hz 반복·±0.6/±3 한계·워치독, `/mil/gimbal`(위치값 단위 변환·위치 추정), `/mil/fire`(탄약·발광·재장전 추정), `/mil/match`·`/mil/state`(제어 허용·피격·배터리), `/mil/oled`, `/image_raw`·`/imu/data`. `mil_api.py` 는 rclpy 없이 시험되는 순수 규칙, `ros_mil.py` 는 rclpy 전송 |
| `team/team_node.py` | 대회 규정 진입점 `~/team/team_node.py` (한 경기 후 종료. 지도·앵커 복원 없는 경기 도중 재시작은 차단) |
| `hal/diffdrive.py` | **차동(스키드 스티어) 구동 변환**: (v, w)↔좌우 바퀴 속도, 슬립·직진 편차 보상, 바퀴 속도 한계(곡률 유지 비례 축소). 아래 "차동 구동" 절 |
| `link/base.py`, `link/schema.py`, `link/http.py` | 서버 연결: `/me` phase·긴 폴링(`/order`, `/target`)을 START/명령/거점/STOP 메시지로 바꾸고, 준비완료 핸드셰이크, M1·M3·M6 JSON 변환(`RED_RADAR`, `CM`, `military_truck`, `R-2`…). 모의 서버로만 시험 |
| `nav/stack.py` | **`NavStack`**: 위치추정(SLAM)+지도+경로계획(Nav2)을 Agent에 주는 단일 인터페이스. `config.nav.backend`로 선택 |
| `nav/nav2_stack.py`, `nav/ros_nav2.py`, `nav/ros_runtime.py` | **SLAM + Nav2 백엔드(실기 기본)**: 경기장↔map 좌표 앵커링, `NavigateToPose` 액션, `/cmd_vel_nav2` 중재, 외벽 점군 발행. rclpy는 `ros_nav2.py`에서만 import |
| `nav/local_stack.py`, `localize.py`, `navigator.py`, `grid.py` | 내장 백엔드(폴백·시뮬): 파티클필터 위치추정+점유격자 매핑, A*, 추종, 막힘 복구, 경로 무효화 즉시 재계획 |
| `nav/safety.py` | LiDAR 안전 제한(어느 백엔드든 Nav2보다 우선) |
| `ros2/` | Nav2·SLAM 파라미터, BT, launch (Humble) — WSL 시뮬 연결까지 검증, 실기는 `ros2/README.md`의 단계별 확인 |
| `calib/`, `maicon26/calib/` | **로봇 보정**: 프로파일(값+출처) → 측정 시험 8종(CSV+rosbag) → 분석 → URDF·구동 보정표·센서 보정·Nav2 설정·Config 생성. 실기 측정값은 아직 없음(60개 가정) — `calib/README.md` |
| `perception/` | 카메라 모델(방위·크기 기반 거리), 검출기(YOLO 래퍼/비동기), ObjectMap(위치 투표), UGV 트래커(카메라+UAV) |
| `mission/command.py` | 한국어 명령 규칙 파서 (+선택적 LLM 병합) → M1 JSON |
| `mission/manager.py` | Agent: **상태 전이표 구현** — WAIT/EXPLORE/STRIKE(APPROACH·AIM)/STAGE/GOAL/RECOVER/SAFE_STOP/HALT/REPORT/DONE (HUNT는 기본 OFF), 아래 "상태 전이" 절 |
| `mission/guard.py`, `mission/submit.py` | 상태 전이 부품: START 래치·match_id/순번 검증, LiDAR 신선도 감시, 발사/주행 잠금 분리, 제출 ACK·재시도 추적 |
| `mission/env.py`, `perception/led.py` | **이벤트 요소(규정 p25 등)**: 스모그·드론 영상 주기·3색 타워 감시기와 고립 반사 필터 / 표적 시그널 LED·UGV 후면 LED·타워 LED 판독. 아래 "이벤트 요소" 절 |
| `geom3d.py`, `perception/voxel_map.py`, `perception/depth_fusion.py`, `hal/depth_io.py`, `calib/depth.py` | **3D 지도**: TF 기구학(RGB: 차체→팬→틸트→카메라, 깊이: 차체에 고정)·핀홀 K / 3D 복셀 점유 지도 / 깊이 등록·픽셀→3D 점·사선·전방 거리·깊이↔LiDAR 감시 / ROS 깊이 메시지 디코드 / 바닥 평면 설치 보정. 아래 "3D 지도" 절 |
| `mission/runner.py` | **`MatchRunner`**: 경기마다 새 Agent 를 만들어 연속 경기를 돌리고, 정지 중 먼저 온 다음 경기 메시지를 새 Agent 에 넘김 |
| `hal/clock.py` | `StampSync`: 센서 stamp 를 `robot.now()` 시계로 맞춤(같은 시계면 그대로) |
| `link/async_link.py`, `nav/cmd_cache.py` | 느린 서버 호출을 제어 루프 밖으로(STOP 경로 분리) / Nav2 목표 전환 때 이전 목표의 속도를 버리는 게이트 |
| `mission/aim.py` | 비주얼 서보 조준(**수광센서 자체를 겨냥**) + 레이저 관리(8발·5 s·재장전·2 s 유지) |
| `perception/receiver.py` | 수광센서 찾기: 표적 면의 흰 원판 / UGV 의 좌우 흰 돔을 영상에서 직접 검출 (PDF 사진 기준, 아래 "수광센서 겨냥" 절) |
| `mission/threat.py` | M6 위험도(생존·잔탄·인식점수 가중) |
| `perception/coverage.py`, `mission/explore_online.py` | **M3 온라인 탐색**: 실제로 훑은 곳 지도 + 박스·면 장부 / 프런티어·다음 최선 시점·마감 |
| `perception/lidar_blobs.py`, `perception/threat_board.py`, `mission/threat_field.py`, `nav/risk_astar.py`, `mission/avoid.py` | **적 UGV 회피**: LiDAR 동적 군집 / 위협 보드 / 시간 층 위협장 / 위험 인지 경로 추정기(A* 대체 아님) / 회피 제어기 |
| `sim/hostile.py`, `sim/scenarios.py`, `sim/bench.py` | 시험용 적대 상대·시나리오·지표 수집 |
| `mission/subgame.py` | 사전 서브게임: 탐색(회전)→접근→조준→발사 |
| `sim/` | 2D 시뮬(카메라 투영·가림·LiDAR·레이저 규칙·UAV 지연), 채점 |

## 수광센서 겨냥 (`perception/receiver.py`, `mission/aim.py`)
bbox 중앙이나 bbox 안 고정 비율이 아니라 **수광센서 자체**를 맞춘다. 모양은 사전 PDF 사진에서 정했다(예시 이미지이고 실물 사진이 아니다).

| 대상 | PDF 근거 | 찾는 방법 |
|---|---|---|
| 지정표적 3종 | p12: 면 이미지 **상단 중앙의 순백색 원판**(가로 50 %, 세로 약 16 %, 지름 약 15 %) = "시설 이미지 상단 레이저 수광장치". 원판은 물체(초소·레이더·통신탑) 위 하늘 쪽에 있다 | 순백색 덩어리 중 원형도·채움비가 높고 안이 균일하며 둘레와 대비가 있는 것. 흰 구름은 가장자리가 흐려 걸러지고, 상단 중앙 사전분포를 점수에 더함 |
| 상대 UGV | p21: **좌/우 앞 모서리의 하얀 반투명 돔 두 개**("레이저 수광센서(좌/우)"), 안테나의 빨강/검정 띠가 피아식별 | 검은 차체 위 흰 덩어리 중 크기·솔리디티가 맞는 한 쌍(높이가 비슷, 좌우로 떨어짐). 작은 정면 램프(크기)·은색 명판(가로로 길다)은 제외 |

- 검출기가 `Detection.extra["sensors"] = [(u, v, 반경 px, 신뢰도)]` 를 준다. YOLO 가 `RECEIVER` 클래스를 학습했으면 그 박스를 우선 쓰고, 아니면 `find_receivers` 가 영상에서 찾는다. 시뮬은 참값을 투영한 위치(노이즈 0.8 px)를 준다.
- 조준: 센서 중심에 영상 오차를 맞추고, UGV 는 두 센서 중 직전 겨냥점에 가까운 쪽을 계속 쫓는다. 허용 오차는 거리별 값과 **센서 반경의 60 %** 중 작은 쪽(하한 0.25°)이라 빔이 원 안에 머문다.
- 센서를 못 찾으면 bbox 비율 추측으로 조준은 계속하되 `aim.require_sensor` 이면 **`sensor_fallback_s`(2 s) 동안 발사하지 않는다**. 끝내 못 찾으면 추측 지점으로 쏜다(끝내 안 쏘는 것보다 낫다고 봤다).
- **시뮬 검증과 한계:** bbox 가 시설 **물체만** 덮고 흰 원이 그 밖에 있는 조건(`perception.sim_target_box="object"`)에서 서브게임 3개 조건×16시드: 센서 겨냥 48/48 성공(시도당 1발), bbox 비율 추측 0/48(272발 낭비). 반면 bbox 가 면 전체를 덮는 기본 시뮬(`"face"`)에서는 둘 다 48/48 이라 시뮬만으로는 차이가 안 보인다 — **실제 YOLO 를 어떻게 라벨링하느냐(면 전체 vs 물체)에 따라 추측 방식은 맞을 수도 틀릴 수도 있어서** 센서를 직접 찾게 했다.
- **카메라-레이저 시차 보정(`aim.laser_lateral_m`, `laser_vertical_m`):** 레이저 발광부가 카메라에서 옆·위로 떨어져 있으면 빔은 카메라 축과 평행하게 나가므로, 거리 `r` 의 표적을 맞히려면 표적이 화면 중심에서 `atan(오프셋/r)` 만큼 치우쳐 보여야 한다. 오프셋 3 cm 면 1.1 m 에서 1.6°, 2.5 m 에서 0.7° 이고 이는 센서 반경 기반 허용 오차(1.1°/0.5°)보다 커서 보정 없이는 중심에 정렬해도 빔이 원을 벗어난다. 거리 `r` 은 LiDAR/크기 기반 추정을 쓴다. **기본값은 0(미측정)이라 실측 전에는 아무것도 보정하지 않는다** — 발광부와 카메라 렌즈 사이의 옆·위 거리를 재서 넣어야 한다. 시뮬(발광부를 4 cm 옆·3 cm 위에 숨김, `laser.sim_origin_*`)에서 보정하면 서브게임 3/3 명중, 안 하면 0/3.
- 시뮬의 수광 영역도 PDF 사진 크기로 줄였다(표적 원판 반경 3.5 cm·높이 0.30~0.37 m, UGV 돔 반경 3.5 cm). 사진에서 짐작한 값이지 실측이 아니다.
- **영상 찾기는 합성 이미지로만 시험했다.** 실제 대회 조명·사진, 돔의 음영과 반사, 구름·고광택 면에서 임계값(`DISC_*`, `DOME_*`)이 맞는지는 현장 사진으로 교정해야 한다. UGV 교전(HUNT)과 3v3 명중률은 이번에 다시 재지 않았다.
- **깊이 카메라:** PDF p20~21 에 RGB-D(OAK Stereo) 깊이 카메라와 RTAB-Map 3D 지원이 명시돼 있다. 이 깊이 영상으로 3D 지도를 만들어 임무에 쓰는 부분은 아래 "3D 지도" 절.

## 3D 지도 (깊이 카메라 → 복셀 지도 → 임무)
**이전:** LiDAR 는 수평 한 평면(약 0.24 m)만 재서 2D 점유격자만 있었고 깊이 카메라는 쓰지 않았다. **지금:** OAK Stereo 깊이 영상으로 3D 복셀 지도를 만들고 항법·거리·사선 확인에 쓴다. 깊이 센서가 없으면(`get_depth()` → None) 예전과 똑같이 LiDAR 2D 로 동작한다.

**연결 (입력 4가지)**
| 입력 | 구현 | 어디서 오나 |
|---|---|---|
| 깊이 영상 | `types.DepthFrame`(m, 촬영 시각, K; pan/tilt 필드는 호환용으로 남았고 **쓰지 않는다**), `Robot.get_depth()`, `hal/depth_io.py`(`sensor_msgs/Image` 16UC1 mm·32FC1 m 디코드, 패딩·엔디언·무효 처리), `RosWingbot._on_depth` | 실기: 토픽 `config.depth.topic`(기본 `/oak/stereo/image_raw` — **가정**). 시뮬: `World.depth_image`(광선 렌더) |
| 카메라 보정값 | RGB: `camera.K` → `CameraModel`(fx, fy, 주점) 이 `aim`·`tracks`·`object_map` 모두에 쓰임(전에는 hfov 만 써서 보정값이 런타임에 안 닿았다). 깊이: `depth.K`(+`K_width/K_height`, 해상도가 다르면 비례 환산) 또는 hfov, `CameraInfo` 의 P/K 를 `camera_info_K` 가 읽음 | `calib/camera.py` → 프로파일 → `scripts/calib_generate.py` → `config/robot_calibrated.json`·`ros2/config/depth_camera_info.yaml` |
| 카메라 자세(TF) | `geom3d.RigModel`. **깊이 카메라는 2축 팬/틸트 '아래' 차체에 고정**(사용자 확인; 처음에 팬/틸트 링크에 달렸다고 잘못 가정했다가 고쳤다): 월드 ← base_link(자세, 높이 `rig.base_z`) ← depth_link(`rig.depth_xyz/rpy`, 기본 (0.10, 0, 0.10) **가정**) ← 광학. 팬/틸트를 움직여도 깊이 시야는 안 변하고 **몸체를 돌려야** 본다(전방 고정 약 72°). RGB 카메라만 base ← pan(`rig.pan_xyz`) ← tilt ← 카메라(`rig.camera_xyz/rpy`)이고 촬영 시점의 팬/틸트를 곱한다. URDF 와 같은 값(`depth_link` 부모 = `base_link`) | `calib/robot_profile.yaml` → `scripts/calib_generate.py` → `config/robot_calibrated.json`(`rig`) |
| 오도메트리·SLAM 자세 | `Agent._ingest_depth`: 깊이 `stamp` 를 `robot.now()` 시계로 바꾸고(`StampSync`, `depth.min_latency_s`) `pose_at(촬영 시각)` — 오도메트리가 예측하고 LiDAR 스캔 정합(내장 SLAM / Cartographer)이 교정한 자세 이력 — 으로 점들을 월드에 등록 | `Agent` 의 자세 이력 |

**지도:** `perception/voxel_map.py` `VoxelMap3D` — 5 cm 복셀, 2D 점유격자와 같은 원점·해상도(겹쳐 쓸 수 있음), 높이 0~0.8 m, 로그오즈(옮겨진 물체는 광선이 다시 지나며 지워짐). 바닥 점(<5 cm)은 점유로 쓰지 않고 공간만 비운다. `DepthFusion`(`perception/depth_fusion.py`)이 영상→월드 점 등록·질의를 맡는다. 파일로 저장: `python scripts/run_sim.py --solo B2 --map3d-out out.ply`(MeshLab·CloudCompare·RViz), `.npz` 는 로그오즈 전체.

**임무에서 하는 일 (4곳)**
1. **항법 — LiDAR 가 못 보는 장애물.** 바닥에서 올라온 키 높이대(0.06~0.16 m) 점유 중 **LiDAR 평면(0.24±0.05 m)에는 점유가 없는 열**만 2D 격자 층(`OccupancyGrid.layer3d`)에 더한다(내장 A*·Nav2 폴백·즉시 재계획 모두 반영). Nav2 백엔드는 같은 셀을 `/depth_obstacles_cloud`(마킹 전용 PointCloud2)로 보내고 `nav2_params.yaml` 의 두 비용지도에 `depth_obstacles` 소스를 추가했다. LiDAR 가 보는 박스·UGV·벽은 LiDAR 격자(클리어링 포함)에 맡긴다.
2. **전방 깊이 안전 정지.** 같은 'LiDAR 가 못 보는' 마스크로 차체 앞 통로 0.22 m 안에 장애물이 있으면 전진 0, 0.50 m 안에서는 비례 감속(`Agent._apply`, 모든 모드 공통).
3. **거리.** `Agent._range`: LiDAR 가 거리를 주면 그대로(mm 급), 못 주거나 크기 추정으로 떨어질 때만 깊이로 잰다(RGB 픽셀 광선을 깊이 영상에 투영해 첫 표면 — 두 카메라의 위치·시각 차이를 함께 처리). 시차 보정·사거리 판단·친구/오표적 사선 비교에 쓰인다.
4. **발사 전 3D 확인 (`_shot_geometry_ok`, M2·M4).** (a) 수광센서의 깊이 기반 높이가 기대 높이대(표적 0.22~0.46 m, UGV 돔 0.04~0.32 m) 밖이면 오검출로 보고 보류. (b) 레이저 발광부→수광센서 선분에 3D 지도의 **표면 복셀**(이웃 3개 이상)이 있으면 보류 — 깊이는 앞면 한 겹만 보이므로 '개수'가 아니라 '면에 닿는가'를 본다. 막힘이 3 s 이어지면 허상으로 보고 통과시킨다(M2 를 통째로 잃지 않게; `stats["beam_override"]`). 깊이로 못 재면 통과(모르는 것을 '나쁨'으로 보지 않는다).

**카메라 자세 점검·보정**
- 운영 중: 깊이 점 중 LiDAR 평면 높이의 것을 같은 방위의 LiDAR 거리와 비교(`DepthFusion._lidar_check`). 중앙값 잔차가 8 cm 를 넘으면 경고(`rig.depth_xyz`·시각 동기 점검). 시뮬: 정상 |잔차| < 3 cm, 카메라가 모델보다 12 cm 앞에 달리면 −12 cm 로 경고. **앞뒤(x) 어긋남만 잡는다** — 평평한 벽에서는 yaw 오차가 거리 잔차로 안 나타난다.
- 설치 전: `calib/depth.py` — 평평한 바닥에서 깊이 영상의 바닥 평면(RANSAC)으로 **roll·pitch·바닥 위 높이**를 구해 `sensors.depth.rpy[0:2]`, `xyz[2]` 에 넣는다. **앞 1.5 m 안에 박스가 없는 빈 바닥에서 16장 이상 중앙값으로** 찍을 것(시뮬: 박스가 섞인 시드에서 roll 이 2~4° 틀어졌고, 빈 바닥·16장·`max_range=1.2`·`tol=0.008` 에서는 숨은 설치 오차 roll 1.15°·pitch 2.9°·높이 −1 cm 를 1° 이내로 복원 — 카메라가 낮아 바닥이 비스듬히 보여 잡음에 민감하다). yaw·x·y 는 바닥으로는 알 수 없다 → 줄자·벽 정면 대칭으로 잴 것.

**시뮬 결과와 한계 (정직하게)**
- 시험 `tests/test_depth3d.py` 27개. 기하(TF 축·RGB 팬/틸트·깊이는 팬/틸트와 무관·URDF 값), K 환산, 복셀 갱신·질의, 렌더 깊이→등록한 점의 99 % 가 실제 표면 2 cm 안, 박스 중심 8 cm 안(4면이 보인 것) + 유령 덩어리 없음, `locate_pixel` 3 cm 안과 가림, 허상 필터, ROS 메시지 디코드, Nav2 점군 전달.
- **LiDAR 에 안 보이는 12 cm 낮은 장벽**을 BLUE 시작과 중앙 사이에 두고 솔로 B2(시드 2)를 340 s 돌린 충돌 판정 틱 수: builtin 깊이 끔 1145 → 켬 82, nav2-fake 끔 1043 → 켬 10(끄면 장벽을 계속 들이받는다; 120 s 에서는 810→82, 505→0). 주행거리도 켠 쪽이 더 길다(26.9→39.7 m, 32.6→38.0 m). 두 경우 모두 M5 는 들어갔다(243~246 s). **0 은 아니다** — 아래 '전방 고정 시야의 결과'. 처음 구현(깊이가 팬/틸트와 같이 움직인다는 틀린 가정 + 3픽셀 간격 샘플)에서는 9/1 이었는데 이는 팬 스윕이 장벽을 우연히 잘 비춘 결과였다. 고정 시야로 고치며 낮은 물체가 몇 줄밖에 안 되는 문제가 드러나 샘플을 2픽셀 간격 + 프레임마다 위치 순환으로 바꿨다. 장벽이 없는 일반 배치(모든 구조물이 0.4 m 박스)에서는 3D 층이 비어 있어, 확인한 시나리오(솔로 B2 시드 2, 구조물 변경 60·150 s, nav2-fake)에서 **결과가 깊이 끈 때와 같았다**(M5 243.9 s·충돌 0). 즉 3D 지도의 이득은 'LiDAR 평면 밖의 것이 있을 때'이며, 규정에 그런 물체가 있는지는 모른다.
- **깊이 영상 자체는 시뮬 렌더(광선 + 스테레오 잡음 모델 σ=0.004·z², 무효 3 %, 가장자리 번짐 30 %)로만 시험했다.** 실제 OAK 의 노이즈·무효 픽셀·반사·연기 속 열화는 모른다. `depth.hfov_deg`(72°)·`min_range`(0.30)·`max_range`(3.5)·토픽명·`unit_scale`·설치 위치/각도는 전부 **가정**이다.
- 번짐 허상 때문에 처음 구현은 구조물이 바뀐 뒤 '박스 윗부분 잔재'(위쪽 층은 광선이 드물게 지나 늦게 지워짐)를 장애물로 오인해 로봇이 멈췄다(가드 447틱, M5 미도달). → 바닥에서 올라온 것만, 이웃 3개 이상, LiDAR 영역 15 cm 둘레 제외로 고쳐 허상 0·결과 동일.
- 시뮬 RGB 카메라는 차체 중심(높이 `robot.cam_height`)에 있고 깊이는 리그 기구학(차체 앞 10 cm·높이 14 cm)을 따른다 — 시뮬에서 RGB↔깊이 거리 계산에 최대 수 cm 의 시뮬 쪽 불일치가 있다(실기는 둘 다 리그 값이 진실).
- **전방 고정 시야의 결과.** 깊이 카메라가 팬/틸트를 따라 움직이지 않으므로 ① 지도는 몸체가 향한 약 72° 쐐기만 쌓인다(탐색 중 360° 회전·이동으로 채워진다) ② 팬이 50° 까지 돌아간 RGB 표적은 깊이 시야 밖일 수 있어 깊이 거리·사선 확인이 '모름'으로 통과한다 ③ 카메라가 낮아(바닥 위 약 14 cm) 12 cm 장벽은 영상에서 몇 줄밖에 안 되고, 최소 거리 0.30 m 안은 못 본다 → 장벽을 비스듬히 스칠 때 충돌이 남는다.
- **실제 Cartographer + Nav2(WSL Humble, 깊이 켬) 결과 — 한 시나리오는 실패했다.** (a) 장벽 없음: M5 47.4 s, 충돌 0, 위치오차 중앙값 2.2 cm, 3D 층 0셀·점군 0점·가드 0틱, 깊이↔LiDAR 잔차 0.9 cm — 깊이를 넣기 전과 같다. 두 비용지도가 `depth_obstacles` 소스를 구독하는 것(`Subscribed to Topics: scan virtual_walls depth_obstacles`)도 확인했다. (b) **낮은 장벽 있음: 실패.** M5 미도달, 장벽에 박힌 채 충돌 1681틱, 주행거리 3.9 m, 점군 13점만 발행, 중간에 SLAM 위치오차가 80 cm 이상으로 튐(밀려난 실제 자세와 오도메트리 불일치). 시작 직후 장벽과 평행하게 달리다 가까이서 마주 향해 장벽을 거의 못 본 채 접촉했고, 점군이 마킹 전용·소수라 Nav2 가 계속 장벽 쪽 속도를 냈다. `nav2-fake`(대역)는 같은 장벽을 통과하지만 이 대역은 점군을 즉시·통째로 반영하므로 **실제 Nav2 에서의 효과는 확인되지 않았다.** **그 뒤 접촉(정체) 감지를 넣었다**(`nav/stall.py`, `Agent._stall_check`): 전진 명령(≥0.08 m/s)이 1.5 s 이어지는데 SLAM 자세가 기대 이동의 30 % 미만이고 LiDAR 앞(±25°)이 0.5 m 넘게 비었으면 '보이지 않는 장애물'로 보고 차체 앞 0.22 m 에 장애물을 기록(90 s)한 뒤 RECOVER — 깊이 층과 같은 경로(내장 A*·Nav2 점군)로 계획에 알린다. 바퀴 오도메트리는 헛돌아도 이동으로 재므로 쓰지 않는다. 시뮬(깊이 끈 채): 충돌 판정 틱 builtin 1145→130, nav2-fake 1043→95. **실제 Nav2 는 아직 불안정하다:** 접촉 감지 도입 뒤 장벽 시나리오 5회 중 M5 도달 2회(47.1 s·45.8 s), 미도달 3회(충돌 116·79·42, 접촉 감지 발동 2회 확인), 깊이 끈 1회는 충돌 722 로 미도달. 실행마다 Nav2 경로가 달라(실시간 비동기) 표본이 작고, 10회 배치는 중단했다 → **현장 실험 F05·F07 로 확인할 항목**이다. 또한 두 실행 모두 초기 `fallbacks: 1`(Nav2 목표 거절)이 보이는데 깊이 도입 전부터 있던 현상이다(액션 서버 기동 전 거절 → 내장 항법으로 폴백).
- Nav2 로의 점군은 **마킹 전용**이라 사라진 물체의 표시를 LiDAR 클리어링이 못 지운다(낮은 물체는 정적이라고 보고 UGV 같은 LiDAR 가 보는 것은 보내지 않는다). 시뮬 대역(`FakeNav2Client`)은 최신 점군으로 통째로 바꾼다.
- RTAB-Map 은 이 PC 에 없어 실행하지 못했다. `ros2/launch/maicon_depth.launch.py` 는 깊이→점군(`depth_image_proc`)과 RTAB-Map 을 **선택**으로 띄우는 미검증 템플릿이고, 임무 루프의 3D 지도는 Agent 가 자체적으로 만든다.
- 구현하지 않음: 3D 지도로 객체(M3)·UGV 분류, 3D 기반 탐색 시점 선정, 스모그 감지에 깊이 무효 비율 사용, 깊이 기반 UGV 추적. 깊이로 박스 윗면(0.4 m, 카메라보다 높음)은 못 본다(앞면·옆면만).

## 현장 실험 (대회장) — 스켈레톤 (`maicon26/field/`, `scripts/field_exp.py`)
**상태: 시뮬 로봇으로 코드가 도는지만 확인했다(시험 30개). 실기에서 실행한 적이 없고, 실험 결과 수치는 아직 하나도 없다.** `config/field_overrides.json` 은 빈 템플릿이다(시험이 비어 있는지 지킨다).

**구성**
| 파일 | 역할 |
|---|---|
| `maicon26/field/tunables.py` | **튜너블 레지스트리**: 코드 속 숫자 중 현장 확인 대상 약 130개의 상태(`PDF`/`ASSUMED`/`SIM_TUNED`/`MEASURED`), 지금 값의 근거, 현장 확인법, 합격 기준, 우선순위, 담당 실험. 값 자체는 복사하지 않고 `Config` 에서 읽는다(시험이 경로 불일치를 잡는다) |
| `maicon26/field/experiments.py` | 실험 F01~F10: 각각 `run`(데이터 수집) + `analyze`(지표·갱신 후보·판정 PASS/CHECK/FAIL). 기록은 `field/records/<날짜>/F0x_*.json` |
| `maicon26/field/overrides.py`, `config/field_overrides.json` | 사람이 확인한 값만 기록(실험·날짜·메모·이전 값). 기본값(`config.py`)은 건드리지 않고, 되돌릴 수 있다 |
| `maicon26/field/runner.py` | 시뮬 드라이런 환경(환경 자동 배치) / 실기 환경(대화형 안내), `field/FIELD_CHECKLIST.md` 생성 |
| `field/FIELD_CHECKLIST.md` | **현장에 들고 갈 체크리스트**(자동 생성): 실험 순서, 항목별 현재값·상태·근거·확인법·합격 기준·체크칸, '시뮬에서 정한 값 모음' |
| `scripts/field_exp.py` | `list` / `run` / `suggest` / `apply` / `status [--todo]` / `revert` / `checklist --write` / `replay-depth` |

**실험 목록** (권장 순서 F09 → F01 → F02 → F04 → F03 → F06 → F05 → F07 → F10 → F08)
| ID | 무엇을 재나 | 갱신 후보 |
|---|---|---|
| F09 | 줄자 값 입력: 깊이·RGB·LiDAR 설치 위치, 바퀴·로봇 치수, 레이저 발광부 위치, 경기장 치수 | `rig.*`, `robot.*`, `aim.laser_*`, `arena.*`, `map3d.*_z` |
| F01 | LiDAR/카메라/깊이 주기·지연·시계 모드(같은 시계인가) | `mission.lidar_stale_s`, `vision_stale_s`, `map3d.sync_max_dt` |
| F02 | 깊이 설치 roll·pitch·높이(바닥 평면, 빈 바닥) | `rig.depth_rpy`, `rig.depth_xyz[z]` |
| F04 | 깊이↔LiDAR 앞뒤 정합(제자리 회전) | `rig.depth_xyz[x]` |
| F03 | 깊이 거리 정확도·유효율(벽, 줄자) | `depth.min/max_range`, `unit_scale`, `sim_noise_k` |
| F06 | 낮은 물체가 없는 환경의 3D 층 허상 수 | `map3d.nav_thr`, `blind_grow_cells` |
| F05 | 낮은 블록을 몇 m 에서 잡나 | `map3d.guard_slow`, `nav_thr`, `nav_min_neighbors` |
| F07 | 자유 주행 vs 보이지 않는 장애물에 막힘의 이동비 | `map3d.bump_ratio` |
| F10 | 레이저↔카메라 평행 오프셋·축 각도(두 거리에서 점 위치) | `aim.laser_lateral_m/vertical_m`, `laser_offset_pan/tilt_deg` |
| F08 | RGB·깊이·LiDAR·오도메트리 저장 → `replay-depth` 로 설정을 바꿔 가며 오프라인 재생(스모그·타워·LED·낮은 블록 임계 튜닝) | `lidar.self_mask_*`, `event.*` 등은 사람이 보고 |

**사용**
```bash
python scripts/field_exp.py list
python scripts/field_exp.py run F09 --robot ros                 # 실기: 대화형. (--robot wingbot 은 WingbotRobot; 둘 다 hal/wingbot.py TODO(API) 가 채워져야 동작)
python scripts/field_exp.py suggest field/records/<날짜>/F02_*.json
python scripts/field_exp.py apply   field/records/<날짜>/F02_*.json     # 값마다 y/N → config/field_overrides.json (시뮬 기록은 기본으로 건너뜀)
python scripts/field_exp.py status --todo                        # 실측으로 바뀐 것 / 아직 안 바뀐 우선순위 1
python scripts/run_real.py ... --overrides                       # 반영된 값으로 실행(경기 코드)
python scripts/field_exp.py run F10 --robot sim                  # 코드 확인용 드라이런 (기록에 '시뮬 — 적용 금지' 경고가 붙는다)
```
**안전·주의**: 레이저는 `--allow-laser` 없이 절대 자동 발사하지 않는다. 구동은 ≤0.2 m/s, 끝나면 반드시 정지 명령. F07 은 모터가 구속되는 구간이 있다(3.5~4 s). 실험들의 합격 기준·갱신 규칙은 `FIELD_CHECKLIST.md` 와 각 `analyze` 에 있고, **시뮬 드라이런의 수치는 코드 확인용이며 설정에 적용하지 않는다.**

**값 갱신 방법**: 시뮬로 정한 값은 `tunables.py` 에서 `SIM_TUNED` + `basis`(무엇을 보고 정했나)로 표시했다. 현장에서 값이 바뀌면 `apply` 가 오버라이드에 날짜·실험 ID 와 함께 쓰고 `status` 에 `MEASURED` 로 나온다. 레지스트리에 항목을 추가/수정한 뒤에는 `python scripts/field_exp.py checklist --write` 로 체크리스트를 다시 만든다(시험이 오래됐는지 확인한다).

**아직 없는 것**: F01 의 센서 '최소 지연' 측정(로봇을 돌리며 정지 표적 방위 오차로 찾는 방법, `hal/clock.py` 문서), M2/M4 사격 명중률 현장 시험(기존 `scripts/run_subgame.py` 는 시뮬 전용), 스모그·타워·LED 영상 판독 임계 교정 도구(F08 로 저장한 뒤 사람이 본다), 실기 HAL.

## 차동 구동 (`hal/diffdrive.py`)
**가정:** WINGBOT 은 6WD 이고 PDF 에 조향 서보 언급이 없어, 좌우 바퀴 속도 차로 도는 스키드 스티어로 본다. **확인된 사실이 아니다**(11/6 멘토링에서 확인).

- Agent 는 여전히 차체 속도 `(v, w)` 를 정한다. 로봇에 보내기 직전(`Agent._apply`)에 `DiffDrive.command` 를 한 번 거친다.
  1. **보상:** `w_cmd = (w_원하는 − lr_bias·v) / yaw_eff`. 보정으로 잰 제자리 회전 효율(`drive.pivot.yaw_efficiency`)과 직진 편차(`drive.lr_bias_rad_per_m`)를 미리 뺀다. 측정 전 값은 중립(1.0, 0.0)이라 아무것도 바꾸지 않는다.
  2. **바퀴 한계:** `v_L = v − w·b/2`, `v_R = v + w·b/2` 가 `wheel_v_max`(0 이면 `v_max`)를 넘으면 `v` 와 `w` 를 **같은 비율로** 줄여 곡률을 유지한다. 시뮬 솔로 경기에서 약 2 % 틱에서 작동했고 경기 시간·충돌은 변하지 않았다.
- 바퀴 속도를 직접 받는 드라이버면 `WingbotRobot.wheel_command(v, w)` 가 좌우 선속도(m/s)와 각속도(rad/s)를 준다. 어느 쪽인지(`/cmd_vel` vs 바퀴 속도)는 아직 모른다.
- 설정: `robot.track_width`(0.20), `wheel_radius`(0.04), `wheel_v_max`, `yaw_eff`, `lr_bias`, `wheel_limit`. 보정 파이프라인이 `config/robot_calibrated.json` 에 바퀴 간격·반경·효율·편차를 채운다.
- **한계:** `yaw_eff` 는 제자리 회전에서 잰 값이라 주행 중 호에도 같은 값을 쓴다. `lr_bias` 는 주행면마다 다를 수 있으나 한 값만 쓴다. `track_width` 는 가정값이라 실측 전에는 바퀴 한계 계산이 부정확하다. 바퀴 속도 피드백(PID)은 여전히 없다 — 드라이버/펌웨어가 속도 루프를 갖는다고 가정한다.
- 시험: `tests/test_diffdrive.py` — 시뮬 로봇에 숨은 슬립(10 %)·편차(0.05 rad/m)를 주고, 보상이 직진 휨과 회전 부족을 지우는지 확인.

## 상태 전이 (`MAICON2026_상태전이표_경쟁상태반영.html` 적용)
한 틱의 순서: **서버 메시지·STOP 래치 → 제출 결과·재시도 → 로컬 만료 확인 → 센서·SLAM → 순위대로 전이 → 출력 1회.** STOP 이 센서·SLAM·검출·네트워크 처리에 막히지 않는다.

| 전이표 순위·경쟁 상태 | 구현 | 시험 (`tests/test_state_machine.py`) |
|---|---|---|
| 00 STOP/로컬 만료 → HALT → REPORT → DONE (R12) | STOP 래치, 시작 후 `t_total+1 s` 에 STOP 이 없으면 로컬 안전 정지. 구동 0·목표 취소·소등·세대 증가, M6 는 ACK 또는 STOP 후 30 s 까지 재시도, 결과를 acked/rejected/unconfirmed 로 구분 | SLAM 준비 전 STOP, 로컬 만료, M6 거부 지속 |
| R01 STOP ↔ 센서 지연 | 서버 poll 을 센서 처리보다 먼저 | `test_stop_before_slam_ready_halts_immediately` |
| R09 중복 START·다른 경기 메시지 | START 1회 래치, `match_id`·`seq` 검증, STOP 뒤 모든 메시지 무시 | 중복 START, 타 경기 STOP/이벤트, STOP 뒤 START |
| 03 START 인데 SLAM 미준비 | SAFE_STOP(pose_not_ready) 후 준비되면 EXPLORE | `test_start_while_slam_not_ready…` |
| 04·05 / R04 LiDAR·자세 노후화 | 새 스캔 '도착 시각' 감시(`lidar_stale_s`), SAFE_STOP 에서는 구동 0, **정지 이후 새 스캔 2개**가 들어와야 복귀 | LiDAR 정지 시나리오 |
| R08 피격 잠금 분리 | `weapon_locked`(발사·틸트) / `drive_locked`(주행, 4회째부터). 하드웨어 상태가 권한, 이벤트는 0.6 s 유예 | 주행 잠금, 발사 잠금 시나리오 |
| 08 영상 노후화 | 카메라만 멈추면 주행은 계속, 발사·객체 판정 중지 | `test_camera_stall…` |
| 06 M5 우선, 07 STAGE, 09~12 STRIKE | 기존 동작 유지 + 조준 시간 예산(`aim_budget_s`) 초과 시 재접근/포기. `state_name` 이 STRIKE_APPROACH / STRIKE_AIM 구분 | 기존 시뮬 시험 |
| 13·14 RECOVER | 항법이 `recover_after_s` 넘게 막히면 제한된 후진·회전 후 재계획. `recover_window_s` 안에 `recover_max` 회 초과 시 SAFE_STOP | `test_blocked_navigation_recovers…` |
| 15 / R14 자체 진입 추정 | GOAL 에서 구동 0 으로 대기만 한다. M6 기산·경기 종료를 확정하지 않는다(서버 STOP 이 기준) | `test_m5_local_entry_holds…` |
| R03 검출↔프레임 짝 | 비동기 검출 결과를 **그 프레임의 stamp·자세**와 묶어 한 번만 반영, 짝을 잃으면 폐기 | 지연 검출기, 고정 검출기 |
| R05 UAV 순서 | 촬영 시각이 단조 증가할 때만 반영 | `test_uav_frame_with_older_stamp…` |
| R06 Nav2 속도 캐시 | 목표 전송·취소 때 캐시를 비우고, 서버가 새 목표를 수락한 뒤의 속도만 채택(`nav/cmd_cache.py`) | 단위 시험 + WSL 실제 Nav2 주행(속도 700개 채택·1개 차단, 충돌 0) |
| R10 제출 | 서버가 False 여도 '완료'로 기록하지 않고 같은 token 으로 재시도. **M1/M3 는 STOP 전까지 무제한**(`submit_max_tries=0`; 거부가 이어지면 간격 1→2→4→5 s 로 늘림), M6 는 보고 기한(STOP 후 30 s)까지 매초. STOP 뒤 M1~M5 신규 제출 금지 | `test_m1_rejection_is_retried`, `tests/test_fixes.py` |
| R02 느린 네트워크 | `AsyncServerLink`: poll/UAV/제출을 별도 스레드로. `run_real.py` 기본 적용 | 느린 제출 중에도 STOP 즉시 수신 |
| R11 M1 늦은 LLM 응답 | 명령 버전이 다르면 폐기 | `test_late_llm_result…` |

기본 전략은 **M2 우선 · 적 UGV 회피 · M5 진입**이라 `mission.engage_ugv` 기본값을 False 로 바꿨다(M4 교전 HUNT 비활성). 적 UGV 는 LiDAR 가 동적 장애물로 보고 회피한다(트랙 기반 위험 영역 주입은 미구현).

**전이표대로 하지 않았거나 못 한 것**
- **R07 레이저 즉시 소등**: 실기 API 를 모른다. `Robot.can_abort_laser=False`(실기 기본)이면 아군 사선 보호 각을 두 배로 넓히는 것까지만 했다.
- **R13 M3 전체 갱신 제출**: 10초에 처음 제출하고, 결과가 바뀌면 5초 간격으로 누적된 전체 결과를 다시 보낸다. 280초에는 전체 스냅샷을 한 번 더 보내며 290초부터 신규·재시도·대기열 전송을 닫는다. 서버 수신 시각이 마감 기준이므로 STOP 또는 5:00 이후 도착은 거부된다. BLUE는 R-S1·R-S2·R-S3·CM, RED는 B-S1·B-S2·B-S3·CM만 채점한다. 매번 네 구역을 모두 포함하고 지정 6종 외의 그림·블록은 보고하지 않는다. 시뮬레이터도 전체 교체와 수신 마감을 동일하게 적용한다.
- 전이표의 STRIKE_APPROACH / STRIKE_AIM 은 `Mode.STRIKE` 하나 안의 단계로 두고 `state_name` 으로만 구분한다.
- `lidar_stale_s`(0.6 s), `vision_stale_s`(0.8 s) 등 워치독 임계값은 **가정**이다. 전이표 9장대로 실기 주기·지연을 재서 정해야 한다.
- 위치 노후화는 별도 시계가 없어 LiDAR 신선도로 대신한다.
- 실기 어댑터(`hal/wingbot.py`, `link/http.py`)는 규격대로 구현했지만 로봇·서버 실물에서는 돌려 보지 못해, 이 전이는 시뮬·WSL 시뮬 연결·FakeTransport/모의 서버에서만 검증됐다.

## 코드 검토에서 나온 결함과 처리 (`tests/test_fixes.py`, `tests/test_fixes2.py`)
| # | 결함 | 처리 |
|---|---|---|
| ① | M1/M3 제출이 6번 거부되면 영구 포기 | STOP 전까지 무제한 재시도(간격 1→2→4→5 s). 포기한 제출은 되살림 |
| ② | 카메라-레이저 시차가 센서 반경 허용 오차보다 큼 | 거리별 시차 보정(`aim.laser_lateral_m`, `laser_vertical_m`). **기본 0 = 미측정, 실측 전엔 효과 없음** |
| ③ | 프레임 16장을 이미지째 보관(≈240 MB) | 보관본에서 이미지 제거 |
| ④ | LiDAR 자기 가림(기둥·안테나가 스캔 평면에 들어옴)이면 전방 정지가 영구 지속 | `lidar.self_mask_sectors`(각도 구간 안의 가까운 반사만 무시)와 `min_range`. 설정이 없어도 **같은 방향·거리의 가까운 반사가 로봇이 돌아도 8 s 이상 남으면** 후보 각도를 담아 경고(자동으로 지우지는 않음). 시뮬: 전방 10 cm 반사가 있으면 60 s 에 0.7 m, 마스크하면 6.3 m |
| ⑤ | 후진은 LiDAR 안전 제한이 보호하지 않음 | 후방 ±35° 반사가 17 cm 안이면 정지, 45 cm 안이면 비례 감속. RECOVER 후진도 매 틱 적용 |
| ⑥ | 검출기가 죽어도 '신선한 빈 결과'로 위장 | 예외 때는 결과·stamp 를 갱신하지 않아 영상 신선도 감시가 발사를 막고, `health()` 와 로그(`WARN detector 문제`)로 원인 노출. 동기 검출기 예외로 `tick()` 이 죽지 않음 |
| ⑦ | 서버 연결(poll/제출) 오류가 로그에 안 나옴 | `ServerLink.health()` → `WARN link 문제: <예외 문구>`(같은 문구는 10 s 에 한 번), 회복도 로그. 11/6 전 스텁(`NotImplementedError`)도 그대로 보인다 |
| ⑧ | 센서 stamp 시계가 로봇 시계와 다르면 `pose_at` 이 이력 밖 시각을 받아 가장자리 자세로 **조용히** 고정 | ① HAL 계약: stamp 는 `robot.now()` 시계로 채운다(`hal/base.py`, `wingbot.py` 안내 정정 — 기존 머리말은 "header stamp 를 쓰라"고 해서 오히려 불일치를 만들었다). ② `hal/clock.py` `StampSync`: 같은 시계면 그대로, 다르면 `stamp + min(now−stamp)` 로 변환(스캔·프레임 도착 시점에). ③ 이력 밖 조회는 `pose_at_clamped` 통계와 `WARN pose_at`, 시계가 다르다는 판단은 `WARN frame stamp` 로 알림. 시뮬(에포크 시계·지연 50~250 ms·제자리 회전 1.8 rad/s): 검출에 입힌 방위 오차 **7.6° → 4.9°**(변환만), 카메라 최소 지연을 재서 `camera.min_latency_s` 에 맞게 넣으면 **0.5°**(같은 시계 0.33°). 틀린 값을 넣으면 틀린 만큼 남는다 |
| ⑨ | 경기 수명과 오래된 메시지 분리 | `MatchRunner`의 연속 경기 기능은 시뮬/시험용으로 유지. 공식 HTTP API에는 경기 ID가 없으므로 실기 `run_real.py`는 `--matches 1`만 허용하고, 운영 프로그램이 경기마다 새 프로세스를 실행한다. |
| ⑩ | 조준의 직전 겨냥점·필터가 모드 변경에서만 초기화 → 대상이 바뀌어도 옛 값과 섞이고 엉뚱한 쪽 센서를 고름 | `AimController` 가 **공백(`lost_timeout` 0.8 s)이나 점프(`target_jump_px` 120 px)** 를 스스로 감지해 추적을 새로 시작하고, 매니저는 표적 면·교전 대상을 새로 고를 때 명시적으로 초기화한다 |

**한계:**
- ④의 마스크는 Agent 가 받는 스캔에만 적용된다. **Cartographer/Nav2 비용지도는 원본 `/scan` 을 쓰므로** 실기에서는 드라이버나 `laser_filters`(angular bounds)에서 같은 각도를 걸러야 한다. 자기 가림 각도는 LiDAR 장착 위치를 알아야 정할 수 있어 기본은 꺼 두었다.
- ⑧의 한계: 다른 시계일 때 시계 차와 최소 지연은 데이터만으로 분리되지 않는다. **최소 지연(`camera.min_latency_s`, `lidar.min_latency_s`)은 미측정(0)이라 지금은 변환만 된다.** 재는 법은 `hal/clock.py` 머리말(일정한 각속도로 돌며 정지 표적의 검출 방위 오차가 0 이 되는 값). 하드웨어 시각 동기(PTP/chrony)가 있으면 더 좋다. 실기 카메라 API 를 몰라 `WingbotRobot.get_frame` 은 여전히 스텁이다.
- ⑨의 한계: 러너는 **SLAM(Cartographer) 재시작과 로봇을 시작 위치에 놓는 일은 하지 않는다** — 경기 사이 운영 절차다(사람이 로봇을 옮기면 `map→odom` 이 어긋날 수 있어 경기마다 `ros2 launch` 를 다시 올리는 것이 안전). `match_id` 없는 서버에서는 보고 구간에 도착한 다음 경기 START 를 잃을 수 있다(이전 경기가 DONE 된 뒤 도착한 메시지는 새 Agent 가 받는다). 시뮬에서는 두 경기를 이어 돌려 확인했고 실기 서버로는 못 했다.
- 실제 Cartographer+Nav2(WSL) 한 판(④~⑦ 후 / ⑧~⑩ 후): M5 진입 46.5 s / 46.4 s, 충돌 0, 경고 없음. 위치오차 중앙값은 4.1 cm / 1.6 cm — 같은 코드 경로에서도 실행마다 이만큼 변동한다(SLAM 은 이 변경들과 무관).

## 이벤트 요소 (본선 규정 — 상태전이표 6장, `E1~E10`)
규정의 이벤트 장치(PDF p25 '미션 이벤트 장치', p14·p22, 멘토링 11/4)를 새 행동 상태 없이 **플래그와 기존 전이의 조건**으로 처리한다. 표·경쟁 상태(R15~R22)·시험 대응은 `MAICON2026_상태전이표_경쟁상태반영.html` 6장.

| ID | 이벤트 요소 | 처리 | 시험 |
|---|---|---|---|
| E1 | 서버 경기상태 이벤트(완료) | `game_state finished` 를 STOP 과 같은 규칙(match_id·종단 래치)으로 받아 HALT(`server_event`). 센서·SLAM 준비를 기다리지 않음 | `test_game_state_finished_event…` |
| E2 | 3색 타워 LED(준비·진행·완료) | 서버 STOP 유실 때만: 시작 후 60 s↑ + '완료' 색 1.5 s 연속 + **서버 연결 이상** → HALT(`tower`). 연결이 정상이면 무시 | `test_tower_…` 4개 |
| E3 | **스모그 발생장치** | LiDAR 고립 반사 비율(전체 빔 대비 ≥4 %, 0.8 s)·영상 대비·서버 이벤트로 `SMOG` 플래그. 켜면 고립 반사 제거, 속도 ×0.8 | `test_smog_…` 9개 |
| E4 | 드론 영상 주기(1→2→5 s→중단) | 새 프레임 도착 간격으로 상태 측정, 트랙 보존 3.0→3.2→8.0 s. 중단돼도 정지하지 않음 | `test_uav_…`, `test_agent_follows_the_drone…` |
| E5 | 지정표적 시그널 LED(빨강 점등) | 영상으로 연속 3프레임 확인 → M2 완료(서버 `target_hit` 없이도) | `test_signal_led_…`, `test_led_on_needs…` |
| E6 | 지정표적 오타격(다른 팀에 가점) | 사선에 우리 것이 아닌 표적이 앞이거나 비슷한 거리면 발사 보류 | `test_other_teams_target…` |
| E7 | (E3 의 효과) 스모그 중 발사 | 검출 신뢰도 ≥0.60 일 때만 | `test_smog_slows_navigation…` |
| E8 | **UGV 후면 LED**(피격색·8칸 탄창) | 뒤쪽 1.8 m 안에서 읽어 M6 증거(생존·잔탄)에 사용 | `test_rear_leds_…`, `test_m6_ranking_uses_led…` |
| E9 | 거점 진입 이벤트 | 기록, 내 진입 확인 시각 저장(종료 확정은 STOP/E1 만) | `test_base_entered_event…` |
| E10 | 비인식용 박스 4종 | M3 집계 제외 | `test_objects_far_away…` |

**측정(시뮬):**
- 스모그: 허상 반사가 8~20 cm 까지 나오는 조건에서 처리 없이는 40 s 에 0.6 m(전방 정지가 계속 걸림), 감지·필터가 있으면 3.7 m. 센서만으로 연기 시작 후 1.4 s 에 켜지고 끝난 뒤 3.3 s 에 꺼짐. 초기에 '가까운 반사 수'를 분모로 잡았다가 시작 거점(벽이 가까움)에서 희석돼 놓쳐 **전체 빔 수**로 바꿨다.
- 후면 LED: 잔탄 추정 오차 0.56→0.32, 생존 0.22→0.18. **하지만 M6 1순위 정답률은 오르지 않았다**(24판 중 10 대 11) — 마지막 판독이 종료 시점보다 중앙값 70 s 전이라 오래됐고 객체 인식 점수 추정이 순위를 좌우한다.

**한계 (정직하게):**
- 서버 API 문서에는 이벤트 푸시가 없다(M2·M4·M5 는 장치가 판정하고 참가팀 API 로 알려 주지 않는다). `normalize_event` 는 시뮬 이벤트 경로 시험용으로만 남았다. 스모그 장치도 로봇에게 이벤트를 주는지 모른다 — 안 줘도 센서로 감지한다.
- **LED·타워 영상 판독은 합성 이미지로만 시험했다.** 3색 타워의 색-상태 대응(노랑=준비·초록=진행·빨강=완료), 후면 LED 의 위치·배치, 시그널 LED 크기·밝기, 연기가 만드는 LiDAR 반사 분포는 규정에 없어 **가정**이다(`config.event.*`).
- 구현하지 않음: 거점 LED 경광등 영상 판독, 출발 광전센서, 후면 LED 를 늦게 다시 확인하러 가는 행동, 스모그 중 카메라 노출 적응.
- 스모그 필터는 SMOG 중에만 켜지고 가는 장애물(상대 UGV 안테나)의 반사를 지울 수 있다 — 감속·LiDAR 안전 제한으로 완화할 뿐 없애지 못한다.

## SLAM + Nav2 (구조물이 실시간으로 바뀌는 경기장)
PDF p15: M5의 핵심 기술은 **ROS2 · SLAM · Nav2**, 로봇은 Cartographer/Gmapping/RTAB-Map 지원. 박스 배치는 경기마다(경기 중에도) 바뀔 수 있어 미리 만든 지도에 의존하지 않는다.

| 층 | 실기 (`--nav nav2`) | 폴백·시뮬 (`--nav builtin`) |
|---|---|---|
| 위치추정 | **SLAM** (Cartographer 2D 기본, slam_toolbox 대안) → TF `map→odom`. 시작 때 벽 정합으로 경기장 좌표에 앵커링 | 오도메트리 + 외벽 파티클필터 |
| 지도 | Agent용: SLAM 자세로 쌓는 LiDAR 로그오즈 격자(박스가 옮겨지면 몇 회 관측에 갱신). Nav2용: 센서 마킹+레이캐스트 클리어링 비용지도 | 같은 로그오즈 격자 |
| 계획·추종 | **Nav2** (NavFn A* + RegulatedPurePursuit, BT 2 Hz 재계획, spin/backup 복구) | A* + 추종 + 막힘 복구 + 경로 근처 새 장애물 시 즉시 재계획 |
| 최종 구동 | Nav2는 `/cmd_vel_nav2`로만 낸다. **Agent가 LiDAR 안전 제한을 적용한 뒤** `/cmd_vel` 전송 → 조준·교전과 구동 충돌 없음 | 동일 |

설계 판단 두 가지(근거는 `ros2/config/nav2_params.yaml` 머리말):
- **Nav2는 SLAM `/map`을 쓰지 않는다.** SLAM 지도는 서브맵이 누적돼 옮겨진 박스가 한동안 남는다(유령 장애물). SLAM은 위치만, 장애물은 센서 클리어링으로.
- 경기장 외벽은 알려진 값이므로 `/arena_walls_cloud`(마킹 전용)로 알려 경로가 벽 밖으로 새지 않게 한다.

**검증 상태:**
- 파이썬 쪽(좌표 변환, 액션 상태 기계, 속도·안전 중재, 앵커링, 폴백)은 시뮬 대역(`sim/fake_nav2.py`)과 단위시험으로 검증했다.
- **WSL(Ubuntu 22.04, Humble)에서 시뮬 로봇을 ROS 센서로 연결해 실제 Cartographer/slam_toolbox + Nav2로 5분 경기를 6회 돌렸다**(구조물 재배치 포함): 모두 M5 진입(244~246 s), 충돌 0, 위치오차 중앙값 1.4~3.3 cm. 자세한 표·고친 점은 `ros2/README.md`.
- **실제 로봇(WINGBOT)에서는 아직 한 번도 실행하지 못했다.** 토픽·프레임명, 풋프린트, 속도, LiDAR 장착 방향, DDS 설정은 현장 확인이 필요하다. 시뮬은 이상적인 LiDAR·키네매틱 로봇이라 실기 성능을 보장하지 않는다.

## M3 온라인 탐색 + 적 UGV 회피 (`docs/M3탐색_적회피_통합_설계.md`)
하나의 항법 구조: 임무 계층이 *다음 관측 시점/안전 경유지*를 고르고, 계획은 기존 Nav2/A*, LiDAR 안전 정지·SAFE_STOP·STOP 이 항상 최우선.
- **M3**: `explore.strategy = "online"`(기본) — 프런티어 + 면 단위 다음 최선 시점 + "실제로 본 곳" 지도(`perception/coverage.py`)와 면 장부(가려서 못 본 것과 본 것을 구분, 박스 이동 감지, 면 중복 계수 방지). `"lattice"` 로 기존 5×5 순회로 즉시 복귀. 집계 `explore.m3_counting = "ledger"|"omap"`.
- **적 회피**: `avoid.enable`(기본 True, M4 는 계속 OFF). 카메라/UAV/LiDAR 동적 군집 → `perception/threat_board.py` → `mission/threat_field.py`(물리 점유 body + 피격 위험 risk, 시간 층) → `mission/avoid.py`(AWARE/EVADE/CLOSE, 안전 경유지 carrot). A*·Nav2 계획기는 바꾸지 않는다.
- 비교 실험: `python scripts/compare_explore.py --seeds 1 2 3 4 5 6 [--shuffle 60 150]`, `python scripts/compare_avoid.py --seeds 1 2 3 --scenarios S1_front_chaser … --variants base avoid avoid_lidar_only …`
- **솔직한 요약**: 면 장부 *집계*가 M3 정밀도를 0.82→0.99 로 올렸다. 온라인 *탐색* 은 이동거리를 줄였지만(−7 %) 재현율은 고정 순회보다 약 5 %p 낮았다. 회피는 적이 쏘는 시나리오에서 피격을 크게 줄였으나 일부에서 M5 가 늦어진다. **실제 YOLO 가중치·UAV 영상 검출기·실기 Nav2 에서의 동작은 검증하지 못했다**(문서 §6).

## 차체 외곽 · LiDAR 오차 예산 · 경기장 유효성 (`docs/차체외곽_LiDAR오차_경기장유효성.md`)
- 충돌·경로계획·안전 정지가 **같은 외곽**(`maicon26/footprint.py`): 직사각형 250×230 mm [PDF p21], 외접 0.1698 m, 반폭 0.115 m. A* 하드 인플레이션 = 반폭 + 계획 여유(0.164), 제자리 회전 = 외접 + 여유(0.219), LiDAR 정지 = 앞 반길이 + 정지 여유(0.192), Nav2 footprint = 직사각형 + 여유. 작은 원(예전 0.14)과 `경로 실패 시 인플레이션 0.8·0.6배` 완화는 폐기(레거시 비교는 `Config.use_legacy_circle()`).
- LiDAR 오차는 **가정값**(`lidar_err.*`, `status="assumed"`, PDF 에 정확도 수치 없음)이고 계층별로 한 번만 더한다(센서 항은 계획·정지 각자 1회, 지연은 정지만, 위치추정은 계획만). 현장에서 `calib/robot_profile.yaml` 의 `model.size`, `sensors.lidar.error.*` 를 `measured` 로 고치면 `calib_gen` 이 Config·Nav2 footprint 를 일관되게 다시 만든다.
- 무작위 경기장은 생성 직후 정적 검사(`sim/arena_check.py`: 박스 배치 실패·이격·필수 표적/객체·출발 자세·6개 출발→상대 3개 목표거점·표적 접근 자세까지 직사각형 경로)를 통과해야 쓴다. 실패하면 결정적 다음 시드로 재생성(상한 30, 초과 시 사유와 함께 `ArenaGenerationError`), 기록은 `world.layout_meta`. `python scripts/compare_footprint.py arena|match|lidar`.
- 시뮬은 2D 직사각형이라 실기 안전성의 증거가 아니다.

## 시뮬 결과 (정직하게)
- 단독 주행: 거점↔거점 약 18 s, 위치오차 중앙값 약 1.6 cm, 시작 자세 추정 72/72, 박스 위치 중앙값 2.4 cm, 거점 표지 6/6.
- 3v3 전체 경기(시드 1~4): M1 전부 정확, M2 성공 로봇은 아군이 이미 LED를 켠 경우 재사격을 생략하므로 일부 'False'로 채점됨,
  M3 재현율 0.57~0.89 / 정밀도 0.74~1.0, M4 8발 중 명중 약 1발(움직이는 상대에 2 s 연속 수광이 어렵다), 오사 0, M6 top-1 정확도는 불안정.
  (이 수치는 상태 전이표 적용 전, M4 교전이 켜져 있던 때의 것이다. 지금 기본값은 M4 OFF 이고 이 3v3 수치는 다시 측정하지 않았다.)
- 서브게임: 거리 2.5~3.5 m, 좌우 ±1 m에서 36/36 성공, 평균 약 9 s (표적이 3 m 안이면 약 6 s).
- 항법 백엔드 비교(3v3, 시드 1~14, 내장 백엔드): 14/14 경기에서 M5로 경기 종료(241.8~245.1 s), 위치오차 중앙값 0.4 cm. 로봇 충돌 횟수(경기당 6대 합)는 중앙값 6, 평균 10.9 — **UGV끼리 엉키는 드문 경기에서 큰 값(최대 61)이 나온다.** `nav2-fake`도 시드 1~4 전부 M5 도달.
- 구조물 변경(60 s, 150 s에 중앙부 박스 6~9개 이동/추가, 솔로 B2, 시드 2·3, builtin·nav2-fake): 4/4 M5 도달, 충돌 0. 경로 위에 박스가 새로 놓이면 1 s 안에 재계획(`tests/test_nav_stack.py`).
- 실측으로 **내장 SLAM 지도를 위치추정에 쓰는 옵션(`nav.slam`)은 끈 채로 둔다**: 외벽만 쓸 때 오차 중앙값 0.4 cm, 지도 박스까지 쓰면 2 cm(자기강화). 오도메트리 오차를 4~10배로 키워도(자이로 편향 0.012 rad/s, 스케일 8%) 외벽만 쓴 쪽이 더 정확했다. 외벽이 박스에 크게 가려지는 배치에서만 켜 볼 것.
- 경로 무효화 재계획은 처음에 '새로 점유된 모든 셀'로 걸었더니 다른 UGV 표면에도 반응해 충돌이 늘었다(평균 21 vs 끈 쪽 13). 로그오즈 ≥1.8(정적 구조물)인 셀만 보도록 고쳐 평균 10.9로 내려갔다. 표본이 작고 꼬리가 길어 "개선"이 아니라 "악화 없음" 수준으로만 읽을 것.
- **시뮬 수치는 실기 성능이 아니다.** 시뮬의 가정(인식 3 m, 수광각, 잡음)이 현실과 다르다.

## 실기 투입 전 교정 목록 (값은 `config/default.json`, `[가정]` 표시)
1. `camera.hfov_deg`(110 가정)·렌즈 왜곡 보정, 카메라-레이저 축 오프셋 `aim.laser_offset_*`
2. `laser.max_range / min_range`(수광 유효거리), 수광부 위치 `aim.aim_point_*`(시설 bbox 내 위치, UGV 좌/우 센서)
3. `arena.py` 거점/구역 좌표(그림에서 근사) — 현장 실측으로 교체
4. LiDAR 장착 방향(`hal/wingbot.py LIDAR_ANGLE_OFFSET`)과 부호(`hal.pan_sign/tilt_sign`; API 는 팬 오른쪽 +, 주행 각속도는 왼쪽 +) — 제자리 회전 시 지도가 도는지, 연습 모드에서 pan=+50 이 오른쪽인지로 확인
5. 로봇 반경·최대속도·가속도 `robot.*`, 내비 인플레이션 `nav.inflate_hard`(0.17)
6. YOLO 학습 클래스명 ↔ `vocab.py` 키 매핑(`YoloDetector(names_map=...)`), UGV 안테나 색 HSV 임계(`team_from_antenna`)
7. `mission.prestage_at / strike_range / ugv_range / spin_w` 현장 튜닝
8. **깊이 카메라·3D 지도**: `depth.topic/info_topic`(OAK 토픽명), `depth.unit_scale`(16UC1 mm), 스트림 해상도와 `depth.K`(depthai `camera_info` 또는 보정), `depth.hfov_deg`(72° 가정), `depth.min_range/max_range`, **설치 위치·각도 `rig.depth_xyz/rpy`(base_link 기준, 팬/틸트와 무관; 줄자·CAD, 각도는 `calib/depth.py` 바닥 평면)·`rig.pan_xyz/camera_xyz`(RGB)·`rig.base_z`**, 깊이↔LiDAR 잔차가 8 cm 안인지(`stats["map3d"]["lidar_resid_m"]`), 깊이 stamp 시계(`depth.min_latency_s`). 임계값 `map3d.nav_thr / nav_min_neighbors / blind_grow_cells`(허상 대 놓침)는 실제 OAK 영상으로 조정
9. **ROS2 연결(`config.nav2`, `ros2/config/*`)**: 토픽·프레임 이름(`/scan`, `/odom`, `odom`, `base_link`), Docker 이미지에 `navigation2`·`cartographer_ros`(또는 `slam_toolbox`) 설치 여부, `desired_linear_vel`·`inflation_radius`(40 cm 통로), `xy_goal_tolerance`

## 멘토링(11/6 19:00)에서 확인할 질문
(답이 나온 항목은 취소선. 근거: 서버 API 안내 문서 / 2차 사전교육 PDF)
1. ~~M1/M3/M6 JSON 키·형식, 엔드포인트, 재제출~~ → 문서로 확인(재제출은 통째로 교체). 단 문서가 「JSON 형식 임시」로 표시한 항목이라 리허설(`--rehearsal`)에서 한 번 더 확인
2. ~~섹터 표기~~ → `B-S1~R-S3`, `CM`(NATO 아님). 명령문이 실제로 어떤 표현을 쓰는지는 리허설 `/order` 로 확인(파서는 「제외하고 나머지 상대진영…」 형태를 가정)
3. 명령에 없는 시설명(예: "통신중계소")이 나오면 어떻게 매핑되나
4. M5(거점 진입) 성공 시 경기 종료 여부 / 같은 팀 다른 UGV 처리
5. 지정표적: LED/성공 판정이 **팀 단위인가 UGV 단위인가**(이미 켜진 표적을 다른 팀원이 다시 쏴도 되는가)
6. ~~레이저 abort·5 s 발광·재장전~~ → abort API 없음(일시정지 때만 꺼짐), 5 s 발광·10 s 재발사는 펌웨어, 탄약은 응답(`empty`)으로만 앎. 남은 질문: HP 0 에서 주행이 멈추는가
7. 수광부 위치·유효거리·유효 입사각, 카메라 FOV/지연. ~~팬틸트 서보 피드백~~ → 없음(추정 모델 사용; `hal.gimbal_latency_s/speed` 실측 필요)
8. UAV 영상: 서버가 JPEG 를 1/2/5 s 주기로 준다(검출은 우리 몫 — 아직 미구현), 아군·적 UGV 식별 가능 수준
9. 서브게임: 표적 면 방향(수광창이 로봇을 향하는가), 성공 판정 방식, 시간 측정 시작 신호
10. ROS2: 드라이버가 `/cmd_vel`을 직접 구독하는지(Nav2 출력을 `/cmd_vel_nav2`로 분리했으므로 문제 없어야 함), `/odom`·TF `odom→base_link`·`/scan` 이름, SLAM(Cartographer/Gmapping)이 이미 기동돼 있는지, 팀 코드(Docker 안/밖) 실행 위치와 네트워크(DDS) 설정
11. 경기 중 구조물이 정말 바뀌는지(규정상 "임의 배치"는 경기 시작 전 세팅일 수 있다). 바뀐다면 로봇이 박스를 밀 수 있는 재질인지
12. 깊이 카메라(OAK): 토픽명·인코딩(16UC1 mm?)·해상도·fps·camera_info 제공 여부, 깊이 카메라의 정확한 설치 위치(팬/틸트 아래 차체에 고정이라는 것은 확인됨)·높이·기울기, RGB 와 깊이의 정렬(정류) 여부, LiDAR 평면보다 낮거나 높은 구조물이 경기장에 있는지(없다면 3D 지도의 항법 이득은 없다)

## 알려진 약점
- 움직이는 상대 UGV 명중률 낮음(조준 지연·리드 튜닝은 실기 데이터가 필요).
- M3는 면 단위 관측이라 가려진 면·좁은 통로는 놓칠 수 있다(재현율 ~0.6~0.9).
- 내장 백엔드의 위치추정은 경기장 벽이 LiDAR에 보인다는 가정이다. 박스가 벽을 가리는 구간에서 오차가 커질 수 있다(실기 SLAM 백엔드는 이 가정이 없다).
- Nav2 백엔드의 경기장 좌표는 시작 때 한 번 정한 앵커에 의존한다. SLAM이 크게 틀어지면 경기장 좌표도 따라 틀어진다(주기적 벽 재정합은 미구현).
- ROS2/Nav2는 시뮬 연결(WSL)까지만 검증했다. 첫 현장 리허설에서 `ros2/README.md` 1~6단계부터.
- Nav2 폴백(내장 A*)은 SLAM 위치(TF)가 살아 있을 때만 의미가 있다. SLAM 노드 자체가 죽으면 내장 파티클필터로 자동 전환하지 않는다(`--nav builtin`으로 재시작 필요).
- 서브게임에서 수광창이 로봇 반대편 면이면 현재 코드는 시설을 돌아 보지 않는다(필요 시 FaceTour 재사용).
