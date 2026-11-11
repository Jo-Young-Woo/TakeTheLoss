# WINGBOT 모델 연결 상태

2026-11-09: 기본 `robot_model:=wingbot`은 `wingbot_description`의 같은 xacro를
자아 1대(`sensors:=full`)와 더미 5대(`sensors:=none`)에 사용한다. 좌표를 경기장
생성기에 다시 적지 않는다. 기존 단순 모델은 `robot_model:=legacy`를 명시해야 한다.
실제 모델 A–D 인수 후 기본값을 전환했다. 일반 실행은 사용자 수동 실험용 B-2 한 대이며,
자아 1대와 더미 5대는 `ugvs:=6`을 지정한다. 실행 자체가 경기를 시작하지 않는다.

## 좌표와 장치 판정

- 장착값 원본은 `wingbot_description/config/wingbot_mods.yaml`이다. 생성된
  `config/frames.yaml`의 원본 SHA256을 확인하고, 오래된 메타데이터는 실행을 거부한다.
- MIL 에뮬레이터와 심판은 모델 패키지의 같은 순기구학 함수와 실제 관절 피드백을 쓴다.
  팬 API 양수는 양의 관절각이며, 원본 관절의 −z축 때문에 오른쪽으로 회전한다.
  틸트 API 양수는 위쪽이다. 명령값만으로 조준 완료나 발사 준비를 가정하지 않는다.
- 레이저 광선은 `laser_link`의 원점과 +x축이다. 카메라 광학 중심을 발광점으로 쓰지
  않는다. 영점에서 레이저−카메라 차이는 원본의 (−0.009, 0, +0.0465)m이다.
- 수광부는 원본 좌우 중심·반지름과 바깥/위쪽 방향을 읽는다. 노출된 반구의 앞면과
  교차해야 하며, 다른 UGV의 실제 차체·덱·머리 충돌 상자 뒤에 가려지면 수광하지 않는다.
- `robot_assumptions.yaml`은 legacy 모델 전용이다. WINGBOT 경로의 별도
  `wingbot_engineering.yaml`에는 감시 주기와 상태 유효시간만 있고 장착 좌표는 없다.
- 기존 임무·탐색·주행·영상 인식·게임서버 코드는 이 연결 작업에서 수정하지 않았다.
  실제 46.5mm 광축 높이 차이로 드러나는 기존 조준 알고리즘의 오차는 이후 시험에서
  시드·시각과 함께 보고할 항목이다.

## LiDAR 각도 감사

Waveshare 원본 `src/ugv_else/ldlidar/src/demo.cpp`는 원시 `point.angle`의
225–315도를 먼저 NaN 처리한다. `launch/stl27l.launch.py`의
`laser_scan_dir=True` 설정 때문에 이후 ROS 배열 인덱스가 반전된다. 따라서 동일한
가림은 ROS `base_lidar_link`의 **45–135도**에 해당한다. 보존한 description 관절의
yaw +1.5708rad를 더하면 차체의 뒤쪽 약 135–225도다. ROS 각도에 225–315도를
직접 적용하면 앞쪽을 가리는 오류가 생긴다.

모델은 360도 원본 스캔을 `/arena/ego/scan_raw`에 내보내고, 에뮬레이터만 검증된
각도 규약을 적용하여 `/scan`을 발행한다. 원본 장착 링크와 방향이 같은 별도
`lidar_scan_link`를 실제 스캔의 frame_id로 쓴다. 가린 점은 NaN이며 빈 공간으로 해석하지
않는다. 타임스탬프·각도 간격·유효한 원본 거리는 보존한다. 다른 frame_id 또는 감사
표시가 없는 규약은 거부한다. 원본 관절 z를 10mm 낮춘 좌표를 그대로 쓰며, 가림을
피하려고 예전 단순 모델처럼 라이다를 0.22m로 올리지 않는다. 원본 Gazebo SDF에
명시된 센서 자체의 +0.04m 위치를 별도 고정 프레임으로 보존하므로, 장착 링크는
z=0.11m이고 스캔 평면은 z=0.15m다. 이 값은 제조사 시뮬레이션의 광학 평면이며,
사진 요약의 약 0.13m와 다른 점은 `scan_plane_physical_verified: false`로 남긴다.

이 감사는 **원본 코드와 모델 좌표의 일치**를 확인한 것이다. 실물 D500의 설치 yaw와
가림 방향은 아직 현장 측정 전이며 `physical_yaw_verified: false`로 남긴다.

## 실행과 검증 범위

모델 조립 검사: `gen_world.py --robot-model wingbot --ugvs 6`. 런치에서는
`robot_model:=wingbot render_engine:=ogre2`를 지정한다. 이미 확인된 Ogre1 360도
LiDAR 문제를 숨기지 않기 위해 WINGBOT 센서 서버의 Ogre1 선택은 거부한다.
GUI는 별도로 `gui_render_engine:=ogre`를 사용할 수 있다.

집중 검증 49개 통과: `test_wingbot_arena_integration.py`,
`test_arena_robot_runtime.py`, `test_arena_package_referee.py`,
`test_arena_lidar_sectors.py` (2.14초). 이 결과는 좌표·API·판정 연결 검증이며,
실제 Gazebo 사진·주행·성능 또는 완주 결과를 대신하지 않는다.

## 실제 모델 A–D 인수

| 항목 | 실측 결과 | 원본 |
| --- | --- | --- |
| A | 외곽·질량·6륜·프레임·관절 방향·기본도형 충돌 검사 통과 | [gate_a.json](../../wingbot_description/validation/gate_a.json) |
| B | 차체 실루엣 IoU0.947498,12개 기준점 허용치 통과, 남은 녹색 디버그 픽셀0 | [photo_validation.json](../../wingbot_description/docs/photo_validation.json) |
| C | 직진1.499890m, 회전2.999473rad, 양/음 속도 제한 확인 | [gate_c.json](../../wingbot_description/validation/drive_v4/gate_c.json) |
| D solo full | 약60초 평균 RTF1.000030 | [gate_d_solo.json](../../wingbot_description/validation/perf_solo_final/gate_d_solo.json) |
| D 6대 standard | 약60초 평균 RTF0.999714, 센서 full1대+none5대 및 UAV | [gate_d_arena.json](../../wingbot_description/validation/perf_arena_standard/gate_d_arena.json) |
| D 6대 lite | 약60초 평균 RTF1.000003, 센서 full1대+none5대 및 UAV | [gate_d_arena.json](../../wingbot_description/validation/perf_arena_lite/gate_d_arena.json) |

D는 이 호스트의 CPU Ogre2·화면 없는 측정이다. 실제 화면 표시 FPS나 다른 GPU의
성능을 보장하지 않는다. C 회전 중 수cm의 중심 이동은 모델 보고서에 별도 남겼다.
기존 1352개 전체 회귀는 새 모델 전의 이력이며, 위 인수는 자율 M3/M5 경기 완주
또는 현재 전체 소스 회귀 통과를 뜻하지 않는다.

## 모델 기하에 근거한 자체 반사 보정

기존 6대 측정에서 제조사 crop 뒤에도 남던 최솟값 약0.031942m는 마스트 기둥의
자기반사였다. [보정 전 실제 스캔](../../wingbot_description/validation/default_smoke/smoke.json)의
276–280번·393–397번 ray는 차체각 약131.29–134.50도·225.10–228.31도,
거리0.03194–0.03431m로, 제조사 뒤쪽90도 crop 경계 바깥의 기둥 표면과 일치한다.

생성기는 원본 `pan_mast.pillars`의 네 원통과, 모든 고정 visual 삼각형이 실제
스캔 평면과 만나는 단면 선분을 `frames.yaml`에 내보낸다. GPU LiDAR가 보는
시각 도형을 사용하며, 단순화된 물리 충돌체로 대체하지 않는다. 관절로 움직이는
머리는 영점에 고정된 단면으로 가정하지 않는다.
에뮬레이터는 실제 스캔 프레임의 위치·yaw·높이와 원통·단면의 교차를 계산하고,
GPU 각도 양자화를 고려해 반 bin 안의9개 방향과 표면 거리 ±1mm에 맞는
유한 측정만 NaN으로 바꾼다. 고정 각도 범위를 넓히거나 모든 근거리 반사를
지우는 방식이 아니다. 동일 방향의 더 가까운 장애물·먼 벽은 유지하며,
가려진 ray를 빈 공간을 뜻하는 +inf로 바꾸지 않는다. 입력 스캔의 frame_id,
타임스탬프와 각도는 보존하고, 차체 좌표로의 변환은 기존 HAL에서 수행한다.

[보정 후 실제 Gazebo 검사](../../wingbot_description/validation/sensor_calibration/smoke.json)에서는
raw 스캔의0.08m 미만 측정84개, 기존 crop112개에 추가 자기반사10개를 합쳐
NaN122개가 되었고, 출력의0.08m 미만 잔여 측정은0개였다. 이는 그 장면의
시뮬레이션 검증이다. 실물 D500의 yaw·실제 스캔 평면은 여전히 현장 측정 전이며
`physical_yaw_verified: false`, `scan_plane_physical_verified: false`를 유지한다.
모델 장착값·원본225–315도 crop·임무/주행/인식 정책은 바꾸지 않았다.

`wingbot_recording` 프로필은 표준 물리·센서를 유지하면서 관찰용 카메라만
경기장 남쪽 경사 시점의1280×720·10Hz로 설정한다. 실제 검사에서 같은 크기의
RGB 영상 수신을 확인했다. 이 관찰 영상은 Agent 입력에 연결하지 않으며,
영상 수신 성공은 자율 임무 완주나 녹화 중 실시간 성능 검증을 대신하지 않는다.

후속 회전 장면에서는0.08m 검사만으로 찾지 못했던 안테나 반사9개도 확인했다.
[보존된 실제 raw 측정](../../wingbot_description/validation/sensor_calibration/live_near_returns.json)의
269–272번·401–405번은 약0.129–0.131m로, 생성된 안테나 막대의 단면과 일치한다.
이 측정의 재생 검사에서 마스트10개와 안테나9개만 추가 제거하고, 같은 장면의
약0.15–0.25m 외부 장애물 측정은 원값을 보존했다. 이 단면 확장의 실제 실행
재검사는 별도이며,0.08m 미만 잔여0개라는 앞선 결과만으로 모든 자체 반사가
없다고 결론 내리지 않는다.

집중 회귀: ROS Humble 환경에서 `test_wingbot_self_returns.py`,
`test_arena_robot_runtime.py`, `test_wingbot_arena_integration.py`,
`test_wingbot_model_audit.py` 합계66개 통과·건너뜀0개(7.47초).
READY에서 영점 정지 목표 유지,
실제 잠금 전환 때의 관절각을 한 번만 저장하는 동작도 포함한다. 잠금 중
중력 처짐 피드백이 정지 목표를 매 주기 낮추는 오류를 재현·차단한다.

### 실제 센서로 실행하는 단일 B-2 자율 시험

`scripts/launch_wingbot_autonomous_trial.py --output recordings/새_시험_이름`을
Humble 및 최신 `wingbot_description`/`maicon_arena_gz` 설치 환경에서 실행한다.
서로 다른 시험은 새 폴더를 사용하며, 완료 후 해당 시험의 Gazebo 프로세스만 종료한다.
`--no-uav`는 UAV 없이 같은 ROS 센서 경로를 실행한다.

기존 Agent와 내장 LiDAR/오도메트리 SLAM·A*를 사용한다. Nav2 시험은 아니다.
게임 API는 로컬 루프백 fixture이며 M1/M6에는 외부 LLM을 연결하지 않는다.
지상 검출기는 제공된 그림을 실제 RGB 영상과 SIFT/Homography로 비교한다.
그림의 배치, 모델 위치, 합성 Detection을 검출기에 전달하지 않는다.
일반 실물 영상에 대한 학습 검출기의 성능을 증명하는 시험은 아니다.

카메라는 실제 CameraInfo K, 촬영 시각 TF 및 관절 피드백을 검사한다.
Fortress Ogre2가 적용하지 않는 SDF 왜곡 계수는 출처로 보존하고 영상에 다시
적용하지 않는다. 유효 화각은 2.0rad, 렌즈 영점 높이는 0.2035m,
카메라 기준 레이저 오프셋은 [-0.009, 0, 0.0465]m이다.
물리 카메라의 실측 보정은 아니며, 기존 2D 객체 지도는 차체 중심에서
렌즈까지 57mm 레버암을 근사하는 잔차가 남아 있다.

산출물:

- `sensor_readiness.json`: 시작 전 raw/보정 스캔, 실제 카메라 K·TF·관절각
- `b2_autonomous_third_person.mp4`: 실제 Gazebo 관찰 카메라 영상
- `*.frames.jsonl`: 영상과 시뮬레이션 시각, 누락/보간 여부
- `agent_trace.jsonl`, `mission_report.json`: 자율 명령과 실제 임무 결과
- `evaluation_truth.jsonl`: 평가 전용 위치. Agent 입력에는 연결하지 않는다.

`scripts/analyze_wingbot_trial.py <trial 폴더>`는 같은 기록 시각끼리 위치를
비교한다. Pose_V 브리지의 수신 시각을 사용하므로 하드웨어 동기화 정확도
검증으로 해석하지 않는다. 영상 검사는
`scripts/verify_gazebo_recording.py <mp4>`로 전 프레임을 디코드한다.
API 접수 성공, 영상 저장 성공, M3 수량 정답 및 실제 거점 점령은 각각 따로 평가한다.

최종 보정의 실제 READY 검사(run02): 같은 시각 스캔에서 raw의 0.15m 미만
93개가 보정 후 0개가 되었고, 기존 crop 112개 + 기둥 10개 + 안테나 9개로
NaN 131개를 확인했다. TF 관절각은 팬 약 0°, 틸트 약 -0.68°로 유지되었으며
READY에서 -15° 하한까지 처지던 목표 추종 오류가 재발하지 않았다.

run02의 최종 영상은 304.8초/3048프레임으로 전부 디코드됐고 누락은 없었다.
실제 위치와 같은 시각으로 대조했을 때 약 190초의 회전 정체 이후 추정이
이탈했다. M3 제출 6개/채점 대상 21개, M2 타격·M5 점령 확인 없음으로
임무 전체 성공에는 미달했다. 상세 결과는 저장된
`recordings/wingbot_b2_20261109_run02/RESULTS.md`에 분리해 기록했다.
