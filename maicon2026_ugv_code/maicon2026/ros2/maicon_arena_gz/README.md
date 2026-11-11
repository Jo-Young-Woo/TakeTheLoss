# MAICON 2026 arena — ROS 2 Humble / Gazebo Fortress

바닥 인쇄 기준 **6.00×4.80m**의 경기장이다. 첨부 `arena_spec.yaml`과 기본 배치는 그대로 보존하고, SDF·바닥·사진 아틀라스를 생성한다. 바닥의 선과 글자는 텍스처 한 장이며 별도 도형이 아니다. 새 대회 모델은 `wingbot_description`의 WINGBOT이다. 사용자 수동 실험은 `robot_model:=wingbot ugvs:=1`로 B-2 한 대를 선택한다. `ugvs:=6`은 같은 xacro의 자아 센서 1세트와 센서 없는 더미 다섯 대를 생성하며, 더미도 구동·팬틸트 관절을 유지한다. 실행만으로 경기 시작이나 자율주행을 하지 않는다.

**현재 WINGBOT 검증:** 모델 A–D를 통과해 기본 모델을 WINGBOT, 일반 실행을 B-2 한 대로 전환했다. 실제 사진 실루엣 IoU 0.94750와12개 기준점, C 직진1.499890m·회전2.999473rad를 확인했다.60초 평균 RTF는 solo1.000030,6대 standard0.999714,6대 lite1.000003이다. [모델 보고서](../wingbot_description/README.md)와 [연결 경계](docs/wingbot_integration.md)에 근거·제한을 기록한다. 6대 시험은 `ugvs:=6`, 과거 단순 모델은 `robot_model:=legacy`를 명시한다. 자체 반사 잔여와 기존 알고리즘의 조준·카메라 가정은 별도 감사 대상으로 남으므로 모델 인수를 자율 미션 성공으로 해석하지 않는다.

아래 **1352개 전체 시험·기존 영상·성능표는 WINGBOT 교체 전 단순 모델의 이력**이다. 새 WINGBOT 소스 전체에 그 결과를 적용하지 않는다. 이번 변경은 신규 모델과 legacy 연결의 집중 회귀로 검증하며, 새 전체 시험이나 새 배포 ZIP 통과를 주장하지 않는다. GPU Ogre1 센서 붕괴와 GPU Ogre2 GL4.3 텍스처 오류도 과거 실패 기록으로 보존한다.

## 파일 구성

```text
maicon_arena_gz/
  package.xml, CMakeLists.txt
  config/
    arena_spec.yaml                   # 제공된 정본
    layouts/layout_default.yaml       # 제공된 기본 배치
    profiles/{lite,standard,high}.yaml
    robot_assumptions.yaml             # legacy 단순 모델 전용
    wingbot_engineering.yaml, simulation_assumptions.yaml
    bridge.yaml, fastdds_shm64.xml     # 후자는 명시적 로컬 영상 전달 설정
    uav_grid_calibration.json          # 새 6×4.8m 영상 실측 보정
  media/textures/
    floor_print_2400x1920.png, floor_print_1200x960_lite.png
    faces/, atlases/
  models/{cube_atlas,floor}/meshes/
  worlds/maicon_arena.{sdf,metadata.json,check.json}
  scripts/
    arena_common.py, gen_layout.py, gen_world.py, make_atlas.py
    check_world.py, overlay.py, robot_models.py
    mil_emulator.py, arena_referee.py, wingbot_runtime.py
    lidar_sector_merger.py             # legacy Ogre1 실험 경로
    perf_probe.py, runtime_probe.py
    arena_transport.py, image_ros_probe.py, image_transport_probe.cc
  src/                               # 실제 렌더 재질 호환 플러그인
  launch/arena.launch.py, hooks/
  docs/, validation/
../mil_interfaces_sim/                # 로컬 메시지/서비스 대체 패키지
../wingbot_description/               # WINGBOT xacro·메시·장착 YAML·frames
```

저장소의 `scripts/build_arena_gz.sh`는 세 ROS 패키지를 공백 없는 Linux 홈 경로에 복사해 빌드한다. Windows 원본 경로의 공백은 ROSIDL 오류를 일으켰고, WSL `/tmp`는 세션 종료 뒤 사라질 수 있어 빌드 위치로 쓰지 않는다. 설치·실행 경계는 [runtime.md](docs/runtime.md)를 참고한다.

## 생성·실행·검증 세 명령

저장소 루트에서 실행한다. ROS 설치 복사본을 빌드하고 그 `install/setup.bash`를 이미 읽은 상태여야 한다. 아래 실행은 검증된 CPU 센서/GPU 화면 분리 구성이다. 화면이 필요 없으면 `gui:=false`로 바꾼다. 설치 복사본을 변경한 뒤에는 다시 빌드해야 한다.

```bash
python3 ros2/maicon_arena_gz/scripts/gen_world.py --robot-model wingbot --profile standard --ugvs 1 --ego-slot B-2 --output ros2/maicon_arena_gz/worlds/maicon_arena.sdf
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch maicon_arena_gz arena.launch.py robot_model:=wingbot profile:=standard ugvs:=1 ego_slot:=B-2 gui:=true render_engine:=ogre2 gui_render_engine:=ogre gui_software_rendering:=false image_transport_profile:=shm64
python3 ros2/maicon_arena_gz/scripts/check_world.py --world ros2/maicon_arena_gz/worlds/maicon_arena.sdf --json ros2/maicon_arena_gz/worlds/maicon_arena.check.json
```

실행 명령의 `layout:=/절대경로/layout_seed_2026.yaml`만 바꾸면 해당 배치로 다시 생성한다. 무작위 배치는 `gen_layout.py --output 새디렉터리 --seed 2026 --count 20`으로 만들며 정본을 덮어쓰지 않는다. 모든 배치는 검증 후 저장된다. `high`는 그림자와 0.002초 물리 스텝을 사용하는 촬영용이다. WINGBOT의 `lite`는 바닥·아틀라스·UAV 해상도를 줄이고 가운데 두 바퀴만 구동하며, 나머지 네 바퀴의 외관·충돌·질량·자유 회전은 보존한다. WINGBOT 센서 사양은 모델 YAML에서 읽으므로 legacy 프로필의 360/450/640점이나 5Hz 설정을 그대로 적용하지 않는다.

사용자 조작용 **B-2 한 대**는 위 명령의 `ugvs:=1`로 실행한다. 기존 `validation/arena_single_ugv_ready.png`와 `single_ugv_ready.json`은 **이전 단순 모델**의 실제 GUI에서 B-2만 존재하고 심판이 `ready`인 상태를 확인한 기록이다.8초 동안450점 전부 유효한 LiDAR80회(10Hz), ego영상80장, UAV8장과 odom·IMU를 받았다. WINGBOT 또는 자율주행 검증 근거로 재사용하지 않는다.

상공 영상의 로컬 HTTP 공급자는 저장소의 [gazebo_local_api.md](../../docs/gazebo_local_api.md)를 따른다. 기존 MockServer에 실제 ROS JPEG와 촬영 시각·진영별 파괴 수를 공급하며 별도 주최측 API를 만들어내지 않는다. 임무 문구·목표거점은 운영자 파일에서 명시해야 한다. 소스 릴리스에 영상은 포함하지 않는다.

## 이전 단순 모델의 A–E 인수 기록

측정 장비는 **WSL2 Ubuntu22.04, Mesa D3D12, RTX4060Ti**다. 요구 장비 RTX5070이나 내장 GPU 노트북에서 측정한 결과로 일반화하지 않는다. 단위 검증과 실제 ROS/Gazebo 실행도 구분한다.

| 항목 | 확인한 결과와 근거 | 남은 확인 / 판정 |
| --- | --- | --- |
| A 치수·배치 | `worlds/maicon_arena.check.json`: 오류0·경고0, 좌표 허용오차±1mm, 덩어리/벽 이격≥0.40m 검사 통과. 기본 배치20큐브·9덩어리·6표적·M3면27개. | 제공 YAML에 대한 정합 통과. 실제 경기장 실측은 별도. |
| A 자원 한도 | UGV 제외 collision28/40, visual57/80, texture22/30, directional light1. 바닥 visual1, 기본 그림자 꺼짐. | 기본 배치 통과. 다른 배치는 생성 시 같은 한도 검사. |
| B 실제 영상 | `gpu_gui_verified.png`에서 GPU 화면의 전체 경기장·6대 확인.6m 카메라의27개 독립점: 지정 자세/FOV 직접 투영 RMS5.083mm·최대8.424mm. UAV23개 독립점 최대6.439mm. | 바닥1cm 정합 통과. 직접 투영은 fitted homography로 배치 오차를 흡수하지 않는다. |
| B 근접면 방향 | `validation/closeups/closeup_report.json`: 실제 표적·M3 카메라 각3프레임. 원본 정방향 NCC0.98595/0.98150, 가장 강한 반전·회전 대안0.59467/0.55484. | 시뮬레이션 두 면의 상하·좌우 방향 통과. 모든 면 또는 인식 정확도 검증은 아님. |
| B UAV 객체 | `uav_cube_candidates.png`: 검은 윗면20/20개 후보, 평가용 배치와 중심 오차1.2cm 이내. 탐지 입력은 영상과 보정뿐. | 후보는 형상만이며 M3 종류는 unknown. 실제 데이터셋·그림자·겹침·다층은 미검증. |
| C standard headless | 60.00365초, 평균RTF0.999139·최저1초창0.993836, 첫 스텝2.935초. `performance_standard_headless_shm64.json`. | 이 장비에서 평균≥0.98·최저≥0.90·기동≤10초 통과. RTX5070 실측은 별도. |
| C lite headless | 60.00020초, 평균RTF0.999997·최저1초창0.999611, 첫 스텝2.574초. `performance_lite_headless_shm64.json`. | 이 장비에서 평균≥0.90·기동≤10초 통과. 내장GPU 노트북 실측은 별도. |
| C standard GUI 포함 | 59.996초, 평균RTF0.999059·최저1초창0.994690, 첫 스텝2.596초. `performance_standard_hybrid_shm64.json`. | headless와 별개로 보존한 화면 포함 측정. |
| C lite GUI 포함 | 60.004초, 평균RTF0.999998·최저0.999620, 첫 스텝2.635초. `performance_lite_hybrid_shm64.json`. | headless와 별개로 보존한 화면 포함 측정. |
| C GPU GUI 렌더 루프 | standard 평균137.23Hz·최저1초96Hz, lite143.4Hz·121Hz. GUI `PreRender` 이벤트를 측정. | 렌더 루프는30Hz 초과. 디스플레이 present FPS는 측정하지 않았으므로 실제 표시≥30FPS 합격으로 대체하지 않는다. |
| D 실제 제어·HTTP | 6대 출발·센서,3개 주행 지점, 실제 짐벌 피드백, 발사/재장전/탄약7발, watchdog 정지 후0m 이동. 로컬API `/me`·JPEG200 및6헤더 확인. | odom/MIL 제어 통합 통과. 레이저 표적 명중·자율 M3/Nav2 전체 경기 성공은 별도. |
| D 여섯 거점 빔 | `validation/base_beams_live.json`: 실제 `set_pose`→Gazebo 자세→ROS 심판 이벤트에서 B-1~R-3 모두 차단/해제 확인. | 6/6 통과. ready 단계에서 시험했고 점령 이벤트0, 원래 위치 복원. 경기 중 M5 점령·자율 진입 시험과 구별. |
| D 판정·랜덤 배치 | 순수 심판의2초 연속 명중·10초 무적/잠금·M5 빔 테스트와2026…2045의20시드 배치 검증. | 단위 검증이 물리 센서/ROS 전 과정 검증을 대체하지 않음. |
| E 전체·ZIP 회귀 | ZIP v2 독립 추출본의 ROS 환경 전체 **1352통과·실패0·skip0**,1065.26초. fixture의0나눗셈 경고1개. pyflakes0, API 추적97/97개 S-ID·159개 spec 마커. | 후보 ZIP611개 멤버 해시 및 시험 후 불변 확인. ZIP에서 새 colcon 빌드:2패키지22.7초. 코드·시험·에셋은 검증본과 해시 대조하며 최종 변경은 문서·검증 근거에 한정. |

성능 원본은 `validation/performance_*`에 장비·렌더러·샘플 길이와 함께 저장한다. CPU 실험의 낮은 최저값과 GPU 실험의 화면 실패를 지우지 않는다. `native_images_shm64.json`/`ros_images_shm64.json`의20초 비교에서 Gazebo 원본과 ROS 모두 ego199프레임(촬영 시각10Hz), UAV20프레임(1Hz),1500×1200 검증 카메라20프레임(1Hz)을 받았고 최대 간격도 각각0.1초·1초였다. 기본 전달 설정에서 있던 큰 영상 누락이 `shm64` 설정에서는 재현되지 않았다. 측정용6m 카메라는 기본 실행에서 꺼져 있다.

후속 전체 실행의 **23실패·1263통과**는 수정 전 이력이다. 현장 fixture 관련31개, ROS 미로드 비교202개 및 환경 격리 후 ROS 검사155개는 선택 결과이며 전체 통과 수로 합산하지 않는다. ZIP 후보 v1 전체 검사는 회피 회귀1개 실패·196개 통과(106.46초)에서 멈췄다. 예정된 적 UGV가 등장 전에 관측·충돌 세계에 존재하던 결함을 수정한 뒤 **v2 전체1352개가 통과**했다. 기존 실패 기록을 보존하며 상세 이력은 [인수 기록](../../docs/ARENA_ACCEPTANCE.md), 최종 배포물·해시 근거는 [RELEASE_VALIDATION.json](../../docs/RELEASE_VALIDATION.json)을 따른다. 전체 시험 통과는 실제 사진 인식 정확도·자율 M3/Nav2 경기 성공이나 실물 검증을 의미하지 않는다.

앞선 GUI 포함 성능은 **5초 워밍업 후60초**,6대(ego 센서1세트+더미5)와1Hz UAV, GUI 켜짐, 검증 카메라 꺼짐 조건이다. 동시에 별도 저장소 pytest 프로세스1개가 실행 중이었고 두 번째 시뮬레이터는 없었다. 이전 SHM 설정 전의 낮은 RTF와 실험용 GPU 센서 결과는 최종 성능으로 섞지 않는다.

별도 headless 측정은 같은6대·ego 센서·1Hz UAV에서 GUI를 끄고 **5초 워밍업 후60초** 수행했다. 두 프로필 모두 정상 구간 UAV60장을 받았다. 동시70초 헤더 검사(기동·워밍업 포함)는 standard ego RGB/BGR659/659장·UAV66장, lite335/334장·UAV67장이다. 최대 시각 간격은 standard ego0.1초, lite0.2초, UAV1초이며 중복·역행0이다. 이때 전체/개별 pytest 부하는 측정 도중 변동했고 정확한 겹침 시간은 계측하지 않았다. 다른 Gazebo는 없었다. CPU 센서 렌더링 결과이며 설치된 RTX4060Ti 또는 요청된 다른 GPU의 headless 가속 성능으로 일반화하지 않는다.

실제 제어 결과는 `recordings/control_20261109_v2/control_trial.json`, 영상은 같은 폴더의 `b2_control_third_person.mp4`(17.2초)·`b2_control_fpv.mp4`(17.5초)다. 전 구간 디코딩을 확인했다.1인칭은 원본10Hz175프레임이며,3인칭은 원본1Hz18장을5fps 영상으로 유지 표시한86프레임(반복68개)이다. 촬영 시각은 frames.jsonl에 보존하며 임의 움직임을 합성하지 않았다. 이 짧은 시험은 자율 M3/Nav2 채점 경기 영상이 아니며 소스 ZIP에는 영상을 넣지 않는다.

기본 `image_transport_profile:=inherit`는 사용자의 Fast DDS XML/RMW를 보존한다. 위 실행 명령은 검증된 `shm64`를 명시적으로 선택한다. 별도 클라이언트 셸에서도 `export FASTRTPS_DEFAULT_PROFILES_FILE="$(ros2 pkg prefix --share maicon_arena_gz)/config/fastdds_shm64.xml"`, `export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"`, `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`를 지정하고 다시 시작해야 같은 설정을 쓴다. OS 전역 설정은 바뀌지 않는다.

상세 근거는 [렌더링·UAV 검증](docs/rendering_and_uav_validation.md), 저장소의 [인수 범위](../../docs/ARENA_ACCEPTANCE.md)에 기록한다. API G1–G15 구현·제한·검증은 [변경 표](../../docs/STRATEGY_DELTAS.md),97개 규정의 코드·테스트 연결은 [추적 표](../../docs/API_TRACEABILITY.md)에 있다. 실전 재시작 시각 복구는 구현했으나 지도·위치 앵커 없는 중간 주행 재시작은 차단한다.

## YAML의 가정과 현장 교체 항목

| 파일 / 키 | 현재 가정 |
| --- | --- |
| `arena_spec.yaml`의 `walls.inner_size`, 벽 두께·모서리 센서 박스 | 바닥6×4.8m, 벽0.06m, 모서리0.4m 정육면체. 정본 수치는 현장 실측본과 비교 후 교체. |
| `bases.photoelectric_beam`, `label_map`, `start_pose` | 빔 위치/높이, 이름↔줄, 시작 중심 위치. |
| `layout_*.yaml`의 배치·면·`stack` | 기본 렌더 추정 배치와 임의 면.1단 기본, 표적 같은 종류 대칭은 가정. 임시 PDF 사진은 실제 인식 성능 근거가 아님. |
| 프로필의 `uav_camera.mount_yaw_offset`, `optical_hfov` | +π/2,130°. 원본 자세/hfov는 보존하며0.4m 높이 외곽까지 화면에 담기 위한 광학 가정. |
| `wingbot_description/config/wingbot_mods.yaml` | WINGBOT의 장착 위치·치수·색·질량. 사진 복원 값이며 실물 실측과 구분. 생성한 frames가 TF/심판의 유일한 기하 입력. |
| WINGBOT `lidar_scan_link` | 원본 장착 링크 z=.11 + 제조사 SDF 센서 오프셋 .04 = 스캔 평면 .15m. 현장 광학 평면은 미확인. |
| `robot_assumptions.yaml` | **legacy 전용** 단순 형상·왜곡0·LiDAR .22m 가정. WINGBOT은 읽지 않음. |
| 프로필의 `ugv.lidar_samples`, 센서 빈도 | **legacy 전용** lite360/standard450/high640점, lite5Hz·표준10Hz. WINGBOT RGB/LiDAR/IMU는 모델 YAML 사양. |
| `simulation_assumptions.yaml` | 심판20Hz, 신선도/접촉 시간 간격, 광선12m, 교차 허용오차1mm. 거점 체류0초는 최초 빔 차단 규칙을 반영. |
| `wingbot_engineering.yaml`의 emulator | 배터리 고정, 감시·상태 빈도. 메시지 패키지/정수폭은 로컬 대체이며 서보 내부 가속도는 미모델링. |
| 프로필의 scene/physics/geometry/validation_camera | 조명·태양 방향·중력·물리 스텝·보조 도형 치수·촬영 설정. |

## 11월18일 현장 측정 체크리스트

- [ ] 실제 벽 내부 너비·깊이와 바닥 인쇄 원점·20cm 격자 오차
- [ ] 모서리 센서 박스의 폭·깊이·높이, 중앙 패드의 실제 용도
- [ ] 여섯 빔의 양 끝 좌표·높이·차단 판정과 거점 점령 시간
- [ ] 거점 이름↔줄 번호, 여섯 시작 위치와 방향
- [ ] 박스 위치·이격·적층 높이, 표적 수광부와 LED 위치
- [ ] `camera=main` 실제 자세·화각·왜곡·해상도·촬영 시각; 실제 영상 격자 다점 보정 후 독립점 검사
- [ ] UGV 길이/폭 방향·바퀴 간격·카메라/짐벌/레이저/수광 돔의 위치와 광축
- [ ] ego 카메라 화각·왜곡·실제사진 인식, LiDAR 토픽·좌표틀·범위·회전점수·자기반사
- [ ] 실기 메시지 패키지·필드형·QoS·정지/잠금·주행/odom 부호·시간 동기화
- [ ] RTX5070와 노트북에서60초 RTF·GUI FPS·기동시간·영상 전달률 재측정

원본 자료와 사진을 외부에 공개하거나 Fuel/인터넷에서 에셋을 받지 않는다. 릴리스는 소스·로컬 에셋·측정 JSON/PNG를 포함하고 비밀값 파일, 빌드·설치 산출물, 영상·rosbag을 제외한다. 영상은 별도 보관물이다.
