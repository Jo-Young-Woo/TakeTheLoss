# 로봇·ROS 실행 경계

`arena.launch.py`는 YAML에서 월드를 매번 생성하고 Fortress 서버만 실행한다. `gui:=true`일 때 GUI를 별도 프로세스로 연다. 모든 에셋은 패키지 내부에 있으며 Fuel을 요청하지 않는다.

모델 A–D 실측 인수 후 기본값은 `robot_model:=wingbot`, B-2 한 대(`ugvs:=1`)다. 일반 수동 사용은 이 구성을 쓰고 6대 성능/경기 구성은 `ugvs:=6`을 명시한다. 이전 단순 모델은 `robot_model:=legacy`로만 선택한다. [WINGBOT 연결 설명](wingbot_integration.md)과 [모델 자체 검증](../../wingbot_description/README.md)을 따른다. 모델 인수와 자율 미션 알고리즘 검증은 구별하며, 잔여 LiDAR 자체 반사와 카메라/조준 보정은 후속 확인 대상이다.

ROSIDL은 공백이 포함된 원본 경로에서 빌드하면 의존 경로를 잘못 나눌 수 있다.
정본은 그대로 두고 **공백 없는 Linux 경로**로 `mil_interfaces_sim`, `wingbot_description`, `maicon_arena_gz` 세 패키지를 복사해 빌드한다.
저장소 루트의 아래 도우미는 기존 대상 패키지가 있으면 중단하며 파일을 지우지 않는다.

```bash
ARENA_BUILD_WS="$HOME/maicon_arena_ws_new" bash scripts/build_arena_gz.sh
source "$HOME/maicon_arena_ws_new/install/setup.bash"
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch maicon_arena_gz arena.launch.py robot_model:=wingbot profile:=standard ugvs:=1 ego_slot:=B-2 gui:=true render_engine:=ogre2 gui_render_engine:=ogre gui_software_rendering:=false image_transport_profile:=shm64
```

`layout:=/절대경로/layout.yaml`, `profile:=lite`, `ugvs:=0|1|6`, `uav_cam:=false`, `validation_camera:=true`를 선택할 수 있다. WINGBOT은 현재 Fortress와 Ogre2 센서 서버를 지원한다. Harmonic 및 Ogre1 WINGBOT 센서 서버 선택은 명시적으로 거부한다. 과거 단순 모델은 `robot_model:=legacy`이며 그 Ogre1 분할 스캔 경로는 별도 실험이다.
GUI의 `gui_render_engine`은 비워 두면 서버와 같고 `gui_software_rendering`은 비워 두면 환경을 상속한다. `true/false`를 지정하면 GUI 프로세스에만 CPU/GPU 설정을 적용한다.
선택한 렌더 구성은 **CPU Ogre2 센서 서버 + GPU Ogre1 GUI + SHM64**다. 위 명령은 센서에 `LIBGL_ALWAYS_SOFTWARE=1`을 적용하고 GUI에만0을 적용한다. `MaiconOpaqueMaterialGui`가 GUI의 실제 렌더 스레드에서 사진 재질을 보정한다. 기존 `validation/gpu_gui_verified.png`는 이전 단순 모델의 전체 경기장·6대 기록이며 WINGBOT 인수 사진과 구별한다. 화면이 필요 없으면 `gui:=false`로 실행한다. GPU Ogre1을 센서 서버로 쓴 과거 실험은 LiDAR 전체가 최솟값으로 붕괴했으므로 WINGBOT 주행 센서 구성에 쓰지 않는다.
`wsl_gpu_compat:=true`는 **실험용**이다. Gazebo 서버·GUI 프로세스에만 `LIBGL_ALWAYS_SOFTWARE=0`, `MESA_GL_VERSION_OVERRIDE=4.3`, `MESA_GLSL_VERSION_OVERRIDE=430`을 적용하고 셸 설정은 바꾸지 않는다. 이 WSL2/Mesa D3D12/RTX4060Ti에서 Ogre2 기동과450점 LiDAR는 통과했지만 텍스처 메시가 사라지는 화면 오류가 남아 있다. GL 버전 지정은 드라이버 기능이나 영상 정확성을 보장하지 않는다. GUI별 CPU 설정은 이 옵션보다 우선한다.
빌드 복사본은 Linux 홈 아래에 보존한다. 이 WSL 환경은 마지막 프로세스가 끝나면 /tmp가 사라질 수 있으므로 빌드·설치 경로에 /tmp를 쓰지 않는다.
`base_map`은 `row1/row2/row3` 키가 있는 YAML 경로이며 원본 사양을 덮어쓰지 않고 생성용 사본에 적용한다.

WINGBOT ego 한 대에는 실제 Gazebo 카메라·LiDAR·IMU가 있다. `ugvs:=6`의 나머지 다섯 대는 **같은 xacro의 sensors=none**이며, 구동·팬틸트 관절과 관절 상태 발행을 유지한다. 모든 모델은 차체2·머리1 box와6개 바퀴 cylinder, 총9개 기본도형 충돌체를 사용한다. 표준은6륜 구동이고 lite는 가운데2륜만 구동하되 나머지 바퀴도 외관·충돌·질량을 보존한 자유 회전 관절이다. 전체 질량은2.9kg이다.
카메라·레이저·돔·안테나 장착값은 모델의 `wingbot_mods.yaml`에서 생성한다. 에뮬레이터와 심판은 같은 `frames.yaml`/순기구학과 실제 팬틸트 피드백을 읽으며, `robot_assumptions.yaml`은 legacy 경로에서만 사용한다. 레이저와 카메라 중심은 서로 다르고, 수광 돔은 측면 중앙의 바깥쪽 반구다.
상공 카메라의 `mount_yaw_offset=+π/2`는 화면 오른쪽을 +X, 위쪽을 +Y로 맞추는 가정상의 광학 장착 회전이다. 첨부 원본 사양의 camera.pose는 변경하지 않는다.
`uav_camera.optical_hfov=130°`도 프로필의 광학 가정이다. 원본123.2°에서는 바닥 평면은 들어오지만 높이0.4m의 벽·모서리 박스 상단이 잘렸다. 높이2.5m와16:9 영상에서 상단까지 보이는 최소 화각은 약129°이므로130°로 여유를 둔다. 정본 사양의 hfov2.15는 보존한다.
WINGBOT LiDAR의 원본 장착 링크를10mm 낮춰 z=.11m로 보존하고, 제조사 SDF 센서 오프셋 .04m를 별도 `lidar_scan_link`로 적용한다. 스캔 평면 .15m는 제조사 시뮬레이션 값이며 실물 측정값이 아니다. 모델은450점·10Hz 원본360도 스캔을 생성하고 에뮬레이터만 제조사 clockwise crop225–315°를 ROS45–135°로 변환하여 NaN 처리한다. URDF yaw +π/2에 따라 차체 뒤쪽을 가린다. 예전의 라이다 .22m 올리기는 legacy 단순 모델의 이력으로만 남는다.

| 외부 ROS 경로 | 공급/수신 경계 |
| --- | --- |
| /cmd_vel | emulator가 속도 한계·0.5초 timeout·경기/피격/심판 신선도 잠금 후 /arena/ego/cmd_vel로 전달 |
| /odom | 실제 DiffDrive 바퀴 odometry, 출발점 기준; 심판 world pose를 제어용 odometry로 사용하지 않음 |
| /image_raw | ros_gz_image의 실제 ego 영상, 행 padding을 보존하며 rgb8→bgr8 변환 |
| /scan | 실제 Gazebo /arena/ego/scan_raw를 에뮬레이터에서 검증된 프레임·각도 규약으로 crop; 타임스탬프 보존 |
| /imu/data | 실제 Gazebo IMU 센서 브리지 |
| /mil/state, /mil/match | 심판 상태에서 변환, depth1 TRANSIENT_LOCAL; 상태2Hz |
| /mil/gimbal | pan 오른쪽+, tilt 위쪽+ 단위 변환·속도 제한; 실제 물리 joint feedback을 TF/심판으로 전달 |
| /mil/fire | 최근 심판 상태·실제 짐벌 피드백을 확인해 요청; 탄약과 피격 판정은 심판이 소유 |
| /mil/oled | 로그 출력만 |
| /ugv_B1/cmd_vel 등 | dummy에 대한 별도 시뮬 제어; ego의 실제 API 경로와 다름 |
| /uav/image_raw | 상공 카메라 실제 영상, 심판 정답이나 M3 종류가 포함되지 않음 |

`/arena/control`, `/arena/fire`, `/arena/gimbal`, `/arena/state`, `/arena/events`는 로컬 시뮬 연결용 JSON 토픽이다. 주최측 API에 이런 경로가 있다고 가정하지 않는다.
심판 시작은 명시적인 `/arena/control`의 `{"cmd":"start"}` 메시지로 한다. 실행만으로 자율주행하거나 경기를 시작하지 않는다.
탄약을 emulator에서 별도로 소비하지 않는다. 서비스의 fired는 최신 심판 상태에 따른 요청 접수이며 실제 발광·탄 소모 결과는 심판 이벤트/상태로 확인한다.

`mil_interfaces_sim`은 문서 필드 이름을 갖는 로컬 대체 메시지 패키지이다. 문서에 없는 정수 폭·패키지 이름은 시뮬 가정이다.
주최측 실제 메시지가 있으면 `interface_package:=실제패키지`를 쓰되 State/Match/OledLine/Gimbal/Fire 클래스 및 필드 호환성을 먼저 확인해야 한다.
기존 RosMilTransport는 실행 중 타입을 찾아 연결하므로 대체 패키지를 위해 team_node 코드를 고치지 않는다.

### 이전 단순 모델의 검증 이력

이 절의 센서 수치·성능·영상·1352개 전체 시험은 **WINGBOT 교체 전 legacy 모델**의 기록이다. 새 모델의 현재 전체 시험 또는 새 배포 ZIP 검증 결과가 아니다. 새 모델은 해당 모델 A–D와 집중 통합 회귀를 별도로 수행한다.

legacy Ogre2는 단일360° 스캔을 썼다. legacy Ogre1은 같은 위치의120° 센서3개를 측정값만으로 합쳐 `/scan`을 공급했다. 표준은151점×3→450점, lite121점×3→360점, high215점×3→640점이다. 3개 실제 시각의 지연·편차·좌표틀·각도·측정 범위를 검증하고, 미관측은NaN으로 남긴다. 보간이나 경기장 정답으로 채우지 않는다. 범위 최솟값으로 붕괴한 센서는 진단에 기록하고 스캔 발행을 막는다. 이 WSL의 GPU Ogre1은 분할 방식도 붕괴했으며, merger 통과 테스트가 해당 드라이버의 정상 작동을 뜻하지 않는다.

실제 설치·실행에서6대 출발 자세, `/clock`, odom, IMU, 영상2개, 심판 상태 및 CPU Ogre2 LiDAR를 수신했다. `validation/runtime_gpu_gl43.json`은7.0초 동안450/450점 유효,70회 스캔(약10Hz), 최솟값0.305m를 기록한다. 단, 그 구성의 영상은 위의 별도 실패 항목이다. 새 센서·모델·정적 생성 회귀43개가 통과했고 Python 정적 검사 경고0이다.
요구 C의 headless 측정 `performance_standard_headless_shm64.json`은60.00365초 평균RTF0.999139·최저1초창0.993836·기동2.935초이고, lite는60.00020초·0.999997·0.999611·2.574초다. CPU Ogre2+SHM64, GUI 꺼짐,6대+ego 센서+1Hz UAV, 검증 카메라 제외,5초 워밍업 뒤60초 조건이며 두 프로필 모두 해당 수치 기준과 기동≤10초를 통과했다. 정상 구간 UAV는 각60장이다. 별도70초 영상 헤더 감사(기동/워밍업 포함)는 standard ego RGB/BGR659/659장·UAV66장, lite335/334장·UAV67장; 최대 시각 간격0.1/0.2초·UAV1초, 중복/역행0이다. 전체/개별 pytest 부하가 측정 도중 변동했으며 정확한 겹침 시간은 계측하지 않았다. 다른 Gazebo는 없었다. 이 결과는 현재 호스트의 CPU 센서 렌더링 결과이며 RTX5070·내장GPU 장비 실측이 아니다.
별도 GUI 포함 `performance_standard_hybrid_shm64.json`은 평균RTF0.999059·최저1초창0.994690·기동2.596초이고, lite는0.999998·0.999620·2.635초다. 두 프로필 모두5초 워밍업 뒤60초,6대+ego 센서+UAV+GUI, 검증 카메라 꺼짐 조건이다. 별도 pytest 프로세스1개가 동시에 실행 중이었고 두 번째 시뮬레이터는 없었다. 이전 기본 전달 설정의 낮은 최저RTF도 과거 보고서에 그대로 보존한다.
GPU GUI의 `PreRender` 콜백은 standard 평균137.23Hz·최저1초96Hz, lite143.4Hz·121Hz였다. 이는 **렌더 루프 빈도이며 디스플레이 present FPS가 아니다**. 실제 표시≥30FPS와 RTX5070/내장GPU 장비 결과로 대신하지 않는다.
실제 제어 시험 `recordings/control_20261109_v2/control_trial.json`에서3개 주행 지점, 짐벌 pan−0.39278rad/tilt0.10475rad 도달, 발사·재장전·탄약8→7, 명령 timeout 정지 후0m 이동을 확인했다.3인칭17.2초·1인칭17.5초 영상은 전 구간 디코딩했다.3인칭은1Hz 실제18장을5fps로 유지 표시하며1인칭은10Hz175장이다. 자율 M3/Nav2 채점 시험과 실제 표적 연속 명중은 이 시험 범위가 아니다.
별도의 실제 거점 검사 `validation/base_beams_live.json`에서는 B-2 한 대를 여섯 거점에 차례로 이동시켜 `set_pose`→실제 Gazebo 자세→ROS 심판의 차단/해제 이벤트를6/6 확인했다. ready 단계를 유지했고 점령 이벤트는0개이며 원래 자세로 복원했다. 이 검사는 경기 중 M5 점령이나 자율 거점 진입 성공을 대신하지 않는다.
6m 검증 카메라의27개 독립점은 지정 카메라 기하에 직접 투영했을 때 최대8.424mm로1cm 바닥 정합 기준을 통과했다. `validation/closeups/closeup_report.json`의 실제 근접 촬영에서는 표적·M3 면 각1개의 상하·좌우 방향이 원본과 일치했다(정방향 NCC0.98595/0.98150, 반전·회전 대안 최대0.59467/0.55484). 정본 YAML로부터 정한 시점에 충돌 없는 임시 카메라를 만들고 각각3프레임 수신 후 제거했으며 로봇 위치는 바꾸지 않았다. 모든 면의 인식 성공·광선 명중 정합과 대상 장비 성능은 추가 확인한다. CPU/GPU 기동 성공이나 SDF 구문 통과만으로 모두 합격으로 표시하지 않는다.
`validation/arena_single_ugv_ready.png`와 `single_ugv_ready.json`은 사용자 조작용 실제 GUI에서 B-2 한 대만 스폰한 ready 상태다.8초 동안450점 모두 유효한 스캔80회(10Hz), ego영상80장·UAV8장 및 odom·IMU를 확인했다. 경기 시작·자율주행 검증이 아니며 단독 실행은 `ugvs:=1`을 사용한다.
최종 ZIP v2 독립 추출본의 ROS 환경 전체 회귀는 **1352개 통과·실패0·skip0,1065.26초**로 완료했다. fixture의0나눗셈 경고1개가 있었으며 pyflakes 경고0, 추적표97/97개 S-ID·159개 spec 마커를 확인했다. 후보 ZIP611개 멤버의 해시와 시험 후 불변을 검사했고, ZIP에서 별도 새 colcon 빌드로2패키지를22.7초에 빌드했다. 최종 배포는 검증한 코드·시험·에셋과 해시를 대조하고 문서·검증 근거만 갱신한다. 상세 근거는 저장소 [RELEASE_VALIDATION.json](../../../docs/RELEASE_VALIDATION.json)을 따른다.
수정 전 전체23실패·1263통과, 현장 fixture31통과, ROS 미로드 비교202통과, 환경 격리 후 ROS155통과는 별도 이력으로 보존하며 합산하지 않는다. ZIP 후보 v1은 회피 회귀1실패·196통과(106.46초)에서 멈췄고, 예정된 적 UGV가 등장 전부터 관측·물리 세계에 존재하던 결함을 수정한 뒤 v2 전체가 통과했다. 전체 회귀는 실제 M3 사진 분류·자율 Nav2 전체 경기·실물 검증의 대체 근거가 아니다. 상세 수정 이력은 저장소 `docs/ARENA_ACCEPTANCE.md`를 따른다.
가속도 acc는 필드를 수용하지만 서보 내부 가감속 모델은 아직 없고 속도 제한만 구현했다. 배터리는 고정 가정값이다.

### 영상 전달 진단

`scripts/image_transport_probe.cc`는 Gazebo 원본 영상의 헤더만 세고, `scripts/image_ros_probe.py`는 브리지 전후 ROS 영상의 헤더만 센다. 같은 시간대에 실행하여 첫64개 촬영 시각, 받은 개수, 시각 간격, 중복·역행 수를 비교한다. 영상 데이터는 출력·저장하지 않는다. 실행 초기 발견 지연과 시뮬레이션 속도를 구분하려면 wall FPS와 촬영 시각 기준 FPS를 함께 본다. 이 숫자는 GUI FPS가 아니다.

`image_transport_profile:=shm64`는 이 환경에서 검증된64MiB 공유메모리와 UDPv4 검색 설정을 해당 launch의 모든 ROS 프로세스에 적용한다. 기본값 `inherit`는 기존 `FASTRTPS_DEFAULT_PROFILES_FILE`, `FASTDDS_DEFAULT_PROFILES_FILE`, RMW를 그대로 둔다. `shm64`를 명시하면 기존 프로필 대신 패키지 XML을 선택한다. 다른 RMW를 명시해 둔 경우에는 조용히 교체하지 않고 오류를 내므로 `inherit`를 유지하거나 Fast DDS를 명시적으로 선택해야 한다. OS 전역 공유메모리 한도·sysctl·네트워크 설정은 바꾸지 않는다.

별도 터미널의 제어·HTTP 어댑터·기록·검사 클라이언트도 시작 전에 같은 프로필을 지정해야 한다. launch 안의 설정은 이미 떠 있는 노드나 다른 셸에 전파되지 않는다. Fast DDS 클라이언트 셸에서 다음과 같이 설정한다.

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE="$(ros2 pkg prefix --share maicon_arena_gz)/config/fastdds_shm64.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

`validation/native_images_shm64.json`과 `ros_images_shm64.json`은 같은 실행에서 약20초 동안 원본과 ROS 양쪽 모두 ego199프레임, UAV20프레임,1500×1200 검증 카메라20프레임을 기록했다. 촬영 시각 최대 간격은 ego0.1초, UAV·검증 카메라1초였고 중복·역행은0이다. RGB→BGR 후 `/image_raw`도199프레임을 받았다. 수신 시작 시각이 약0.2초 다르므로 전체 프레임을 바이트 단위로 일대일 비교했다는 뜻은 아니다. 이전 기본 전달 설정의 UAV 약0.4FPS·큰 영상 미수신 문제는 이 측정에서 재현되지 않았다.

64MiB는 ego RGB와BGR, UAV, 검증 카메라의 depth5 영상 총50,040,000바이트에25% 여유를 둔62,550,000바이트를 수용한다. 이는 보수적인 프레임 저장 예산이며 모든 DDS 내부 할당이나 임의의 추가 카메라·큐 깊이를 보장하지 않는다. 새 영상 크기나 구독 구성을 추가하면 다시 측정한다.
