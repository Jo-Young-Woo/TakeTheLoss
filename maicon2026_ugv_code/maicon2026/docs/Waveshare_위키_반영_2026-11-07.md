# Waveshare 위키 기반 코드 개선 — 2026-11-07

기존 M1~M6, M3 온라인 탐색·면 장부와 대회 `/mil/*` 제어 중재를 유지하면서 ROS 설정, 깊이 영상 검증, LLM 연결, Gazebo 시간·센서 처리를 개선했다. 이 변경은 실제 기체 보정이나 대회 주행 성공을 뜻하지 않는다.

## 원문 확인 범위

요청한 10개 URL의 직접 접속은 모두 HTTP 403이었다. 1·2·5·7·8·9·10장은 공식 Waveshare 검색색인 본문(약 4~5개월 전 수집)을 읽고 제조사 코드와 대조했다. **4·6·11장은 해당 Beast 문서 본문을 확보하지 못했다.** 다른 모델 문서를 해당 문서로 간주하지 않고, 공통 제조사 저장소 코드와 확인 가능한 다른 장의 참조만 사용했다. 최신 웹페이지를 전부 확인했다고 해석하면 안 된다.

제조사 코드 기준: [ugv_ws ros2-humble-develop](https://github.com/waveshareteam/ugv_ws/tree/ros2-humble-develop), 검토한 로컬 소스 커밋 `f0b3ad9c16e7acb6d6145f261116d71bf6338f4a`. 대회 전용 API·속도 제한·팀 노드 수명은 두 사전교육 PDF와 공식 경기 서버 문서를 우선한다.

| 요청한 문서 | 확인 내용과 이번 적용 |
|---|---|
| [1. Preparation](https://www.waveshare.com/wiki/UGV_Beast_Jetson_Orin_ROS2_1._Preparation) | 제조사 일반 프로그램과 ROS 실행 환경의 자원 공유를 확인. 팀 코드가 운영진 드라이버를 종료하거나 Docker·부팅 설정을 변경하지 않음. 기존 베이스 위에 overlay만 실행 |
| [2. RViz 모델](https://www.waveshare.com/wiki/UGV_Beast_Jetson_Orin_ROS2_2._RViz_View_Product_Model) | GUI가 실물 구동과 연결될 수 있음. launch의 차체 프레임·Cartographer 설정 일치를 검증하고, 단순 모델 보기와 구동을 구분 |
| [4. LiDAR 지도](https://www.waveshare.com/wiki/UGV_Beast_Jetson_Orin_ROS2_4._2D_Mapping_Based_on_LiDAR) | 본문 미확보. 제조사 Cartographer/Gmapping launch 대조에 근거해 센서 토픽을 SLAM·비용지도·팀 코드에 일관되게 전달 |
| [5. 깊이 3D 지도](https://www.waveshare.com/wiki/UGV_Beast_Jetson_Orin_ROS2_5._3D_Mapping_Based_on_Depth_Camera) | 제조사 RTAB-Map의 RGB-D+LiDAR 구성을 확인. 우리 복셀 입력에 CameraInfo·광학 프레임·TF 검증을 추가. RTAB-Map을 대회 실행에 추가로 중첩하지 않음 |
| [6. 자동 항법](https://www.waveshare.com/wiki/UGV_Beast_Jetson_Orin_ROS2_6._Auto_Navigation) | 본문 미확보. 제조사 nav launch와 9·10장의 AMCL 참조 확인. 고정 사전지도 기반 항법으로 일괄 교체하지 않고 현재 SLAM+Nav2 유지 |
| [7. 항법과 SLAM](https://www.waveshare.com/wiki/UGV_Beast_Jetson_Orin_ROS2_7._Navigation_and_SLAM_Mapping) | SLAM+Nav2, explore_lite 구조 확인. M3에는 기존 탐색·면 장부를 사용하고 외부 탐색기가 같은 Nav2 목표를 덮어쓰지 않도록 실행 지침 수정 |
| [8. 자연어](https://www.waveshare.com/wiki/UGV_Beast_Jetson_Orin_ROS2_8._Web_Natural_Language_Interaction) | Ollama 연결 개념을 M1·M6 JSON 해석 어댑터에 적용. 명령문을 Python 코드로 실행하거나 바퀴 명령으로 직접 변환하지 않음 |
| [9. 웹 제어](https://www.waveshare.com/wiki/UGV_Beast_Jetson_Orin_ROS2_9._Web-based_Control_Tool) | 웹 조작·내비게이션이 제어 발행자가 된다는 점 확인. `/cmd_vel` 단독 중재 유지, 상대 토픽명으로 분리 검사를 우회하는 설정 거부 |
| [10. 명령 상호작용](https://www.waveshare.com/wiki/UGV_Beast_Jetson_Orin_ROS2_10._Command_Interaction) | `/behavior` 및 저장된 항법 지점 방식 확인. 대회에서는 기존 Agent가 목표를 결정하고, LLM 응답은 M1/M6 허용값 검증을 거침 |
| [11. Gazebo](https://www.waveshare.com/wiki/UGV_Beast_Jetson_Orin_ROS2_11._Gazebo_Simulation_Debugging) | 본문 미확보. 제조사 Classic 구성과 현재 Fortress 구성을 코드로 구분. 현재 Gazebo의 센서 QoS·신선도, 일시정지 시간, LiDAR 프레임을 교정 |

## 같은 설정으로 Agent와 ROS 실행

`maicon26/launch_profile.py`가 실기 진입점과 자동 실행 노드의 설정 로딩을 공유한다. CLI의 마지막 `--config`/`--overrides`/`--nav`가 실제로 실행되는 SLAM/Nav2에도 반영된다.

- `hal.scan_topic`, `hal.odom_topic`, `nav2.cmd_topic`, `nav2.base_frame`을 자식 launch로 전달한다.
- 깊이 비용지도 입력은 `depth.enable`, `map3d.enable`, `map3d.use_for_nav`, `nav2.publish_depth_obstacles`가 모두 켜져 있을 때만 활성화한다.
- `use_sim_time`은 중첩된 비용지도에도 전달한다. 실기 wrapper는 false를 유지한다.
- Cartographer Lua를 차체 프레임에 맞춰 자동 선택하고 내장 설정과 프레임이 다르면 거부한다.
- `--nav builtin` 또는 `--help`에는 불필요한 Nav2 자식 프로세스를 실행하지 않는다.
- 두 실행 파서에서 옵션 축약을 거부해 `--conf`가 한쪽에서만 적용되는 상황을 막는다. 준비 제한시간의 NaN/무한대도 거부한다.
- 자동 wrapper가 지원하지 않는 지도·액션·점군 이름은 오류로 알린다. 이름만 바뀐 Agent와 기본 ROS 그래프를 조용히 섞지 않는다.

로봇에서 ROS 환경을 source한 뒤 프로젝트 디렉터리에서 실행하는 예:

```bash
export TEAM_LAUNCH_NAV=1
export MAICON_SLAM=cartographer  # cartographer | slam_toolbox | none
python3 team/team_node.py --preflight-only
```

이미 운영 중인 SLAM을 이용할 때는 `MAICON_SLAM=none`, 이미 같은 Nav2까지 실행 중이면 `TEAM_LAUNCH_NAV`를 끈다. 자동 실행의 환경변수는 운영진이 팀 노드를 실행하는 프로세스에 전달되어야 한다.

현장 보정한 Nav2 YAML과 BT XML은 `MAICON_NAV_PARAMS`, `MAICON_NAV_BT`로 지정할 수 있다. 이것은 파일을 선택하는 기능이며, 모든 측정값을 Nav2 파일로 자동 재생성했다는 뜻이 아니다. 구동·외곽·정지거리 보정 결과를 기존 보정 도구로 생성한 후 선택한다.

## 깊이 카메라를 켜는 조건

실기 기본값은 계속 OFF다. 위키에서 OAK-D/RTAB-Map을 지원한다고 해서 대회 기체의 토픽명·장착값이 확정되는 것은 아니다.

실측 후 설정할 항목은 `depth.require_calibration=true`, `depth.calibration_verified=true`, 실제 `depth.optical_frame`, `depth.topic`, `depth.info_topic`, 그리고 `rig.depth_xyz`, `rig.depth_rpy`, `rig.base_z`다. 보정 확인 플래그만 켜도 동작하는 구조가 아니다.

- CameraInfo의 유효한 K/P, 해상도, 프레임을 보존하고 깊이 Image와 대조한다. 서로 다르면 폐기한다.
- 가정 HFOV나 다른 해상도의 K를 실기에서 조용히 대신 사용하지 않는다.
- 촬영 시각의 ROS TF와 설정된 RigModel 광학축을 비교한다. 기본 허용 차이는 위치 2cm, 회전 2도이며 이는 실측 정확도 보장이 아닌 일치 검사 기준이다.
- 잘못된 새 입력이 들어오면 이전 깊이 프레임도 제거한다. 엄격 모드에서 깊이가 없거나 오래되면 최종 주행 출력을 0으로 제한한다.
- 거리 범위 안의 유효 깊이가 한 픽셀도 없거나 같은 시각의 프레임만 반복되면 정상 센서 수신으로 취급하지 않는다. CameraInfo/TF 실패 원인은 사전 점검 오류에 포함한다.
- 시뮬레이션의 가상 센서 설정은 유지한다. 우리 복셀은 임무용 장애물 표현이고 RTAB-Map의 DB·루프 폐합 지도와 같은 자료구조가 아니다.

제조사 RTAB-Map launch는 드라이버/OAK를 함께 시작하고 mapping 모드에서 `-d`를 사용한다. 기존 DB를 보존하려는 환경이나 이미 드라이버·SLAM이 실행 중인 대회 프로그램 위에서 그대로 실행하지 않는다. 상세 근거는 [HAL 문서](hardware_integration.md)를 참조한다.

## Ollama 연결

```bash
export MAICON_OLLAMA_MODEL='<이미 설치된 모델 이름>'
export MAICON_OLLAMA_URL='http://127.0.0.1:11434'
export MAICON_LLM_FACTORY='maicon26.llm.ollama:create_client'
```

`run_real.py`를 직접 실행할 때는 `--llm-factory maicon26.llm.ollama:create_client`를 사용한다. JSON 응답, 시간 제한, 응답 크기, 종료와 늦은 응답 처리를 검증했다. 실제 모델을 설치하거나 내려받지 않았고, Jetson에서의 지연·한국어 해석 정확도는 별도 측정이 필요하다. [Ollama 연결 문서](ollama_integration.md)에 설정과 시험법이 있다.

## Gazebo와 검증 범위

현재 사용자가 구성한 Fortress 경기장·시각 자료를 보존하면서 실행 경계의 오류를 교정했다. 기존 SDF가 센서 좌표를 잘못 표시한 경우 새 생성기로 별도 파일을 생성해야 하며, 이 작업에서 기존 경기장 파일을 덮어쓰지 않았다. 상세 실행 조건은 [Gazebo 실행 계약](gazebo_runtime_contract.md)을 참조한다.

테스트는 새 기능의 모의 ROS/HTTP 및 기존 보정·깊이·HAL 회귀 검사로 수행한다. ROS launch의 인자 및 중첩 YAML 변경은 WSL ROS2 Humble에서도 확인했다. 실물 로봇 주행, 실제 Ollama 모델 추론, 경기 서버 접속을 완료했다는 뜻은 아니다.

| 실행한 검사 | 결과 |
|---|---|
| 최종 설정·실기 진입점·ROS launch·Ollama·깊이 입력 계약 통합 | **139 passed**, 3.42초 |
| Gazebo 계약·기존 고정 경기장·ROS launch | **39 passed** |
| 기존 깊이/복셀 회귀 `test_depth3d.py` | **31 passed** |
| HAL/MIL와 깊이 계약 중간 통합 | **82 passed**. 이후 추가한 깊이 사례는 위 139개 검사에 포함 |
| 보정·현장 실험·기본 단위 검사 | 처음 50 passed, 1 skipped, 체크리스트 불일치 1건. 새 설정 항목을 포함해 체크리스트 재생성 후 해당 검사도 통과 |
| 실제 WSL ROS2 Humble | launch 인자 및 중첩 RewrittenYaml 확인, Gazebo 셸 스크립트 `bash -n` 통과 |

검사군이 겹치므로 개수를 더하지 않는다. 전체 경기 성공률을 다시 측정한 결과도 아니다. OpenCV 어안 보정 skip은 기존 Windows 환경 조건이다. 원시 기록과 변경 전 백업은 Git에서 제외된 `review_2026-11-07/`에 있다.

최종 통합 검사 명령(프로젝트 디렉터리, Windows PowerShell):

```powershell
$env:PYTHONUTF8='1'
python -m pytest -q -p no:cacheprovider --disable-warnings tests/test_launch_profile.py tests/test_real_preflight.py tests/test_real_runtime.py tests/test_ros_hardware_launch.py tests/test_ollama.py tests/test_llm_integration.py tests/test_depth_contract.py
```
