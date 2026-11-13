# TakeTheLoss | MAICON 2026 Gazebo UGV

MAICON 2026의 **6.12×5.00m 경기장**과 WINGBOT UGV를 Gazebo Fortress에서 시험하는 저장소입니다. 현재 실행 코드는 `maicon2026_ugv_code/maicon2026/maicon26/`입니다. 과거 후보 02 동결본과 선택 프로필은 현재 실행에 사용하지 않습니다.

## 현재 검증한 구성

- B-2 한 대, Cartographer 위치 추정, Nav2 Navfn A* 경로 계획
- 2D LiDAR 자기 반사 필터, UAV 영상 기반 박스 지도, 지상 카메라 기반 면 식별
- M1 지정 구역 R-S1·R-S2 탐색, 면 장부와 UAV 지도를 포함한 30fps 녹화
- Nav2 실패 시 해당 관측 지점을 제외하고 다른 접근을 재시도

2026-11-13의 5분 시험에서 R-S1은 **6/6**, R-S2는 **3/3**으로 M3 종류별 보고 수가 일치했습니다. Nav2 정체는 3회 있었으며 모두 복구했고, 정체 판정 후 실제로 15cm 이동하기까지 최대 시간은 이전 84.6초에서 5.2초로 줄었습니다. 정체 발생 자체가 제거됐거나 실물 성능을 검증했다는 의미는 아닙니다. [검증 요약](maicon2026_ugv_code/maicon2026/docs/NAV2_M3_VALIDATION_20261113.json)을 참고하세요.

## 실행 준비

Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress, colcon, ROS/Gazebo 브리지, Nav2 및 Cartographer가 필요합니다. ROS 의존성과 설정은 [ROS 실행 안내](maicon2026_ugv_code/maicon2026/ros2/README.md), [경기장 패키지](maicon2026_ugv_code/maicon2026/ros2/maicon_arena_gz/package.xml), [로봇 패키지](maicon2026_ugv_code/maicon2026/ros2/wingbot_description/package.xml)를 확인하세요.

```bash
git clone https://github.com/Jo-Young-Woo/TakeTheLoss.git
cd TakeTheLoss/maicon2026_ugv_code/maicon2026
pip install numpy scipy pytest pyyaml opencv-python requests
ARENA_BUILD_WS="$HOME/maicon_arena_ws" bash scripts/build_arena_gz.sh
export WINGBOT_MODEL_INSTALL="$HOME/maicon_arena_ws/install"
source /opt/ros/humble/setup.bash
source "$WINGBOT_MODEL_INSTALL/setup.bash"
```

빌드 도우미는 원본을 공백 없는 Linux 작업공간으로 복사합니다. 기존 빌드 작업공간을 재사용하기 전에 [빌드 안내](maicon2026_ugv_code/maicon2026/ros2/maicon_arena_gz/docs/runtime.md)를 읽으세요.

## B-2 단독 5분 경기와 녹화

```bash
python3 scripts/run_wingbot_selfplay.py \
  --output recordings/b2_nav2_m3 \
  --ugvs 1 --nav-backend nav2 \
  --duration 300 --seed 0 --variant progress --scenario selfplay \
  --record-view observer --observer-video-fps 30 --render-device cpu \
  --ros-domain-id 210 --ready-timeout 240 --wall-timeout 3600
```

실행할 때마다 새로운 출력 폴더를 지정하세요. 생성되는 `wingbot_selfplay_third_person.mp4`에는 실제 Gazebo의 비스듬한 시점과 B-2 면 장부·UAV 지도가 표시됩니다. `selfplay_report.json`과 `B-2/mission_report.json`에 경기 기록이 저장됩니다. CPU 렌더링은 이번 검증 환경에서 사용한 설정이며, GPU는 별도로 확인한 환경에서 `--render-device gpu`로 선택할 수 있습니다.

Nav2 경기 실행기는 현재 **한 대 구성만 지원**합니다. 6대 자가 대전은 `--ugvs 6 --nav-backend builtin`을 사용하며, 위 단독 시험 결과를 3대3의 검증 결과로 해석하지 마세요.

## 코드와 자료

| 위치 | 내용 |
| --- | --- |
| [maicon26](maicon2026_ugv_code/maicon2026/maicon26/) | 임무, 항법, 카메라·LiDAR·UAV 인식 코드 |
| [경기장](maicon2026_ugv_code/maicon2026/ros2/maicon_arena_gz/README.md) | 경기장 사양, 배치, 월드, 표적·바닥 자산 |
| [WINGBOT](maicon2026_ugv_code/maicon2026/ros2/wingbot_description/README.md) | 로봇 형상, 센서, 물리 모델 |
| [scripts](maicon2026_ugv_code/maicon2026/scripts/README.md) | 실행·녹화·분석 도구 |
| [tests](maicon2026_ugv_code/maicon2026/tests/) | 단위·통합·회귀 시험 |
| [문서](maicon2026_ugv_code/maicon2026/docs/README.md) | 설계 및 시점별 검증 기록 |

이번 업로드에는 실행 코드와 검증 요약을 포함합니다. 새 녹화 원본·임시 실험 데이터는 추가하지 않습니다. 과거 보고서는 해당 시점의 결과이며, 현재 구현의 성능 보증이나 공식 경기 점수가 아닙니다. 경기 자료와 참고 모델 출처는 [자산 기록](maicon2026_ugv_code/maicon2026/ros2/wingbot_description/vendor/LICENSE_AUDIT.md)에 정리돼 있습니다.
