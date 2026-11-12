# Waveshare 대회 UGV와 ROS 항법 연결

검토 기준: 2026-11-06, Waveshare `ugv_ws/ros2-humble-develop` 커밋 `f0b3ad9c16e7acb6d6145f261116d71bf6338f4a`, 2차 사전교육 PDF p10~18. 실제 대회 이미지에서의 동작 확인은 별도이다.

## 소유권과 데이터 흐름

```text
대회 제공 베이스/센서/MIL 노드
  /scan, /odom, /imu/data, /image_raw, /mil/state, /mil/match
  TF odom → base_footprint → base_link → 센서
       ↓
우리 SLAM: map → odom
       ↓
우리 Nav2: /navigate_to_pose → /cmd_vel_nav2
       ↓
우리 Agent: 임무·회피·안전·경기 제어 허용 판단 → /cmd_vel
```

`maicon_waveshare.launch.py`는 기존 로봇 시스템 위에 SLAM/Nav2만 추가한다. 드라이버, EKF, `robot_state_publisher`, `/odom` TF 변환기를 다시 띄우지 않는다. Nav2 controller와 behavior의 속도 출력은 모두 `/cmd_vel_nav2`이며 `/cmd_vel`의 최종 발행자는 Agent 하나로 제한한다. PDF p15의 동시 속도 명령 금지를 지키기 위한 구조이다.

제조사 EKF launch에서는 `robot_localization`이 `/odom_raw`와 `/imu/data`를 융합해 `/odom`으로 내보내고 `odom → base_footprint`를 발행한다. IMU 입력은 yaw와 yaw 각속도이며 가속도 이중 적분 기반 위치 추정이 아니다. `base_footprint → base_link`와 센서 TF는 제조사 URDF 쪽 소유다. 대회 이미지가 다른 프레임을 사용한다면 실제 TF를 먼저 확인해야 한다.

제조사 일반 저장소에는 `/mil/*` 경기 API가 없다. 제조사 bringup만 실행한다고 대회 API가 생기지 않는다. 대회 제공 시스템의 해당 노드를 사용하며 게임 제어를 우회하는 직렬 포트 명령은 사용하지 않는다.

## 실기 실행

대회 관리자가 제공한 기본 시스템이 실행 중인 상태에서 시작한다. `/scan`, `/odom`, TF와 `/mil/*`를 확인한 뒤 프로젝트 루트에서:

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash  # 실제 배포 경로가 다르면 그 경로 사용
ros2 launch ros2/launch/maicon_waveshare.launch.py
```

이 실행은 `base_footprint`를 Nav2 BT·local/global costmap·behavior와 Cartographer/slam_toolbox에 일치시킨다. Agent 역시 `config/waveshare_real.json`의 `nav2.base_frame=base_footprint`를 사용해야 한다. 시뮬레이션용 `maicon_nav.launch.py`의 기본값 `base_link`는 유지된다.

이미 대회 시스템에 SLAM이나 AMCL 등이 `map → odom`을 발행 중이면 다음처럼 우리 SLAM만 끈다:

```bash
ros2 launch ros2/launch/maicon_waveshare.launch.py slam:=none
```

다른 Nav2도 이미 실행 중이면 controller/action 이름과 명령 발행이 중복되므로 동시에 실행하지 않는다. 제조사 `ugv_slam/cartographer.launch.py`는 LiDAR bringup까지 포함하므로 대회 베이스 위에서 무작정 함께 실행하지 않는다.

깊이 카메라의 실제 장착 TF·CameraInfo·시간 동기화를 확인하기 전에는 실기 launch의 깊이 장애물 입력을 기본 비활성화한다. `depth_obstacles:=true`로 바꿀 때는 Agent의 깊이/복셀 설정도 함께 활성화하고 낮은 장애물 시험을 해야 한다. 2D LiDAR만 활성화된 상태에서는 낮은 장애물 안전성이 확보되었다고 주장할 수 없다.

Nav2 외곽은 현재 프로젝트의 가정값을 유지한다. PDF의 250(가로)×230(세로)mm를 전방 길이/좌우 폭으로 어떻게 대응시킬지는 현장에서 확인해야 하며, 제조사 다른 모델 URDF 치수를 그대로 대회 UGV 치수로 채택하지 않는다.

## 현장에서 확인할 항목

```bash
ros2 topic info /odom --verbose
ros2 topic hz /odom
ros2 topic info /cmd_vel --verbose
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 topic echo /scan --once
ros2 topic echo /mil/state --once
ros2 topic echo /mil/match --once
```

`/scan.header.frame_id`의 실제 이름까지 TF로 연결되어 있어야 한다. `/odom`은 대회 PDF상 20Hz지만 제조사 일반 launch의 EKF 설정은 30Hz이므로 실측 토픽 주기와 신선도를 기준으로 확인한다. 제조사 오도메트리 코드의 바퀴 간격 0.175m도 실측 보정 대상이다. Agent 시작 전에는 다른 주행 명령 발행자를 중지하고, 시작 후에는 `/cmd_vel`에 최종 발행자가 하나인지 확인한다.

## 검증 범위

- ROS 없이 실행되는 launch 계약 테스트: 하드웨어 드라이버/TF 중복 실행 없음, 최종 속도 중재, 프레임 일치, 외부 SLAM 선택, 깊이 입력 비활성화, 기존 시뮬 기본값 유지.
- WSL Ubuntu 22.04 / ROS2 Humble에서 `ros2 launch ... --show-args` 성공.
- 실제 Humble `RewrittenYaml`로 중첩 costmap 프레임과 깊이 관측 소스 변경 확인. 외벽 관측 및 A* 설정 보존.
- localhost 전용 ROS domain 187에서 실제 launch 기동 확인: Cartographer 설정 파싱과 Nav2 노드 생성 성공, `/cmd_vel` 발행자 0개, `/cmd_vel_nav2` 발행자는 controller/behavior만 존재, `/odom`·`/scan` 발행자 0개로 드라이버 중복 없음. 센서를 넣지 않아 `odom → base_footprint` 대기 상태이며 활성화/주행 성공 시험은 아니다.
- 위 무센서 시험을 SIGINT로 종료할 때 Humble controller_server가 활성화 대기 중 lifecycle context 오류와 종료 코드 -6을 기록했다. 실행 중 기동 오류는 없었지만 종료 경로는 추가 실기 검증 대상이다. 전체 로그는 작업 폴더의 `review_2026-11-06/integration/ros_hardware_smoke.log`에 있다.
- 실물 로봇 센서·구동·경기 서버 통합 주행은 미검증. launch 로딩 성공을 주행 성공으로 해석하지 않는다.

## 2026-11-07 위키 기반 연결 보완

제조사 위키의 일반 제품 예제와 대회 프로그램의 제어 소유권을 구분한다. [Beast Jetson 위키 7장](https://www.waveshare.com/wiki/UGV_Beast_Jetson_Orin_ROS2_7._Navigation_and_SLAM_Mapping)은 Nav2+SLAM, `explore_lite` 탐색, `map_saver_cli` 지도 저장을 안내한다. 제조사 `ugv_nav/slam_nav.launch.py`는 LiDAR bringup까지 함께 실행하므로 대회 베이스에 그대로 중첩하지 않는다. 이 프로젝트는 기존 온라인 탐색·면 장부를 유지하며, 별도 `explore_lite`를 Agent와 동시에 실행하지 않는다. 두 프로그램이 같은 NavigateToPose에 목표를 보내면 서로 목표를 선점할 수 있다.

4·6장의 정확한 Beast Jetson 원문은 접근 제한으로 본문을 확보하지 못했고, 동명 Rover/PI 문서를 해당 문서로 간주하지 않았다. 2D 매핑 및 기존 지도 항법 동작은 같은 upstream의 `ugv_slam/launch/cartographer.launch.py`, `ugv_nav/launch/nav.launch.py`를 교차 확인했다. 경기 중 박스가 이동할 수 있으므로 현재 온라인 지도 구성을 저장 지도 기반 AMCL/EMCL 항법으로 무조건 바꾸지 않는다. 저장한 PGM/YAML은 측정·검토 자료이며, 경기장 좌표 앵커·현재 상태의 재확인 없이 다음 경기의 정답 지도로 사용하지 않는다.

필터된 LiDAR를 사용할 때는 모든 소비자에 같은 토픽을 전달한다:

```bash
ros2 launch ros2/launch/maicon_waveshare.launch.py \
  scan_topic:=/scan_filtered odom_topic:=/odom nav_cmd_topic:=/cmd_vel_nav2
```

Agent 설정도 `hal.scan_topic=/scan_filtered`, `hal.odom_topic=/odom`, `nav2.cmd_topic=/cmd_vel_nav2`로 일치시켜야 한다. launch는 Cartographer/slam_toolbox·local/global 비용지도·선택적 odom TF bridge에 이 값을 전달한다. `nav_cmd_topic=/cmd_vel`은 거부한다. 이 변경은 필터 노드를 만들거나 마스킹 각도를 임의로 정하지 않으며, 실측된 각도와 센서 프레임이 필요하다.

`base_frame=base_link`이면 `cartographer_2d.lua`, `base_frame=base_footprint`이면 `cartographer_waveshare.lua`를 자동 선택한다. 내장 Lua와 프레임을 서로 다르게 명시하면 노드를 시작하기 전에 오류로 중단한다. 다른 프레임은 `ros2/config` 내의 명시적 사용자 Lua를 요구한다. 사용자 Lua 내부 tracking_frame은 작성자가 실제 TF와 맞춰야 한다.

일반 launch의 `use_sim_time`은 중첩 비용지도까지 명시적으로 치환한다. 실기 wrapper는 실제 시계를 유지한다. 사용자 조정 YAML은 유지하며 선택한 프레임·토픽·시계만 덮어쓴다. Humble에서 `RewrittenYaml`이 없는 controller의 odom_topic 키를 추가하지 못하는 경우를 고려해 controller 파라미터도 직접 설정한다.

검증: launch 계약 시험 16개 통과, WSL ROS2 Humble `--show-args` 및 실제 RewrittenYaml 변환으로 중첩 비용지도 프레임·시계·필터 토픽과 controller odom 파라미터 확인. 실제 주행 성공률 시험은 이번 변경 범위에 포함하지 않는다.

## 원문 링크

- [Waveshare EKF 실행](https://github.com/waveshareteam/ugv_ws/blob/f0b3ad9c16e7acb6d6145f261116d71bf6338f4a/src/ugv_main/ugv_bringup/launch/bringup_imu_ekf.launch.py)
- [Waveshare EKF 설정](https://github.com/waveshareteam/ugv_ws/blob/f0b3ad9c16e7acb6d6145f261116d71bf6338f4a/src/ugv_main/ugv_bringup/param/ekf.yaml)
- [Waveshare 오도메트리](https://github.com/waveshareteam/ugv_ws/blob/f0b3ad9c16e7acb6d6145f261116d71bf6338f4a/src/ugv_main/ugv_base_node/src/base_node_ekf.cpp)
- [대회 서버 API](https://maicon.kr/wp-content/kossa/2026MAICON/game-server-api.html)
