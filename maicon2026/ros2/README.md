# SLAM + Nav2 (ROS2 Humble) — 실기 항법

대회 Waveshare 시스템 위에서는 `ros2 launch ros2/launch/maicon_waveshare.launch.py`를 사용한다. 베이스 드라이버·EKF·정적 TF를 중복 실행하지 않고 `base_footprint`를 기준으로 연결한다. 깊이 장애물 입력은 실측 전 기본 OFF이다. 자세한 실행 및 프레임/토픽 확인은 [Waveshare 연결 안내](../docs/waveshare_ros_integration.md)를 따른다. 아래 기존 `maicon_nav.launch.py` 예제는 `base_link`를 사용하는 시뮬레이션/일반 구성을 유지한다.

> **상태: WSL Ubuntu 22.04 + ROS2 Humble에서 2D 시뮬레이터에 연결해 실제 Cartographer/slam_toolbox + Nav2로 돌려 봤다. 실제 로봇에서는 아직 실행하지 못했다.**
> 아래 "WSL 시뮬 검증"의 수치는 시뮬 센서(이상적인 LiDAR, 키네매틱 로봇) 기준이다. 로봇의 토픽·프레임·속도·풋프린트는 현장에서 다시 확인한다.

## WSL 시뮬 검증 (`scripts/ros_sim_demo.sh`)
```bash
# WSL Ubuntu-22.04 안에서 (프로젝트 폴더로 이동한 뒤)
bash scripts/ros_sim_demo.sh cartographer --robot B2 --seed 2 --shuffle 60 150     # 약 4~5분(실시간)
NAV2=false bash scripts/ros_sim_demo.sh cartographer --robot B2 --t-base 40         # Nav2 없이 SLAM만 → 내장 A* 폴백 시험
```
`sim/ros_bridge.py`가 시뮬을 `/clock`, `/scan`, `/odom` + TF `odom→base_link`로 발행하고 `/cmd_vel`을 받는다(Nav2의 `/cmd_vel_nav2`는 로봇에 연결하지 않는다 — 실기와 같은 중재 구조).
모든 ROS 노드는 `use_sim_time:=true`.

결과(단일 UGV, 60 s·150 s에 중앙부 박스 6~9개 이동/추가, 5분 전체 경기):
| 실행 | SLAM | M5 진입 | 충돌 | 위치오차 중앙값/최대 (SLAM+앵커 vs 참값) |
|---|---|---|---|---|
| B2 시드2 | Cartographer | 245.9 s | 0 | 1.4 / 6.1 cm |
| R2 시드3 (map 프레임이 경기장과 180° 어긋남) | Cartographer | 246.3 s | 0 | 2.5 / 8.1 cm |
| B3 시드4 | Cartographer | 244.5 s | 0 | 2.6 / 9.4 cm |
| R1 시드5 | Cartographer | 245.7 s | 0 | 1.8 / 7.4 cm |
| B1 시드6 | slam_toolbox | 244.4 s | 0 | 3.3 / 7.3 cm |
| B2 (Nav2 끔, 거점명령 40 s) | Cartographer | 44.6 s | 0 | 1.6 / 6.1 cm — 폴백 8회 |

처음 돌렸을 때 실제로 고친 것:
1. `bt_navigator`가 `ComputePathThroughPoses` BT 노드를 못 읽고 죽음 → `plugin_lib_names`에 설치된 플러그인 전부 추가(설치된 Nav2 기준).
2. (당시) 직사각형 풋프린트에서 `collision ahead`가 매치당 ~400회 → 원형 `robot_radius` 0.12 로 줄였다. **이후 원형이 모서리 충돌을 가린다고 보고 다시 직사각형(250×230 mm + 계획 여유 49 mm, `footprint_padding: 0.0`)으로 되돌렸다.** 실제 Nav2 10회: 9회 M5 도착·충돌 0, 6회 내장 A\* 폴백("목표 연속 실패(rejected)"), `collision ahead` 2~296회(편차 큼). 원인(거부 사유) 미분석 — `docs/차체외곽_LiDAR오차_경기장유효성.md` 참조.
3. 종료 시 `rclpy-spin` 스레드 예외 → `ros_runtime.shutdown()`으로 순서 정리.
4. Nav2 서버가 없을 때 로봇이 가만히 서서 목표만 계속 재전송함 → `Nav2Stack`이 서버 부재(1.5 s) 또는 한 목표 연속 실패 시 내장 A*로 폴백.

남은 것: 복구(spin/backup) 동작이 매치당 5~9회 남아 있고 그동안 몇 초씩 정지한다(경기 시간 영향은 이 시나리오에서 M5 도달 시각 기준 ±1 s). 한 대만 ROS로 연결했으므로 3v3 상호 간섭은 이 경로로는 시험하지 않았다.

## 왜 이렇게 나눴나 — 구조물이 바뀌는 경기장
| 문제 | 대응 |
|---|---|
| 박스 배치가 경기마다(경기 중에도) 바뀜 → 미리 만든 지도 불가 | **SLAM**으로 매번 실시간 위치추정 (Cartographer 2D 또는 slam_toolbox) |
| SLAM `/map`은 서브맵 누적이라 옮겨진 박스가 한동안 남음 → 유령 장애물 | Nav2는 **`/map`을 안 쓴다.** SLAM은 TF(`map→odom`)만 책임 |
| 박스가 옮겨지면 즉시 반영돼야 함 | 전역·지역 비용지도 모두 **LiDAR 마킹 + 레이캐스트 클리어링**(obstacle_layer). 전역 재계획 2 Hz |
| 경기장 밖으로 경로가 새면 안 됨 | `maicon26`이 외벽 둘레를 `/arena_walls_cloud`(마킹 전용 PointCloud2)로 알림 |
| Nav2와 조준·교전 코드가 구동을 두고 충돌 | Nav2 출력은 **`/cmd_vel_nav2`**. `maicon26` Agent가 읽어 LiDAR 안전 제한·가속 제한을 거쳐 `/cmd_vel`로 보냄 |

## 데이터 흐름
```
로봇 드라이버 ──/scan, /odom, TF odom→base_link──┐
                                                 ├─ SLAM(cartographer|slam_toolbox) ── TF map→odom
Nav2 (controller/planner/behavior/bt_navigator) ◄┘   (비용지도: /scan 마킹+클리어링, /arena_walls_cloud)
   │ 액션 /navigate_to_pose ◄── maicon26 Nav2Stack (경기장 좌표 ↔ map 좌표 변환, 시작 때 앵커링)
   └ 속도 /cmd_vel_nav2 ───► maicon26 Agent ── 안전 제한 ──► /cmd_vel (로봇)
```
`map` 프레임의 원점은 SLAM이 시작한 자세다. `Nav2Stack.start()`가 벽 기준으로 구한 경기장 시작 자세와 TF를 짝지어
**앵커**(경기장←map 변환)를 한 번 정하고, 이후 모든 목표·위치를 그 변환으로 오간다. BLUE(+x 향함)/RED(−x 향함) 모두 처리한다.

## 준비
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-cartographer-ros ros-humble-slam-toolbox   # Docker 안에서; 인터넷 필요
```
로봇 드라이버가 이미 내야 하는 것: `/scan`(LaserScan), `/odom`와 TF `odom→base_link`, TF `base_link→라이다 프레임`.
`base_frame`, `scan_topic`, `odom_topic`, `nav_cmd_topic` launch 인자로 Agent 설정과 일치시킨다. 기본 Cartographer Lua는 base_link/base_footprint에 맞춰 자동 선택한다. 그 외 차체 프레임은 일치하는 `cartographer_config`를 명시해야 한다. `odom` 프레임 자체를 변경하는 경우에는 SLAM·Nav2 설정도 함께 변경한다.

## 단계별 확인 (각 단계가 통과해야 다음으로)
1. **SLAM만**: `ros2 launch ros2/launch/maicon_nav.launch.py slam:=cartographer nav2:=false` 후
   `ros2 run tf2_tools view_frames` 에 `map→odom→base_link` 가 이어지는지, RViz에서 `/map`이 그려지는지.
   제자리 회전 시 지도가 같이 돌면 실제 LiDAR 프레임의 장착 TF·방향과 오도메트리 회전 부호를 확인한다.
2. **Nav2 기동**: 로그에 `lifecycle_manager_navigation ... Managed nodes are active`. 실패하면 `ros2 param list /controller_server` 로
   `nav2_params.yaml`의 이름을 대조(특히 RPP 컨트롤러·`behavior_server`). BT 오류면 `fast_replan:=false`.
3. **속도 리맵**: `ros2 topic info /cmd_vel_nav2` 에 발행자가 controller_server/behavior_server 인지, `/cmd_vel` 에는 Nav2가 **없는지**.
4. **손으로 목표**: RViz "2D Goal Pose" 또는
   `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"`
   로 `/cmd_vel_nav2` 가 나오는지 시뮬레이션에서 확인한다(Agent가 중재하지 않으면 로봇이 움직이지 않는 것이 정상).
   실기에서는 Agent와 외부 목표 발행자를 동시에 실행하지 않는다. 단순 topic relay로 `/cmd_vel`에 연결하면 임무·경기 상태와 안전 제한을 우회하므로 사용하지 않는다.
5. **maicon26**: `python scripts/run_real.py --nav nav2 ...`. 로그 `init ... backend=nav2 pose=(...)` 의 시작 자세가 맞는지,
   `stack.diagnostics()`의 `tf_misses`가 0 근처인지.
6. **구조물 변경**: 로봇이 주행 중 박스를 경로 위로 옮겨 본다 → 1 s 안에 우회, 박스를 치우면 직선 복귀하는지.
   (안 비워지면 `obstacle_layer`의 `raytrace_max_range`/`clearing` 확인.)

## 튜닝 포인트
| 증상 | 손볼 곳 |
|---|---|
| 40 cm 통로에서 자주 멈춤/중단 | `inflation_radius`(0.30→0.27), `cost_scaling_factor`, footprint 실측 |
| 느림 | `desired_linear_vel`(0.35), `use_cost_regulated_linear_velocity_scaling` |
| 박스 근처에서 진동 | `lookahead_dist`, `rotate_to_heading_min_angle`, 전역 `update_frequency` |
| 목표 근처에서 계속 도는 | `xy_goal_tolerance`(0.08), `maicon26` 쪽 허용오차(`goal_tol`, 거점 0.10) |
| SLAM이 박스 이동 뒤 비틀림 | Cartographer `POSE_GRAPH.constraint_builder.min_score` 올리기 / `optimize_every_n_nodes`, 또는 `slam_toolbox.yaml`의 `do_loop_closing: false` |
| Gmapping을 쓰고 싶음 | `slam:=none` 으로 띄우고 직접 Gmapping 실행. `map→odom` TF만 나오면 Nav2 쪽은 그대로 |

## 알려진 한계
- `map` 프레임이 경기장과 회전/이동 관계라는 점을 **시작 한 번**의 벽 정합으로 정한다. 시작 자세 추정이 1°·1 cm 틀리면 그만큼의 고정 오프셋이 생긴다(시뮬 0.5 cm 수준).
- SLAM이 크게 틀어지면(루프 클로저 오류) 경기장 좌표 위치가 따라 틀어진다. 필요하면 주기적으로 벽 재정합(앵커 보정)을 추가해야 한다.
- 지도 원점에 회전이 있는 SLAM 지도는 가정하지 않는다(`grid_source: slam_map` 사용 시에만 해당).

## LiDAR 자기 가림 (실기에서 확인할 것)
팬틸트 기둥·안테나가 LiDAR 평면에 들어오면 그 방향이 항상 '아주 가까운 장애물'로 읽혀 전방 정지가 영구 지속된다. maicon26 은 자기가 받는 스캔에 `lidar.self_mask_sectors` 마스크를 걸지만, **Cartographer 와 Nav2 비용지도는 원본 `/scan` 을 직접 구독하므로 같은 각도를 ROS 쪽에서도 걸러야 한다**(예: `laser_filters` 의 `LaserScanAngularBoundsFilter`/`ScanShadowsFilter` 로 필터링한 토픽을 쓰거나, 드라이버 설정에서 해당 각도 제외). 각도는 로봇이 제자리에서 돌 때 `WARN LiDAR 자기 가림 의심` 로그가 후보를 알려 준다.

외부 패키지 없이 같은 마스크를 ROS 쪽에도 거는 작은 노드가 있다: `ros2/scripts/scan_self_mask.py` 는 `/scan` 을 받아 `lidar.self_mask_sectors` 와 같은 `SelfMask.apply` 로 지운 `/scan_filtered` 를 낸다(지운 광선은 NaN, header·각도·range·QoS는 입력과 같다). 켜는 방법은 `scan_self_mask:="170:15,-170:15"`(중심deg:반폭deg), `scan_self_mask_range`(기본 0.35), `scan_min_range`(기본 0) launch 인자이고, `maicon26.launch_profile.nav_launch_command` 가 설정(`lidar.self_mask_sectors/self_mask_range/min_range`)에서 이 인자를 만든다. 마스크가 비어 있으면 노드도 인자도 없고 SLAM·Nav2 는 지금처럼 `/scan` 을 쓴다. Agent 는 계속 원본 `/scan` 을 받아 자기 마스크를 쓰므로 두 번 걸리지 않는다. **아직 로봇에서 돌려 보지 못했다** — 처음 켤 때 `ros2 topic hz /scan_filtered` 와 RViz 에서 기둥 방향이 사라졌는지 본다.
