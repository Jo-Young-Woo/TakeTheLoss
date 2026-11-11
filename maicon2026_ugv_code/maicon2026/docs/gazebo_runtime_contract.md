# Gazebo 실행 계약과 검증 범위

2026-11-09 갱신. 센서·좌표 계약과 실제 Gazebo 녹화의 검증 범위다. 실물 로봇 시험은 별도로 필요하다.

## 현재 시뮬레이터와 Waveshare 예제의 차이

| 구분 | 확인한 코드 | 실행 방식 |
| --- | --- | --- |
| Waveshare `ugv_ws/ros2-humble-develop` | `ugv_gazebo/launch/bringup/bringup.launch.py`, `spawn_ugv.launch.py` | `gazebo_ros`, `gzserver.launch.py`, `gzclient.launch.py`, `spawn_entity.py`: Gazebo Classic 경로 |
| 현재 프로젝트 | `scripts/gazebo_nav_demo.sh`, `generate_gazebo_world.py` | `ign gazebo`, `ros_gz_bridge`, `ignition::gazebo::systems::DiffDrive`: Gazebo Fortress 경로 |

Waveshare 참조 커밋은 `f0b3ad9c16e7acb6d6145f261116d71bf6338f4a`다. 위키 11장 원문을 확보하지 못한 상태에서 그 장의 세부 동작을 확인했다고 주장하지 않는다. 위 구분은 저장소의 실제 launch 파일과 현재 스크립트에 근거한다. 두 시뮬레이터의 플러그인·spawn 명령을 섞지 않는다.

`CONTROL=match`의 지상 카메라 분류, 레이저 판정, 경기 이벤트는 기존 Python `World`/`SimServer`가 합성한다. 물리 이동·LiDAR·오도메트리·시계·녹화 영상은 실제 Gazebo 입력이다. UAV `image` 모드의 구조물 위치는 실제 `/uav/image`에서 추출하며, 영상 제공자가 검출하지 않은 UGV 위치를 시뮬레이터 정답으로 보충하지 않는다. 따라서 임무 결과는 실제 YOLO나 실물 레이저·대회 서버의 정확도 검증이 아니다.

기본 LiDAR `sector160`의 160도는 로컬 렌더러 제약이다. `LIDAR_MODE=sectors360`은 세 개의 실제 120도 측정을 합쳐 전 방향을 복원한다. 자기 몸체에 가려진 빔은 미관측으로 유지한다. [D500 제조사 사양](https://www.waveshare.com/product/d500-lidar-kit.htm)은 360도다. 현재 WSL GPU의 잘못된 LiDAR 출력과 OGRE2 오류가 확인돼 센서 렌더링에는 검증된 CPU 설정을 사용한다. [실행·녹화 설명](gazebo_recording.md)을 참고한다.

## 센서와 시간

`GazeboRobot`은 `/clock`, `/scan`, `/odom`을 `qos_profile_sensor_data`로 구독한다. BEST_EFFORT 센서 발행자와 RELIABLE 발행자 모두 연결 가능한 구독 정책이다.

`ready()`는 다음을 함께 검사한다.

- 세 토픽이 실제로 수신됐고 `/clock > 0`이어야 한다.
- LiDAR와 오도메트리의 header 시각이 현재 시뮬레이션 시각보다 0.75초 이상 오래되지 않아야 한다. 콜백 도착 순서를 위해 최대 0.1초의 미래 시각을 허용한다.
- 세 토픽의 마지막 수신 이후 벽시계 기준 2초가 지나지 않아야 한다. 시뮬레이터가 멈추거나 브리지가 끊겨도 오래된 값이 준비 완료로 남지 않는다.
- 시계가 역행한 실행은 센서가 다시 들어와도 준비 완료로 복구하지 않는다. Agent와 referee까지 새 실행으로 다시 시작해야 한다.

한계값은 `GazeboRobot(max_sensor_age=.75, max_receive_age=2.0)`으로 조정할 수 있다. 속도가 빠른 시뮬레이션에서 허용 지연을 바꾸면 실제 sim-time 지연을 함께 확인해야 한다.

오래된 입력에서는 `get_scan()`이 `None`을 돌려주고 새 이동·발사 요청을 거부한다. 0.1초 주기의 벽시계 watchdog은 마지막 이동 명령도 정지시킨다. `get_odom()`은 기존 HAL 형식 때문에 마지막 자세를 보존하지만, 그 값이 현재 유효하다는 뜻은 아니다.

`run_gazebo_match.py`는 시계가 증가할 때만 Agent·상대 로봇·경기 판정을 갱신한다. `/clock`이 그대로면 로봇을 정지시키고 대기하며, 반복 횟수로 0.001초씩 경기 시간이 증가하던 문제를 제거했다. 시계 역행·비유한 시각·진행 중 센서 신선도 상실은 로봇을 정지시키고 실행을 실패 처리한다. 긴 일시정지 후 재개 시 센서가 아직 복구되지 않았다면 재시작이 필요하다. 기존 벽시계 실행 제한은 유지한다.

## LiDAR 좌표와 이전 SDF 이전

현재 SDF의 LiDAR 위치는 `base_link` 기준 `(0.039815, 0, 0.16)m`, 회전은 `(0, 0, 0)rad`다. 이는 **현재 시뮬레이터 모델의 좌표**이며 실물 센서 측정값이 아니다. `maicon26/hal/gazebo_model.py`에 기준값을 모았다.

기존 SDF는 이 위치에서 측정하면서 `LaserScan.header.frame_id=base_link`라고 보냈다. SLAM이 센서를 차체 중심에 있다고 해석하므로 약 4cm의 수평 원점 오차가 생겼다. 이제 두 경로를 일치시킨다.

1. ROS `/scan`: 원래 센서 좌표 그대로 `gazebo_lidar`로 발행한다. 데모 스크립트가 `base_link → gazebo_lidar` 정적 TF를 위 위치로 한 번 발행한다. SLAM/Nav2는 TF로 변환한다.
2. Agent의 내부 `Scan`: Gazebo HAL이 같은 위치를 사용해 측정 끝점을 차체 기준으로 변환한다. 사선의 각도와 거리도 다시 계산하며, 관측하지 않은 방향은 NaN으로 유지한다. ROS 토픽의 값을 변경하지 않으므로 SLAM에 변환이 두 번 적용되지 않는다.

여섯 로봇 실험의 다른 로봇은 `R1/gazebo_lidar`처럼 프레임을 구분한다. 현재 Nav2·전역 TF는 주 로봇 B2용 한 세트이고, 다른 로봇 HAL은 각 센서를 자체 차체 기준으로 변환한다. 다중 로봇 Nav2 구성이 완성됐다는 뜻은 아니다.

**기존에 저장해 둔 SDF는 자동 변경하지 않았다.** 다음에 실험을 시작할 때 새 generator로 새 파일을 생성하고 수정된 `gazebo_nav_demo.sh`를 사용해야 한다. `base_link`로 라벨된 이전 scan은 HAL이 설명 메시지와 함께 거부한다. 데모를 사용하지 않고 수동 실행한다면 다음 TF가 필요하다.

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.039815 --y 0 --z 0.16 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id gazebo_lidar
```

데모가 발행하는 TF와 위 명령을 동시에 실행하지 않는다. 현재 Fortress 데모는 `robot_state_publisher`를 띄우지 않으며 `gazebo_lidar`는 실물 URDF의 센서 프레임과 구분한 이름이다. 나중에 URDF로 통합한다면 TF 소유자를 하나로 정해야 한다.

## 이번에 확인한 것

```powershell
python -m pytest -q -p no:cacheprovider tests/test_gazebo_contract.py tests/test_gazebo_fixed.py tests/test_ros_hardware_launch.py
```

ROS 없이 QoS 선택, 벽시계·센서 시각 신선도, 시계 정지·역행, 출력 차단, 이전 SDF 거부, SDF/정적 TF 수치 일치, 기존 고정 경기장 조건을 검사한다. WSL에서는 스크립트의 `bash -n` 구문 검사만 수행한다. 실제 브리지 발행값, SLAM 정확도, 일시정지 후 재개, watchdog 정지 지연과 경기 성공률은 별도 Gazebo 실행에서 확인해야 한다.

수정 전 네 파일은 워크스페이스 루트의 `review_2026-11-07/gazebo_before/`에 복사했다. 기존 `ros2/gazebo` world·asset 파일과 `tests/test_gazebo_fixed.py`는 수정하지 않았다.
