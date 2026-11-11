# 대회 UGV 하드웨어 연결 계약

검토 기준: `prestudy(2)_2026MAICON.pdf` p9–18, Waveshare `ugv_ws`의 `ros2-humble-develop` 커밋 `f0b3ad9c16e7acb6d6145f261116d71bf6338f4a`.

## 제공 프로그램과 참가팀 프로그램

제조사 저장소는 UGV 드라이버·URDF·바퀴 오도메트리·IMU·EKF·SLAM/Nav2 예제를 제공한다. `bringup_imu_ekf.launch.py`는 `/odom/odom_raw`의 좌우 이동량을 `/odom_raw`로 변환하고 `robot_localization`으로 `/imu/data`와 융합하여 `/odom`을 만든다. EKF는 2D 모드이며 IMU의 yaw/각속도를 사용한다. IMU 가속도를 두 번 적분해 위치를 만드는 설정이 아니다. `base_node_ekf.cpp`의 회전량 계산 분모 0.175 m는 제조사 구동계 값으로, 차체 외곽 폭을 그대로 넣는 값이 아니다.

대회 로봇의 `/mil/*`는 제조사 저장소에서 찾지 못한 별도 대회 인터페이스다. 참가팀 코드는 이를 사용하며 제조사 시리얼 드라이버를 직접 실행하거나 대회 제어 잠금을 우회하지 않는다. 자료에 커스텀 메시지 패키지명이 없으므로 `RosMilTransport`가 ROS 그래프에서 타입을 찾고 `hal.type_override`로 명시할 수도 있다.

| 입력/출력 | 참가팀 처리 |
| --- | --- |
| `/odom` | BEST_EFFORT 센서 QoS 수신. 기본 0 자세와 실제 수신 구분. 메시지 child 프레임이 설정과 다르면 TF로 변환. |
| `/scan` | 메시지 시각의 센서→차체 TF로 반사점 변환 후 각도 격자로 재배치. 같은 칸은 가까운 반사점 유지. TF 없거나 비수평 스캔은 사용하지 않는다. |
| `/image_raw` | BGR 영상, 최대 10 FPS 처리. 팬틸트 위치는 피드백이 없는 추정값. |
| 깊이 Image / CameraInfo | 실기에서 선택 활성화. 보정 확인, 영상·CameraInfo 해상도/광학 프레임 일치, 시각별 TF와 장착 모델 일치가 모두 확인된 프레임만 사용. 두 구독 모두 BEST_EFFORT 센서 QoS. |
| `/imu/data` | 자세/가속도에서 기울기·충격 상태 계산. 입력 단위는 ROS 표준 SI. |
| `/mil/state`, `/mil/match` | TRANSIENT_LOCAL depth 1. 실제 상태를 받기 전에는 구동·팬틸트·발사 불허. standby 상태 수신은 준비 조건 충족이며 구동 허용과 다르다. |
| `/cmd_vel` | 단독 발행, 약 10 Hz 반복. 제어 루프 정체·경기 잠금·종료 시 0. Nav2는 별도 토픽으로 리맵한다. |
| `/mil/gimbal` | 내부 degree→API 단위 변환, pan 부호 반전. 늦은 예전 요청 응답은 새 요청/경기 상태를 덮어쓰지 않는다. pause 시 추정 서보 위치도 정지. |
| `/mil/fire` | fired 응답만 성공. 제한 시간 초과는 성공으로 간주하지 않으며 미확정 상태에서 반복 발사하지 않는다. 지연 응답은 상태를 갱신하지만 pause 뒤 발광 상태를 되살리지 않는다. |

## 준비 상태 확인

`robot.readiness()`는 `(준비됨, 부족한 항목 목록)`을 돌려준다. LiDAR, 오도메트리, RGB 영상, IMU의 최근 수신(기본 1초), 상태의 최근 수신(2초), 경기 모드 수신, 팬틸트/발사 서비스 가용성, OLED 타입 발견을 확인한다. 실기에서 보정 검증 모드로 깊이를 활성화했다면 최근 1초 내에 검증된 깊이도 필요하다. 깊이를 끈 실기 프로필과 보정 검증을 요구하지 않는 시뮬레이션에는 이 추가 조건이 없다. 이 시간 기준은 명세의 보장값이 아닌 점검용 값이며 현장 주기에 맞춰 검증해야 한다.

실제 표본이 없는 `Pose2D(0,0,0)`를 준비 완료 근거로 쓰지 않는다. 제어권 허용 전의 준비 상태 확인을 위해 `control=True` 자체는 요구하지 않는다. 최종 `hal.cmd_topic`의 발행자 수가 자체 HAL을 포함해 2개 이상이면 준비 실패로 보고한다. Nav2의 중간 `/cmd_vel_nav2` 토픽은 이 제한 대상이 아니다. 이는 시작 전 검사이며 실행 중 새 발행자 출현을 지속 감시하는 기능은 아니다.

## TF와 남은 실기 확인

- 제조사 EKF는 `odom→base_footprint`를 발행한다. 참가팀의 추가 odom TF 브리지는 끈다. 해당 브리지는 입력 프레임을 다른 이름으로 덮어쓰지 않고 불일치를 거부한다.
- `Scan` 내부 모델은 차체 원점의 2D 극좌표 격자다. 센서 오프셋 변환은 반사점을 재배치하는 근사이며 가려진 공간을 새로 관측한 것은 아니다. 원본 `/scan`을 변경하지 않으므로 ROS SLAM은 원래 센서 TF를 사용한다.
- 회전·평행이동 TF와 스캔 시각이 맞아야 한다. 현재는 수평 LiDAR 평면만 지원하며 1도 넘는 roll/pitch TF는 명시적으로 거부한다. 대회 장착이 역방향/기울어진 형태면 변환 모델 검증 후 확장한다.
- 카메라 K/팬틸트 지연·속도/깊이 카메라 장착/바퀴 슬립/차체 footprint는 실측 필요하다. 제조사 예시 URDF나 0.175 m 값이 대회 개조 차체에 정확하다고 가정하지 않는다.
- 테스트는 가짜 전송과 합성 센서 입력에 대한 회귀 검증이다. 실제 ROS 서비스 타입, 대회 Docker의 노드 구성, 실제 로봇 주행·발사는 검증되지 않았다.

## OAK 깊이 활성화 계약

실기 프로필은 깊이와 3D 복셀 처리를 기본으로 끈다. Waveshare 5장 예제는 OAK의 RGB·깊이·LiDAR를 RTAB-Map에 연결하는 별도 SLAM 예제다. 현재 참가팀의 `DepthFusion`은 깊이를 내부 복셀에 누적하는 기능이며 그 예제를 실행한다는 뜻이 아니다. 제조사 예제의 토픽 이름만으로 실제 대회 로봇의 깊이 정렬 방식과 보정 상태를 확정할 수 없다. upstream OAK 설정에는 RGB 정렬 여부가 명시되지 않아 실제 메시지를 확인해야 한다.

깊이를 사용하려면 실제 `Image`와 대응 `CameraInfo`를 확인하고, 실측한 `rig.depth_xyz`, `rig.depth_rpy`, `rig.base_z`를 반영한 다음 `depth.calibration_verified=true`를 설정한다. `depth.optical_frame`은 두 메시지의 실제 `header.frame_id`를 입력한다. `depth.require_calibration=true`를 유지한다. 광학 프레임 이름이나 장착값을 코드가 추측하거나 TF에 맞춰 자동 변경하지 않는다.

1. `CameraInfo`의 폭·높이·K/P를 검사한다. 초점거리 두 축은 양수여야 하며 행렬에 NaN/Inf가 없어야 한다. Image와 폭·높이·광학 프레임이 정확히 같아야 한다. 실기 검증 모드에서는 다른 해상도의 K를 재사용하거나 추정 HFOV로 대체하지 않는다.
2. Image 촬영 시각의 `base_frame ← optical_frame` TF를 조회한다. 0 시각을 최신 TF로 대신하지 않는다. 동일 원본 시각의 반복 프레임은 신선도를 갱신하지 않으며, 시각이 역행하면 폐기한다. TF가 없거나 위치/쿼터니언이 유한하지 않거나 쿼터니언 길이가 1에서 0.001 넘게 벗어나면 폐기한다.
3. TF를 `RigModel`의 광학 축 회전까지 포함한 변환과 비교한다. `base_footprint`는 바닥에서의 `base_z`를 포함하고 `base_link`는 제외한다. 그 밖의 기준 프레임은 아직 지원하지 않는다. 기본 허용 차이는 위치 0.02 m, 회전 2도이며 실측 결과에 맞춰 `depth.tf_tolerance_m`, `depth.tf_tolerance_deg`를 검토한다.
4. 잘못된 CameraInfo가 오면 이전 K와 깊이 프레임을 함께 제거한다. TF/영상 검증 실패도 이전 깊이 프레임과 준비 상태를 제거한다. 전체 픽셀이 무효이거나 `depth.min_range`–`depth.max_range` 안에 유효 깊이가 하나도 없으면 검증 실패다. 임의의 유효 픽셀 비율 문턱은 두지 않는다. 원인은 `depth_info_error`, `depth_tf_error`, `depth_error`에 남는다.
5. 보정 검증 모드로 깊이를 켠 동안 깊이 데이터가 사라지거나 1초 넘게 오래되면 최종 속도 발행에서 선속도·각속도를 모두 0으로 만든다. 잘못된 새 데이터가 들어오면 즉시 0을 발행한다. 복구된 검증 프레임과 유효한 주행 명령이 있어야 정상 이동할 수 있다. 깊이를 사용하는 전제로 출발한 뒤 조용히 2D만으로 계속 주행하지 않도록 한 제한이다.

TF와 설정이 일치한다는 것은 두 소프트웨어 표현의 일관성 검사다. 실제 장착 위치, 깊이 거리 오차, RGB 정렬 여부, 타임스탬프 지연이 정확하다는 실측 증명은 아니다. `StampSync`의 시계 차 추정만으로 촬영 지연을 정확히 분리할 수 없으므로 센서 시각 동기와 최소 지연은 현장에서 측정해야 한다. 카메라가 다른 위치에 고정되어 있거나 팬틸트와 함께 움직이면 현재 고정 깊이 모델을 그대로 사용할 수 없다.

Waveshare 1장의 `app.py` 종료·시작 설정 변경은 일반 제품 환경 준비 절차다. 대회에서는 제공 프로그램의 소유권을 먼저 확인하고, 이 코드는 해당 프로세스를 자동 종료하지 않는다. 2장의 URDF 표시 자체는 모델 확인이지만 `ugv_driver`와 함께 실행하는 Joint State Publisher GUI 조작은 실제 팬틸트를 움직일 수 있다. 대회에서는 `/mil/*` 제어 계약을 우선한다.

## 근거

- [Waveshare EKF 실행](https://github.com/waveshareteam/ugv_ws/blob/f0b3ad9c16e7acb6d6145f261116d71bf6338f4a/src/ugv_main/ugv_bringup/launch/bringup_imu_ekf.launch.py)
- [EKF 설정](https://github.com/waveshareteam/ugv_ws/blob/f0b3ad9c16e7acb6d6145f261116d71bf6338f4a/src/ugv_main/ugv_bringup/param/ekf.yaml)
- [바퀴 오도메트리](https://github.com/waveshareteam/ugv_ws/blob/f0b3ad9c16e7acb6d6145f261116d71bf6338f4a/src/ugv_main/ugv_base_node/src/base_node_ekf.cpp)
- 대회 2차 사전교육 p10–17: 토픽·서비스·QoS·단위·잠금·주기 계약.
- [Waveshare 1장 환경 준비](https://www.waveshare.com/wiki/UGV_Beast_Jetson_Orin_ROS2_1._Preparation)
- [Waveshare 2장 RViz 모델 표시](https://www.waveshare.com/wiki/UGV_Beast_Jetson_Orin_ROS2_2._RViz_View_Product_Model)
- [Waveshare 5장 깊이 카메라 3D 지도](https://www.waveshare.com/wiki/UGV_Beast_Jetson_Orin_ROS2_5._3D_Mapping_Based_on_Depth_Camera)
- [제조사 RTAB-Map RGB-D 실행 코드](https://github.com/waveshareteam/ugv_ws/blob/f0b3ad9c16e7acb6d6145f261116d71bf6338f4a/src/ugv_main/ugv_slam/launch/rtabmap_rgbd.launch.py)
