# WINGBOT 모델 검증 결과 — 2026-11-09

제공된 v2 YAML 수치는 바꾸지 않았습니다. 원본 UGV Rover 메시를 바탕으로 실제 Fortress에서 모델 A–D를 검증했습니다. 기존 임무 알고리즘의 성공률을 검증한 결과는 아닙니다.

| 검사 | 결과 | 실측 |
|---|---|---|
| A 수치 | 통과 | 외곽 254.96 × 240.44 × 281.08 mm, 2.9 kg, 6륜, 충돌 9개, 추가 삼각형 2,792개 |
| B 사진 | RED·BLUE 통과 | 하부 IoU 0.94750, 실제 TF 12/12·RGB 12/12, 보조선 0픽셀 |
| C 주행 | 통과 | 0.3 m/s × 5 s = 1.499890 m, 1 rad/s × 3 s = 2.999473 rad, 양·음 속도 제한 통과 |
| D 자아 1대 full | 통과 | RTF 1.000030 / 기준 0.98 |
| D 자아 1+더미 5 standard | 통과 | RTF 0.999714 / 기준 0.98 |
| D 자아 1+더미 5 lite | 통과 | RTF 1.000003 / 기준 0.90 |

D는 각각 60초 동안 GUI 없이 CPU Ogre2 센서 서버에서 측정했습니다. 6대 구성에는 UAV 카메라도 포함했습니다. standard/lite 모두 실제 UGV 6대와 자아 RGB 640×480·10 Hz, LiDAR 450점·10 Hz, IMU 20 Hz 수신을 확인했습니다. GUI 프레임률이나 GPU 렌더링 성능으로 해석하지 않습니다.

## 사진 기준점 오차

고정된 1057×1054 카메라에서 얻은 RED 결과입니다. RGB 관측기는 TF 정답을 받지 않고 실제 영상의 부품 경계·색을 측정합니다. BLUE도 같은 검사에 통과했습니다.

| 기준점 | 실제 TF 투영 오차(px) | 실제 RGB 관측 오차(px) | 기준(px) |
|---|---:|---:|---:|
| laser_lens_center | 2.75 | 5.07 | 12 |
| camera_lens_center | 0.68 | 0.47 | 12 |
| camera_led | 8.61 | 9.22 | 12 |
| headlight_left(+y) | 3.91 | 2.96 | 12 |
| headlight_right(-y) | 4.10 | 5.06 | 12 |
| receiver_left(+y) | 16.92 | 18.61 | 20 |
| receiver_right(-y) | 11.16 | 11.06 | 20 |
| iff_left_tip | 5.81 | 5.00 | 12 |
| iff_left_sleeve_top | 4.06 | 9.00 | 12 |
| iff_left_sleeve_bottom | 5.49 | 7.81 | 12 |
| depth_camera_front_ctr | 5.86 | 6.03 | 12 |
| lidar_top_center | 6.51 | 7.36 | 12 |

[사진과 Gazebo 비교](model_vs_pdf.png) · [윤곽 겹침](outline_on_photo.png) · [BLUE](model_blue.png) · [정사영 진단](ortho.png) · [사진 검증 방법](photo_validation_notes.md)

## 원본 분석과 구현 해석

- 원본 외곽 0.251000 × 0.230420 × 0.253629 m, 질량 3.917124 kg, 바퀴 4개·지름 0.080 m를 확인했습니다. 기존 Gazebo의 wheel_diameter 0.05 오류를 Fortress wheel_radius 0.040으로 수정했습니다.
- 원본 팬 양수는 오른쪽, 틸트 양수는 위입니다. 링크·관절 이름은 유지했습니다. 카메라 CAD 프레임은 렌즈 중심이 아니며 부모가 회전되어 있어 local-z −6.5 mm를 그대로 적용하면 옆으로 이동합니다. YAML의 물리 렌즈 좌표로 재기준화한 예외를 README와 frames.yaml에 기록했습니다.
- LiDAR 장착 프레임 z=0.110 m와 yaw=+π/2를 보존했습니다. 별도 광학 프레임은 원본 Gazebo 센서 pose +0.040 m를 따라 z=0.150 m입니다. 광학 평면은 현장 실측값이 아닙니다. 원본 드라이버 각도 반전을 반영해 raw 225–315°를 에뮬레이터에서만 자릅니다.
- 바퀴 자전에 따라 마찰 방향이 바뀌던 문제를 차축 방향 기준으로 고쳤습니다. 횡 마찰 0.03, 진행 마찰 0.8, effective_separation 0.1745는 시뮬레이션의 공학적 설정입니다.

## 검증의 범위와 남은 현장 확인

직진 거리·회전각의 바퀴 odom 오차는 최대 0.102%입니다. 별도로 제자리 3 rad 회전 중 중심 이동 4.29 cm, 고속 회전 중 6.44–6.97 cm가 관측됐습니다. 바퀴 odom은 이 횡방향 미끄러짐을 표현하지 않습니다. 위치 추정 전체가 0.102% 정확하다는 뜻은 아닙니다.

경기장에서는 규정된 90° crop 이후에도 약 3.2 cm의 자기 반사가 남습니다. 최종 단일 로봇 실측에서 450점 중 112점이 crop되고, 차체 각도 131.29–134.50°와 225.10–228.31°에 가까운 반사 10점이 남았습니다. 사진 형상대로 만든 마스트 기둥의 예상 위치와 일치합니다. [원시 진단](../validation/default_smoke/smoke.json)에 각도와 거리를 남겼습니다. 모델 외형이나 규정 crop을 검사 성적에 맞춰 없애지 않았습니다. 기존 주행 알고리즘에 연결할 때는 이 자기 반사와 카메라 높이/내부 파라미터를 별도로 보정해야 합니다. 이번 작업에서 mission/nav/perception/link 전략을 수정하거나 임무 성공을 주장하지 않았습니다.

제공 YAML 가정: camera.hfov_rad, 카메라 렌즈 y=0, lidar.range/samples/rate_hz, iff_antennas 대칭·lean 및 사진 280 mm와 사양 320 mm의 차이. 런타임 가정: k1=-0.25, 광학 클립 거리, 선택 깊이 센서 사양, 마찰·접촉·관절 제어 이득. [현장 측정표](field_measure.md)로 교체하고 A·C를 다시 검사하세요.

주행 측정은 시뮬레이터가 찍은 PoseStamped를 odom 시각에 보간해 비교했습니다(최대 보간 간격 12 ms, 외삽 금지). 외부 위치를 주행 제어기에 전달하지 않았습니다. [Fortress PosePublisher 구현](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/pose_publisher/PosePublisher.cc)의 모델 발행 조건과 timestamp 처리를 확인했습니다.

## 재현과 원시 자료

```bash
source /home/jojo0429/wingbot_model_ws_20261109/install/setup.bash
ros2 launch maicon_arena_gz arena.launch.py gui:=false image_transport_profile:=shm64
# 기본: WINGBOT 1대. 6대는 ugvs:=6, 이전 단순 모델은 robot_model:=legacy.
```

- [A 원시 결과](../validation/gate_a.json)
- [B RED](photo_validation.json), [B BLUE](photo_validation_BLUE.json)
- [C 시간 정렬 실측](../validation/drive_v4/gate_c.json)
- [D 1대](../validation/perf_solo_final/gate_d_solo.json), [D standard](../validation/perf_arena_standard/gate_d_arena.json), [D lite](../validation/perf_arena_lite/gate_d_arena.json)
- [전체 모델 게이트 요약](../validation/gate_summary.json)

최종 집중 회귀 검사 **219개 통과**(18.22초), Python 정적 검사 오류 0개, 세 ROS 패키지 빌드 통과를 확인했습니다. [검사 로그](../validation/focused_tests.log)를 보관했습니다. 기본값으로 다시 실행해 실제 모델이 ugv_B2 한 대이며 RGB·UAV 영상과 LiDAR가 수신되는 것도 확인했습니다. [1인칭 화면](../validation/default_smoke/front.png) · [UAV 화면](../validation/default_smoke/uav.png).

이전 전체 1,352개 테스트 및 이전 영상은 단순 모델의 과거 결과입니다. 이번에 전체 1,352개를 다시 실행했다고 주장하지 않습니다. 대회 자료와 사진은 로컬에만 보관했습니다.
