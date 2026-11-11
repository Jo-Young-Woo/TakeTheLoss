# WINGBOT description

2026-11-09 실제 Fortress 모델 검사 A–D를 모두 통과했습니다. [정량 결과·12개 기준점·성능·측정 한계](docs/model_validation.md)와 [사진 비교](docs/model_vs_pdf.png)를 확인하세요. MAICON 경기장의 기본 모델은 WINGBOT 1대입니다. 기존 임무 알고리즘의 새 카메라·LiDAR 자기 가림 보정 및 임무 성공률은 이번 모델 검사 범위에 포함하지 않습니다.

Waveshare UGV Rover의 실제 STL을 사용한 MAICON WINGBOT 모델입니다. 차체·네 바퀴·팬틸트·D500·깊이 카메라 메시와 원본 링크/관절 이름을 보존하고, 가운데 두 바퀴와 제공된 v2 사진 복원 부품을 추가했습니다. 대회 사진은 실측 CAD가 아닙니다. 원본 입력 `config/wingbot_mods.yaml`과 `config/photo_fit/`은 제공 파일을 바이트 그대로 복사했습니다.

```bash
python3 ros2/wingbot_description/scripts/gen_from_yaml.py
colcon build --packages-select wingbot_description
source install/setup.bash
ros2 launch wingbot_description view.launch.py team:=RED
```

ROS 워크스페이스는 공백 없는 Linux 경로에서 빌드하세요. Gazebo 실행 중에는 모델 생성기를 다시 실행하지 마세요. 외부 Fuel 다운로드는 필요하지 않습니다. 패키지 환경 hook이 로컬 메시 검색 경로를 추가합니다.

기존 Fortress 월드에 추가하려면 `ros2 launch wingbot_description spawn.launch.py world:=default team:=BLUE slot:=B-2 x:=0 y:=0 yaw:=0`을 사용합니다. 이 launch는 시뮬레이터를 새로 시작하지 않습니다. 사진 시험 launch는 `photo_view.launch.py`입니다. A–D 시험 통과 전 경기 시뮬레이션 결과를 주장하지 않습니다.

## 모델과 토픽

`urdf/wingbot.urdf.xacro` 인자는 `model_name`, `team=BLUE|RED`, `prefix`, `namespace`, `sensors=full|none`, `depth=false|true`, `wheels=6|2`, `effective_separation`, `friction_lateral`, `friction_longitudinal`입니다. 기본 모델은 `wingbot`, 토픽은 `/wingbot/cmd_vel`, `/wingbot/odom`, `/wingbot/tf`, `/wingbot/joint_states`, `/wingbot/pan/cmd_pos`, `/wingbot/tilt/cmd_pos`, `/wingbot/camera/image`, `/wingbot/camera/camera_info`, `/wingbot/scan`, `/wingbot/imu`입니다. namespace를 바꾸면 이 접두사가 바뀝니다. prefix는 TF 링크/관절 이름에만 붙습니다.

`wheels=6`은 여섯 바퀴를 구동합니다. `wheels=2`도 여섯 바퀴의 외관·충돌·질량을 보존하고 가운데 두 바퀴만 구동하며 나머지는 자유 회전합니다. `sensors=none`은 RGB/LiDAR/IMU/깊이 센서를 모두 제거하지만 관절 상태와 구동 플러그인을 유지합니다. 깊이 카메라 외관은 센서 활성화와 무관하게 남습니다.

물리 충돌은 차체 2 box, 머리 1 box, 바퀴 6 cylinder로 총 9개입니다. 추가 부품은 기본 도형을 16분할 원통·8×24 돔·작은 팔각기둥 OBJ로 시각화해 실제 파일 기준 2792 삼각형을 사용합니다. 모든 충돌은 native 기본 도형이며 메시 충돌, light, wideanglecamera는 없습니다. 질량은 YAML대로 합계 2.9 kg이며 기본 도형 공식 관성을 씁니다. `effective_separation=0.1745`는 물리 바퀴 간격과 별도로 주행 실측을 통해 보정할 파라미터입니다.

## 좌표 해석과 원본 차이

`scripts/wingbot_geometry.py`가 URDF parent-local FK를 계산합니다. `frames.yaml`과 `config/frames.yaml`은 같은 생성 결과이며 실제 joint tree, 0점 프레임, 레이저 +X, 수광 중심/반지름/방향, 안테나 끝, 사진 기준점의 frame/local 좌표를 포함합니다. 심판은 현재 관절 측정값으로 이 tree를 계산해야 합니다. 사진의 `model_xyz` 정답을 역으로 넣지 않습니다.

원본 `pt_link2`는 roll=1.5708이므로 카메라 관절의 **local z를 6.5 mm 내리면 세계 Y가 움직입니다**. 원본 카메라 관절 origin은 `[.047686,-.000569,.02]`, RPY `[3.14,-1.5708,0]`이며, 그 CAD 프레임은 렌즈 중심도 +X 전방도 아닙니다. 따라서 원본 이름과 부모는 보존하고 YAML의 실제 렌즈 `[.057,0,.2035]`와 세계 +X를 parent-local로 변환해 재기준화했습니다. 변환 수치는 `frames.yaml.camera_frame_reanchor`에 기록됩니다. 카메라 STL 대신 제공된 참고 형상의 보드·배럴·렌즈·LED를 사용합니다. 이는 YAML 숫자 수정이 아니라, 제공 지시의 local-z 약식 표현과 실제 회전된 CAD 프레임 사이의 충돌을 해결한 것입니다. 다른 원본 관절 origin은 D500 z −.010 외에 변경하지 않았습니다.

D500은 description URDF의 yaw +1.5708을 보존합니다. 별도 vendor Gazebo URDF의 yaw=0과 다릅니다. 원본 드라이버 `ugv_else/ldlidar/src/demo.cpp`는 clockwise raw 각 225–315°에서 crop 후 scan 순서를 뒤집으므로 ROS scan 각 45–135°가 대응하며, description yaw를 더하면 몸체 뒤쪽입니다. 이 소스 해석을 metadata에 기록합니다. 모델 센서는 360°를 모두 생성하며 crop은 에뮬레이터에서 적용합니다. 실기 yaw/scan 평면은 아직 현장 확인이 필요합니다.

원본 Gazebo SDF의 `hls_lfcd_lds` 센서는 장착 링크와 별개로 `pose=0 0 .04 0 0 0`을 사용합니다. 이 파일을 보존하고 생성기가 해당 pose를 읽어 `lidar_scan_link`를 추가합니다. 낮춘 장착 프레임 z=.110 + 원본 센서 오프셋 .040으로 실제 시뮬레이션 스캔 평면은 z=.150이며, 센서 메시의 상단 약 .145보다 위입니다. 장착 링크를 센서 평면으로 오인하면 deck 안에서 스캔하게 됩니다. `.150`은 vendor 설정을 보존한 공학적 선택이고 실기 D500 광학 평면의 실측값은 아닙니다. 사진 주석의 약 .13과 차이는 현장 확인 목록에 남깁니다. semantic.lidar, 센서 메시지 frame_id, 심판 metadata가 모두 `lidar_scan_link`를 사용합니다.

원본 직접 분석 결과: STL+원본 URDF FK 외곽은 `[.251000,.230420,.253629] m`, 질량 합계는 `3.917124 kg`, 바퀴는 4개입니다. `pt_link2`의 local +Z가 세계 −Y이므로 원본 tilt +가 위이고, pan 원본 −Z 축은 +명령이 오른쪽입니다. 원본 Gazebo DiffDrive의 wheel_diameter=.05는 STL 지름 .08과 다르므로 새 Fortress 모델은 radius=.04를 사용합니다. 전체 원본 비교는 생성물 A 검사와 별개로 수행했습니다.

## 가정과 검증

v2 YAML의 `camera.hfov_rad`, `lidar.range/samples/rate_hz`, 안테나 대칭/lean, 렌즈 y=0 등은 제공 자료에 표시된 가정입니다. 추가 런타임 가정은 카메라 k1=-0.25, near/far=.01/20 m, 선택 깊이 센서 320×240/5 Hz/hfov1.2, 횡/진행 마찰 .03/.8, 접촉 kp=100000/kd=100, 관절 제어 이득 6/0/.1 및 head 질량의 pan/tilt 2:5 분배입니다. 마찰 첫 방향은 바퀴의 local Z 차축으로 명시합니다. 이 축은 바퀴가 돌아도 변하지 않으며 local X를 쓰면 자전 중 방향이 위아래로 바뀝니다. 초기 미지정 방향의 .8/.3 조합에서 회전 미달이 관측되어 분리했고, 최종 성능은 실제 주행 보고서를 기준으로 판단합니다. 동적 성능은 실제 Gazebo 측정으로 확인합니다. 관절·충돌·외관 좌표를 실험 성적에 맞춰 조정하지 않습니다.

원본 사양 320 mm와 사진 복원 안테나 끝 280 mm 차이는 유지합니다. [현장 측정표](docs/field_measure.md)를 사용해 확인하세요. 라이선스 근거는 [vendor/LICENSE_AUDIT.md](vendor/LICENSE_AUDIT.md)입니다. 모델 A–D의 실제 결과와 사진 비교는 별도 검증 보고서에 기록합니다.

### 실제 주행 C — 2026-11-09 v4

[`validation/drive_v4/gate_c.json`](validation/drive_v4/gate_c.json)의 CPU Ogre2 빈 월드 시험은 통과했습니다. 횡/진행 마찰은 `.03/.8`, `fdir1=0 0 1`, `effective_separation=.1745`이며 YAML 장착 치수는 그대로입니다. 100 Hz 실제 모델 위치를 odom 시각에 정렬해 비교했습니다. 명령 0.3 m/s × 5 s에서 실제 1.499890 m, 1 rad/s × 3 s에서 2.999473 rad였고, 양·음 명령의 ±0.6 m/s 및 ±3 rad/s 제한도 확인했습니다. 직선 변위/회전각에 대한 odom 상대오차는 최대 약 0.102%였습니다.

회전각 정확도와 별개로 3 rad 회전 중 중심이 4.29 cm, 고속 회전 제한 시험에서는 6.44–6.97 cm 이동했습니다. 바퀴 odom은 이 횡이동을 표현하지 못하므로 전체 위치 오차가 같은 수준이라는 뜻은 아닙니다. 이는 현재 6륜 접촉·미끄러짐 모델의 제한으로 보존합니다. C 통과를 근거로 물리 모델을 추가 변경하지 않았으며, D 성능과 B 사진 결과는 각각 별도 실제 측정으로 판정합니다.
