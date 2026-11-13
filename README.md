# TakeTheLoss · MAICON 2026 실기 코드

대회장에 가져갈 실기 코드만 모은 폴더다. `MAICON2026/maicon2026_ugv_code/maicon2026`에서 2026-11-13에 복사했고, 파일 내용은 원본과 같다(170개, SHA-256 대조). 대회 자료는 팀 밖으로 공유하지 않는다.

## 구성

| 위치 | 내용 |
|---|---|
| `maicon2026/team/team_node.py` | 대회 진입점. 운영 프로그램이 경기 대기 때 실행한다 |
| `maicon2026/scripts/run_real.py` | 실기 임무 실행기 (`team_node.py`가 부른다) |
| `maicon2026/maicon26/` | 임무·항법·인식·HAL·서버 통신 코드. 실기 진입점과 현장 도구가 import하는 파일만 담았다 |
| `maicon2026/config/` | `waveshare_real.json`(실기 설정), `field_overrides.json`(현장 실측값이 쌓이는 곳), `robot_calibrated.json`, `oak/` |
| `maicon2026/ros2/` | SLAM·Nav2 launch, Cartographer·Nav2 설정, BT, LiDAR 자기 가림 필터, URDF, `arena_spec.yaml` |
| `maicon2026/scripts/` | 현장 도구: `field_exp.py`, `uav_boxes_tool.py`, `calibrate_uav*.py`, `calib_*.py`, `check_command.py` 등 |
| `maicon2026/tools/` | 실기 센서 점검 도구와 OAK 점검 노트북 |
| `maicon2026/calib/`, `maicon2026/field/` | 로봇 프로파일·보정 산출물, 코드 기본값 기준 현장 체크리스트 |
| `실측_체크리스트.md` | 실기 설정 기준으로 뽑은 실측·확인 값 288개 |
| `teleop_rosbag/` | 키보드 텔레옵, rosbag 기록·점검, UAV 영상 수집 도구. 2026-11-13에 새로 만들었고 실제 로봇에서는 아직 돌려 보지 않았다 |

## 실행

로봇에서 ROS 2 Humble 환경을 불러온 뒤 실행한다. 서버 주소와 임무코드는 환경변수로 넣고 이 폴더에 저장하지 않는다.

```bash
cd maicon2026
python3 scripts/run_real.py --preflight-only          # 센서·TF·서비스만 확인. 서버 요청·이동·발사 없음
MAICON_SERVER_URL=http://<서버> MAICON_DETECTOR=<가중치 경로> TEAM_LAUNCH_NAV=1 python3 team/team_node.py
```

- `TEAM_LAUNCH_NAV=1`이면 SLAM·Nav2 launch를 함께 띄운다. `MAICON_OAK=1`이면 OAK 드라이버도 띄운다.
- `config/field_overrides.json`이 있으면 자동으로 읽는다.
- 필요한 것: `numpy`, `opencv-python`(어안 보정은 4.x), `pyyaml`, `ultralytics`, ROS 2 Humble의 `rclpy`·Nav2·Cartographer.

## 여기 없는 것

- YOLO 가중치: 원본 저장소에도 없다. 따로 가져와 `MAICON_DETECTOR`로 지정한다.
- `calib/uav_main_calibration.json`: 현장에서 UAV 영상으로 만든다.
- 시뮬레이션·Gazebo(경기장 패키지, 로봇 모델, 자가 대전 실행기), 시험 코드(`tests/`), 녹화·결과(`results/`), 설계 문서(`docs/`), 작업 백업(`_hw_work/`)은 옮기지 않았다. `MAICON2026`에 그대로 있다.
- `maicon26/sim/`에 7개 파일이 남아 있다. 실기 코드가 고정 시설 좌표(`sim/fixed_maicon.py`)와 현장 도구의 드라이런에 쓰기 때문이다.
