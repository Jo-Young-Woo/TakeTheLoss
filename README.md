# MAICON 2026 UGV — source code

이 저장소에는 MAICON UGV **후보 02 자율 임무 코드**와 Gazebo 경기장·ROS 연동을 위해 작성한 소스 코드만 담았습니다. 선택된 `maicon26/` Python 파일 122개의 내용은 검증 당시와 같습니다.

## 코드 찾기

| 경로 | 내용 |
| --- | --- |
| [maicon26/](maicon26/) | 후보 02의 임무, 탐색, 인식, 항법, 서버 연결 코드 |
| [scripts/](scripts/) | Gazebo 경기 실행·녹화·분석 도구 |
| [ros2/maicon_arena_gz/scripts/](ros2/maicon_arena_gz/scripts/) | 경기장 생성, 심판·센서 연결 코드 |
| [ros2/maicon_arena_gz/src/](ros2/maicon_arena_gz/src/) | Gazebo 렌더 재질 플러그인 소스 |
| [ros2/mil_interfaces_sim/](ros2/mil_interfaces_sim/) | 로컬 ROS 메시지·서비스 정의 |
| [team/](team/) | 팀 실행 진입점 |

주요 흐름은 **Gazebo 센서 → `maicon26/perception`·`nav` → `maicon26/mission` → 로봇 구동·경기 서버 연결**입니다. `scripts/run_wingbot_selfplay.py`가 6대 시험을 조정하고, `scripts/evaluate_wingbot_missions.py`가 로컬 채점 자료를 분석합니다.

## 소스 무결성 확인

`source_hashes.json`은 선택된 후보 02의 Python 소스 해시입니다. 아래 명령은 122개 파일의 현재 내용을 확인하며 Gazebo를 시작하지 않습니다.

```bash
python3 verify_source.py
```

## 공개 범위와 실행 제한

이것은 **코드 열람용 공개 스냅샷**입니다. 대회 PDF·API 원문, 사진·바닥·표적 텍스처, 제공받은 WINGBOT CAD 메시, 완성된 월드, 보정값, 녹화 영상·점수 자료와 비밀값은 포함하지 않았습니다. 따라서 `scripts/run_selected_gazebo.sh`와 ROS 패키지는 이 저장소만 받아서는 전체 MAICON 경기장을 재현하거나 실행할 수 없습니다. 별도 자산과 ROS 2 Humble·Gazebo Fortress 설치가 필요합니다.

대회 자료의 외부 공유 제한과 제공 모델의 재배포 조건이 확인되지 않은 상태를 고려해 해당 자산은 게시하지 않았습니다. 이 코드의 공개를 시뮬레이션 성적이나 실물 로봇 성능 검증으로 해석하지 마세요.
