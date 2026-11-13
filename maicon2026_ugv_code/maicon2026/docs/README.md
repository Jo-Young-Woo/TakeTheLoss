# 문서 안내

처음 방문했다면 저장소 [첫 화면](../../../README.md)에서 실행 순서를 확인하세요. 이 폴더는 설계 근거와 검증 기록을 보존합니다. 날짜가 있는 문서는 해당 시점의 기록이며, 후보 02 동결본과 선택 프로필은 2026-11-11에 삭제되어 지금은 개발 코드 `../maicon26/`가 실행됩니다.

| 주제 | 문서 |
| --- | --- |
| 최신 B-2 Nav2·M3 시험 | [2026-11-13 검증 요약](NAV2_M3_VALIDATION_20261113.json) |
| 경기장 모델의 인수 범위 | [ARENA_ACCEPTANCE.md](ARENA_ACCEPTANCE.md) · [Gazebo 실행 계약](gazebo_runtime_contract.md) |
| 자율주행·로봇 연결 | [실기 통합](hardware_integration.md) · [Waveshare ROS 연결](waveshare_ros_integration.md) |
| UAV 영상 좌표 보정 | [uav_calibration.md](uav_calibration.md) · [LiDAR 지도 정합·외곽면 기록](UAV_LIDAR_ALIGNMENT.md) · [UAV 원본 영상 기반 면 장부](UAV_IMAGE_FACE_LEDGER.md) |
| M3 탐색과 객체 면 관측 | [M1 지정 두 섹터 외곽면 경로](M3_EXHAUSTIVE_ROUTE.md) · [M3 탐색·회피 설계](M3탐색_적회피_통합_설계.md) · [도착 기하](M3_ARRIVAL_GEOMETRY.md) |
| 경기 서버 API | [API 추적표](API_TRACEABILITY.md) · [준수 검토](server_api_compliance.md) |
| 영상과 실행 결과 | [녹화 안내](gazebo_recording.md) · [후보 02 결과](../results/README.md) |
| 원본 모델 출처 | [WINGBOT 자산·라이선스 기록](../ros2/wingbot_description/vendor/LICENSE_AUDIT.md) |

실행 코드는 `../maicon26/`입니다. 과거 후보 02 동결본(`runtime/selected_candidate02`)은 삭제되었으므로, 문서의 성능 수치와 과거 수정 기록은 지금 코드의 결과가 아닙니다.
