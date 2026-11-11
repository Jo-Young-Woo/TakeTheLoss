# 실행 스크립트 안내

처음 실행한다면 [저장소 첫 화면](../../../README.md)의 설치·실행 순서부터 확인하세요. 이 폴더에는 현재 경기를 돌리는 진입점과 과거 분석·보정 도구가 함께 있습니다.

| 목적 | 진입점 | 설명 |
| --- | --- | --- |
| ROS/Gazebo 패키지 빌드 | [build_arena_gz.sh](build_arena_gz.sh) | 경기장·WINGBOT·로컬 인터페이스 세 패키지를 공백 없는 Linux 경로에 빌드 |
| 실행 코드 선택 | [selected_runtime.py](selected_runtime.py) | `config/active_candidate.json`이 없으면 개발 코드 `maicon26/`를 사용(후보 02 삭제 후 기본). 경기를 시작하지 않음 |
| 전체 경기 | [run_selected_gazebo.sh](run_selected_gazebo.sh) | 기본 6대 자가 대전·시드 0·최대 300초 (개발 코드) |
| 6대 실행 상세 | [run_wingbot_selfplay.py](run_wingbot_selfplay.py) | 시드·시나리오·UAV·녹화 옵션을 처리하는 실제 실행기 |
| B-2 단독 경기 | [launch_wingbot_autonomous_trial.py](launch_wingbot_autonomous_trial.py) | 경기장 준비와 단독 실행 절차 |
| 임무 채점 검토 | [evaluate_wingbot_missions.py](evaluate_wingbot_missions.py) | 로컬 점수 추정; 공식 서버 확정 점수가 아님 |

`compare_*`, `probe_*`, `benchmark_*` 등은 개발·진단용입니다. 기본 실행에서 사용자가 직접 호출할 필요가 없습니다. 알고리즘 본문은 `../maicon26/`에 있습니다(후보 02 동결본은 삭제됨).

새 출력은 `../recordings/`에 저장되며 Git에서 제외됩니다. 보존한 완료 경기와 영상은 [결과 안내](../results/README.md)를 확인하세요.
