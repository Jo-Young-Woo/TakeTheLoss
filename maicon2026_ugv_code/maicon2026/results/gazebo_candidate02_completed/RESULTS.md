# 후보 02 실제 Gazebo 완료 경기

2026-11-09, 6대 자가 대전, seed 0, `progress` 변형, UAV 사용. 선택된 후보 02의 동결 Python 122개 파일을 `scripts/selected_runtime.py`로 검증한 후 실행했다. 영상과 출력은 WSL 내부 `/home/jojo0429/maicon_candidate02_runs/full_20261109_01`에 먼저 저장하여 이전 OneDrive 파일 저장 오류를 피했다.

경기는 300초 상한에 도달하기 전에 **252.336초**에 종료됐다. BLUE가 R-2 거점을 점령한 경기 종료 이벤트(`m5_capture`)이며, `completed=true`, 여섯 Agent 종료 코드가 모두 0이고 오류 목록은 비었다. 이 종료는 규칙에 따른 정상적인 경기 완료다.

[3인칭 영상](wingbot_selfplay_third_person.mp4)은 1280×720, 10fps, **252.3초·2,523프레임**이다. 녹화 보고서에서 누락·중복 시각·프레임 복제·큐 폐기가 모두 0이고, 영상 사본의 SHA256은 WSL 원본과 일치한다(`ff211ac51b7782a3d88cbb01a9b9222db2bf85c256912ffc5d35786f121bb9e7`). [전체 실행 보고서](selfplay_report.json), [소스 해시](source_manifest.json), [녹화 보고서](wingbot_selfplay_third_person.recording.json)를 함께 보존했다.

여섯 로봇의 `target_done`은 모두 false이며, 심판 이벤트에는 M2 명중·피격이 없다. 종료 시 로봇별 M3 보고 항목 합계는 B-1 19, B-2 19, B-3 9, R-1 11, R-2 8, R-3 10이다. 이 수는 **신고한 분류 수**이며 정답과 대조한 채점 점수나 개별 면 탐지율이 아니다. 따라서 2D 시험의 92/96면이나 M2+M3 30.83점을 이 Gazebo 경기 결과로 옮겨 적지 않는다. 영상 인식은 참고 사진 기반 검출기이며 실제 학습된 현장 검출기 성능을 뜻하지 않는다.

[오프라인 점수 분석](SCORE_ANALYSIS.md)에서 마지막 접수된 M3 보고와 경기장 정답을 대조했다. B-3의 `R-S2`만 정확히 일치해 M3 7점이며, 나머지 M3 구역은 0점이다. M6 동점 처리 방식은 잠정적이므로 전체 점수는 확정 점수가 아닌 추정치다.
