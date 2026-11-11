# 후보 02 결과 안내

이 폴더의 영상 7개는 모두 후보 02를 시험한 기록입니다. `comparison/`에는 다른 후보와 비교한 **JSON 선정 근거만** 남겨 두었습니다. 각 영상과 보고서의 시드·채점 범위를 함께 확인하세요.

| 기록 | 영상 | 보고서 |
| --- | --- | --- |
| 6대 자가 대전, 시드 0 | [3인칭 영상](candidate02_four_runs/selfplay_seed0/wingbot_selfplay_third_person.mp4) | [4경기 점수 요약](candidate02_four_runs/RESULTS.md) |
| 6대 자가 대전, 시드 1 | [3인칭 영상](candidate02_four_runs/selfplay_seed1/wingbot_selfplay_third_person.mp4) | [4경기 점수 요약](candidate02_four_runs/RESULTS.md) |
| B-2 단독, 시드 0 | [3인칭 영상](candidate02_four_runs/solo_seed0/b2_autonomous_third_person.mp4) | [4경기 점수 요약](candidate02_four_runs/RESULTS.md) |
| B-2 단독, 시드 1 | [3인칭 영상](candidate02_four_runs/solo_seed1/b2_autonomous_third_person.mp4) | [4경기 점수 요약](candidate02_four_runs/RESULTS.md) |
| 먼저 완료한 6대 경기 | [3인칭 영상](gazebo_candidate02_completed/wingbot_selfplay_third_person.mp4) | [경기 보고](gazebo_candidate02_completed/RESULTS.md) · [점수 분석](gazebo_candidate02_completed/SCORE_ANALYSIS.md) |
| 중단된 부분 기록 | [부분 영상](gazebo_candidate02_partial/wingbot_selfplay_third_person.mp4) | [부분 기록 설명](gazebo_candidate02_partial/RESULTS.md) |
| 2D 선정 시험 | [진단 영상](selected_candidate02/diagnostic.mp4) | [선정 기준](SELECTION.md) · [원시 결과](selected_candidate02/result.json) |

선택 설정(`config/active_candidate.json`, 2026-11-11 삭제됨)은 2D 시험의 M3 누락과 M2+M3 점수로 후보를 정했습니다. Gazebo 후속 점수는 로컬 채점 결과이며 공식 서버 점수가 아닙니다. **2D 선정 점수와 실제 Gazebo 경기 점수를 같은 시험 결과로 읽지 마세요.**

`cleanup_manifest.json`은 후속 4경기 이전의 정리 시점을 기록합니다. 그 파일의 영상 수 3개는 현재 전체 영상 수가 아닙니다.
