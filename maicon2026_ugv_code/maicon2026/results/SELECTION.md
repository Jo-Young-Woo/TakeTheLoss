# 최종 선택: 후보 02

2026-11-09. 사용자의 지시에 따라 후보 선택을 완료하고 추가 실험을 종료했다.
완료된 동일 조건의 **2D selfplay, seed 0**에서 먼저 M3 면 누락을 최소화하고, 동률이면 M2+M3 점수를 최대화했다.

| 후보 | 210초 M3 면 누락 | 로봇별 M2+M3 평균 |
|---|---:|---:|
| **02 (선택)** | **4 / 96** | **30.8333** |
| 03 | 4 / 96 | 23.1667 |
| 04 | 8 / 96 | 26.5000 |
| v2 | 11 / 96 | 18.1667 |

M3 100% 달성이나 실제 Gazebo 최고 성능을 뜻하지 않는다. 다른 시드에 대한 보장도 없다. 구형 교착 평가기의 판정에는 알려진 문제가 있어 선택 근거로 사용하지 않았다. 선택은 원자료의 M3 관측 면수와 M2+M3 점수에 근거한다.

## 실행 코드

- 선택 프로필: `config/active_candidate.json` (2026-11-11 삭제됨)
- 검증한 후보 02의 Python 파일 122개: `runtime/selected_candidate02/maicon26` (2026-11-11 삭제됨; 삭제 직전 사본은 `_hw_work/candidate02_runtime_backup.tar.gz`)
- 두 Gazebo 실행기는 선택된 패키지를 먼저 불러온다. 개발 소스와 회귀시험은 보존했다.
- 원본 시험의 소스 해시와 선택 패키지의 모든 파일 해시를 대조했다.
- 공용 모델·설정·텍스처는 원래 폴더를 사용한다. Git에 담기지 않는 선택 폴더의 자산 연결은 첫 검증 때 자동으로 생성된다.
- Gazebo 실행 기반은 `/home/jojo0429/wingbot_model_ws_20261109/install`이다.

WSL Ubuntu-22.04의 프로젝트 폴더에서 다음 명령으로 실행한다. 정리 작업 중에는 새 시뮬레이션을 시작하지 않았다.

```bash
bash scripts/run_selected_gazebo.sh
```

## 보존 영상과 근거

후속 요청에 따라 수행한 [후보 02의 6대 자가 대전 2회와 B-2 단독 2회 점수·영상](candidate02_four_runs/RESULTS.md)도 보존했다. 현재 영상은 총 7개다. 아래 목록은 이전 3개를 설명하며, 정리 기록의 영상 수는 추가 실행 전 시점의 기록이다.

1. [후보 02 — 완료된 실제 Gazebo 3인칭 영상](gazebo_candidate02_completed/wingbot_selfplay_third_person.mp4): 후속 요청으로 다시 실행하여 252.336초에 거점 점령으로 정상 종료했다. [실행 결과](gazebo_candidate02_completed/RESULTS.md).
2. [후보 02 — 완료된 2D 진단 영상](selected_candidate02/diagnostic.mp4)
3. [후보 02 — 실제 Gazebo 3인칭 부분 영상](gazebo_candidate02_partial/wingbot_selfplay_third_person.mp4): 미리보기 저장 오류로 약 168.3초에서 중단된 이전 기록이다.

[선택 시험 결과](selected_candidate02/result.json), [정확한 시험 설정](selected_candidate02/configuration.json), [비교 결과](comparison/)를 보존했다.
나머지 영상과 중간 실험 데이터·중복 압축본·캐시의 실제 정리 내역은 [정리 기록](cleanup_manifest.json)에 남겼다. 2026-11-09 후속 정리에서 5,100개 파일(958.60 MiB)을 삭제했고, MAICON2026 폴더에 후보 02 영상 세 개만 남겼다. 동결 소스 해시와 실행 자산·스크립트 구문을 다시 확인했다.
대회 원본 PDF·API 자료, 소스·시험·모델·설정은 보존한다. 이 문서가 최종 후보 선택을 설명하며, 파일 정리 완료를 뜻하지 않는다.
