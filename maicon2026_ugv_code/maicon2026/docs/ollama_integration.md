# Ollama를 M1·M6 언어모델 어댑터로 연결하기

2026-11-07 검토. 제조사 위키의 자연어 처리 기능을 기존 `complete(prompt, timeout) -> str` 인터페이스로 연결했다. 모델은 M1 명령 해석과 M6 위험도 순위에만 참여하며, 이동·회전·레이저를 직접 실행하지 않는다. 모델 다운로드, Ollama 서버 설치, GPU 설정, 실제 모델 정확도·추론시간 검증은 이번 구현에 포함되지 않는다.

## 제조사 예제에서 수용한 부분

| 근거 | 제조사 동작 | 이 프로젝트 반영 |
| --- | --- | --- |
| [위키 8: Web Natural Language Interaction](https://www.waveshare.com/wiki/UGV_Beast_Jetson_Orin_ROS2_8._Web_Natural_Language_Interaction) | 별도 PC의 Ollama에 자연어를 보내 JSON 명령을 생성하고 로봇의 behavior 액션으로 전달한다. | Ollama JSON 응답을 받는 부분을 어댑터로 구현한다. 기존 M1·M6 파서가 임무 데이터로 해석한다. |
| [위키 9: Web-based Control Tool](https://www.waveshare.com/wiki/UGV_Beast_Jetson_Orin_ROS2_9._Web-based_Control_Tool) | Vizanti에서 TF·지도 표시와 조이스틱, 초기 위치, 항법 목표 입력을 제공한다. | 관찰과 현장 점검 절차의 참고 자료다. 대회 실행기에 Vizanti 조종 경로를 추가하지 않았다. |
| [위키 10: Command Interaction](https://www.waveshare.com/wiki/UGV_Beast_Jetson_Orin_ROS2_10._Command_Interaction) | `/behavior` 액션으로 전진·후진·회전·정지, 좌표 저장·이동을 요청한다. | 경기 중 속도 명령은 기존 Agent·MIL 중재를 거친다. `/behavior` 액션을 호출하지 않는다. |

위키 직접 요청은 HTTP 403이어서 검색 서비스가 보관한 공식 본문을 확인하고 제조사 저장소와 대조했다. 위키 8·9·10은 4~5개월 전 색인일 수 있다. 코드 근거는 [ugv_chat_ai/app.py](https://github.com/waveshareteam/ugv_ws/blob/f0b3ad9c16e7acb6d6145f261116d71bf6338f4a/src/ugv_main/ugv_chat_ai/ugv_chat_ai/app.py), [behavior_ctrl.py](https://github.com/waveshareteam/ugv_ws/blob/f0b3ad9c16e7acb6d6145f261116d71bf6338f4a/src/ugv_main/ugv_tools/ugv_tools/behavior_ctrl.py)다. 해당 behavior 예제는 문자열 명령을 `exec`하고 `/cmd_vel`을 직접 발행한다. 제조사 데모와 대회 Agent를 동시에 구동하면 속도 명령이 충돌할 수 있으므로 데모 조종 프로그램은 별도 점검 때만 사용한다.

## 기존 미션과 연결되는 방식

```text
M1 서버 명령 → 기존 build_prompt → Ollama JSON 문자열
             → 기존 parse_llm_json/merge → M1 제출

M6 관측 증거 → 기존 llm_rank 프롬프트 → Ollama JSON 문자열
             → 상대 ID 순열 검증 → M6 제출

주행·회피·발사 → 기존 Agent / Nav2 / MIL 경로 유지
```

M1은 어댑터가 설정되어 있으면 모델을 호출한다. 기존 규칙으로 명확히 판단한 필드는 유지하며, 누락되거나 모호한 필드에서 모델 값을 검토한다. 실패·시간초과 시 기존 규칙 결과를 사용한다. M6는 상대 3대 ID가 중복 없이 모두 있는 순위만 받아들이며, 실패하면 기존 관측 기반 순위로 돌아간다. 모델이 추정한 정보가 서버의 비공개 정답이 되는 것은 아니다.

## 설정과 실행

먼저 별도 절차로 준비한 Ollama 서버와 설치된 모델이 있어야 한다. `MAICON_OLLAMA_MODEL`에는 `ollama list`에서 확인한 **실제 모델 이름**을 넣는다. 위키의 Gemma 모델명은 예시이며 자동으로 선택하거나 내려받지 않는다.

ROS 2 Humble 환경의 프로젝트 실행 터미널에서 설정한다. 프로젝트 위치는 `maicon2026_ugv_code/maicon2026`이다.

```bash
cd /path/to/MAICON2026/maicon2026_ugv_code/maicon2026
export MAICON_OLLAMA_MODEL='YOUR_INSTALLED_MODEL'
export MAICON_OLLAMA_URL='http://127.0.0.1:11434'
```

`MAICON_OLLAMA_URL`은 `/api/chat`을 붙이지 않은 서버 주소다. 생략하면 `http://127.0.0.1:11434`다. Ollama가 별도 PC에 있다면 로봇에서 접근할 수 있는 해당 PC 주소로 설정한다. 여기서 localhost는 **Agent 프로세스가 실행되는 컴퓨터/컨테이너**다. URL 내 계정·비밀번호, 쿼리, 경로는 허용하지 않는다. 이 어댑터는 인증이 필요한 Ollama Cloud API나 인증 프록시를 지원하지 않는다. 주변 환경의 HTTP 프록시와 HTTP 리다이렉트도 사용하지 않는다.

ROS를 실행하지 않고 모델 연결만 확인할 수 있다. 이 확인은 모델 서버에 텍스트를 보내며 로봇을 움직이지 않는다.

```bash
python3 -c "from maicon26.llm.ollama import create_client; c=create_client(); print(c.complete('JSON 객체 하나로 ok=true를 응답하라', 6)); c.close()"
```

실기 드라이버·SLAM·Nav2와 필수 센서 준비는 [실기 재구현 가이드](실기_재구현_가이드.md)를 먼저 따른다. 기존 실행 명령에 다음 어댑터 인자를 추가한다. 아래 서버 주소, 임무코드, 가중치 경로는 현장 값으로 바꾼다.

```bash
python3 scripts/run_real.py \
  --config config/waveshare_real.json \
  --server http://SERVER:PORT --mission-code ABCD1234 \
  --detector /path/to/team_best.pt \
  --llm-factory maicon26.llm.ollama:create_client
```

주최측이 `team/team_node.py`를 실행하는 흐름은 같은 프로세스에 아래 환경변수를 전달한다. 다른 터미널에서 `export`한 값이 이미 실행 중인 프로세스에 자동 반영되지는 않는다.

```bash
export MAICON_LLM_FACTORY='maicon26.llm.ollama:create_client'
```

## 응답과 시간 제한

[Ollama 공식 Chat API](https://docs.ollama.com/api/chat)에 맞춰 `/api/chat`에 `model`, `messages`, `stream=false`, `format=json`을 전송하고, 완료 응답의 `message.content`를 읽는다. tool 정의를 보내지 않으며 tool 호출 응답을 거부한다. 반환 문자열은 JSON 객체여야 하며 코드블록·비정상 JSON·NaN·Infinity를 거부한다. 임무별 허용값 검증은 기존 M1·M6 파서가 수행한다.

- 호출자가 전달한 양수 timeout을 사용하며 60초를 상한으로 둔다. 현재 M1 기본값은 6초다.
- HTTP 응답은 기본 65,536바이트까지, 프롬프트는 UTF-8 기준 65,536바이트까지 허용한다.
- 호출 대기와 종료는 제한 시간 안에 반환한다. 느린 DNS/응답이 남아 있더라도 어댑터당 HTTP 작업 1개만 유지하며, 해당 작업이 끝나기 전 새 요청은 즉시 실패한다.
- timeout이나 `close()`는 Ollama 서버 내부의 생성 작업이 취소되었다는 뜻은 아니다. 늦게 도착한 결과는 해당 호출에 반영하지 않는다.
- HTTP 오류는 원문 응답이나 URL을 포함하지 않는 일반 오류로 반환한다. 어댑터는 API 자격증명·모델 응답을 로그에 출력하지 않는다.

첫 추론은 모델 적재 때문에 늦을 수 있다. 대회 전 위의 연결 확인과 실제 한국어 명령문으로 지연·정확도를 측정해야 한다. 이 어댑터를 연결했다는 사실만으로 6초 이내 추론이나 미션 성공이 보장되지는 않는다.

## 검증

```bash
python3 -m pytest -q -p no:cacheprovider tests/test_ollama.py tests/test_llm_integration.py
```

테스트는 로컬 모의 HTTP 서버와 지연·비정상 응답으로 M1/M6 연결, 시간 제한, 응답 크기 제한, 종료, ID 순열 검증을 확인한다. 실제 Ollama 모델 추론이나 GPU 성능 시험은 아니다. 기존 사용자 Gazebo 파일은 이 변경에서 수정하지 않았다.
