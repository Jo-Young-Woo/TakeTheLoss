# 경기 서버 API 대응 기록

확인일: 2026-11-06. 기준은 [참가팀 단계별 API 안내](https://maicon.kr/wp-content/kossa/2026MAICON/game-server-api.html#p0)의 p0~p12 전체이며, 1차 PDF의 임무 설명과 2차 PDF의 로봇 API·프로세스 운용을 함께 적용한다. HTTP 서버 API와 로봇의 ROS `/mil/*` API는 서로 다른 인터페이스다.

검토한 HTML의 SHA256: `834c7a098a8fbc6b6b528840f8d8b81b194604aa443f05081106e3b9d2699c88`.

아래의 “구현”은 코드와 모의 HTTP 서버 테스트를 뜻한다. 실제 대회 서버 연결·현장 인증·레이저 및 거점 장치 판정 검증은 수행하지 않았다. 원문에서 M1·M3·M6 JSON은 여전히 임시 형식으로 표시되어 있으므로 현장 안내 변경 시 이 표와 `link/schema.py`를 함께 갱신한다.

| 페이지 | 공식 역할 | 코드 대응 및 검증 범위 |
|---|---|---|
| p0 | API·시간 흐름 | `/api/report/`, GET 쿼리/POST 본문 `mission_code`, 8자리 검증, 오류에서 임무코드 가림. 남은 시간·점수·위치 API를 가정하지 않음 |
| p1 | 준비완료 | `/me`의 `phase`, `ready`, `slot` 확인 후 `/ready`. 초기화가 완료된 프로세스만 핸드셰이크 종료. 이미 ready라도 재시작한 로컬 센서는 다시 검사. `slot=null`은 미편성으로 처리 |
| p2 | 전술명령 | `/order?wait=120` 장기 폴링, HTTP 제한시간은 대기시간보다 길게 설정. `accepted:false`는 재시도, 명령과 `issued_at` 전달 |
| p3 | 출발·주행 | 서버에 주행 API를 만들지 않음. 로봇 `/mil/match` 및 `/cmd_vel`은 HAL 담당. `/me`의 running은 보조 시작 알림 |
| p4 | 드론 JPEG | `/api/report/drone`, 원본 `UavFrame.jpeg` 및 OpenCV 사용 가능 시 BGR `image`. 캡처 시각 중복/역순/잘못된 타임존은 폐기. 1/2/5초 주기와 파괴 수 메타데이터 전달. `main/test1/test2` 연습 카메라는 명시 선택 |
| p5 | M1 해석 제출 | `/interpretation`, target·search_sectors·base를 공식 코드로 변환. 상대 진영 접두어, `COMMON→CM`. 전체 스냅숏을 제출하며 서버의 덮어쓰기 방식 유지 |
| p6 | M2 지정표적 | 타격 성공 보고 HTTP 엔드포인트를 만들지 않음. 장치가 판정. 로봇 조준·레이저 및 자체 관측은 HAL/임무 계층 책임 |
| p7 | M3 집계 제출 | `/detection`, BLUE는 R-S1~3+CM, RED는 B-S1~3+CM을 제출. 빈 구역도 포함. 객체 6종 코드·count 0~99 표현. 비동기 제출 시 깊은 복사로 집계 도중 payload 변경 방지 |
| p8 | M4 UGV 타격 | 타격 보고 HTTP 없음. 상대 점수·탄약 정답을 서버에서 받는다고 가정하지 않음. 우리 코드의 적 회피와 구분 |
| p9 | M5 목표거점 | `/target?wait=120`, `objective`와 `objective_label` 모두 내부 메시지에 보존. 타이머만 믿고 임의 거점을 생성하지 않음 |
| p10 | 경기 종료 | `/me` reporting/finished를 stop 알림으로 변환. 조기 M5 종료도 phase 전환으로 감지. 정지/보고 처리는 임무 계층 담당 |
| p11 | M6 위험도 제출 | `/situation`, 상대 호출부호 1~3개, 중복·아군 제거. 유효 순위가 없으면 네트워크 제출 전 실패 처리. 409는 제출기에서 재시도 |
| p12 | 오류·리허설 | 401/409/422를 제출 실패로 처리하고 근거 문구 보존. 리허설의 me/ready/order/target/보고는 `/api/rehearsal/`. **리허설에는 drone이 없으므로 기본 수신 중지**; 연습 카메라를 선택하면 항상 `/api/report/drone?camera=...` |

## 연결 인터페이스

```python
link = HttpServerLink(
    server_url,
    mission_code,
    rehearsal=True,
    drone_camera="test1",  # 연습 모드에서만 명시. 실제 경기에는 None.
    decode_drone=True,
)
```

`drone_camera=None`인 정규 경기에서는 running 단계에서만 화면을 요청한다. 리허설 기본값은 화면 요청을 하지 않는다. `decode_drone=False`이면 JPEG만 반환한다. OpenCV가 없어도 원본 JPEG와 헤더는 보존된다. 이미지로부터 UGV의 경기장 좌표를 얻으려면 카메라 기하 보정과 검출 모듈이 별도로 필요하다. HTTP 코드는 좌표를 만들어 넣지 않으며 `detections=[]`로 둔다.

## 프로세스·타이밍 경계

- 한 `HttpServerLink` 인스턴스는 한 경기용이다. 공식 API에 경기 ID가 없고 `/order`는 경기 종료 뒤에도 반환되므로, 동일 인스턴스의 START/STOP 래치를 임의로 풀어 다음 경기라고 추측하지 않는다. 대회 팀 노드의 경기별 실행 구조를 따른다.
- `/me`에서 슬롯이 null로 바뀌면 기존 슬롯과 생성자에서 준 슬롯도 폐기한다. 미편성 상태에서 이전 팀 기준 보고를 보내지 않는다.
- M1·M3·M6는 **서버 도착 시각** 기준으로 접수된다. 로컬에서 전송을 시도했다는 것만으로 성공으로 기록하지 않는다. HTTP 200과 `accepted=true`를 확인한다.
- M3 값이 99를 넘으면 기존 변환기의 상한 적용을 유지한다. 잘못된 자료형은 예외로 제어 스레드를 죽이지 않고 제출 실패로 반환한다. 자료형 오류는 근본적으로 집계 생성 측에서 해결해야 한다.
- 영상 요청은 주행 루프 밖에서 수행한다. 기존 장기 폴링은 별도 데몬 스레드이며 종료 이벤트 후 진행 중인 urllib 요청은 타임아웃까지 남을 수 있다. 로봇 정지 자체가 그 요청 완료를 기다리지 않도록 한다.

## 검증

```powershell
python -m pytest -q -p no:cacheprovider tests/test_server_link.py
```

24개 테스트 통과(2026-11-06). 준비완료 재시도, 초기화되지 않은 재실행 차단, 슬롯 철회, 비동기 STOP 경로, 공식 보고 코드, 리허설 경로, 연습 카메라, JPEG 보존, 영상 시각 역전, 잘못된 제출값 및 큐 payload 스냅숏을 검사했다. 전체 테스트 결과는 통합 실행 기록을 확인한다.
