# 실제 Gazebo UAV 영상을 사용하는 로컬 API

`scripts/run_arena_mock_api.py`는 `/uav/image_raw`의 실제 픽셀과 `/arena/state`의 경기 단계·시설 파괴 수를 기존 `MockServer`에 공급한다.
영상 합성, 세계 좌표 정답, M3 객체 종류를 참가팀 입력에 넣지 않는다. HTTP 요청·응답·오류·영상 헤더의 형식은 기존 MockServer가 처리한다.

기본 수신 주소는 **로컬 컴퓨터의 loopback만**이다. `0.0.0.0`이나 외부 IP로 열 수 없다. 이 서비스는 주최측 서버가 아니며 개발용이다.
실제 로컬 실행에서 `/me`와 `/api/report/drone`의200 응답, 정상 JPEG 및6개 영상 헤더를 확인했다. 근거는 `ros2/maicon_arena_gz/validation/api_live_probe.json`과 실제 응답 영상 `api_uav_live.jpg`이다. 주최측 실서버 접속이나 전체 임무 성공을 뜻하지 않는다.

```bash
# ROS/Gazebo와 패키지 환경을 먼저 활성화한다.
# MAICON_CODE(기존 MAICON_MISSION_CODE도 지원)는 셸 이력에 값을 쓰지 않는 방법으로 설정하거나,
# --code-file을 사용한다. 없으면 화면에 표시되지 않는 입력을 최대3회 받는다.
python3 scripts/run_arena_mock_api.py --slot B-2 --code-file /비밀파일/경로
```

기본 포트는8000이며 `--port`로 바꾼다. `--bind`는 literal loopback IP만 허용한다.
코드를 명령행 값으로 받는 옵션은 없다. HTTP 접근 로그는 GET 쿼리에 들어간 코드를 노출할 수 있어 출력하지 않는다.
오류 출력·원시 HTTP 파싱 오류·진단 기록에도 코드를 남기지 않는다. 마지막128개 요청 기록만 유지한다.
큰 영상이 빠지지 않도록 이 클라이언트도 `FASTRTPS_DEFAULT_PROFILES_FILE`과 `FASTDDS_DEFAULT_PROFILES_FILE`에 설치된 `maicon_arena_gz/config/fastdds_shm64.xml`을 지정하고 Fast DDS로 시작한다. launch의 `image_transport_profile:=shm64`는 다른 터미널에 자동 전파되지 않는다. 설정·실제20초 전송 결과는 패키지 `docs/runtime.md`에 있다.

## 기존 Agent에 실제 영상의 박스 후보 연결

`run_real.py`에 아래 옵션을 **명시한 시뮬레이션 실행에서만** HTTP JPEG를 보정해 박스 형상 후보를 만든다. 기존 로봇 설정·모델 경로·로컬 서버·코드 입력은 평소와 같이 제공한다.

```text
--arena-spec ros2/maicon_arena_gz/config/arena_spec.yaml
--simulation-uav-calibration ros2/maicon_arena_gz/config/uav_grid_calibration.json
```

`--arena-spec`이 없거나 보정의6×4.8m 치수·0.4m 윗면 높이가 다르거나 독립 검증점이 통과하지 않으면 하드웨어 초기화 전에 실패한다. 이 보정은 왼쪽 아래가(0,0)인 기존 항법 좌표와 같으므로 Gazebo 중심 좌표의 오프셋을 두 번 더하지 않는다. 레이아웃 YAML·정답·모델 pose는 검출 입력에 들어가지 않는다.

검출은 `HttpServerLink.get_uav()`의 선택적 콜백으로 실행되고, 실기 진입점에서는 기존 `AsyncServerLink`의 UAV 작업 스레드가 처리한다. 제어 루프에서 이미지 처리를 기다리지 않는다. `layout_source="gazebo_uav_image"`인 형상 후보만 전달하며 UGV 식별이나 M3 사진 종류를 만들지 않는다. 지상 사진으로 확인하기 전의 M3 집계는0이다. 두 번 일치한 새 프레임부터 기존 CoverageMap의 탐색 후보가 된다.

원래 캡처 epoch·JPEG·헤더를 보존하고, 명시적 시뮬 경로에서는 HTTP Date의 시계 대응으로 영상 나이를 계산한다. 디코딩·검출 전에 받은 시각을 기록해 처리 지연을 숨기지 않으며, Agent가 그 나이를 로봇 시계로 바꾼다. 설정된 영상 만료시간(기본12초)을 넘거나 HTTP Date 정밀도 이상 미래인 사진, 중복·역행·잘못된 크기·검출 오류는 새 배치 증거로 쓰지 않는다. 빈 영상에는 빈 후보 목록만 남긴다.

옵션을 주지 않은 실제 로봇 실행은 기존처럼 JPEG/BGR만 받고 이 Gazebo 외형 가정을 적용하지 않는다. 보정이나 카메라 화각·자세·자르기가 바뀌면 실측과 독립 검증을 다시 해야 한다.

## 영상과 시각

- ROS rgb8/bgr8/rgba8/bgra8/mono8의 실제 행 길이를 보존해 JPEG로 바꾼다. 기본 품질85, 원본 해상도 유지이다.
- 캡처시각은 이미지 header의 simulation stamp다. `/clock`을 기준으로 시각을 진행시키며 HTTP 대기 때문에 시뮬 시간을 빠르게 넘기지 않는다.
- 기본 UTC 기준점은 첫 `/clock`을 받은 당시의 현재 UTC에서 simulation stamp를 뺀 값이다. `--epoch`에 timezone이 있는 ISO8601을 주면 재현 가능한 기준점을 사용할 수 있다.
- `X-Captured-At`은 해당 기준점+원래 캡처시각을 한국 시간 `+09:00`으로 표현한다. 늦게 수신한 프레임에 수신시각을 새 캡처시각으로 붙이지 않는다.
- 기본2초보다 늦게 도착한 입력, 중복·역행·깨진 JPEG는 채택하지 않는다. `/clock`보다 최대0.25초 먼저 도착한 영상은 카메라별1장만 보관하고 시계가 따라온 후에만 공개한다. 그보다 미래 시각은 거부한다.
- 이미 채택한 정상 영상은 후속 영상이 깨지거나 끊겨도 **원래 시각으로** 계속 제공할 수 있다. 공식 마지막 정상 화면 정책이며, 최신 영상으로 꾸미지 않는다. 소비자는 Captured-At으로 오래된 영상을 판단해야 한다.
- 자기 진영 시설 파괴 수0/1/2개는1/2/5초 간격,3개는403이다. 반대 진영 영상 주기를 함께 낮추지 않는다. 테스트 모드에서는 명시적으로 받은 해당 카메라 영상만 사용하며 test1에 main 영상을 대신 넣지 않는다.
- Gazebo 시계가 뒤로 돌아가면 이전 영상·경기를 비우고 닫힌 상태로 남는다. 새 경기 환경으로 어댑터를 다시 시작해야 한다.

## 명령·보고 연결

`/api/report`의 me/ready/order/drone/interpretation/detection/target/situation를 기존 MockServer로 처리한다.
시작·목표거점·종료를 임의로 만들지 않고 `/arena/state`의 단계를 따른다.
`--operator-config`가 없으면 명령문·목표거점을 게시하지 않는다. 요청은 안내서의 accepted:false와 reason을 받는다.

운영자 JSON의 구조는 다음과 같다. 실제 시험 명령과 목표값을 운영자가 명시해야 한다.

```json
{"orders": {}, "targets": {}, "test_mode": false}
```

`orders`는 호출부호→명령문이다. `targets`는 BLUE/RED 각각에 `objective`와 `objective_label`을 넣는다.
거점 코드는 해당 진영의 상대편 시설이어야 한다. 명령은 관찰한 시작시각, 목표는240초 게시시각으로 기록된다.
설정이 없는 팀의 임무를 다른 팀에서 복사하지 않는다. 실제 심판 목표와 이 설정의 목표를 맞추는 책임은 운영자에게 있다.

받은 M6는 로컬 ROS `/arena/control`에 `{"cmd":"m6_received","slot":...}`를 보낸다.
이는 심판 내부 연결이며 공식 HTTP 경로를 추가한 것이 아니다. M1/M3는 reporting에서409이며 기존 서버의 전체 대체·정규화 규칙을 따른다.
현재 CLI는 코드 하나와 슬롯 하나를 등록한다. 여섯 참가팀 모두를 연결하려면 별도 자격증명 구성과 전체 연동 검증이 필요하다.

리허설 경로의 형식은 MockServer에 있지만 이 어댑터는 실제 경기장의 report 상태만 연동한다. 리허설 운영·카메라 test1/test2 공급·공식 포털을 구현했다고 주장하지 않는다.
실제 서버와 동일한 네트워크 서비스 품질이나 보안 격리를 제공하는 제품 서버가 아니다. 일반 POST 본문은64KiB로 제한하고 긴 폴링은 시뮬이 멈춰도 벽시계 timeout으로 끝낸다.
