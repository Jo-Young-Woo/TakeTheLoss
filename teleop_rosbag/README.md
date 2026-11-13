# teleop_rosbag · 키보드 텔레옵과 rosbag 기록

연습 시간에 로봇을 직접 몰면서 데이터를 모으는 도구다. 경기 중에는 쓰지 않는다. 대회 자료와 기록한 bag 은 팀 밖으로 공유하지 않는다.

**실제 로봇과 ROS 2 에서 돌려 본 적이 없다.** 2026-11-13에 가짜 ROS(`tests/fake_ros`, `tests/fake_bin/ros2`)로 흐름만 확인했다. 처음 쓸 때는 아래 "첫 실행 점검"을 순서대로 한다.

## 파일

| 파일 | 하는 일 |
|---|---|
| `teleop_record.sh` | 기록을 켜고 텔레옵을 한다. 텔레옵을 끝내면 기록도 닫는다 |
| `teleop_keys.py` | 방향키 텔레옵 (`teleop_core.py` 가 명령 규칙과 안전장치) |
| `record.sh` | 모든 토픽을 rosbag 으로 기록 (`ros2 bag record -a -s mcap`) |
| `bag_check.py` | 기록 직후 점검: 토픽별 개수·주기, 빠진 토픽, 마커 시각 |
| `scan_secret.py` | 기록에 임무코드가 남았는지 검사 (값은 출력하지 않는다) |
| `extract_images.py` | bag 에서 raw 영상을 PNG 로 뽑는다 (마커 구간 단위) |
| `uav_collect.py` | UAV(상공 카메라) 영상을 일정 간격으로 받아 저장 |
| `tests/` | 로봇 없이 도는 시험과 가짜 ROS |

## 쓰는 순서

로봇(젯슨)에 SSH 로 들어가 ROS 2 Humble 환경을 불러온 터미널에서 실행한다. 텔레옵과 기록은 로봇 드라이버와 같은 컨테이너 안에서 돌린다.

```bash
# 0) 팀 노드를 끈다. /cmd_vel 송신자가 둘이면 텔레옵은 아무것도 보내지 않고 3초 뒤 끝난다
# 1) 기록하면서 텔레옵 (-t 는 분류 태그)
./teleop_record.sh -t cal_cam            # 체스판 촬영
./teleop_record.sh -t drive              # 직진·회전·정지 거리
./teleop_record.sh -t lidar              # 정지 스캔, 경기장 한 바퀴
./teleop_record.sh -t faces              # 상자 면 영상
./teleop_record.sh -t laser -- --allow-fire     # 레이저 영점 (-- 뒤는 텔레옵 인자)

# 2) UAV 영상은 rosbag 에 안 들어간다. 노트북에서 따로 받는다
MAICON_CODE=<임무코드> python3 uav_collect.py --server http://<서버> --camera main --label layout1

# 3) 기록 폴더 점검 (teleop_record.sh 가 끝날 때 자동으로 한 번 돈다)
python3 bag_check.py ~/bags/<세션 폴더>
MAICON_CODE=<임무코드> python3 scan_secret.py ~/bags/<세션 폴더>

# 4) 체스판 구간만 뽑아 같은 보정 코드로 다시 보정
python3 extract_images.py ~/bags/<세션 폴더> --list                      # 마커와 구간 번호 보기
python3 extract_images.py ~/bags/<세션 폴더> --segment 1 --hz 1 --out cal_images
python3 ../maicon2026/scripts/calib_fit.py --camera-images cal_images/*.png --board 9x6 --square 0.025 --camera-model fisheye
```

윈도우를 거쳐 복사하면 실행 권한이 빠질 수 있다. `Permission denied` 가 나오면 `bash teleop_record.sh …` 로 실행하거나 `chmod +x *.sh *.py` 를 한 번 한다.

기록 폴더(세션)는 `~/bags/<날짜_시각>_<태그>_<호스트>/` 이고 안에 `bag/`(rosbag), `notes.txt`(시각·태그·설정 해시, 메모 칸), `info.txt`, `record.log`, `teleop_events.jsonl` 이 생긴다. 위치를 바꾸려면 `-o <폴더>` 또는 `BAG_ROOT`.

## 키

| 키 | 동작 |
|---|---|
| ↑ ↓ ← → | 누르고 있는 동안만 전진·후진·좌회전·우회전. 손을 떼면 선다 |
| `Space` | 즉시 정지 |
| `1`~`5` | 속도 단계. 최대(0.6 m/s, 3 rad/s)의 20~100 %. 시작은 1단계(0.12 m/s, 0.6 rad/s) |
| `W` `S` | 틸트 위·아래 (한 번에 5단위 = 0.75°) |
| `A` `D` | 팬 왼쪽·오른쪽 (한 번에 5단위 = 2.5°). 대문자(Shift)는 20단위 |
| `C` | 팬틸트 가운데 |
| `M` | 구간 마커. 시나리오가 바뀔 때마다 누른다 |
| `F` `F` | 발사. `--allow-fire` 로 켰을 때만, 1초 안에 두 번 |
| `H` / `Q` | 도움말 / 종료 |

터미널은 키 두 개를 동시에 알려 주지 않아서 직진과 회전을 섞은 곡선 주행은 안 된다. 직진과 제자리 회전을 번갈아 쓴다.

## 안전장치

- **경기 모드 차단**: `/mil/match`(또는 `/mil/state`)의 mode 가 `mission` 이면 아무것도 보내지 않고 끝난다. mode 를 아직 못 받았으면 0 만 보낸다. 상태 토픽이 없는 환경에서만 `--no-state-guard` 로 푼다.
- **/cmd_vel 단독 송신**: 다른 송신자가 보이면 0 도 보내지 않고, 3초 이어지면 끝난다.
- **손을 떼면 정지**: 키 반복이 0.2초 끊기면 0 으로 간다. 로봇도 명령이 0.5초 끊기면 스스로 선다.
- **종료 시 정지**: `Q`, Ctrl-C, SSH 끊김(SIGHUP), SIGTERM, 예외 모두에서 0 을 5번 보내고 끝난다.
- **무입력 종료**: 120초 동안 키가 없으면 끝난다.
- **속도 한도**: 0.6 m/s, 3 rad/s 를 넘겨 보낼 수 없다.
- **발사**: 기본은 꺼짐. 사람 눈 방향으로 쏘지 않는다.
- **디스크**: 남은 공간이 10 GB 미만이면 기록을 시작하지 않고, 3 GB 아래로 내려가면 기록을 닫는다. 640×480 raw 영상은 10 fps 에서 분당 약 0.55 GB, 30 fps 에서 약 1.7 GB 다.
- **임무코드**: `notes.txt` 에 환경변수를 남기지 않는다. `MAICON_CODE` 가 설정돼 있으면 기록이 끝날 때 `scan_secret.py` 가 자동으로 돈다.

## 서비스 호출과 마커

ROS 2 Humble 의 rosbag 은 서비스 호출을 기록하지 않는다. 그래서 팬틸트·발사 요청과 응답, 속도 단계, 정지, 마커를 `/teleop/events`(`std_msgs/String`, JSON 한 줄)로 발행하고 `teleop_events.jsonl` 에도 남긴다.

```json
{"t": 1794560342.856, "type": "marker", "n": 1}
{"t": 1794560343.120, "type": "gimbal", "pan": 20, "tilt": 0, "resp": "ok"}
```

## 첫 실행 점검 (로봇에서)

- [ ] 충전 케이블을 빼고 넓은 곳에서 1단계로 시작한다.
- [ ] `ros2 topic list` 에 `/cmd_vel`, `/odom`, `/image_raw`, LiDAR 토픽이 보이는지. LiDAR 이름이 `/scan` 이 아니면 `bag_check.py --scan-topic <이름>`.
- [ ] 텔레옵 화면 끝에 `practice/…` 가 뜨는지. `?/?` 로 남으면 상태 토픽 타입을 못 찾은 것이다. `ros2 topic list -t | grep mil` 로 타입을 보고 `--type match=<pkg/msg/이름>` 으로 지정한다.
- [ ] ↑ 가 전진, ← 가 좌회전, `D` 가 팬 오른쪽, `W` 가 틸트 위인지.
- [ ] 키를 떼면 바로 서는지. 누르고 있는데 가다 서다 하면 `--first-hold 0.8` 또는 `--release 0.3` 으로 늘린다(터미널의 키 반복 지연에 맞춘다).
- [ ] 텔레옵 중에 SSH 를 끊어도 로봇이 서는지.
- [ ] 팬틸트 응답이 화면에 `ok` 로 뜨는지. `서비스 없음` 이면 `ros2 service list -t | grep mil` 로 확인하고 `--type gimbal=<pkg/srv/이름>`.
- [ ] `./record.sh -t test` 를 10초 돌리고 Ctrl-C. `결과: 정상` 과 분당 용량을 본다. `ros2 bag record --help` 의 옵션이 다르면 `record.log` 에 오류가 남는다.
- [ ] `ros2 bag info ~/bags/<세션>/bag` 에 `/teleop/events` 와 `/tf_static` 이 있는지.

## 로봇 없이 연습·시험

```bash
python3 tests/test_all.py              # 단위 시험 28개 (키 해석, 유지형 판정, 안전장치, bag 점검, 비밀값 검사)
python3 tests/test_integration.py      # 가짜 ROS 로 텔레옵·기록·UAV 수집 전체 흐름 16개 (리눅스, 약 1분 30초)
PYTHONPATH=tests/fake_ros python3 teleop_keys.py     # 가짜 ROS 로 키 조작 연습
```

## 만들지 않은 것

- 키를 동시에 받는 창(pygame) 방식, bag 재생·압축 스크립트, 기록 프로필(`match`, `nav`)은 없다. 필요하면 `ros2 bag play` 를 직접 쓰되, 살아 있는 로봇 앞에서는 `--remap /cmd_vel:=/bag/cmd_vel` 을 붙인다.
- `/cmd_vel` 이 끊긴 구간 찾기는 없다. `bag_check.py` 는 평균 주기만 본다.
