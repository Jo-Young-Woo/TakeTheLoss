# 2차 사전교육 자료 + 경기 서버 API 반영 (2026-11-06)

근거 두 가지를 코드에 반영했다.
1. **「2차 사전교육 자료」 PDF(`prestudy(2)_2026MAICON.pdf`, 20쪽)** — 로봇 ROS2 API(쪽수는 PDF).
2. **「참가팀 단계별 API 안내」(`game-server-api.html`, 2026-11-04 스냅숏)** — 경기 서버 HTTP API(단계는 문서의 1~11단계).

> **상태: 로봇·서버 실물에서는 실행해 보지 못했다.** 로봇 쪽은 `FakeTransport`(rclpy 없음), 서버 쪽은 문서 규칙을 흉내 낸 모의 서버로만 시험했다.
> 문서에 없어서 정한 값은 `[가정]`이고 `field/FIELD_CHECKLIST.md` 에 확인 항목으로 올라가 있다.

## 1. 구조 변경

| 영역 | 이전 | 이후 |
|---|---|---|
| 로봇 HAL | `NotImplementedError` 스텁 | `hal/wingbot.py`(로직) + `hal/mil_api.py`(순수 규칙) + `hal/ros_mil.py`(rclpy 전송) |
| 서버 링크 | 스텁 | `link/http.py`: `/me` 폴링·긴 폴링·제출·핸드셰이크·드론 |
| 제출 JSON | 추정 | `link/schema.py` `*_to_server`: 내부 표기 → 서버 표기 한 곳에서 변환 |
| 진입점 | `scripts/run_real.py` | + `team/team_node.py`(= `~/team/team_node.py`) |
| 설정 | — | `config.HalCfg`(`cfg.hal.*`), `Config.api_issues()` |

## 2. 로봇 ROS2 API (PDF) → 코드

| PDF 규칙 | 쪽 | 코드 |
|---|---|---|
| `/cmd_vel` linear ±0.6 m/s, angular ±3 rad/s(+좌회전), **약 10 Hz 반복, 0.5 s 끊기면 자동 정지**, 여러 노드 동시 발행 금지 | p15 | `limit_cmd_vel` 로 한계 적용, `pump()` 가 `cmd_hz`(10 Hz)로 마지막 속도를 반복, Agent 가 `cmd_stale_s`(0.3 s) 넘게 안 주면 HAL 이 0, 정지(0)는 즉시 발행. Nav2 는 `/cmd_vel_nav2` 라 단독 발행자 유지 |
| `/odom` 20 Hz, **`qos_profile_sensor_data`**, 출발점 기준 위치 | p15 | 센서 QoS 로 구독. (예전 템플릿은 기본 RELIABLE 이라 BEST_EFFORT 발행자와 **연결이 안 됐을 것**) |
| `/mil/gimbal` 서비스, pan/tilt 는 **각도가 아닌 위치값** −100~100, **1단위 = 팬 0.5° / 틸트 0.15°**, pan + = **오른쪽**, `ok` 는 접수일 뿐, speed 1~270, acc 권장 20 | p14 | `quantize_pan_tilt`(부호 반전·정수 반올림·±100 한계), 비동기 호출(루프를 막지 않음), 응답 `ok/locked/None` 처리 |
| 위 서비스엔 **위치 피드백이 없다** | p14 | `GimbalModel`: 접수 시각 + 지연 후 `speed` 로 움직인다고 보고 현재·촬영 시각의 팬/틸트를 추정(`get_pan_tilt`, `Frame.pan/tilt`). 지연·속도는 실측 필요 |
| `/mil/fire`: 1회 5 s 발광, **시작 후 10 s** 재발사, 응답 `fired/empty/locked/reloading` + `left_ms`, 피격 후 10 s 조준·발사 제한, HP 0 이어도 발사 가능 | p16 | `FireModel`(탄약·발광·재장전 추정; **탄약 토픽이 없어** 가득에서 시작해 `empty` 응답으로 보정), 서비스 응답 대기는 `fire_wait_s`(0.06 s)만 — 늦으면 낙관적 True 후 모델이 정정 |
| `/mil/match` mode(practice/mission), cmd(idle standby start pause resume report stop), **`control = practice or cmd ∈ {start,resume}`** | p11·12 | `control_allowed`. mission 에서 잠금이면 주행은 무시·조준/발사 locked → HAL 은 0 속도·서비스 미호출, Agent 는 `SAFE_STOP(match_locked)` |
| 일시정지: 레이저 꺼짐, 모터·팬틸트 정지, **재개 시 스스로 안 움직임 → 명령 다시 전송** | p12 | 일시정지 때 발광 추정 취소, 재개 때 팬틸트 강제 재전송·속도 0 에서 시작, Agent 가 새 센서 표본 뒤 복귀 |
| `/mil/state`(2 Hz) hp·hp_max·battery_pct, QoS depth 1 + TRANSIENT_LOCAL | p10·11 | `HitModel`: HP 감소 = 피격 → 10 s 잠금(`disabled_left`). 연습 모드의 HP 회복은 피격으로 세지 않음. 배터리 `battery_low_pct` 아래면 깊이 영상 처리 중단(배터리 교체 불가, p6) |
| `/mil/oled` 4줄×21자(영문·숫자·기호), 전송 간격 ≥0.5 s, 10 s 뒤 기본 화면 | p13 | `OledScreen`: 바뀐 줄만 한 번에 한 줄, 8 s 마다 갱신, 비ASCII 는 `?`. Agent 가 1 Hz 로 팀·모드 / HP·배터리 / 경기 시각·상태 / 제출 상태 표시 |
| `/image_raw` 640×480 BGR, **10 FPS 이하 권장**, 팬틸트 카메라(깊이 아님) | p17 | 10 FPS 이하만 변환(numpy, cv_bridge 불필요), 깊이는 별도 토픽 |
| `/imu/data` 20 Hz, 기울기·충격 시 `Twist()` 정지 | p17 | roll/pitch → `tilt_stop_deg`(25°, 해제 15°) 넘으면 `SAFE_STOP(tilt)`, 충격 g 는 기록만 |
| `team_node.py`: standby 때 자동 실행, 한 경기 후 report→stop 이면 종료, 비정상 종료 시 경기당 **최대 3회** 자동 재실행, step() ≤ 0.1 s, AI 추론 별도 스레드 | p18 | `team/team_node.py`(1경기 후 종료, Nav2 launch 자식 프로세스), `--restore`: 경기 시작 벽시계 시각 저장 → 재실행 시 **경기 시계만** 복원. `Agent.tick()` 시간 측정(`tick_ms_max`, `tick_over_budget`) |

**설정이 API 한계를 넘는지**는 `Config.api_issues()` 가 알려 준다(`run_real` 시작 시 경고). 현재 `robot.pan_limit=50°`, `tilt_limit=15°` 는 ±100 단위와 정확히 일치, `v_max=0.45`, `w_max=2.2` 는 한계 안.

**조준 허용오차:** 팬 1단위 0.5° 라 양자화 오차가 ±0.25° — `aim.sensor_min_tol_deg` 를 0.25→0.30° 로 올렸다.

## 3. 경기 서버 API (문서) → 코드

| 문서 규칙 | 코드 |
|---|---|
| 모든 창구 `/api/report/…` + **임무코드**(영문+숫자 8자리, GET `?mission_code=`, POST 본문). 리허설은 `/api/rehearsal/…`. 임무코드는 로그에 찍지 말 것 | `HttpServerLink(base, mission_code, rehearsal=…)`, 오류·상태 문구에서 임무코드를 가린다. 코드는 `MAICON_MISSION_CODE` 환경변수 권장 |
| **1단계** `/me` → phase=ready & ready=false 면 `POST /ready`. 모델·필수 센서 초기화 **뒤**에. 409·통신 실패는 2 s 뒤 반복. 안 보내면 **그 경기 0점** | `ready_handshake(init_ok)`: LiDAR 수신 확인 뒤 전송(`--ready-timeout` 30 s 넘으면 경고하고 그래도 전송). 이미 `ready:true`(재실행)면 바로 통과 |
| **2단계** `GET /order?wait=120` 긴 폴링 → `tactical_command`. 클라이언트 타임아웃 > wait. 메인 루프를 막지 말 것 | 전용 스레드가 기다렸다가 `command` 메시지로 낸다. `poll()` 은 큐만 비워 **블로킹 없음** |
| **9단계** `GET /target?wait=120` → `objective`(예 `RED_MAINTENANCE`), `objective_label` | `base_command` 메시지(`data.objective`). Agent 는 보고값으로 거점 종류를 정하고(없으면 이름 문장 해석), 우리 진영 거점이면 경고 |
| **3·10단계** 시작 신호가 곧 0:00. 끝은 **팀 시계가 아니라 report 신호**(로봇 `/mil/match` report, 서버 `/me` phase=reporting). 서버는 남은 시간을 주지 않음 | Agent 가 **로봇 `cmd=start`** 를 0:00 으로(서버 phase=running 은 보조, 먼저 온 쪽이 래치). 정지는 서버 phase 와 로봇 `report/stop` 중 먼저 온 것 |
| **5단계 M1** `POST /interpretation` `{target:"RED_RADAR", search_sectors:["R-S1","CM"], base:"RED_SUPPLY_DEPOT"}`. 진영 접두어는 **그 시설이 놓인 진영**(상대). **다시 보내면 통째로 바뀜** | `m1_to_server`: 상대 진영 접두어, `COMMON→CM`, 서버에 없는 NATO 이름 제거, 아는 항목만(없는 항목을 틀린 값으로 채우지 않음) |
| **7단계 M3** `POST /detection` 섹터별 `{type,count}`, 6종(`military_truck mortar tank missile ammo_box drum`), 섹터는 이름. 마지막 제출 채점, **진행 중에만**(5:00 뒤 409). 채점 구역 = 상대 3구역 + CM | `m3_to_server`: 채점 구역 4개를 항상 포함(없으면 `[]`), 종류 변환, count ≤ 99. **첫 제출을 60 s 로 앞당기고 15 s 마다 갱신**(`m3_first_s`, `m3_resubmit_s`; 이전엔 235 s 한 번) |
| **11단계 M6** `POST /situation` `ranking:["R-2","R-1","R-3"]`, 상대 진영 호출부호만(자기 진영이면 422), reporting 에서만 | `m6_to_server`: `R-UGV-2→R-2`, 자기 진영·중복 제거 |
| **4단계 드론**: JPEG, 헤더 `X-Frame-Interval-S`(1/2/5), `X-Destroyed`, 3개 파괴면 403 | `get_uav()` 가 새 장마다 `UavFrame(interval_s, destroyed)` — `UavMonitor` 가 주기 변화를 읽는다. **UGV 위치 검출은 하지 않는다(미구현)** |
| 오류 401(코드)·409(단계)·422(형식) | 모두 `False` 로 돌려 `Submitter` 가 재시도(409 는 단계가 바뀌면 통과), 사유는 `health()` 로 로그 |

## 4. 코드가 바뀐 동작 (눈여겨볼 것)

- `RobotStatus` 에 `control / match_mode / match_cmd / hp / hp_max / battery_pct / roll / pitch / accel_g` 추가(시뮬은 `None` = 제한 없음).
- `LaserManager.ready()` 가 `control` 을 본다.
- `MatchLatch` 의 START 는 서버 메시지 또는 로봇 `cmd=start` 중 먼저 온 것 하나.
- M5: 서버가 주는 `objective` 보고값을 우선, 문장 해석은 대체.
- `CommandParser`: `중앙진지/공통/CM` 별칭, **부정(제외) 섹터를 목록에서 제거**, 팀을 알면 `나머지/상대진영` 을 상대 3구역 + 공통에서 제외분을 뺀 목록으로 펼침. (서버 문서의 예문 「상대 중앙진지를 제외하고 나머지 상대진영 탐색 구역…」 형태를 가정 — **실제 문장은 리허설 `/order` 로 확인**)
- 일시정지 중 경기 시계를 멈춘다(`hal.freeze_clock_on_pause`, [가정]).

## 5. 아직 모르는 것 / 미구현 (정직하게)

| 항목 | 상태 |
|---|---|
| LiDAR(D500) 토픽 이름 | PDF 에 없다. `hal.scan_topic="/scan"` 가정 |
| `/mil/*` 메시지·서비스 **패키지/타입 이름, 필드 이름** | PDF 에 `OledLine(line,text)`, `Gimbal.Request(pan,tilt,speed,acc)`, `Fire.Request()`, 응답 `message`/`left_ms` 만 있다. 타입은 실행 중인 토픽/서비스에서 자동 탐색, 안 되면 `hal.type_override`. 응답 필드는 `message/result/status` 순으로 시도 |
| 로봇이 TF `odom→base_link`, `base_link→laser` 를 내는가 | 자료에 `/odom` 만 있다. **변환 노드를 만들었다**: `scripts/odom_tf_bridge.py`(`maicon26/hal/odom_tf.py`)가 `/odom` 으로 `odom→base_link` 를 낸다. 정적 TF(`base_link→laser/imu/camera`)는 `robot_description.launch.py`(URDF). 아래 "TF 구성" 참조 |
| HP 0 에서 주행 정지, 일시정지 시간을 서버가 경기 시간에서 빼는지 | 문서에 없음 → `hal.drive_pause_s`, `hal.freeze_clock_on_pause` 가정 |
| 드론 영상에서 UGV 위치 검출 | 코드 없음(이전과 동일). 주기·파괴 수만 사용 |
| 재시작 복원 | 경기 시계만. M1 명령·표적·객체 집계·SLAM 앵커는 복원 못 함. 서버는 `/order` 를 몇 번 물어도 같은 내용을 주므로 명령은 다시 받는다(문서 2단계) |
| M1 JSON 형식, M6 동점 처리, M5 센서 앞 체류 시간 | 문서가 「임시/잠정/결정 중」으로 표시 |
| 카메라 160° 광각 왜곡 | `/image_raw` 640×480 이 보정된 영상인지 모른다 → `camera.K` 보정 필요 |
| 세 팀 연합·실제 상대 | 서버 문서엔 상대 정보를 주는 API 가 없다 |

## 5-1. TF 구성 (로봇이 TF 를 안 낼 때)
```bash
ros2 run tf2_ros tf2_echo odom base_link          # 먼저 확인: 이미 나오면 아래 odom_tf 는 필요 없다
ros2 launch ros2/launch/robot_description.launch.py          # 정적 TF: base_link→laser/imu/camera/depth (URDF; 로봇이 같은 정적 TF 를 내면 쓰지 말 것)
ros2 launch ros2/launch/maicon_nav.launch.py odom_tf:=auto   # 동적 TF: /odom → odom→base_link. auto = 이미 다른 발행자가 있으면 쉰다, force = 항상
```
WSL(ROS2 Humble)에서 실제 rclpy 로 확인: ① 발행자가 없으면 `/odom` 위치·방향이 그대로 TF 로 나간다, ② 다른 노드가 같은 TF 를 내면 쉰다(중복 발행 0), ③ `--force` 는 항상 발행.
**한계:** 바퀴 오도메트리 그대로라 `map→odom`(SLAM)이 보정해야 한다. LiDAR 장착 위치·방향(`rig.lidar_xyz`, URDF)은 아직 가정값이다(`calib_run.py lidar_square`로 측정).

## 6. 현장에서 가장 먼저 할 일 (순서)

1. `ros2 topic list -t`, `ros2 service list -t` — `/mil/*` 타입·`/scan`·`/oak/*` 이름. 자동 탐색이 안 되면 `hal.type_override`.
2. 연습 모드: `pan=+50` 이 오른쪽인지, `angular.z=+1` 이 왼쪽 회전인지(`hal.pan_sign`).
3. `ros2 topic echo /odom`, `ros2 run tf2_ros tf2_echo odom base_link` — TF 유무. 없으면 5-1 의 `odom_tf:=auto` + URDF 정적 TF.
4. 팬틸트 명령 후 영상이 움직이기 시작하는 시간·속도 → `hal.gimbal_latency_s`, `hal.gimbal_speed`.
5. 리허설 경기(`--rehearsal`): 준비완료 → `/order` 문장 → M1 정답 비교(포털) → M3/M6 → 시간표(0:00, 4:00, 5:00, 5:30).
6. `python scripts/field_exp.py status`, `field/FIELD_CHECKLIST.md`.
