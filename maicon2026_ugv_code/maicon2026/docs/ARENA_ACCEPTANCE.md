# MAICON 경기장 재구성·API 인수 기록

기준일: 2026-11-09. **배포 후보 ZIP을 별도 경로에 풀어 ROS Humble 환경의 전체 시험1,352개를 통과했고, 정적 검사·규칙 추적·새 ROS 패키지 빌드도 완료했다.** 실제 경기장 표시·바닥 정합·근접면 방향·제어·여섯 거점 빔·영상 전달과60초 RTF도 확인했다. 실제 디스플레이 표시 FPS, 요청 대상 장비, 새 Gazebo의 자율 M3/Nav2 전체 경기와 실물 검증은 남아 있다. 배포 근거는 [RELEASE_VALIDATION.json](RELEASE_VALIDATION.json), 실행 명령은 [패키지 실행 안내](../ros2/maicon_arena_gz/docs/runtime.md)를 따른다.

## 기준 자료와 좌표

| 구분 | 기준 |
| --- | --- |
| 새 경기장 정본 | `ros2/maicon_arena_gz/config/arena_spec.yaml` — 제공된 실측 사양을 그대로 보존 |
| 기본 배치 정본 | `config/layouts/layout_default.yaml` — 제공본 보존, 별도 시드 생성본과 구별 |
| 주행면 | **6.00 × 4.80 m**, 0.20 m 격자 |
| Gazebo 좌표 | 바닥 중심 원점, +X RED 쪽, +Y 1번 줄, +Z 위 |
| 기존 항법 좌표와 연결 | `ArenaGeometry.from_spec`와 명시적 `world_offset`으로 양의 격자 좌표에 변환. Gazebo 원점을 임의로 옮기지 않음 |
| 공식 API 기준 | `docs/source/MAICON2026_game-server-api_p0-p12.html`, 2026-11-04 스냅숏 |
| 원문 식별 | 115,114 bytes / SHA-256 `834c7a098a8fbc6b6b528840f8d8b81b194604aa443f05081106e3b9d2699c88` |
| 원문 우선순위 | 이후 배포되는 룰북과 SERVER ↔ UGV 통신 규약이 안내서 스냅숏보다 우선 |

기존 `Config()` 기본 경기장과 자동 생성된 [현장 체크리스트](../field/FIELD_CHECKLIST.md)의 6.12 × 5.00 m 수치는 이전 도면 근사다. 새 YAML의 주행면 치수와 혼용하지 않는다. 기존 문서는 과거 기록으로 보존하며, 새 환경의 실측 체크는 아래 표를 사용한다. 이전 B2/3v3 영상, 위치 오차, M3 성공 수치 역시 그 당시 경기장과 카메라에 대한 결과다. 새 경기장 성능을 입증하지 않는다.

## 경기장 A~E 인수 상태

| 항목 | 현재 확인한 것 | 상태·남은 검증 |
| --- | --- | --- |
| A 치수·배치 | 실제 SDF XML·메시·이미지 자원을 읽는 검사 결과 `ok=true`, 오차 허용 ±0.001 m. 정적 collision 28 / visual 57 / texture 22 / light 1. 기본 큐브 20개·9덩어리·표적 6개·M3 노출 면 27개 | 정적 검사 통과. 최종 생성물을 바꾸면 같은 검사를 다시 실행 |
| A 무작위 배치 | 20개 시드 회귀에서 20 cm 격자, 모든 7섹터, 덩어리 간·벽·고정 설비 이격, 거울 대칭 표적, 숨은 면, 정답 재계산, 접근 가능한 관측 지점을 확인 | 단위·기하 검사 통과. 접근 가능은 차체 이동 기하이며 실제 사진 인식 성공 보장은 아님 |
| B 실제 화면·정합 | CPU Ogre2 센서 + GPU Ogre1 GUI에서 경기장·6대 표시 확인(`gpu_gui_verified.png`).6m 카메라의27개 독립점: 지정 카메라 기하 직접 투영 RMS5.083mm·최대8.424mm. UAV23개 독립점 최대6.439mm | 바닥≤1cm 정합 및 전체 표시 통과 |
| B 표적·M3 근접면 | 실제 Gazebo 표적·M3 면을 각3프레임 촬영. 정방향 NCC0.98595/0.98150, 가장 강한 반전·회전 대안0.59467/0.55484 | 두 면 모두 위아래·좌우 방향 통과. 모든 면 또는 인식 정확도 검증은 아님 |
| C standard headless | 60.00365초 평균RTF **0.999139**, 최저1초창 **0.993836**, 첫 스텝 **2.935초** | 이 장비에서 평균≥0.98·최저≥0.90·기동≤10초 통과. RTX5070 실측은 별도 |
| C lite headless | 60.00020초 평균RTF **0.999997**, 최저1초창 **0.999611**, 첫 스텝 **2.574초** | 이 장비에서 평균≥0.90·기동≤10초 통과. 내장GPU 노트북 실측은 별도 |
| C 별도 GUI 포함 측정 | standard 평균RTF0.999059·최저0.994690·기동2.596초, lite0.999998·0.999620·2.635초. GPU GUI 렌더 루프 평균/최저 standard137.23/96Hz, lite143.4/121Hz | 과거 GUI 포함60초 결과 보존. 렌더 루프는 display-present FPS가 아니므로 실제 표시≥30FPS 검증을 대신하지 않음 |
| D 실제 제어 | 실제6대 시작 위치·센서 수신.3개 주행 지점 도달, 실제 pan−0.39278/tilt0.10475rad, 발사·재장전·탄약8→7, watchdog 정지 후0m 이동(`control_trial.json`) | odom/MIL 통합 통과. 실제 표적 연속 명중·자율 M3/Nav2 전체 경기 성공은 아직 별도 |
| D 실제 여섯 거점 빔 | B-2 단독, `set_pose`→Gazebo 자세→ROS 심판 이벤트에서 B-1~R-3 차단/해제6/6 통과(`base_beams_live.json`) | ready 단계·점령 이벤트0·원래 자세 복원. 경기 중 M5 점령 및 자율 주행 검증과 구별 |
| D B-2 단독 준비 화면 | 실제 모델은 B-2 한 대, 심판 ready.8초 동안450점 모두 유효한 LiDAR80회·ego영상80장·UAV8장·odom·IMU 수신(`single_ugv_ready.json`) | 사용자 조작용 GUI 준비 상태 확인. 경기 시작·자율 임무 성공을 의미하지 않음 |
| D 영상·HTTP | 약20초 Gazebo/ROS 모두 ego199프레임·UAV20·검증카메라20. 로컬 `/me`·실제 JPEG200와6개 헤더 확인 | 실제 ROS→HTTP 경로 통과. 공식 실서버·무선망 품질 검증은 아님 |
| E 자동 회귀·배포 | 독립 추출한 ZIP 후보 v2에서 **1,352개 통과·실패0·skip0**,1065.26초. pyflakes0, 추적97/97개 S-ID·159개 spec 마커.611개 멤버 해시 일치·시험 중 소스 변경0, 새 ROS 패키지2개 빌드22.7초 | 소프트웨어 배포 검증 통과. fixture의0 나눗셈 경고1개를 보존. 실물 및 새 Gazebo 자율 전체 경기 성공을 뜻하지 않음 |

정적 개수와 치수의 직접 근거는 [maicon_arena.check.json](../ros2/maicon_arena_gz/worlds/maicon_arena.check.json)이다. 생성기 메타데이터만 비교한 결과가 아니다.6m 카메라의 직접 투영 검사는 fitted homography가 바닥 배치 오차를 흡수하지 않게 했으며, 맞춤 변환의 별도 독립점 최대오차는5.873mm다.

근접면 근거는 [closeup_report.json](../ros2/maicon_arena_gz/validation/closeups/closeup_report.json), [표적 원본/촬영 비교](../ros2/maicon_arena_gz/validation/closeups/target_source_vs_capture.png), [M3 원본/촬영 비교](../ros2/maicon_arena_gz/validation/closeups/m3_source_vs_capture.png)다. 비교 그림은 왼쪽 원본·오른쪽 실제 촬영이며 원본을 촬영 영상으로 대체하지 않았다. 정본의 표적 `cube_0_0` E면과 트럭 `cube_14_0` E면을0.65m에서 촬영했다. 충돌 없는 임시 카메라만 추가 후 제거했고 로봇·경기장 배치는 바꾸지 않았다. 방향 오류를 일부러 주입한 회귀도3개 통과했다. [여섯 거점 원본 보고서](../ros2/maicon_arena_gz/validation/base_beams_live.json)는 익명 빔 이벤트12개(차단6·해제6)를 기록하며 ready 단계에서 점령 이벤트가 발생하지 않았음을 확인했다.

요구 C의 headless 근거는 [standard](../ros2/maicon_arena_gz/validation/performance_standard_headless_shm64.json)·[lite](../ros2/maicon_arena_gz/validation/performance_lite_headless_shm64.json)이다. **WSL2 Ubuntu22.04의 CPU Ogre2 + SHM64**, GUI 꺼짐,6대(ego1+더미5)·상공카메라 포함·검증카메라 제외,5초 워밍업 뒤60초 조건이다. 두 실행 모두 정상 구간 UAV60장을 받았다. 기동·워밍업을 포함한 별도70초 헤더 검사는 standard ego RGB/BGR659/659장·UAV66장, lite335/334장·UAV67장을 기록했다. 시각 중복·역행0, 최대 간격 standard ego0.1초/lite0.2초/UAV1초였다. 전체/개별 pytest 부하는 측정 중 변동했고 정확한 겹침 시간은 계측하지 않았다. 다른 Gazebo는 없었다. 호스트에는 RTX4060Ti가 있지만 센서 렌더링은 CPU이며 요청 RTX5070급/내장GPU 장비 실측을 대신하지 않는다.

앞선 화면 포함 성능 근거 [standard](../ros2/maicon_arena_gz/validation/performance_standard_hybrid_shm64.json)·[lite](../ros2/maicon_arena_gz/validation/performance_lite_hybrid_shm64.json)는 별도로 보존한다. 구성은 **CPU Ogre2 센서 서버 + GPU Ogre1 GUI + SHM64**다.5초 워밍업 뒤60초,6대·상공카메라 포함·검증카메라 제외 조건이며 별도 pytest 프로세스1개가 함께 실행됐다. GPU GUI의60개1초 버킷 `PreRender` 빈도는 실제 표시 FPS와 구별한다. 이전 GPU Ogre1 센서 붕괴·GPU Ogre2 텍스처 실패·기본 전달 설정의 낮은 RTF도 원본 보고서로 보존한다.

단독 준비 상태는 [실제 GUI 화면](../ros2/maicon_arena_gz/validation/arena_single_ugv_ready.png)과 [8초 센서 보고서](../ros2/maicon_arena_gz/validation/single_ugv_ready.json)를 따른다. 심판의 진영 전체 상태 표와 실제 스폰 모델 수를 구별하며, 이 보고서의 실제 UGV 모델은 `ugv_B2` 하나다.

[제어 시험 결과](../ros2/maicon_arena_gz/recordings/control_20261109_v2/control_trial.json)와 같은 폴더에 실제3인칭17.2초·1인칭17.5초 MP4가 있다. 전 구간 디코딩을 확인했다.3인칭은 원본1Hz18장을5fps로 유지 표시한86프레임(반복68),1인칭은 원본10Hz175프레임이며 캡처시각을 별도로 보존한다. 영상은 소스 ZIP에서 제외되는 별도 제공물이다. 이 시험은 odometry/MIL 제어 점검이며 자율 M3/Nav2 채점 경기로 표시하지 않는다. 실제 API 근거는 [api_live_probe.json](../ros2/maicon_arena_gz/validation/api_live_probe.json)과 `api_uav_live.jpg`이다.

## API·G12 선택 검증

추적표는 **97개 S-ID: 구현 70 / 규칙 27 / 해당없음 0**을 코드와 실제 시험 이름에 연결한다. 독립 추출한 ZIP 후보에서 기계 검사 **97/97개 S-ID·159개 spec 마커**, pyflakes **경고0**을 재확인했다. 이 수는 문서 추적 범위이며 시험 통과 개수가 아니다. 원문에서 생략된 오류 문구는 알려진 접두만 비교하고 `ERROR_DETAILS_UNSPECIFIED`에 남긴다. `422 detail`의 공식 목록 형태와 클라이언트의 추가 방어적 문자열·사전 처리를 구별한다.

| 선택 실행 | 확인 결과 | 한계 |
| --- | --- | --- |
| G6 Agent → HttpServerLink → MockServer | 6개 통과. direct/Async 경로, BLUE/RED, reporting 재시작. STOP 이후 새 M1/M3 없음, 가짜 시계 1초 이내 M6 | 외부 서버나 실제 무선망 시험 아님 |
| G12 및 관련 통합 선택 | `test_spec_field_policy`, `test_spec_conformance`, `test_nav_stack`, `test_exploration_continuity`, `test_real_runtime`, `test_gazebo_api`, `test_mockserver_spec`: **138개 통과, 112.46초** | 선택 목록의 합계. 다른 실행과 중복되므로 누적 합산하지 않음 |
| S9.7 자기 거점 보호 | BLUE/RED 경계 시각, YAML 좌표, 초기 내부 탈출·깊은 진입 차단, 회전 모서리, Nav2 앵커 변환·해제·미지원 대체, 최종 출력 제한 | 실제 ROS Nav2 KeepoutFilter 수신·주행과 실물 제동거리 검증 미완료 |
| 전체 회귀 중단 기록 | 172개 통과·2개 실패 뒤 사용자 요청으로 중단. `tmp/full_pytest_20261109.log` 보존 | 과거 이력. 후속 수정과 최종 v2 전체 통과를 아래에 구분 |
| 물리·안전 복귀 선택 재검증 | **92개 통과·1개 실패, 192.69초**. 기존 전방 차단/M5 및 낮은 장벽 시험은 assertion 변경 없이 통과. 추가한 직접 충돌8개·안전 복귀8개 포함 | 과거의1개 실패는 UAV tracker spy가 기존 `self_ident` 인자를 받지 못한 fixture 오류. 호출을 맞추고 실제 식별자 assertion을 추가했으며 최종 전체에서 통과 |
| 재구성 후 첫 전체 실행 이력 | **23개 실패·1263개 통과·경고1, 1071.71초**. `tmp/full_pytest_arena_20261109.log` | 수정 전 결과를 보존. 탐색2개·현장 fixture1개·로깅/HTTP20개 실패이며 최종 전체 결과로 표시하지 않음 |
| 현장 fixture 후속 선택 검사 | **31개 통과** | 이전 현장 체크리스트 fixture 수정 후 관련 선택 검사. 전체 회귀가 아님 |
| 로깅 원인 분리: ROS 미로드 | **202개 통과**. 넓은 별도 선택1155개 통과·1개ROS 미설치 skip | 수정 전 환경 비교. ROS 환경을 source한 sentinel3개는 같은 실패 재현. 서로 합산하거나 수정 후 통과로 읽지 않음 |
| 로깅 환경 격리 후 ROS 검사 | **155개 통과, 8.98초**(`tmp/logging_ros_fixed.log`). 실제 ROS 비전파 로거의 초기화 전/후 설치 검사2개 포함 | 미사용 두 launch-testing 플러그인만 pytest 설정에서 제외. 기존 assertion·production 로거 설정 보존. 전체/ZIP 회귀와 구별 |
| ZIP 후보 v1 전체 실행 | **1개 실패·196개 통과, 106.46초**에서 중지 | 과거의 출현 전 적 UGV 결함을 확인한 이력. 수정 후 v2 전체에서 원래 회피 assertion 통과 |
| ZIP 후보 v2 전체 실행 | **1,352개 통과·실패0·skip0·경고1,1065.26초** | `/home/jojo0429/maicon_release_validation_20261109_v2` 독립 추출 경로, ROS Humble. 경고는 기존 복도 fixture의 배열0 나눗셈이며 숨기지 않음 |

위 실패·중단 결과는 당시 스냅숏의 이력이며 최종 ZIP 후보 v2 전체 통과로 후속 검증을 완료했다. 로깅20개 실패는 Humble의 자동 pytest 플러그인이 `LaunchLogger(propagate=False)`를 기본 로거로 설정하여 root 기반 로그 수집을 우회한 결과다. `pytest.ini`는 사용하지 않는 두 launch-testing 플러그인만 제외한다. 실제 ROS의 비전파 로거·개별 스트림 핸들러에서 비밀값/예외 URL 가리기와 설정 보존을 별도 자식 프로세스로 검사하며 기존 전송 assertion을 완화하지 않았다.

ZIP 후보 v1에서 확인한 회피 실패는 나중에 등장하도록 예약한 적 UGV가 그 전부터 기본 위치에 남아 UAV·카메라·LiDAR와 충돌 세계에 노출되던 결함과 관련됐다. `spawn_rel` 적은 실제 배치가 성공하기 전에는 관측·물리 세계에서 제외하고 이후 추가하도록 수정했다. v1 실패 이력과 회피 기준을 보존했으며 v2 전체에서 해당 시험도 통과했다.

두 기존 실패는 전방 차단 UGV 시나리오의 M5 지연 한도(기준 242.7초, 회피 사용 272.6초, 허용 +20초)와 낮은 장벽 builtin 시험의 이동거리 비교(깊이 사용 22.0 m, 미사용 25.0 m)다. 해당 충돌·피격 비교 조건은 통과했다. 낮은 장벽 시험은120초 종료이므로225초 자기 거점 규칙과240초 M5 게시 변경이 원인이 될 수 없다.

낮은 장벽은 별도 프로세스에서도 같은 배치 해시로 재현되었다(`tmp/barrier_off.json`, `tmp/barrier_on.json`). 수정 전 깊이 미사용은93충돌·25.0 m, 사용은0충돌·22.0 m다. 경로를 확인하니 미사용 로봇이19.75초에(x=1.752, y=3.292)에서 장벽 내부를 통과했다. 기존 Python World의 `_resolve_rect`는 장애물 크기와 관계없이 중심이0.6 m 밖이면 SAT검사를 건너뛰어3.9 m 장벽의 일부를 빠뜨렸다. 이는 시험 상태 오염이 아니라 긴 장애물 물리 판정 결함이다. 실제 장애물 경계와 회전 차체 경계로 검사하도록 수정했다.

전방 차단도 독립 실행에서 같은 실패가 재현됐다(`tmp/blocker_off.json`, `tmp/blocker_on.json`). M5 진입 중4회째 피격으로 정지한 뒤 무조건 EXPLORE로 복귀하면서 회피 제어가 구동권을 가져가255.95~266.05초에 목표 접근이 미뤄졌다. 안전 복귀 시 현재 M5 게시·시간으로 GOAL/STAGE/EXPLORE를 재평가하고 같은 틱의 회피 맥락을 갱신했다. 두 기존 시험은 수정 후 **원래 assertion 그대로 통과**했고 후보 v2 전체에서도 통과했다. 회피 전략·임계치는 바꾸지 않았다.

## G1~G15 판정

`fixed`는 해당 변경 코드와 관련 시험이 존재하며 배포 후보의 전체 회귀를 통과했다는 뜻이다. 실서버·실물·새 Gazebo 자율 전체 경기의 성공 보장은 아니다. 최종 시험 skip은0이며 아직 실행하지 않은 현장 검증은 구분해 적는다.

| 항목 | 판정 | 현재 구현과 남은 확인 |
| --- | --- | --- |
| G1 received 검증 | fixed | 중앙 `equal_received`: M1/M3 순서·대소문자 정규화, M6 순위 보존. 불일치는 성공 처리하지 않고 제한 재시도 |
| G2 accepted:false reason | fixed | 게시 전·미편성·누락·리허설 없음 분류, 간격과 상태 변경 경고. 알려지지 않은 원문 접미는 창작하지 않음 |
| G3 409 분류 | fixed | 영구 거부는 해당 보고 종류를 중단, 단계 오류는 단계 변화에 맞춰 재시도 |
| G4 UAV 헤더 | fixed | 캡처 시각·간격·카메라·연습 모드·진영·파괴 수 처리, `/me` 편성 우선, 오래된/중복 프레임 배제. 실제 새 카메라 보정은 별도 |
| G5 비밀값 입력 | fixed | 환경·파일·비표시 입력, 8자리 검사. 비밀값 명령행 인자 금지. 실전 자격 정보는 현장 제공 |
| G6 STOP·M3·M6 | fixed | STOP 틱의 M1/M3 취소, 225초 이후 변경 M3 전체본 1초 간격, 즉시 M6. HTTP 경로 6개 회귀 통과 |
| G7 모의 서버 | fixed | FakeClock·단계·서버 수신 순서·준비·오류·드론 저하·리허설·결함 주입·6개 팀 M6. 로컬 Gazebo 어댑터가 실제 JPEG 공급, 정답을 참가팀에게 제공하지 않음 |
| G8 추적/적합성 | fixed | 독립 추출 ZIP에서97/97개 S-ID·159개 spec 마커 및 전체1,352개 시험 통과 |
| G9 long polling | fixed | wait 기본20초는 허용 범위의 선택, 최대120초, HTTP timeout은 wait+5초 이상. 제어루프 밖 실행 |
| G10 M3 대상 | fixed | 사진6종만, UGV/비대상 그림 제외, 같은 종류 합산·0~99 범위, 관측 면 중복과 미확정 기록 차단 |
| G11 문서 | fixed | 전체 API 요약·추적·통신·전략·인수 기록과 최종 검증 수치·배포 경로 저장. 후보와 문서 갱신 후 최종 ZIP의 검증 범위 구별 |
| G12 잠정값·자기 거점 | fixed | 임시 wire 형식 중앙화, M5 dwell 기본0, M6 동점 특별 규칙 없음, 미안내 LLM 주소/키/형식 슬롯. `OwnBasePolicy` 경로·차체 출력 제약 구현. 실물 주행 검증 미완료 |
| G13 노출 방지 | fixed | 로그·예외·URL·echo·대소문자·길이 제한과 배포 누출 차단 회귀를 전체 시험에서 통과. ZIP의 비밀 파일·로그·캐시·영상 제외 정책 및 멤버 해시 검증 완료 |
| G14 재시작 | fixed | 동기화된 HTTP Date/KST 게시시각으로 경기 시계 복구. reporting은 정지 상태에서 M6. **SLAM 지도·앵커 없는 실전 중간 주행 재시작은 지원하지 않고 차단** |
| G15 상수 | fixed | 시설12·객체6·섹터7·호출부호·진영 접두·채점 구역 표 회귀. 새 YAML 좌표는 명시적 변환 적용 |

G12의 `mission.forbid_own_bases_at=225.0`은 요청된 팀 전략이다. 공식 규칙이 225초부터 출입을 금지한다는 뜻이 아니다. 감지 지도와 분리한 계획 마스크를 builtin/Nav2에 공급하고, 실제 최종 속도 명령의 차체 회전·이동 궤적을 다시 제한한다. 내부에서 시작하면 경기장 쪽으로 빠져나갈 통로를 열고 더 깊은 진입은 막는다. `own_base_guard_horizon_s=0.5`, `own_base_guard_step_m=0.01`은 엔지니어링 설정이며 실물 제동 성능으로 확정하지 않는다.

## 가정·잠정 설정

| 위치 | 교체하거나 검증할 항목 |
| --- | --- |
| `arena_spec.yaml` 제공본 | 벽 두께, 내부 경계, 모서리 박스, 빔 높이·위치, 이름↔줄, 출발 자세, UAV 장착·화각, 로봇 치수·센서 항목의 원문 가정 |
| `layout_default.yaml` | 추정 배치와 임시 면 사진. 공식 경기마다 위치·수량·사진이 달라질 수 있음 |
| `profiles/*.yaml` | 물리 스텝·조명·텍스처·그림자·센서 주기, 검증 카메라. UAV `mount_yaw_offset`과 `optical_hfov`는 광학 장착 가정 |
| `robot_assumptions.yaml` | 단순 차체·바퀴·팬틸트·카메라·레이저·수광부·안테나·LiDAR 높이/토픽·인터페이스·배터리 |
| `simulation_assumptions.yaml` | 심판20Hz·최대 포즈 나이·연속 접촉 허용 시간차·레이저 기하 한계·수치 오차·`base_dwell_s=0` |
| `MissionCfg`, `LlmCfg`, `schema.py` | 자기 거점 전략 시간·속도 예측, M5 체류 슬롯, 임시 M1/M3/M6 JSON, LLM 미설정·시간제한 |

원본 camera hfov 2.15 rad는 보존한다. 현재 standard 프로필의 별도 `optical_hfov=130°`와 광학 yaw +π/2는 높이 0.4 m인 모서리 구조물의 상단까지 포함하고 화면 좌표를 맞추기 위한 가정이다. LiDAR 높이 0.22 m도 단순 짐벌의 자기 가림을 피하기 위한 가정이다. 이 값들을 실측 CAD 또는 실물 보정값으로 표시하지 않는다. 프로필이 바뀌면 실제 영상으로 다시 보정한다. 바닥 평면 보정만으로 높이 0.4 m 큐브 윗면 좌표가 맞는다고 가정하지 않고 높이 시차를 별도로 처리한다.

## 11/18 현장 확인표

- [ ] 주행면의 내부 가로·세로, 벽 안쪽과 바닥 인쇄의 관계, 벽 두께·높이.
- [ ] 모서리 센서 박스의 실제 가로·세로·높이와 중앙 패드 크기.
- [ ] 각 거점 빔의 양 끝 위치·높이, 센서가 가려지는 실제 차체 범위와 첫 점령 판정.
- [ ] 거점 이름↔줄 번호, 실제 여섯 출발 자세·방향.
- [ ] 장애물 0.4 m 규격·배치·쌓기·면 사진·표적 위치와 수광부·시그널 위치.
- [ ] `camera=main` 실제 이미지에서 해상도·네 모서리·독립 검증점·광학 중심·화각·UAV 높이. 기존 경기장 보정파일 재사용 금지.
- [ ] UGV 길이·폭·높이·바퀴 돌출·카메라 위치와 화각, LiDAR 토픽·높이·360° 유효 데이터.
- [ ] ROS `/odom`, TF, 카메라/짐벌 부호·지연과 서보 속도, 레이저·좌우 수광부 실제 위치.
- [ ] 225초 정책의 builtin/Nav2 실제 경로, mask 수신·갱신·해제, 초기 거점 내부 탈출, 차체 모서리·제동 여유.
- [ ] 현장 서버 주소·임무코드 입력 방식·리허설, wait 상한·네트워크 지연·시각 동기화.
- [ ] 주최측 LLM 주소·키·요청 형식, 최종 wire 스키마, M5 체류 시간·M6 동점 규칙.
- [ ] 실제 장비별 60초 RTF·GUI FPS·기동시간 및 한 대/6대 센서 부하 차이.

## 배포 검증 결과와 남은 현장 범위

- [x] 독립 추출 ZIP의 전체 pytest1,352개 통과·실패0·skip0. 기존 실패 assertion 보존.
- [x] Python 코드·시험·새 ROS 스크립트·launch의 pyflakes 경고0.
- [x] 추적 검사97/97개 S-ID·159개 spec 마커 통과.
- [x] 비밀값 노출·배포 차단 회귀,611개 멤버 SHA-256 일치, 시험 중 소스 파일 변경0 확인.
- [x] 실제 A 치수·B 렌더/정합·C 성능·D 제어/영상/빔 결과와 한계 기록.
- [x] 추출한 새 ROS 패키지2개를 공백 없는 Linux 경로에서 새로 빌드:22.7초, 성공.
- [x] 검증 환경·수치·배포 경로 및 별도 영상 제공 범위 기록.

배포 파일은 `maicon2026_ugv_code/maicon2026_ugv_code_20261109.zip`(저장소 루트 기준 `../maicon2026_ugv_code_20261109.zip`)이다. 전체 시험 대상은 ZIP 후보 v2이며, 결과를 적은 문서와 검증자료를 갱신해 최종 재포장한다. **최종 ZIP 바이트 자체를 다시 전체 시험했다는 뜻은 아니다.** [RELEASE_VALIDATION.json](RELEASE_VALIDATION.json)에 후보 검증을 담고, ZIP 옆 `maicon2026_ugv_code_20261109_verification.json`에 최종 해시와 코드·시험·설정·자원의 후보 대비 동일성 대조를 기록한다. 시험한 후보의 SHA-256은 `506f77e5520fafe888223ace6b3dac487754603d85ea48261af5c6de57e6d0ad`다.

남은 범위는 요청 RTX5070급·내장GPU 장비의 실측, 실제 표시 FPS, 실물 치수/센서/제동·현장 서버/LLM 규약, 새 Gazebo의 모든 M3 면 분류·자율 Nav2 전체 경기다. 위 소프트웨어 전체 통과와 짧은 제어 영상을 이 결과로 대신하지 않는다.

API의 상세 차이·원문에서 확정할 수 없는 부분은 [API_SPEC_NOTES](API_SPEC_NOTES.md), 규칙별 연결은 [API_TRACEABILITY](API_TRACEABILITY.md), 전략 경계는 [STRATEGY_DELTAS](STRATEGY_DELTAS.md), 작업 이력은 [API_WORKLOG](API_WORKLOG.md)를 따른다.
