# 현장 실험 체크리스트 (자동 생성 — 직접 고치지 말 것)

생성: `python scripts/field_exp.py checklist --write` · 기준: `maicon26/field/tunables.py` + `config/field_overrides.json`

> **이 표의 '현재값'은 코드 기본값이거나 시뮬에서 정한 값이다. 대회장 실측값이 아니다.** `MEASURED` 가 된 것만 현장에서 갱신된 값이다.

상태 집계: PDF 15 · ASSUMED(가정) 217 · SIM_TUNED(시뮬 튜닝) 56 · MEASURED(실측) 0

## 권장 순서

| 순서 | 실험 | 소요 | 하는 일 | 갱신 대상 수 |
|---|---|---|---|---|
| 1 | **F09** | 15 분 | 줄자·눈으로 잰 값 입력(설치 위치·치수·경기장) — 빈 칸은 건너뜀 | 19 |
| 2 | **F01** | 3 분 | 센서 주기·지연·시계 확인 (LiDAR/카메라/깊이) | 7 |
| 3 | **F02** | 5 분 | 깊이 카메라 설치 roll·pitch·높이 (바닥 평면 맞춤) | 1 |
| 4 | **F04** | 4 분 | 깊이↔LiDAR 정합(앞뒤 설치 오차) — 제자리 회전 | 1 |
| 5 | **F03** | 10 분 | 깊이 거리 정확도·유효율 (평평한 벽, 줄자) | 4 |
| 6 | **F06** | 5 분 | 깊이 3D 층 허상(낮은 물체 없는 환경에서 층이 비어야 한다) | 4 |
| 7 | **F05** | 8 분 | LiDAR 평면 아래 낮은 물체: 깊이 3D 층이 몇 m 에서 잡나 | 6 |
| 8 | **F07** | 8 분 | 접촉(정체) 감지: 자유 주행 vs 보이지 않는 장애물에 막힘 | 5 |
| 9 | **F10** | 10 분 | 레이저↔카메라 축 오프셋(옆·위 평행 오프셋 + 축 각도) — 두 거리에서 점 위치 | 4 |
| 10 | **F08** | 5 분 | 센서 데이터 저장(RGB·깊이·LiDAR·오도메트리) — 오프라인 임계값 튜닝용 | 15 |

실험 코드가 없는 항목(`MANUAL`)과 기존 보정 도구 항목(`calib:*`)은 아래 '실험 코드 없음' 절에 있다.

## 사용 순서(요약)

```bash
python scripts/field_exp.py list                         # 실험 목록
python scripts/field_exp.py run F09 --robot ros          # 실행(대화형). 기록: field/records/<날짜>/F09_*.json
python scripts/field_exp.py apply field/records/<날짜>/F09_*.json     # 제안값을 보고 y/N 로 config/field_overrides.json 에 반영
python scripts/field_exp.py status                       # 무엇이 실측으로 바뀌었나
python scripts/run_real.py ... --overrides config/field_overrides.json   # 반영된 값으로 실행
```

## F09 · 줄자·눈으로 잰 값 입력(설치 위치·치수·경기장) — 빈 칸은 건너뜀

소요 약 15 분 · 필요: 줄자, 경기장·로봇 접근

- 각 항목을 줄자로 재서 입력한다. 모르면 Enter(건너뜀).
- base_link = 차체 중심, 높이는 차축(바퀴 중심) 높이. x 앞 +, y 왼쪽 +, z 위 +.

| ☐ | 우선 | 경로 | 현재값 | 상태 | 지금 값의 근거 | 현장 확인·갱신 | 합격 기준 |
|---|---|---|---|---|---|---|---|
| ☐ | ★★★ | `arena.H` | 5 m | ASSUMED | 5.00(도면 근사) | 경기장 실측 |  |
| ☐ | ★★★ | `arena.W` | 6.12 m | ASSUMED | 6.12(도면 근사) | 경기장 실측 |  |
| ☐ | ★★★ | `arena.base_cy` | (4.16, 2.5, 0.83) m | ASSUMED | (4.16, 2.50, 0.83) 도면 근사 | 거점 1·2·3 중심 y 실측 |  |
| ☐ | ★★★ | `arena.base_depth` | 0.8 m | ASSUMED | 0.80(도면 근사) | 거점 영역 깊이 실측 |  |
| ☐ | ★★★ | `arena.base_height` | 0.9 m | ASSUMED | 0.90(도면 근사; 규격서는 1200x400) | 거점 영역 폭 실측 |  |
| ☐ | ★★★ | `nav.rotate_clear` | 0.219 m | ASSUMED | 0.219 = 외접 0.1698 + 계획 여유 0.049 (파생). 40 cm 통로(반폭 0.20)에서는 제자리 회전 불가 | 통로 안에서 실제로 돌 수 있는 최소 폭 |  |
| ☐ | ★★★ | `rig.depth_xyz` | (0.1, 0, 0.1) m (base_link 기준) | ASSUMED | (0.10, 0, 0.10) 가정: 팬/틸트 '아래' 차체 고정이라는 것만 확인됨(사용자). 위치는 모른다 | 줄자로 base_link(차체 중심·차축 높이)에서 깊이 렌즈까지 x(앞), y(왼쪽), z(위). 그 뒤 F04 로 x 를 교차 확인 | F04 잔차 |중앙값| < 0.02 m |
| ☐ | ★★★ | `rig.oak_rgb_xyz` | (0.1, 0, 0.1) m (base_link 기준) | ASSUMED | rig.depth_xyz 와 같은 값 가정 — URDF 3d_camera_link 값과 다르다. 높이 z 가 OAK 식별 거리를 가장 크게 바꾼다(z 0.06 → 약 1.06 m, 0.10 → 0.94 m, 0.15 → 약 0.80 m) | 줄자로 base_link 에서 OAK 컬러 렌즈까지 x(앞), y(왼쪽), z(위). 바닥에서 렌즈까지 높이는 ±1 cm 로 |  |
| ☐ | ★★★ | `robot.length` | 0.25 m | PDF | 0.25 [PDF p21] 250 mm — 줄자로 실제 확인(안테나·바퀴 돌출 포함 여부) | 차체 최대 길이(돌출부 포함)를 줄자로 | PDF 값과 ±5 mm 이내 |
| ☐ | ★★★ | `robot.width` | 0.23 m | PDF | 0.23 [PDF p21] 230 mm — 바퀴 바깥 폭 포함 여부 확인 | 차체 최대 폭(바퀴 포함)을 줄자로 | PDF 값과 ±5 mm 이내 |
| ☐ | ★★ | `arena.box_size` | 0.4 m | PDF | 0.40 | 박스 실측 |  |
| ☐ | ★★ | `arena.common_half` | 0.615 m | ASSUMED | 0.615(도면 근사) | COMMON 띠 반폭 실측 |  |
| ☐ | ★★ | `arena.wall_h` | 0.4 m | ASSUMED | 0.40 | 외벽 높이 실측 |  |
| ☐ | ★★ | `rig.base_z` | 0.04 m | ASSUMED | 바퀴 반경 0.04 가정 | 바닥~base_link(차축 높이) 줄자 |  |
| ☐ | ★★ | `rig.camera_xyz` | (0.05, 0, 0.03) m (tilt_link 기준) | ASSUMED | (0.05,0,0.03) 가정 | 팬/틸트 축에서 RGB 렌즈까지 줄자 |  |
| ☐ | ★★ | `rig.pan_xyz` | (0, 0, 0.2) m | ASSUMED | (0,0,0.20) 가정 | base_link→팬 축 위치 줄자 |  |
| ☐ | ★★ | `robot.radius` | 0.1698 m | ASSUMED | 0.1698 = 외접 반경 √(0.125²+0.115²) (length/width 에서 파생). 직진 통과 기준이 아니다 | length/width 를 바꾸면 Config.apply_budget() 이 다시 계산 |  |
| ☐ | · | `robot.cam_height` | 0.25 m | ASSUMED | 0.25(시뮬 거리 추정용) | 바닥~RGB 렌즈 줄자(rig 와 일치해야 한다) |  |
| ☐ | · | `robot.wheel_radius` | 0.04 m | ASSUMED | 0.04 | 줄자 |  |

측정 날짜: ________  측정자: ________  기록 파일: ________  판정: ☐ PASS ☐ CHECK ☐ FAIL

## F01 · 센서 주기·지연·시계 확인 (LiDAR/카메라/깊이)

소요 약 3 분 · 필요: robot 정지, 센서 드라이버 실행

- 로봇을 정지시킨 채 센서 드라이버만 켠다(Agent 불필요).
- 측정 시간 동안 움직이지 않는다.

| ☐ | 우선 | 경로 | 현재값 | 상태 | 지금 값의 근거 | 현장 확인·갱신 | 합격 기준 |
|---|---|---|---|---|---|---|---|
| ☐ | ★★★ | `lidar_err.latency_s` | 0.1 s | ASSUMED | 0.10 [가정] | 스캔 stamp 와 수신 시각 차, 회전 중 방위 오차 |  |
| ☐ | ★★★ | `mission.lidar_stale_s` | 0.6 s | ASSUMED | 0.6(10 Hz 기준 6주기 가정) | F01 의 LiDAR 간격 p95 에서 제안 |  |
| ☐ | ★★★ | `mission.vision_stale_s` | 0.8 s | ASSUMED | 0.8 | F01 카메라 간격 p95 + 검출기 지연(YOLO 추론 시간) |  |
| ☐ | · | `camera.min_latency_s` | 0 s | ASSUMED | 0(미측정). 같은 시계면 쓰이지 않는다 | F01 로 시계 모드 확인 |  |
| ☐ | · | `depth.min_latency_s` | 0 s | ASSUMED | 0(미측정). 같은 시계면 쓰이지 않는다(hal/clock.py) | F01 로 시계 모드 확인. 다른 시계일 때만 의미: 로봇을 일정 속도로 돌리며 정지 표적 방위 오차가 0 이 되는 값을 찾는다 |  |
| ☐ | · | `lidar.min_latency_s` | 0 s | ASSUMED | 0(미측정) | F01 |  |
| ☐ | · | `map3d.sync_max_dt` | 0.25 s | ASSUMED | 0.25 | F01 이 카메라/깊이 프레임 간격에서 제안 |  |

측정 날짜: ________  측정자: ________  기록 파일: ________  판정: ☐ PASS ☐ CHECK ☐ FAIL

## F02 · 깊이 카메라 설치 roll·pitch·높이 (바닥 평면 맞춤)

소요 약 5 분 · 필요: 평평한 빈 바닥(앞 1.5 m 안에 박스·벽 없음), 로봇 정지

- 로봇을 평평한 빈 바닥에 둔다. 앞 1.5 m 안에 박스·사람·벽이 없어야 한다(있으면 roll 이 2~4° 틀어진다).
- 16장 이상을 모아 중앙값으로 푼다.

| ☐ | 우선 | 경로 | 현재값 | 상태 | 지금 값의 근거 | 현장 확인·갱신 | 합격 기준 |
|---|---|---|---|---|---|---|---|
| ☐ | ★★★ | `rig.depth_rpy` | (0, 0, 0) rad | ASSUMED | (0,0,0) 가정 | 평평한 빈 바닥에서 F02(바닥 평면)로 roll·pitch. yaw 는 벽 정면에서 영상 좌우 대칭으로 눈 확인 | 바닥 인라이어 ≥ 40 %, RMS < 10 mm |

측정 날짜: ________  측정자: ________  기록 파일: ________  판정: ☐ PASS ☐ CHECK ☐ FAIL

## F04 · 깊이↔LiDAR 정합(앞뒤 설치 오차) — 제자리 회전

소요 약 4 분 · 필요: 벽·박스가 3 m 안에 있는 곳(경기장 시작 거점), 로봇 정지 가능한 자리(제자리 회전)

- 경기장 시작 거점에 로봇을 두고 시작 자세를 입력한다.
- 로봇이 제자리에서 약 0.5 rad/s 로 한 바퀴 돈다.
- 깊이 점의 LiDAR 평면 높이 부분을 같은 방위의 LiDAR 거리와 비교한다. rig.depth_xyz 의 x 를 바꾼 뒤 한 번 더 해서 잔차가 줄었는지 본다.
- **안전:** 제자리 회전: 로봇 둘레 0.5 m 비우기

| ☐ | 우선 | 경로 | 현재값 | 상태 | 지금 값의 근거 | 현장 확인·갱신 | 합격 기준 |
|---|---|---|---|---|---|---|---|
| ☐ | · | `map3d.lidar_check_bias_m` | 0.08 m | ASSUMED | 0.08(중앙값 잔차 경고) | F04 정상 설치 잔차 분포의 3배 정도 |  |

측정 날짜: ________  측정자: ________  기록 파일: ________  판정: ☐ PASS ☐ CHECK ☐ FAIL

## F03 · 깊이 거리 정확도·유효율 (평평한 벽, 줄자)

소요 약 10 분 · 필요: 평평한 벽 하나, 줄자, 로봇 정지

- 평평한 벽 정면에 로봇을 정렬해 세운다(깊이 렌즈 → 벽 수직거리를 줄자로 잰다).
- 안내하는 거리마다 로봇을 옮기고 줄자 값을 입력한다. 거리마다 10프레임.

| ☐ | 우선 | 경로 | 현재값 | 상태 | 지금 값의 근거 | 현장 확인·갱신 | 합격 기준 |
|---|---|---|---|---|---|---|---|
| ☐ | ★★ | `depth.max_range` | 3.5 m | ASSUMED | 3.5 가정(오차 ∝ 거리²) | F03: 편향 < 3 cm, 표준편차 < 2 cm 인 최대 거리 |  |
| ☐ | ★★ | `depth.min_range` | 0.3 m | ASSUMED | 0.30 가정. 확장 시차를 켰을 때만 맞는다(기본 설정의 최소 거리는 약 0.8 m) | F03: 가까운 거리에서 유효율·편향이 괜찮은 최소 거리. RGBD 프로파일(oak.profile=rgbd)에서 줄자로 확인 | 유효율 ≥ 80 %, |편향| < 3 cm |
| ☐ | · | `depth.sim_dropout` | 0.03 비율 | SIM_TUNED | 시뮬 무효 픽셀 3 % 가정 | F03 의 유효율(1−valid) |  |
| ☐ | · | `depth.sim_noise_k` | 0.004 m⁻¹ | SIM_TUNED | 시뮬 깊이 잡음 모델 σ=k·z² 의 k=0.004(스테레오 일반론에서 가정). 시뮬 충실도용 값이라 로봇 동작에는 직접 안 쓰인다 | F03 이 실제 σ/z² 를 제안 |  |

측정 날짜: ________  측정자: ________  기록 파일: ________  판정: ☐ PASS ☐ CHECK ☐ FAIL

## F06 · 깊이 3D 층 허상(낮은 물체 없는 환경에서 층이 비어야 한다)

소요 약 5 분 · 필요: 낮은 물체가 없는 경기장(박스·벽만), 앞 1.5 m 직선

- 경기장에 낮은 물체가 없는지 확인하고(바닥 케이블·테이프 단차 포함) 시작 자세를 입력한다.
- 제자리 한 바퀴 + 1 m 전진 동안 3D 층(LiDAR 가 못 보는 장애물)의 셀 수를 센다. 0 이어야 한다.
- **안전:** 앞이 1.8 m 이상 비어야 전진한다. 아니면 회전만 한다.

| ☐ | 우선 | 경로 | 현재값 | 상태 | 지금 값의 근거 | 현장 확인·갱신 | 합격 기준 |
|---|---|---|---|---|---|---|---|
| ☐ | ★★★ | `map3d.nav_thr` | 1.8 로그오즈 | SIM_TUNED | 1.8(약 3회 관측). 시뮬에서 허상 0 이 되도록 정함(가장자리 번짐 30 % 가정) | F06: 빈 환경에서 3D 층 셀 수. >0 이면 올림, F05 에서 낮은 블록이 안 잡히면 내림 | F06 셀 ≤ 3 & F05 감지 |
| ☐ | ★★ | `depth.sim_flying_pixels` | 0.3 확률 | SIM_TUNED | 시뮬 가장자리 번짐 30 % 가정 — 이 허상 때문에 3D 층 필터(nav_thr 등)를 정했다 | F06 에서 빈 환경의 허상 셀 수로 간접 확인 |  |
| ☐ | ★★ | `map3d.blind_grow_cells` | 3 칸(5 cm) | SIM_TUNED | 3: 박스 둘레 15 cm 번짐을 'LiDAR 가 보는 곳'으로 처리. 시뮬 잡음에 맞춤 | F06 |  |
| ☐ | ★★ | `map3d.nav_min_neighbors` | 3 개 | SIM_TUNED | 3(면/덩어리만 통과, 광선 방향 한 줄 허상 제거). 시뮬에서 정함 | F05/F06 와 함께 조정 |  |

측정 날짜: ________  측정자: ________  기록 파일: ________  판정: ☐ PASS ☐ CHECK ☐ FAIL

## F05 · LiDAR 평면 아래 낮은 물체: 깊이 3D 층이 몇 m 에서 잡나

소요 약 8 분 · 필요: 낮은 블록(높이 5~12 cm, 폭 ≥ 20 cm), 앞 4 m 직선, 줄자

- 로봇 정면 약 3 m 앞 바닥에 낮은 블록을 놓는다(LiDAR 평면 0.24 m 아래).
- 시작 자세와 로봇 중심→블록 앞면 거리를 줄자로 입력한다.
- 로봇이 0.15 m/s 로 접근하다 블록 앞 0.45 m 에서 멈춘다. 감지된 최초 거리·허상 수를 기록한다.
- **안전:** 블록에 부딪히기 전에 멈춘다(앞 0.45 m). LiDAR 앞이 0.35 m 안이면 중단.

| ☐ | 우선 | 경로 | 현재값 | 상태 | 지금 값의 근거 | 현장 확인·갱신 | 합격 기준 |
|---|---|---|---|---|---|---|---|
| ☐ | ★★★ | `map3d.guard_dist` | 0.22 m | ASSUMED | 0.22(LiDAR stop_dist 0.17 보다 약간 큼) | F05: 실제 정지거리(calib brake)보다 커야 한다 | 정지거리 + 여유 |
| ☐ | ★★ | `map3d.blind_floor_layers` | 2 층(5 cm) | SIM_TUNED | 2: 바닥에서 올라온 5~15 cm 물체만. 박스 윗부분 잔재를 거르려고 정함 | F05 에서 실제 낮은 물체 높이를 바꿔 가며 |  |
| ☐ | ★★ | `map3d.guard_slow` | 0.5 m | ASSUMED | 0.50 | F05: 감지 거리보다 작아야 의미가 있다 | 감지 최초 거리 > guard_slow |
| ☐ | ★★ | `map3d.integrate_hz` | 5 Hz | SIM_TUNED | 5(시뮬 깊이 주기와 같게) | Jetson CPU 부하 보고 조정(깊이 프레임 처리 시간 측정) | 처리 < 50 ms |
| ☐ | ★★ | `map3d.nav_z_lo` | 0.06 m | SIM_TUNED | 0.06(바닥 잡음 위) | F05: 바닥이 장애물로 찍히면 올림 |  |
| ☐ | · | `map3d.stride` | 2 px | SIM_TUNED | 2 + 프레임마다 위치 순환: 12 cm 물체가 영상에서 몇 줄밖에 안 돼서 3→2 로 바꿨다(시뮬 해상도 96x72 기준) | 실기 해상도(예 640x400)에서는 같은 물체가 더 많은 줄이므로 키울 수 있다 — F05 로 확인 |  |

측정 날짜: ________  측정자: ________  기록 파일: ________  판정: ☐ PASS ☐ CHECK ☐ FAIL

## F07 · 접촉(정체) 감지: 자유 주행 vs 보이지 않는 장애물에 막힘

소요 약 8 분 · 필요: 앞 2 m 이상 LiDAR 가 보는 벽/물체가 정면에, LiDAR·깊이에 안 보이는 낮은 장애물(≤12 cm), 로봇이 밀어도 되는 상황

- 앞 2 m 이상 열린 곳에서 0.2 m/s 로 2.5 s 전진(자유 주행 기준선).
- 낮은 장애물을 범퍼 바로 앞(5~10 cm)에 놓고 3.5 s 전진 명령(바퀴가 헛돈다).
- SLAM 대신 LiDAR 정면 거리 변화를 '실제 이동'으로 쓴다. 자유/막힘 이동비를 비교해 bump_ratio 를 제안한다.
- **안전:** 막힘 구간은 모터가 구속된다 — 3.5 s 이내, 사람이 정지 버튼 근처에 있을 것. 자유 주행은 앞 0.8 m 에서 자동 중단.

| ☐ | 우선 | 경로 | 현재값 | 상태 | 지금 값의 근거 | 현장 확인·갱신 | 합격 기준 |
|---|---|---|---|---|---|---|---|
| ☐ | ★★★ | `map3d.bump_ratio` | 0.3 비율 | SIM_TUNED | 0.3(실제/기대 이동). 시뮬의 SLAM 자세 오차 ~1 cm 에 맞춤 | F07: 자유 주행 비율 최솟값과 막힘 비율 최댓값 사이 | 분리 마진 ≥ 2배 |
| ☐ | ★★★ | `map3d.bump_window_s` | 1.5 s | SIM_TUNED | 1.5: 시뮬에서 정체를 1.5 s 안에 알아챔 | F07 |  |
| ☐ | ★★ | `map3d.bump_v_min` | 0.08 m/s | SIM_TUNED | 0.08 | F07 + 실제 최소 구동 속도(calib speed_steps) |  |
| ☐ | · | `map3d.bump_ahead_m` | 0.22 m | ASSUMED | 0.22(차체 반경 + 여유) | F07: 접촉 때 범퍼에서 장애물까지 실제 거리 |  |
| ☐ | · | `map3d.bump_lidar_clear_m` | 0.5 m | ASSUMED | 0.5 | F07 |  |

측정 날짜: ________  측정자: ________  기록 파일: ________  판정: ☐ PASS ☐ CHECK ☐ FAIL

## F10 · 레이저↔카메라 축 오프셋(옆·위 평행 오프셋 + 축 각도) — 두 거리에서 점 위치

소요 약 10 분 · 필요: 평평한 표적면 + 십자 표시, 줄자, 레이저 보호 안경

- 팬/틸트 0°, 표적면에 십자 표시. 카메라 영상 중심(십자선)을 표시에 맞춘다.
- 레이저 점이 표시에서 왼쪽/위로 몇 cm 벗어나는지 잰다(왼쪽·위가 +).
- 거리 r1(예 1.0 m)와 r2(예 2.5 m) 두 곳에서 반복. 평행 오프셋은 거리와 무관, 축 각도 오차는 거리에 비례한다.
- **안전:** 레이저는 --allow-laser 를 줬을 때만 자동 발사한다. 아니면 사람이 따로 발사한다. 눈 방향 금지.

| ☐ | 우선 | 경로 | 현재값 | 상태 | 지금 값의 근거 | 현장 확인·갱신 | 합격 기준 |
|---|---|---|---|---|---|---|---|
| ☐ | ★★★ | `aim.laser_lateral_m` | 0 m (왼쪽 +) | ASSUMED | 0(미측정): 레이저 발광부가 카메라보다 옆으로 떨어진 거리 | F10(두 거리에서 점 위치) 또는 줄자 또는 텔레옵 영점 → `field_exp.py zero-import`(4개 한꺼번에) | F10 두 값 일관 |
| ☐ | ★★★ | `aim.laser_offset_pan_deg` | 0 deg | ASSUMED | 0(미측정): 카메라 축 대비 레이저 축 수평 오프셋(Gazebo 모델은 0) | F10 또는 `zero-import` |  |
| ☐ | ★★★ | `aim.laser_offset_tilt_deg` | 0 deg | ASSUMED | 0(미측정)(Gazebo 모델은 0) | F10 또는 `zero-import` |  |
| ☐ | ★★★ | `aim.laser_vertical_m` | 0 m (위 +) | ASSUMED | 0(미측정). Gazebo 모델은 0.0465 m(레이저가 카메라 4.65 cm 위) | F10 또는 `zero-import` |  |

측정 날짜: ________  측정자: ________  기록 파일: ________  판정: ☐ PASS ☐ CHECK ☐ FAIL

## F08 · 센서 데이터 저장(RGB·깊이·LiDAR·오도메트리) — 오프라인 임계값 튜닝용

소요 약 5 분 · 필요: 저장 공간(깊이 프레임당 약 1 MB)

- 찍고 싶은 상황(스모그 가동, 3색 타워, 후면 LED, 낮은 블록, 빈 바닥 등)을 만들고 태그 이름을 정한다.
- 로봇을 사람이 천천히 밀거나 주행시키며 저장한다. 나중에 `field_exp.py replay-depth <폴더>` 로 임계값을 바꿔 가며 다시 돌려 본다.
- LiDAR 확인: 검은 블록·벽 앞 0.3 / 0.6 / 1.0 m 에서 /scan 점 수를 확인한다 [p43: 검정·유리·거울 면은 잘 잡히지 않는다].

| ☐ | 우선 | 경로 | 현재값 | 상태 | 지금 값의 근거 | 현장 확인·갱신 | 합격 기준 |
|---|---|---|---|---|---|---|---|
| ☐ | ★★★ | `lidar.self_mask_sectors` | () (deg, deg) | ASSUMED | ()(기본 끔 — 장착 위치를 모른다). [p6·p43] LiDAR 가 팬틸트 기둥 아래라 기둥·안테나가 가린다. 이제 /scan 을 쓰는 SLAM·Nav2 에도 적용된다(런치 인자 scan_self_mask) | F08 로 LiDAR 를 저장해 제자리에서 0.3 m 안에 늘 같은 방위에 나오는 반사(팬틸트 기둥·안테나)를 찾는다. 로그의 lidar_suspects 도 참고 | 자기 가림 방위가 없어야 한다 |
| ☐ | ★★ | `event.smog_off_ratio` | 0.015 비율 | SIM_TUNED | 0.015 | 〃 |  |
| ☐ | ★★ | `event.smog_on_ratio` | 0.04 비율 | SIM_TUNED | 0.04: 시뮬 스모그(허상 반사 16 %·30 % 가 8~20 cm)에서 정함. 실제 연기의 LiDAR 반사 분포는 모른다 | 스모그 장치가 켜진 상태로 F08(LiDAR 저장) → 고립 반사 비율 분포 확인 |  |
| ☐ | ★★ | `event.tower_finished_color` | red | ASSUMED | red | 〃 |  |
| ☐ | ★★ | `event.tower_ready_color` | yellow | ASSUMED | yellow(규정에 색 없음) | 3색 타워 실물 사진(F08) |  |
| ☐ | ★★ | `event.tower_running_color` | green | ASSUMED | green | 〃 |  |
| ☐ | · | `event.smog_near_m` | 1.2 m | SIM_TUNED | 1.2: 이보다 가까운 고립 반사만 본다 | 스모그 장치 켠 LiDAR(F08) |  |
| ☐ | · | `event.smog_off_s` | 3 s | SIM_TUNED | 3.0 | 〃 |  |
| ☐ | · | `event.smog_on_s` | 0.8 s | SIM_TUNED | 0.8 | 〃 |  |
| ☐ | · | `event.speckle_nbr_m` | 0.25 m | SIM_TUNED | 0.25: 이웃 빔 거리차 기준 | 정상 장면의 고립 반사 수(0~2 가정) 확인 |  |
| ☐ | · | `event.tower_conf_min` | 0.6 | ASSUMED | 0.6 | 타워 LED 검출 신뢰도 |  |
| ☐ | · | `event.ugv_led_range_m` | 1.8 m | ASSUMED | 1.8(후면 LED 판독 거리) | 상대 UGV 뒤 LED 를 거리별로 찍어(F08) 판독 가능 거리 확인 |  |
| ☐ | · | `lidar.min_range` | 0 m | ASSUMED | 0(끔) | 몸체 안쪽 반사가 있으면 그 거리로 |  |
| ☐ | · | `lidar.self_mask_range` | 0.35 m | ASSUMED | 0.35 | 자기 가림 반사 거리 |  |
| ☐ | · | `map3d.smog_max_range` | 1.5 m | ASSUMED | 1.5: 연기 속 스테레오 열화를 가정 | 스모그 장치가 켜졌을 때 F08 로 깊이를 저장해 유효율·잡음 확인 |  |

측정 날짜: ________  측정자: ________  기록 파일: ________  판정: ☐ PASS ☐ CHECK ☐ FAIL

## 실험 코드 없음 (줄자·눈·ros2 명령 / 기존 `calib` 도구)

| ☐ | 우선 | 경로 | 현재값 | 상태 | 실험 | 지금 값의 근거 | 현장 확인·갱신 | 합격 기준 |
|---|---|---|---|---|---|---|---|---|
| ☐ | ★★★ | `avoid.enable` | True | SIM_TUNED | MANUAL | True. 끄면 기존 동작(위협을 모르는 항법) | 현장에서 상대 UGV 가 있을 때 켜고 끈 5분 주행 비교 | 피격↓·M5 시각 +10 s 이내 |
| ☐ | ★★★ | `avoid.exposure_limit_s` | 1.2 s | ASSUMED | MANUAL | 1.2(= 0.6×dwell_s 2 s [PDF]): 적이 이미 조준 중일 수 있다는 마진 | 상대 조준 지연 실측 |  |
| ☐ | ★★★ | `avoid.lidar_blob_max` | 0.36 m | ASSUMED | MANUAL | 0.36 | 〃 (박스 면 0.4 m 와 구별되는 상한) |  |
| ☐ | ★★★ | `avoid.lidar_blob_min` | 0.12 m | ASSUMED | MANUAL | 0.12 | 상대 UGV LiDAR 군집 폭(F12 제안): 거리별 호 길이 |  |
| ☐ | ★★★ | `avoid.lidar_blob_min_speed` | 0.15 m/s | SIM_TUNED | MANUAL | 0.15: 새로 드러난 정적 박스의 자세 오차성 겉보기 이동을 거르려고 정함(시뮬 오탐 120 s 에 0~2회) | 실기 LiDAR·SLAM 지터에서 정적 장면 오탐률 측정 | 정적 장면 오탐 0 |
| ☐ | ★★★ | `avoid.p_cons` | 0.7 | SIM_TUNED | MANUAL | 0.7: 팀 미확인 UGV 의 최소 위험 가중 | 카메라 팀 판정 정확도(안테나 색)를 현장 사진으로 측정한 뒤 조정 |  |
| ☐ | ★★★ | `avoid.r_core` | 2.2 m | ASSUMED | MANUAL | 2.2(= laser.max_range − 0.3): 사정거리 안·시야가 트이면 거리와 무관하게 맞는다는 시뮬 규칙 | 수광센서 유효거리와 거리별 피격 확률 실측 |  |
| ☐ | ★★★ | `depth.calibration_verified` | False | ASSUMED | MANUAL | false: 운영자가 깊이 광학 프레임의 TF·rig.depth_xyz 를 실측한 뒤에만 true. false 면 실기 시작 때 depth.enable 이 꺼지고 거리 측정은 LiDAR·크기 추정으로 돌아간다 | ros2 run tf2_ros tf2_echo base_footprint <깊이 광학 프레임> 값이 rig.depth_xyz/rpy 와 2 cm·2° 이내인지 확인 후 depth.optical_frame 도 실제 이름으로 채운다 | TF 오차 < 2 cm·2° |
| ☐ | ★★★ | `depth.gate_drive` | True | ASSUMED | MANUAL | true(기본·실기 설정 false). 엄격 깊이에서 영상이 끊기면 구동·준비 판정을 막는 기존 규칙. 깊이를 피아식별·거리에만 쓰면 false — 영상이 끊겨도 로봇은 멈추지 않는다(항법 장애물은 2D LiDAR) | OAK 드라이버를 끈 채(또는 USB 를 뽑고) 연습 모드에서 주행해 로봇이 서지 않는지 본다 | 깊이 영상 없이 주행 가능, 로그에 depth 경고만 |
| ☐ | ★★★ | `depth.info_topic` | /oak/stereo/camera_info | ASSUMED | MANUAL | /oak/stereo/camera_info 가정 | ros2 topic list | grep oak |  |
| ☐ | ★★★ | `depth.topic` | /oak/stereo/image_raw | ASSUMED | MANUAL | /oak/stereo/image_raw (depthai-ros 기본 가정) | ros2 topic list | grep oak (실제 깊이 토픽 이름) |  |
| ☐ | ★★★ | `depth.unit_scale` | 0.001 m/단위 | ASSUMED | MANUAL | 0.001(16UC1 = mm) 가정 | ros2 topic echo --once depth topic 의 encoding 확인; F03 에서 줄자와 크기가 맞는지로 교차 확인 | F03 편향 < 3 cm |
| ☐ | ★★★ | `explore.lidar_ugv_exclude_m` | 0.3 m | ASSUMED | MANUAL | 0.30: UGV 위치 주변 제외 반경. 2D LiDAR 는 UGV 궤도(LiDAR 평면 아래)를 못 볼 수 있어 크기로 구분하지 않는다 | UGV 를 큐브 옆에 세우고 LiDAR 로 훑어 UGV 윗부분이 면·장애물로 기록되지 않는지, 궤도가 Nav2 비용지도에 안 잡히는 위험은 별도로 확인 | UGV 가 면 장부에 0건 |
| ☐ | ★★★ | `explore.m1_priority_enemy_first` | False | ASSUMED | MANUAL | false(실기 설정 true): M1 이 지시한 구역 중 적 섹터를 먼저 돌고 CM 구역은 그 다음에 돈다(CM 은 적 섹터가 끝나거나 예산이 지나면 풀린다) | 연습에서 M1 로 적 섹터+CM 을 지정하고 로그 'M1 우선순위' 와 stats['m1_sweep']['tier'] 로 적 섹터 먼저 도는지 확인 | 적 섹터 면을 먼저 소화 |
| ☐ | ★★★ | `explore.m1_sector_sweep` | False | ASSUMED | MANUAL | false(실기 설정 true): M1 이 지정한 구역의 면을 우선 방문해 전부 확인·분류하고 면 식별(FaceReg)로 등록한다. 'M1 지정 구역 = 탐색 허용 구역'이라는 해석이다 — 서버 문서의 M1 정의로 확인 | 연습 모드에서 M1 구역을 지정한 뒤 stats['m1_sweep'](구역별 면 수·완료)와 로그 'M1 섹터 면 확인' 으로 그 구역의 면이 모두 식별로 바뀌는지 본다 | 지정 구역의 면 전부 identified/empty/hidden |
| ☐ | ★★★ | `explore.m3_counting` | ledger | SIM_TUNED | MANUAL | ledger(박스 면 1개=객체 1개). omap 은 위치 병합이라 같은 면을 두 번 세는 오류가 시뮬에서 났다 | 현장 객체 개수와 제출 개수 대조(중복·누락) |  |
| ☐ | ★★★ | `explore.strategy` | online | SIM_TUNED | MANUAL | online(시뮬 6시드에서 이동거리 -7 %·충돌 감소, 재현율은 고정 순회보다 약 5 %p 낮음). lattice 로 되돌리면 기존 5x5 순회 | 현장 5분 주행 2~3회로 lattice 와 online 의 M3 정답 대조(면 단위) | online 재현율 ≥ lattice |
| ☐ | ★★★ | `explore.uav_census` | False | ASSUMED | MANUAL | false(실기 설정 true): UAV 초기 영상에서 뽑힌 면을 '미탐지'로 장부(FaceReg)에 기록해 두고, 로봇이 보고 식별하면 '탐지'로 바꾼다 | 연습에서 로그 'UAV 지도 면 전환: 미탐지 → 탐지' 와 stats['uav_map'] 의 미탐지 수가 줄어드는지 본다 | 탐지된 면은 미탐지 목록에서 사라진다 |
| ☐ | ★★★ | `hal.imu_mount_yaw_deg` | 0 deg | ASSUMED | MANUAL | 0: IMU 축이 차체 축과 같다고 가정(가속도 부호로 충돌 방향을 분류). 다르면 이 각도로 돌려서 본다 | 로봇을 손으로 앞·뒤·왼·오른쪽에서 톡 쳐 보고 로그의 충돌 방향이 맞는지 확인(어긋나면 90·180·-90 으로) | 4방향 분류 정확 |
| ☐ | ★★★ | `hal.pan_sign` | -1 | ASSUMED | MANUAL | −1: 내부 팬은 왼쪽 +, API 는 오른쪽 + [PDF p14·15] | 연습 모드에서 pan=+50 을 보내 실제로 오른쪽으로 도는지, angular.z=+1 이 왼쪽 회전인지 눈으로 확인 | 방향 일치 |
| ☐ | ★★★ | `hal.scan_topic` | /scan | ASSUMED | MANUAL | /scan: LiDAR(D500) 토픽은 API 자료에 없다 | ros2 topic list -t | grep -i scan. 같이: 실기에서 LiDAR 드라이버 기동 시간을 재서 run_real.py --ready-timeout(실행 인자, 기본 60 s)을 정한다 [p53] — 이 안에 LiDAR·Nav2·깊이가 안 뜨면 저하 모드로 /ready 를 보낸다 |  |
| ☐ | ★★★ | `hal.tilt_sign` | 1 | ASSUMED | MANUAL | +1: 위 + | tilt=+50 이 위로 가는지 | 방향 일치 |
| ☐ | ★★★ | `hal.type_override` | {} | ASSUMED | MANUAL | {}: /mil/* 메시지·서비스 타입은 실행 중인 토픽/서비스에서 자동 탐색(패키지 이름을 모른다) | ros2 topic list -t / ros2 service list -t 에서 /mil/state·/mil/match·/mil/oled·/mil/gimbal·/mil/fire 타입 확인. 자동 탐색이 실패하면 'pkg/msg/Name' 으로 직접 적는다 | stats['mil_ready'] 5개 모두 True |
| ☐ | ★★★ | `impact.enable` | False | ASSUMED | MANUAL | false(실기 설정 true). 충돌을 감지하면 정지 → 방향·접촉 대상 분류 → 후진/재계획. 감지 문턱은 가정이다 | 연습 모드에서 벽·큐브·다른 UGV 에 낮은 속도로 부딪히고 stats['impact'] 와 로그의 충돌 분류(방향·대상)를 본다 | 충돌 감지율 ≥ 90 %, 평지 주행 오검출 0 |
| ☐ | ★★★ | `impact.trigger_dg` | 0.45 g | ASSUMED | MANUAL | 0.45: 기준선에서 이만큼 벗어난 가속도를 충돌로 본다(잡음의 5배 문턱과 큰 쪽) | 빈 바닥에서 최고 속도로 1 분 주행해 최대 편차를 재고, 살짝 부딪혔을 때 편차를 잰다. 그 사이로 정한다 | 주행 오검출 0, 살짝 부딪힘 감지 |
| ☐ | ★★★ | `laser.dwell_s` | 2 s | PDF | MANUAL | 2(2초 연속 수광 = 유효) | 표적 LED 가 켜지는 조건 확인 |  |
| ☐ | ★★★ | `laser.max_range` | 2.5 m | ASSUMED | MANUAL | 2.5(수광 유효거리 가정) | 표적에 수광되는 최대 거리를 거리별로 쏴 보며 |  |
| ☐ | ★★★ | `lidar_err.pose_sigma_m` | 0.01 m | ASSUMED | MANUAL | 0.010 [가정; 시뮬 단독 주행 중앙값 약 1.6 cm] | SLAM 위치와 줄자 위치 비교(정지 표적 N회) |  |
| ☐ | ★★★ | `lidar_err.range_bias_m` | 0.005 m | ASSUMED | MANUAL | 0.005 [가정 — PDF 에 LiDAR 정확도 없음] | 정지 평면 벽 앞 0.3/1/2 m 에서 줄자 거리와 LiDAR 평균 차이 |  |
| ☐ | ★★★ | `lidar_err.range_sigma_m` | 0.008 m | ASSUMED | MANUAL | 0.008 [가정] | 같은 측정의 표준편차 |  |
| ☐ | ★★★ | `mission.strike_max_incidence_deg` | 40 deg | ASSUMED | MANUAL | 40: 지정표적 사격 입사각 한계(창 ±3.5 cm 라 비스듬하면 빗나간다) | 실기 표적 창 크기·수광 각도 |  |
| ☐ | ★★★ | `nav2.cmd_topic` | /cmd_vel_nav2 | ASSUMED | MANUAL | /cmd_vel_nav2(Nav2 출력; 드라이버가 구독하는 /cmd_vel 과 분리) | ros2 topic list, 드라이버 구독 확인 |  |
| ☐ | ★★★ | `nav2.map_topic` | /map | ASSUMED | MANUAL | /map | ros2 topic list |  |
| ☐ | ★★★ | `oak.power_gating` | False | ASSUMED | MANUAL | false(실기 설정 true): 평소 OAK 를 꺼 두고 식별 지점 근처에서만 켠다(start/stop Trigger 서비스) | ros2 service list | grep oak 로 서비스 이름을 확인하고 oak.start_srv/stop_srv 에 넣는다. 켠 뒤 첫 프레임까지 걸리는 시간을 재서 oak.wake_dist_m·start_timeout_s 를 정한다 | 켜기 명령 → 첫 프레임 ≤ start_timeout_s |
| ☐ | ★★★ | `oak.rgb_info_topic` | /oak/rgb/camera_info | ASSUMED | MANUAL | /oak/rgb/camera_info 가정 | ros2 topic list | grep oak |  |
| ☐ | ★★★ | `oak.rgb_topic` | /oak/rgb/image_raw | ASSUMED | MANUAL | /oak/rgb/image_raw 가정(depthai-ros 기본 이름) | ros2 topic list | grep oak |  |
| ☐ | ★★★ | `oak_det.depth_aligned_to_rgb` | True | ASSUMED | MANUAL | true: 깊이가 컬러에 정렬돼 있다(config/oak/rgbd.yaml 의 i_align_depth). 정렬이 아니면 깊이 거리를 쓰지 않는다 | OAK 드라이버 파라미터와 깊이 영상 해상도(컬러와 같은지)를 확인 | 깊이 영상 크기 = 컬러 영상 크기 |
| ☐ | ★★★ | `oak_det.depth_range` | False | ASSUMED | MANUAL | false(실기 설정 true): UGV 상자 가운데의 깊이로 거리를 잰다(LiDAR·크기 추정보다 먼저). 깊이 프레임이 없으면 기존 순서 그대로 | UGV 를 0.8·1.2·1.6·2.0 m 에 놓고 stats['oak_depth'] 의 사용 횟수와 트랙 거리를 줄자와 비교 | |오차| < 0.10 m (≤ 2 m) |
| ☐ | ★★★ | `oak_det.fit_vfov_deg` | 42 deg | ASSUMED | MANUAL | 42: 1280×720(16:9) 스트림의 세로 화각(센서 4:3 의 세로 54° 를 잘라 쓴다). CameraInfo 의 K 가 오면 그 값을 쓴다 | ros2 topic echo --once /oak/rgb/camera_info 의 K·크기로 화각을 계산. 스트림을 4:3 으로 바꾸면 54° 이고 식별 거리가 약 0.95 → 0.75 m 로 짧아진다 | 계산 화각 = 실제 영상에서 재 본 화각 ±2° |
| ☐ | ★★★ | `oak_det.identify_dist_m` | 0.8 m | ASSUMED | MANUAL | 0.8: OAK RGB 로 면 하나를 식별할 거리 | OAK 영상에서 면 그림이 YOLO 가 잡을 만큼 크게(≥ min_box_px) 보이는 거리를 현장에서 잰다 | 식별 대상 면에서 YOLO 검출 성공 |
| ☐ | ★★★ | `oak_det.identify_primary` | False | ASSUMED | MANUAL | false(실기 설정 true): 면 식별을 처음부터 OAK(Depth 카메라 RGB)로 한다. 못 보는 면(잘림·프레임 없음·보일 자리 없음·검출 없음)만 팬틸트로 넘긴다 | 연습에서 LiDAR 로 찾은 큐브 4면을 OAK 로 식별해 보고, 폴백으로 넘어간 면과 이유(로그 oak_identify / fallback_stats)를 센다 | OAK 식별 성공 면 ≥ 70 %, 폴백 면은 팬틸트로 식별 |
| ☐ | ★★★ | `oak_det.iff_max_range_m` | 2.2 m | ASSUMED | MANUAL | 2.2: 이보다 먼 UGV 는 안테나 색(피아)을 믿지 않는다. 깊이로 거리를 잴 수 있을 때만 적용 | 거리별(1.0~3.0 m)로 아군·적 안테나 색 분류가 맞는 최대 거리를 센다 | 오분류 < 5 % 인 최대 거리 |
| ☐ | ★★★ | `oak_det.ugv_only` | False | ASSUMED | MANUAL | false(실기 설정 true): OAK 검출 중 UGV 만 쓴다(트랙·피아식별). 면 득표에는 넣지 않는다 — 면 식별은 팬틸트가 한다 | 연습에서 OAK 앞에 아군·적 UGV 를 세워 stats['oak_ugv_dets'] 가 늘고 면 장부(m3_faces)는 OAK 로 바뀌지 않는지 확인 | UGV 검출 성공, 면 득표 변화 0 |
| ☐ | ★★★ | `pose_guard.enable` | False | ASSUMED | MANUAL | false(실기 설정 true). 충격 뒤 LiDAR 방위가 틀어졌는지 추정해 보정(스캔 회전·자세 보정)하고, 크게 틀어졌으면 내장 A* 로 넘어간다. TF 가 끊기면 짧게 추측항법 | 충돌 후 stats['pose_guard'] 의 yaw_shift_deg 가 실제 틀어짐(각도기)과 맞는지 확인 | 오차 < 3° |
| ☐ | ★★★ | `pose_guard.link_check` | True | ASSUMED | MANUAL | true: 고정 구조물(벽·13개 구조물)에 라이다 스캔이 얼마나 맞는지로 UAV 지도 연결을 점검한다(UAV 상자 위치와 무관한 기준) | 경기장에서 정지·주행 중 stats['map_link'] 의 맞음 비율이 안정적인지 본다 | 정상 시 맞음 비율 변동 < 0.1 |
| ☐ | ★★★ | `pose_guard.link_trim` | False | ASSUMED | MANUAL | false(기본): 연결 점검 결과로 지도 정렬을 조금씩 보정하는 기능. 경기장 크기·구조물 좌표가 틀리면 오히려 정렬을 망치므로, 먼저 link_check 값만 보고 결정한다 | 연습에서 link_trim=false 로 며칠 돌려 맞음 비율이 낮아지는 위치를 보고, 구조물 좌표가 맞다고 확인된 뒤에만 켠다 |  |
| ☐ | ★★★ | `pose_guard.reanchor` | False | ASSUMED | MANUAL | false(기본): 충격·SLAM 도약·연결 불량 뒤 정지 상태에서 벽·고정 구조물에 LiDAR 를 다시 맞춰 SLAM 지도→경기장 앵커를 바꾼다(지도는 그대로). 경기장 도면(외벽·구조물 좌표)이 틀리면 오히려 앵커를 망친다 | 연습에서 먼저 false 로 stats['map_link'] 의 맞음 비율과 로그 '경기장 연결 기준선' 잔차를 본다. 그 뒤 true 로 놓고 일부러 로봇을 벽에 부딪혀 밀어(또는 SLAM 을 재시작해) 앵커가 어긋난 상태를 만들고, 로그 '앵커 재설정 결정/적용/확인' 과 이동량이 실측(줄자)과 맞는지 본다 | 어긋남 10 cm·5° 를 3 cm·1.5° 안으로 복구, 어긋나지 않은 충돌 20 회에서 오적용 0 |
| ☐ | ★★★ | `pose_guard.reanchor_plan_spread_m` | 0.06 m | ASSUMED | MANUAL | 0.06: 연결이 좋은 동안 자리마다 잰 도면 잔차의 퍼짐이 이보다 크면 경기장 도면(구조물 좌표)을 믿을 수 없다고 보고 앵커 재설정을 막는다(0 = 검사 끔). 구조물 좌표가 5/10/15 cm 틀리면 퍼짐은 약 4/9/14 cm | 경기장 여러 곳(구석·중앙·구조물 옆)에서 정지해 stats['pose_guard']['reanchor']['plan_spread'] 의 xy 를 본다. 3 cm 이상이면 줄자로 구조물 좌표부터 재확인 | 퍼짐 < 3 cm |
| ☐ | ★★★ | `pose_guard.shift_detect_deg` | 2.5 deg | ASSUMED | MANUAL | 2.5: 충격 전후 (SLAM yaw − 몸체 yaw) 의 계단이 이보다 크면 LiDAR 가 틀어진 것 | LiDAR 마운트를 일부러 5°·10° 돌려 놓고 감지·추정값을 본다 | 5° 틀어짐 감지 |
| ☐ | ★★★ | `uav_boxes.enable` | False | ASSUMED | MANUAL | false(실기 설정 true). 켜져도 uav_boxes.calibration 파일이 없으면 검출기는 꺼진 채 시작한다(경고만) | scripts/uav_boxes_tool.py fetch → detect 로 상자 20개 전부 잡히는지 눈으로 확인한 뒤 켠다 | 상자 개수·위치가 실물과 일치(오차 < 0.1 m), 오검출 0 |
| ☐ | ★★★ | `uav_boxes.ignore_rects` | () m | ASSUMED | MANUAL | (): 검출에서 제외할 고정 구조물 사각형 (x0,y0,x1,y1)… — 코드에 하드코딩하지 않았다 | 연습장 빈 경기장에서 detect 를 돌려 나오는 덩어리 위치를 적는다 | 빈 경기장에서 검출 0 |
| ☐ | ★★★ | `uav_boxes.top_color` | black | ASSUMED | MANUAL | black: 상자 윗면이 검정이라는 가정(실기 상자 색 미확인). grey/both 로 바꿀 수 있다 | 상자 윗면·바닥 몇 점을 probe 로 재어 제안값 확인 | 상자 윗면 V 가 바닥 V 보다 뚜렷이 어둡다 |
| ☐ | ★★★ | `uav_boxes.v_black_max` | 45 0..255 | ASSUMED | MANUAL | 45: 윗면 밝기(V) 상한. 시뮬(바닥 V≈47) 기준이라 실제 조명에서는 달라진다 | scripts/uav_boxes_tool.py probe --box X Y … --floor X Y … 의 제안값을 config 에 옮긴다 | 연습장 상자 전부 검출, 그림자·UGV 오검출 0 |
| ☐ | ★★ | `aim.aim_point_target` | (0.5, 0.16) bbox 비율 | ASSUMED | MANUAL | (0.5, 0.16): 센서를 못 찾을 때의 대체 추측 | 실제 YOLO 라벨링 방식(면 전체 vs 물체)에 따라 |  |
| ☐ | ★★ | `aim.gimbal_settle_s` | 0 s | PDF | MANUAL | 0(끔). [p72] /mil/gimbal 의 ok 는 접수 — 사격 전 0.3–0.5 s 대기를 권장 | 현장에서 0 / 0.3 / 0.5 를 비교한다(큰 짐벌 명령 직후 첫 발의 빗나감) | 빗나감이 줄어드는 가장 짧은 값 |
| ☐ | ★★ | `aim.hold_before_fire` | 0.2 s | SIM_TUNED | MANUAL | 0.20: 정렬이 이만큼 안정되면 발사. [p72] 짐벌 ok 는 접수일 뿐이라 aim.gimbal_settle_s 와 함께 본다 | 실기 서보 정착 시간에 맞춰 |  |
| ☐ | ★★ | `aim.kp_pan` | 0.9 | SIM_TUNED | MANUAL | 0.9: 시뮬 키네매틱 서보 기준 | 실기 서보 응답(과도/진동)을 보며 | 진동 없이 3 틱 안 정착 |
| ☐ | ★★ | `aim.kp_tilt` | 0.9 | SIM_TUNED | MANUAL | 0.9 | 〃 |  |
| ☐ | ★★ | `aim.sensor_radius_frac` | 0.6 비율 | ASSUMED | MANUAL | 0.6(센서 반경의 60 %) | 수광센서 크기·유효 입사각 확인 후 |  |
| ☐ | ★★ | `aim.tol_deg` | 1.2 deg | SIM_TUNED | MANUAL | 1.2: 시뮬 서보·검출 지터 기준 | 실기 서보 정착 오차·검출 지터로 재조정(서브게임 반복) |  |
| ☐ | ★★ | `aim.tol_deg_far` | 0.7 deg | SIM_TUNED | MANUAL | 0.7 | 〃 |  |
| ☐ | ★★ | `avoid.cap_s` | 25 s | SIM_TUNED | MANUAL | 25(60 s 창): 회피가 구동을 쥐는 시간 상한 — 임무 지연 방지 | 현장 비교 |  |
| ☐ | ★★ | `avoid.fresh_s` | 1.6 s | SIM_TUNED | MANUAL | 1.6: 소유권(EVADE/CLOSE)을 거는 관측의 최대 나이. UAV 지연 2 s 에서 오래된 위치로 움직이면 기준선보다 맞았다(시뮬) | 실기 UAV 영상 지연·주기 확인 후 |  |
| ☐ | ★★ | `avoid.friend_gate_m` | 0.5 m | ASSUMED | MANUAL | 0.5(+ friend_max_speed × 경과 시간): 같은 아군으로 이을 허용 거리 | UAV 위치 오차(경기장 좌표 변환 오차 포함)와 UGV 속도로 확인 — 정지한 아군의 UAV 위치가 프레임마다 흔들리는 폭보다 커야 한다 |  |
| ☐ | ★★ | `avoid.friend_lost_s` | 4 s | ASSUMED | MANUAL | 4.0: 아군이 이만큼 확인되지 않으면 놓친 것(UAV 주기의 2.5 배보다 길게). 놓치면 '식별 안 됨'으로 남기고 다시 잇지 않는다 | 실기 UAV 영상 주기(1·2·5 s)와 아군이 가려지는 시간을 보고 정한다. 너무 짧으면 멀쩡한 아군을 자주 놓치고, 너무 길면 자리 바꾼 적이 신원을 가져갈 여지가 커진다 | 로그 '아군 … 식별 안 됨' 사유가 대부분 정말 놓친 경우 |
| ☐ | ★★ | `avoid.r_aware` | 3 m | SIM_TUNED | MANUAL | 3.0 | 시뮬 스윕. 실기 LiDAR 인지거리·정지거리로 |  |
| ☐ | ★★ | `avoid.r_body` | 0.13 m | ASSUMED | MANUAL | 0.13(시뮬 UGV 반경) | 상대 UGV 실물 치수 |  |
| ☐ | ★★ | `avoid.r_close` | 0.8 m | SIM_TUNED | MANUAL | 0.8 | 〃 |  |
| ☐ | ★★ | `avoid.r_evade` | 1.6 m | SIM_TUNED | MANUAL | 1.6 | 〃 |  |
| ☐ | ★★ | `avoid.r_laser_extra` | 0.3 m | ASSUMED | MANUAL | 0.3 | 수광센서 유효거리 실측 |  |
| ☐ | ★★ | `avoid.risk_extra_inflate_m` | 0.07 m | SIM_TUNED | MANUAL | 0.07: 위험 인지 경로가 벽을 타지 않게. 시뮬에서 벽 따라가다 LiDAR 제한에 걸려 M5 를 놓친 뒤 정함 | 실기 LiDAR 제한 거리 |  |
| ☐ | ★★ | `avoid.sigma_cam` | 0.08 m | ASSUMED | MANUAL | 0.08 | 정지 상대 UGV 반복 측정 |  |
| ☐ | ★★ | `avoid.sigma_lidar` | 0.06 m | ASSUMED | MANUAL | 0.06 | 정지 상대 UGV 를 LiDAR 로 반복 측정 |  |
| ☐ | ★★ | `avoid.sigma_uav` | 0.15 m | ASSUMED | MANUAL | 0.15(시뮬은 0.04). 실기 UAV 상자 검출기(uav_boxes)는 있으나 실제 영상으로 검증한 적 없다 | uav_boxes 검출 결과의 상자 위치를 줄자와 대조해 오차를 측정(UGV 위치 σ 와는 별개) |  |
| ☐ | ★★ | `avoid.t_margin_s` | 8 s | SIM_TUNED | MANUAL | 8: M5 시간 여유 계산 상수 | 실기 이동 시간 분산 |  |
| ☐ | ★★ | `avoid.v_enemy_max` | 0.45 m/s | ASSUMED | MANUAL | 0.45(= robot.v_max 가정) | 상대 속도 관찰 |  |
| ☐ | ★★ | `avoid.w_risk` | 3 | SIM_TUNED | MANUAL | 3.0: 위험 비용 가중. 시뮬에서 피격↔시간 균형 | 현장 비교로 조정 |  |
| ☐ | ★★ | `camera.usable_hfov_deg` | None deg | ASSUMED | MANUAL | None(= 모델 전체 화각). 어안 가장자리는 검출이 약할 수 있다 | 가장자리에서의 검출률을 보고 커버리지(본 것으로 치는 범위)에 쓸 수평 화각을 정한다 |  |
| ☐ | ★★ | `depth.K` | None px | ASSUMED | MANUAL | None(= hfov 로 근사) | ros2 topic echo /oak/stereo/camera_info → k 9개와 해상도(K_width/K_height) |  |
| ☐ | ★★ | `depth.hfov_deg` | 72 deg | ASSUMED | MANUAL | OAK-D 계열 72° 가정(실제 모델 미확인) | depthai 가 내는 camera_info 의 K 로 대체(K 가 있으면 hfov 는 안 쓴다) |  |
| ☐ | ★★ | `explore.deadline_margin_s` | 3 s | SIM_TUNED | MANUAL | 3: prestage_at 앞 여유 | 실기 이동 시간 분산을 보고 |  |
| ☐ | ★★ | `explore.lidar_closure_min_cells` | 4 칸(0.05 m) | ASSUMED | MANUAL | 4: 닫힌 도형 안쪽 칸 수 하한 | 큐브 한 개를 사방에서 훑은 뒤 '닫힘'이 되는지 확인(안 되면 줄이고, 열린 도형이 닫힘으로 나오면 늘린다) | 4면 큐브 닫힘, 한 면만 본 큐브 열림 |
| ☐ | ★★ | `explore.lidar_face_min_L` | 1.5 log-odds | ASSUMED | MANUAL | 1.5: 두 번 이상 맞은 셀만 점유로 센다 | 실기 LiDAR 잡음에서 빈 공간에 유령 면이 생기면 올리고, 큐브 면이 안 잡히면 내린다 | 유령 면 0, 큐브 면 누락 0 |
| ☐ | ★★ | `explore.lidar_face_min_len_m` | 0.4 m | ASSUMED | MANUAL | 0.40: 규정 큐브 한 변. 0.9 배(0.36 m)까지 허용 — 0.05 m 격자 양자화 | 실기 LiDAR 로 큐브 한 개 앞을 비추어 면 길이가 0.36~0.45 m 로 잡히는지(격자 지도 화면) 확인 | 큐브 한 면이 면으로 기록된다 |
| ☐ | ★★ | `explore.m1_enemy_budget_s` | 150 s | ASSUMED | MANUAL | 150: 적 섹터 탐색에 쓸 최대 시간(0 = 무제한). 넘으면 CM 을 풀어 준다(적 섹터에 갇혀 CM 을 못 도는 것을 막는다) | 적 섹터 한 개를 도는 실제 시간을 재어 정한다 |  |
| ☐ | ★★ | `explore.nbv_dists` | (0.3, 0.35, 0.45, 0.6, 0.8, 1, 1.4) m | ASSUMED | MANUAL | (0.45,0.6,0.8,1.0,1.4): 면 앞 시점 거리 후보. perception.max_obj_range(2.2) 안 | 실기 YOLO 가 객체를 안정적으로 분류하는 최소·최대 거리 |  |
| ☐ | ★★ | `explore.spin_clear_m` | 0.32 m | ASSUMED | MANUAL | 0.32: 제자리 360° 회전에 필요한 몸 둘레 여유. 스키드 조향 밀림은 시뮬 값 | 실기 제자리 회전 시 쓸리는 반경 |  |
| ☐ | ★★ | `explore.stage_margin_s` | 4 s | SIM_TUNED | MANUAL | 4: M5 명령(t_base_cmd) 앞 여유 | 〃 |  |
| ☐ | ★★ | `explore.view_max_incidence_deg` | 70 deg | ASSUMED | MANUAL | 70: 면을 이 입사각보다 비스듬히 보면 '본 것'으로 안 친다 | 실제 카메라로 면 사진: 몇 도까지 객체 종류를 알아보는가 | 분류 정확도 ≥ 90 % |
| ☐ | ★★ | `hal.cmd_stale_s` | 0.3 s | ASSUMED | MANUAL | 0.3: Agent 가 이 시간 새 속도를 안 주면 HAL 이 0 을 낸다(PDF: 0.5 s 끊기면 로봇이 자동 정지) | 제어 루프 최대 지연 로그(tick_ms_max) |  |
| ☐ | ★★ | `hal.drive_pause_s` | 10 s | ASSUMED | MANUAL | 10: 누적 피격이 hp_max 에 이르면 주행 정지 시간(예전 코드 규칙을 옮김; 2차 자료엔 없다) | 룰북/실기 확인: HP 0 에서 주행이 정지하는가 |  |
| ☐ | ★★ | `hal.freeze_clock_on_pause` | True | ASSUMED | MANUAL | True: cmd=pause 동안 경기 시계를 멈춘다. 서버가 일시정지 시간을 빼 주는지 문서에 없다 | 주최 측에 확인 |  |
| ☐ | ★★ | `hal.frozen_frames` | 20 장 | ASSUMED | MANUAL | 20: 같은 영상이 이만큼 연속이면 카메라 멈춤으로 본다. 정지 장면을 멈춤으로 잘못 보면 안 된다 | 로봇을 멈추고 정지 장면을 1분 찍어 오탐(멈춤으로 잘못 판단)이 0 인지 확인 | 오탐 0 |
| ☐ | ★★ | `hal.gimbal_latency_s` | 0.05 s | ASSUMED | MANUAL | 0.05: 팬틸트 명령 접수→움직임 시작(위치 추정용). 위치 피드백이 없어 모델로 추정한다. [p72] ok 는 접수 — aim.gimbal_settle_s 와 함께 본다 | 고속 카메라/영상으로 명령 후 화면이 움직이기 시작하는 시간 | 촬영 시각 팬각 오차 < 0.5° |
| ☐ | ★★ | `hal.gimbal_speed` | 150 deg/s | ASSUMED | MANUAL | 150: 서비스 speed(1~270). 추정 위치 모델도 같은 값으로 움직인다고 본다 | pan 0→+50 이동 시간을 재서 실제 속도 확인 | 실측/명령 속도 ±15 % |
| ☐ | ★★ | `hal.tilt_stop_deg` | 25 deg | ASSUMED | MANUAL | 25: |roll|·|pitch| 가 넘으면 정지 [PDF p17: 기울기·충격 감지 시 정지] | 경기장 경사·박스 넘기 때 IMU 값 기록 | 정상 주행에서 오동작 0 |
| ☐ | ★★ | `impact.contact_lidar_m` | 0.35 m | ASSUMED | MANUAL | 0.35: 접촉 방향 LiDAR 가 이보다 가까운 물체를 보면 '보이는 장애물과 접촉'. 안 보이면 낮은 물체로 보고 그 자리를 장애물로 기록 | 낮은 물체(바닥 가까운 큐브 가장자리)에 부딪혀 기록이 남는지 본다 |  |
| ☐ | ★★ | `impact.gyro_dps` | 180 deg/s | ASSUMED | MANUAL | 180: yaw 각속도가 이 이상이면 가속도가 약해도 충격(자이로를 줄 때만) | 제자리 회전 최대 각속도(약 120~150)보다 크게, 옆구리 충돌의 튕김보다 작게 | 회전 오검출 0 |
| ☐ | ★★ | `impact.major_dg` | 1.2 g | ASSUMED | MANUAL | 1.2: 이 이상이면 큰 충격(정지 길게) | 벽에 빠르게 부딪힌 편차로 정한다 |  |
| ☐ | ★★ | `impact.scan_skip_s` | 0.35 s | ASSUMED | MANUAL | 0.35: 충돌 직후 이 시간의 스캔은 Agent 격자에 쌓지 않는다(차체 흔들림) | 충돌 직후 격자에 유령 장애물이 생기는지 본다 | 유령 장애물 0 |
| ☐ | ★★ | `impact.ugv_contact_m` | 0.9 m | ASSUMED | MANUAL | 0.90: 접촉 방향 이 거리 안에 UGV 트랙이 있으면 UGV 와의 접촉으로 본다 | 아군·적 UGV 와 접촉 시 분류 결과를 로그로 본다 |  |
| ☐ | ★★ | `laser.ammo` | 8 발 | PDF | MANUAL | 8(PDF 탄창) | 로봇 펌웨어 탄창 수와 일치하는지 |  |
| ☐ | ★★ | `laser.cooldown_s` | 5 s | PDF | MANUAL | 5(재발사 대기) | 〃 |  |
| ☐ | ★★ | `laser.min_range` | 0.35 m | ASSUMED | MANUAL | 0.35 | 〃 |  |
| ☐ | ★★ | `laser.shot_s` | 5 s | PDF | MANUAL | 5(1발 5초 발광) | 펌웨어가 5 s 를 직접 하는지(mentoring 질문 6) |  |
| ☐ | ★★ | `lidar_err.angle_sigma_deg` | 0.3 deg | ASSUMED | MANUAL | 0.3 [가정] | 직각 모서리/기둥의 방위 반복 측정 |  |
| ☐ | ★★ | `lidar_err.dropout_p` | 0.02 비율 | ASSUMED | MANUAL | 0.02 [가정] — 거리 여유로 메우지 않고 지속성 필터로 다룬다 | 정지 장면에서 무효 빔 비율 |  |
| ☐ | ★★ | `lidar_err.outlier_p` | 0.005 비율 | ASSUMED | MANUAL | 0.005 [가정] | 정지 장면에서 이웃과 크게 어긋난 빔 비율 |  |
| ☐ | ★★ | `map3d.target_window_z` | (0.22, 0.46) m | ASSUMED | MANUAL | (0.22, 0.46): PDF p12 사진에서 짐작한 흰 원판 높이 0.30~0.37 둘레 | 표적 면 흰 원판 중심 높이를 줄자로 재서 ±0.08 로 |  |
| ☐ | ★★ | `map3d.ugv_receiver_z` | (0.04, 0.32) m | ASSUMED | MANUAL | (0.04, 0.32): PDF p21 사진에서 짐작 | 상대 UGV 수광 돔 높이 줄자 |  |
| ☐ | ★★ | `mission.fire_range_slack` | 0.15 m | ASSUMED | MANUAL | 0.15: strike_range+이 값 이하에서만 발사 | 실기 서보 지터에서 맞는 최대 거리 |  |
| ☐ | ★★ | `mission.m3_first_s` | 10 s | ASSUMED | MANUAL | 60: M3 첫 제출. 서버 문서 7단계: 마지막 제출을 채점하고 5:00 이후는 409 → 미리·자주 보내라 |  |  |
| ☐ | ★★ | `mission.prestage_at` | 222 s | ASSUMED | MANUAL | 222(M5 사전 이동 시작) | 주행 시간 실측에 맞춰 |  |
| ☐ | ★★ | `mission.stage_standoff_m` | 0.3 m | ASSUMED | MANUAL | 0.30: 적 가운데 거점 경계에서 경기장 쪽으로 떨어진 대기 거리(stage_policy=enemy_middle) | 현장 배치에서 앞 장애물·거점 경계와 0.22 m 이상 여유가 있는지 눈으로 확인 |  |
| ☐ | ★★ | `mission.strike_range` | 1.1 m | SIM_TUNED | MANUAL | 1.10(시뮬 서브게임에서) | laser.max_range 와 서보 안정성에 맞춰 |  |
| ☐ | ★★ | `nav2.action_name` | navigate_to_pose | ASSUMED | MANUAL | navigate_to_pose | ros2 action list |  |
| ☐ | ★★ | `nav2.base_frame` | base_link | ASSUMED | MANUAL | base_link | 〃 |  |
| ☐ | ★★ | `nav2.map_frame` | map | ASSUMED | MANUAL | map | ros2 run tf2_tools view_frames |  |
| ☐ | ★★ | `oak.launch_cmd` | (ros2, launch, depthai_ros_driver, camera.launch.py) | ASSUMED | MANUAL | ros2 launch depthai_ros_driver camera.launch.py 가정(패키지·launch 파일 이름 미확인) | ros2 pkg list | grep depthai 와 실제 launch 파일 이름. MAICON_OAK=1 로 띄워 토픽이 뜨는지 |  |
| ☐ | ★★ | `oak.profile` | rgb_lowpower | ASSUMED | MANUAL | rgb_lowpower(config/oak/<profile>.yaml). 드라이버 판(구 문서 / v3)에 따라 파라미터 이름이 다르다 | ros2 param dump 로 파일의 파라미터 이름이 실제와 맞는지 확인. 전력은 [p44]와 Luxonis 사양(기본 2.5–3 W + 기능별 추가)으로 본다 |  |
| ☐ | ★★ | `oak_det.conf` | 0.35 | ASSUMED | MANUAL | 0.35: 미측정 | 같은 장면에서 팬틸트와 OAK 의 정밀도·재현율을 비교 |  |
| ☐ | ★★ | `oak_det.identify_dwell_s` | 2.5 s | ASSUMED | MANUAL | 2.5: 면을 향해 정렬한 채 머무는 시간 | OAK 검출 주기(max_hz 3)로 identify_min_frames 프레임이 모이는지 확인 |  |
| ☐ | ★★ | `oak_det.max_hz` | 3 Hz | ASSUMED | MANUAL | 3.0: 미측정. 한 모델을 나눠 쓰므로 팬틸트 검출이 먼저다 | Jetson GPU 여유를 보며 정한다(OAK 를 켜도 팬틸트 검출 주기가 줄지 않는 최대) | 팬틸트 검출 Hz 가 줄지 않는다 |
| ☐ | ★★ | `oak_det.ugv_width_m` | (0.08, 0.7) m (최소, 최대) | ASSUMED | MANUAL | (0.08, 0.70): 깊이로 잰 상자 폭의 허용 범위. 영상 가장자리에 걸치지 않았는데 벗어나면 UGV 가 아니다 | 실제 UGV 를 정면·옆에서 보며 stats 의 폭 범위를 본다(정면 약 0.2~0.4 m) | UGV 는 통과, 사람 다리·벽은 거부 |
| ☐ | ★★ | `perception.conf_min` | 0.4 | ASSUMED | MANUAL | 0.40 | 학습한 YOLO 의 PR 곡선 |  |
| ☐ | ★★ | `perception.max_obj_range` | 2.2 m | ASSUMED | MANUAL | 2.2(먼 객체 오분류 방지) | 거리별 인식 정확도 |  |
| ☐ | ★★ | `perception.ugv_conf_min` | 0.5 | ASSUMED | MANUAL | 0.50 | 〃 |  |
| ☐ | ★★ | `pose_guard.dead_reckon_s` | 3 s | ASSUMED | MANUAL | 3: TF 가 끊겼을 때 오도메트리로 이어 가는 최대 시간(그 뒤 'pose_lost' 로 정지) | SLAM 노드를 잠깐 멈춰 로봇이 얼마나 가다 서는지 본다 | 3 s 안에 정지·복귀 |
| ☐ | ★★ | `pose_guard.fallback_deg` | 5 deg | ASSUMED | MANUAL | 5: 추정 틀어짐이 이 이상이거나 검증을 못 했으면 Nav2 대신 내장 A* 로 간다 |  |  |
| ☐ | ★★ | `pose_guard.jump_m` | 0.45 m | ASSUMED | MANUAL | 0.45: SLAM 자세가 한 번에 이만큼 뛰면 도약 의심(연속 jump_confirm_n 번 확인 전까지 보류) | SLAM 루프 폐쇄가 정상일 때 도약 크기를 로그로 본다 | 정상 루프 폐쇄는 통과 |
| ☐ | ★★ | `pose_guard.reanchor_ambiguity` | 0.06 | ASSUMED | MANUAL | 0.06: 최적과 '다른 후보' 의 점수 차 하한. 대칭·반복 구조·벽 하나만 보이는 자리를 걸러 낸다 | 경기장 곳곳에서 정지해 로그 거부 사유(ambiguous)가 나오는 자리를 본다 | 구석·중앙 모두에서 적합 성공, 긴 벽 한쪽만 볼 때 거부 |
| ☐ | ★★ | `pose_guard.reanchor_min_ratio` | 0.4 | ASSUMED | MANUAL | 0.40: 새 자리에서 벽·구조물에 설명되는 끝점 비율 하한(박스·UGV 가 많은 구역은 낮게 나온다) | 박스가 많은 구역에서 정상 자세의 맞음 비율을 재어 그보다 낮게 정한다 |  |
| ☐ | ★★ | `pose_guard.reanchor_search_xy` | 0.5 m | ASSUMED | MANUAL | 0.50: 기준선 오프셋 둘레 탐색 반경(±). reanchor_max_m 보다 넉넉히 | 충돌로 실제로 얼마나 어긋나는지(줄자)를 보고 정한다 |  |
| ☐ | ★★ | `pose_guard.reanchor_ugv_margin_m` | 0.45 m | ASSUMED | MANUAL | 0.45(+반경 0.30): 앵커 재설정 적합에서 알려진 UGV 둘레 이 반경 안의 스캔 끝점을 뺀다(UGV 는 구조물이 아니다). UGV 위치는 UAV 틀이고 끝점은 어긋났을 수 있는 앵커로 옮긴 것이라 여유를 둔다 | UGV 를 벽 옆 0.5 m 에 세우고 로그/stats['pose_guard']['reanchor']['ugv_excluded'] 와 맞음 비율(UGV 가 있어도 기준선 근처 유지)을 본다. UGV 가 구조물로 섞이면(맞음 하락) 반경을 키운다 | UGV 근처에서도 재설정이 오적용 0 |
| ☐ | ★★ | `robot.tilt_limit` | 15 deg | PDF | MANUAL | ±15 (PDF: 수직 30°, 중심 가정) | 서보 영점(수평)이 맞는지 |  |
| ☐ | ★★ | `uav_boxes.edge_margin_m` | 0.3 m | ASSUMED | MANUAL | 0.30: 벽에서 이 거리 안의 덩어리는 버린다(벽 센서 상자·중앙 패드 등 고정 구조물 오검출 방지). 대신 벽 바로 옆 상자를 놓칠 수 있다 | 벽 0.4 m 옆에 둔 상자가 잡히는지 확인, 잡히지 않으면 줄인다 |  |
| ☐ | ★★ | `uav_boxes.fill_min` | 0.82 | SIM_TUNED | MANUAL | 0.82: 0.4 m 칸에서 윗면이 차지하는 비율 하한. 시뮬 20개 상자에서 0.75~0.80 까지 20/20·오검출 0, 0.70 이하는 오검출 | 실제 영상 detect 결과의 셀별 fill 과 reject 사유를 본다 |  |
| ☐ | ★★ | `uav_boxes.v_black_rel` | 0.45 | ASSUMED | MANUAL | 0.45: 실제 문턱 = min(v_black_max, max(rel×바닥 V, v_black_min)) — 조명이 바뀌어도 바닥 대비로 따라간다 | probe 제안값 |  |
| ☐ | · | `aim.body_kp` | 1.6 | SIM_TUNED | MANUAL | 1.6: 팬 → 차체 회전 | 실기 차체 회전 응답 |  |
| ☐ | · | `aim.lost_timeout` | 0.8 s | SIM_TUNED | MANUAL | 0.8: 이 시간 안 보이면 추적 해제 | 검출 주기·끊김에 맞춰 |  |
| ☐ | · | `aim.sensor_min_tol_deg` | 0.3 deg | ASSUMED | MANUAL | 0.30: 팬 1단위 = 0.5° 라 양자화 오차가 ±0.25° [PDF p14] (틸트 0.15°) | 팬 1단위 이동으로 센서 원이 화면에서 몇 픽셀 움직이는지 |  |
| ☐ | · | `aim.settle_move_deg` | 1 deg | ASSUMED | MANUAL | 1.0: 이보다 큰 짐벌 명령 뒤에만 gimbal_settle_s 만큼 기다린다 | gimbal_settle_s 를 정한 뒤, 작은 보정에도 대기가 필요한지 본다 |  |
| ☐ | · | `avoid.m2_defer_max_s` | 30 s | SIM_TUNED | MANUAL | 30: 회피로 M2 가 미뤄진 합계 상한 |  |  |
| ☐ | · | `avoid.strike_cooldown_s` | 3 s | SIM_TUNED | MANUAL | 3: 회피 직후 지정표적 재접근 쿨다운 |  |  |
| ☐ | · | `avoid.w_close` | 1.5 | SIM_TUNED | MANUAL | 1.5 | 〃 |  |
| ☐ | · | `event.smog_fire_conf` | 0.6 | ASSUMED | MANUAL | 0.60 | 스모그 중 검출 신뢰도 분포로 |  |
| ☐ | · | `event.smog_speed_scale` | 0.8 배 | ASSUMED | MANUAL | 0.8 | 스모그 중 실제 시야에 맞춰 |  |
| ☐ | · | `event.tower_min_t` | 60 s | ASSUMED | MANUAL | 60: 시작 후 이 시간 전에는 타워로 정지 안 함 |  |  |
| ☐ | · | `event.uav_off_s` | 8 s | ASSUMED | MANUAL | 8: 새 프레임이 이만큼 없으면 드론 중단 | UAV 영상 주기 변화 실제 값(1→2→5 s) |  |
| ☐ | · | `explore.lidar_scout_standoff_m` | 0.65 m | ASSUMED | MANUAL | 0.65: 가려진 쪽 정찰 지점에서 떨어질 거리 | 정찰 지점이 장애물에 너무 가깝지 않은지(Nav2 팽창 반경)·뒷면 LiDAR 광선이 통과하는지 확인 |  |
| ☐ | · | `explore.m1_cm_defer` | 0.3 | ASSUMED | MANUAL | 0.3: 적 섹터가 끝나기 전 CM 가중에 곱하는 보류 계수(0 에 가까울수록 CM 을 미룬다) |  |  |
| ☐ | · | `explore.m1_cm_weight` | 1.6 | ASSUMED | MANUAL | 1.6: CM 구역 후보 가중(풀린 뒤) |  |  |
| ☐ | · | `explore.m1_enemy_weight` | 2.4 | ASSUMED | MANUAL | 2.4: M1 이 지시한 적 섹터 면을 향하는 후보에 곱하는 가중 | 적 섹터와 CM 이 섞인 시나리오에서 순서를 확인 |  |
| ☐ | · | `explore.m1_sector_weight` | 1.6 | ASSUMED | MANUAL | 1.6: M1 지정 구역 탐색 후보에 곱하는 가중(1 = 다른 구역과 동일) | M1 구역과 다른 구역이 섞인 시나리오에서 구역 소화 시간 비교 |  |
| ☐ | · | `explore.uav_census_window_s` | 4 s | ASSUMED | MANUAL | 4: 첫 UAV 상자 채택 뒤 이 시간 안에 들어온 상자만 '초기 영상'으로 센다(이후 상자는 변화로 취급) | UAV 영상 첫 장 이후 상자가 늘어나는 시간을 본다 |  |
| ☐ | · | `explore.w_threat` | 0.8 | SIM_TUNED | MANUAL | 0.8: 효용 = 정보×(1−min(0.9,w·위험))/시간. 시뮬에서 정한 임의 가중치 | 회피 시나리오 비교 곡선으로 조정 |  |
| ☐ | · | `hal.battery_low_pct` | 20 % | ASSUMED | MANUAL | 20: 이 아래면 깊이 영상 처리 중단(배터리 교체 불가, 약 40분 [PDF p6]) | 연속 경기 후 실제 배터리 소모 |  |
| ☐ | · | `hal.dark_frames` | 30 장 | ASSUMED | MANUAL | 30: 아주 어두운 영상이 이만큼 연속이면 암흑으로 본다 | 경기장의 가장 어두운 곳에서 1분 찍어 오탐이 0 인지 확인 | 오탐 0 |
| ☐ | · | `hal.gimbal_acc` | 20 | PDF | MANUAL | 20 (권장값) [PDF p14] |  |  |
| ☐ | · | `hal.impact_g` | 3 g | ASSUMED | MANUAL | 3.0: 충격 이벤트 기록 한계(정지는 하지 않음) | 벽/박스 접촉 때 가속도 크기 기록 |  |
| ☐ | · | `hal.locked_hold_s` | 1 s | ASSUMED | MANUAL | 1.0: 제어 허용 중 fire/gimbal 이 locked 를 주면 이 시간 피격 잠금으로 본다 | 피격 후 locked 응답이 실제 몇 초 동안 오는지 |  |
| ☐ | · | `impact.hold_s` | 0.4 s | ASSUMED | MANUAL | 0.40: 충돌 직후 최소 정지(큰 충격 0.80). 이 동안 LiDAR 틀어짐 검증용 정지 표본을 모은다 | 충격 후 SLAM·스캔이 안정되는 시간을 로그로 본다 |  |
| ☐ | · | `impact.tilt_scan_skip_deg` | 8 deg | ASSUMED | MANUAL | 8: |roll|·|pitch| 가 이보다 크면 그 스캔은 격자에 쌓지 않는다 | 경사로·충돌 때 스캔이 바닥·천장을 훑는 각도를 본다 |  |
| ☐ | · | `laser.disabled_s` | 10 s | PDF | MANUAL | 10(피격 후 정지) | 규정집 |  |
| ☐ | · | `laser.practice_empty_backoff_s` | 10 s | ASSUMED | MANUAL | 10: 연습 모드에서 empty 응답 뒤 다시 쏘기까지 쉬는 시간. [p70] 연습 모드 탄약 회복 속도를 모른다 | 연습 모드에서 탄약이 다시 차는 속도를 재서 정한다 |  |
| ☐ | · | `map3d.beam_margin_m` | 0.1 m | ASSUMED | MANUAL | 0.10(표적 면 자체는 사선 막힘에서 제외) | 표적 앞 박스 모서리가 사선을 가리는 상황을 눈으로 확인 |  |
| ☐ | · | `map3d.beam_timeout_s` | 3 s | ASSUMED | MANUAL | 3(허상으로 보고 통과) | 실기에서 사선 막힘이 오래 가는 오탐이 있는지 로그(stats beam_override) |  |
| ☐ | · | `map3d.bump_ttl_s` | 90 s | ASSUMED | MANUAL | 90 | 경기 길이 300 s 에 맞춘 값 — 장애물이 움직이는지에 따라 |  |
| ☐ | · | `map3d.guard_half_width` | 0.15 m | ASSUMED | MANUAL | 0.15(차체 폭 0.23 의 절반 + 여유) | 실제 폭 줄자 |  |
| ☐ | · | `mission.aim_budget_s` | 25 s | SIM_TUNED | MANUAL | 25: 조준 단계 시간 예산 | 실기 조준 소요 시간 분포로 |  |
| ☐ | · | `mission.m3_resubmit_s` | 5 s | ASSUMED | MANUAL | 15: M3 갱신 제출 간격(집계가 바뀔 때만) |  |  |
| ☐ | · | `mission.recover_after_s` | 4 s | SIM_TUNED | MANUAL | 4: 항법이 이 시간 넘게 막히면 RECOVER |  |  |
| ☐ | · | `mission.recover_back_s` | 0.7 s | SIM_TUNED | MANUAL | 0.7 후진 시간(≈ 7 cm @0.1 m/s) | 실기 후진 거리·후방 여유 |  |
| ☐ | · | `mission.spin_w` | 1.1 rad/s | SIM_TUNED | MANUAL | 1.1(시점 도착 후 360° 회전 속도) | 회전 중 검출 블러·미끄러짐 |  |
| ☐ | · | `mission.t_base_cmd` | 240 s | PDF | MANUAL | 240(종료 1분 전 목표거점 명령) | 규정집 |  |
| ☐ | · | `mission.t_report` | 30 s | PDF | MANUAL | 30(종료 후 M6 제출 기한) | 규정집 |  |
| ☐ | · | `mission.t_total` | 300 s | PDF | MANUAL | 300(경기 5분) | 규정집 |  |
| ☐ | · | `mission.ugv_range` | 1.4 m | SIM_TUNED | MANUAL | 1.40(M4 교전 거리) | M4 는 기본 OFF |  |
| ☐ | · | `oak.check_topics_after_s` | 8 s | ASSUMED | MANUAL | 8: 기동 뒤 이 시간이 지나면 토픽이 있는지 본다 | MAICON_OAK=1 로 띄워 토픽이 나오기까지 걸린 시간 |  |
| ☐ | · | `oak.launch_args` | () | ASSUMED | MANUAL | ()(예: camera_model:=OAK-D-LITE — 드라이버가 모델 이름을 요구하는지 미확인) | 드라이버 launch 파일의 인자 목록 |  |
| ☐ | · | `oak.params_file_arg` | params_file | ASSUMED | MANUAL | params_file(파라미터 파일을 넘기는 런치 인자 이름 가정) | 드라이버 launch 파일에서 실제 인자 이름 |  |
| ☐ | · | `oak_det.depth_agree_tol` | 0.45 비율 | ASSUMED | MANUAL | 0.45: 깊이 거리와 크기 추정이 이 비율 넘게 다르면 깊이를 버린다(상자 안에 바닥·다른 물체) | stats['oak_depth'] 의 disagree 횟수가 정상 관측에서 거의 없어야 한다 | 정상 관측의 버림 < 10 % |
| ☐ | · | `oak_det.depth_max_dt_s` | 0.35 s | ASSUMED | MANUAL | 0.35: 검출 프레임과 깊이 프레임의 촬영 시각 차 상한. 검출이 늦게 나와 촬영 시각으로 짝을 찾는다 | stats['oak_depth'] 의 no_frame 비율이 높으면 올린다(UGV 가 느리므로 0.5 까지) | 깊이 사용률 ≥ 70 % |
| ☐ | · | `oak_det.depth_roi_frac` | 0.5 비율 | ASSUMED | MANUAL | 0.50: 상자 가운데 이 비율만 본다(가장자리는 배경이 섞인다) | UGV 가 작게 보일 때(멀리) 거리 오차가 크면 0.6, 배경이 섞이면 0.4 |  |
| ☐ | · | `oak_det.iff_weight_boost` | 1.5 배 | ASSUMED | MANUAL | 1.5: 깊이로 거리를 확인한 가까운 UGV 의 피아 득표 가중(1 = 없음) | 아군 UGV 를 적으로 오분류하는 횟수가 줄어드는지 비교 |  |
| ☐ | · | `oak_det.save_max` | 8 장 | ASSUMED | MANUAL | 8(0–10). 확인용 링 버퍼 크기 | 5–10장 범위에서 저장 공간과 노트북 확인 편의를 보고 조정 |  |
| ☐ | · | `oak_det.vote_window` | None 프레임 | ASSUMED | MANUAL | None(= 팬틸트 쪽과 같은 규칙, 창 없음). 동물 탐지 경험값 15–17프레임은 카메라·용도가 달라 비교 후보로만 둔다 | 기본 / 8 / 16 을 같은 주행으로 비교 |  |
| ☐ | · | `oak_det.weight` | 0.5 | ASSUMED | MANUAL | 0.5: OAK 관측 한 번 = 팬틸트의 절반 표(근거 없는 가정) | OAK 만 본 면의 오분류율을 팬틸트와 비교해 정한다 |  |
| ☐ | · | `pose_guard.degraded_speed_scale` | 0.75 배 | ASSUMED | MANUAL | 0.75: 자세를 못 믿는 동안(저하 상태) 속도에 곱한다 |  |  |
| ☐ | · | `pose_guard.reanchor_cooldown_s` | 15 s | ASSUMED | MANUAL | 15: 적용 뒤 다음 시도까지 최소 간격(연쇄 충돌에서 앵커가 흔들리지 않게). 한 경기 최대 reanchor_max_count=4 |  |  |
| ☐ | · | `pose_guard.reanchor_hold_s` | 3 s | ASSUMED | MANUAL | 3: 큰 충격·도약 뒤 표본을 모으려고 멈춰 서 있는 최대 시간 | 정지 표본 3 개를 모으는 데 실제로 걸리는 시간을 본다 |  |
| ☐ | · | `pose_guard.reanchor_max_m` | 0.6 m | ASSUMED | MANUAL | 0.60: 한 번 보정 한도(넘으면 거부하고 보고만). 누적 한도 reanchor_total_m=1.2 |  |  |
| ☐ | · | `pose_guard.shift_max_deg` | 40 deg | ASSUMED | MANUAL | 40: 이보다 큰 추정은 믿지 않는다(자세 상실로 처리) |  |  |
| ☐ | · | `rig.camera_rpy` | (0, 0, 0) rad | ASSUMED | MANUAL | (0,0,0) 가정 | 틸트 0° 일 때 RGB 광축이 수평인지 수평계로 확인 |  |
| ☐ | · | `rig.oak_rgb_rpy` | (0, 0, 0) rad | ASSUMED | MANUAL | (0,0,0) 가정 | 렌즈 축이 정면인지·기울었는지 수평계·눈으로 확인 |  |
| ☐ | · | `robot.pan_limit` | 50 deg | PDF | MANUAL | ±50 (PDF: 수평 100°) | 서보 실제 가동 범위 확인 |  |
| ☐ | · | `uav_boxes.min_conf` | 0.6 | ASSUMED | MANUAL | 0.6: 이 미만은 manager 가 받지 않는다(coverage.ingest_uav_boxes 의 문턱과 같다) |  |  |
| ☐ | · | `uav_boxes.ppm` | 128 px/m | ASSUMED | MANUAL | 128: 윗면 평면으로 펴는 해상도. 128 이면 상자 한 변이 약 51 px | 느리면 낮춘다(검출 시간 elapsed_ms 로 확인) | elapsed_ms < 100 |
| ☐ | ★★★ | `nav.inflate_hard` | 0.164 m | ASSUMED | calib:avoid | 0.164 = 반폭 0.115 + 계획 여유 0.049 (파생; 직진 통과 기준). 경로 확정은 직사각형 검증이 따로 한다. 예전 0.17 + 완화(0.8·0.6배)는 폐기 | 40 cm 통로 직진·비스듬한 진입 실주행, calib_run.py avoid |  |
| ☐ | ★★ | `nav.side_lim` | 0.182 m | ASSUMED | calib:avoid | 0.178 = 반폭 0.115 + 정지 여유 0.067 (파생) | 40 cm 통로 중앙(0.20)에서 측면 보정이 작동하지 않는지 |  |
| ☐ | · | `nav.lookahead` | 0.38 m | SIM_TUNED | calib:avoid | 0.38: 경로 추종 선행 거리 | 실기 추종 진동 여부 |  |
| ☐ | · | `nav.stuck_move` | 0.05 m | SIM_TUNED | calib:avoid | 0.05 |  |  |
| ☐ | · | `nav.stuck_s` | 3 s | SIM_TUNED | calib:avoid | 3: 내장 항법기 끼임 판정 시간 |  |  |
| ☐ | ★★★ | `nav.stop_dist` | 0.192 m | ASSUMED | calib:brake | 0.192 = 앞 반길이 0.125 + 정지 여유 0.067 (센서 오차+지연; 파생). 제동거리는 slow_dist 감속 구간이 맡는다 | 벽 앞 정지 위치(차체 앞 끝과 벽 사이)를 줄자로 |  |
| ☐ | ★★ | `avoid.reverse_speed` | 0.12 m/s | ASSUMED | calib:brake | 0.12: 근거리 후진 속도 | 실제 후진 정지거리 |  |
| ☐ | ★★ | `nav.slow_dist` | 0.45 m | ASSUMED | calib:brake | 0.45 | 속도별 정지거리에서 유도 |  |
| ☐ | ★★★ | `camera.D` | None (k1..k4) | ASSUMED | calib:camera | None(어안일 때만 필요, 없으면 경고 후 핀홀) | camera.model 과 같은 어안 보정에서 K 와 함께 나온다 | RMS < 0.5 px |
| ☐ | ★★★ | `camera.K` | None px | ASSUMED | calib:camera | None(= hfov 로 근사). 어안 보정 때 K·D 를 같이 구한다(camera.model·camera.D) | 체커보드 보정 |  |
| ☐ | ★★★ | `camera.hfov_deg` | 110 deg | ASSUMED | calib:camera | 110(160° 렌즈의 보정 후 유효 화각 가정) | 체커보드 보정(calib_fit.py --camera-images) | RMS < 0.5 px |
| ☐ | ★★★ | `camera.model` | pinhole | ASSUMED | calib:camera | "pinhole"(기본). [p45] OV5640 160° 어안 — 보정으로 K·D 를 구한 뒤에만 "fisheye" 로 둔다 | calib/camera.py 어안 보정을 640×480 으로 한다. OpenCV 4.x 를 쓴다(5.x 에서는 cv2.fisheye.calibrate 가 수렴하지 않음) | RMS < 0.5 px |
| ☐ | ★★ | `lidar_err.mount_xy_m` | 0.005 m | ASSUMED | calib:lidar_square | 0.005 [가정] | calib_run.py lidar_square 잔차 |  |
| ☐ | ★★ | `lidar_err.mount_yaw_deg` | 0.5 deg | ASSUMED | calib:lidar_square | 0.5 [가정] | 〃 |  |
| ☐ | ★★ | `rig.lidar_xyz` | (0, 0, 0.2) m | ASSUMED | calib:lidar_square | (0,0,0.20) 가정 | calib_run.py lidar_square 로 x, y, yaw; z 는 줄자 |  |
| ☐ | ★★ | `robot.track_width` | 0.2 m | ASSUMED | calib:rotate | 0.20 [p78] 판매 사양(4륜 구동, 타이어 80 mm)과 대회 사양이 다르다 — 대회 차체로 다시 잰다 | 좌우 바퀴 중심 간격 줄자 |  |
| ☐ | ★★ | `robot.yaw_eff` | 1 비율 | ASSUMED | calib:rotate | 1.0 [p78] 판매 사양과 대회 사양이 달라 대회 차체로 다시 잰다 | calib_run.py rotate |  |
| ☐ | ★★★ | `robot.v_max` | 0.45 m/s | ASSUMED | calib:speed_steps | 0.45 가정 | calib_run.py speed_steps → calib_fit.py → calib_generate.py |  |
| ☐ | ★★ | `nav.v_cruise` | 0.38 m/s | ASSUMED | calib:speed_steps | 0.38 | calib 에서 유도 |  |
| ☐ | ★★ | `robot.a_max` | 1.2 m/s² | ASSUMED | calib:speed_steps | 1.2 가정 | calib_run.py speed_steps / brake |  |
| ☐ | · | `nav.v_slow` | 0.18 m/s | SIM_TUNED | calib:speed_steps | 0.18 |  |  |
| ☐ | ★★ | `robot.lr_bias` | 0 rad/m | ASSUMED | calib:straight | 0 | calib_run.py straight |  |
| ☐ | ★★ | `robot.w_max` | 2.2 rad/s | ASSUMED | calib:turn_steps | 2.2 가정 | calib_run.py turn_steps |  |
| ☐ | ★★★ | `uav_boxes.calibration` | None | ASSUMED | calib:uav | None(실기 설정 calib/uav_main_calibration.json). 시뮬 보정 파일을 실기에 쓰지 않는다 | UAV 'main' 카메라의 4점 또는 격자 보정 파일을 만든다(캘리브 도구) — 영상은 왜곡 보정된 핀홀이어야 하고, 보정 영역 크기가 cfg.arena 와 같아야 한다 | 검출기 로딩 시 크기 일치 점검 통과 |

## 시뮬에서 정한 값 모음 (실기에서 다시 확인할 것)

시뮬의 잡음·허상·키네매틱 서보 가정에 맞춰 정한 값이다. 근거를 적어 두었으니 현장 결과와 비교해 갱신한다.

| 경로 | 현재값 | 실험 | 정한 근거 |
|---|---|---|---|
| `depth.sim_noise_k` | 0.004 | F03 | 시뮬 깊이 잡음 모델 σ=k·z² 의 k=0.004(스테레오 일반론에서 가정). 시뮬 충실도용 값이라 로봇 동작에는 직접 안 쓰인다 |
| `depth.sim_dropout` | 0.03 | F03 | 시뮬 무효 픽셀 3 % 가정 |
| `depth.sim_flying_pixels` | 0.3 | F06 | 시뮬 가장자리 번짐 30 % 가정 — 이 허상 때문에 3D 층 필터(nav_thr 등)를 정했다 |
| `map3d.nav_thr` | 1.8 | F06 | 1.8(약 3회 관측). 시뮬에서 허상 0 이 되도록 정함(가장자리 번짐 30 % 가정) |
| `map3d.nav_min_neighbors` | 3 | F06 | 3(면/덩어리만 통과, 광선 방향 한 줄 허상 제거). 시뮬에서 정함 |
| `map3d.blind_grow_cells` | 3 | F06 | 3: 박스 둘레 15 cm 번짐을 'LiDAR 가 보는 곳'으로 처리. 시뮬 잡음에 맞춤 |
| `map3d.blind_floor_layers` | 2 | F05 | 2: 바닥에서 올라온 5~15 cm 물체만. 박스 윗부분 잔재를 거르려고 정함 |
| `map3d.nav_z_lo` | 0.06 | F05 | 0.06(바닥 잡음 위) |
| `map3d.stride` | 2 | F05 | 2 + 프레임마다 위치 순환: 12 cm 물체가 영상에서 몇 줄밖에 안 돼서 3→2 로 바꿨다(시뮬 해상도 96x72 기준) |
| `map3d.integrate_hz` | 5 | F05 | 5(시뮬 깊이 주기와 같게) |
| `map3d.bump_window_s` | 1.5 | F07 | 1.5: 시뮬에서 정체를 1.5 s 안에 알아챔 |
| `map3d.bump_v_min` | 0.08 | F07 | 0.08 |
| `map3d.bump_ratio` | 0.3 | F07 | 0.3(실제/기대 이동). 시뮬의 SLAM 자세 오차 ~1 cm 에 맞춤 |
| `aim.tol_deg` | 1.2 | MANUAL | 1.2: 시뮬 서보·검출 지터 기준 |
| `aim.tol_deg_far` | 0.7 | MANUAL | 0.7 |
| `aim.kp_pan` | 0.9 | MANUAL | 0.9: 시뮬 키네매틱 서보 기준 |
| `aim.kp_tilt` | 0.9 | MANUAL | 0.9 |
| `mission.strike_range` | 1.1 | MANUAL | 1.10(시뮬 서브게임에서) |
| `event.smog_on_ratio` | 0.04 | F08 | 0.04: 시뮬 스모그(허상 반사 16 %·30 % 가 8~20 cm)에서 정함. 실제 연기의 LiDAR 반사 분포는 모른다 |
| `event.smog_off_ratio` | 0.015 | F08 | 0.015 |
| `mission.ugv_range` | 1.4 | MANUAL | 1.40(M4 교전 거리) |
| `aim.hold_before_fire` | 0.2 | MANUAL | 0.20: 정렬이 이만큼 안정되면 발사. [p72] 짐벌 ok 는 접수일 뿐이라 aim.gimbal_settle_s 와 함께 본다 |
| `aim.lost_timeout` | 0.8 | MANUAL | 0.8: 이 시간 안 보이면 추적 해제 |
| `aim.body_kp` | 1.6 | MANUAL | 1.6: 팬 → 차체 회전 |
| `mission.aim_budget_s` | 25 | MANUAL | 25: 조준 단계 시간 예산 |
| `mission.recover_after_s` | 4 | MANUAL | 4: 항법이 이 시간 넘게 막히면 RECOVER |
| `mission.recover_back_s` | 0.7 | MANUAL | 0.7 후진 시간(≈ 7 cm @0.1 m/s) |
| `mission.spin_w` | 1.1 | MANUAL | 1.1(시점 도착 후 360° 회전 속도) |
| `nav.lookahead` | 0.38 | calib:avoid | 0.38: 경로 추종 선행 거리 |
| `nav.v_slow` | 0.18 | calib:speed_steps | 0.18 |
| `nav.stuck_s` | 3 | calib:avoid | 3: 내장 항법기 끼임 판정 시간 |
| `nav.stuck_move` | 0.05 | calib:avoid | 0.05 |
| `event.smog_near_m` | 1.2 | F08 | 1.2: 이보다 가까운 고립 반사만 본다 |
| `event.smog_on_s` | 0.8 | F08 | 0.8 |
| `event.smog_off_s` | 3 | F08 | 3.0 |
| `event.speckle_nbr_m` | 0.25 | F08 | 0.25: 이웃 빔 거리차 기준 |
| `explore.strategy` | online | MANUAL | online(시뮬 6시드에서 이동거리 -7 %·충돌 감소, 재현율은 고정 순회보다 약 5 %p 낮음). lattice 로 되돌리면 기존 5x5 순회 |
| `explore.m3_counting` | ledger | MANUAL | ledger(박스 면 1개=객체 1개). omap 은 위치 병합이라 같은 면을 두 번 세는 오류가 시뮬에서 났다 |
| `explore.w_threat` | 0.8 | MANUAL | 0.8: 효용 = 정보×(1−min(0.9,w·위험))/시간. 시뮬에서 정한 임의 가중치 |
| `explore.deadline_margin_s` | 3 | MANUAL | 3: prestage_at 앞 여유 |
| `explore.stage_margin_s` | 4 | MANUAL | 4: M5 명령(t_base_cmd) 앞 여유 |
| `uav_boxes.fill_min` | 0.82 | MANUAL | 0.82: 0.4 m 칸에서 윗면이 차지하는 비율 하한. 시뮬 20개 상자에서 0.75~0.80 까지 20/20·오검출 0, 0.70 이하는 오검출 |
| `avoid.enable` | True | MANUAL | True. 끄면 기존 동작(위협을 모르는 항법) |
| `avoid.r_aware` | 3 | MANUAL | 3.0 |
| `avoid.r_evade` | 1.6 | MANUAL | 1.6 |
| `avoid.r_close` | 0.8 | MANUAL | 0.8 |
| `avoid.p_cons` | 0.7 | MANUAL | 0.7: 팀 미확인 UGV 의 최소 위험 가중 |
| `avoid.lidar_blob_min_speed` | 0.15 | MANUAL | 0.15: 새로 드러난 정적 박스의 자세 오차성 겉보기 이동을 거르려고 정함(시뮬 오탐 120 s 에 0~2회) |
| `avoid.w_risk` | 3 | MANUAL | 3.0: 위험 비용 가중. 시뮬에서 피격↔시간 균형 |
| `avoid.w_close` | 1.5 | MANUAL | 1.5 |
| `avoid.risk_extra_inflate_m` | 0.07 | MANUAL | 0.07: 위험 인지 경로가 벽을 타지 않게. 시뮬에서 벽 따라가다 LiDAR 제한에 걸려 M5 를 놓친 뒤 정함 |
| `avoid.t_margin_s` | 8 | MANUAL | 8: M5 시간 여유 계산 상수 |
| `avoid.fresh_s` | 1.6 | MANUAL | 1.6: 소유권(EVADE/CLOSE)을 거는 관측의 최대 나이. UAV 지연 2 s 에서 오래된 위치로 움직이면 기준선보다 맞았다(시뮬) |
| `avoid.cap_s` | 25 | MANUAL | 25(60 s 창): 회피가 구동을 쥐는 시간 상한 — 임무 지연 방지 |
| `avoid.strike_cooldown_s` | 3 | MANUAL | 3: 회피 직후 지정표적 재접근 쿨다운 |
| `avoid.m2_defer_max_s` | 30 | MANUAL | 30: 회피로 M2 가 미뤄진 합계 상한 |

## 레지스트리에 없는 설정 경로

`Config` 에는 있지만 튜너블 표에 없는 값(= 현장 확인 대상으로 분류하지 않은 값): `robot.height`, `robot.collision_model`, `robot.radius_legacy`, `robot.pan_rate`, `robot.wheel_v_max`, `robot.wheel_limit`, `hal.cmd_topic`, `hal.odom_topic`, `hal.image_topic`, `hal.imu_topic`, `hal.imu_heading_fusion`, `hal.imu_heading_max_age_s`, `hal.state_topic`, `hal.match_topic`, `hal.oled_topic`, `hal.gimbal_srv`, `hal.fire_srv`, `hal.ros_namespace`, `hal.v_cap`, `hal.w_cap`, `hal.cmd_hz`, `hal.pan_deg_per_unit`, `hal.tilt_deg_per_unit`, `hal.gimbal_unit_max`, `hal.gimbal_min_interval_s`, `hal.gimbal_timeout_s`, `hal.locked_retry_s`, `hal.fire_wait_s`, `hal.cam_max_fps`, `hal.tilt_clear_deg`, `hal.fire_left_ms_max`, `hal.oled_cols`, `hal.oled_lines`, `hal.oled_min_interval_s`, `hal.oled_refresh_s`, `hal.oled_period_s`, `hal.match_cmd_stop_backup`, `hal.step_budget_s`, `hal.restart_state_path`, `camera.width`, `camera.height`, `camera.calibration_profile`, `depth.enable`, `depth.width`, `depth.height`, `depth.K_width`, `depth.K_height`, `depth.require_calibration`, `depth.optical_frame`, `depth.tf_tolerance_m`, `depth.tf_tolerance_deg`, `depth.nav_allowed`, `depth.blind_reverse_mps`, `depth.sim_hz`, `map3d.enable`, `map3d.res`, `map3d.z_max`, `map3d.floor_z`, `map3d.max_free_range`, `map3d.hit` …

