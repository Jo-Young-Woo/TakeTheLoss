# 적 UGV 회피 알고리즘 설계서 · 검증 계획

> **갱신(구현 후)**: 이 문서는 구현 *전* 설계다. 실제로 구현한 것과 달라진 점(위험 반경 `r_core` 0.9→2.2 m, GOAL/STAGE 에서 EVADE 소유권 불가, 경유지 히스테리시스, 정지 금지 구역, LiDAR 군집 조건 등)과 시뮬 결과는 `docs/M3탐색_적회피_통합_설계.md` 가 기준이다. 이 문서에서 `[미구현]`으로 표시된 `perception/lidar_blobs.py`·`mission/threat_board.py`(실제 위치 `perception/threat_board.py`)·`mission/threat_field.py`·`mission/avoid.py` 는 지금 구현되어 있다. 아직 없는 것: 돔(전방 ±105°) 노출 계수, `NavStack.follow_path`, Nav2 `FollowPath` 직접 전송, 현장 실험 F11~F13 코드.
> 작성 2026-11-05 · **원래 설계 문서이며 이 문서를 쓸 때는 코드를 수정하지 않았다.** 아래 사실은 README가 아니라 소스(`maicon26/…`)와 설정값(`config.py`, `ros2/config/nav2_params.yaml`)을 직접 읽고 쓴 것이다.
> 표기: **[확인]** 소스에서 확인한 사실 · **[가정]** 하드웨어·규정·상대 행동에 대한 가정(현장 확인 전) · **[설계]** 이번 제안 · **[미구현]** 지금 코드에 없음.

---

## 0. 한 장 요약

**문제의 정체(소스로 확인).** 기본 설정(`engage_ugv=False`)에서 적 UGV 정보는 **이동 결정에 한 번도 쓰이지 않는다.** `UgvTracker`는 만들어지지만 **트래커 결과의 소비처는 ① M6 위험도 증거(`threat.py`), ② HUNT 후보(`_hunt_candidates`, HUNT가 꺼져 있어 도달 안 함) 둘뿐**이다. (사격선 아군 확인 `_friend_in_line`은 트래커가 아니라 그 틱의 카메라 검출 `self._dets`를 직접 쓴다.) 이동은 전적으로 LiDAR 일반 장애물 처리(`SafetyLimiter` + Nav2/내장 A*)이고, 막히면 **적과 무관한 방향으로** 돌거나 후진한다.

**권고 한 가지.** `ThreatBoard`(증거 사다리 + 예측) → `ThreatField`(물리 점유 + 시간 층별 피격 위험) → **Agent 쪽 위험 인지 경로계획(내장 A*에 비용 층 추가)** → 짧은 **임시 경유지(carrot)** 를 Nav2에 전달(내장 백엔드는 같은 경로를 직접 추종). Nav2 비용지도 계층(C++)·마킹 점군·keepout 마스크는 채택하지 않는다(근거 §3.2).
근거리(≤0.8 m)는 `AvoidController`가 소유권을 가져가 **정지·재관측·측면 우회·후퇴 후보 점수화**를 하고, 충돌 안전 제한(LiDAR 정지/감속, 깊이 가드)은 **모든 경로의 마지막 필터로 항상 유지**한다.

**지금 코드로 되는 것 / 안 되는 것.**
| 입력 | 현재 상태 |
|---|---|
| LiDAR 360° | **된다.** 단 적 UGV를 구분하지 못한다(분류 없음) |
| 카메라 YOLO UGV + 안테나색 팀 | 코드는 있으나 **가중치 파일이 저장소에 없고**(`*.pt/*.onnx/*.engine` 0개) HSV 임계는 미교정 → 실기 **미검증** |
| UAV 관측 | **시뮬에서만.** 실기 `HttpServerLink.get_uav()`는 `None` 반환 스텁, UAV 영상→경기장 좌표 검출기는 **소스에 없다** |
| 깊이 카메라 | 지도·사선 확인용. UGV 분류에는 안 쓴다(LiDAR-visible은 3D 층에서도 제외) |

따라서 이 설계는 **UAV·YOLO 없이도(LiDAR만으로) 최소 동작**하고, 카메라·UAV가 있으면 정확도가 올라가는 구조로 짠다.

---

## 1. 현재 구조 파악 (소스 기준)

### 1.1 이동 경로와 적 정보의 연결 — 어디서 끊기는가
```
카메라 프레임 ─YoloDetector(UGV,team_from_antenna)─▶ Agent._ingest_vision ─▶ UgvTracker.update_camera ┐
UAV 프레임(시뮬만) ─▶ Agent._sense ─▶ UgvTracker.update_uav ────────────────────────────────────────────┤
                                                                                                       ▼
                                                                                      tracker.tracks (x,y,vx,vy,team_votes)
                                                                  ┌─ threat.evidence_from_tracks → M6 순위(STOP 후)
                                                                  ├─ _hunt_candidates → HUNT   (engage_ugv=False → 도달 안 함)
                                                                  └─ (아군 확인) _friend_in_line은 self._dets 직접 사용(트래커 아님)
LiDAR ─▶ SafetyLimiter / Nav2 costmap / OccupancyGrid  (적을 '장애물'로만) ─▶ _nav_to ─▶ _apply
                       ▲▲ 여기에 적 정보가 들어가는 선이 없다
```
[확인] 근거: `manager.py` `_arbitrate`(963–978: `hunt_ok = mc.engage_ugv and …` → 기본 False), `_hunt_candidates`(950), `_nav_to`(1008–1016: 목표만 받음), `_decide`(880–929).

### 1.2 구성요소별 실제 상태
| 구성요소 | 실제로 하는 일 [확인] | 한계 |
|---|---|---|
| `mission/manager.py` `_decide` | 순서: SAFE_STOP 사유 → 발사 잠금 → **M5(GOAL) 최우선(t ≥ t_base_cmd−1=239 s, m5_type 있을 때)** → RECOVER → 발사 중 유지 → STAGE(≥222 s) → `_arbitrate`(STRIKE/HUNT/EXPLORE) | 어느 단계도 적 위치를 입력으로 받지 않음 |
| `_enter_recover` / `_mode_recover` (1031–1065) | 후방 ±40°가 0.30 m 넘게 비면 −0.10 m/s 로 **0.7 s(≈7 cm)** 후진, 이어서 **좌/우 LiDAR(±70°±35°) 중 더 트인 쪽으로 0.9 rad/s 1.4 s 제자리 회전**(≈72°). 3회/40 s 초과 시 `recover_exhausted` SAFE_STOP 5 s | 회전 방향은 **LiDAR 여유 비교뿐**. 적이 트인 쪽에 있어도 그쪽으로 돈다 |
| `nav/safety.py` `SafetyLimiter.limit` (24–61) | 전방 ±35° < `stop_dist` 0.17 m → v=0, **가장 트인 쪽으로 w=±0.9**; 2 s 넘게 막히면 `blocked_long`(복구 신호). 0.45 m 안은 감속. 후진도 후방 ±35° 검사 | 팀·위협 무관. 회전 방향이 적 쪽일 수 있음 |
| `nav/navigator.py` `step` (132–170) | 끼임 3 s(이동 <5 cm) → 1.3 s **후진 −0.12 + `rng.choice([-1,+1])` 로 무작위 회전**(156–157) | 무작위 방향 = 적 쪽 가능 |
| `nav/nav2_stack.py` `step` | 목표를 Nav2 `NavigateToPose`로 보내고(`_send`), `/cmd_vel_nav2`를 `limit()` 후 사용. 연속 실패 시 `_enter_fallback`(내장 A* 항법기로 이 목표만) | Nav2 자체 계획·복구(BT spin/backup)는 적을 모름 |
| `ros2/config/nav2_params.yaml` | 전역/지역 비용지도 = LiDAR 마킹+클리어링 + `/arena_walls_cloud`(마킹 전용) + `/depth_obstacles_cloud`(마킹 전용), 인플레이션 반경 0.30, RPP(0.35 m/s, `use_collision_detection: true`), `movement_time_allowance: 8` | 적 UGV = 인플레이션 0.30 의 일반 장애물. 위험 영역 개념 없음 |
| `perception/tracks.py` `UgvTracker` | 카메라 검출(`ugv_conf_min` 0.50, LiDAR로 거리, r ≤ 4.0 m)과 UAV 검출을 **0.55 m(+0.35·나이) 게이트**로 같은 트랙에 합치고 속도 EMA(a=0.5). `team` = 득표 ≥ 65 %. UAV 식별은 **t<3 s 에 거점 중심 0.75 m 이내**일 때만(`ident`) | 공분산·신뢰도 없음. `enemies()`는 **팀 미상(+ident 없음)을 적으로 포함**(178행), `friends()`는 미상을 제외 → 보수적 방향은 이미 맞음 |
| `perception/detector.py` | `YoloDetector`: UGV 박스 → `team_from_antenna`(bbox 상단 40 % 의 빨강/파랑 HSV 비율, 불확실하면 None). `SimDetector`: 시뮬 검출 전달 | **가중치 없음**, HSV 임계는 "현장 사진으로 교정할 것"(주석) |
| 시뮬 센서 (`sim/world.py`) | UGV 검출: 거리 ≤ 3.5 m, 박스에 가리면 안 보임, 누락 5 %+4 %/m(2 m 초과), **팀 4 % 뒤집힘**, 오검출(false positive) 없음. UAV: 1 Hz, 지연 0.4 s, 위치 σ 0.04 m, **팀 정확** | 실기보다 훨씬 후한 가정 |
| 카메라 시야 | RGB: hfov 110° [가정], 팬 ±50° [PDF] → 몸체 기준 약 ±105° 만 볼 수 있다. **후방 약 150°는 카메라 사각** | "뒤따라오는 적"은 카메라로 못 본다 |
| LiDAR | 360°, 최대 8 m [가정: D500 `range_max`], 10 Hz | UGV와 박스·벽을 구분하지 않음 |
| 피격 규칙 (`config` + `sim/world.py:_beam_hit/_contact`) | 수광센서 2개(좌/우 앞 모서리 돔), **발사자가 피격자 헤딩 ±105° 안에 있어야 유효**(뒤에서 쏘면 차체가 가림, 시뮬 가정), 유효거리 `laser.max_range` **2.5 m [가정]**, **2 s 연속 수광 [PDF]**, 0.3 s 유예, 피격 후 10 s 무적·무장 정지, **4번째 누적 시 10 s 주행 정지** | 돔이 앞쪽에만 있다는 것은 PDF p21 사진에서의 **[가정]** |

### 1.3 "구현 완료로 간주하지 않는" 항목
1. 실기 HAL(`hal/wingbot.py`)·서버 링크(`link/http.py`) — 스텁.
2. YOLO 가중치·클래스 `UGV`·안테나색 HSV — 학습·교정 전.
3. UAV 영상 수신·검출·경기장 좌표 변환 — 소스에 없음(시뮬은 정답 좌표를 직접 줌).
4. 깊이 카메라 실기 영상 — 시뮬 렌더로만 시험.
5. 상대 UGV의 실제 행동(추격·사격 여부) — 모름. 시뮬 상대는 같은 Agent(HUNT OFF)라 **지금 시뮬에서는 우리가 맞을 일이 거의 없다** → 검증 전에 적대적 상대부터 만들어야 한다(§7).

---

## 2. 위협 판단

### 2.1 증거 사다리 (세 단계 + 아군)
```
OBJECT  물체만 확인    LiDAR 군집(크기 0.15~0.35 m, 지도에 없던 것 또는 움직임) — 카메라/UAV 확인 없음
UGV     UGV로 확인     YOLO UGV(conf ≥ ugv_conf_min) 또는 UAV 검출이 군집과 연관됨, 팀은 미상/충돌
ENEMY   적 UGV로 확인  UGV이면서 p_enemy ≥ 0.85 (팀 판정이 서로 일관되게 누적)
FRIEND  아군 UGV       UGV이면서 p_friend ≥ 0.90  (위험 계수 0.15, 물리 점유는 그대로)
```
**처리 원칙(보수성).** `OBJECT`가 **움직이면** `UGV(팀 미상)` 취급, 팀 미상은 **적과 같은 위험 가중**(`p_cons = 0.7`)을 쓴다. 아군 확정은 높은 문턱(0.90)과 일정 횟수 일관 관측으로만. 근거: 아군이라 믿고 접근했는데 적이면 피격, 적이라 믿고 피했는데 아군이면 시간 손실뿐이다(비대칭).

### 2.2 팀 믿음 갱신 (`p_enemy`)
관측 하나(팀 라벨 `c ∈ {RED, BLUE, None}`, 출처 `s ∈ {cam, uav}`)마다 로그오즈 갱신:
```
if label is None: 갱신 없음
else:  LLR = ln(r_s / (1 − r_s)) · sign(label is opponent team ? +1 : −1) · min(1, conf)
       logit(p_enemy) += LLR · decay(age)           # r_cam, r_uav: 라벨 신뢰도
시간 감쇠: 마지막 라벨 관측이 오래되면 p_enemy → 0.5 쪽으로(반감기 T_half=6 s)
```
- `r_cam`: 시뮬 0.96(뒤집힘 4 %)은 시뮬 값. **실기는 현장 사진으로 측정해야 한다**(§7, 필드 실험 F11 제안). 초깃값 0.80 [가정].
- UAV는 `ident`(t<3 s 거점 식별)가 확정이면 p=1/0 로 고정(다만 트랙 ID 스위치 가능성이 README에도 적혀 있다 → 위치 연속성이 깨지면 해제).

### 2.3 상태 추정: 위치·속도·오차
`Track`에 **없는 것을 추가**한다([미구현]): `sigma`(위치 불확실도), `src_mask`(cam/uav/lidar 마지막 관측 시각), `p_enemy`, `level`.
```
σ_obs   LiDAR 군집 0.06 m [가정, 시뮬 σ 0.008+중심 추정오차] · 카메라(LiDAR 거리) 0.08 · UAV 0.04(시뮬)/0.15 [가정, 실기 모름] + 지연 보정 v·0.4 s
σ(t)    sqrt(σ_obs² + (σ_v · age)²),  σ_v = 속도 추정 오차(UAV 1 Hz 차분 → ≈ √2·σ_obs/Δt)
속도    |v| ≤ v_enemy_max = robot.v_max(0.45) [가정: 상대도 같은 로봇]. 초과 추정은 클램프
```
UAV 영상 지연·주기 변화는 **`UavMonitor`(E4, 구현됨)** 의 `state/period` 를 `age` 계산에 반영한다: `slow2/slow5/off` 면 `σ_v·age` 가 커져 위험 반경이 자동으로 커진다. **UAV가 `off` 이거나 실기에서 미구현이어도 LiDAR·카메라만으로 동작해야 한다**(필수 요건).

### 2.4 LiDAR 만으로 하는 최소 감지 (카메라·UAV 없이) — `perception/lidar_blobs.py` [미구현, 설계]
```
입력: 스캔(마스크 적용 후), SLAM 자세, 정적 지도 L(OccupancyGrid)
1) 스캔 점 → 월드. '새 점' = 그 셀의 L ≤ 0(자유/미지)이었는데 지금 반사가 생긴 점, 정적 점유 셀과 1칸 이상 떨어진 것
2) 3 스캔 연속 지속 + 거리 0.12 m 군집 → 크기(호 길이) 0.15~0.35 m 만 UGV 후보 (큰 것은 박스·벽)
3) 중심 = 군집 중심에서 시선 방향으로 +r_body(0.13) 이동(원기둥의 앞쪽 호만 보이므로)
4) 10 Hz 연관(게이트 0.4 m) → 속도. 로봇 회전 중(|w|>1.0 rad/s)에는 새 점 판정을 보류(등록 오차로 정적 모서리가 '움직임'으로 보이는 것 방지)
출력: Blob(x, y, vx, vy, size, n_scans) → ThreatBoard 에서 OBJECT 로 시작
```
- **정지한** 적은 이 방법으로 안 잡힌다(정적 장애물로 격자에 들어감). 정지 UGV 후보를 "박스 격자(40×40 cm)에 안 맞는 작은 고립 군집"으로 잡는 규칙은 박스 한 면만 보인 경우와 구분이 어려워 **기본 OFF**로 두고 카메라 확인에만 맡긴다.
- 깊이 카메라 높이(UGV 0.32 vs 박스 0.40)로 보강할 수 있으나 실기 영상 미검증 → 보조 옵션.
- 한계: SLAM 자세 지터가 큰 곳에서 오탐 가능 → 오탐은 위험 계수를 낮게 시작(`p_obj = 0.5`)해 영향 제한. 실제 오탐률은 **시뮬 회귀로 측정**한 뒤 문턱 결정.

### 2.5 예측과 접근 방향
```
예측 위치   p̂(τ) = p + v·τ,  τ ∈ {0, 0.75, 1.5, 2.25} s (K=4 시간 층; 마지막 층은 이후에도 유지)
예측 반경   ρ(τ) = r_body + k_σ·σ(τ),  k_σ = 1.5       (r_body = 0.13 시뮬 UGV_R / 실기 0.17 [가정])
접근도     closing = −d(d)/dt = −(p−q)·(v_e−v_us)/|p−q|     (q = 우리 위치)
TTC        (d − r_body − r_us)/max(closing, 0.05)
CPA        상대속도로 최근접 시각·거리(횡단 시 yield 판단)
```
**우리 접근 방향**을 반영하는 이유는 두 가지다: ① 경로 위 각 지점의 도착 시각 `t_arr = s/v_cmd` 에서의 적 예측 위치와 비교(시간 층 선택), ② **돔 노출**: 우리 헤딩 ±105° 안에서만 맞는다는 가정 하에 `facing 계수`(2단계, §3.3).

### 2.6 위협 수준(LEVEL) — 진입/해제 문턱이 다르다 (히스테리시스, §4.5)
| 수준 | 진입(OR) | 해제(AND, 유지시간) | 하는 일 |
|---|---|---|---|
| L0 CLEAR | — | — | 기존 동작 |
| L1 AWARE | 위협 d < 3.0 m 또는 TTC < 6 s | d > 3.6 m 그리고 TTC > 9 s, 1.5 s 유지 | 위험 비용 켠 경로계획, 속도 ×0.85, 팬을 위협 쪽으로 편향(팀 확인), 로그 |
| L2 EVADE | d < 1.6 m 또는 TTC < 2.5 s 또는 경로 연속 노출 > 1.0 s | d > 2.2 m 그리고 TTC > 5 s, 2.0 s 유지 | 높은 `w_risk`, **M2 조준 중단**, Nav2 목표 취소(소유권 이전), 재계획 2 Hz |
| L3 CLOSE | d < 0.8 m 또는 LiDAR 전/후방 정지구간에 위협 근접 | d > 1.1 m, 1.0 s 유지 | §4 근거리 기동(정지/재관측/측면/후퇴). 충돌 안전 최우선 |

(수치는 **시뮬 초깃값 + 실측 보정 대상** — §6.3.)

### 2.7 LiDAR가 UGV를 못 알아봐도 유지하는 기본 안전
`SafetyLimiter`·깊이 가드·접촉(정체) 감지는 **그대로 두고 항상 마지막 필터**다. 팀 미상이거나 `OBJECT`만 확인된 UGV는 L1~L3 에서 **적과 같은 보수 규칙**을 쓴다. 위협 인지가 통째로 실패해도(트랙 0개) 동작은 지금과 같다 → **후퇴 가능성(degrade-safe)**.

---

## 3. 경로계획

### 3.1 두 영역을 따로 모델링
| | 물리 점유(Body) | 피격 위험(Risk) |
|---|---|---|
| 의미 | 적 차체가 **실제로 있을 수 있는** 공간. 닿으면 충돌 | 적이 **우리를 맞힐 수 있는** 공간·시간 |
| 모양 | 시간 층 k 마다 원판 `ρ(τ_k)` 의 합집합 | 거리 감쇠 × 시야 × (선택)돔 노출 × 팀 가중 |
| 사용 | **하드 차단**(A* `blocked`) — 우리 반경 인플레이션 포함 | **소프트 비용**(A* 가중) + 연속 노출 시간 제한 |
| 해제 | 트랙 만료 시 ghost 로 확대 후 감쇠(§3.5) | 같음 |

```
Body_k(cell)  = 1  if  dist(cell, p̂_i(τ_k)) ≤ ρ_i(τ_k) + r_us_hard   for some threat i   (r_us_hard = nav.inflate_hard 0.17)
Risk_k(cell)  = Σ_i  w_team(i) · f_range(d_ik) · V_ik(cell)  · [face_ik(cell)]      (상한 1)
  w_team  = max(p_enemy, p_cons=0.7)   (FRIEND: 0.15)
  f_range = 1                         d ≤ r_core (0.9 m)
          = ((r_out − d)/(r_out − r_core))²   r_core < d < r_out,  r_out = laser.max_range + 0.3 = 2.8 m [가정]
          = 0                         d ≥ r_out
  V_ik    = 적 예측 위치에서 cell 까지 정적 점유 셀을 가로지르지 않으면 1 (시야가 트임), 아니면 0.15 (잔여: 예측 오차·모서리)
  face    = 1 (1단계: 항상). 2단계: 우리 진행방향이 적을 향하면(±105°) 1, 등을 지면 0.15
```
- `V_ik`(시야): 격자에서 적 중심 기준 극좌표 72 레이 × 56 스텝(2.8 m/0.05) 광선 투사 → 가려지면 0. **박스가 레이저를 막는다**는 시뮬 규칙(`_beam_hit`의 큐브 검사)과 같은 가정이다. 추정 비용: 위협 1개당 ≈ 4 k 샘플(numpy 벡터화).
- 시간 층 K=4: `Body_k`, `Risk_k` 를 4장(127×104 격자)만 만든다. 위협 3개 × 4층 × (원판+시야) ≈ 수 ms [가정, Jetson 측정 필요].

### 3.2 Nav2 3가지 방안 비교와 권고
| 방안 | 어떻게 | 장점 | 결정적 약점(이 프로젝트 사정) |
|---|---|---|---|
| **A. 동적 비용지도 계층** (`nav2_costmap_2d::Layer` 플러그인) | C++ 플러그인이 위험장을 비용으로 그린다 | 등급 있는 비용, Nav2 계획기·컨트롤러가 직접 사용 | **C++ 빌드·Jetson Docker 배포**가 필요하고 이 PC에서 컴파일·시험 불가(WSL에서도 플러그인 미작성). 내장 A* 백엔드와 **코드가 둘로 갈라진다** |
| **B. 제한 구역** (keepout/speed 필터 마스크, 또는 마킹 점군) | 위험 영역을 `/filter_mask` 나 PointCloud2 로 발행 | 설정만으로 가능 | ① 마킹 점군은 **지워지지 않는다**(이 프로젝트에서 이미 확인한 `obstacle_layer` 한계: 사라진 물체의 표시를 LiDAR 클리어링이 못 지움) → 움직이는 적에게 **치명적**. ② 필터 마스크 값의 비용 환산·갱신 방식은 **Humble 소스/실측 미확인**. ③ 등급 비용이 아니라 사실상 이진 |
| **C. 임시 경유지 선택** (Agent가 위험 비용 A*로 경로를 만들고 그 위 **전진 경유지 carrot** 을 Nav2 `NavigateToPose`로 전송) | `Nav2Stack._send` 재사용(목표가 0.15 m 넘게 바뀌면 재전송 이미 구현) | **내장 A* 백엔드와 같은 코드**(경로계획이 Agent에 있다). Nav2 변경 없음. 시뮬(`nav2-fake`)과 WSL 실제 Nav2 모두에서 시험 가능 | Nav2가 carrot 사이를 자체 계획하므로 위험을 100 % 지키진 못한다(carrot 간격 ≤1.0 m로 편차 제한). 목표가 계속 움직여 컨트롤러가 흔들릴 수 있음 → **WSL 실측 필요** |

**권고: C.** 근거리(L3)와 Nav2 실패 시에는 이미 있는 **내장 추종기로 소유권 이전**(`_enter_fallback` 경로, `Navigator._follow/_limit` 재사용). Nav2 `FollowPath` 액션에 경로를 직접 넣는 방법은 후속 옵션(Humble API 실측 후).

### 3.3 위협 비용 계산식 (A*, 내장 백엔드와 Nav2 경유지 공통)
간선 (a→b), 길이 `ℓ`, 도착 시각 `t_b = s_b / v_plan` (s: 누적 거리, `v_plan = nav.v_cruise × speed_scale`):
```
k      = min(K−1, floor(t_b / Δτ))
blocked(b) = static_blocked(b)  OR  Body_k(b)                       # 하드
edge_cost  = ℓ · c_static(b)                                        # 기존 cost_maps 비용(soft 인플레이션 포함)
           + w_risk · Risk_k(b) · (ℓ / v_plan)                      # 위험 × 노출 '시간' (risk·초)
           + w_close · max(0, (r_close_cost − d_enemy(b,k))/r_close_cost)²       # 근접 추가 항(근거리 압박)
           + [2단계] w_dome · Risk_k(b) · face(dir, b) · (ℓ / v_plan)
w_risk_eff = w_risk · (0.35 + 0.65·κ)           # κ: 시간 여유 계수(§5.4)
```
A* 휴리스틱은 기존(거리 휴리스틱)을 그대로 쓴다 — 추가 항이 모두 **양수**라 허용(admissible)이다. 시간 의존 비용이라 FIFO가 정확히 성립하진 않지만(기다리지 않는 한 늦게 도착해 더 싼 경우가 드묾) 구현은 단순하며, **최적성 증명이 아니라 시험으로 확인**한다(§7).

**경로 후검증(연속 노출 제한).** A* 결과 경로에서 `Risk_k(b) ≥ r_hit(0.5)` 인 구간의 **연속 시간** `T_exp` 를 계산:
```
T_dwell = laser.dwell_s (2.0 s, [PDF]) ,  T_exp_limit = 0.6·T_dwell = 1.2 s  (안전 마진)
if T_exp > T_exp_limit:   w_risk ← ×2 후 재계획(≤3회) → 그래도 초과면 후보 경로 없음 → §4 기동(HOLD/우회/후퇴) 선택
```
유효 피격에는 연속 2 s 수광이 필요하다는 **[PDF] 규칙**이 곧 "짧은 횡단은 허용, 긴 노출은 금지"의 근거다. (적이 이미 조준 중일 수 있어 마진 0.6배를 둔다. 적의 서보 지연·조준 시간은 **[가정]**.)

### 3.4 내장 A* 백엔드에서 같은 원칙이 작동하는 방법
- `OccupancyGrid.cost_maps(...)`(현재 `hard, soft, soft_w, extra_block, extra_r` 인자)에 **`threat: Optional[ThreatField]`** 를 추가: `blocked |= Body_k` 는 A* 안에서 `k` 가 경로 길이에 따라 정해지므로 `astar()`에 `layer_of(t)` 콜백을 넘기는 변형(`astar_threat`)을 둔다. 층 수 K=4라 파이썬 오버헤드는 셀당 인덱싱 한 번.
- `Navigator._plan` 의 **인플레이션 완화 루프(1.0→0.8→0.6)** 는 위협 층에서는 `Body_k` 에 적용하지 않는다(적 주변 반경을 줄여 좁은 데로 파고들지 않게). 정적 장애물에만 완화 허용.
- `Navigator._path_invalidated` 에 **위협 변화 검사**를 추가: 기존 경로를 현재 위험 층으로 재평가해 비용이 `Δ>X` 늘거나 `Body_k` 와 교차하면 즉시 재계획.
- `Nav2Stack` 은 같은 `self.grid`(= `_fb.grid`)로 **같은 A\*** 를 돌려 경로를 얻고 carrot 을 보낸다. 따라서 **백엔드가 달라도 위험 계산은 한 곳**(`mission/threat_field.py`).

### 3.5 갱신·확대·해제 규칙
| 상황 | 규칙 |
|---|---|
| 적이 움직임 | 매 관측마다 `p̂, σ` 갱신, 층 재생성은 **0.25 m 이상 이동했거나 분류 변경 또는 0.5 s 경과** 시에만(재계획 폭주 방지) |
| 관측이 끊김(트랙 나이 증가) | `age < T_hold(2 s)`: 예측 유지. 이후 **ghost**: `ρ_ghost(age) = min(ρ0 + v_enemy_max·(age − T_hold), 1.2 m)` 로 확대, `T_ghost=6 s` 후 위험 가중을 선형 감쇠해 해제(LiDAR가 실제로 보는 것은 물리 장애물로 남는다) |
| UAV `slow2/slow5/off` | `age` 임계를 `uavmon.track_max_age`(이미 구현)와 같은 비율로 늘림. UAV가 유일한 관측일 때 `σ_obs`를 키워 반경 확대 |
| 팀 미상 → 아군 확정 | 위험 가중만 0.15 로 감소(2 s 이상 일관). 물리 점유는 유지 |
| 적과 아군이 동시에 근접 | 합집합. 아군도 `Body` 하드 차단 |
| 경기장 구조물 재배치 | 정적 격자 갱신으로 `V_ik`(시야) 재계산 트리거 |

---

## 4. 근거리 회피와 후퇴

### 4.1 공통 장치
**상대 기하 분류**(우리 몸체 기준 적 방위 β, 거리 d, 접근속도 closing):
```
FRONT      |β| < 35°  (정면 가로막음)
LATERAL    35° ≤ |β| < 110°  (옆에서 접근/횡단)
REAR       |β| ≥ 110°  (뒤에서 추격) — 카메라 사각, LiDAR/UAV 만으로 인지
CORRIDOR   로컬 통로 폭 W_c ≤ 0.8 m (좌우 LiDAR 여유 합) 이고 적이 통로 안에 있음
```
**Breadcrumb(지나온 경로) 저장** — `SafeWaypointBuffer`:
```
0.25 m 이동마다 {x, y, th, t, clearance(그때 LiDAR 최소거리), threat_free(그때 위협 d>2.5 m 였나)} 를 링버퍼(최대 40점≈10 m)에 저장
후퇴 후보 = breadcrumb 점 + 링 샘플(반경 1.0·1.5·2.0 m, 16방위)
```
**후보 검증(무조건 후진 금지)** — 후보 c 는 다음을 **현재** 시점에 모두 통과해야 한다:
1. 정적 자유·인플레이션 통과(옛 점이 새 박스에 막히지 않았나)
2. 후방/진행 경로 LiDAR 여유(`min_in_sector`) ≥ 0.6 m(후진은 ≥ 0.6, 전진 회피는 정지거리 이상)
3. 도착 시각에서 적 예측 위치와의 거리 ≥ `ρ + 0.5 m`, 그리고 `Body_k` 와 교차 안 함
4. `Risk_k(c)` 가 현재 위치 위험보다 **작다**(같은 위협으로 되돌아가는 것 방지) — "적에게 접근하는 후퇴" 배제
5. 적과의 거리가 후퇴 동안 **단조 비감소**(경로를 따라 `d_enemy(s)` 가 현재 d 이상)
점수 = `경로 위험 적분 + 0.3·길이 − 0.2·min(clearance)`, 최저 선택. 통과 후보가 없으면 §4.6.

**후퇴 실행 방식.** 후진은 **≤ 0.5 m, ≤ 0.15 m/s** 에서만(후방 LiDAR 검사 매 틱). 그보다 길면 "적을 등지게 회전 → 전진" — 회전 방향은 **적 방위를 우리 헤딩 ±105° 밖으로 가장 빨리 보내는 쪽**(돔 노출 최소화 [가정: 돔 전방만]).

### 4.2 FRONT (적이 앞을 가로막음)
```
if 적이 정지/후퇴(closing ≤ 0.05) and d ≥ 0.6 and 적 헤딩이 우리와 마주 보지 않음:
        HOLD(정지) + 팬 관측(팀 확인), 최대 T_hold_max=3 s
elif 측면 우회 후보(좌/우) 중 [far-side 여유 ≥ 0.45 m(=2·r_inflate+여유) and Risk 적분 < 정면 대기]:
        측면 우회(위험 비용 A*가 자연히 선택. 적 정면 통과 금지: 적 전방 ±60° 부채꼴은 w_close ×2)
elif 후퇴 후보 존재:  후퇴(§4.1)
else:   §4.6
```
- **안전 제한이 먼저 회전하는 경우**(`front < 0.17`): `SafetyLimiter`의 "트인 쪽" 선택을 **`turn_pref` 힌트로 대체**(§5.3). 힌트가 없으면(위협 없음) 기존 동작.

### 4.3 LATERAL (옆에서 접근/횡단)
CPA 계산(§2.5). `t_cpa < 4 s` 이고 `d_cpa < r_safe(0.8 m)` 이면:
- **YIELD(감속·정지)**: 우리가 `HOLD +1.5 s` 했을 때 경로 위험 적분이 줄면 대기(층 인덱스에 +1.5 s 오프셋을 줘 재평가하는 것으로 구현 — 위험 층 재사용, 새 계산 없음)
- 아니면 **적 후방 쪽으로 통과**(적 속도 벡터의 뒤쪽 경로가 `Risk` 적분이 작다. 단, 적이 우리 쪽으로 방향을 틀 가능성을 `σ` 로 반영)
- 우리가 **가로지르는 중 정지 금지**(노출 시간 연장). 이미 노출 구간 진입 후엔 계속 진행(속도 ↑ ≤ v_cruise)이 더 안전 [가정: 2 s 연속 수광 필요]

### 4.4 REAR (뒤에서 추격)
- 카메라로 못 본다 → **LiDAR 군집/UAV**가 유일한 근거(L2 승격은 LiDAR 군집이 0.5 s 이상 우리 뒤 ±70° 에서 **접근 속도 ≥ 0.1 m/s** 일 때).
- 기본 대응: **계속 앞으로 간다**(등을 보이는 한 돔이 가려진다는 [가정]). **정지·제자리 회전 금지**(회전하면 돔을 노출). M2 조준(정지) 금지.
- 시야 차단 우선: 경로 비용의 `V_ik` 항이 박스 뒤로 숨는 경로를 선호.
- 추격자가 d < 0.8 이고 전방이 막혔으면 → 측면 우회 후보(적 반대편) → 없으면 HOLD 후 §4.6.
- **막다른 길 회피**: 후보 경로 평가에 "출구 수(기하 개방도)" 항(도착점 주변 0.8 m 반경 자유 각도 ≥ 120°)을 넣는다.

### 4.5 CORRIDOR (좁은 통로에 함께 있음)
로봇 직경 ≈ 0.28~0.34 m [가정] 이라 폭 0.6 m 미만에서는 **교행 불가**.
```
if 적이 통로 안 '앞쪽' 이고 접근/정지:  YIELD — 가장 가까운 '넓은 곳'(국소 폭 ≥ 1.2 m 인 교차로·구역) 까지 후퇴 후보로 평가
       (breadcrumb 이 통로 입구면 그대로, 후방 0.6 m 여유 확인) — 후퇴가 불가능하면 HOLD 최대 4 s 후 재관측
if 적이 통로 안 '뒤쪽'(추격):  통로 출구가 1.5 m 안이면 계속 전진해 빠져나간다. 출구가 멀면 속도 유지, 정지 금지
통로 안 진입 전: 경로가 폭 < 0.6 m 구간을 지나고 그 구간에 적의 예측(σ 포함)이 걸치면 진입 금지(Body_k 가 하드 차단 → 대안 경로)
```

### 4.6 안전한 후퇴가 불가능할 때 (정지 · 재관측 · 측면 우회 선택)
```
후보 점수(작을수록 좋음):
 HOLD(정지+관측)   = Risk_now · T_hold_expect  + 0 (이동 없음)         조건: 적이 접근 중이 아님(closing<0.1) 또는 d>1.0
 SIDEBYPASS        = A*(적 반대편, w_close×2) 위험 적분                  조건: 측면 여유 충분
 SLOWCREEP         = 같은 경로 v=0.1 로 통과, 노출 시간 증가 → 비선호     조건: 다른 길 없음 & 시간 압박
 FREEZE_SAFE       = 정지 + 팬 스윕으로 팀 확정, 5 s 후 재평가           조건: 위 모두 불가
선택 후 타이머: 같은 선택이 T_commit_min(1.5 s) 동안 유지(진동 방지), 총 대기 예산 초과 시 SLOWCREEP/돌파
```
`SAFE_STOP`(구동 0)은 **센서 노후·피격 주행 잠금·복구 소진**용으로만 남기고, 적 회피 대기는 `AVOID.HOLD`(충돌 안전 제한 활성, 팬 관측 가능)로 구분한다 — 그래야 재관측으로 팀을 확정할 수 있다.

### 4.7 임무 복귀와 히스테리시스
```
복귀 조건(모두): level ≤ L1 가 2.0 s 유지 · 마지막 위협 d > 2.2 m · 현재 위치에서 원래 목표까지 위험 적분 < Risk_ok · AVOID 최소 유지 1.0 s 경과
복귀 시: stack.clear_goal() (옛 AVOID 목표 취소) → 원래 모드의 목표 재설정(_cur_goal=None) · aim.reset() · STRIKE 는 phase="nav" 로 재진입(조준 재시작)
M2 재접근 쿨다운: 같은 적이 ≤2.0 m 에 있으면 T_cool=3 s 동안 STRIKE 재진입 금지, 그 뒤 재개. M2 포기 상한: 이번 경기 AVOID 로 인한 M2 연기 합계 > T_m2_defer(30 s) 이면 위험 가중을 낮춰 진행
진동 방지: ① 진입/해제 문턱 분리(§2.6) ② 최소 유지시간 ③ 같은 위협에 대한 EVADE 재진입 횟수 제한(30 s 에 4회 초과 시 AWARE 로 격하하고 위험 비용만 사용)
```

---

## 5. 상태 전이와 안전 우선순위

### 5.1 우선순위 (높음 → 낮음)
| 순위 | 항목 | 현재 | 변경 |
|---|---|---|---|
| 0 | **STOP/HALT**(서버 STOP·game_state finished·타워 보조·로컬 만료) | `_serve` 가 센서·SLAM보다 먼저, `_enter_halt` | **AvoidController.reset(), carrot 취소** 추가. 회피 중이든 즉시 구동 0 |
| 1 | **SAFE_STOP**(pose_not_ready, lidar_stale, drive_locked, recover_exhausted) | `_unsafe_reason` | 변경 없음. 회피 상태는 보존하지 않고 재개 시 L0 부터 재평가 |
| 2 | **물리 충돌 방지**(LiDAR 정지/감속, 깊이 가드, 접촉 감지) | `SafetyLimiter`, `_depth_guard`, `_stall_check` | **어느 소유자의 명령이든 마지막에 적용**(`_apply` 마지막 단계). 예측 `Body_k` 위반 시 v 상한 |
| 3 | **AVOID L3 CLOSE** | 없음 | **GOAL 보다도 우선**(단 `commit_dash` 예외 §5.4) |
| 4 | **M5 GOAL** | `_decide` 에서 M5가 RECOVER/STAGE/STRIKE 보다 위 | L1/L2 에서는 **GOAL 이 소유권 유지, 경로계획만 위협 인지**(§5.4) |
| 5 | **AVOID L2 EVADE** | 없음 | STRIKE·STAGE·EXPLORE 선점 |
| 6 | **RECOVER**(막힘 복구) | 후진 7 cm + 트인 쪽 72° 회전 | **방향 제약**(§5.3) |
| 7 | **M2 STRIKE**(접근·조준·발사) | `_mode_strike` | L2 이상에서 조준 중단·재접근 쿨다운 |
| 8 | **STAGE / M3 EXPLORE** | | L1 은 위험 인지 경로로 진행 |

### 5.2 상태 전이표 (회피 상태 `AvoidState`는 `Mode`와 직교)
| 현재 | 이벤트/조건 | 다음 | 동작 | 비고 |
|---|---|---|---|---|
| (모든) | STOP/HALT 래치 | HALT | `avoid.reset()`, 목표·carrot 취소, 구동 0 | 순위 0 |
| (모든) | `_unsafe_reason` ≠ None | SAFE_STOP | 구동 0, `avoid` 일시중지 | 재개 시 L0 부터 |
| CLEAR | 위협 d<3.0 또는 TTC<6 | AWARE | 위험 비용 계획 on, 속도 ×0.85, 팬 편향 | |
| AWARE | EVADE 진입 조건 | EVADE | 소유권 이전(`stack.clear_goal()`, `epoch+=1`), STRIKE 조준 중단 | |
| AWARE | 해제 조건 1.5 s | CLEAR | 정상 | |
| EVADE | CLOSE 진입 조건 | CLOSE | §4 기동 선택 | |
| EVADE | 해제 조건 2.0 s 유지 + 최소 유지 1.0 s | AWARE | 복귀 절차 §4.7 | |
| CLOSE | 후퇴/우회 후 d>1.1, 1.0 s | EVADE | | |
| CLOSE | 후보 없음 | CLOSE.HOLD/FREEZE | §4.6 | 대기 예산 소모 |
| 어느 L | 위협 트랙 만료(ghost 해제) | 한 단계 하향 | 위험 가중 감쇠 | 한 번에 두 단계 내리지 않음 |
| GOAL 중 CLOSE | `commit_dash` 성립 | (GOAL 유지) | 돌진 §5.4 | |
| RECOVER 진입 | 위협 있음 | RECOVER(방향 제약) | §5.3 | |

### 5.3 명령 중재 — 소유권(owner)과 이전 요청·속도 정리
**단일 쓰기 원칙(이미 있음):** `_count_output` 이 한 틱에 두 번 출력을 세어 `multi_output` 으로 드러낸다. 이를 **소유자 토큰**으로 확장한다.
```
owner ∈ {MISSION(탐색/STRIKE/STAGE/GOAL), AVOID, RECOVER, SAFE, HALT}
owner 변경 시(예: MISSION→AVOID):
   1) stack.clear_goal()        → Nav2: client.cancel() + CmdCache.invalidate()  (옛 /cmd_vel_nav2 속도 폐기, 이미 구현된 게이트)
   2) self.epoch += 1           (이전 목표의 늦은 결과 무시 — 기존 R04 메커니즘)
   3) Nav2Stack._sent_goal=None (재전송 억제 상태 초기화)
   4) _v_cmd 램프: 정상 복귀는 램프 유지, 위협 급접근은 emergency=True 로 v 를 즉시 0~후진 상한으로(램프 우회, 안전 제한은 유지)
복귀(AVOID→MISSION): 새 목표로만 stack.set_goal() — 옛 목표 재개 금지(_cur_goal=None)
```
**RECOVER 방향 제약.**
```
rear_ok  = 후방 ±40° ≥ 0.30  AND  threat_behind(적 예측 위치가 후방 ±60° 에서 d<1.5 m) == False
turn dir = argmax over {좌,우} of  (LiDAR 여유 + w_t · 적 방위와의 각 벌어짐)   — 적 쪽으로 돌지 않는다
          (적이 정면이면 적 반대 쪽으로 돌아 적 방위가 ±105° 밖으로 나가게)
적이 후방 근접이면 후진 금지(회전도 하지 않고 전진 가능 쪽 우선, 전진도 막혔으면 HOLD)
```
- `SafetyLimiter.limit(..., turn_pref=None)` 인자 추가: 값이 있으면 "더 트인 쪽" 대신 그 방향. `Navigator` 의 `rng.choice([-1,1])` 도 `turn_pref` 로 대체.
- Nav2 BT 의 spin/backup 복구(`navigate_replan_fast.xml`; 값은 `calib/robot_profile.yaml` 의 `nav.recovery`: `spin_dist 1.57`, `backup 0.25 m @0.05` 기준이며 **XML 본문은 이번에 읽지 않았다**)는 적을 모른다 → **위협 L2 이상에서는 Nav2 목표를 아예 보내지 않는다**(소유권 AVOID). L1 에서 Nav2 복구가 시작되면 `Nav2Stack` 이 이미 갖고 있는 `client.recovering()` 신호를 감지해 같은 방향 제약을 적용하거나 목표를 취소.

### 5.4 시간(M5) 예산 — 회피가 M5를 삼키지 않도록
```
t_left    = t_total − t
ETA_goal  = 위험 무시 최단 경로 길이 / v_cruise           (현재 경로계획기로 계산)
slack     = t_left − 1.3·ETA_goal − T_margin(8 s)
κ         = clamp(slack / T_soft(25 s), 0, 1)              (여유 비율)
w_risk_eff= w_risk · (0.35 + 0.65·κ)                        (여유 없으면 위험 가중 ↓, 단 0.35배 아래로는 안 내림)
hold_budget = max(0, slack)                                  (HOLD/후퇴에 쓸 수 있는 총 시간)
```
- `slack ≤ 0` ("커밋 모드"): HOLD/후퇴 금지, 가장 위험이 낮은 **전진** 경로로 돌파. 그래도 **하드 차단(`Body_0`·LiDAR 정지·깊이 가드)은 유지**.
- **commit_dash**(M5 진입 직전): 남은 거리 ≤ 1.0 m 이고 예측 연속 노출 ≤ 1.2 s(< 2 s 수광 요건) 이면 돌진(속도 ≤ v_max·0.9). 적이 이미 조준 락 중일 수 있어 마진 1.2 s [가정].
- M2: `t > t_prestage − 20 s`(≈202 s) 이후에는 새 STRIKE 접근 금지(기존 STAGE 이행과 충돌하지 않게; 값은 시뮬로 결정).
- 경기 규칙상 M5 점수·종료 규칙과 M2 대비 가중은 **[가정]** — 확인 전에는 위 κ 상수를 점수 환산 없이 시간 기준으로만 둔다.

---

## 6. 설계 결과물

### 6.1 데이터 흐름도
```
                     ┌────────────── 서버: START/STOP/UAV(실기 미구현) ──────────────┐
카메라+YOLO ─Detection(team?)──┐                                                     │
LiDAR 스캔 ─lidar_blobs(동적 군집)─┼──▶ ThreatBoard ──ThreatTrack[] (level,p_enemy,σ,v̂)
UAV 관측(σ,지연,off) ──────────┘          │
                                          ▼
 정적 격자 L ───────────────────▶ ThreatField (K=4 시간 층: Body_k, Risk_k, visibility)
                                          │
        ┌─────────────────────────────────┼───────────────────────────────┐
        ▼                                 ▼                               ▼
 Navigator/A* (builtin)         AvoidController (L1~L3)           risk 후검증(연속 노출)
 cost_maps+Risk/Body             상태·기동·breadcrumb·예산              │
        │                                 │                              │
        └────────── path ─────────────────┴───────▶ CommandArbiter(owner) ◀─ GOAL κ,예산
                                                      │
                          ┌───────────────────────────┴───────────────┐
                          ▼                                           ▼
                 builtin follower(_follow)                Nav2Stack(carrot 경유지 → NavigateToPose)
                          └──────────────▶ SafetyLimiter + 깊이 가드 + 접촉 감지(항상 마지막) ──▶ DiffDrive ──▶ 로봇
```

### 6.2 주요 자료구조·인터페이스 [설계]
```python
# mission/threat_board.py
class Evidence(IntEnum): NONE=0; OBJECT=1; UGV=2; ENEMY=3   # FRIEND 는 플래그
@dataclass
class ThreatTrack:
    tid: int; x: float; y: float; vx: float; vy: float
    sigma: float                 # 위치 불확실도(m)
    evidence: Evidence; friend: bool
    p_enemy: float
    t_obs: Dict[str, float]      # {"cam":.., "uav":.., "lidar":..} 마지막 관측 시각
    src_ident: Optional[str]
    def predict(self, tau: float) -> Tuple[float, float, float]:  # (x, y, rho)
class ThreatBoard:
    def update(self, tracker: UgvTracker, blobs: List[Blob], uav_state: str, now: float) -> List[ThreatTrack]
    def level(self, q: Pose2D, v_us: float, now: float) -> ThreatLevel   # 히스테리시스 포함(상태 보유)

# mission/threat_field.py  (순수 numpy, ROS·로봇 무관)
@dataclass
class ThreatField:
    body: np.ndarray        # (K, ny, nx) bool   물리 점유(예측·σ 확대 포함)
    risk: np.ndarray        # (K, ny, nx) float  0..1 피격 위험
    dist: np.ndarray        # (K, ny, nx) float  가장 가까운 위협 중심까지 거리(근접 항용)
    dtau: float; version: int
def build_field(tracks, grid: OccupancyGrid, cfg: AvoidCfg, now) -> ThreatField
def path_exposure(path, v_plan, field) -> Tuple[float, float, List[Tuple[int,int]]]  # (총 위험적분, 최대 연속 노출 s, 구간)

# nav/grid.py
def astar_threat(grid, blocked, cost, start, goal, field, v_plan, w_risk, w_close, max_expand) -> Optional[List[Pt]]

# mission/avoid.py
class AvoidState(Enum): CLEAR; AWARE; EVADE; CLOSE
@dataclass
class AvoidCmd:
    owner: str; v: float; w: float; pan: float; turn_pref: Optional[float]
    goal: Optional[Pt]; path: Optional[List[Pt]]; emergency: bool; why: str
class AvoidController:
    def step(self, now, dt, ctx: AvoidCtx) -> Optional[AvoidCmd]     # None = MISSION 계속(위험 인지 경로만 사용)
    def reset(self) -> None                                           # HALT/SAFE/새 경기
    def breadcrumb(self, pose: Pose2D) -> None
    def pick_retreat(self, ctx) -> Optional[Pt]                       # §4.1 5조건 + 점수
    def budget(self, t, eta_goal) -> Tuple[float, float]              # (κ, hold_budget)

# nav/stack.py (기본 구현은 no-op)
NavStack.set_threat(field: Optional[ThreatField], w_risk: float, w_close: float) -> None
NavStack.follow_path(path: List[Pt], speed_scale: float) -> NavOut     # 내장 추종기로 이 경로를 따라감
NavStack.set_carrot(path, lead_m: float) -> None                       # Nav2: 경로 위 앞선 점을 목표로(재전송 0.15 m 규칙 재사용)

# nav/safety.py
SafetyLimiter.limit(scan, v, w, now, turn_pref: Optional[float] = None)   # 기본 None = 지금과 동일
```
의사코드(위협 인지 경로계획 + 소유권):
```python
def tick_avoid(now, dt):
    tracks = board.update(tracker, blobs, uavmon.state, now)
    lvl    = board.level(pose, v_cmd, now)
    field  = field_cache.get_or_build(tracks, grid, now)           # 0.25 m/0.5 s 규칙
    κ, hold = avoid.budget(t, eta_goal(field=None))
    stack.set_threat(field, w_risk*(0.35+0.65*κ), w_close)
    if lvl in (CLEAR, AWARE):    return None                        # MISSION 이 그대로 진행(경로만 위험 인지)
    cmd = avoid.step(now, dt, ctx)                                  # EVADE/CLOSE: 기동 선택
    if cmd.owner != self.owner: arbiter.transfer(cmd.owner)         # clear_goal, epoch++, 램프 정책
    return cmd                                                      # _apply() → 안전 제한 → 로봇
```

### 6.3 새 설정값 — 무엇을 어떻게 정하나
`config.py` 에 `AvoidCfg` 신설.
| 값 | 초깃값 | 정하는 방법 | 구분 |
|---|---|---|---|
| `r_aware / r_evade / r_close` | 3.0 / 1.6 / 0.8 m | 시뮬 스윕 → 실기 LiDAR 인지거리·정지거리(calib brake) | **시뮬 결정 + 실측 검증** |
| 해제 문턱(2.2/3.6/1.1 m, 유지시간) | 위 값의 +0.6 m, 1.0~2.0 s | 시뮬 진동 지표로 | 시뮬 결정 |
| `r_laser_eff` (= `laser.max_range`+0.3) | 2.8 m | **현장 수광 유효거리 실측** | **실측 필수** [가정 2.5 m] |
| `dwell_s` (2.0 s) | PDF | 규정 | 규정 |
| `T_exp_limit` | 1.2 s (0.6·dwell) | 시뮬(상대 조준 지연 모델 포함) + 실측 | 시뮬+실측 |
| `p_cons` | 0.7 | 오분류 비용 비교 시뮬 | 시뮬 |
| `r_cam / r_uav`(팀 라벨 신뢰도) | 0.80 / 0.90 | **현장 사진·UAV 영상으로 정확도 측정** | **실측 필수** |
| `σ_obs`(LiDAR/카메라/UAV) | 0.06/0.08/0.15 | 실기 정지 표적 반복 측정 | **실측 필수** |
| `v_enemy_max` | `robot.v_max` 0.45 | 상대 속도 관찰 | [가정] |
| `r_body` | 0.13 (시뮬) | 실물 치수 | 실측 |
| 시간 층 `K, Δτ` | 4, 0.75 s | 시뮬 | 시뮬 |
| `w_risk, w_close, w_dome` | 3.0, 1.5, 1.0 | 시뮬(피격↔시간 트레이드오프 곡선) | 시뮬 결정 |
| `face` 잔여계수 0.15 | | 돔 위치 실물 확인 후 | **실측**(돔이 앞쪽뿐이라는 것은 [가정]) |
| HOLD 최대 / 쿨다운 / 최소 유지 | 3 s / 3 s / 1.0 s | 시뮬 | 시뮬 |
| `κ` 상수(`T_margin=8, T_soft=25`) | | 시뮬(경기 길이 5분, 이동 시간 분포) + 점수 규칙 | 시뮬 + 규정 확인 |
| `commit_dash` 거리/노출 | 1.0 m / 1.2 s | 시뮬 + 실측(조준 지연) | 시뮬+실측 |
| `lidar_blobs` 문턱(크기·지속·연관 게이트) | 0.15~0.35 m, 3 스캔, 0.4 m | 시뮬 오탐/누락 곡선 → **실기 LiDAR로 재측정** | 시뮬 후 실측 |
| breadcrumb 간격/길이 | 0.25 m / 10 m | 시뮬 | 시뮬 |
| 후진 한도 | 0.5 m, 0.15 m/s | 실제 후진 정지거리(calib) | 실측 |

### 6.4 파일별 변경 계획
| 파일 | 변경 |
|---|---|
| `config.py` | `AvoidCfg` 추가(위 표), `Config.avoid`. `tunables.py` 에 항목 등록(상태 `SIM_TUNED`/`ASSUMED` 표시, 현장 실험 연결) |
| `perception/tracks.py` | `Track` 에 `sigma, p_enemy, t_obs{cam,uav,lidar}, evidence` 추가; `team_votes` 대신(또는 병행) `p_enemy` 로그오즈 갱신; `update_lidar_blobs()`; UAV `uav_state` 반영한 σ |
| **신규** `perception/lidar_blobs.py` | 동적 군집 검출(§2.4) |
| **신규** `mission/threat_board.py` | 증거 사다리·수준·히스테리시스 |
| **신규** `mission/threat_field.py` | `Body_k`, `Risk_k`, 시야, `path_exposure` |
| **신규** `mission/avoid.py` | `AvoidController`, breadcrumb, 후퇴 후보, HOLD/우회/CLOSE 기동, 예산 |
| `nav/grid.py` | `astar_threat`, `cost_maps(..., field)` |
| `nav/navigator.py` | `_plan` 에 위협 층 적용(정적에만 인플레이션 완화), `follow_path`, `_path_invalidated` 위험 검사, 복구 회전 `rng` → `turn_pref` |
| `nav/safety.py` | `turn_pref` 인자, 위협 쪽으로 회전 금지 |
| `nav/stack.py`, `nav/local_stack.py`, `nav/nav2_stack.py` | `set_threat/follow_path/set_carrot`, 소유권 이전 시 `clear_goal`·`_sent_goal` 초기화, Nav2 복구 감지 시 처리 |
| `mission/manager.py` | `_sense` 에서 LiDAR 군집·ThreatBoard 갱신; `_decide` 에 회피 게이트(순위 3·5, §5.1); `_arbitrate` 와 STRIKE 조준 중단/쿨다운; `_mode_goal` κ·commit_dash; `_enter_recover` 방향 제약; `_apply` 소유자 토큰·emergency 램프; `_enter_halt` 에서 `avoid.reset()`; 통계·로그 |
| `sim/` | `hostile.py`(적대적 상대 스크립트), `world.py`(검출 누락/팀 뒤집힘/UAV 지연·중단·오차를 설정으로), `run.py`(시나리오 빌더·프로브) |
| `tests/` | `test_threat_board.py`, `test_threat_field.py`, `test_avoid_controller.py`, `test_avoid_scenarios.py`(§7) |
| `field/` | 현장 실험 F11~F13 추가(팀 판정 정확도, 적 UGV LiDAR 신호, 돔 노출 기하) + `tunables.py` 항목 |
| `README.md`, 상태 전이표 HTML | 회피 상태·우선순위 반영 |

### 6.5 방식 비교
| | 기존(일반 장애물) | 단순 되돌아가기(breadcrumb 후진) | 제안 |
|---|---|---|---|
| 적 정보 사용 | 없음 | 적 접근을 트리거로만 | 위치·속도·σ·팀·시야 |
| 후퇴 방향 | LiDAR 여유/무작위 | 지나온 길 그대로 | **현재 시점에 5조건 검증한 후보**(옛 경로는 후보 중 하나) |
| 적 쪽으로 접근할 위험 | **있음**(트인 쪽 회전·무작위 회전) | 적이 뒤쪽이면 적에게 후진 | 후보 4·5조건으로 배제 |
| 후방 안전 | LiDAR 후방 검사(후진 시) | 같음 | + 예측 적 위치·새 장애물 재검사 |
| 시간 압박 | 없음 | 후퇴만 하다 시간 소진 가능 | κ·예산·commit_dash |
| 구현 비용 | 0 | 낮음 | **높음**(신규 모듈 5개) |
| 오탐 영향 | — | 유령 적 때문에 후퇴 반복 | p_cons·감쇠·진입/해제 비대칭으로 제한 |
| 한계 | 적에게 접근 | 막다른 길·좁은 통로에서 무의미 | **센서 신뢰도(팀 판정, UAV)에 성능 의존**, 튜닝 상수 많음 |

---

## 7. 구현 순서와 완료 기준

| 단계 | 내용 | 완료 기준(시험으로 확인) |
|---|---|---|
| **P0** | 시뮬 적대 상대·프로브·노이즈 노브(§8.1). **기준선 측정**(변경 전 코드) | 시나리오 12종 × 20시드 기준선 표 생성(충돌·피격·최소거리·M5·진동 지표). 코드 변경 전 값 저장 |
| **P1** | `ThreatBoard`+`Track` 확장(σ, p_enemy, 수준·히스테리시스), `lidar_blobs` | 단위 시험: 팀 라벨 뒤집힘 4 %/20 %에서 p_enemy 수렴, 수준 진입/해제 문턱 분리(채터 0), LiDAR 군집이 시뮬 적을 검출(재현율/오탐 표) |
| **P2** | `ThreatField`+`astar_threat`+`Navigator` 통합(내장 백엔드, 로봇 동작 변경은 플래그 OFF 기본) | `test_threat_field`: 합성 장면에서 Body/Risk 정답 일치, 박스 뒤 시야 차단, `path_exposure` 계산, A* 가 위험 우회 경로 선택·동일 입력 결정성. 플래그 OFF 시 기존 회귀 시험(현재 225개 통과·1개 스킵) **동일 통과** |
| **P3** | `SafetyLimiter.turn_pref`, RECOVER·Navigator 복구 방향 제약, `CommandArbiter` 소유권 | 시험: 위협이 우측일 때 좌측으로 회전, 후방 위협 시 후진 안 함, 소유권 이전 시 `clear_goal` 호출·`multi_output`=0 |
| **P4** | `AvoidController`(L1~L3 기동, breadcrumb, HOLD/우회/후퇴, 복귀) | §8 시나리오 S1~S6 통과(기준선 대비 지표 개선, 아래 합격선) |
| **P5** | M5 예산 κ·commit_dash, M2 조준 중단/쿨다운, STOP·SAFE 동시 처리 | S7~S12 통과, STOP 지연 ≤ 1 틱 |
| **P6** | Nav2 carrot 통합 + WSL 실제 Cartographer/Nav2 회귀 | WSL: 적 정면 조우 5회 이상에서 크래시·교착 없음, `fallbacks`/`recoveries` 기준선 이하, M5 도달 |
| **P7** | 현장: 팀 판정·LiDAR 신호·돔 기하 실험(F11~F13), 상수 갱신 | 실측 `r_cam`, `σ_obs`, 돔 위치 반영 후 시나리오 재측정 |

---

## 8. 검증 계획

### 8.1 시뮬 확장 (먼저 필요 — 지금 시뮬은 상대가 안 쏜다)
현재 상대는 같은 `Agent`(`engage_ugv=False`)라 **우리가 맞을 일이 거의 없고 회피 성능을 잴 수 없다.** 두 종류의 상대를 만든다.
1. **현실 상대**: `engage_ugv=True` 인 Agent(기존 HUNT 코드). 같은 인지 한계를 가지므로 현실에 가깝다.
2. **스크립트 상대(최악)**: `sim/hostile.py` — ① `Chaser`(우리 위치로 pursue, 완벽 정보, 사정거리 내 자동 조준·발사), ② `Patroller`(정해진 경로 횡단), ③ `Blocker`(정지·우리 진로 차단), ④ 선택 속도 0.3~0.45 m/s.
3. **노브**(현재 하드코딩): UGV 검출 누락률(5 %+4 %/m), 팀 뒤집힘(4 %), 오검출(0), UAV 지연(0.4 s)·주기·중단·위치 σ(0.04). 설정으로 노출해 스윕.
4. **프로브**: 매 스텝 정답 기반 `min_dist = |p_us−p_en|−2R`, `exposure_s`(적이 사정거리·시야·우리 헤딩 ±105° 조건에서 겨눌 수 있는 누적 시간), 충돌/피격/모드·회피 상태 전이 로그.

### 8.2 지표 정의
| 지표 | 정의 |
|---|---|
| 충돌 | `SimUGV.collisions` 를 **1 s 창으로 묶어 이벤트화**(틱 수가 아니라 사건 수; 장벽 시험의 "틱 수"와 구분해 기록) |
| 피격 | `hits_taken`(+ 4회째 주행 정지 발생 여부) |
| 최소 거리 | 경기·시나리오 구간 동안 `min_dist` |
| 노출 시간 | `exposure_s` 합 |
| 복귀율 | 회피 종료 후 **T_resume(15 s)** 안에 원래 모드(M2/M3/M5) 재개한 비율 |
| M5 도착률·시간 | 기존 `score()` 의 `m5_reached` |
| M2 성공 | `m2_ok` |
| 진동 | `AvoidState`·`Mode` 전이 중 **2 s 내 되돌아오는 쌍**의 수/분(chatter), HOLD↔이동 반복 수 |
| 정지 지연 | STOP 수신 틱 → 속도 0 출력 틱 |

### 8.3 시나리오 (각 20 시드 × {기준선, 제안}, 같은 시드·같은 배치로 **쌍 비교**)
| ID | 시나리오 | 상대 | 핵심 확인 |
|---|---|---|---|
| S1 | **정면 조우**(좁지 않은 곳, 마주 접근) | Chaser/Blocker | 정면 통과 안 함, 측면 우회 또는 HOLD, 피격·최소거리 |
| S2 | **측면 횡단**(경로를 가로지름) | Patroller | YIELD vs 통과 선택, 노출 시간 ≤ 한도 |
| S3 | **뒤따라오는 적** | Chaser(후방 시작) | 정지·회전 안 함, 등 보이며 전진, 박스 뒤 차폐 |
| S4 | **좁은 통로**(폭 0.5·0.8 m) | Blocker/Patroller | YIELD·교행 불가 판단, 교착·SAFE_STOP 횟수 |
| S5 | **적·아군 오분류** | 팀 뒤집힘 4 %/20 %/40 %, 아군을 적으로·적을 아군으로 | 적을 아군으로 믿은 경우의 피격, 아군을 적으로 믿은 경우의 시간 손실(비용 비교) |
| S6 | **YOLO 누락** | 누락 0/30/60/100 % | LiDAR 군집만으로 회피(100 %) 시 지표가 기준선 이하로 나빠지지 않는가, `degrade-safe` |
| S7 | **UAV 지연·중단** | 지연 0.4/1.0/2.0 s, 주기 1/2/5 s, 중단, **UAV 없음** | σ 확대·ghost 동작, UAV 없을 때 S1~S3 지표 유지 |
| S8 | **구조물 재배치**(60·150 s) | Chaser | 시야·정적 격자 갱신 후 경로 안정, 유령 위협 없음 |
| S9 | **M2 조준 중 조우** | Chaser(조준 시작 후 접근) | 조준 중단 시각, 재접근 쿨다운, 탄 낭비, M2 성공률 |
| S10 | **M5 진입 직전 조우**(남은 거리 0.5~1.5 m, t≈242 s) | Blocker(거점 앞) | commit_dash vs 우회, M5 도착률·시간 |
| S11 | **STOP과 회피 동시** | 임의 | STOP 수신 틱 내 구동 0, carrot/Nav2 취소, 이후 재시작 시 상태 초기화 |
| S12 | **3 vs 3 전체 경기**(상대 `engage_ugv=True`) | 현실 상대 | M2/M3/M5·피격·충돌 종합, 기준선 대비 악화 없음 |

### 8.4 합격 기준 (초안 — **기준선 측정(P0) 뒤에 확정**)
수치를 지금 정하지 않는다(측정 전 수치는 근거가 없다). 확정 규칙:
1. **안전 불변**: 충돌 이벤트 수와 `SafetyLimiter` 위반(정지거리 안 진입)이 기준선보다 **많아지지 않는다**(유의수준은 Wilson 구간·쌍 비교).
2. **피격**: S1~S4·S9 에서 평균 피격이 기준선보다 낮고, 4번째 피격(주행 정지) 발생률이 증가하지 않는다.
3. **임무**: M5 도착률은 기준선 − 5 %p 이내, 도착 시간 중앙값은 기준선 + 10 s 이내(예산 규칙의 목표). M2 성공률 저하 ≤ 10 %p.
4. **진동**: chatter 쌍 수/분 ≤ 기준선 이하이고, 같은 위협에 대한 EVADE 재진입 ≤ 4회/30 s.
5. **degrade-safe**: S6 100 % 누락·S7 UAV 없음에서 기준선 대비 **지표 악화 없음**.
6. 실제 Nav2(WSL) 실행: 크래시·교착 없음, `fallbacks`·`recoveries`가 기준선을 넘지 않음. (실시간이라 실행마다 달라지므로 **≥ 10회**, 이전에 한 번의 실행으로 판단하다 틀린 경험이 있음)

### 8.5 위험요인 · 한계 (솔직히)
- **돔이 앞쪽에만 있다**는 가정이 틀리면 §4.4(뒤에서 추격 시 계속 전진)의 근거가 약해진다. 현장 실험(F13)으로 먼저 확인.
- 실제 상대가 추격하지 않고 M2→M5만 하면 회피는 거의 발동하지 않고 비용(오탐 후퇴·시간 손실)만 생긴다 → **오탐 비용**을 S5·S6 로 반드시 측정.
- `laser.max_range` 2.5 m, 서보 조준 지연, 적 속도 등은 전부 **[가정]**. 위험 반경이 틀리면 과회피 또는 과소회피.
- A* 시간 의존 비용의 최적성은 보장하지 않는다. 대신 연속 노출 후검증과 시험으로 보완.
- UAV는 실기 구현 자체가 없어 **설계에서 필수 입력이 아니다**.
- LiDAR 군집 검출은 SLAM 지터·팬틸트 기둥 가림에 취약 → 오탐 곡선을 시뮬과 현장에서 각각 측정.
- Nav2 carrot 방식의 실제 거동(컨트롤러 흔들림)은 WSL/현장 실측 전 미확인.

### 8.6 현장 실험 추가 제안 (`field/`)
- **F11 팀 판정 정확도**: 실제 안테나 UGV(아군·적색/청색)를 거리·각도별로 촬영 → `team_from_antenna` 정확도, 미상 비율 → `r_cam`, HSV 임계.
- **F12 적 UGV의 LiDAR·깊이 신호**: 정지/주행 UGV를 거리별로 스캔 → 군집 크기·호 길이·중심 오차, 박스와의 구별 → `lidar_blobs` 문턱, `σ_obs`.
- **F13 돔 노출 기하**: 우리 UGV를 고정하고 방위별로 상대 레이저가 수광센서에 닿는 각 범위 확인(전방 ±105° 가정 검증), 유효거리·입사각 → `face`, `r_laser_eff`.

---

## 부록: 이 설계가 근거로 삼은 소스 위치
`mission/manager.py` — `_decide` 880–929, `_hunt_candidates` 950, `_arbitrate` 963, `_apply` 981, `_nav_to` 1008, `_enter_recover` 1031, `_mode_recover` 1052, `_friend_in_line` 1109, `_mode_goal` 1373 · `nav/safety.py` 24–61 · `nav/navigator.py` `step` 132–170(복구 142–159) · `nav/nav2_stack.py` `step`·`_enter_fallback` · `perception/tracks.py` 21–200(`enemies` 170–181) · `perception/detector.py` 51–66, 84–122 · `link/http.py` 40–42 · `sim/robot.py` `get_uav`, `sim/world.py` UGV 검출(678–705), `_beam_hit`·`_contact`(725–826) · `config.py` `RobotCfg/LaserCfg/NavCfg/MissionCfg` · `ros2/config/nav2_params.yaml` · `maicon26/field/tunables.py`.
