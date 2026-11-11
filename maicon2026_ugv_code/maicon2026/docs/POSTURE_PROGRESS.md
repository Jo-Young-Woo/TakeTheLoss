# 진척 감시와 자세 제어

새 정책은 `config.progress.enabled=true`인 비교 시험에서 켠다. 기존 시험과 경기 성능에 대한 검증이 끝나기 전에는 실기 기본값을 바꾸지 않는다. `engage_ugv=false`, 시간 소모 회피의 `avoid.evade_min_hits=2`, M3 전체 재제출과 M6는 유지한다. `ENGAGE_OK`는 위험을 감수하며 임무 방향으로 이동할 수 있다는 뜻이며 자동 사냥 허용이 아니다.

## 구조와 전이

`maicon26/nav/progress.py::ProgressMonitor`는 임무 목표의 남은 경로가 누적 0.15 m 줄거나 새 면 관측·사격 진척·도착이 생겼을 때만 정체 시간을 초기화한다. 명령 재전송, 바퀴 회전, 경로 재계획, 일반 목표 변경만으로는 초기화하지 않는다. 다른 목표로 바뀌면 남은 거리 비교 기준만 갱신하며, 기존 정체 시간·단계는 유지한다. 일시적으로 후보가 없어져도 진행 중인 정체는 사라지지 않는다. 최초 작업 또는 명시적 포기 후 새 시도에서만 별도 에피소드를 시작한다. 목표 키는 회피 경유지가 아닌 실제 임무 목표로 유지해야 한다.

| 무진척 시간 | 한 번만 내보내는 조치 |
|---|---|
| 3 s | 재계획 |
| 6 s | 교전 감수 자세 요청 |
| 9 s | 앞이 물리적으로 막혔으면 후진·우측 통행 회복 |
| 13 s | 목표 실패, 30 s 재선택 금지 |

같은 목표를 두 번 실패하면 해당 경기의 후보에서 제외한다. 정상 대기와 발광 중 정지는 정체 시계를 일시 정지하며, 이미 쌓인 정체를 지우지는 않는다. 목표가 없는 상태의 정체도 통합 계층에서 감시해야 한다. 13 s 포기 규칙만으로는 “12 s 초과 무진척 0건”을 보장하지 않는다. 재계획·회복이 그 전에 유효 진척을 만들어야 하며, 포기한 사건도 평가에서 제외하지 않는다.

`maicon26/mission/posture.py::PostureController`는 임무 FSM과 별도의 두 축을 관리한다.

| 축 | 전이 기준 |
|---|---|
| 위협 L0→L1 | 가시선 위협, E≥1 또는 피격 1회 |
| 위협 →L2 | E≥2 또는 피격 2회 |
| 위협 →L3 | E≥3이면서 피격 2회 이상 |
| 위협 하강 | 낮아질 조건이 3 s 지속될 때 한 단계씩 |
| AVOID→ENGAGE_OK | CM, 정체 6 s, 피격 무적, GOAL 또는 명시적으로 켠 시간 압박 |
| ENGAGE_OK 유지 | 진입 이유가 남아 있거나 4 s/순이동 0.6 m 커밋 조건이 아직 미충족 |
| ENGAGE_OK→AVOID | 피격 3회 이상이면 즉시. 무적 또는 GOAL 마지막 1 m는 예외 |
| →COVER | L3이고 시간 소모 회피를 허용하는 피격 수 이상일 때만 |

자세는 원칙적으로 최소 3 s 유지한다. 피격 3회 안전 전환은 이 유지 시간과 진동 고정보다 우선한다. 위험 증가에 따른 단계 상승은 즉시 적용한다. 감소는 3 s 이상 조건이 유지되어야 하며, 10 s 내 네 번째 일반 결정 변경이 된다면 4 s 더 고정한다. 위협 상승은 고정 중에도 적용하므로 안전 우선 전이 자체가 엄격한 진동 지표를 넘을 수 있다. 진단은 그런 실제 전이를 그대로 기록하며 생략하지 않는다. 왕복 이동은 커밋 거리를 채우지 못한다.

## 설정

| 그룹 | 주요 기본값 |
|---|---|
| risk | w={L0:0.5,L1:1.5,L2:4,L3:10,engage:0.3}, range_m=3, update_hz=5 |
| risk | team_weight={enemy:1,unknown:0.7,ally:0.3}, max_exposed_dwell_s=1.5, dwell_cap_from_hits=2 |
| risk.cost_table | exposure=1, hit_history=2, bottleneck=1.5, closing=0.5, ally_fire_lane=1 |
| threat | level_up=[1,2,3], level_down_s=3, lidar_min_width_m=0.12, lidar_max_width_m=0.40 |
| posture | commit_s=4, commit_dist_m=0.6, min_dwell_s=3, engage_exit_hits=3 |
| progress | stall_*_s=3/6/9/13, min_gain_m=0.15, retry_after_s=30, max_goal_failures=2, jitter=0.2 |
| engage | min_clearance_m=0.30, fire_on_blocker=false, m2_ammo_reserve=3, enemy_confidence=0.8 |
| threat.lidar_crosstalk | enabled=false, 실기 검증 전 채택 금지 |

서빈님 비용표가 제공되지 않아 `risk.cost_table`은 가정값이다. 소속 미상과 아군은 차단 상대 사격 대상이 될 수 없다. 사격 허용 함수는 수광부 가시성, 사선 안전, 잠금, M2 잔탄, 거리 0.6~1.5 m, 대상별 10 s 재발사 간격을 모두 검사한다. 실제 발사는 기존 조준·레이저 경로를 통과해야 한다.

## Gazebo의 여섯 Agent

`arena.launch.py`의 `full_sensors:=true ugvs:=6`은 동일한 WINGBOT 모델 여섯 대에 각각 실제 RGB 카메라·LiDAR·IMU를 둔다. 기본 `full_sensors:=false`는 기존 단일 센서 로봇과 경량 상대 구성을 유지한다. YAML 기구 사양과 프레임 ID는 변경하지 않는다.

| 용도 | B1 예 |
|---|---|
| 원시 센서와 구동 | /arena/robots/B1/image, scan_raw, odom, joint_states, camera_info, cmd_vel |
| Agent 공식 API | /robots/B1/image_raw, scan, odom, imu/data, cmd_vel, mil/* |
| 독립 TF | /robots/B1/tf, /robots/B1/tf_static |
| 공통 | /clock, /uav/image_raw, 심판 제어·상태 |

`scripts/run_wingbot_selfplay.py`는 여섯 독립 프로세스의 센서 준비를 확인한 뒤 심판을 시작한다. 이 조정기는 로봇 속도·목표·사격 명령을 내리지 않는다. 모델의 실제 위치는 평가 로그와 영상의 식별 테두리에만 사용한다. 영상의 위험 지도와 UGV 추적 화살표는 B2 Agent의 추정값이다. `full_sensors`에서는 로봇별 bridge 프로세스를 분리한다. 단일 bridge에서 마지막 두 LiDAR 연결이 활성화되지 않던 현상을 실제 준비 시험에서 확인했기 때문이다.

녹화는 실제 Gazebo 카메라 프레임을 사용하고 누락·유지 프레임 수를 별도 기록한다. CPU Ogre2 렌더링 환경에서는 시뮬레이션 시간과 실제 소요 시간이 다르다. 경량 상대를 쓰던 기존 좌표 제어 교전 영상은 이 자가 대전의 성능 증거로 대체하지 않는다.

`--scenario`는 `selfplay`, `vs_evaders`, `vs_hunters`, `vs_blockers`, `mixed_allies`, `goal_race`를 선택한다. 혼합 시험의 더미도 개별 센서와 공식 API를 사용하며 중앙에서 실제 좌표로 조종하지 않는다. `--no-uav`는 UAV 카메라 생성과 Agent 영상 전달을 함께 끈다. 운영 기본값과 `--variant v2`는 새 진척 정책과 IMU 방위 융합을 끈 상태다.

후보 시험은 `hal.imu_heading_fusion=true`로 IMU의 **상대 방위 변화**와 바퀴 이동량을 결합한다. 전역 IMU 방위를 위치의 정답으로 주입하지 않는다. TF로 수평 고정 장착을 확인하고, IMU와 odom의 시각 차이를 최대 0.15초로 제한한다. 검증 실패·오래된 측정은 바퀴 odom으로 연속 복귀한다. `imu_odometry.jsonl`과 `imu_odometry_summary.json`은 원시·융합 odom 및 실제 적용·거절 횟수를 저장한다.

사후 분석의 `metric_version=4`는 WAIT/HALT/REPORT/DONE 초기화를 주행 진동으로 세지 않는다. 일반 EXPLORE 관찰 대기 중의 판단 변경은 계속 센다. 혼합 시나리오의 합격 기준은 `policy=agent` 로봇에 적용하고, 다른 더미의 진단도 별도로 보존한다. 독립 진척 검증은 활동 중 관찰 면·발사 누적 카운터와 시간 정렬된 위치 로그가 모두 있을 때만 가능하다. 단일 경기로 면별 100% 커버리지나 v2 대비 회귀 기준을 통과 처리하지 않는다.

## 실제 처리 시간과 조준 증거

후속 실행의 `Scan.receipt_wall_s`는 ROS scan 처리 함수에 들어온 순간의 단조 시계이고 `receipt_seq`는 해당 전송 인스턴스의 수신 순번이다. 센서의 ROS stamp와 섞거나 제어 판단에 사용하지 않는다. 자기 반사 마스킹과 스모그 필터를 통과해도 두 값이 보존된다. 위험 지도 작업은 **자신이 실제로 사용한 scan**의 순번·수신 시각·ROS stamp를 함께 보관한다.

`scan_lineage.json`은 수신→제출→계산 시작→지도 발행 시각을 연결한다. 새 scan의 긴급 갱신, 동일 scan의 주기 재계산, 대기 중 새 요청으로 교체된 작업, 더 최신 scan이 제출된 뒤 완료된 작업을 구분한다. 수신 순번의 건너뜀은 위험 지도에 제출되지 않은 scan이며 DDS 손실을 뜻하지 않는다. 종료 때 폐기된 대기 작업도 별도 카운터다. 최대 10,000개 발행 기록과 총 발행 수를 저장해 전체 경기 보관 여부를 확인한다.

`risk_worker_summary.json`은 기존 요청→발행·계산 시간과 `scan_callback_to_publish`를 별도 항목으로 제공한다. 특히 `urgent_fresh_scan`과 `timer_same_scan`을 합쳐 새 측정의 반응 지연이라고 부르지 않는다. callback 이전 전송 대기와 발행 후 Agent의 소비·물리 반응은 측정 범위 밖이다. 계측이 없었던 이전 경기 값으로 이 구간의 지연을 추정하지 않는다.

조준 진단은 기존 조준·안전 함수의 실제 호출 결과만 기록한다. 같은 검출 재사용 횟수, 촬영 시각·자세·팬틸트, 현재 상태, 선택한 센서 픽셀·거리, 정렬 오차·유지 시간과 실제 호출된 안전 조건을 남긴다. 단락 평가 때문에 호출되지 않은 조건은 미확인으로 표기한다. `aim_evidence`에는 소비한 검출과 촬영 stamp가 정확히 일치하는 원본 영상만 로봇별 최대 24장, 1초 간격으로 비동기 저장한다. 대응 영상이 없으면 최신 영상으로 대체하지 않고 누락을 기록한다. `m3_arrival_events`는 관찰 지점 도착·수정 판단의 제한된 이력을 보관한다.
