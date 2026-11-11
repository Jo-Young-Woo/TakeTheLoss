# LiDAR 간섭 직접 감지: 미검증, 기본 끔

실기 데이터를 아직 받지 않았다. Gazebo는 LiDAR 광학 간섭과 카메라 적외선 차단 필터를 재현하지 않으므로, 시뮬레이션 결과로 채택 여부를 결정하지 않는다. `threat.lidar_crosstalk.enabled`는 `false`다. 빠른 위협 감지의 주 경로는 거리 스캔의 정적 배경 제거·군집화·칼만 추적이다.

## 준비된 측정 도구

로봇과 배경을 고정하고 상대 LiDAR를 끈 기준 데이터부터 기록한다. 실제 토픽은 현장에서 확인한다. 아래 명령은 저장만 하며 주행·발사를 명령하지 않는다.

```sh
python tools/lidar_crosstalk_probe.py record --topic /scan --seconds 60 --output off.jsonl
python tools/lidar_crosstalk_probe.py record --topic /scan --seconds 60 --opponent-on --bearing-deg 0 --distance-m 1 --output on.jsonl
python tools/lidar_crosstalk_probe.py analyse on.jsonl --baseline off.jsonl --output lidar-study.json
python tools/camera_ir_probe.py record --topic /image_raw --seconds 60 --distance-m 1 --opponent-on --output camera-on.jsonl
python tools/camera_ir_probe.py analyse camera-on.jsonl --output camera-study.json
```

거리 도구는 기준 스캔과 다른 1~2스캔짜리 점을 추출하고 intensity가 있으면 보존한다. 스캔 행에 독립적인 동적 추적기의 `tracked_ranges`(angle_min/angle_max, range_min/range_max)를 추가하면 실제 이동 물체를 제외한다. 상대의 실측 방위와 거리도 함께 저장한다. 카메라 도구는 프레임 최대 밝기와 99.9백분위, 이전 프레임 대비 밝은 순간 변화 픽셀을 기록한다. 고정된 흰 물체는 섬광으로 세지 않는다.

ON/OFF를 섞은 독립 표지 구간 파일을 `--labels labels.json`으로 주면 감지율·오탐/분·6.1 추적기보다 빠른 시간을 계산한다. 모든 시각은 해당 기록 시작 이후 초로 맞춘다. 예:

```json
[
  {"start_s": 0, "end_s": 10, "present": true, "lidar_detected_s": 0.4},
  {"start_s": 10, "end_s": 70, "present": false}
]
```

여러 거리(0.5/1/2m), 방위, 정적 조명과 움직이는 배경에서 반복한다. 양성 구간 80% 이상 검출, 음성 구간 1회/분 이하, 같은 장면에서 6.1보다 먼저 검출이라는 조건을 모두 만족해야 후보가 된다. 표지·음성 대조·6.1 시각 중 하나라도 빠지면 통과 판정을 내리지 않는다. 순간 점이나 섬광만으로 상대의 조준 또는 적외선 응답을 확정할 수 없다. 분석 도구는 설정을 자동으로 켜지 않는다.

## 빠른 스캔 경로의 현재 검증 범위

`tests/test_fast_threat.py`는 합성 스캔으로 이미 관측한 자유 공간의 정지/이동 UGV 후보, 관측된 정적 박스 제거, 칼만 속도 추정, 오래된 UAV 식별 감쇠, 위험 비용 경로, 비동기 갱신을 확인한다. 아직 관측하지 못한 구조물은 자유 공간으로 추정해서 UGV로 확정하지 않는다. LiDAR만으로 소속은 알아내지 못한다.

현재 Windows CPU에서 640점·5개 후보·100회 측정 결과, 스캔 처리부터 위험 층 발행까지 p50 24.81ms, p95 26.69ms, 최대 37.42ms였다. 재현: `python scripts/benchmark_fast_threat.py`. 파일: `recordings/deadlock_fast_threat_20261109/latency.json`. 이는 합성 입력의 계산 지연이며, 실제 Jetson 또는 가시선 확보부터 센서 수신까지의 지연 검증은 아니다.

현장 미확정: `risk.range_m`, `risk.w`, 비용표 수치, 실제 LiDAR 토픽·스캔 주기, 카메라 IR 필터, 깊이 카메라 제공 여부. 서빈님 비용표 원본이 없어 기본 항목 exposure/hit_history/bottleneck/closing/ally_fire_lane의 가정값을 유지했다.
