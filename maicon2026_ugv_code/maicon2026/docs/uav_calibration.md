# UAV 실측 격자 보정

기본 보정은 영상에서 실제로 확인한 여러 격자 교점을 사용한다. 경기장 크기는
**6.12×5.00m**, 보정 파일은 `ros2/gazebo/uav_grid_calibration.json`이다.
영상 위에 그린 격자는 결과 확인용이며 보정 입력이나 정확도 증거가 아니다.

현재 예제의 입력은 실제 Gazebo 고정 카메라 프레임
`ros2/gazebo/assets/m3_random4_overhead.png`(1280×720)이다.
다른 카메라나 실기 영상에 이 대응점·보정값을 그대로 적용하지 않는다.

## 측정점의 출처와 분리

`ros2/gazebo/uav_grid_seeds.json`에는 바닥의 실제 격자 교점을 사람이 식별한
경기장 좌표와 초기 픽셀 위치가 있다. `scripts/measure_uav_grid_fixture.py`가
각 위치 주변의 영상 밝기 능선을 찾아 픽셀 좌표를 정밀화한다. 가림이나 대비가
부족한 점은 거부한다. 이 과정은 기존 네 모서리 보정이나 새 호모그래피에서
픽셀 좌표를 역산하지 않는다.

- `uav_grid_measurements.json`: 실제 영상에서 측정한 교점 36개.
- `uav_grid_fit_points.json`: 그중 보정에 사용하는 27개.
- `uav_grid_checkpoints.json`: 적합 전에 남겨 둔 교점 9개와 별도로 읽은 흰색
  구역 경계 교점 4개, 총 13개. 적합 입력으로 재사용하지 않는다.

측정·검증 목록의 기본 형식은 다음과 같다. `plane_z`는 검증점의 높이이며,
적합점은 모두 `--reference-height` 평면 위에 있어야 한다.

```json
{"pixel": [552.88235, 127.27273], "arena": [2.2, 4.8], "plane_z": 0.001}
```

실기에서는 줄자 등으로 좌표를 확인한 격자 교점·표지를 경기장 전체에 배치하고,
왜곡을 제거한 영상에서 픽셀을 측정한다. 예측한 격자 좌표를 클릭값 대신 넣지
않는다. 적합점과 검증점은 측정 단계부터 분리한다. 최소 8개의 유효 적합점이
필요하며, 점들이 각 축의 60% 이상과 경기장 면적의 25% 이상을 덮어야 한다.

현재 영상의 밝기 기반 측정을 새 파일로 재현하는 명령:

```bash
python3 scripts/measure_uav_grid_fixture.py \
  --image ros2/gazebo/assets/m3_random4_overhead.png \
  --seeds ros2/gazebo/uav_grid_seeds.json \
  --output tmp/uav_grid_measurements_recheck.json
```

## 보정 생성과 검사 이미지

저장된 측정 목록으로 현재 보정·검사 이미지를 다시 만드는 명령이다.
`--overwrite`는 아래 출력 파일을 의도적으로 갱신한다.

```bash
python3 scripts/calibrate_uav_grid.py \
  --image ros2/gazebo/assets/m3_random4_overhead.png \
  --points ros2/gazebo/uav_grid_fit_points.json \
  --checkpoints ros2/gazebo/uav_grid_checkpoints.json \
  --arena-size 6.12 5.00 --camera-center 3.06 2.50 2.465 \
  --reference-height .001 --object-height .4 \
  --fit-tolerance .015 --checkpoint-tolerance .02 --grid-step .5 \
  --output ros2/gazebo/uav_grid_calibration.json \
  --overlay ros2/gazebo/recordings/uav_grid_calibration_overlay.png \
  --birdseye ros2/gazebo/recordings/uav_grid_calibration_floor.png \
  --overwrite
```

검사 영상은 원본 픽셀 크기를 유지하며 아래에 범례를 붙인다. 녹색 `F01…`은
적합에 채택한 측정점, 빨간색은 제외한 점, 자홍색 `C01…`은 별도 검증점이다.
원은 측정 픽셀, 흰 십자는 변환으로 예측한 픽셀이며 화살표는 잔차를 나타낸다.
점 옆에는 짧은 ID만 표시한다. 각 점의 수치 오차는 JSON의 `fit_errors_m`와
`checkpoint_errors_m`에 같은 순서로 들어 있다.

0.5m 간격의 청록색 선은 **적합된 변환의 역투영**이다. 격자가 그럴듯하게
겹친다는 사실만으로 정확도를 판정하지 않는다. 정사영 이미지는 실제 영상을
기준 바닥 평면으로 옮긴 것이며, 박스의 높이에 따른 어긋남은 남아 있다.

## 현재 예제의 검증 결과

| 구분 | 결과 |
|---|---|
| 적합 입력 / 채택 | 27개 / 26개 |
| 적합점 오차 기준 | 15mm |
| 제외점 | F11, 잔차 약 44.25mm |
| 채택한 적합점 RMS | 약 1.10mm — 학습 오차이며 독립 검증 아님 |
| 별도 검증점 | 13개 |
| 별도 검증점 RMS / 최대 | 약 5.34mm / 10.69mm |
| 별도 검증 허용 오차 | 20mm |

이 수치는 **저장된 Gazebo 영상의 바닥 측정점 결과**다. 실제 UAV의 위치 정확도,
실기 렌즈 보정 정확도, 박스 윗면 검출 정확도를 보장하지 않는다. 적합점 중
실측 오차가 큰 한 점도 원본 목록에 남겨 두며, 이를 삭제해서 결과를 좋게
보이지 않는다. 별도 검증이 허용 오차를 넘으면 검사 이미지만 저장하고 새
보정값 저장을 거부한다. 로더도 저장된 검증 결과를 그대로 믿지 않고 재계산한다.

## 높이와 렌즈 조건

현재 예제에서 보이는 바닥 표시 평면은 **Z=0.001m**, 박스 윗면은 **Z=0.4m**다.
같은 픽셀이라도 어느 높이의 점인지에 따라 경기장 좌표가 달라진다. 윗면 좌표를
보정하려면 같은 경기장 좌표계에서 측정한 카메라 **광학 중심 XYZ**가 필요하다.
현재 Gazebo 설정은 **(3.06, 2.50, 2.465)m**이며, 드론 몸체 높이 2.50m와 다르다.
실기에서는 실제 표지 평면·상자 높이·광학 중심을 측정해 입력한다.

입력은 **이미 렌즈 왜곡이 제거된 핀홀 영상**이어야 한다. 이 예제의 Gazebo
고정 카메라는 렌즈 왜곡이 없다. 여러 평면 대응점으로 맞춘 호모그래피가 렌즈
왜곡 보정을 대신하지는 않는다. 실기에서는 먼저 내부 파라미터와 왜곡 계수를
측정하고, 보정용 영상과 실행 중 영상에 동일한 왜곡 제거를 적용한다.

카메라 이동·회전, 줌 변경, 해상도·자르기 변경 후에는 다시 측정하고 독립
검증한다. 실기 검증점은 바닥뿐 아니라 0.4m 높이에도 별도로 마련한다.

## 시뮬레이션에 적용

```bash
UAV_LAYOUT=image \
UAV_CALIBRATION="$PWD/ros2/gazebo/uav_grid_calibration.json" \
VIEW=thirdperson PHYSICAL_ROBOTS=one \
  bash scripts/run_gazebo_fpv_trial.sh
```

고정 경기 실행의 기본 보정도 이 격자 파일이다. 다른 카메라를 쓰면 다시 측정한 뒤
`--uav-calibration <JSON 경로>`로 지정한다. 보정 결과는
경기 보고서의 UAV 영상 진단에 기록된다. UAV 입력을 끈 실험에는 필요하지 않다.

UAV는 박스 위치·크기 후보를 제공한다. 위에서 **M3 사진의 존재나 종류를 판별하지
않으며**, 0.4m 정육면체 후보의 옆면은 UGV가 확인한다. 현재 색상 기반 윗면 검출은
Gazebo 외관에 맞춘 것이므로 좌표 보정만으로 실기 객체 검출까지 검증된 것은 아니다.

기존 `scripts/calibrate_uav.py`와 `uav_four_corner_calibration.json`은 네 모서리
방식의 호환 예제로 유지한다. 모서리 순서는 `(0,0), (W,0), (W,H), (0,H)`이며,
영상에서 왼쪽부터 정렬하는 방식이 아니다. 기본 작업은 위의 다점 격자 방식이다.

