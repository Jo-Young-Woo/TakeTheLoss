# 사진 겹침 검증 B — 실제 Gazebo / TF

정본 `wingbot_mods.yaml`, `cam_fit.json`, `photo_keypoints.yaml`, `photo_mask.npy`의 숫자를 변경하지 않았다. RED와 BLUE 모두 Gazebo Fortress CPU Ogre2에서 별도로 실제 촬영했다. 원본 사진은 대회 제공 자료이며 외부 공개하지 않는다.

## 결과

두 팀 모두 **PASS**. 1057×1054, 정본 자세·HFOV, 검은 배경, 그림자 없음, 로봇 원점·관절 0. 실제 카메라 3프레임을 받은 뒤 마지막 프레임을 저장했다. 차체·바퀴 행≥640의 실루엣 IoU는 **0.947498122**로 기준0.93을 통과했다. 실제 TF12개와 실제RGB 관측12개 모두 각 허용치 안이다. 녹색 debug 픽셀은0이며 픽셀을 삭제하거나 마스크로 숨기지 않았다.

| 기준점 | 실제 TF 투영 오차(px) | 실제 RGB 관측 오차(px) | 허용(px) | 이미지 관측 방법 |
|---|---:|---:|---:|---|
| laser_lens_center | 2.753 | 5.065 | 12 | `actual_blue_glass_component_centroid` |
| camera_lens_center | 0.681 | 0.467 | 12 | `actual_blue_glass_component_centroid` |
| camera_led | 8.605 | 9.222 | 12 | `actual_pale_surface_component_centroid` |
| headlight_left(+y) | 3.907 | 2.962 | 12 | `actual_pale_surface_component_centroid` |
| headlight_right(-y) | 4.098 | 5.060 | 12 | `actual_pale_surface_component_centroid` |
| receiver_left(+y) | 16.921 | 18.609 | 20 | `actual_pale_surface_component_centroid` |
| receiver_right(-y) | 11.164 | 11.065 | 20 | `actual_pale_surface_component_centroid` |
| iff_left_tip | 5.809 | 5.000 | 12 | `actual_rod_top_silhouette` |
| iff_left_sleeve_top | 4.058 | 9.000 | 12 | `actual_team_sleeve_boundary` |
| iff_left_sleeve_bottom | 5.487 | 7.810 | 12 | `actual_team_sleeve_boundary` |
| depth_camera_front_ctr | 5.858 | 6.031 | 12 | `actual_front_rectangle_diagonal_intersection` |
| lidar_top_center | 6.511 | 7.363 | 12 | `actual_flat_cap_region_ellipse` |

## 관측 경계

- TF 경로는 실제 Gazebo JointState→robot_state_publisher와 실제 model dynamic pose다. `frames.yaml`의 실제 프레임/로컬 오프셋을 변환했다. 검증에 `photo_keypoints.model_xyz`나 `model_px`를 대입하지 않는다.
- RGB 검출은 사진 주석의 넓은 관심영역과 실제 픽셀만 받는다. TF·3D좌표·예상 투영점은 검출 함수에 전달하지 않는다. 렌즈는 파란 유리 성분, LED/전조등/돔은 흰 표면, 안테나는 색 슬리브의 실제 끝과 막대 실루엣으로 측정한다.
- 원본 깊이 카메라 STL은 앞면에 중심 렌즈 표식이 없으므로 관측 가능한 앞 사각형의 네 영상 모서리 대각선 교점을 쓴다. 이는 렌즈 패턴 인식이라는 뜻이 아니다. LiDAR는 실제 상판 색 영역에서 세로 측면을 분리한 뒤 타원 중심을 측정한다.
- 미검출은 `pixel=null`·`image_error_px=null`로 실패한다. TF가 없어도 RGB 측정은 동일하게 유지되지만 전체 게이트는 실패한다. 유효한 좌표만으로 사진 관측을 대체하지 않는다.
- 흰 표면 임계값135와 평면 색 허용차2, LiDAR 수평 opening31px는 고정 조명·해상도에 대한 검증기 설정이다. 로봇 인식/임무 알고리즘에는 연결하지 않았으며 실물 카메라용 범용 검출기를 주장하지 않는다.
- 실루엣은 RGB 최대채널>3을 그대로 사용하며 사진 마스크에 맞춘 이동·크기 변경·구멍 메우기를 하지 않았다. 아래쪽만 평가하므로 근거리 안테나의 사진 자세 차이 등 모든 상부 외관의 일치를 보장하지 않는다.

## 재현 및 산출물

```bash
ros2 launch wingbot_description photo_view.launch.py team:=RED output_dir:=/absolute/output
ros2 launch wingbot_description photo_view.launch.py team:=BLUE output_dir:=/absolute/output
```

두 launch는 센서 없는 실제 xacro를 빈 월드에 두고 검증 카메라1개만 추가한다. 프로세스에만 Fast DDS64MiB 프로필을 적용하며 실제 캡처·평가가 끝나면 종료한다. 자동화는 launch의 종료코드만 보지 말고 `photo_validation.json`의 `passed`를 확인한다.

- `model_vs_pdf.png`: 왼쪽 원본 사진, 오른쪽 실제 Gazebo RGB. 제공된 이상적 모델 렌더를 촬영 영상 대신 사용하지 않았다.
- `outline_on_photo.png`: 실제 RGB 외곽선·부품 관측(십자), 실제 TF 투영(원)을 원본에 겹친다. 색은 레이저 하늘색·카메라 주황·수광 초록·안테나 분홍·전조등 노랑이다.
- `photo_render_RED.png`, `model_blue.png`: 원본 크기의 실제 RED/BLUE 카메라 프레임.
- `photo_validation.json`, `photo_validation_BLUE.json`: 기준점별 관측 방법·오차·실제 TF/관절/model pose·입력 해시.
- `ortho.png`, `ortho_provenance.json`: 실제 확장 URDF 시각 메시와 관절값의 **CPU 정사영 진단 그림**이다. Gazebo RGB 촬영이나 B 합격 근거로 사용하지 않는다. `photo_geometry_views.write_ortho`로 생성한다.

## 실패 이력과 회귀

첫 실제 촬영은 실루엣·TF를 통과했지만 인접 회색면의 흰색 성분 병합, 깊이 카메라 중심 미검출, LiDAR 가장자리 오검출로 B가 실패했다. YAML을 바꾸지 않고 실제 영상 특징의 분리를 수정했다. BLUE에서는 유리 렌즈와 안테나가 같은 청색이라 지배적인 세로 슬리브 성분과 같은 열에 있는 조각만 묶는다.

녹색 보조선416픽셀은 scene grid/origin을 꺼도 남았다. 설치된 SDF 스키마에서 light의 visualize 기본값이 true임을 확인하고 false로 끈 실제 재촬영에서0픽셀을 확인했다. GUI visibility bit도 제외하며 보조선이 다시 검출되면 게이트가 실패한다.

`tests/test_wingbot_photo.py` **14개 통과**: 빈 영상 실패, TF 없는 실제RGB 관측 유지/게이트 실패, 실제색영역·평면 모서리/타원 픽셀 이동 추종, 예상점 스냅 금지, BLUE 렌즈/슬리브 분리, 정본 카메라와 mask, 그리드·광원보조선 비활성화를 검사한다. 이 시험은 실제 사진분류 정확도나 자율 임무의 검증이 아니다.
