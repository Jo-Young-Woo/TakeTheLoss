"""로봇 보정 도구: 측정(experiments) → 분석(analysis) → 프로파일(profile) → 생성물(generate: URDF/센서/Nav2/Config).

흐름                          스크립트
  측정 + 기록(CSV, rosbag)    scripts/calib_run.py
  분석 → 프로파일 갱신        scripts/calib_fit.py
  프로파일 → 산출물           scripts/calib_generate.py
  미측정(가정) 값 목록        scripts/calib_report.py
"""
