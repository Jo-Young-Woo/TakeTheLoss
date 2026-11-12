# 분석 결과 (시뮬 기록)

### static (정지)
- `sensors.odom.noise_v` = 0.003111
- `sensors.odom.noise_w` = 0.003095
- `sensors.odom.gyro_bias_z` = 0.003893
- `sensors.imu.gyro_bias_z` = -0.006056
- `sensors.imu.gyro_noise_z` = 0.001658
### speed_steps (직선 속도 스텝)
- `drive.v_max` = 0.5202
- `drive.v_max_reverse` = 0.5197
- `drive.a_accel` = 0.8844
- `drive.cmd_latency_s` = 0.11
- `drive.rise_time_s` = 0.3079
- `sensors.odom.latency_s` = 0
### turn_steps (제자리 각속도 스텝)
- `drive.w_max` = 2.452
- `drive.alpha_max` = 6.419
- `sensors.imu.latency_s` = 0.0156
- `sensors.imu.gyro_scale` = 1.016
### straight (직진)
- `sensors.odom.lin_scale` = 1.031
- `drive.lr_bias_rad_per_m` = 0.02502
- `drive.surfaces.default.slip` = 0  (오도메트리/참 = 1.031, 직진편차 +0.0250 rad/m, 끝 횡편차 +0.079 m, n=2)
### brake (급정지)
- `drive.a_brake` = 1.63
- `drive.stop_distance` = {'0.15': 0.023811203501622562, '0.30': 0.05627170060244799, '0.45': 0.10761539113675485}  (제동 시작 지연 0.099 s, 감속 fit 1.57 m/s²)
- `drive.surfaces.default`: a_brake 1.63 m/s² → μ ≥ 0.17
### rotate (제자리 회전)
- `sensors.odom.yaw_scale` = 1.024
- `drive.pivot.yaw_efficiency` = 0.9201
- `drive.pivot.drift_m_per_rad` = 0.02127
- `drive.pivot.overshoot_deg` = 18.25
### lidar_square (LiDAR 장착)
- `sensors.lidar.xyz[0:2]` = (+0.040, -0.010) m, `rpy[2]` = +0.0692 rad (두 벽 yaw 불일치 0.0013 rad, 직선 RMS 3.9 mm)
- `sensors.lidar.latency_s` = 0.006514
