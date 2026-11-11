#!/usr/bin/env bash
set -eo pipefail

repo='/mnt/c/Users/jojo0/OneDrive/바탕 화면/MAICON2026/maicon2026_ugv_code/maicon2026'
out="$repo/results/lidar_height_trial_20261111"
source /opt/ros/humble/setup.bash
source /home/jojo0429/wingbot_model_ws_20261109/install/setup.bash
set -u
cd "$repo"
export ROS_DOMAIN_ID=166
export IGN_PARTITION=maicon_lidar_height_20261111
export LIBGL_ALWAYS_SOFTWARE=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jojo0429/wingbot_model_ws_20261109/install/maicon_arena_gz/share/maicon_arena_gz/config/fastdds_shm256.xml
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1

arena_pid=''
shutdown_arena() {
  if [[ -n "$arena_pid" ]] && kill -0 "$arena_pid" 2>/dev/null; then
    kill -INT -- "-$arena_pid" 2>/dev/null || kill -INT "$arena_pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$arena_pid" 2>/dev/null || break
      sleep .5
    done
    if kill -0 "$arena_pid" 2>/dev/null; then
      kill -TERM -- "-$arena_pid" 2>/dev/null || true
    fi
    wait "$arena_pid" 2>/dev/null || true
  fi
}
trap shutdown_arena EXIT

setsid ros2 launch maicon_arena_gz arena.launch.py \
  robot_model:=wingbot profile:=wingbot_recording ugvs:=1 ego_slot:=B-2 \
  gui:=false render_engine:=ogre2 validation_camera:=true uav_cam:=true \
  image_transport_profile:=shm256 > "$out/launch.log" 2>&1 &
arena_pid=$!
printf '%s\n' "$arena_pid" > "$out/arena.pid"
sleep 3
kill -0 "$arena_pid"

python3 scripts/run_wingbot_autonomous_trial.py \
  --slot B-2 --variant progress --seed 0 \
  --simulate-impact \
  --ready-timeout 180 --wall-timeout 1200 \
  --output-dir "$out/B-2" > "$out/agent.log" 2>&1
