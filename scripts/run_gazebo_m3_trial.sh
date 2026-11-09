#!/usr/bin/env bash
# One isolated, reproducible M3 trial. Override SEED to generate another layout.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"
source /opt/ros/humble/setup.bash
set -u
export SEED="${SEED:-2}" M3_COUNT="${M3_COUNT:-4}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-195}" ROS_LOCALHOST_ONLY=1
export IGN_PARTITION="${IGN_PARTITION:-maicon_m3_$$}"
export GZ_PARTITION="$IGN_PARTITION"
export CONTROL=match ROBOT=B2 LAYOUT=fixed HEADLESS="${HEADLESS:-0}"
REPORT="${REPORT:-$HERE/ros2/gazebo/recordings/m3_seed${SEED}_$(date +%Y%m%d_%H%M%S).json}"
mkdir -p "$(dirname "$REPORT")"
setsid bash scripts/gazebo_nav_demo.sh > "${REPORT%.json}.launch.log" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
echo "M3 trial: seed=$SEED cubes=$M3_COUNT report=$REPORT"
python3 -u scripts/run_gazebo_match.py --seed "$SEED" --m3-count "$M3_COUNT" \
  --opponents "${OPPONENTS:-none}" \
  --uav-layout "${UAV_LAYOUT:-image}" --uav-schedule "${UAV_SCHEDULE:-}" \
  --report "$REPORT" > "${REPORT%.json}.log" 2>&1
python3 scripts/plot_m3_trial.py "$REPORT" --output "${REPORT%.json}.path.png"
echo "Finished: $REPORT"
