#!/usr/bin/env bash
# Actual Gazebo camera recording: firstperson (B2) or thirdperson (whole arena).
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"
source /opt/ros/humble/setup.bash
set -u
PHYSICAL_ROBOTS="${PHYSICAL_ROBOTS:-one}"
case "$PHYSICAL_ROBOTS" in
  one) export ROBOT=B2; OPPONENTS=none ;;
  six) export ROBOT=all; OPPONENTS=virtual ;;
  *) echo 'PHYSICAL_ROBOTS must be one or six' >&2; exit 2 ;;
esac
export SEED="${SEED:-2}" M3_COUNT="${M3_COUNT:-4}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-197}" ROS_LOCALHOST_ONLY=1
export IGN_PARTITION="${IGN_PARTITION:-maicon_fpv_$$}" GZ_PARTITION="${IGN_PARTITION:-maicon_fpv_$$}"
VIEW="${VIEW:-firstperson}"
case "$VIEW" in
  firstperson) export FPV=1 THIRD_PERSON=0; CAMERA_TOPIC=/B2/fpv/image; CAMERA_FPS=10 ;;
  thirdperson) export FPV=0 THIRD_PERSON=1; CAMERA_TOPIC=/recording/arena/image; CAMERA_FPS=5 ;;
  *) echo 'VIEW must be firstperson or thirdperson' >&2; exit 2 ;;
esac
export CONTROL=match LAYOUT=fixed HEADLESS="${HEADLESS:-1}"
OUTPUT="${OUTPUT:-$HERE/ros2/gazebo/recordings/${VIEW}_${PHYSICAL_ROBOTS}_$(date +%Y%m%d_%H%M%S).mp4}"
STEM="${OUTPUT%.mp4}"
for suffix in .mp4 .json .log .launch.log .camera.log .ready.json .recording.json .frames.jsonl .first.png .last.png .path.png; do
  [[ ! -e "$STEM$suffix" ]] || { echo "Refusing to overwrite $STEM$suffix" >&2; exit 2; }
done
mkdir -p "$(dirname "$OUTPUT")"
LAUNCH_PID="" REC_PID=""
stop_recording() {
  if [[ -n "$REC_PID" ]]; then
    kill -TERM "$REC_PID" 2>/dev/null || true
    wait "$REC_PID" || REC_STATUS=$?
    REC_PID=""
  fi
}
cleanup() {
  stop_recording
  if [[ -n "$LAUNCH_PID" ]]; then
    kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid bash scripts/gazebo_nav_demo.sh > "$STEM.launch.log" 2>&1 &
LAUNCH_PID=$!
python3 -u scripts/record_gazebo_fpv.py --output "$OUTPUT" --ready-file "$STEM.ready.json" \
  --topic "$CAMERA_TOPIC" --fps "$CAMERA_FPS" \
  --first-frame-timeout 90 --idle-timeout 60 --max-wall 1800 > "$STEM.camera.log" 2>&1 &
REC_PID=$!
for ((i=0; i<95; i++)); do
  [[ ! -f "$STEM.ready.json" ]] || break
  kill -0 "$REC_PID" 2>/dev/null || { cat "$STEM.camera.log" >&2; exit 1; }
  kill -0 "$LAUNCH_PID" 2>/dev/null || { tail -30 "$STEM.launch.log" >&2; exit 1; }
  sleep 1
done
[[ -f "$STEM.ready.json" ]] || { echo "No $VIEW camera frame received" >&2; exit 1; }
echo "Recording $VIEW $PHYSICAL_ROBOTS-robot trial: $OUTPUT"
MATCH_STATUS=0 REC_STATUS=0
MATCH_ARGS=()
[[ "${REQUIRE_PERFECT_M3:-0}" != "1" ]] || MATCH_ARGS+=(--require-perfect-m3)
[[ -z "${UAV_CALIBRATION:-}" ]] || MATCH_ARGS+=(--uav-calibration "$UAV_CALIBRATION")
python3 -u scripts/run_gazebo_match.py --seed "$SEED" --m3-count "$M3_COUNT" \
  "${MATCH_ARGS[@]}" \
  --physical-robots "$PHYSICAL_ROBOTS" --opponents "$OPPONENTS" \
  --duration "${DURATION:-300}" --base-at "${BASE_AT:-240}" --smog-at "${SMOG_AT:-120}" \
  --uav-layout "${UAV_LAYOUT:-image}" --uav-schedule "${UAV_SCHEDULE:-}" \
  --report "$STEM.json" > "$STEM.log" 2>&1 || MATCH_STATUS=$?
stop_recording
if [[ -f "$STEM.json" ]]; then
  python3 scripts/plot_m3_trial.py "$STEM.json" --output "$STEM.path.png"
fi
echo "Saved: $OUTPUT (match status=$MATCH_STATUS, camera status=$REC_STATUS)"
[[ "$MATCH_STATUS" == 0 && "$REC_STATUS" == 0 ]]
