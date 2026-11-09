#!/usr/bin/env bash
# Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress. Run from any directory.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
set -u
export IGN_GAZEBO_RESOURCE_PATH="$HERE/ros2/gazebo${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
command -v ign >/dev/null || { echo 'Gazebo Fortress is missing: sudo apt install ros-humble-ros-gz' >&2; exit 1; }
ros2 pkg prefix ros_gz_bridge >/dev/null || { echo 'ros_gz_bridge is missing: sudo apt install ros-humble-ros-gz' >&2; exit 1; }
SEED="${SEED:-2}"
ROBOT="${ROBOT:-B2}"
LAYOUT="${LAYOUT:-fixed}"
M3_COUNT="${M3_COUNT:-4}"
LIDAR_SAMPLES="${LIDAR_SAMPLES:-360}"
LIDAR_MODE="${LIDAR_MODE:-sector160}"
SLAM="${SLAM:-cartographer}"
CONTROL="${CONTROL:-nav2}"
case "$CONTROL" in
  nav2) BRIDGE_CONFIG="$HERE/ros2/gazebo/bridge.yaml" ;;
  agent) BRIDGE_CONFIG="$HERE/ros2/gazebo/bridge_agent.yaml" ;;
  match) if [[ "$ROBOT" == "all" ]]; then
           BRIDGE_CONFIG="$HERE/ros2/gazebo/bridge_match.yaml"
         else
           BRIDGE_CONFIG="$HERE/ros2/gazebo/bridge_agent.yaml"
         fi ;;
  *) echo "CONTROL must be nav2, agent or match" >&2; exit 2 ;;
esac
if [[ "$CONTROL" == "match" && "$ROBOT" != "all" && "$ROBOT" != "B2" ]]; then
  echo "CONTROL=match requires ROBOT=B2 or ROBOT=all" >&2
  exit 2
fi
WORLD="${WORLD:-/tmp/maicon_arena_${LAYOUT}_${SEED}_${ROBOT}_m3_${M3_COUNT}_$$.sdf}"
FPV_ARGS=()
[[ "${FPV:-0}" != "1" ]] || FPV_ARGS+=(--fpv)
[[ "${THIRD_PERSON:-0}" != "1" ]] || FPV_ARGS+=(--third-person)
[[ -f "$HERE/ros2/gazebo/assets/field_markings.obj" ]] || python3 "$HERE/scripts/render_gazebo_field_texture.py"
python3 "$HERE/scripts/generate_gazebo_world.py" --layout "$LAYOUT" --seed "$SEED" \
  --robot "$ROBOT" --lidar-samples "$LIDAR_SAMPLES" --lidar-mode "$LIDAR_MODE" --m3-count "$M3_COUNT" "${FPV_ARGS[@]}" \
  --effective-track "${SIM_EFFECTIVE_TRACK:-.17452}" \
  --render-engine "${SIM_RENDER_ENGINE:-ogre}" \
  --banner-texture "$HERE/ros2/gazebo/assets/front_banner.png" \
  --field-mesh "$HERE/ros2/gazebo/assets/field_markings.obj" \
  --block-texture "$HERE/ros2/gazebo/assets/dark_block.png" --output "$WORLD"

SIM_LIBGL_ALWAYS_SOFTWARE="${SIM_LIBGL_ALWAYS_SOFTWARE:-1}"
GUI_LIBGL_ALWAYS_SOFTWARE="${GUI_LIBGL_ALWAYS_SOFTWARE:-0}"
LIBGL_ALWAYS_SOFTWARE="$SIM_LIBGL_ALWAYS_SOFTWARE" setsid ign gazebo -r -s "$WORLD" &
GZ_PID=$!
GUI_PID=""
if [[ "${HEADLESS:-0}" != "1" ]]; then
  LIBGL_ALWAYS_SOFTWARE="$GUI_LIBGL_ALWAYS_SOFTWARE" \
    setsid ign gazebo -g --render-engine "${GUI_RENDER_ENGINE:-ogre}" &
  GUI_PID=$!
fi
setsid ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$BRIDGE_CONFIG" &
BRIDGE_PID=$!
LIDAR_MERGE_PID=""
if [[ "$LIDAR_MODE" == "sectors360" && "$ROBOT" != "none" ]]; then
  MERGE_ROBOTS=("$ROBOT")
  [[ "$ROBOT" != "all" ]] || MERGE_ROBOTS=(B1 B2 B3 R1 R2 R3)
  setsid python3 -u "$HERE/scripts/merge_gazebo_lidar.py" --samples "$LIDAR_SAMPLES" \
    --robots "${MERGE_ROBOTS[@]}" &
  LIDAR_MERGE_PID=$!
fi
# This SDF has no robot_state_publisher. Only the primary UGV uses Nav2/TF;
# the other match robots use their own topic-prefixed HAL. Keep this transform
# equal to maicon26/hal/gazebo_model.py (checked by test_gazebo_contract.py).
setsid ros2 run tf2_ros static_transform_publisher \
  --x 0.039815 --y 0.0 --z 0.16 --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id gazebo_lidar &
LIDAR_TF_PID=$!
setsid ros2 launch "$HERE/ros2/launch/maicon_nav.launch.py" \
  slam:="$SLAM" use_sim_time:=true odom_tf:=auto depth_obstacles:=false \
  cartographer_config:="${CARTOGRAPHER_CONFIG:-cartographer_gazebo_2d.lua}" &
NAV_PID=$!
cleanup() {
  [[ -z "$LIDAR_MERGE_PID" ]] || kill -INT -- "-$LIDAR_MERGE_PID" 2>/dev/null || true
  [[ -z "$GUI_PID" ]] || kill -INT -- "-$GUI_PID" 2>/dev/null || true
  kill -INT -- "-$NAV_PID" "-$LIDAR_TF_PID" "-$BRIDGE_PID" "-$GZ_PID" 2>/dev/null || true
  sleep 2
  [[ -z "$LIDAR_MERGE_PID" ]] || kill -KILL -- "-$LIDAR_MERGE_PID" 2>/dev/null || true
  [[ -z "$GUI_PID" ]] || kill -KILL -- "-$GUI_PID" 2>/dev/null || true
  kill -KILL -- "-$NAV_PID" "-$LIDAR_TF_PID" "-$BRIDGE_PID" "-$GZ_PID" 2>/dev/null || true
  wait "$NAV_PID" "$LIDAR_TF_PID" "$BRIDGE_PID" "$GZ_PID" ${GUI_PID:+"$GUI_PID"} ${LIDAR_MERGE_PID:+"$LIDAR_MERGE_PID"} 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
echo "Gazebo arena layout=$LAYOUT seed=$SEED robot=$ROBOT control=$CONTROL lidar=$LIDAR_MODE | /scan /odom /clock /navigate_to_pose"
wait -n "$GZ_PID" "$BRIDGE_PID" "$LIDAR_TF_PID" "$NAV_PID" ${GUI_PID:+"$GUI_PID"} ${LIDAR_MERGE_PID:+"$LIDAR_MERGE_PID"}
