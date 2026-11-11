#!/usr/bin/env bash
# ROSIDL does not reliably build from paths containing spaces. Keep the canonical
# source untouched and create a fresh, space-free Linux workspace for colcon.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_ws="${ARENA_BUILD_WS:-$(mktemp -d "$HOME/maicon_arena_ws.XXXXXX")}"
if [[ "$build_ws" != /* || "$build_ws" =~ [[:space:]] ]]; then
  echo 'ARENA_BUILD_WS must be an absolute Linux path without whitespace.' >&2
  exit 2
fi
for package in mil_interfaces_sim wingbot_description maicon_arena_gz; do
  if [[ -e "$build_ws/src/$package" ]]; then
    echo 'Destination already has an arena package; choose a new ARENA_BUILD_WS.' >&2
    exit 2
  fi
done
mkdir -p -- "$build_ws/src"
for package in mil_interfaces_sim wingbot_description maicon_arena_gz; do
  cp -a -- "$repo_dir/ros2/$package" "$build_ws/src/$package"
done
set +u
source /opt/ros/humble/setup.bash
set -u
cd -- "$build_ws"
colcon build --base-paths src/mil_interfaces_sim src/wingbot_description src/maicon_arena_gz
printf 'Build workspace: %s\n' "$build_ws"
printf 'Activate: source %q/install/setup.bash\n' "$build_ws"
printf '%s\n' 'Launch: ros2 launch maicon_arena_gz arena.launch.py profile:=standard gui:=false'
