#!/bin/bash
# WSL Ubuntu 22.04 (Humble)에서: SLAM + Nav2 launch 를 백그라운드로 띄우고 시뮬 브리지를 실행한다.
# 사용: bash scripts/ros_sim_demo.sh [slam=cartographer|slam_toolbox] [run_sim_ros.py 인자...]
#   예: bash scripts/ros_sim_demo.sh cartographer --robot B2 --seed 1 --t-base 40
# 로그: $LOGDIR(기본 ~/maicon_ros_logs) / launch.log.  동시에 여러 개 돌릴 때는 ROS_DOMAIN_ID, LOGDIR 를 서로 다르게.
SLAM="${1:-cartographer}"; shift || true
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGDIR="${LOGDIR:-$HOME/maicon_ros_logs}"; mkdir -p "$LOGDIR"
source /opt/ros/humble/setup.bash
export PYTHONIOENCODING=utf-8
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
PARAMS_ARG=""; [ -n "${PARAMS:-}" ] && PARAMS_ARG="params_file:=$PARAMS"     # 파라미터 변형 실험용: PARAMS=/경로/yaml
NAV2_ARG="nav2:=${NAV2:-true}"                                                 # NAV2=false → SLAM 만(폴백 시험)
setsid ros2 launch "$HERE/ros2/launch/maicon_nav.launch.py" slam:="$SLAM" use_sim_time:=true $NAV2_ARG $PARAMS_ARG > "$LOGDIR/launch.log" 2>&1 &
LPID=$!      # setsid → 새 프로세스 그룹의 리더. 정리는 그 그룹만 (다른 동시 실행에 영향 없음)
cleanup() { kill -INT -- -$LPID 2>/dev/null; sleep 2; kill -KILL -- -$LPID 2>/dev/null; true; }
trap cleanup EXIT
sleep 6
cd "$HERE" && python3 scripts/run_sim_ros.py "$@"
