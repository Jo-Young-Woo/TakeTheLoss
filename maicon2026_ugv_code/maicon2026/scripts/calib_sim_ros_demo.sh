#!/bin/bash
# WSL(Ubuntu 22.04, Humble): 시뮬 로봇을 ROS 로 띄우고 실제 ROS 경로(RosIO, rosbag)로 보정 시험 → 분석 → 산출물 생성까지.
# 실기 리허설용 — 실기에서는 sim_bridge_only 대신 로봇 드라이버가 같은 토픽을 낸다.
#   bash scripts/calib_sim_ros_demo.sh [출력폴더] [시험들...]
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$HERE/calib/records/demo_ros}"; shift
EXPS="${@:-static speed_steps turn_steps straight rotate brake lidar_square}"
source /opt/ros/humble/setup.bash
export PYTHONIOENCODING=utf-8 ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-90}"
cd "$HERE"
setsid python3 scripts/sim_bridge_only.py > /tmp/sim_bridge.log 2>&1 &
BPID=$!
trap 'kill -INT -- -$BPID 2>/dev/null; sleep 1; kill -KILL -- -$BPID 2>/dev/null' EXIT
sleep 3
rm -rf "$OUT"
python3 scripts/calib_run.py --io ros --out "$OUT" --truth-topic /ground_truth --teleport-topic /sim/teleport \
        --non-interactive --experiments $EXPS --d-front 1.12 --d-left 1.0
