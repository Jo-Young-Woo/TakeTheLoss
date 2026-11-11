#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
WINGBOT_MODEL_INSTALL="${WINGBOT_MODEL_INSTALL:-/home/jojo0429/wingbot_model_ws_20261109/install}"
source "$WINGBOT_MODEL_INSTALL/setup.bash"
cd "$PROJECT_ROOT"
python3 scripts/selected_runtime.py
exec python3 scripts/run_wingbot_selfplay.py --variant progress --scenario selfplay --seed 0 \
  --duration 300 --ready-timeout 180 --wall-timeout 3600 \
  --output "recordings/dev_$(date +%Y%m%d_%H%M%S)" "$@"
