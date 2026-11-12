#!/bin/bash
set -eo pipefail
source /etc/axisforge/env.bash
cd /workspace
bash scripts/install_ur_models.sh
args=(--config "${AXISFORGE_CONFIG:-/workspace/src/axisforge_bringup/config/ur_master.yaml}"
      --port "${AXISFORGE_SYNC_PORT:-8081}")
if [[ "${AXISFORGE_SYNC_ENABLE:-0}" == 1 ]]; then args+=(--enable-motion); fi
exec ros2 run print_control axisforge_sync_worker "${args[@]}"
