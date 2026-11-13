#!/bin/bash
set -eo pipefail
source /etc/axisforge/env.bash
cd /workspace
exec python3 -m uvicorn bridge_dashboard.robot_connection:app \
  --app-dir /workspace/Klipper-AxisForge-Bridge --host 127.0.0.1 \
  --port "${AXISFORGE_ROBOT_CONNECTION_PORT:-8082}"
