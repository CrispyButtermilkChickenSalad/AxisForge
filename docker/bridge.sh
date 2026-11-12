#!/bin/bash
set -eo pipefail
source /etc/axisforge/env.bash
cd /workspace

if [[ ! -f .docker/install/local_setup.bash ]] || \
        ! ros2 pkg prefix axisforge_bringup >/dev/null 2>&1 || \
        ! ros2 pkg prefix print_control >/dev/null 2>&1; then
    echo 'Build the shared workspace first: docker compose exec dev axisforge-build' >&2
    exit 1
fi
if [[ ! -f Klipper-AxisForge-Bridge/bridge_dashboard/app.py ]]; then
    echo 'The mounted workspace must contain Klipper-AxisForge-Bridge/bridge_dashboard/app.py.' >&2
    exit 1
fi
if [[ ! -d "${AXISFORGE_GCODE_ROOT:-}" || ! -f "${AXISFORGE_CONFIG:-}" ]]; then
    echo 'AXISFORGE_GCODE_ROOT must be a directory and AXISFORGE_CONFIG a YAML file inside the container.' >&2
    exit 1
fi

# The colcon install is shared, but each container has its own virtual environment.
# Install the mounted custom models here too, without rebuilding the shared workspace.
bash scripts/install_ur_models.sh
exec python -m uvicorn bridge_dashboard.app:app \
    --app-dir /workspace/Klipper-AxisForge-Bridge \
    --host 127.0.0.1 --port "${BRIDGE_PORT:-8080}"
