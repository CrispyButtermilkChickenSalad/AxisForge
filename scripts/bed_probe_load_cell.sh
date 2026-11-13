#!/usr/bin/env bash
# Run from any host directory; use the workspace and lock shared by the worker.
set -euo pipefail
probe_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$probe_root"
probe_exec_flags=()
if [[ ! -t 0 || ! -t 1 ]]; then probe_exec_flags+=(-T); fi
exec docker compose exec "${probe_exec_flags[@]}" dev bash -lc \
  'source /etc/axisforge/env.bash; export AXISFORGE_ROBOT_LOCK=/workspace/.axisforge-robot-control.lock; exec ros2 run print_control bed_probe --mode load-cell --config /workspace/src/axisforge_bringup/config/ur_master.yaml "$@"' \
  bed-probe "$@"
