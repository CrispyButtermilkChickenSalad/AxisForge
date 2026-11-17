#!/bin/bash

set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CALIBRATION_FILE="${UR_CALIBRATION_FILE:-$SCRIPT_DIR/../src/axisforge_bringup/config/ur16e_calibration.yaml}"
if [[ ! -r "$CALIBRATION_FILE" ]]; then
    echo "UR16e calibration file not found: $CALIBRATION_FILE" >&2
    exit 1
fi
source /opt/ros/humble/setup.bash

# Use the reviewed Humble completion/cancellation fix for both controller
# libraries. Fail closed if a new image or rebuild invalidates the verification.
CONTROLLER_FIX="$SCRIPT_DIR/../.docker/controller-fix"
exec 9>"$SCRIPT_DIR/../.axisforge-controller-build.lock"
flock -n -s 9 || { echo 'Controller overlay is being rebuilt; retry after it finishes.' >&2; exit 1; }
if [[ ! -r "$CONTROLLER_FIX/verified-libraries.sha256" ||
      ! -r "$CONTROLLER_FIX/verified-dependencies.sha256" ||
      ! -r "$CONTROLLER_FIX/install/local_setup.bash" ]] ||
   ! (cd "$CONTROLLER_FIX" && sha256sum --check --status verified-libraries.sha256 verified-dependencies.sha256); then
    echo 'Controller completion fix is missing or outdated. Run bash scripts/build_controller_fix.sh with the PC driver stopped.' >&2
    exit 1
fi
source "$CONTROLLER_FIX/install/local_setup.bash"

exec ros2 launch "$SCRIPT_DIR/../src/axisforge_bringup/launch/ur_driver.launch.py" \
    ur_type:=ur16e \
    robot_ip:="${ROBOT_IP:-192.168.0.100}" \
    kinematics_params_file:="$CALIBRATION_FILE" \
    controller_spawner_timeout:=60 \
    launch_rviz:=false "$@"
