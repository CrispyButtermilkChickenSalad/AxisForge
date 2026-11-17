#!/bin/bash
# Build a private overlay; never restarts a service or sends hardware commands.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FIX="$ROOT/.docker/controller-fix"
# Shared mount lock also works across Compose PID namespaces. Never overwrite
# mapped controller libraries while a driver using this overlay is running.
exec 9>"$ROOT/.axisforge-controller-build.lock"
flock -n -x 9 || { echo 'Stop the PC driver before rebuilding its controller overlay.' >&2; exit 1; }
source /opt/ros/humble/setup.bash
mkdir -p "$FIX/downloads" "$FIX/src"
python3 - <<'PY'
import xml.etree.ElementTree as ET
for package, expected in [('joint_trajectory_controller', '2.54.1'), ('ur_controllers', '2.15.0'), ('realtime_tools', '2.15.1')]:
    version = ET.parse('/opt/ros/humble/share/'+package+'/package.xml').findtext('version')
    if version != expected:
        raise SystemExit(f'{package} {version}: overlay requires reviewed version {expected}')
PY
fetch() {
    local name="$1" url="$2" checksum="$3" subtree="$4"
    if [[ ! -f "$FIX/downloads/$name" ]]; then
        curl -fsSL --max-time 90 "$url" -o "$FIX/downloads/$name.part"
        mv "$FIX/downloads/$name.part" "$FIX/downloads/$name"
    fi
    printf '%s  %s\n' "$checksum" "$FIX/downloads/$name" | sha256sum --check --status
    tar -xzf "$FIX/downloads/$name" -C "$FIX/src" --strip-components=1 "$subtree"
}
fetch ros2_controllers-2.54.1.tar.gz \
    https://codeload.github.com/ros-controls/ros2_controllers/tar.gz/refs/tags/2.54.1 \
    2322184f8d1f7d5ed8b1462c50a9cb0435e580213737667beebf4cf8f00aaa3c \
    ros2_controllers-2.54.1/joint_trajectory_controller
fetch ur-driver-2.15.0.tar.gz \
    https://codeload.github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tar.gz/refs/tags/2.15.0 \
    538b7024db1aa46619a107a03aaf80108744b2e5ff107002f01cf97386ff4a99 \
    Universal_Robots_ROS2_Driver-2.15.0/ur_controllers
python3 "$ROOT/scripts/patch_controller_goal_handle.py" \
    /opt/ros/humble/include/realtime_tools/realtime_tools/realtime_server_goal_handle.hpp \
    "$FIX/include/realtime_tools/realtime_server_goal_handle.hpp"
cmake -S "$ROOT/tests/controller_goal_completion" -B "$FIX/test-patched" \
    -DCMAKE_BUILD_TYPE=Release -DPATCHED_INCLUDE="$FIX/include"
cmake --build "$FIX/test-patched" -j2
ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1 "$FIX/test-patched/goal_completion"
# Both libraries instantiate runNonRealtime; rebuilding just one is insufficient.
# Keep the patched header private: no changes to /opt/ros or trajectory algorithms.
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon --log-base "$FIX/log" build \
    --base-paths "$FIX/src" --build-base "$FIX/build" --install-base "$FIX/install" \
    --packages-select joint_trajectory_controller ur_controllers --executor sequential \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
    "-DCMAKE_CXX_FLAGS=-I$FIX/include"
for package in joint_trajectory_controller ur_controllers; do
    cmake -S "$ROOT/tests/controller_goal_completion" -B "$FIX/test-$package" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCONTROLLER_LIBRARY="$FIX/install/$package/lib/lib$package.so"
    cmake --build "$FIX/test-$package" -j2
    ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1 \
        LD_LIBRARY_PATH="$FIX/install/ur_controllers/lib:$FIX/install/joint_trajectory_controller/lib:$LD_LIBRARY_PATH" \
        "$FIX/test-$package/goal_completion"
done
# Record successful tests/builds; the driver additionally checks library hashes.
(
    cd "$FIX"
    sha256sum install/joint_trajectory_controller/lib/libjoint_trajectory_controller.so \
        install/ur_controllers/lib/libur_controllers.so > verified-libraries.sha256.tmp
    mv verified-libraries.sha256.tmp verified-libraries.sha256
    sha256sum /opt/ros/humble/share/{joint_trajectory_controller,ur_controllers,realtime_tools,rclcpp,rclcpp_action,controller_interface,hardware_interface}/package.xml \
        /opt/ros/humble/include/realtime_tools/realtime_tools/realtime_server_goal_handle.hpp \
        > verified-dependencies.sha256
)
echo 'Controller overlay built and tested. Activation requires a stopped driver restart.'
