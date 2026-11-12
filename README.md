# AxisForge

## Experimental Klipper follower work

**Status: offline validation only, not a live robot follower.** This fork contains
a separate, opt-in adapter for the experimental `robot_motion/stream` exporter
in [CrispyButtermilkChickenSalad/Klipper-ROS2-Bridge](https://github.com/CrispyButtermilkChickenSalad/Klipper-ROS2-Bridge).
It does not import ROS, load a robot model, open a network connection, publish
extrusion, or execute robot motion. No synchronized-start or hardware-safety
claim is made.

Klipper is intended to own motion timing, extrusion, heaters, fans and other
auxiliary electronics; the UR16e would follow its planned toolhead motion.
The current exporter exposes **retained scheduled trapq moves and history**,
not a guaranteed complete or timely future execution queue. It is not sufficient
to drive that live architecture.
Do not run the standalone AxisForge G-code interpreter concurrently with
Klipper for the same job: that interpreter independently plans/retimes motion
and commands the Portenta extruder.

### Why the existing executor is not connected

The inspected standalone path is:

1. `gcode_interpreter.py` parses G-code and preflights the entire job.
2. `gcode_path.py` applies coordinate/bed transforms, look-ahead, IK and cubic
   joint interpolation. `retime()` and `continuous_motion()` stretch trajectory
   time to satisfy robot limits, then derive extrusion from that new timeline.
3. `motion_executor.py` sends serial, **unstamped** `FollowJointTrajectory`
   actions to `/scaled_joint_trajectory_controller/follow_joint_trajectory`.
   Execution starts at controller acceptance; host monotonic time estimates
   elapsed extrusion time. Each action waits for completion.

This cannot preserve an external Klipper timeline. The new
`print_control.klipper_follower` path does not call any of those planners or
executors. Existing standalone launch/configuration behavior is unchanged.

### Assumed experimental protocol v1

The companion `klipper_extras/robot_motion_bridge.py` registers
`robot_motion/stream` with mux `name="all"`. Its websocket batches put the
payload in `message.params`. The adapter accepts that wrapper or a bare payload
for offline capture files. It does **not** accept the subscription's initial
header/template response; capture only actual motion batch messages.

```json
{
  "params": {
    "protocol": 1,
    "sequence": 7,
    "clock": "klipper_print_time",
    "latest_motion_time": 12.25,
    "streams": {
      "toolhead": [{
        "print_time": 12.0,
        "duration": 0.25,
        "start_velocity": 20.0,
        "acceleration": 0.0,
        "start_position": [10.0, 20.0, 30.0],
        "direction": [1.0, 0.0, 0.0]
      }]
    }
  }
}
```

`streams` maps queue names to nonempty record arrays; empty notifications are
not emitted. Only `toolhead` records are interpreted. Positions are XYZ mm in
the exported Klipper frame, times/durations are seconds in `klipper_print_time`,
scalar speeds are mm/s, scalar accelerations are mm/s squared and direction
components are dimensionless. At elapsed time `t` within a phase, position is
`start_position + direction * (start_velocity*t + acceleration*t*t/2)`.
The adapter preserves direction, units, absolute times, phase boundaries and
supplied acceleration; it never rescales or retimes.
`TimedToolheadMove.sample(print_time)` evaluates Cartesian position, velocity
and acceleration only within that phase.

Extruder queues use their native extrusion coordinates, **not Cartesian XYZ**.
They are ignored, never fused with toolhead records or sent to the Portenta.
They still count toward the per-batch record bound. `latest_motion_time` is
required and must be finite/nonnegative but is informational: it is the maximum
record end in that batch, not a host-clock timestamp, execution deadline,
synchronization anchor, or completeness guarantee.
Additional fields are ignored; unsupported protocol versions/clocks are rejected.

These are assumptions about an experimental exporter, not a finalized public
API. Confirm them against the exact custom Klipper build before recording.
There is no network/subscription implementation here and no HTTP G-code path.

### Setup and offline use

Use this fork, not upstream:

```bash
git clone https://github.com/CrispyButtermilkChickenSalad/AxisForge.git
cd AxisForge
# Save actual motion batch messages as one JSON object per line; no pretty-printing.
# This module needs only Python's standard library, not ROS or the UR driver.
PYTHONPATH="$PWD/src/print_control" python3 -m print_control.klipper_follower \
  --experimental-klipper-follower --input /path/to/capture.ndjson
```

The JSON example above can be saved on a single line to smoke-test the format.
It is a synthetic format example, not a robot trajectory. After the normal ROS
workspace build (`./scripts/build.sh` and `source install/setup.bash`), the same
offline command is available as:

```bash
ros2 run print_control klipper_follower \
  --experimental-klipper-follower --input /path/to/capture.ndjson
```

The command is disabled without the explicit flag. It prints a JSON validation
summary only after the entire file passes, with `mode=capture_validation_only`
and `live_execution_available=false`. Invalid input produces stderr and exit 1.
There is deliberately no `--execute` switch or live launch integration.

### Buffer limits, sequence handling and recovery

Default limits are 1 MiB per NDJSON line (`--max-line-bytes`), 4096 records per
batch/pending toolhead FIFO (`--max-records`) and a 10-second pending time span
including gaps (`--max-buffer-s`). Oversized input is rejected, not truncated.
Offline validation drains each batch immediately, so the horizon is per pending
batch there; API consumers that do not drain are bounded across batches too.

`KlipperCaptureBuffer(experimental=True)` exposes `ingest(message)`, `pop()` and
`len(buffer)` for hardware-free consumers. The first sequence may be any
nonnegative integer, allowing partial captures; every following batch must
increment by exactly one, including batches containing only extruder records.
The exporter's first sequence after the shared helper starts is 1, but a client
joining an existing subscription can observe a higher first sequence. There is
no epoch identifier when the helper restarts after all clients disconnect.
Queue timestamps must not overlap or regress, even after draining (1 ns
comparison tolerance for floating-point arithmetic; timestamps are never snapped).
Gaps are preserved and counted in the CLI summary, not interpolated into a
hold or motion. No Cartesian continuity, reachability or collision check is
performed. Missing records cannot be reconstructed.

Zero-duration position-set markers are rejected: without an explicit marker
type or guaranteed delivery, the adapter cannot interpret them as safe coordinate
resets. The exporter's cursor does not guarantee deduplication, gap-free coverage
or continuity after queue edits/reconnects.

Ingestion is transactional. A sequence gap/reset/duplicate, malformed phase,
overflow, overlap or capacity violation latches a fault and makes pending data
unavailable. `pop()` on an empty buffer also faults. There is no automatic
retry, eviction or sequence resynchronization: discard the adapter and start a
new capture/session explicitly. The CLI treats end-of-file as offline completion,
not proof that a live job has ended. A file with no toolhead phases is rejected.
No wall-clock freshness or live underrun deadline is inferred from this capture.

### Testing the adapter

Run focused tests without robot dependencies:

```bash
PYTHONPATH="$PWD/src/print_control" python3 -m unittest discover \
  -s src/print_control/test -p test_klipper_follower.py -v
```

The implementation was checked with 15 focused unit tests, Python compilation
and a subprocess smoke test of the documented CLI, including default-disabled
behavior and rejection of `--execute`. These are offline checks only; ROS/UR
runtime, custom Klipper behavior, calibration and hardware synchronization have
not been validated.

### Blockers before any live UR follower

The following need an agreed interface and live validation, not an enable flag:

1. A **future** Klipper motion schedule with guaranteed look-ahead/lead time,
   backpressure, job identity, start barrier, reset/reconnect semantics, idle/end
   markers and a coordinated stop mechanism. A diagnostic mixture of retained
   scheduled moves/history and sequence numbers alone cannot provide these.
   Test the exporter on the actual custom Klipper build; records may already be
   too late when received.
2. A measured Klipper print-time to ROS/controller-clock mapping, with bounded
   offset/drift, timestamp age and transport jitter. Define late-data and buffer
   underrun deadlines. Neither a realtime kernel nor starting two jobs together
   establishes synchronization.
3. A UR controller interface for continuous, timestamped trajectory consumption
   without stop/start action gaps or silent speed scaling. Validate acceptance
   latency, trajectory replacement, feedback and protective-stop behavior.
   The existing scaled action executor is not such an interface.
4. Calibrated Klipper-frame to robot-base/TCP/orientation mapping (including bed
   mesh/leveling/kinematic transforms already applied by Klipper). Do not apply
   AxisForge bed compensation twice. The exporter carries no orientation/A-axis
   or robot redundancy contract. AxisForge's nominal Robotics Toolbox UR16e model
   is not the driver-specific calibration loaded by `scripts/ur_ros2_driver.sh`;
   neither model agreement nor calibration has been validated by this change.
5. Timing-preserving IK/Jacobian conversion and model/controller interpolation
   checks against joint speed, acceleration, singularity, workspace and collision
   constraints. Reject infeasible Klipper timing upstream or replan **at Klipper**;
   AxisForge must not independently stretch a follower trajectory.
6. Cross-device stop/pause/fault recovery for robot, extrusion and auxiliaries,
   plus measured tracking error on the actual UR16e under load. Start with
   simulation/captured data and no extrusion; hardware trials require the user's
   established robot safety process.

This adapter is a non-hardware-dependent starting point, not calibration,
hardware validation, a simultaneous-job runner, or permission to operate a robot.

## Repository Layout

```text
src/
├── print_control/       # G-code assets, paths, print execution and keyboard control
├── extrusion_control/   # Extruder steps/mm and feed-forward extrusion profiles
├── axisforge_bringup/   # launch/ and config/, including robot calibration YAML
└── diagnostics/         # Robot and Arduino latency ROS nodes
models/ur16e/           # Custom UR16e DH and URDF classes for Robotics Toolbox
firmware/               # Arduino sketches for extrusion and latency measurements
scripts/                # Setup, build, and launch scripts for ROS 2 and UR16e
docker/                 # Container dependencies, shell setup, and colcon defaults
Klipper-AxisForge-Bridge/ # Companion Bridge checkout: dashboard and Klipper master
STL/                    # Printable example meshes
```

## Docker Development Environment

The container setup follows the existing project environment: four `ament_python`
packages built with colcon, ROS 2 Humble, the UR ROS 2 driver, NumPy 1.26.4,
SciPy 1.11.4, and custom Robotics Toolbox UR16e models. The base image is the
official [`ros:humble-ros-base-jammy`](https://hub.docker.com/_/ros) image
(Ubuntu 22.04 / Python 3.10). The current launch file and driver script run without
RViz, so a desktop image and display forwarding are not needed for normal use.

The Dockerfile installs ROS dependencies using `rosdep` and the four `package.xml`
manifests. The manifests' `ament_python` entry has no Humble rosdep key; that one
key is skipped because colcon's Python/ROS extensions and setuptools provide the
build support. Python dependencies are installed in `/opt/axisforge-venv` with
`--system-site-packages`, preserving access to apt-installed ROS Python packages.
`docker/constraints.txt` selects the Robotics Toolbox versions recorded in the
model installer; `docker/requirements-dev.txt` adds the plotting dependencies
(`pandas`, `seaborn`) and `debugpy`. Native extensions are built against the pinned
NumPy version. Firmware compilation/flashing remains an Arduino IDE task on the host.

**The repository root (`.`) is bind-mounted at `/workspace`. Project source is
never copied into the image.** `.dockerignore` limits the build context to dependency
manifests and container support files. Container colcon output lives in the ignored
`.docker/{build,install,log}` directories, separate from native builds and `.venv`.

### First start

Use a Linux host with Docker Engine and Docker Compose v2, with permission to
access the Docker daemon. Run these commands from the repository root:

```bash
cd /path/to/AxisForge
# Match the container user's ownership to your local checkout.
export LOCAL_UID="$(id -u)"
export LOCAL_GID="$(id -g)"

docker compose up -d --build dev
# Install the mounted UR16e models, validate their kinematics, and build all ROS packages.
docker compose exec dev axisforge-build
docker compose exec dev bash
```

The container runs as a non-root `developer` user. UID/GID default to 1000; retain
the exports above in terminals that rebuild the image, or put `LOCAL_UID` and
`LOCAL_GID` in the ignored root `.env` file. That file can also set `ROBOT_IP`,
`ROS_DOMAIN_ID`, `MICROROS_DEVICE` and `MICROROS_BAUD`. See `.env.example` for
these settings and the optional Klipper dashboard endpoints.

`sleep infinity`, `init: true` and `restart: unless-stopped` keep the development
container available until explicitly stopped. Exiting an interactive shell leaves
it running. Starting the container does not launch any robot-control nodes.

Every new Bash session loads ROS, the container virtual environment, and the built
workspace. Run `axisforge-build` after recreating a container (including after an
image rebuild), because the custom models are installed into that container's
virtual environment. If building inside an already-open shell, refresh it with
`source /etc/axisforge/env.bash` afterward.

### Edit, run, debug and test

Open as many development terminals as needed with `docker compose exec dev bash`.
Changes on the host are immediately visible under `/workspace`.
The build uses [`--symlink-install`](https://colcon.readthedocs.io/en/released/reference/verb/build.html),
so edits to existing Python modules are used the next time a node starts; restart
an already-running Python process to load its changes. No image rebuild is needed.
For YAML/G-code edits, pass their source paths directly when launching.

Inside the development shell, try a small path within the provided probe grid
using the existing launch file, without hardware:

```bash
cat > /tmp/axisforge-smoke.gcode <<'GCODE'
G90
M82
G92 E0
G0 X110 Y110 Z1 F600
G1 X111 Y110 Z1 E0.05 F600
G1 X111 Y111 Z1 E0.10 F600
G1 X110 Y111 Z1 E0.15 F600
G1 X110 Y110 Z1 E0.20 F600
GCODE
ros2 launch axisforge_bringup ur_master.launch.py \
  config:=/workspace/src/axisforge_bringup/config/ur_master.yaml \
  file:=/tmp/axisforge-smoke.gcode \
  dry_run:=true
```

For project G-code, replace `file` with a path such as
`/workspace/src/print_control/gcode/star3d.gcode`. Preflight checks use the selected
bed calibration and configuration, so each path must fit those settings.

For a non-interactive command from the host, use Bash so the workspace environment
is loaded (Docker `exec` itself does not run the image entrypoint):

```bash
docker compose exec dev bash -c 'ros2 pkg list | grep -E "^(axisforge_bringup|print_control|extrusion_control|diagnostics)$"'
```

Run the project's existing tests from the container shell:

```bash
python /usr/bin/colcon test --base-paths src --event-handlers console_direct+
python /usr/bin/colcon test-result --verbose
```

The container defaults explicitly select pytest because the existing `setup.py`
files do not declare `tests_require`; otherwise colcon runs zero tests. The
repository currently includes ament lint tests; failures report source-style
issues as well as environment issues. `python -m pdb` and `debugpy` are available
for Python debugging, and `gdb` is installed for native extensions. Plotting defaults
to the headless `Agg` backend; save figures into `/workspace` and view them on the host.

- After editing package metadata, entry points, adding resource files, or changing
  `models/ur16e`, run `axisforge-build` again and open a fresh shell. The model
  installer copies model resources into the virtual environment at runtime.
- After changing `requirements.txt`, `docker/constraints.txt`,
  `docker/requirements-dev.txt`, `Klipper-AxisForge-Bridge/requirements.txt`,
  ROS dependencies in `package.xml`, or container
  support files, run `docker compose up -d --build dev` and then
  `docker compose exec dev axisforge-build`.
- Packages installed manually with pip are local to that container; record lasting
  dependency changes in the dependency files and rebuild.

### Connect the UR16e and Portenta (optional)

Compose uses Linux host networking and IPC so DDS discovery, Fast DDS shared memory
and the UR driver's reverse connections can work across the host and containers.
Set the same `ROS_DOMAIN_ID` for every participating ROS process. The host must have
network access to the robot. Docker Desktop requires additional host-network and
USB configuration; the provided hardware workflow targets native Linux Docker Engine.

In one host terminal, start the existing UR driver (change the address as needed):

```bash
docker compose exec -e ROBOT_IP=192.168.0.100 dev bash -c './scripts/ur_ros2_driver.sh'
```

Connect the flashed Portenta, then start the optional micro-ROS Agent service:

```bash
export MICROROS_DEVICE=/dev/ttyACM0
docker compose --profile hardware up -d micro-ros
docker compose logs -f micro-ros
```

This service uses the [micro-ROS Agent image](https://hub.docker.com/r/microros/micro-ros-agent/)
and maps only the selected serial device. It replaces `./scripts/microros.sh` for
this Docker workflow; the agent is not installed in the `dev` image. Default serial
baud is 115200; set `MICROROS_BAUD` if the firmware requires another value. No device
mapping is required for the default development service. After unplugging/replugging
the board, recreate the bridge with
`docker compose --profile hardware up -d --force-recreate micro-ros`.

In another development shell, launch the control system with the appropriate
calibration/configuration and G-code paths, using `dry_run:=false` for execution.
The container shares the host kernel: install/configure any required real-time
kernel on the host, as described below. For a UR driver requiring real-time scheduling,
add `cap_add: [SYS_NICE]` and `ulimits: {rtprio: 99, memlock: -1}` to `dev` in a local
Compose override, then recreate the service. The default development container does
not request those permissions.

### Stop and resume

```bash
docker compose stop dev                       # Keep the container and its environment
docker compose start dev                      # Resume it
docker compose --profile hardware stop micro-ros
docker compose --profile hardware down        # Remove containers; keep local source/artifacts
```

The bind-mounted source and `.docker/` build/test output remain on the host after
`down`. The next `up` creates a fresh container; run `axisforge-build` again before
launching nodes.

## Klipper master / AxisForge follower

The synchronized workflow is **Upload → Run preflight → Start synchronized job**.
Klipper owns the timing and extrusion; AxisForge follows its timed XYZ/ABC path.
A/B/C mean rotations about X/Y/Z relative to the configured home orientation.
The worker never sends Portenta extrusion commands.

Place the matching [Bridge checkout](https://github.com/CrispyButtermilkChickenSalad/Klipper-AxisForge-Bridge)
in `Klipper-AxisForge-Bridge/` before building the Docker image. This local workspace
already contains that folder. Keep `.env` out of version control:

```ini
MOONRAKER_URL=http://192.168.137.66:7125
ROBOT_IP=192.168.0.100
AXISFORGE_SYNC_ENABLE=1
```

```bash
docker compose up -d --build dev
docker compose exec dev axisforge-build
docker compose --profile klipper up -d
# Run the driver separately if it is not already running:
docker compose exec dev bash -lc './scripts/ur_ros2_driver.sh'
```

Install the matching Klipper module on the **Klipper host**, following
[Bridge setup](Klipper-AxisForge-Bridge/README.md#klipper-host). It requires
`kinematics: none` and the current extra-axis/motion_queuing API. On this installation
Moonraker is `192.168.137.66:7125`; the UR robot is the separate `192.168.0.100` host.
The worker binds to loopback port 8081 and the dashboard to port 8080.

Open **http://127.0.0.1:8080**:

1. **Upload to both**; the uploaded file is selected automatically.
2. Select **Check extruder heater** and **Check bed heater** independently, then
   **Run preflight**. Klipper verifies the file and generates the timeline. It
   tests selected heaters at G-code targets, then switches all
   targets to zero before robot IK and timing checks. The robot does not move.
3. Start the UR External Control program with speed scaling at **100%** and
   NORMAL safety mode. Click **Start synchronized job** after both checks pass.
   Start reheats, approaches the entry point and runs acknowledged timed chunks.

A heater that is not configured defaults to unchecked. Its G-code commands are
ignored for that job with a warning, so an unheated bed can accept slicer files
containing M140/M190. For configured heaters, disabling a preflight check skips
only the test; Start still follows G-code temperatures. Off targets need no test.
Rerun preflight after changing a heater option. Cold-extrusion limits remain active.

Both systems must complete a chunk before the next begins. Missing heartbeats,
stale feedback, timing faults or controller errors stop the job and turn heaters
off. Aborting already queued extrusion shuts Klipper down; resolve the fault,
restart Klipper and run preflight again. Mid-job resume is not supported.

This first version stops at each G-code move and waits at chunk boundaries. It
has automated protocol, planner and fault tests, but physical synchronization
accuracy and printing quality still need commissioning. The local integration
check uploads a no-heat/no-extrusion file and validates it without starting motion.
See [worker guide](docs/synchronized-jobs.md) and
[Bridge protocol](Klipper-AxisForge-Bridge/docs/synchronized-jobs.md) for limits,
configuration, supported G-code and recovery.

To reload Python changes while no job is active:

```bash
docker compose --profile klipper restart sync-worker bridge
```

After a network interface change, if the UR driver sees joints but the dashboard
reports stale feedback, restart rosbridge so DDS discovery uses the current
interfaces: `docker compose --profile klipper restart rosbridge`.
The ALPS iframe retains local-network/serial permission delegation. Allow the
browser to access the Klipper LAN host, or use **Open in new tab**.

The older `klipper_follower` command remains an offline capture validator. It is
separate from `axisforge_sync_worker` and is not used for live jobs; see
[its guide](docs/klipper-follower.md).

## Installation and Setup

This guide provides step-by-step instructions for setting up the complete software environment.

### Prerequisites

- Ubuntu 22.04 LTS (Jammy Jellyfish)
- Ubuntu Pro account (free for personal use) for RT kernel access

### 1. Real-Time Kernel Installation

For optimal robot control performance, install the RT kernel via Ubuntu Pro:
```bash
# Sign up for Ubuntu Pro and attach PC
sudo pro attach <your-token>

# Enable RT kernel
sudo pro enable realtime-kernel

# Reboot system
sudo reboot
```

### 2. ROS 2 Humble Installation
```bash
# Set up sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ROS 2 Humble
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

# Environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. UR ROS 2 Driver Installation
```bash
sudo apt-get install ros-humble-ur
```

### 4. Python Dependencies Installation
```bash
# Install Python development tools
sudo apt install python3-dev python3-pip python3-venv

# Install required packages from requirements.txt
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/CrispyButtermilkChickenSalad/AxisForge.git
cd AxisForge
# Keep ROS packages visible while installing project dependencies in .venv.
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

### 5. Custom UR16e Models Installation

Install the custom UR16e DH and URDF models from [`models/`](models/README.md)
into the project's `.venv`:
```bash
./scripts/install_ur_models.sh
```

### 6. Micro-ROS Setup

For Arduino Portenta H7:

1. Install Arduino IDE 2.x from https://www.arduino.cc/en/software
2. Add Arduino Mbed OS Portenta Boards via Board Manager
3. The micro-ROS sketch uses the Portenta core's hardware timer support; install `AccelStepper` only for the standalone serial sketch
4. Download latest `micro_ros_arduino` for Humble from [Releases](https://github.com/micro-ROS/micro_ros_arduino/releases)
5. Upload the downloaded ZIP to the IDE using `Sketch -> Include library -> Add .ZIP Library...`
6. Select the Portenta H7 Main Core (M7) target and upload `firmware/DRV8825_microros/DRV8825_microros.ino`
7. Install micro-ROS agent on PC: https://github.com/micro-ROS/micro_ros_setup

### 7. Build the Workspace

```bash
cd /path/to/AxisForge
source /opt/ros/humble/setup.bash
source .venv/bin/activate
# Build entry points with the Python environment containing the UR16e models.
./scripts/build.sh
source install/setup.bash
```

### Usage

```bash
# Terminal 1: Start the UR16e driver
cd /path/to/AxisForge
./scripts/ur_ros2_driver.sh

# Terminal 2: Start the serial bridge to the Portenta extruder
cd /path/to/AxisForge
./scripts/microros.sh

# Terminal 3: Launch control system
cd /path/to/AxisForge
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 launch axisforge_bringup ur_master.launch.py
# Use the source YAML so parameter edits do not require another build.
ros2 launch axisforge_bringup ur_master.launch.py \
  config:="$PWD/src/axisforge_bringup/config/ur_master.yaml" \
  file:="$PWD/src/print_control/gcode/star3d.gcode" dry_run:=true
```

### Bed probing

The bed is movable: probe it again every time it is placed. With the UR driver running
(Terminal 1), jog the nozzle onto a 3x3 grid (G-code X/Y = 30, 117.5, 205) and record each
touch. Probing always uses an upright nozzle (roll 180°, pitch 0°), with the printing yaw
and TCP from `ur_master.yaml`; each point starts `clearance_mm` above `z_offset`, and jogs
stop 3 mm below it.

```bash
cd /path/to/AxisForge   # bed_probe_file is relative to this directory
ros2 run print_control bed_probe \
  --config "$PWD/src/axisforge_bringup/config/ur_master.yaml"   # --dry-run to simulate
```

Keys: `w`/`s` Z up/down, `1`-`4` Z step 1/0.1/0.05/0.01 mm, `i`/`k` +Y/-Y and `j`/`l`
-X/+X in 10 mm steps (`--xy-step-mm`), space records (paper test; `--paper-mm` default 0.1
is subtracted), `n`/`p` next/previous point, `h` lift, `q` lift, save and quit. Ctrl-C
cancels the active trajectory. The result goes to `bed_probe_file` unless `--output` is given.


Configuration is in [`ur_master.yaml`](src/axisforge_bringup/config/ur_master.yaml):

| Parameters | Meaning |
| --- | --- |
| `x_offset`, `y_offset`, `z_offset`, `origin_at_center`, `bed_size_mm` | Preserve the old placement convention; center mode adds half the bed dimensions to X/Y offsets |
| `bed_probe_file`, `bed_z_adjust_mm`, `bed_probe_extrapolation_mm` | Probed bed surface (relative to the launch directory; `''` uses the flat `z_offset`), first-layer fine adjust (+ raises the nozzle), and the allowed XY distance beyond the probed points |
| `nozzle_roll_deg`, `nozzle_pitch_deg`, `wrist_angle` | Home tool orientation in degrees (`wrist_angle` is yaw). G-code `A`, `B`, `C` are absolute rotations in degrees about X, Y and Z, added to roll, pitch and yaw respectively (default 0). Bed probing always uses roll 180°, pitch 0° |
| `tcp_offset_mm` | Nozzle-tip offset in tool0 coordinates; zero preserves old tool0 targets. With a measured TCP, the XYZ offsets must refer to the nozzle tip |
| `waypoint_spacing_mm` | Spatial sampling interval, replacing `split_threshold` |
| `path_tolerance_mm`, `orientation_tolerance_deg` | Model-based checks between IK samples using cubic interpolation |
| `max_joint_speed_deg_s`, `max_joint_acceleration_deg_s2` | Trajectories are stretched in time to respect these joint limits |
| `print_speed_multiplier`, `extrusion_scale_factor` | Apply before speed limiting; zero extrusion scale disables all material motion |
| `max_print_speed_mm_s`, `max_travel_speed_mm_s`, `max_extrusion_speed_mm_s` | Print, travel and filament feed limits |
| `first_layer_speed_factor`, `first_layer_height_mm` | Prefer `;LAYER:0`; use the height threshold only without layer metadata |
| `minimum_segment_duration_s` | Minimum motion duration; with look-ahead this applies to a whole run, not each source segment |
| `lookahead_enabled` | Defaults to true: combine connected spatial moves of the same extrusion sign into one action; false restores per-segment stops |
| `max_path_acceleration_mm_s2` | Path acceleration used for forward/backward look-ahead; sampled cubic TCP acceleration is also checked and retimed |
| `corner_blend_tolerance_mm` | Allowed corner rounding (default 0.05 mm); must be smaller than `path_tolerance_mm`. Zero keeps exact corners with zero junction speed |
| `max_corner_speed_mm_s` | Upper speed bound within a corner blend, further constrained by curvature and joint limits |
| `max_continuous_points` | Memory/message-size guard per run; exceeding it rejects preflight rather than silently inserting stops |
| `clearance_mm`, `approach_duration_s` | Approach above the first position and lift at completion; no automatic HOME movement |
| `extrusion_update_period_s` | Preflight plans extrusion E(t) from robot trajectory timing and converts it to stepper pulses. During execution, `/stepper/speed` receives the next-period average feed rate, with gradual correction for host scheduling delays. Actual motor position and transport latency are not measured. Keep the UR speed slider at 100% |
| `pressure_advance_s` | Klipper `pressure_advance` K for depositing motions: the planned position is E + K * dE/dt. It cancels at the rest-to-rest end, so total E is exact; the extruder briefly reverses while decelerating. 0 keeps extrusion in sync with motion |
| `extrusion_move_margin_s` | E-only moves (retract/unretract) are sent to `/stepper/move` as `[signed steps, steps/s]` and counted by the firmware; the node waits the move duration plus this margin so transport jitter cannot truncate it |
| `ignored_commands` | Explicit list of externally managed commands; every encountered code is reported |
