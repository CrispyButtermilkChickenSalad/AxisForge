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
MOONRAKER_URL=http://192.168.0.96:7125
ROBOT_IP=192.168.0.100
AXISFORGE_SYNC_ENABLE=1
```

```bash
docker compose up -d --build dev
docker compose exec dev axisforge-build
docker compose --profile klipper up -d
# Run the driver separately if it is not already running:
# Open http://localhost:8080 → Setup & calibration → Connect robot
# Complete power/brake/External Control startup manually using your UR controls.
```

Install the matching Klipper module on the **Klipper host**, following
[Bridge setup](Klipper-AxisForge-Bridge/README.md#klipper-host). It requires
`kinematics: none` and the current extra-axis/motion_queuing API. On this installation
Moonraker is `192.168.0.96:7125`; the UR robot is the separate `192.168.0.100` host.
The AxisForge PC (`192.168.0.10/24`), robot and Klipper Ethernet port share the
same switch. Klipper's persistent `AxisForge wired sync` network profile assigns
`192.168.0.96/24` without changing the default route or DNS. Its Wi-Fi address
`192.168.137.96` remains available for recovery; synchronization uses Ethernet.
The worker binds to loopback port 8081 and the dashboard to port 8080.

After changing the Klipper address, update `MOONRAKER_URL` and any explicit
`MAINSAIL_URL`, `FLUIDD_URL` or `PRINTER_UI_URL` in `.env`. With the synchronized
workflow and standalone AxisForge idle, apply the settings with
`docker compose --profile klipper up -d --no-deps --force-recreate bridge`.
A container restart alone does not reload Compose environment values. Confirm
Klipper is connected in the dashboard and run a fresh preflight before Start.

Open **http://127.0.0.1:8080**:

1. **Upload to both**; the uploaded file is selected automatically.
2. Select **Check extruder heater** and **Check bed heater** independently. Choose
   a **Corner motion** policy (Klipper SCV, Marlin JD, smooth geometric blend,
   or conservative stops), then
   choose **Clock timing** (default 10 ms; profiles include network and speed
   guidance, plus custom limits). **Run preflight**. Klipper verifies the file and generates the timeline. It
   tests selected heaters at G-code targets, then switches all
   targets to zero before robot IK and timing checks. The robot does not move.
3. Start the UR External Control program with speed scaling at **100%** and
   NORMAL safety mode. Select **Remote Control** on the pendant so the independent
   dashboard Stop command is available. Click **Start synchronized job** after both checks pass.
   Start reheats, approaches the entry point and executes prequeued timed runs.

A heater that is not configured defaults to unchecked. Its G-code commands are
ignored for that job with a warning, so an unheated bed can accept slicer files
containing M140/M190. For configured heaters, disabling a preflight check skips
only the test; Start still follows G-code temperatures. Off targets need no test.
Rerun preflight after changing heater, corner or clock options. Cold-extrusion limits remain active.
Clock profiles use an illustrative 0.10 mm timing-displacement allowance for speed
guidance; they do not change feed rates or certify a connection. See the
[clock timing guide](Klipper-AxisForge-Bridge/docs/synchronized-jobs.md#clock-timing-profiles).
Klipper heartbeat transport is independent of clock sampling. A guarded recovery
window applies only to robot preflight with heater targets off; heating and motion
retain strict supervision. Update the Bridge and Klipper module together for
[cold-preflight heartbeat support](Klipper-AxisForge-Bridge/docs/synchronized-jobs.md#heartbeats-during-preflight-and-printing).

Synchronized extrusion uses Klipper's native pressure advance and smoothing even
with `kinematics:none`; the master retains the XY metadata needed for PA. The
robot worker sends no extrusion or second PA correction. Standalone
`pressure_advance_s` is not used for this workflow, and choosing Marlin JD does
not switch to Marlin Linear Advance. A G-code `SET_PRESSURE_ADVANCE` value must
match the Klipper host before preflight.

Both systems must complete a run before the next begins. Missing heartbeats,
stale feedback, timing faults or controller errors stop the job and turn heaters
off. Aborting already queued extrusion shuts Klipper down; resolve the fault,
restart Klipper and run preflight again. Mid-job resume is not supported.

With `continuous_motion: True`, transport handoffs occur at G-code barriers
instead of every two seconds. Corner policies share one master timeline and
remain subject to actual joint interpolation checks. Smooth blending defaults to
0.05 mm path deviation; tighter rounding can take longer. Physical synchronization
accuracy, PA calibration and printing quality still need commissioning. The local integration
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

The bed is movable: probe it again every time it is placed. For an ALPS load cell,
use the dashboard's **Home & probe bed** step described below. Manual paper probing
also remains available: with the UR driver running (Terminal 1), jog the nozzle onto
a 3x3 grid (G-code X/Y = 30, 117.5, 205) and record each touch. Paper probing uses an
upright nozzle (roll 180°, pitch 0°), with the printing yaw and TCP from
`ur_master.yaml`; each point starts `clearance_mm` above `z_offset`, and jogs stop
3 mm below it.

```bash
cd /path/to/AxisForge   # bed_probe_file is relative to this directory
ros2 run print_control bed_probe \
  --config "$PWD/src/axisforge_bringup/config/ur_master.yaml"   # --dry-run to simulate
```

Keys: `w`/`s` Z up/down, `1`-`4` Z step 1/0.1/0.05/0.01 mm, `i`/`k` +Y/-Y and `j`/`l`
-X/+X in 10 mm steps (`--xy-step-mm`), space records (paper test; `--paper-mm` default 0.1
is subtracted), `n`/`p` next/previous point, `h` lift, `q` lift, save and quit. Ctrl-C
cancels the active trajectory. The result goes to `bed_probe_file` unless `--output` is given.

#### Guided ALPS homing and mesh from the dashboard

In **Setup & calibration**, the **Home & probe bed** card runs the homing and mesh routine
inside Klipper–AxisForge Bridge. AxisForge moves the arm and records nozzle TCP
height from fresh joint feedback; Klipper supplies ALPS samples. The bare nozzle
is the contact probe, so no paper thickness or separate XY probe offset is used.
Klipper `PROBE` and `BED_MESH_CALIBRATE` do not move the arm in this arrangement.

Start the UR driver and External Control program, with Remote Control selected and
the speed slider at 100%. Position the clean, cool nozzle **upright and unloaded**,
clear of the flat build surface. Both heater targets must be zero and the nozzle
must be below 50 °C. The routine checks the starting orientation and preserves it;
it does not make an initial joint-space approach or automatically rotate the tool.
Leave room for the subsequent lift and mesh travel before starting.

1. Press **Home & probe bed**. Confirm that the flat build surface is beneath the
   nozzle, and that the nozzle is clean, cool, clear of the surface and unloaded.
   This confirmation establishes the session's unloaded sensor baseline.
2. The arm descends continuously in **Z only**, accelerating to **10 mm/s**,
   stopping on sustained load detection. Descent is bounded to **20 mm** below
   the saved starting position; failure to find contact stops the routine.
3. It retracts **3 mm**, verifies that the load clears, and makes **two precision
   contacts at 0.2 mm/s**, retracting and checking for released load between them.
   Only these slow contacts determine bed height. They must agree within the
   selected repeatability tolerance; the fast touch is an approximate location.
4. It lifts vertically to **10 mm above the original starting Z**, then moves in
   XY to the existing first mesh point: G-code **X30 Y30** with the default grid,
   transformed using the configured bed origin and nozzle TCP. The dashboard waits
   for confirmation that this is the intended **front-leftmost mesh point**.
5. After that confirmation, it automatically measures the selected serpentine
   grid (default **3×3**, selectable **2–9 points per axis**) at **0.1 mm/s**. Each point uses **five contacts**; the default accepted Z
   spread is **0.02 mm**. The median measured contact is saved.
6. It lifts to a clear height, returns in XY, then moves in Z to the **saved starting
   nozzle position and orientation**. This returns to the pose captured at the
   beginning, not a separate pendant-defined Home pose.

Upward homing and mesh moves use **20 mm/s** by default, including the 0.5 mm
retraction between mesh contacts, the 3 mm homing retraction, and clearance lifts.
The dashboard can adjust this independently from downward probing (1–20 mm/s).
XY moves at clearance height inherit AxisForge’s `max_travel_speed_mm_s`, currently
**120 mm/s** in `ur_master.yaml`, instead of the old bridge-specific 5 mm/s.
Travel still accelerates/decelerates and respects joint limits; very short lifts
may finish before reaching the requested speed. A legacy `homing.travel_speed_mm_s`
can lower XY speed but cannot exceed the AxisForge limit.

Mesh X/Y counts are independent integers from **2 to 9** (4–81 points total).
Changing density preserves the saved bed dimensions, edge margin (30 mm by default)
and front-left start, with evenly spaced serpentine rows. The saved map includes its grid shape;
a denser grid takes longer because every point receives the selected repeat contacts.

Use the **Build surface** fields in the same card to set bed **width X**, **depth Y**,
and **probe edge margin**. A W×D bed with margin m probes X=m..W−m and Y=m..D−m
using the selected point counts. Zero margin reaches the physical edges; the
margin must be less than half the smaller dimension. Resize only to the actual
flat surface under the nozzle. Saving keeps the physical front-left corner fixed
by compensating the legacy center-mode X/Y offsets; the nozzle TCP is unchanged.
After resizing, run Home & probe bed and repeat preflight. Existing mesh files
are retained, but a map recorded for different dimensions is rejected for printing.

Both standalone and synchronized AxisForge preflight reject resolved G-code XY
outside **X=0..width, Y=0..depth**, including start purge lines, travel, and end
parking. Relative moves, inch units and G92 offsets are resolved before checking;
a configured bed map can impose tighter coverage limits. This is an XY build-area
check, in addition to existing robot reach, joint and path checks.

Each homing pass uses one trajectory with smooth acceleration/deceleration ramps
and a constant-speed middle at the selected speed. It does not stop at individual
waypoints or wait for settled samples during descent. ALPS is monitored throughout;
contact cancels the trajectory, and fresh stationary feedback is required before
recording height or retracting. Contact close to the starting position can happen
during acceleration. An infeasible path or joint limit rejects the pass instead
of silently slowing its cruise. Mesh approaches also use continuous, slow motion;
settling and repeated measurements occur at contact. Scale calibration retains
its **0.1 mm/s** stepped contact search. Legacy `first_step_mm` and
`second_step_mm` fields are accepted but are not used by continuous homing.

Use **Setup & calibration → Home & probe bed → Homing and mesh settings** to save coarse
search speed, precision rehoming speed, upward speed, mesh speed, X/Y point counts,
readings per mesh point, and repeatability tolerance. Settings can be edited with hardware disconnected and
apply on the next run. Saving is blocked during robot operations and invalidates
job preflight. Revision checking prevents one browser from overwriting another's
changes. Reload after a conflict. The profile is `bridge_bed_probe.yaml`; its
acceleration and safety limits remain available for deliberate file-based tuning.
Lower speed and repeatable readings reduce measurement variation; the repeatability
setting does not certify absolute accuracy or compensate hotend deflection.

Continuous homing rejects latest ALPS samples older than **150 ms**, uses a
**200 ms** action-cancellation deadline with independent UR Stop fallback, and
rejects observed travel beyond **0.5 mm after host detection**. These are software
rejection limits, not verified physical stopping distances or force limits. The
observed ALPS delivery is about 100 ms per batch: at 10 mm/s the nozzle can travel
about 1 mm before a contact report arrives, plus stopping travel. Commission actual
contact and stopping distance under supervision. Different pass speeds can affect
post-stop deflection and repeatability. Stop acknowledgement time and observed
post-detection travel are recorded with the homing readings.

The active bed map is replaced only after the entire grid **and the return move**
succeed. Partial results remain in `bed_probe.pending.yaml`; the previous map is
backed up as `bed_probe.yaml.<timestamp>.bak`. The file records the homing contacts,
repeated mesh readings, their units and the session baseline. Starting the routine
clears saved job preflight: run **Run preflight** again before synchronized Start.
An existing valid bed map can still be used when the bed has not moved.

**Stop probing** cancels the active trajectory. A missing bridge heartbeat, closed
control pipe, expired confirmation, stale samples, excessive noise, ADC saturation
or errors, excessive signal excursion, heater activation, a competing job, or a
failure to clear contact stops the routine. Abort and fault paths do not attempt
an automatic return through an unverified scene. **Simulate routine** exercises the
same confirmation flow with synthetic sensor and robot data; its separate output
does not replace the active map or count as completed physical homing.

The default native profile is
[`bridge_bed_probe.yaml`](src/axisforge_bringup/config/bridge_bed_probe.yaml). It uses
**raw ADC-count changes**, so grams calibration is not required: contact is a
sustained absolute change of **20,000 counts** from the unloaded baseline, with a
maximum excursion of **60,000 counts**. Both pressure polarities are accepted.
The baseline belongs to this session; the routine does not change Klipper's
`counts_per_gram`, reference tare, calibration flag or printer configuration.

These numbers are configurable commissioning values. [Mellow's ALPS FAQ](https://mellow.klipper.cn/en/docs/ProductDoc/ExtensionBoard/fly-alps/faq/)
uses `set 20000` as an example, not a factory default. The
[Mellow configuration tool](https://mellow.klipper.cn/en/docs/ToolsDoc/fly-alps-tool/)
reads the connected device's setting and displays `8000` as an example placeholder.
Its onboard filtered signal is not established to be equivalent to the raw Klipper
ADS131M02 stream, so neither example supplies a verified force limit for this arm.
Check sensor response and tune the profile for the actual assembly. Counts are
shown as counts, not grams. `contact_offset_mm` adds a measured correction for
hotend/load-cell deflection; zero does not compensate contact-force deflection.
The host/network feedback loop is not a hardware force limiter.

The dashboard supervises the ROS entry point `ros2 run print_control bridge_bed_probe`
with `--config` and the adjacent default profile; `--profile` selects another native
profile. This entry point expects newline-delimited JSON commands on stdin,
including heartbeats and stage/token confirmations, and emits `AXISFORGE_PROBE`
status events. It is not a keyboard-interactive terminal script. Use the dashboard
for normal operation rather than launching another instance beside it.

#### Calibrated grams mode and terminal probing

The existing terminal helper remains available with
[`load_cell_bed_probe.yaml`](src/axisforge_bringup/config/load_cell_bed_probe.yaml).
Its default **grams mode** requires a known-load calibration. The configured
`counts_per_gram: 138` alone is not proof of calibration. With the robot stationary,
the nozzle upright, cool and unloaded, and both heater targets off, use the Klipper
console:

1. Run `LOAD_CELL_CALIBRATE LOAD_CELL=load_cell_probe`.
2. Remove external load from the nozzle and run `TARE`.
3. Apply a known, measured load through the nozzle/load-cell load path, within the
   assembly's rating. Run `CALIBRATE GRAMS=<actual known grams>` while the load is
   steady. Enter the actual number, not the placeholder. This host's calibration
   tool requires at least 50 g and checks that the ADC change is large enough.
   Never use robot pressure against the bed as an unknown calibration load.
4. Only after calibration succeeds, run `ACCEPT`, remove the load, then run
   `SAVE_CONFIG`. This saves the scale/reference and restarts Klipper; wait for Ready.
   If calibration fails, run `ABORT` and resolve it before trying again.

The commands above match the installed Klipper module. See also
[Klipper load-cell documentation](https://www.klipper3d.org/Load_Cell.html).
Grams mode checks both `is_calibrated` and the reference/scale before creating a
robot controller. Calibration should use the same orientation and hotend assembly
as probing. Raw mode in the guided dashboard routine does not require this
conversion to grams.

From the AxisForge root, with the existing Docker `dev` service running:

```bash
# Read the real sensor, including before calibration; no robot connection/motion.
./scripts/bed_probe_load_cell.sh --sensor-check

# Exercise all nine points with a synthetic sensor and simulated robot.
./scripts/bed_probe_load_cell.sh --dry-run --automatic \
  --output /workspace/.docker/bed_probe.simulated.yaml

# Calibrated grams mode: space/t probes a point; a scans all nine.
./scripts/bed_probe_load_cell.sh

# Alternatively, immediately start the entire physical probing sequence:
./scripts/bed_probe_load_cell.sh --automatic
```

For physical terminal probing, start the UR driver's External Control program and
position the upright nozzle in a clear approach area. Confirm `z_offset` and `clearance_mm`
place the hover grid above the whole bed: the initial approach is a joint-space move.
The existing X/Y origin, nozzle TCP, upright orientation and 3x3 grid remain in use.
Keep the nozzle clean and below 50 °C, with heater targets at zero. Quit with `q`
to lift, or Ctrl-C to cancel without a further move. Manual downward/XY jogging is
disabled in load-cell mode. Original paper probing remains available with
`ros2 run print_control bed_probe --mode paper ...`.

The helper reads `MOONRAKER_URL` and optional `MOONRAKER_API_KEY` from the root `.env`
or its process environment. The guided dashboard routine uses the bridge's configured
Moonraker address. The ALPS bulk bridge must run on the same host, port **7126**
(`--alps-url` or `ALPS_BRIDGE_URL` overrides it). For this wired installation those
are `http://192.168.0.96:7125` and `http://192.168.0.96:7126`. The sensor-check command
verifies the actual sample stream. There is no fallback to the logger's averaged
Moonraker readings. On native ROS, run from the same workspace root and use
`ros2 run print_control bed_probe --mode load-cell --config "$PWD/src/axisforge_bringup/config/ur_master.yaml"`.
All controllers must share the same `AXISFORGE_ROBOT_LOCK` file; Docker sets this
to `/workspace/.axisforge-robot-control.lock`. Restart `sync-worker` and `bridge`
after upgrading, while idle, to load their probing-ownership checks.

The terminal grams-mode defaults are an initial contact threshold **50 g**,
maximum observed load **200 g**, **0.02 mm** steps at **0.2 mm/s**, **0.5 mm** retract,
and three touches with at most **0.05 mm** Z spread.
These are commissioning settings, not a measured load rating for the hotend. Verify
the calibrated force response and assembly limits before the first physical probe.
Detection cancels the current step, confirms action termination, checks sustained
contact, records measured TCP position, and retracts before another touch. Loss of
samples, ADC errors, overload, heater activation, a competing Klipper job, failure
to clear contact, or failure to find the bed before `--floor-mm` stops the session.
This host/network feedback loop is not a hardware force limiter; step size and
speed bound the commanded contact motion, and latency/compliance still matter.

The terminal helper also saves partial measurements to `bed_probe.pending.yaml`
and promotes a complete nine-point map with a backup. It retains its original
hover/approach workflow rather than the dashboard's two-confirmation homing and
return sequence. Re-run synchronized preflight after changing the bed map. Dry
runs write a separate file, and printing rejects simulated or explicitly incomplete
maps.


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

### Dashboard startup and scale calibration

Open <http://localhost:8080>. **Setup & calibration** is the first tab; the G-code
upload/preflight/start workflow is in the next tab. Complete these steps:

1. Press **Connect robot** to launch the existing PC-side UR driver. It detects an
   existing ROS driver instead of launching a duplicate. Readiness checks show
   Ethernet/joint feedback, Remote Control, arm state, External Control, speed
   scaling, and the saved AxisForge nozzle offset. Power-on, brake release, mode
   selection, program Load/Play, installation TCP and payload remain manual.
2. For ALPS calibration, press **Start scale routine**. Manually move to a home
   pose with an upright, clean, cool nozzle, then confirm. Place the weighing
   scale on the buildplate below the nozzle with a small unloaded gap (≤20 mm),
   then confirm separately. The routine descends along base Z at **0.1 mm/s** in
   **0.01 mm** steps, stops on raw ALPS contact, and lifts **5 mm** at 0.5 mm/s.
3. Tare the weighing scale and confirm. Use **Up / Down** for individual
   **10, 1, 0.1 or 0.05 mm** Z steps; 0.5, 0.01 and 0.001 mm are also available.
   Downward steps larger than 0.05 mm must end at least 0.5 mm above detected
   contact. Travel is bounded to **25 mm above contact and 0.25 mm below it**.
   Coarse clearance moves use 1 mm/s and stop if unexpected contact is detected;
   fine/load-release moves use 0.1 mm/s. Rotation and XY remain
   fixed. Obtain two distinct, stable scale readings and enter each measured
   weight in **ALPS Logger → Calibration** alongside its sensor reading. Return
   to Setup to adjust the second load. Do not tare ALPS while loaded.
4. Use **Up** to release the scale, then **Finish — scale unloaded**. Finish
   holds the current pose and does not return home automatically. Remove the
   scale, use **Home & probe bed**, choose job settings, then continue to the
   G-code dashboard. Calibration/homing invalidate old job preflight results.

The scale routine uses raw counts without declaring the sensor calibrated. Its
contact threshold is `max(8000, ceil(4.1 × unloaded peak-to-peak noise))`, capped
at 20000 counts; excess noise rejects the routine before movement. Five successive
samples confirm contact and 60000 counts is the independent stop limit. See
[`alps_load_calibration.yaml`](src/axisforge_bringup/config/alps_load_calibration.yaml).
These are commissioning values, not a vendor factory default or known gram loads.
Mellow's [tool](https://mellow.klipper.cn/en/docs/ToolsDoc/fly-alps-tool/) and
[FAQ](https://mellow.klipper.cn/en/docs/ProductDoc/ExtensionBoard/fly-alps/faq/)
describe threshold tuning; the onboard filtered signal differs from the raw
Klipper ADC stream used here. The routine never writes ALPS/Klipper calibration
or the bed map. Saving in the logger is a separate operator action; verify the
calibration required by any later grams-based Klipper workflow independently.

The existing sensor/robot watchdogs, heater-off checks, process lock and independent
stop fallback apply to calibration. There is no queued jogging or automatic retry
of motion. Abort stops without a return move. **Simulate scale routine** exercises
the confirmations and synthetic contact without contacting either device.

### Nozzle orientation and pendant settings

The saved AxisForge nominal orientation is **roll 180°, pitch 0°, yaw 90°** in its
robot Base frame. G-code `A=0 B=0 C=0` means no rotation added to that orientation;
it does not mean all robot joints, or all pendant TCP angles, are zero. Guided
probing rejects a tilted starting tool and preserves the measured near-nominal
orientation throughout its Z moves. Use the pendant for initial XY positioning,
upright orientation, and the robot installation's measured TCP/payload setup.

UR's default orientation fields use a rotation vector (axis-angle, radians),
while AxisForge uses roll/pitch/yaw in degrees. The pendant's selected feature and
active TCP also affect its displayed pose. Do not copy AxisForge's degree values
into rotation-vector fields or zero the installation TCP. See the official
[UR Pose Editor documentation](https://www.universal-robots.com/manuals/EN/HTML/SW5_21/Content/prod-usr-man/complianceUR3e/SW_sections/first_program/pose_editor.htm).

The `robot-connection` container supervises only the PC driver, separate from
`bridge`, so refreshing/restarting the web dashboard does not shut down that driver.
Its HTTP service binds to loopback port 8082. It starts passively at boot and only
launches the UR driver after **Connect robot**; it never enables headless arm startup.
The Docker daemon must start at boot. After the normal build, create the services:

```bash
docker compose --profile klipper up -d
python3 scripts/open_dashboard.py --install
```

Containers use `restart: unless-stopped`; explicitly stopped containers remain
stopped until started again. The installer adds **AxisForge Bridge** to the desktop
application menu and opens the dashboard after desktop login (waiting up to 90 s
for the local service). Remove `~/.config/autostart/axisforge-bridge.desktop` to
turn off browser autostart. Klipper/Moonraker and its ALPS stream service run on the
printer host independently; no PC-side SSH/startup script is required. Verify their
host services after reconnecting Ethernet. Real nozzle/scale contact, the saved TCP,
and safe physical travel require a supervised hardware commissioning run.

### Edit the AxisForge nozzle TCP from the dashboard

In **Setup & calibration → Nozzle TCP offset**, edit X, Y and Z in **millimetres**
and press **Save nozzle TCP**. These describe the nozzle tip relative to the
AxisForge model's **tool0** frame, not robot Base coordinates. Existing values are
loaded from `gcode_interpreter.ros__parameters.tcp_offset_mm` in `AXISFORGE_CONFIG`
(`src/axisforge_bringup/config/ur_master.yaml` in the Docker stack).

Saving works with hardware offline and makes no robot command. Printing, homing,
and ALPS scale calibration read the new values on their next run; no driver or
worker restart is required. UR controller TCP, orientation, payload and installation
settings remain manual. The editor accepts finite values from −1000 to +1000 mm
per axis; these input limits do not establish a safe physical tool or workspace.

The dashboard blocks saving during jobs, preflight, homing, load calibration and
driver startup. The shared robot lock also protects controllers launched outside
the dashboard. A successful change clears saved preflight. A bed mesh made with a
different TCP is preserved but rejected by the planner: use **Home & probe bed**
again before printing. The TCP card reports whether the mesh's TCP matches.

The writer changes only the TCP list, preserves other YAML settings and comments,
and atomically replaces the file. The previous full configuration is kept beside
it as `ur_master.yaml.tcp-backup` (the latest change only). **Reload saved values**
discards form edits. If another tab/editor changed the configuration, or the save
response was lost, reload and review before saving again. YAML configurations with
anchors/aliases must be made explicit before using this editor.
