# Klipper–AxisForge Bridge

Upload G-code, run preflight, then start a job from the local dashboard. Klipper
owns the XYZ/ABC/E timeline; the AxisForge robot follows timestamped joint
trajectories and Klipper drives extrusion. A/B/C are absolute degrees about
X/Y/Z relative to the configured nozzle orientation.

**Run preflight** verifies the identical file on both hosts, generates a Klipper
schedule, heats selected heaters to the maximum targets requested in the file, verifies temperature,
and switches all heater targets to zero **before** robot IK and timing checks.
Robot preflight does not move the arm. **Start synchronized job** reheats according
to the file, approaches its entry point, and executes acknowledged timed chunks.
The worker never publishes Portenta extrusion commands.

This first implementation stops at each source move and waits between chunks.
It does not provide continuous lookahead, independent speed override, or mid-job
resume. Live timing and print quality still require commissioning on the physical
cell. See [protocol, limits and fault behavior](docs/synchronized-jobs.md).

## Setup

Use matching versions of this repository and
[AxisForge](https://github.com/CrispyButtermilkChickenSalad/AxisForge).
The AxisForge Docker workflow starts rosbridge, the robot worker and this dashboard.
Copy/clone this repository into `AxisForge/Klipper-AxisForge-Bridge` before building.
Set the actual Moonraker URL and enable the worker in AxisForge's `.env`:

```ini
MOONRAKER_URL=http://192.168.137.66:7125
AXISFORGE_SYNC_ENABLE=1
```

From AxisForge:

```bash
docker compose up -d --build dev
docker compose exec dev axisforge-build
docker compose --profile klipper up -d
```

The UR driver must also be running. Start the pendant's External Control program
and set the speed slider to 100% before Start. Fresh six-joint feedback, NORMAL
safety mode and an available trajectory controller are required. A stopped
External Control program can still allow nonmoving preflight.

### Klipper host

The new master requires `kinematics: none` and Klipper's `motion_queuing`/extra-axis
API (integration checked against `0cee841e31d701595b3e509a0b502f6e320f52a8`).
It is not a drop-in module for older toolhead APIs or Cartesian printers.
Copy `scripts/install_sync.py` and `klipper_extras/` to the Klipper host, keeping
the directory structure, then run there:

```bash
python3 scripts/install_sync.py \
  --klipper-dir ~/klipper --config ~/printer_data/config/printer.cfg
```

The installer backs up existing files and adds `[axisforge_sync]` before
`SAVE_CONFIG`. Existing sync settings are preserved. With no active print or
queued movement and heater targets zero, restart the Klipper service with
`sudo systemctl restart klipper`. A G-code RESTART alone may keep old Python
modules loaded when upgrading an existing installation. Verify:

```bash
curl http://192.168.137.66:7125/printer/axisforge_sync/capabilities
```

Expect `protocol: axisforge.sync.v1` and `control_ready: true`.
`robot_motion_bridge` is an optional read-only exporter; it is not used by this
workflow. The original file must not be launched separately through Fluidd/Mainsail.

### Native ROS installation

Build AxisForge and source ROS Humble and its workspace. From the AxisForge root:

```bash
ros2 run print_control axisforge_sync_worker \
  --config src/axisforge_bringup/config/ur_master.yaml --enable-motion
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=127.0.0.1
```

Run those long-lived services in separate terminals. In this Bridge checkout,
install `requirements.txt` into an environment that can also launch AxisForge:

```bash
export MOONRAKER_URL=http://192.168.137.66:7125
export AXISFORGE_GCODE_ROOT=/path/to/AxisForge/src/print_control/gcode
export AXISFORGE_CONFIG=/path/to/AxisForge/src/axisforge_bringup/config/ur_master.yaml
export AXISFORGE_SYNC_URL=http://127.0.0.1:8081
uvicorn bridge_dashboard.app:app --host 127.0.0.1 --port 8080
```

The Bridge and worker must share the same host monotonic clock (Linux host-network
containers do). Keep the worker's HTTP port on loopback; Moonraker controls access
to the Klipper endpoints. Open **http://127.0.0.1:8080**.

## Use and recovery

1. **Upload to both** and select the uploaded file.
2. Choose **Check extruder heater** and **Check bed heater** independently, then
   **Run preflight**. Only selected heaters are heated for testing; all targets
   are zero during the following robot checks. Failures explain the rejected line,
   calibration, timing limit, heater or connection.
3. **Start synchronized job** when both checks pass and the UR interlocks are
   ready. A result expires after 15 minutes, a file/configuration change or a run.

**Abort both** is available during preflight, heating, approach and execution.
A fault during committed extrusion shuts Klipper down to stop queued MCU steps.
After resolving the cause, restart Klipper and rerun preflight. Pause/resume and
changing speed during execution are unsupported; external PAUSE/CANCEL aborts.

Heaters configured in Klipper default to checked; unconfigured heaters default
to unchecked. The choices are remembered in this browser. A 0 °C/off command or
no heater command is reported as skipped and never requires warming or cooling.

For a configured heater, unchecking skips only its preflight heat test: **Start
still follows its G-code temperature commands**. If a heater is not configured,
unchecking explicitly ignores its commands for this job, with a visible warning;
this permits bed commands in slicer output on an unheated-bed setup. Checked,
unconfigured heaters with positive targets still fail. Temperature limits and
cold-extrusion protection remain active. Changing a choice requires a matching
preflight before Start. Update both the dashboard and Klipper extra together.

If robot timing validation fails, reduce `max_velocity` / `max_accel` or the angular
limits in `[axisforge_sync]`, restart idle Klipper, and rerun preflight. AxisForge
never stretches the master schedule to hide an infeasible path.

Clock checks use an authenticated persistent Moonraker WebSocket, taking at least
five samples when time permits and retrying jitter for up to 20 samples / 1.5 s.
The best round trip must still establish uncertainty of at most **10 ms**, including
the 1 ms MCU estimation allowance. Checks run before heating/approach and at every
stopped chunk boundary. A failure reports the measured uncertainty and round trip;
check the host/network load, prefer wired networking, and rerun preflight. Increasing
the timing limit is not a recovery step. A failed host check aborts Klipper without
sending an unknown-session abort to a robot that has not received this preflight.

Klipper and robot leases renew independently, so a delayed reply from one does
not block the other's renewal. A heartbeat response timeout of 0.5 s aborts both;
the participants' watchdogs remain active if the Bridge itself becomes unavailable.

Fluidd/Mainsail and ALPS Logger remain accessible through the tabs. Embedded ALPS
uses browser local-network/serial permission; allow the local dashboard to reach
the printer host, or open the logger in a separate tab.

## Tests

```bash
python -m unittest discover -s tests -v
```

The suite includes the actual Klipper module with fake heaters/MCU queues, the
async workflow, clock mapping, cancellation, failures and API gates. Robot tests
live in AxisForge. Automated tests do not establish physical synchronization
accuracy. Never use the dashboard in place of the physical emergency stop.
