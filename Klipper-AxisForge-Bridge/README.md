# Klipper–AxisForge Bridge

Home and probe the bed when it is placed or moved, upload G-code, run preflight,
then start a job from the local dashboard. Klipper
owns the XYZ/ABC/E timeline; the AxisForge robot follows timestamped joint
trajectories and Klipper drives extrusion. A/B/C are absolute degrees about
X/Y/Z relative to the configured nozzle orientation.

**Run preflight** verifies the identical file on both hosts, generates a Klipper
schedule, heats selected heaters to the maximum targets requested in the file, verifies temperature,
and switches all heater targets to zero **before** robot IK and timing checks.
Robot preflight does not move the arm. **Start synchronized job** reheats according
to the file, approaches its entry point, and executes prequeued timed runs.
The worker never publishes Portenta extrusion commands.

Enable `continuous_motion: True` in Klipper's `[axisforge_sync]` section to remove
the repeated two-second chunk handoffs. Before preflight, **Corner motion** offers
Klipper square corner velocity, Marlin Junction Deviation, a bounded geometric
blend, and the conservative stop-at-corners policy. The recommended blend starts
at 0.05 mm deviation; tight tolerances may require slower turns. Both hosts must
be updated, and changed options require preflight again.

Native Klipper pressure advance remains active for positive XY extrusion with
`kinematics:none`. The bridge preserves XYZ metadata and schedules nominal E;
Klipper applies advance and smoothing once. The file's `SET_PRESSURE_ADVANCE`
value must match the host. Selecting Marlin corner planning does not enable
Marlin Linear Advance. See [corner policies and pressure advance](docs/synchronized-jobs.md).

Independent speed override and mid-job resume are unsupported. Live tracking,
pressure-advance calibration and print quality still require physical testing.

## Setup

Use matching versions of this repository and
[AxisForge](https://github.com/CrispyButtermilkChickenSalad/AxisForge).
The AxisForge Docker workflow starts rosbridge, the robot worker and this dashboard.
Copy/clone this repository into `AxisForge/Klipper-AxisForge-Bridge` before building.
Set the actual Moonraker URL and enable the worker in AxisForge's `.env`:

```ini
MOONRAKER_URL=http://192.168.0.96:7125
AXISFORGE_SYNC_ENABLE=1
```

From AxisForge:

```bash
docker compose up -d --build dev
docker compose exec dev axisforge-build
docker compose --profile klipper up -d
```

When the Klipper host address changes, update `MOONRAKER_URL` and any explicit
printer UI URLs in AxisForge's `.env`. While both workflows are idle, recreate
the dashboard with `docker compose --profile klipper up -d --no-deps --force-recreate bridge`.
Restarting an existing container does not reload Compose environment values.
Verify the dashboard connection and run a fresh preflight before Start.

The UR driver must also be running. Start the pendant's External Control program
and set the speed slider to 100% before Start. Fresh six-joint feedback, NORMAL
safety mode and an available trajectory controller are required. A stopped
External Control program can still allow nonmoving preflight.

Select **Remote Control** in the pendant's top-right menu for the independent
dashboard Stop path. Enabling the feature in Settings alone does not select it;
the dashboard query `is in remote control` must return `true`. UR documents the
dashboard `stop` command as [Remote Control only](https://www.universal-robots.com/manuals/EN/HTML/SW5_25_1/Content/prod-dashboard/Dashboard_table.htm).

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
curl http://192.168.0.96:7125/printer/axisforge_sync/capabilities
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
export MOONRAKER_URL=http://192.168.0.96:7125
export AXISFORGE_GCODE_ROOT=/path/to/AxisForge/src/print_control/gcode
export AXISFORGE_CONFIG=/path/to/AxisForge/src/axisforge_bringup/config/ur_master.yaml
export AXISFORGE_SYNC_URL=http://127.0.0.1:8081
uvicorn bridge_dashboard.app:app --host 127.0.0.1 --port 8080
```

The Bridge and worker must share the same host monotonic clock (Linux host-network
containers do). Keep the worker's HTTP port on loopback; Moonraker controls access
to the Klipper endpoints. Open **http://127.0.0.1:8080**.

## Use and recovery

1. Use the first pipeline card, **Home & probe bed**, when the build surface is
   placed or moved. Follow its surface and front-left-point confirmations as
   described below. An existing valid bed map can still be used for an unchanged bed.
2. **Upload to both** and select the uploaded file.
3. Choose **Check extruder heater** and **Check bed heater** independently, then
   **Run preflight**. Only selected heaters are heated for testing; all targets
   are zero during the following robot checks. Failures explain the rejected line,
   calibration, timing limit, heater or connection.
4. **Start synchronized job** when both checks pass and the UR interlocks are
   ready. A result expires after 15 minutes, a file/configuration change or a run.

### Home and probe the bed with ALPS

The dashboard owns this routine and supervises its native AxisForge ROS process.
The ALPS load cell under the hotend detects bare-nozzle contact; the measured
robot nozzle TCP supplies height. Klipper does not command robot Z, so this flow
does not use Klipper `PROBE` or `BED_MESH_CALIBRATE` to move the arm.

Before pressing **Home & probe bed**, run the UR driver and External Control
program, select Remote Control and set the speed slider to 100%. The nozzle must
already be upright in the configured probing orientation, clean, cool (below
50 °C), clear of the surface and unloaded. Both heater targets must be zero.
The routine rejects a tilted starting pose; it makes no initial joint-space
approach or automatic rotation. Ensure the lift and travel area is clear.

The dashboard walks through these stages:

1. **Confirm the surface:** verify a flat build surface is under the nozzle and
   the nozzle is unloaded. The routine then records its unloaded sensor baseline
   and descends only in Z, at **0.5 mm/s** in **0.05 mm** steps. It stops on sustained
   contact or fails after **20 mm** of descent from the starting height.
2. **Repeat more slowly:** retract **3 mm**, verify the load clears, then approach
   again at **0.1 mm/s** in **0.01 mm** steps. The contacts must agree within the
   configured tolerance before accepting the homed height.
3. **Confirm the front-left mesh point:** lift to **10 mm above the original
   starting Z**, then travel in XY to the existing first grid point (G-code
   **X30 Y30** by default). Confirm that the nozzle is above the intended first
   point and that it is the front-leftmost point of the mesh.
4. **Measure and return:** automatically probe the nine-point serpentine grid,
   with three contacts per point and a default maximum Z spread of **0.05 mm**.
   Finish by lifting, returning in XY, and moving in Z to the saved starting nozzle
   pose. This uses the pose captured at the start, not a separate pendant Home pose.

Only a complete grid followed by a successful return replaces the active bed map.
Until then, readings remain in `bed_probe.pending.yaml`; the old map is preserved
and is backed up before replacement. Starting homing clears saved job preflight,
so run **Run preflight** again afterward. **Simulate routine** uses synthetic
sensor and robot data, follows the confirmations, and writes a separate result;
it neither changes the active map nor counts as completed physical homing.

**Stop probing** cancels motion. The process also stops on missing bridge
heartbeats, closed control input, expired confirmation, stale sensor/joint feedback,
excessive noise, ADC saturation/errors, excess load signal, heater activation,
another job taking control, failure to find contact, or failure to clear contact
after retraction. A cancelled or failed routine does not automatically return
through an unverified scene. Resolve the reported condition before restarting it.

The default profile is AxisForge's
[`src/axisforge_bringup/config/bridge_bed_probe.yaml`](https://github.com/CrispyButtermilkChickenSalad/AxisForge/blob/main/src/axisforge_bringup/config/bridge_bed_probe.yaml).
It uses **raw ADC counts**, with a contact delta of **20,000 counts** and a maximum
excursion of **60,000 counts** relative to the unloaded session baseline. Either
pressure polarity can trigger. No conversion to grams is needed, and the routine
does not modify Klipper's scale, tare reference, calibration flag or configuration.

[Mellow's FAQ](https://mellow.klipper.cn/en/docs/ProductDoc/ExtensionBoard/fly-alps/faq/)
gives `set 20000` as an example, not a factory default. Its
[configuration tool](https://mellow.klipper.cn/en/docs/ToolsDoc/fly-alps-tool/)
reads the connected device's threshold; `8000` is an example placeholder there.
The vendor's onboard filtered signal is not established to match the Klipper
ADS131M02 raw stream. These bridge values therefore need commissioning on the
actual assembly and are not verified force limits. The dashboard displays counts
as counts. `contact_offset_mm` provides a measured correction for nozzle/load-cell
deflection; zero does not compensate deflection at contact. This host/network loop
does not provide a hardware force guarantee.

The routine requires the existing ALPS bulk stream on the Moonraker host's port
**7126** (override with `ALPS_BRIDGE_URL`), plus its `load_cell_probe` Klipper object.
The native entry point is `ros2 run print_control bridge_bed_probe`, using
`--config` and optionally `--profile`. It is a supervised JSON protocol on stdin
with heartbeats and single-use stage/token confirmations, not an interactive
keyboard command; use the dashboard for normal operation. All motion processes
must share `AXISFORGE_ROBOT_LOCK`; the AxisForge Docker setup provides it.

The existing terminal load-cell script and calibrated grams mode remain available.
Grams mode requires known-load calibration; raw mode above does not. See
[AxisForge's probe and calibration instructions](https://github.com/CrispyButtermilkChickenSalad/AxisForge/blob/main/README.md#calibrated-grams-mode-and-terminal-probing).

### Synchronized job recovery

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
The default scheduling uncertainty and running alignment limits are both **10 ms**.
The **Clock timing** selector offers 5, 10, 15 and 20 ms alignment profiles with
network and print-speed guidance, plus bounded custom limits. The 15/20 ms profiles
retain a 10 ms scheduling limit; every measurement includes a 1 ms MCU allowance.
Changing limits requires a fresh preflight. The actual planned peak speed and
illustrative timing displacement are reported without changing the motion.
See [clock profiles and their assumptions](docs/synchronized-jobs.md#clock-timing-profiles).
Checks run before heating/approach, at run boundaries and every two seconds during
motion. A failure reports the measured uncertainty or alignment; check host/network
load and prefer wired networking. Raising a limit does not repair the connection.
A failed host check aborts Klipper without
sending an unknown-session abort to a robot that has not received this preflight.

Klipper and robot leases renew independently, so a delayed reply from one does
not block the other's renewal. Klipper heartbeats use a dedicated persistent
WebSocket with minimal acknowledgements. Heating and motion retain a 0.5 s reply
deadline and the configured strict watchdog (normally 1 s). While robot preflight
is non-actuating and all heater targets are off, a guarded 3 s Klipper planning
lease permits transport retries for at most 2 s. Strict control must be acknowledged
again before preflight passes or execution can start. Robot-worker deadlines stay
unchanged. See [heartbeat phases, diagnostics and deployment](docs/synchronized-jobs.md#heartbeats-during-preflight-and-printing).

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

## Setup before the G-code dashboard

The first tab is now **Setup & calibration**: connect the PC UR driver, follow the
manual arm-startup checklist, calibrate ALPS with a weighing scale, home/probe the
bed, and choose heater/motion/network options. Upload, preflight and synchronized
Start remain in **G-code dashboard**. Arm power, brakes, Remote selection and
program Load/Play have no dashboard controls.

**Start scale routine** asks you to move home manually and confirm scale placement,
finds contact with a slow Z-only descent, lifts 5 mm, and waits for scale taring.
Then use single-click Z baby steps to establish two measured loads. Record both
in **ALPS Logger → Calibration**. Use Up to unload before Finish. This routine
uses uncalibrated raw counts and never writes the logger/Klipper calibration or
bed mesh itself. Simulations need no hardware. Physical validation is still needed
with the nozzle, scale and robot connected.

In the parent AxisForge workspace, start the stack with
`docker compose --profile klipper up -d`. The passive `robot-connection` service
(port 8082, loopback only) launches the configured driver on **Connect robot**;
no Docker socket is exposed. `python3 scripts/open_dashboard.py --install` adds
a desktop menu entry and dashboard autostart after login. Klipper starts on its
own host. See the parent [setup and calibration guide](../README.md#dashboard-startup-and-scale-calibration)
for limits, configuration and boot behavior.

Additional APIs: `GET /api/robot/connection`, `POST /api/robot/connect`, and
`/api/load-calibration/{status,start,confirm,action,abort}`. Status is GET; the
other calibration routes are POST. Confirmation requires its current stage/token;
a step requires `{"command":"step","token":"<current token>","delta_mm":-0.01}`.
Actions are single-use and expire; they cannot be queued or replayed. All motion
entry points share the existing ownership and preflight interlocks.

### Nozzle TCP editor

Use **Setup & calibration → Nozzle TCP offset** to load and save AxisForge's
X/Y/Z nozzle offset in millimetres. It changes only `tcp_offset_mm` in the shared
AxisForge configuration; UR controller tool settings remain manual. Updates apply
on the next planner/probe/calibration run without a service restart. Save is blocked
during active operations and clears old preflight. Re-probe when the bed mesh was
measured with another TCP. The original configuration is retained in the adjacent
`.tcp-backup` file, and stale browser edits require an explicit reload.

`GET /api/setup/tcp` returns saved `tcp_offset_mm`, a configuration `revision`, and
bed-mesh TCP compatibility. `POST /api/setup/tcp` accepts finite numeric `x_mm`,
`y_mm`, `z_mm` (±1000 mm per axis) and that exact `revision`. It does not move the
robot or set the UR installation TCP. Hardware can be offline when saving.
