# Dashboard setup and job guide

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
Marlin Linear Advance. See [corner policies and pressure advance](synchronized-jobs.md).

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

In **Setup & calibration**, **Redo homing & mesh** starts a fresh scan even when
a saved mesh exists. The current map stays active until all new points are
measured and the arm has returned successfully; the replacement keeps a timestamped
backup. A failed or cancelled run does not replace the active map. Each new run
requires fresh surface and first-point confirmations and clears job preflight.

For a mesh at printing temperature, enable **Probe at printing temperatures**,
set the nozzle/bed targets in Fluidd or Mainsail, and click **Use current Klipper
targets** (0 leaves that heater off). Select a soak (5–1800 seconds; default 60), then start homing. The routine
waits for the selected temperatures to remain within ±2°C for the soak before
motion and unloaded sensor taring. It monitors temperatures throughout and saves
the measurement temperatures with the map. Heater targets remain under manual
Klipper control after completion or Stop. A bed heater must be configured in
Klipper to select a nonzero bed target. See [native probe setup](native-probe.md)
for native module update requirements.

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

### Bed dimensions and reusable G-code filenames

In **Setup & calibration → Home & probe bed → Build surface**, set width X, depth Y
and probe edge margin. The selected 2–9 by 2–9 grid spans X=margin..width−margin
and Y=margin..depth−margin. The defaults remain **235×235 mm with 30 mm margins**.
Zero margin is allowed; otherwise it must be less than half the smaller dimension.
The native homing routine anchors the mesh centre at the nozzle's starting XY.
Bed direction sets grid rotation about that centre; at 0° the bed follows robot
Base +X/+Y. Resizing updates coverage around the next homing centre. Legacy maps
keep their existing offset convention until replaced by a complete new mesh. Saving clears job
preflight. Re-probe after resizing: a map recorded for different dimensions is
rejected. The previous map and configuration backup are kept.

AxisForge checks all resolved XY moves against **0..width and 0..depth** in both
standalone and synchronized preflight. This includes purge lines and parking;
relative coordinates, G92 offsets and inch units cannot bypass the check. Bed-mesh
coverage and robot reach/joint checks also apply.

Every upload gets a new version name, such as `part--a1b2c3d4e5f6.gcode`, on both
hosts. Re-uploading `part.gcode` preserves earlier versions and never reuses their
preflight results. To use a file already in AxisForge, select it under **G-code
dashboard → Job preflight**, press **Copy selected file to Klipper**, then run
preflight on the automatically selected new version. This copies the original
bytes unchanged and does not start a print. A failed upload removes only the new
local copy; its source remains.

### Start/end G-code compatibility

An AxisForge-only dry run is **not** a Klipper compatibility certificate. The
synchronized workflow compiles the file using this bridge’s defined command set,
verifies identical file hashes, and then checks the timed plan against AxisForge.
Use **Start synchronized job** in this dashboard after both preflights; uploading
a file does not make ordinary Fluidd/Mainsail printing coordinate the robot.

The synchronized parser supports linear G0/G1 XYZ/ABC/E, G20/G21, G90/G91, G92,
M82/M83, M220/M221, G4/M400, primary-fan M106/M107, T0 and explicit heater
M104/M109/M140/M190. An initial G28 is replaced by the robot entry approach, with
a warning; it does not perform ALPS homing or call Klipper’s Cartesian homing.
Bare M84 does not release the robot’s brakes or disable robot joints.
`SET_PRESSURE_ADVANCE ADVANCE=…` must match Klipper’s configured value.

The synchronized parser currently rejects **M105**, **M204**, axis-qualified
**M84 X Y E**, arcs, and arbitrary macros including **START_PRINT/END_PRINT**.
Klipper itself supports more commands and configured macros; this stricter parser
must know every operation before it can synchronize it. See the official
[Klipper commands](https://www.klipper3d.org/G-Codes.html) and
[slicer guidance](https://www.klipper3d.org/Slicers.html) for that distinction.

For this workflow, use explicit supported heater and fan commands in the slicer’s
start/end blocks. Remove redundant M105 temperature-query lines. Configure speed,
acceleration and corner policy through the bridge/Klipper settings instead of
slicer M204 commands. Omit axis-specific stepper-disable lines; keep robot power
and program control manual. Do not blindly remove or ignore unknown macros: review
their homing, offsets, purge, heater and shutdown effects first. Keep purge and park
coordinates inside both the configured bed and the measured mesh coverage.

The offline audit of the bundled files found most Cura examples first fail on
M105; their Ender-3 end block also contains `M84 X Y E`. Their X0.1 purge and
X0/Y235 park positions are outside the current mesh’s 30 mm edge margin.
`primitive_test.gcode` first fails on M204. The two angle-calibration matrix files
and `sync-validation-no-extrusion.gcode` passed the offline dialect test with test
heater limits; live heater, extrusion, clock and robot preflight are still required.
The audit did not modify any bundled G-code.

### Home and probe the bed with ALPS

For a nozzle-contact mesh, `contact_offset_mm` and `bed_z_adjust_mm` default to
zero. Print Z is **measured bed height + G-code Z + bed_z_adjust_mm**. The old
flat-bed `z_offset` is not added when a mesh is loaded, and the synchronized
planner preserves the file's layer heights. A load cell triggers at finite force:
toolhead/bed deflection can still make the measured surface lower than unloaded
first contact. Zero configured offset does not remove that mechanical effect.

With `post_print_lift_mm: 10.0` and `return_to_homing_pose: false` in AxisForge's
parameters, successful synchronized prints turn heaters off and lift the nozzle
10 mm in robot Base +Z from the final print point, holding X/Y and orientation.
Preflight checks the lift against the robot model and controller safety limits;
the worker rechecks a path from fresh feedback before executing it. Heartbeat
ownership continues until the endpoint is verified. Abort/failure keeps the arm
stopped. The lift does not depend on the saved Home height and does not change
bed-homing or mesh-failure recovery. Set the lift to zero to disable it. The
older saved-Home return remains available only with the lift disabled.

The synchronized parser accepts `SET_VELOCITY_LIMIT ACCEL=<positive number>`
before the first XY entry move. It caps that request at `[axisforge_sync]`
`max_accel` and uses the effective acceleration for both robot and extrusion
timing. Larger file values produce a preflight warning; they do not raise machine
limits. Other parameters and mid-print changes remain explicitly unsupported.

Repeated unchanged `M106`/`M107` settings no longer split a continuous run. Actual
fan changes and explicit synchronization barriers still do. Native Klipper
pressure advance remains active; the bridge does not add a second PA correction.
The timing panel reports measured nozzle position/Z error and estimated motion
lag from timestamped joint feedback during printing. These measurements are
separate from network clock alignment and do not measure filament flow or nozzle
deflection. Use a new print's measurements before changing timing compensation
or pressure advance; increasing the clock limit does not correct mechanical lag.

Clock synchronization accounts for both the starting offset and the rate of
Klipper's MCU print clock, including Linux's RAW/monotonic clock conversion.
The timing panel shows the measured rate in parts per million. Each robot run's
times and velocities are converted before it is queued; the running mapping
stays fixed, and unexpected phase drift still stops both participants at the
selected alignment limit. This avoids accumulating normal oscillator drift as
an apparent alignment fault during long continuous runs.

The dashboard owns this routine and supervises its native AxisForge ROS process.
The ALPS load cell under the hotend detects bare-nozzle contact; the measured
robot nozzle TCP supplies height. Klipper does not command robot Z, so this flow
does not use Klipper `PROBE` or `BED_MESH_CALIBRATE` to move the arm.

An optional [Klipper MCU contact source](native-probe.md) is staged for
calibrated load-cell probes. The existing streamed source remains the default;
installing the module does not start motion. Native 200 g contact detection needs
a verified scale and tare reference before activation, and still relies on the
PC and UR controller to stop the arm.

Before pressing **Home & probe bed**, run the UR driver and External Control
program, select Remote Control and set the speed slider to 100%. The nozzle must
be within 0.5° of the configured probing orientation, clean, clear of the
surface and unloaded. Cold probing requires a nozzle at or below 50 °C and zero
heater targets; heated probing uses the explicit temperature selection above.
After surface confirmation, the routine corrects a small tilt around the nozzle
tip over at least 3 seconds, before enabling Z positioning. The full planned path
must pass the UR controller safety checks and keep the tip within 0.0025 mm of
its starting position. Fresh stopped feedback must verify an upright residual
of at most 0.05° and an unloaded sensor before homing can proceed. Larger tilts,
sensor faults and failed corrections stop the routine. Ensure the toolhead has
room to rotate and the lift and travel area is clear.

The dashboard walks through these stages:

1. **Confirm the surface:** position the nozzle above the **centre of the bed**;
   this starting XY defines the bed centre. Verify a flat build surface is under the nozzle and
   the nozzle is unloaded. If alignment is needed, the confirmation shows its
   angle and explicitly authorizes it. The routine records its unloaded sensor
   baseline, aligns upright, then enables **Up / Down 10, 1, 0.1 or 0.05 mm** positioning. Every downward
   positioning step stops on contact. Choose **Continue — start Z homing** to
   capture the adjusted starting pose and descend continuously in Z, accelerating
   to **5 mm/s** by default. It stops on sustained
   contact or stops at the saved **Maximum Z search** distance below the starting
   height (default **350 mm**, adjustable **5–350 mm**).
2. **Precision rehoming:** retract **3 mm**, verify the load clears, then make one
   precision approach at the saved fine speed (**0.5 mm/s in the current profile**), followed by another retraction. Within
   0.3 mm above the approximate surface, use landings of at most **0.01 mm**,
   each ending at zero commanded speed. A contact during a landing permits only
   that already executing, bounded segment to finish; no following step starts.
   Each segment must fit within 75% of the contact stop deadline. Overload,
   stale feedback, excess stop travel or a missed deadline still cancels immediately.
   This avoids the UR controller's cancellation hold target being taken from a
   noisy instantaneous encoder sample and releasing the contact. This fine contact
   determines the homed height; the
   fast first contact is used only to locate the surface approximately. Its
   trigger remains a locator if load relaxes after the confirmed stop; it still
   requires fresh stationary feedback and all overload/stop-travel checks. Only
   the fine contact and mesh contacts require sustained load for a height measurement.
3. **Confirm the front-left mesh point:** lift to **10 mm above the original
   starting Z**, then travel in XY to the first grid point (G-code
   **X30 Y30** with a 30 mm edge margin). Its offset from the homed centre is
   `(margin − width/2, margin − depth/2)`, rotated by **Bed direction**.
   For a 220×220 mm bed and 30 mm margin at 0°, this is **−80, −80 mm**.
   A 200×200 mm bed has corners at ±100 mm; a point at −50, −50 has 50 mm margins.
   Confirm that the nozzle is above the intended first
   point and that it is the front-leftmost point of the mesh.
4. **Measure and return:** automatically probe the selected serpentine grid (default **3×3**),
   at the saved mesh speed and contact count (**0.5 mm/s and two per point** in the current profile), with a maximum Z spread
   of **0.02 mm**. Each new point uses **0.1 mm near-contact search steps**, with
   immediate cancellation on load, then retracts and takes the configured **0.01 mm
   precision contacts**. The locating touch never counts toward repeatability.
   Mesh XY travel uses **3 mm clearance above the highest measured surface**;
   the fine rehoming backoff and first-point confirmation height stay separate.
   Save the median measured contact.
   Finish by lifting, returning in XY, and moving in Z to the saved starting nozzle
   pose. This uses the corrected pose after alignment and any initial Z positioning,
   not a separate pendant Home pose.

Probe callbacks use a consistent 2 ms ROS polling interval. On ROS Humble,
alternating polling timeouts can reset the ready-callback queue and starve
speed or action feedback behind the 500 Hz joint stream. Freshness deadlines
remain unchanged; a delayed stationary observation may be replaced only
by an entirely fresh window within the original settling deadline.
Each supervision call drains at most eight callbacks within a 4 ms budget so
path planning can keep up with both high-rate feedback streams. The bundled
`probe_dds.xml`, beside the robot YAML, selects UDP for dashboard probe and
calibration subprocesses to avoid intermittent Docker shared-memory delivery
stalls. An explicit `FASTRTPS_DEFAULT_PROFILES_FILE` environment setting takes
precedence. Other ROS services retain their existing transport configuration.

Upward retractions and clearance lifts default to **10 mm/s**, independently
adjustable from 1–20 mm/s. XY travel inherits AxisForge’s `max_travel_speed_mm_s`
(**120 mm/s** in the current configuration), replacing the bridge’s old 5 mm/s.
Acceleration and joint limits remain active; short lifts may not reach cruise speed.

The complete mesh stores its measured bed centre and direction. Print planning
uses that same frame, so G-code XY and probing agree even when legacy X/Y offsets
differ. Bed direction rotates the grid, not the nozzle orientation.

The commissioned profile also sets `kinematics_file` to the same
`ur16e_calibration.yaml` used by the UR driver. Probing, printing, FK, IK and
joint-limit validation therefore share the calibrated Base-to-tool0 model.
New meshes record its hash; preflight rejects meshes measured with another
model, including legacy generic-model maps. Changing this file requires a new
mesh and preflight. The controller's TCP and payload are not edited.

Pauses between contacts include trajectory planning, UR controller safety queries,
load release/tare, and a clean 1.4-second stationary measurement window. They are
not fixed 10–20-second dwell commands. The dashboard shows safety-query counts
and elapsed time. Repeat contacts at the same mesh point search only as far as
0.5 mm below its first measured height (or the original floor if higher), retaining
all safety samples. The first contact still checks the full configured search.
Setup travel planning services ROS and sensor supervision during IK, retiming,
and interpolation checks; long XY calculations must not starve the feedback
watchdog before the controller safety queries begin.

Before Home is established, a recoverable homing contact failure, or a contact
failure in the separate calibration session,
offers upward **Lift clear** controls with the original unloaded ALPS reference.
Use 0.1 mm or finer while loaded; larger upward steps become available after
fresh readings confirm release. **Finish recovery** closes this attempt without
saving a partial mesh/calibration. Start a fresh routine and repeat its
confirmations before automatic descent. Sensor/control faults, an unconfirmed
stop, or a child process that has already exited do not enable dashboard jogging.
Those manual recovery stages do not move automatically.

Once both homing contacts and their unload checks have established Home, a later
mesh failure automatically attempts one return to that cycle's corrected starting
pose. It confirms motion has stopped, retains the original ALPS baseline and,
if loaded, releases upward in guarded **0.1 mm** steps, at most **5 mm** total.
After fresh readings confirm release, it lifts above all observed points
(including rejected contacts), travels back in XY and lowers to Home. Existing
robot/controller, thermal, heartbeat and sensor checks remain active; new contact
during the return stops it. The destination must be at least 0.5 mm above the
observed surface. A rejected return is not automatically retried.

The dashboard reports **Returning Home after failure**, then keeps the original
scan marked **Failed**, even after a successful return. The old valid mesh stays
active; rejected/partial points are not promoted. A failure during the normal
end-of-scan return is also not retried automatically.
Choose **2–9 points along each axis** for a square or rectangular mesh (4–81 points).
Density changes preserve the bed dimensions, edge margins and centred grid frame.

Each homing pass uses one trajectory with smooth acceleration/deceleration and a
constant-speed middle. ALPS is monitored during motion; there are no per-step
settling pauses. Contact cancels the trajectory, and fresh stationary feedback is
required before recording height or retracting. Settling uses a 1.4-second window
of fresh joint positions and velocities after confirmed cancellation. It rejects
motion, drift, raw excursions, stale feedback and unresolved pose estimates;
initial residual motion may age out of that window within a fixed 4-second
observation deadline after cancellation is acknowledged. A delayed historical
sample rejects its entire window; only a wholly fresh replacement window can
pass before that same deadline. This allows servo
deceleration to finish before collecting the full clean window; it does not
extend the cancellation deadline or widen any movement/load bound.
Motion after the first accepted stationary pose is immediately fatal;
the accepted joint estimate and its calculated nozzle position supply the contact
height. This avoids requiring every noisy position sample to agree within 0.01 mm.
The load reading comes from a fresh ALPS window already contained in that same
stationary interval. No second sampling wait or shifted pose estimate follows
acceptance; missing or stale overlapping data rejects the measurement.
Rejected contacts retain recent native joint and ALPS observations in
`bed_probe.contact-failure.json` beside the bed map. The diagnostic is written
in the background after cancellation, so file I/O cannot delay recovery's ROS
feedback processing. It is a diagnostic record, not a usable mesh or a command
to resume motion.
Confirmation waits and dashboard Z controls use a fixed stationary median as
their reference, while checking later raw positions against the same movement
bound. This avoids comparing two opposite encoder-noise extremes.
The estimate's stability bounds do not establish absolute robot accuracy. Contact during acceleration can
occur before cruise speed is reached. Infeasible speeds fail rather than silently
retiming. Mesh contacts use the same continuous approach at their own slow speed;
scale calibration keeps its **0.1 mm/s** stepped contact search. Set homing acceleration
in the native profile's `homing` section;
legacy homing step-size fields are accepted but no longer used.

In **Setup & calibration → Home & probe bed → Homing and mesh settings**, edit and save
coarse search speed, maximum Z search distance, precision speed, upward speed, mesh speed, X/Y point counts,
sample count (1–5 contacts per mesh point) and repeatability. Initial homing is
exactly one quick contact followed by one fine contact; only the fine contact
sets the height. One contact per mesh point cannot measure repeatability, so its
saved spread is null. With two or more contacts, the selected spread limit applies.
Initial search accepts **0.5–10 mm/s**; **5 mm/s** is the default, not the minimum.
Keep the fine rehoming speed below the initial speed and click **Save homing
settings** to apply edits to the next run.
This works offline, takes effect on the next run and invalidates job preflight.
Active operations block saves, and concurrent edits require Reload. These settings
control measurement repeatability, not certified absolute bed-height accuracy.

For **Maximum Z search**, estimate the vertical nozzle-to-bed gap at the arm's
home/start pose and add **50 mm**. A roughly 300 mm gap therefore uses the default
350 mm limit. Review this value whenever the starting pose changes. Contact stops
the search early; failure to find contact within the selected distance stops the
routine. The full search must pass the robot's reach, path and joint-limit checks
before motion. Saving a distance does not start homing; surface confirmation is
still required.

Before each live setup move, the bridge also queries the UR controller's
`is_within_safety_limits` predicate using small, non-moving secondary programs.
It checks every trajectory waypoint, intermediate samples and cubic joint
extrema against the active installation's safety planes, tool-orientation and
joint limits. A rejected pose or unavailable check blocks that move, including
the entire proposed search depth, even if contact is expected sooner. Remote
Control and access to the robot's Primary Interface (TCP 30001) are required.
This sampled planning check does not replace the controller's safety system or
provide continuous collision checking; the active UR TCP and configured safety
geometry remain installation settings. No safety settings, TCP, payload or main
robot program are changed by the query. Long, slow searches take additional time
to check; sensor monitoring, cancellation and heartbeat checks remain active.

The continuous passes reject latest samples older than **150 ms**, use a **200 ms**
cancellation deadline with independent UR Stop fallback, and reject more than
**0.5 mm observed travel after host detection**. These are software checks, not
physical force/stopping-distance guarantees. The updated Klipper module requests
25 ms host batches, replacing the previous 100 ms batching. At 5 mm/s, 25 ms
corresponds to 0.125 mm of travel before stopping travel; actual delivery timing
remains monitored. Supervised contact commissioning remains necessary; saved
homing readings include stop acknowledgement time and observed stopping travel.

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
after retraction. **Stop probing**, a lost control lease, unconfirmed termination,
unhealthy sensor/feedback, invalid thermal conditions or rejected return path
blocks automatic Home recovery. The dashboard reports why the return could not
finish. Resolve that condition before restarting the routine.

After a failed or stopped run, **Retry homing** starts a fresh attempt using the
saved settings and requests surface confirmation again. First unload the nozzle
and check the search distance from its current position; Retry does not lift the
arm or resume the interrupted trajectory. It stays unavailable while another
operation is active or robot termination is unconfirmed. A simulation instead
offers **Retry simulation**, which remains a simulation.

The default profile is AxisForge's
[`src/axisforge_bringup/config/bridge_bed_probe.yaml`](https://github.com/CrispyButtermilkChickenSalad/AxisForge/blob/main/src/axisforge_bringup/config/bridge_bed_probe.yaml).
It uses **raw ADC counts**, with a contact delta of **20,000 counts** and a maximum
excursion of **120,000 counts** relative to the unloaded session baseline. Either
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
The bridge measures the clock rate between the PC and Klipper host in the
background, then combines it with the MCU clock rate. It needs at least 60 seconds
of recent stable measurements after connection/restart. Preflight waits for this
before heating or moving. Samples are combined as bounded send/receive intervals;
clock jumps or unstable fits still fail. No Internet time service is required.
The default scheduling uncertainty and running alignment limits are both **10 ms**.
The **Clock timing** selector offers 5, 10, 15 and 20 ms alignment profiles with
network and print-speed guidance, plus bounded custom limits. The 15/20 ms profiles
retain a 10 ms scheduling limit; every measurement includes a 1 ms MCU allowance.
Changing limits requires a fresh preflight. The actual planned peak speed and
illustrative timing displacement are reported without changing the motion.
See [clock profiles and their assumptions](synchronized-jobs.md#clock-timing-profiles).
Checks run before heating/approach, at run boundaries and every two seconds during
motion. A failure reports the measured uncertainty or alignment; check host/network
load and prefer wired networking. Raising a limit does not repair the connection.

On the commissioned Fly Pi V2, repeated `eth0: Link is Down` events coincided
with the unused Realtek Wi-Fi adapter waking to scan, interrupting ALPS delivery
for 1–2 seconds. Check `journalctl -k` when fresh-sample failures recur even over
Ethernet. With the host already connected through Ethernet, `nmcli radio wifi off`
disables the unused radio; `nmcli radio wifi on` restores it when needed. Verify
the stream across several former dropout intervals before probing again. This
host setting is separate from the bridge's sample-age and load limits.
A failed host check aborts Klipper without
sending an unknown-session abort to a robot that has not received this preflight.

Klipper and robot leases renew independently, so a delayed reply from one does
not block the other's renewal. Klipper heartbeats use a dedicated persistent
WebSocket with minimal acknowledgements. Heating and motion retain a 0.5 s reply
deadline and the configured strict watchdog (normally 1 s). While robot preflight
is non-actuating and all heater targets are off, a guarded 3 s Klipper planning
lease permits transport retries for at most 2 s. Strict control must be acknowledged
again before preflight passes or execution can start. Robot-worker deadlines stay
unchanged. See [heartbeat phases, diagnostics and deployment](synchronized-jobs.md#heartbeats-during-preflight-and-printing).

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
manual arm-startup checklist, home/probe the bed, calibrate ALPS with a weighing
scale, and choose heater/motion/network options. Upload, preflight and synchronized
Start remain in **G-code dashboard**. Arm power, brakes, Remote selection and
program Load/Play have no dashboard controls.

**Start scale routine** asks you to confirm the current upright start pose and
unloaded scale placement. Use the dashboard Z buttons to leave a gap of at most
20 mm above the scale, then choose **Continue — find scale**. It finds
contact in 0.01 mm Z steps at 0.1 mm/s, stopping after at most 20 mm downward
travel, then lifts 5 mm at 0.5 mm/s and waits for scale taring. This is a scale
preparation routine, not the 350 mm bed-homing search, and it does not move to
the initial home/start pose or read the weighing scale automatically.
Confirming the unloaded scale tare starts Klipper's load-cell calibration and
records its unloaded reference. Use the eight direct **Up / Down 10, 1, 0.1 or
0.05 mm** buttons (or the finer selector) to establish a stable measured load.
Travel is bounded to 25 mm above contact and 0.25 mm below it. Downward steps
larger than 0.1 mm must end at least 0.5 mm above contact; unexpected load stops
coarse descent. Enter the actual **Scale reading (g)**, from 50–25,000 g, then
press **Record measured load**. The input range is not the sensor's mechanical
rating; the configured raw limit and verified assembly capacity still apply.

Use Up to unload, then **Save calibration & finish**. Save requires an accepted
measurement and fresh unloaded feedback. It holds the arm stationary, turns
heaters off, runs Klipper `SAVE_CONFIG`, waits through the restart and verifies
the saved calibration. A save or verification error is reported as incomplete.
After verified success, ALPS Logger can read Klipper's `counts_per_gram` and
`reference_tare_counts`; no separate two-weight logger calibration is needed.
Correcting an accepted reading requires stopping and restarting this routine.
The bed mesh is unchanged. Simulations need no hardware and do not save calibration.
Physical validation is still needed with the nozzle, scale and robot connected.

If Start reports an unfinished Klipper calibration from an earlier console
session, use **Clear unfinished Klipper calibration** while connected and idle,
then start the routine again. This explicit action discards the unfinished helper
session and keeps saved calibration values. Starting a routine never clears
another session automatically. The message immediately above the Z buttons
identifies the current stage or failure that keeps them disabled; the buttons
only work during the routine's Z-adjustment stage. **Stop calibration** is unavailable during
`SAVE_CONFIG` and verification because a save/restart cannot be undone; the robot
has already stopped and verification continues.

Use the pendant for initial XY and orientation. AxisForge's nominal nozzle
orientation is **roll 180°, pitch 0°, yaw 90°**; G-code `A=B=C=0` means no added
rotation, not zero joint angles or zero pendant TCP angles. Probing preserves the
measured upright orientation after alignment. The homing cycle can correct a
saved Home waypoint's tilt of up to 0.5° each time, after its clear-surface
confirmation; you do not need access to change that waypoint. The bridge does
not edit saved UR waypoints. The separate scale-calibration routine still
requires an upright starting pose and does not perform this correction.
The pendant normally displays a rotation
vector in radians, not AxisForge's roll/pitch/yaw degrees; keep the measured UR
installation TCP and payload settings. See the official [UR Pose Editor guide](https://www.universal-robots.com/manuals/EN/HTML/SW5_21/Content/prod-usr-man/complianceUR3e/SW_sections/first_program/pose_editor.htm).

In the parent AxisForge workspace, start the stack with
`docker compose --profile klipper up -d`. The passive `robot-connection` service
(port 8082, loopback only) launches the configured driver on **Connect robot**;
no Docker socket is exposed. `python3 scripts/open_dashboard.py --install` adds
a desktop menu entry and dashboard autostart after login. Klipper starts on its
own host. See the parent [setup and calibration guide](https://github.com/CrispyButtermilkChickenSalad/AxisForge#dashboard-startup-and-scale-calibration)
for limits, configuration and boot behavior.

The connection service automatically reconnects the UR dashboard status socket
after a Local/Remote transition drops it. A lost socket is shown as a dashboard
connection fault, rather than an instruction to switch to Remote. Reconnect
attempts are limited to one per 10 seconds and never power the arm, release
brakes, load or start a program. If External Control stops during a mode change,
start it manually after selecting Remote; fresh readiness checks must pass again.

Additional APIs: `GET /api/robot/connection`, `POST /api/robot/connect`, and
`/api/load-calibration/{status,start,confirm,action,abort}`. Status is GET; the
other calibration routes are POST. Confirmation requires its current stage/token;
a step requires `{"command":"step","token":"<current token>","delta_mm":-0.01}`.
Recording requires `{"command":"record","token":"<current token>","grams":500.0}`;
the number must be the actual stable scale reading. Saving requires
`{"command":"finish","token":"<current token>"}` after `calibration_recorded`
and `unloaded` are both true. Each action consumes its current token.

`GET /api/setup/bed` returns dimensions, mesh coverage and a revision. `POST` accepts
`{"revision":"<current revision>","width_mm":235.0,"depth_mm":235.0,"probe_margin_mm":30.0}`.
`POST /api/jobs/copy-to-klipper` accepts `{"job":"existing.gcode"}` and returns its new version name.

`GET /api/setup/probing` returns saved settings and a revision. `POST` accepts
`{"revision":"<current revision>","settings":{"first_speed_mm_s":10.0,"second_speed_mm_s":0.2,"lift_speed_mm_s":20.0,"mesh_columns":3,"mesh_rows":3,"mesh_yaw_deg":0.0,"max_descent_mm":50.0,"mesh_speed_mm_s":0.1,"mesh_samples":5,"mesh_repeatability_mm":0.02}}`.
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
