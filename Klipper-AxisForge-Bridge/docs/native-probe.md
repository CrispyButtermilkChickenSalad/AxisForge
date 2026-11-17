# Klipper MCU contact detection for AxisForge probing

This is an optional, staged contact source for the guided homing and mesh routine.
The existing `stream` source remains the default. The native source uses the
configured Klipper `[load_cell_probe]` MCU trigger to detect nozzle contact;
AxisForge still plans, executes and cancels every robot movement.

The module is **`[axisforge_probe]`**. Arming it does not issue `G28`, `PROBE`,
`BED_MESH_CALIBRATE`, or any robot motion command. Do not switch the live probing
profile until calibration and the commissioning checks below have passed.

## Calibration is required

The configured native contact threshold is currently **200 g**. That value is
meaningful only after a real known-load calibration and an unloaded tare
reference. It is not equivalent to the legacy **20,000 raw-count** trigger.

At the 16 November 2026 commissioning check, the native adapter was installed and
reported `supported: true`, but the sensor reported `calibrated: false`.
The ALPS configuration's
**138 counts/gram** had not been verified using a known load, and
`reference_tare_counts` was null. Those values do not establish calibration.
Native arming must remain blocked until Klipper reports a calibrated sensor with
a valid scale and tare reference. Do not insert an arbitrary tare value or change
a calibration flag merely to pass this check.

Use a known load and Klipper's load-cell calibration procedure to establish the
scale. Confirm an unloaded baseline, verify the indicated load against the
reference load, and save the real calibration. The module's per-contact tare is
an operational baseline; it does not replace known-load calibration.

The installed Klipper calibration helper rejects a loaded/unloaded ADC change
smaller than **1% of positive full scale**: approximately **83,886 counts** for
this signed 24-bit sensor. Ordinary bed probing and the initial search for the
weighing scale use a **120,000-count** overload limit. The manual calibration
stage has a separately adjustable limit for recording the known-load reading.

In **Setup & Calibration**, the manual calibration limit defaults to **120,000
raw counts** and can be set from **90,000 to 250,000 counts**. It applies only after
the scale has been found, the nozzle has lifted, and the operator has confirmed
the unloaded scale tare. The routine then allows supervised Z steps to obtain a
known load measured by the scale. Changing this calibration limit does not
change the bed-probing limit or Klipper's configured contact force. Raw counts
describe ADC excursion; they are not a rated mechanical force for the sensor or
hotend mounting.

Reaching the manual calibration limit cancels the active movement and requires a
confirmed terminal robot result before further movement. The routine then permits
only **upward steps of at most 0.1 mm**, at 0.1 mm/s, to release the load. Downward
steps and finishing remain unavailable until fresh stable readings confirm the
load has cleared. It keeps the original unloaded baseline, and increasing load
during an upward release aborts the routine. Sensor, temperature and controller
faults abort; they do not enable release-only recovery.

The dashboard exposes eight direct Z buttons: **Up / Down 10, 1, 0.1 and 0.05
mm**. Finer 0.01 and 0.001 mm steps remain in the selector. The 10 and 1 mm down
moves are clearance moves and must finish at least 0.5 mm above detected contact.
Steps of 0.1 mm or finer may enter the contact region, within the existing travel,
raw-load and temperature limits. Each click requires fresh feedback and consumes
one action token; movements are not queued.

After confirming the unloaded scale tare, the bridge starts Klipper's
`LOAD_CELL_CALIBRATE` helper and captures the unloaded reference. Apply a stable
load, enter the actual **Scale reading (g)** from **50 to 25,000 g**, and click
**Record measured load**. The range validates input, not the load cell's or
mount's mechanical capacity. The helper also requires a sufficiently large ADC
change as described above. Recording does not move the arm, and an accepted
measurement is single-use; stop and restart the routine to correct that reading.

Use Up to release the load, then click **Save calibration & finish**. The bridge
requires the recorded measurement and fresh stable unloaded readings before
accepting the calibration. It completes robot/sensor cleanup, holds the robot
stationary, turns heaters off and invokes `SAVE_CONFIG`. That command restarts
Klipper. The dashboard waits for Klipper and verifies the saved calibration
before reporting success. A failed save or verification is shown as incomplete;
do not assume the new calibration is usable. After verified success, ALPS Logger
can read the saved `counts_per_gram` and `reference_tare_counts` from Klipper; do not
perform a separate two-weight logger calibration. Simulations do not write
calibration or restart Klipper.

For the logger integration, query Moonraker's
`/printer/objects/query?configfile=config` and read
`result.status.configfile.config.load_cell_probe.counts_per_gram` and
`reference_tare_counts` from the same section. Configuration values are strings;
parse them as numbers. These are the loaded saved values after the verified
restart, not the pending calibration edits.

If an earlier console `LOAD_CELL_CALIBRATE` session is still open, the routine
rejects startup. While connected and idle, click **Clear unfinished Klipper
calibration**, then start again. The button discards only the unfinished helper
session; saved values remain unchanged. No other session is aborted automatically.
During saving and verification, **Stop calibration** is disabled: the robot has
already stopped, and the Klipper save/restart cannot be undone. The dashboard
continues verification and reports its result.

## Calibration temperature

Cold calibration remains the default. For an explicitly selected hot calibration,
choose a nozzle target from **50 to 300°C** and set the same target manually in
Klipper. The current operator-selected target is **210°C**. The calibration
routine never turns heaters on. The operator maintains the selected target during
measurement; **Save calibration & finish** turns heaters off after the load is
released, before `SAVE_CONFIG`. The bed heater must remain off.

Before recording the unloaded baseline or allowing robot motion, fresh nozzle
readings must remain within **±2°C of the selected target for 30 continuous
seconds**. A change of target, loss of fresh readings, or departure from that band
after readiness aborts the session. The unloaded reference and known-load
measurements must be taken at the same stable measurement temperature. The nozzle
reading does not measure the load cell's temperature or establish its temperature
rating.

Hot calibration uses the calibration routine's raw stream. Bed probing has a
separate **Probe at printing temperatures** option in Setup & calibration.
Without that option, probing requires heater targets off and a nozzle at or
below **50°C**. With it, set the chosen targets in Klipper first (nozzle, bed, or
both; 0 means off), then use **Use current Klipper targets** and start the routine.
Selected heaters must stay within ±2°C for the chosen soak (5–1800 seconds,
default 60) before the robot is initialized and a fresh unloaded ALPS baseline
is recorded. Target changes, temperature drift after the soak, or stale feedback
stop probing. Heater commands are not issued by this routine; targets remain
under the operator's Klipper controls after completion, failure or Stop.

Native heated probing requires `heated_probing: true` from the installed
`[axisforge_probe]` module. Update the module and restart the Klipper service to
load new Python code; a G-code `RESTART` alone does not reload that code.
The same calibration, raw overload, sensor-health and motion checks apply in
both cold and heated modes. An absent bed heater is supported with bed target 0.
Nozzle/bed temperature is not a measurement of load-cell temperature; allow
sufficient soak for the toolhead and bed, and use a clean nozzle and a surface
suited to the selected temperatures.

An explicitly heated nozzle also selects a bounded unloaded-noise allowance:
the sample range may reach 40% of the contact threshold, versus 25% for cold or
bed-only probing. At the default 20,000-count trigger this is 8,000 versus 5,000
counts peak-to-peak. The wider range must remain centred within the unchanged
25% unloaded-load bound; the medians of the first and second halves of a sample
window must differ by at most 12.5% of the trigger. This rejects asymmetric spikes
and thermal drift rather than treating them as ordinary noise. Contact detection,
every-sample overload checking, ADC headroom, saved-reference checks and release
criteria remain active. The dashboard and baseline log report the current limit.

Hot scale calibration uses the same bounded noise allowance when choosing its
existing 8,000–20,000-count initial-search trigger. It does not increase that
trigger ceiling or the initial-search overload ceiling. A shorter temperature
soak does not waive the load-cell stability checks.

If the hot calibration's reference is incompatible with a later cold unloaded
reading, native arming rejects it; do not insert a replacement reference merely to
pass that check.

The 200 g threshold remains owned by Klipper's `[load_cell_probe]` configuration.
The bridge does not convert its 20,000-count setting into 200 g or silently replace
the configured force threshold. Review the native threshold and the independent
raw overload limit together after calibration.

## Install and inspect while stationary

The adapter is installed on the commissioning host; native motion remains staged
while the sensor is uncalibrated. To install it on another host, use the following
procedure with the robot stationary, no active print, and heater targets off:

```bash
python3 scripts/install_sync.py \
  --klipper-dir ~/klipper \
  --config ~/printer_data/config/printer.cfg \
  --native-probe
```

The native section uses the existing load-cell probe configuration; there is no
separate sensor selection or force threshold in `[axisforge_probe]`:

```ini
[axisforge_probe]
```

The adapter requires the supported native `trigger_analog` API, `kinematics:
none`, and zero Klipper probe steppers. Unsupported configurations fail the
capability check rather than scheduling a Klipper move.

Follow the installer's backup and restart instructions. Initially perform only
read-only capability checks:

```bash
curl http://<klipper-host>:7125/printer/axisforge_probe/capabilities
```

Protocol `1` and `supported: true` confirm adapter compatibility. Arming additionally
requires `calibrated: true` and finite positive `trigger_force_g` and
`counts_per_gram`; the commissioning host currently reports `calibrated: false`.
A reported threshold alone does not prove calibration. Investigate an error or
unsupported capability before attempting to arm; the client does not fall back
to streamed contact detection after a native failure.

## Select the contact source

After installation, real calibration and stationary event validation, select the
native source in AxisForge's `bridge_bed_probe.yaml`:

```yaml
contact:
  trigger_source: klipper_probe
  trigger_mode: raw
  # Preserve the other reviewed contact and overload settings here.
```

Merge this field into the existing `contact` mapping. Keeping
`trigger_source: stream`, or omitting it, retains the existing streamed-contact
workflow. This setting selects detection only: saving it does not start homing or
move the arm. The usual surface and first-mesh-point confirmations still apply.
Native mode requires `trigger_mode: raw` for the independent guard units; its
primary force trigger still requires the real calibration described above.

In native mode, the MCU event is the primary contact trigger. The ALPS stream
remains required for freshness, noise, ADC-health, raw overload and load-release
checks, including while the UR trajectory is stopping. Loss of that monitoring
still aborts probing. The module also constrains native raw safety thresholds
using the armed tare and the supplied maximum raw-count excursion.
The PC derives its load-persistence and release comparison level from the native
force threshold times the verified counts-per-gram scale, rather than treating
the saved 20,000-count streamed threshold as the native contact setting.

## Session and stop behavior

The client uses a new session for each contact; stale events cannot satisfy the
next descent. Moonraker exposes these module endpoints:

| Endpoint | Purpose |
| --- | --- |
| `/printer/axisforge_probe/capabilities` | Read support, calibration and configured native threshold. |
| `/printer/axisforge_probe/arm` | Tare and arm the MCU trigger for one session after checking readiness. |
| `/printer/axisforge_probe/wait` | Renew the short lease and obtain an armed, triggered, fault or disarmed result. |
| `/printer/axisforge_probe/disarm` | Confirm cleanup of that session's native trigger. |

Arming requires a ready, idle printer, real calibration, no conflicting print
lease and the selected thermal policy. The default is heaters off and a cool
nozzle; explicit heated sessions provide `thermal_targets` for `extruder` and/or
`heater_bed`, with unselected heaters off. It returns armed only after the
scheduled arm time and an MCU state query reporting **`homing: 1`**, with no early
contact. A command acknowledgment alone is insufficient. Only a classified
contact event returns `trigger_time`; a fault must not be treated as bed contact.
The lease is at most 0.5 seconds, and each wait is bounded to 0.1 seconds.

**`stop_required: true` requests immediate UR cancellation**, even if the response
still says `armed` while Klipper finishes classifying the event. Stop first;
accept a contact only after a `triggered` response with a valid `trigger_time`.
Waiting for final classification must not leave the robot descending.

The MCU event does **not** directly stop the UR controller. The PC must receive the
event and cancel the robot trajectory, then confirm termination and stationary
feedback before measuring or retracting. Existing cancellation deadlines, raw
overload checks, maximum descent, joint/path limits and the independent UR Stop
fallback remain active. A failed or unconfirmed stop does not authorize automatic
retraction or another descent.

## What this improves, and what still needs validation

The observed ALPS ADC rate is about **488 samples/second**. MCU contact detection
avoids waiting for the approximately **100 ms bulk-sample delivery** used by the
streamed trigger. This does not establish a 2 ms end-to-end stop time: network
delivery, host processing, ROS cancellation and UR deceleration remain in the
path. Measure actual trigger-to-stop behavior on the commissioned assembly.

Native triggering and stationary-pose acceptance are separate checks. Continuous
homing and mesh contacts require a fresh 1.4-second window after confirmed robot
cancellation. Joint velocity, timestamps, raw excursions and filtered pose drift
must pass before the accepted joint estimate supplies the contact height. Initial
residual velocity, pose drift or oscillation may age out of the rolling window
within the original 4-second observation deadline, with overload and travel checks active
throughout. Every accepted window must still meet the 0.01 mm / 100 microradian
drift limits; drift failures include the measured values. Renewed motion after
the first accepted stationary pose rejects the contact. The force samples must
overlap that accepted stationary interval and remain fresh; there is no extra
load-sampling wait after pose acceptance. A valid
contact event can still be rejected if the arm does not meet these settling or
repeated-height criteria. Feedback stability is not absolute robot accuracy.

Dashboard Z positioning runs inside the same monitored homing/calibration
session, after explicit unloaded surface confirmation. Every downward positioning
step stops on contact. Continue captures the adjusted pose before automatic
searching. A recoverable contact failure retains the original unloaded baseline
and exposes upward release only; it never tares the loaded nozzle. Small upward
steps (0.1 mm or finer) are required until fresh samples confirm release. Finish
recovery closes the failed attempt; another automatic search requires a new
routine and confirmations. Lost feedback, an unconfirmed stop and terminated
sessions do not offer this recovery path.

The default **5 mm/s initial search** is a requested setting, not a validated
contact speed. Keep the arm stationary while validating capabilities,
calibration, arming, event classification and disarming. Live commissioning then
requires an explicitly confirmed bounded movement, measured stopping behavior,
and acceptable repeated contacts before proceeding to a complete mesh. This
document does not certify a successful physical homing or mesh run.

During dashboard upward recovery, a load rise above the retained release ceiling
stops the requested step immediately. The recovery session remains open with
further steps blocked until a full fresh ALPS window returns below the same
ceiling. No step repeats automatically, the baseline/ceiling is not raised, and
the hard overload, freshness and confirmed-stop checks remain active. A persistent
rise requires inspection of the toolhead and contact direction. Stop and Retry
retain the previous reported issue in the dashboard for diagnosis.

## Raw sample delivery

`[axisforge_probe]` also supports `sample_batch_interval_s: 0.025` (default,
allowed range 0.020–0.100). For a sensor using Klipper's `BatchBulkHelper`, this
drains the existing sample queue more frequently. It does not change ADC rate,
MCU acquisition, calibration, trigger thresholds, or motion freshness limits.
An already faster interval is preserved. Sensors without that helper retain
their existing delivery behavior. The capabilities endpoint reports the applied
`sample_batch_interval_s`, or `null` when no compatible helper is present.

The original 100 ms ADS131M0X batches leave little jitter margin under the
bridge's 150 ms homing freshness guard. After installing the adapter, restart
the **Klipper service** while idle with heaters off and verify delivery before
live probing. The G-code `RESTART` command can retain an already imported Python
module; verify the new capability field rather than relying on a ready status.
This changes raw stream delivery even when native calibrated triggering is not
selected; it does not make an uncalibrated native trigger usable.

