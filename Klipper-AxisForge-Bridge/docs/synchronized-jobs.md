# Synchronized jobs: axisforge.sync.v1

## Ownership and timing

Klipper's `axisforge_sync` extra parses the staged G-code once into an immutable,
SHA-256-addressed schedule. It uses the host's extrusion limits and conservative
configured linear/angular limits. Each move has a scalar trapezoid u(t): XYZ,
absolute ABC and nominal filament E share u(t). With `continuous_motion: True`,
look-ahead carries velocity through tangent-compatible straight moves and queues
one run between explicit synchronization barriers. Each run starts and ends at
rest. The selected corner policy controls internal junction speeds and geometry.
Klipper applies its configured pressure advance to positive XY extrusion. It does
not run a second virtual-SD copy. The robot worker applies the bed transform and
TCP once, samples IK and cubic joint interpolation, and rejects any joint velocity,
acceleration, position, orientation or branch-continuity violation. It preserves
all master timestamps. Only the initial nonextruding approach may be retimed.

```mermaid
sequenceDiagram
    participant UI as Dashboard
    participant K as Klipper master
    participant R as AxisForge worker
    UI->>K: Verify file hash; prepare; acquire lease
    UI->>K: Heat to file targets and verify
    UI->>K: Switch all heater targets to zero
    UI->>R: Validate the same timed plan and entry path
    UI->>K: Release lease
    UI->>R: Release lease
    Note over UI: User clicks Start
    UI->>K: Acquire; reheat per file
    UI->>R: Acquire; approach entry point
    loop Each run (legacy mode: each chunk)
        UI->>K: Sample clock; reserve future T0
        UI->>R: Submit trajectory stamped at mapped T0
        R-->>UI: Controller accepted before deadline
        UI->>K: Commit extrusion at T0
        Note over K,R: Execute the same progress profile
        R-->>UI: Run completed at endpoint
        UI->>K: Verify master time passed; acknowledge
    end
    UI->>K: Complete; heaters off
    UI->>R: Complete
```

The Bridge maps Klipper `print_time` to local monotonic time using five request/
response samples, taking the smallest RTT; uncertainty is RTT/2 plus 1 ms and must
meet the selected scheduling limit (10 ms by default). It refreshes the mapping before each run and checks clock drift
every two seconds during execution. A running trajectory keeps its original
timestamps; an alignment estimate above the selected running limit (10 ms by default)
aborts both participants. The estimate is the absolute offset change plus the latest
sampling uncertainty; it is not measured physical robot lag. The worker and Bridge
must run on the same Linux host. A future ROS action stamp is derived from that
host's ROS/system clock; a >5 ms ROS clock step during motion aborts. UR scaling
must remain 100%; speed slider changes cause a stop, not independent robot retiming.

### Clock timing profiles

Choose **Clock timing** in the job preflight card before running preflight.
The two limits have different roles: scheduling uncertainty bounds each clock
measurement used to place a trajectory; running alignment bounds the periodic
comparison against that trajectory's original clock mapping. Both accept finite
values from 2 to 20 ms, with uncertainty no greater than alignment. Defaults stay
at 10/10 ms. The 15/20 ms profiles keep scheduling uncertainty at 10 ms.

| Profile | Scheduling / running limit | Recommended network setup | Illustrative peak XYZ speed |
| --- | --- | --- | --- |
| Wired precision | 5 / 5 ms | Both hosts on one Ethernet switch or a direct wired link; verify low jitter | ≤20 mm/s |
| Standard (default) | 10 / 10 ms | Both hosts on one Ethernet switch | ≤10 mm/s |
| Wi-Fi commissioning | 10 / 15 ms | Strong local Wi-Fi, power saving off, avoid hotspot/repeater hops; wired preferred | ≤6.67 mm/s |
| Slow commissioning | 10 / 20 ms | Controlled slow trials on a measured local connection | ≤5 mm/s |
| Custom | 2–20 / 2–20 ms | Measure the connection and validate on the cell | 100 ÷ running limit in ms |

These speeds use an **illustrative 0.10 mm timing-displacement allowance**:
speed = distance / time. They are engineering starting points, not manufacturer
ratings, certified safe speeds or guarantees of print quality. A tighter allowance
needs a proportionally lower speed. Rotary motion, joint tracking, acceleration and
pressure-advance alignment need separate validation. Raising a limit accepts more
uncertainty; it does not synchronize the clocks or repair packet loss.

Preflight reports the actual planned peak XYZ speed (including travel) and the
distance travelled at that speed over the selected running limit. Exceeding the
illustrative allowance produces a guidance warning; it never silently changes the
G-code feed or master timeline. Change the source feed or Klipper master's velocity
limits and repeat preflight if you need a smaller allowance. The robot's speed
slider must remain at 100%.

The browser remembers the selection. Preflight records the canonical limits and
binds them to the worker session; changing either limit requires preflight again.
Start and every scheduled run must use the worker's prepared values. An older
worker without clock-options support is rejected before heating. The Klipper
module and its G-code plan hash are unchanged by these acceptance settings.
Deploy the updated Bridge and AxisForge worker together while idle.

The dashboard shows the last measurement, its age and the limits used. Sampling
still retries for at most 20 requests / 1.5 s, running checks remain every two
seconds and clock mappings expire after five seconds. Clock profiles do not alter
the separate heartbeat policy below, robot tracking checks, the 5 ms ROS clock-step guard or cold-extrusion
protection. Qualify a connection under sustained load and verify a controlled
print; a short idle measurement does not establish reliable operation.

API callers can include the same `clock_options` in preflight and Start:

```json
{"max_uncertainty_ms": 10, "max_alignment_error_ms": 15}
```

Omitting it on preflight selects 10/10 ms; omitting it on Start uses the saved
preflight settings. `/api/sync/status` exposes `clock_profiles`, `clock_defaults`,
`clock_options`, `clock_measurement` and `clock_speed_guidance`.

### Heartbeats during preflight and printing

Klipper heartbeats use an authenticated persistent WebSocket dedicated to
`printer.axisforge_sync.heartbeat`. Clock sampling has its own socket and lock;
large HTTP plan transfers, telemetry and status requests use neither socket.
Each socket continuously drains Moonraker notifications between requests so
an idle clock connection cannot fill its receive queue and block ping/pong.
The heartbeat acknowledgement contains only the session, state, lease flag,
lease mode and watchdog duration. It performs no full-status construction,
configuration hashing, sensor reads, file access or MCU-clock conversion.

| Phase | Klipper lease | Bridge behaviour |
| --- | --- | --- |
| Heater checks, Start, heating, approach and printing | Configured strict watchdog, normally 1 s | Abort on a failed/late heartbeat exchange; 500 ms reply deadline |
| Robot preflight with all heater targets zero, before any print/motion reservation | 3 s | Retry transport failures within 2 s of the last acknowledged request's send time; each exchange remains bounded by 500 ms |

The longer lease is a **cold-preflight-only ownership lock**, not permission to
actuate. The master accepts entry only for a prepared, unstarted session with
no reservation, no completed chunks, no active virtual-SD print and zero heater
targets. Existing manual-command guards stay active, and heater, fan, reserve
and commit RPCs are forbidden in this mode. The watchdog checks the cold-state
invariant every 50 ms and aborts if it is violated. “Cold” means heater targets
are off; the nozzle does not have to cool to ambient before planning.

The bridge changes mode only after verifying heaters are off, and only after
the master acknowledges the new mode. It returns to strict mode when robot
planning finishes: Klipper revalidates the file and configuration, the bridge
requires a fresh strict heartbeat acknowledgement, and clock alignment is sampled
again before preflight can pass. Start reacquires strict ownership and verifies
another heartbeat before heating or approaching. Expired leases and rejected RPCs
are never retried or revived. Lost replies cannot cause unlimited recovery even
if the server received the renewals; the bridge's two-second window is independent.

Robot-worker heartbeats remain independent, with their original 500 ms reply
deadline and one-second lease. They use a dedicated persistent HTTP/1.1 connection
so trajectory submissions and status traffic cannot consume their connection.
ROS feedback, calibration reads and controller cancellation run outside the
worker's lease lock; the session, generation and lease are rechecked before state
changes. Large ROS trajectory messages are built during cold preflight, with
fresh execution stamps assigned at scheduling. Their joint samples and timing
are unchanged.

While a goal is active, the worker also requires its motion-supervision loop to
progress within 250 ms. This independently stops a stalled tracking/check loop
even if heartbeats are arriving. Late goals are cancelled, with a dashboard stop
fallback if cancellation fails. The first fault is retained when late controller
responses arrive. Clock limits, joint tracking limits and MCU shutdown protection
remain in force. A sustained connection failure still ends
preflight or printing; the recovery window is not a substitute for reliable
networking.

The dashboard and `/api/sync/status` expose `heartbeat`: last/max exchange latency,
cold retry count, recovery state and maximum observed bridge event-loop delay.
The separate `robot_heartbeat` field reports robot exchange latency; the worker's
own status includes heartbeat counts, maximum lock wait and maximum renewal gap.
These measurements help distinguish local scheduling stalls from slow replies;
they are not a measurement of network latency alone. Clock-limit selection does
not change heartbeat settings.

Deploy the matching Bridge and Klipper module together. Capabilities must report
`heartbeat_protocol: 2` and `cold_preflight_watchdog_seconds: 3`; older modules are
rejected before heating. Use the installer to back up/update the extras, then
restart the Klipper service while idle with heaters off. Configuration values and
the strict `watchdog_seconds` are preserved. A G-code RESTART does not reload an
already imported Python module.

### Scheduling and commit

Reservation defaults to 1 second ahead. The robot must accept at least 250 ms
before T0, then Klipper must finish prequeueing with at least 250 ms remaining. A random session
and reservation token prevent accidental cross-job/replayed commits. No extrusion
is queued at reservation. Both endpoints must finish before the next reservation.
All moves inside a continuous run are already queued on both sides, with adjacent
timestamps. Klipper's native motion queue generates MCU steps ahead of execution;
the Bridge does not pace individual source moves over HTTP.

## Continuous motion

Update both the Klipper extras and AxisForge worker, then enable this setting in
the existing `[axisforge_sync]` section while idle and restart Klipper:

```ini
continuous_motion: True
```

Rerun preflight after the update. New installations enable this mode; the installer
preserves existing configuration, and the module defaults to False when the option
is absent. Set it to False to restore the previous two-second chunk behavior.
Capabilities report `execution: preplanned_continuous_runs` when enabled.

The look-ahead performs backward braking and forward acceleration passes. It keeps
XYZ/ABC velocity continuous across collinear moves and bounds changes in E/mm by
Klipper's configured extrusion corner velocity. Reversals, rotary tangent changes
and transitions between extrusion and travel retain zero junction speed. M400,
G4, heater and fan commands end a run. Select the policy in the dashboard's
**Corner motion** controls before preflight:

| Policy | Parameter defaults | Behavior |
| --- | --- | --- |
| Conservative (`exact`) | None | Keeps source geometry; stops at sharp corners. Existing installations retain this default. |
| Klipper (`klipper`) | `square_corner_velocity: 1` mm/s | SCV-derived centripetal junction-speed model, including the half-segment contact bound. Source vertices remain unchanged. |
| Marlin (`marlin`) | `junction_deviation_mm: 0.01` mm | Junction Deviation speed model with the small-segment arc bound. Source vertices remain unchanged. |
| Smooth blend (`blend`) | `blend_tolerance_mm: 0.05`, `blend_speed_mm_s: 5` | Rounds eligible constant-orientation XYZ corners within the selected deviation. Curvature and extrusion limits can reduce the chosen speed. |

The firmware choices implement their **junction-speed policies**, not complete
firmware planners. They do not reproduce Marlin Classic Jerk, Linear Advance,
S-curve acceleration, Klipper minimum-cruise-ratio or input shaping. See the
[Klipper explanation](https://www.klipper3d.org/Kinematics.html) and
[Marlin explanation](https://marlinfw.org/docs/features/junction_deviation.html).
The worker produces finite, shared joint velocities at nominal sharp corners,
then validates the actual controller cubic interpolation against the configured
path/orientation and joint acceleration limits. A firmware-policy corner can
fail robot preflight; lower SCV/JD rather than weakening joint limits.

Geometric blending constructs quintic curves with tangent and zero-curvature
line joins, then flattens each into 32 shared robot/extrusion segments. Half the
requested tolerance bounds the displacement of that polyline from corresponding
points on the source path. Robot preflight checks the actual cubic-interpolated
TCP against the transformed source path at the full tolerance, or the robot's
configured tolerance if tighter. Curvature bounds use derivative control hulls;
half the linear acceleration budget is reserved for turning. Sharp reversals,
changing ABC, retractions and changes in native PA eligibility are not rounded.
Each source move's nominal E total is preserved and redistributed by length over
its half of a shortened corner. This can increase local E/mm; cross-section,
extrusion speed/acceleration and extrusion-junction limits are rechecked. Tight
tolerances can make a smooth turn slower than a stop-and-turn. Standalone
AxisForge's `corner_blend_tolerance_mm` does not override these per-job choices.

All choices are hashed into the plan and saved preflight. The dashboard remembers
them, locks controls while busy, and blocks Start after any change. Both services
must advertise support; an older worker is rejected before heating for non-default
policies. Values must be finite JSON numbers, with no unknown fields. Setting SCV,
JD, blend tolerance or blend speed to zero disables the corresponding corner
continuation. The existing straight-line continuation remains available.

Each run is limited by AxisForge's `max_continuous_points` (default 100,000).
Plan preparation has a separate 60-second RPC timeout for larger rounded plans;
pure geometry/timing computation runs off the Klipper reactor thread before
acquiring motion/heater leases. The reactor keeps handling temperatures, clock
queries and shutdown. Concurrent preparation/acquisition/manual motion is blocked,
and settings changes or shutdown invalidate the result. Control RPC timeouts and the
independent 0.5-second heartbeat response deadline are unchanged.
Excessive runs fail preflight; insert M400 at an intentional stopping point to
split them. Larger action messages must still meet the scheduling deadline;
`start_lead_seconds` can be increased up to 5 if needed. All joint, clock, tracking,
lease and abort checks remain active throughout a run.

## Pressure advance with kinematics:none

Pressure advance remains **native Klipper pressure advance in every corner mode**.
The adapter preserves G-code XYZ displacement metadata and passes the scheduled
E ratio, starting/cruising velocities, acceleration and phase times to the real
`PrinterExtruder.process_move`. Positive extrusion with X or Y displacement is
PA-eligible even without physical XYZ steppers; E-only priming, retraction and
pure Z extrusion retain native non-PA behavior. Sending isolated E-only commands
outside this bridge does not provide the same XY metadata.

Klipper's native step generator applies the configured advance and smoothing:
nominal E position plus advance times nominal E velocity, followed by the native
time-window smoothing. No PA correction is baked into the uploaded E values and
no second correction is applied by AxisForge. Changing corner speed changes the
shared nominal flow profile, so PA naturally follows the new acceleration and
deceleration. The robot timeline must continue to pass tracking and clock checks.
See [Klipper's pressure-advance model](https://www.klipper3d.org/Kinematics.html#pressure-advance).

The provided calibration file specifies `SET_PRESSURE_ADVANCE ADVANCE=0.04`.
It passes only when the host is already set to 0.04; a mismatch is rejected before
heating. Runtime PA and smoothing settings are fingerprinted. This scheduler does
not execute dynamic PA changes, M900 or tuning-tower macros inside a job. Selecting
Marlin JD changes corner planning only; it does not enable Marlin Linear Advance.
Native PA behavior does not establish that 0.04 is calibrated for a given filament,
nozzle and temperature; physical tuning remains as on a conventional printer.

For an offline adapter/native-queue comparison against an installed Klipper tree:

```bash
python scripts/check_pressure_advance.py --klipper-dir /path/to/klipper --plan plan.json
```

`plan.json` is the compiled master plan returned by prepare. The script runs native
Move/process_move code with an in-memory queue sink and never connects to hardware.

## Preflight and faults

Files are limited to 32 MiB and plans to 20,000 operations / one million robot
samples. Preflight fingerprints file content, Klipper settings, robot config,
calibration and bed measurements. It expires after 15 minutes; Start rechecks.
The worker rebuilds the nonextruding approach from current joints.

Preflight reads heater targets from the complete file, rejects out-of-range
requests, verifies selected heaters' peak temperatures within 3 °C for at least one
second, then confirms all targets are zero before robot planning. No heat command
is silently inferred for files lacking one. Default heat timeout is 300 seconds.
Start waits at every positive heater command, including M104/M140. Bed heater
commands above zero fail if no bed heater exists and its check is enabled. With
its check disabled, commands for that unconfigured heater are removed from the
job and reported as warnings. Off-only commands pass without a configured sensor.
Configured heaters retain all execution commands when their preflight check is
disabled. Every extrusion chunk still passes Klipper's live cold-extrusion check.
The two boolean options are included in the plan hash and the saved preflight.
The UI defaults to checking configured heaters and remembers operator choices;
changing options disables Start until they match the validated result.

A passed preflight validates the path; Start also requires the robot to be ready
for live motion. On the UR teach pendant, run the program containing External
Control and set the speed slider to 100%. The dashboard shows `Start blocked`
and the current reason beside the Start button until all live conditions are
met. It updates automatically as the robot becomes ready. If the preflight has
expired or heater choices changed, run it again with the intended choices.

Both participants hold independent 1-second heartbeat leases. Bridge shutdown,
disconnection, heater errors/timeouts, stale joints (>250 ms), UR safety/program/
speed faults, tracking error (>2 degrees), rejected/late goals and late chunk
completion cause an abort. Targets turn off. Cancelling robot goals has an
independent UR dashboard Stop fallback. Already committed MCU steps require a
Klipper shutdown; merely discarding a host queue would not stop them. External
PAUSE, CANCEL_PRINT or TURN_OFF_HEATERS also aborts an owned job. Known manual
motion/heater mutation commands are blocked while leased. This is software
coordination, not a certified simultaneous emergency-stop circuit; physical stop
latency is bounded by the controllers/network/watchdogs and must be measured.

## Supported dialect and deliberate limits

- G0/G1 XYZABC and E, modal F, G20/G21, G90/G91, G92 XYZE, M82/M83, M220/M221.
  ABC always means absolute degrees, including relative linear mode/inch units.
- M104/M109, M140/M190, primary fan M106/M107, G4 and M400. T0 and M84 are no-ops.
- Initial G28 is replaced by the validated robot approach, with a plan warning.
  The first nonextruding XY move defines entry; earlier Z/orientation establish
  its pose. A file needs a subsequent motion chunk.
- SET_PRESSURE_ADVANCE is accepted only if ADVANCE equals the host configuration.
  Unknown macros, arcs, unsupported tool changes/parameters and malformed input
  fail before heating. This is intentionally not arbitrary Klipper G-code.
- Without continuous mode, each source move stops and chunks nominally span
  2 seconds. A source move's rest-to-rest profile may take up to 120 seconds.
  Neither mode supports automatic resume, adaptive speed changes or a hardware
  clock lock between the joint controller and extrusion MCU.
- Preflight validates the configured kinematic/workspace constraints; it does not
  model every external obstacle or certify the cell as collision-free.

## RPC and configuration

Moonraker forwards `/printer/axisforge_sync/{action}` to the Klipper extra.
Actions: capabilities, clock, prepare, acquire, heartbeat, heat, heaters_off, fan,
reserve, commit, ack, release, complete, abort, status. Mutations require the
session token; commit/ack additionally require the reservation token. The robot
loopback worker on port 8081 provides status/capabilities plus prepare, acquire,
heartbeat, release, approach, schedule, complete and abort. Browser requests use
the dashboard, not the robot worker directly.

The Bridge exposes `/api/jobs/upload`, `/api/jobs/{job}/preflight/both`,
`/api/jobs/{job}/preflight`, `/api/sync/status`, `/api/sync/start`, `/api/sync/abort`.
Legacy arm/disarm calls return 409. Preflight is asynchronous; poll sync/status.
Both preflight and Start accept `heater_checks` in their JSON bodies, for example:

```json
{"heater_checks": {"extruder": true, "heater_bed": false}}
```

Start also requires `job` and rejects choices different from the saved preflight.
Preflight and Start can also contain `motion_options`, for example:

```json
{"policy":"blend","square_corner_velocity":1.0,"junction_deviation_mm":0.01,
 "blend_tolerance_mm":0.05,"blend_speed_mm_s":5.0}
```

Put this object under the `motion_options` key. Omitted preflight options use the
master's configured defaults. Optional `[axisforge_sync]` settings use the same
numeric names, with `corner_policy` selecting the default policy. Capabilities
advertise `corner_policies`, `motion_defaults` and the current `pressure_advance`.
Omitting preflight options retains both checks enabled for API compatibility.
Names and values are strict: unknown keys, numbers and string booleans fail.
Klipper capabilities advertise `heater_check_options: true` and configured
`heaters`; Bridge sync/status exposes `configured_heaters` and the active choices.
Results distinguish `pass`, `skip` and `warn`. All configured heaters are switched
off before robot planning even when both checks are disabled.

Klipper defaults: enabled false, max_velocity 20 mm/s, max_accel 20 mm/s²,
max_angular_velocity 10 °/s, max_angular_accel 20 °/s², continuous_motion false,
chunk_seconds 2 (legacy mode only),
watchdog_seconds 1, start_lead_seconds 1, heater_timeout_seconds 300. Installer
enables the module and continuous motion explicitly for new configurations.
AxisForge worker defaults to motion disabled;
`--enable-motion` / Docker `AXISFORGE_SYNC_ENABLE=1` permits user-requested Start.
Neither setting moves anything at service startup.

## Rollback

Stop Bridge and the sync worker while idle. Restore printer.cfg and any previous
extras from the installer's `axisforge-sync-backup-*` directory. If this was a
first install, remove `axisforge_sync.py`, `axisforge_schedule.py` and
`axisforge_corners.py` after restoring
the configuration. Restart idle Klipper and verify ready. The legacy read-only
exporter can remain installed. Restore source from version control and restart
services to roll back the dashboard/worker. Do not resume a partially printed job.
