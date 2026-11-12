# Synchronized jobs: axisforge.sync.v1

## Ownership and timing

Klipper's `axisforge_sync` extra parses the staged G-code once into an immutable,
SHA-256-addressed schedule. It uses the host's extrusion limits and conservative
configured linear/angular limits. Each move has a scalar trapezoid u(t), starting
and ending at zero speed: XYZ, absolute ABC and nominal filament E share u(t).
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
    loop Each chunk
        UI->>K: Sample clock; reserve future T0
        UI->>R: Submit trajectory stamped at mapped T0
        R-->>UI: Controller accepted before deadline
        UI->>K: Commit extrusion at T0
        Note over K,R: Execute the same progress profile
        R-->>UI: Chunk completed at endpoint
        UI->>K: Verify master time passed; acknowledge
    end
    UI->>K: Complete; heaters off
    UI->>R: Complete
```

The Bridge maps Klipper `print_time` to local monotonic time using five request/
response samples, taking the smallest RTT; uncertainty is RTT/2 plus 1 ms and must
be at most 10 ms. It refreshes the mapping for each chunk. The worker and Bridge
must run on the same Linux host. A future ROS action stamp is derived from that
host's ROS/system clock; a >5 ms ROS clock step during motion aborts. UR scaling
must remain 100%; speed slider changes cause a stop, not independent robot retiming.

Reservation defaults to 1 second ahead. The robot must accept at least 250 ms
before T0, then Klipper must commit with at least 250 ms remaining. A random session
and reservation token prevent accidental cross-job/replayed commits. No extrusion
is queued at reservation. Both endpoints must finish before the next reservation.

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
- Each source move stops; chunks nominally span 2 seconds but may contain a
  longer source move (up to 120 seconds). This sacrifices throughput and surface
  quality for explicit synchronized boundaries. There is no seamless lookahead,
  automatic resume, adaptive speed change or joint/extruder hardware clock lock.
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
Omitting preflight options retains both checks enabled for API compatibility.
Names and values are strict: unknown keys, numbers and string booleans fail.
Klipper capabilities advertise `heater_check_options: true` and configured
`heaters`; Bridge sync/status exposes `configured_heaters` and the active choices.
Results distinguish `pass`, `skip` and `warn`. All configured heaters are switched
off before robot planning even when both checks are disabled.

Klipper defaults: enabled false, max_velocity 20 mm/s, max_accel 20 mm/s²,
max_angular_velocity 10 °/s, max_angular_accel 20 °/s², chunk_seconds 2,
watchdog_seconds 1, start_lead_seconds 1, heater_timeout_seconds 300. Installer
enables the module explicitly. AxisForge worker defaults to motion disabled;
`--enable-motion` / Docker `AXISFORGE_SYNC_ENABLE=1` permits user-requested Start.
Neither setting moves anything at service startup.

## Rollback

Stop Bridge and the sync worker while idle. Restore printer.cfg and any previous
extras from the installer's `axisforge-sync-backup-*` directory. If this was a
first install, remove `axisforge_sync.py` and `axisforge_schedule.py` after restoring
the configuration. Restart idle Klipper and verify ready. The legacy read-only
exporter can remain installed. Restore source from version control and restart
services to roll back the dashboard/worker. Do not resume a partially printed job.
