# Klipper-timed robot worker

`print_control.sync_runtime` provides the loopback `axisforge_sync_worker` service.
It consumes the `axisforge.sync.v1` schedules produced by the matching
[Klipper-AxisForge-Bridge](https://github.com/CrispyButtermilkChickenSalad/Klipper-AxisForge-Bridge).
The source schedule is in G-code bed coordinates. The worker applies AxisForge's
bed correction, XYZ offsets, nozzle orientation and TCP exactly once, solves IK,
and validates cubic-interpolated joint speed/acceleration and tool pose at the
master's times. An infeasible schedule is rejected; it is never silently slowed.

Preflight reuses the model's kinematic chains and tries IK from the preceding
joint solution before preparing a full random restart search. If that attempt
fails, the original search still runs. Link-height and nozzle-pose checks use
the same DH geometry through the library's compiled evaluator. This optimization
is automatic: it keeps every timing/interpolation sample, tolerance and joint
limit. Each new preflight builds a fresh model from the current configuration;
it does not reuse a previous job's validation result.

## Run

Build the workspace and install the repository's UR16e models as in the main
README. Run from the AxisForge root so relative bed probe paths resolve:

```bash
ros2 run print_control axisforge_sync_worker \
  --config src/axisforge_bringup/config/ur_master.yaml --port 8081 --enable-motion
```

Without `--enable-motion`, nonmoving plan validation works but motion is rejected.
Docker's `sync-worker` service translates `AXISFORGE_SYNC_ENABLE=1` to that flag.
Its launcher uses one BLAS thread for the small IK systems, avoiding numerical
thread contention with ROS feedback during preflight. For a native launch on a
multicore host, use `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ros2 run ...` as well.
The Bridge and worker must run on the same host clock. The server binds only to
127.0.0.1 and rejects browser Origin headers. Configure the Bridge with
`AXISFORGE_SYNC_URL=http://127.0.0.1:8081`.

The worker connects to the existing UR ROS driver:

| Interface | Purpose |
| --- | --- |
| `/joint_states` | All six named joints, source timestamps and freshness |
| `/scaled_joint_trajectory_controller/follow_joint_trajectory` | Future-stamped timed chunks |
| `/speed_scaling_state_broadcaster/speed_scaling` | Require 100% while moving |
| `/io_and_status_controller/robot_program_running` | External Control interlock |
| `/io_and_status_controller/safety_mode` | Require NORMAL mode |
| `/dashboard_client/stop` | Independent stop fallback if goal cancellation fails |

The [UR speed-scaling broadcaster](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/2.15.0/ur_controllers/src/speed_scaling_state_broadcaster.cpp)
reports **percent**, while the controller's internal scaling factor uses 0–1.
The worker explicitly divides the topic value by 100;
it never guesses units from the value. In feedback, `speed_scaling` is the factor
and `speed_scaling_percent` is the reported percentage. A topic value of `1.0`
therefore means 1% and blocks synchronized motion. The 100% guard and freshness,
External Control and NORMAL-safety requirements remain active.

It creates no extrusion publisher. Existing standalone AxisForge/Portenta jobs
remain separate; do not launch those concurrently with a synchronized job.
The dashboard blocks its own standalone launch controls while the sync workflow
owns the robot.

## Workflow and faults

The Bridge uploads the same file to both hosts. Klipper derives the master plan
and checks the selected extruder/bed heater temperatures, then confirms all
configured targets are zero. The two preflight checks are independent. An unchecked,
unconfigured heater has its commands omitted from the job with a warning;
configured heaters still execute their G-code commands at Start. Off-only targets
need no heat test. Heater options are part of the plan hash. Only then
does the worker validate the timed trajectory and the entry approach. Preflight
returns the plan hash and a hash of configuration/calibration/bed inputs. Start
must use those exact inputs. The entry approach is rebuilt from current joints;
only this nonextruding approach may be retimed.

At each run boundary, Klipper reserves a future start, the worker gets controller
acceptance, and only then does Klipper queue matching extrusion. Start times use
a bounded mapping of Klipper print_time to this host's monotonic clock. No timestamp
is inferred from percentage progress or the presumed duration of a separate job.

Set `continuous_motion: True` in Klipper's `[axisforge_sync]` section after updating
both the master extras and this worker. It queues one trajectory between G-code
barriers instead of waiting at two-second transport boundaries. The master plans
nonzero junction speeds through compatible straight XYZ/ABC moves; the worker
shares one joint velocity at each junction and checks both adjoining cubic
intervals. Select the corner policy in the Bridge before preflight. The default
`exact` retains the previous stop-at-corners behavior. `klipper` uses the SCV
junction-speed model; `marlin` uses Junction Deviation. These two preserve the
source vertices, with actual joint interpolation still subject to path and
acceleration checks. They are speed-policy implementations, not complete firmware
planners or permission to exceed robot limits.

`blend` creates a shared, bounded XYZ polyline from quintic curves and redistributes
each source move's E amount across its portion of the shortened path. It reserves
half the chosen tolerance for geometry and checks actual cubic-interpolated TCP
positions against the transformed original path at the full tolerance (or the
configured robot path tolerance if tighter). Default tolerance is 0.05 mm and the
blend speed cap is 5 mm/s; curvature and joint constraints may require less speed.
Tight blends can take longer than stopping at a corner. Changing ABC, retractions,
near reversals and PA-eligibility changes are not rounded. M400, G4, heater and fan
commands still end a run. Standalone `corner_blend_tolerance_mm` does not override
the master's per-job `motion_options`.

The worker advertises supported `corner_policies`; the Bridge rejects an older
worker before heating when a new policy is selected. Options are hashed into the
master plan and compared at Start, so changed values require a fresh preflight.
Klipper's native pressure advance remains active for positive XY extrusion in all
modes, using the same planned E profile despite `kinematics:none`. No pressure
advance is applied in the robot worker, and choosing Marlin JD does not select
Marlin Linear Advance. See the companion Bridge guide for PA verification and
the exact corner-policy scope. Physical PA calibration and tracking accuracy
still require print measurements.

The worker limits each continuous run to `max_continuous_points` (default 100,000)
and the whole job to one million samples. Add M400 at an intentional stopping
point if a run exceeds that limit. Every sample and interpolation check remains
part of preflight. ROS trajectory messages are constructed during cold preflight;
only their execution stamps are assigned when scheduled. The bridge uses a
dedicated persistent HTTP/1.1 connection for robot heartbeats. ROS/file checks
and cancellation never hold the worker's lease lock, and session/generation
checks reject a validation result that became stale while those calls ran.
An active goal also has a 250 ms motion-supervision deadline: a stalled tracking
loop stops the goal even if heartbeats still arrive. The 500 ms heartbeat reply
deadline, one-second lease and joint tolerances are unchanged. Worker status
reports heartbeat count, maximum lock wait and renewal gap; the bridge exposes
robot exchange timings separately from Klipper timings.

Larger action messages must still be accepted before the
commit deadline; Klipper's `start_lead_seconds` can be increased up to 5 if needed.
The Bridge checks master clock drift every two seconds during a run and aborts
above the preflighted alignment limit (10 ms by default), without changing either
participant's timestamps. The Bridge's **Clock timing** selector provides 5/10/15/20 ms
alignment profiles with network and speed guidance, and custom limits from 2 to
20 ms. Scheduling uncertainty is a separate limit, no greater than alignment;
the 15/20 ms profiles retain 10 ms scheduling uncertainty. The worker advertises
`clock_options_supported`, stores the canonical `clock_options` at prepare, and
rejects acquire/schedule requests that differ from that session. Omitting options
on a worker request means 10/10 ms, preserving the legacy strict default.

The speed guidance uses an illustrative 0.10 mm timing-displacement allowance
(speed = distance / alignment time), not a certified safe speed or a measured
robot error. Preflight warns when the planned peak exceeds this allowance; no
speed is changed automatically. Clock options are stored alongside the preflight
and do not alter the Klipper G-code plan hash. Update the Bridge and worker
together while idle; no Klipper service restart is needed for this feature.

The worker requires a 1-second lease, fresh feedback within 250 ms, speed scaling
100%, NORMAL safety and a running UR program. It monitors joint tracking to
2 degrees, endpoint error to 0.2 degrees, completion deadlines and ROS clock jumps.
A failure cancels the action and uses dashboard Stop if cancellation fails. The
Bridge also has a separate rosbridge Stop path for worker failure. Klipper has its
own watchdog and shuts down if already committed extrusion must be aborted.
Lease renewal uses only in-memory session state, keeping ROS queries and
configuration-file reads out of the heartbeat handler. These checks still run
for motion requests and execution monitoring. The Bridge renews each participant
independently and aborts if a heartbeat response takes longer than 0.5 s.
The Klipper side uses a separate, guarded cold-preflight mode: after heater targets
are confirmed zero and before any print/motion reservation, its ownership lease
is 3 s and the bridge retries transport failures for at most 2 s. The master blocks
actuation in this mode, and strict control must be acknowledged again before
preflight passes or Start heats/moves. The local robot worker retains its original
one-second lease and 0.5 s reply deadline throughout. Matching Bridge/Klipper
modules are required; this does not change clock-timing profiles or robot limits.
Neither software watchdogs nor preflight replace the physical stop circuit or
validate all external obstacles. Physical timing must be commissioned on the cell.

There is no mid-run pause/resume or adaptive speed slider support. With continuous
motion disabled, the legacy schedule stops at every source move and waits between
two-second chunks. Reduce the master's
linear/angular speed and acceleration limits and repeat preflight if the arm
cannot follow the schedule. Do not relax joint limits to conceal a timing error.

An error naming a G-code line concerns the master-timed print path. The report
includes the limiting joint, measured speed/acceleration and configured bound.
Adjust Klipper's `[axisforge_sync]` limits for that path; `approach_duration_s`
affects only the entry approach. The worker coalesces sub-nanosecond duplicate
phase timestamps caused by floating-point roundoff, preserving the master's
move duration and all phase boundaries representable by ROS timestamps.

## Validation

From the sourced AxisForge environment at the repository root:

```bash
python -m unittest discover -s src/print_control/test -p "test_sync*.py" -v
python -m unittest discover -s src/print_control/test -p "test_motion_planner.py" -v
```

The sync tests use actual UR16e IK to check rotated nozzle paths, exact master
times, continuous junction velocities, rejection of sharp-corner velocity jumps,
excessive speeds and coordinate-frame/hash errors. Worker
tests inject deadline, feedback, lease, cancellation and configuration failures.
Planner tests compare cached forward kinematics with every reference DH link
frame at random and singular joint poses, compare seeded IK on rotary targets,
and exercise fallback search, configured limits and geometry/acceleration errors.
The Bridge suite exercises the real Klipper extra against nonactuating heater/MCU
and robot doubles, including heater shutoff before IK and the two-phase commit.
These tests do not measure physical tracking accuracy or stop latency.
