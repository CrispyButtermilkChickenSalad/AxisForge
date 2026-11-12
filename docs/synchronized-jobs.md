# Klipper-timed robot worker

`print_control.sync_runtime` provides the loopback `axisforge_sync_worker` service.
It consumes the `axisforge.sync.v1` schedules produced by the matching
[Klipper-AxisForge-Bridge](https://github.com/CrispyButtermilkChickenSalad/Klipper-AxisForge-Bridge).
The source schedule is in G-code bed coordinates. The worker applies AxisForge's
bed correction, XYZ offsets, nozzle orientation and TCP exactly once, solves IK,
and validates cubic-interpolated joint speed/acceleration and tool pose at the
master's times. An infeasible schedule is rejected; it is never silently slowed.

## Run

Build the workspace and install the repository's UR16e models as in the main
README. Run from the AxisForge root so relative bed probe paths resolve:

```bash
ros2 run print_control axisforge_sync_worker \
  --config src/axisforge_bringup/config/ur_master.yaml --port 8081 --enable-motion
```

Without `--enable-motion`, nonmoving plan validation works but motion is rejected.
Docker's `sync-worker` service translates `AXISFORGE_SYNC_ENABLE=1` to that flag.
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

At each chunk boundary, Klipper reserves a future start, the worker gets controller
acceptance, and only then does Klipper queue matching extrusion. Start times use
a bounded mapping of Klipper print_time to this host's monotonic clock. No timestamp
is inferred from percentage progress or the presumed duration of a separate job.

The worker requires a 1-second lease, fresh feedback within 250 ms, speed scaling
100%, NORMAL safety and a running UR program. It monitors joint tracking to
2 degrees, endpoint error to 0.2 degrees, completion deadlines and ROS clock jumps.
A failure cancels the action and uses dashboard Stop if cancellation fails. The
Bridge also has a separate rosbridge Stop path for worker failure. Klipper has its
own watchdog and shuts down if already committed extrusion must be aborted.
Neither software watchdogs nor preflight replace the physical stop circuit or
validate all external obstacles. Physical timing must be commissioned on the cell.

There is no mid-chunk pause/resume or adaptive speed slider support. This v1
schedule stops at every source move and waits between chunks. Reduce the master's
linear/angular speed and acceleration limits and repeat preflight if the arm
cannot follow the schedule. Do not relax joint limits to conceal a timing error.

## Validation

From the sourced AxisForge environment at the repository root:

```bash
python -m unittest discover -s src/print_control/test -p "test_sync*.py" -v
```

The sync tests use actual UR16e IK to check rotated nozzle paths, exact master
times, rejection of excessive speeds and coordinate-frame/hash errors. Worker
tests inject deadline, feedback, lease, cancellation and configuration failures.
The Bridge suite exercises the real Klipper extra against nonactuating heater/MCU
and robot doubles, including heater shutoff before IK and the two-phase commit.
These tests do not measure physical tracking accuracy or stop latency.
