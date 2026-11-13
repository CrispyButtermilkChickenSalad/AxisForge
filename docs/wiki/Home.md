# AxisForge wiki

ROS 2 Humble workspace that drives a UR16e as a 6-DOF 3D-printing robot, with
extrusion on a Portenta (micro-ROS), an ALPS load cell for bed probing, and an
optional Klipper master via the
[Klipper-AxisForge Bridge](https://github.com/CrispyButtermilkChickenSalad/Klipper-AxisForge-Bridge).

| Page | Contents |
| --- | --- |
| [Architecture](Architecture.md) | Packages, nodes, data flow, frames |
| [Setup](Setup.md) | Docker and native install, `.env`, first run |
| [Workflow](Workflow.md) | Calibration, bed probing, standalone and Klipper-synchronized printing |
| [Troubleshooting](Troubleshooting.md) | Common failures |

Detailed references: [synchronized jobs](../synchronized-jobs.md),
[Klipper follower notes](../klipper-follower.md).

## Axis convention

X/Y/Z are linear mm; A/B/C are absolute degrees about X/Y/Z (orientation =
`RPY(roll+A, pitch+B, yaw+C)`). The earlier "A = pitch" convention is now **B**;
use `scripts/migrate_gcode_a_to_b.py` on old G-code.
