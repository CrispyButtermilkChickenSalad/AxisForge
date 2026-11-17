# Klipper–AxisForge Bridge

A local dashboard that runs synchronized 3D prints between a **Klipper/Moonraker**
host and an **AxisForge** UR16e robot. Klipper owns the XYZ/ABC/E timeline and
extrusion; the robot follows timestamped joint trajectories. A/B/C are absolute
degrees about X/Y/Z relative to the configured nozzle orientation (Klipper ignores
them).

**Run preflight** verifies the same file on both hosts, builds a Klipper schedule,
heat-tests the selected heaters (then sets all targets to zero), and validates robot
IK and timing without moving the arm. **Start synchronized job** reheats, approaches
the entry point and executes prequeued timed runs. Independent speed override and
mid-job resume are unsupported. Physical testing is still required for live tracking,
pressure advance and print quality. This is not a safety system: keep the UR pendant
and physical E-stop reachable.

## Documentation

Full documentation is in the **[wiki](https://github.com/CrispyButtermilkChickenSalad/Klipper-AxisForge-Bridge/wiki)**:

- [Architecture](https://github.com/CrispyButtermilkChickenSalad/Klipper-AxisForge-Bridge/wiki/Architecture)
- [Setup](https://github.com/CrispyButtermilkChickenSalad/Klipper-AxisForge-Bridge/wiki/Setup)
- [Bed probing and calibration](https://github.com/CrispyButtermilkChickenSalad/Klipper-AxisForge-Bridge/wiki/Bed-probing-and-calibration)
- [Workflow](https://github.com/CrispyButtermilkChickenSalad/Klipper-AxisForge-Bridge/wiki/Workflow)
- [Operations and recovery](https://github.com/CrispyButtermilkChickenSalad/Klipper-AxisForge-Bridge/wiki/Operations-and-recovery)
- [Dashboard reference](https://github.com/CrispyButtermilkChickenSalad/Klipper-AxisForge-Bridge/wiki/Dashboard-reference)
- [Troubleshooting](https://github.com/CrispyButtermilkChickenSalad/Klipper-AxisForge-Bridge/wiki/Troubleshooting)

Protocol, clock profiles, heartbeats, corner policies and pressure advance:
[docs/synchronized-jobs.md](docs/synchronized-jobs.md).

The [dashboard setup and job guide](docs/dashboard-guide.md) covers adjustable
homing and mesh speeds, 2×2 through 9×9 grids, bed dimensions and bounds,
ALPS calibration jogging, versioned uploads and start/end G-code compatibility.

## Quick start

Use matching versions of this repo and
[AxisForge](https://github.com/CrispyButtermilkChickenSalad/AxisForge), with this
repo at `AxisForge/Klipper-AxisForge-Bridge`.

1. **Klipper host** (needs `kinematics: none`): copy `scripts/install_sync.py` and
   `klipper_extras/` there, run
   `python3 scripts/install_sync.py --klipper-dir ~/klipper --config ~/printer_data/config/printer.cfg`,
   restart the Klipper service, and check
   `curl http://<klipper-host>:7125/printer/axisforge_sync/capabilities`
   shows `protocol: axisforge.sync.v1` and `control_ready: true`.
2. **Dashboard host**: in AxisForge's `.env` set
   `MOONRAKER_URL=http://<klipper-host>:7125` and `AXISFORGE_SYNC_ENABLE=1`, then:

   ```bash
   docker compose up -d --build dev
   docker compose exec dev axisforge-build
   docker compose --profile klipper up -d
   ```

3. **Robot**: start the UR driver and External Control, select Remote Control,
   set speed to 100%.
4. Open **http://127.0.0.1:8080**, then follow *Setup & calibration* and the
   G-code dashboard.

Native (non-Docker) install and all environment variables are in the wiki's Setup page.

## Tests

```bash
pip install -r requirements.txt pytest
python -m pytest tests --ignore-glob="tests/browser_*"
```

Run on Linux (several tests use POSIX-only calls). Automated tests do not establish
physical synchronization accuracy; robot tests live in AxisForge.

Clock-alignment validation is still incomplete; see the [current validation status](docs/synchronized-jobs.md#current-validation-status--17-november-2026).
