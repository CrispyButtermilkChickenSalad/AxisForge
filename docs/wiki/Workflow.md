# Workflow

1. **Calibrate the ALPS scale** (dashboard → load calibration, or
   `alps_load_calibration`). Stored in `alps_load_calibration.yaml`.
2. **Set the nozzle TCP** (dashboard TCP editor). Saving invalidates any prior
   preflight and requires re-probing if the TCP changed.
3. **Probe the bed.** Manual 3x3 paper probe: `bed_probe`. Automatic:
   `scripts/bed_probe_load_cell.sh` (load cell contact, slow final approach) or
   the dashboard **Home & probe bed**. Output: `bed_probe.yaml`
   (`bed_probe.simulated.yaml` for simulation is git-ignored).
4. **Prepare G-code** with A/B/C rotary words (migrate old files with
   `scripts/migrate_gcode_a_to_b.py`).
5. **Print**
   - Standalone: launch `ur_master.launch.py` and execute through the planner.
   - Klipper-synchronized: use the Bridge dashboard (upload → preflight →
     Start). See [synchronized jobs](../synchronized-jobs.md).
6. Stop/resume: see README *Stop and resume*.

## Tests

```bash
PYTHONPATH=src/print_control:src/extrusion_control python -m pytest src/print_control/test
```

Run inside the Docker `dev` container or a sourced ROS environment; some tests
need `control_msgs`, `ament_index_python` and the UR16e DH model.
