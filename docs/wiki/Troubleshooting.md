# Troubleshooting

| Symptom | Fix |
| --- | --- |
| `roboticstoolbox.models.DH has no attribute UR16e` | Run `scripts/install_ur_models.sh`; pip's package lacks the custom model. |
| IK tolerance failures | Verify the UR16e model install and TCP offset. |
| Robot won't move | Another owner holds `.axisforge-robot-control.lock`; check External Control, Remote Control, 100% speed. |
| Dashboard can't see Klipper | Fix `MOONRAKER_URL` in `.env`, recreate the bridge container; add this host to Moonraker `trusted_clients`. |
| Probe rejects start | Nozzle must be upright, clean, cool (<50 °C), unloaded, heaters at zero. |
| Old G-code tilts wrongly | Old `A` was pitch; run `scripts/migrate_gcode_a_to_b.py`. |
