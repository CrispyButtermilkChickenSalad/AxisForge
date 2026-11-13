# Setup

## Docker (recommended, Linux host)

```bash
cp .env.example .env   # if present; otherwise create .env (never commit it)
docker compose up -d --build dev
docker compose exec dev axisforge-build
docker compose --profile klipper up -d      # rosbridge, sync worker, dashboard
```

Minimal `.env`:

```ini
ROBOT_IP=<ur-controller-ip>
MOONRAKER_URL=http://<klipper-host>:7125
AXISFORGE_SYNC_ENABLE=1
LOCAL_UID=1000
LOCAL_GID=1000
```

Containers use host networking (ROS DDS and UR reverse ports). Recreate the
bridge after changing `.env`:
`docker compose --profile klipper up -d --no-deps --force-recreate bridge`.
Place the Bridge checkout at `Klipper-AxisForge-Bridge/`.

## Native install (Ubuntu 22.04)

1. Real-time kernel (Ubuntu Pro `pro enable realtime-kernel`), reboot.
2. ROS 2 Humble.
3. UR ROS 2 driver.
4. `python3 -m venv --system-site-packages .venv && pip install -r requirements.txt -c docker/constraints.txt`.
5. `scripts/install_ur_models.sh` (custom UR16e models).
6. `scripts/microros.sh` (micro-ROS agent for the Portenta).
7. `scripts/build.sh` then source `install/setup.bash`.

## First run

```bash
scripts/ur_ros2_driver.sh                    # terminal 1: UR driver
ros2 launch axisforge_bringup ur_master.launch.py   # terminal 2
scripts/open_dashboard.py                    # opens http://localhost:8080
```

Then, in the dashboard, **Setup & calibration → Connect robot**, power on the
arm manually, start External Control, select Remote Control, speed 100%.

Windows is not supported for ROS nodes: `fcntl`, `termios`, `os.killpg` are used.
