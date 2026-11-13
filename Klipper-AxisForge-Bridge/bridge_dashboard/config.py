"""Runtime configuration loaded from environment variables."""

from dataclasses import dataclass
import os
from pathlib import Path
from urllib.parse import urlsplit


def _web_url(value, name, default=None):
    """Accept only browser navigable HTTP(S) links from configuration."""
    url = value or default
    if url is None:
        return None
    parsed = urlsplit(url)
    if parsed.scheme not in ("http", "https") or not parsed.netloc:
        raise ValueError(f"{name} must be an absolute HTTP(S) URL")
    return url


def _printer_ui_url(moonraker_url):
    """Guess the conventional web UI host when Moonraker uses its default port."""
    parsed = urlsplit(moonraker_url)
    if parsed.port != 7125 or parsed.hostname is None:
        return None
    host = parsed.hostname
    if ":" in host:
        host = f"[{host}]"
    return f"{parsed.scheme}://{host}/"


@dataclass(frozen=True)
class Settings:
    moonraker_url: str
    moonraker_api_key: str | None
    rosbridge_url: str
    fluidd_url: str | None
    mainsail_url: str | None
    printer_ui_url: str | None
    alps_logger_url: str
    gcode_root: Path | None
    axisforge_config: Path | None
    launch_package: str
    launch_file: str
    ros2_command: str
    sync_klipper_delay_s: float = 10.0
    robot_sync_url: str = 'http://127.0.0.1:8081'
    heater_check_timeout_s: float = 300.0
    robot_connection_url: str | None = None
    robot_ip: str = '192.168.0.100'
    robot_driver_script: Path | None = None
    robot_calibration_file: Path | None = None

    @classmethod
    def from_env(cls):
        """Create settings without requiring hardware endpoints at startup."""
        gcode_root = os.getenv("AXISFORGE_GCODE_ROOT")
        config_path = os.getenv("AXISFORGE_CONFIG")
        moonraker_url = os.getenv("MOONRAKER_URL", "http://127.0.0.1:7125").rstrip("/")
        return cls(
            moonraker_url=moonraker_url,
            moonraker_api_key=os.getenv("MOONRAKER_API_KEY") or None,
            rosbridge_url=os.getenv("ROSBRIDGE_URL", "ws://127.0.0.1:9090"),
            fluidd_url=_web_url(os.getenv("FLUIDD_URL"), "FLUIDD_URL"),
            mainsail_url=_web_url(os.getenv("MAINSAIL_URL"), "MAINSAIL_URL"),
            printer_ui_url=_web_url(
                os.getenv("PRINTER_UI_URL"),
                "PRINTER_UI_URL",
                _printer_ui_url(moonraker_url),
            ),
            alps_logger_url=_web_url(
                os.getenv("ALPS_LOGGER_URL"),
                "ALPS_LOGGER_URL",
                "https://crispybuttermilkchickensalad.github.io/ALPS-logger/",
            ),
            gcode_root=Path(gcode_root).expanduser().resolve() if gcode_root else None,
            axisforge_config=Path(config_path).expanduser().resolve() if config_path else None,
            launch_package=os.getenv("AXISFORGE_LAUNCH_PACKAGE", "axisforge_bringup"),
            launch_file=os.getenv("AXISFORGE_LAUNCH_FILE", "ur_master.launch.py"),
            ros2_command=os.getenv("ROS2_COMMAND", "ros2"),
            sync_klipper_delay_s=float(os.getenv("SYNC_KLIPPER_DELAY_S", "10")),
            robot_sync_url=os.getenv('AXISFORGE_SYNC_URL', 'http://127.0.0.1:8081'),
            heater_check_timeout_s=float(os.getenv('HEATER_CHECK_TIMEOUT_S','300')),
            robot_connection_url=os.getenv('AXISFORGE_ROBOT_CONNECTION_URL') or None,
            robot_ip=os.getenv('ROBOT_IP', '192.168.0.100'),
            robot_driver_script=Path(os.environ['AXISFORGE_ROBOT_DRIVER_SCRIPT']).resolve()
                if os.getenv('AXISFORGE_ROBOT_DRIVER_SCRIPT') else None,
            robot_calibration_file=Path(os.environ['UR_CALIBRATION_FILE']).resolve()
                if os.getenv('UR_CALIBRATION_FILE') else None,
        )
