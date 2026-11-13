#!/usr/bin/env python3
"""Open the local bridge after desktop login; never start hardware motion."""
import argparse
from pathlib import Path
import time
import urllib.error
import urllib.request
import webbrowser


ROOT = Path(__file__).resolve().parents[1]


def dashboard_url():
    port = 8080
    env = ROOT/'.env'
    if env.is_file():
        for line in env.read_text().splitlines():
            key, _, value = line.partition('=')
            if key.strip() == 'BRIDGE_PORT':
                value = value.strip().strip('\"\'')
                if value.isdecimal() and 1 <= int(value) <= 65535: port = int(value)
    return 'http://127.0.0.1:%d/' % port


def install():
    # Explicit --install makes the desktop changes reproducible and removable.
    content = ('[Desktop Entry]\nType=Application\nName=AxisForge Bridge\n'
               'Comment=Open robot setup and the synchronized printing dashboard\n'
               'Exec=/usr/bin/python3 "'+str(Path(__file__).resolve())+'"\n'
               'Terminal=false\nCategories=Utility;\nX-GNOME-Autostart-enabled=true\n')
    for directory in (Path.home()/'.config/autostart', Path.home()/'.local/share/applications'):
        directory.mkdir(parents=True, exist_ok=True)
        (directory/'axisforge-bridge.desktop').write_text(content)
    print('Installed AxisForge Bridge menu entry and desktop-login autostart.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install', action='store_true')
    args = parser.parse_args()
    if args.install: return install()
    url, deadline = dashboard_url(), time.monotonic()+90.
    client = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    while time.monotonic() < deadline:
        try:
            with client.open(url+'api/jobs', timeout=2.): break
        except (urllib.error.URLError, TimeoutError): time.sleep(1.)
    webbrowser.open(url)


if __name__ == '__main__': main()
