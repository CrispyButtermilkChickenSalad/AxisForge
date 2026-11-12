#!/usr/bin/env python3
"""Install the Klipper sync extras and configuration, with a dated backup.

Run on the Klipper host after copying this script and klipper_extras/ together.
Does not restart Klipper or send any heater/motion commands.
"""
import argparse
import datetime
from pathlib import Path
import re
import shutil


def install(source, klipper, config):
    extras=klipper/'klippy/extras'
    if not extras.is_dir() or not config.is_file():
        raise ValueError('Check --klipper-dir and --config paths')
    modules=['axisforge_sync.py','axisforge_schedule.py']
    for name in modules:
        compile((source/name).read_text(),name,'exec')
    original=config.read_text()
    # Preserve an existing installation and operator-selected limits.
    section='''\n# Klipper owns synchronized robot/extrusion job timing.
[axisforge_sync]
enabled: True
max_velocity: 20
max_accel: 20
max_angular_velocity: 10
max_angular_accel: 20
chunk_seconds: 2
watchdog_seconds: 1
start_lead_seconds: 1
heater_timeout_seconds: 300

'''
    backup=config.parent/('axisforge-sync-backup-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    backup.mkdir()
    shutil.copy2(config,backup/config.name)
    for name in modules:
        if (extras/name).exists(): shutil.copy2(extras/name,backup/name)
        shutil.copy2(source/name,extras/name)
    if not re.search(r'^\s*\[axisforge_sync\]\s*$',original,re.M):
        marker='#*# <---------------------- SAVE_CONFIG'
        offset=original.find(marker)
        updated=original+section if offset<0 else original[:offset]+section+original[offset:]
        config.write_text(updated)
    print('Installed modules. Backup: '+str(backup))
    print('When idle with heaters off: sudo systemctl restart klipper; then check /printer/axisforge_sync/capabilities.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=Path(__file__).resolve().parents[1]/'klipper_extras')
    parser.add_argument('--klipper-dir',type=Path,default=Path.home()/'klipper')
    parser.add_argument('--config',type=Path,default=Path.home()/'printer_data/config/printer.cfg')
    args=parser.parse_args()
    install(args.source,args.klipper_dir,args.config)
