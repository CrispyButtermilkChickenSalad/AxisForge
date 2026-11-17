"""UR Base -> tool0 using the same corrected transforms as ur_description."""
import hashlib
import math
from pathlib import Path

import roboticstoolbox as rtb
from spatialmath import SE3
import yaml

LINKS = ('shoulder', 'upper_arm', 'forearm', 'wrist_1', 'wrist_2', 'wrist_3')
NOMINAL_MODEL = {'kind': 'nominal_ur16e'}


def calibrated_model(path, limits_deg):
    """Build the six calibrated Rz joints and the fixed wrist_3 -> tool0 link.

    UR's `base` and `base_link_inertia` have the same transform: their two
    base_link rotations cancel. Calibration values are metres and radians.
    No controller TCP, payload, or installation setting is changed.
    """
    raw = Path(path).expanduser().read_bytes()
    data = yaml.safe_load(raw)
    values = data.get('kinematics') if isinstance(data, dict) else None
    if (not isinstance(values, dict) or set(values) != {*LINKS, 'hash'} or
            not isinstance(values['hash'], str) or not values['hash'].startswith('calib_')):
        raise ValueError('Invalid UR kinematics calibration file')
    chain, links = rtb.ETS(), []
    for index, name in enumerate(LINKS):
        pose = values[name]
        if (not isinstance(pose, dict) or set(pose) != {'x','y','z','roll','pitch','yaw'} or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in pose.values())):
            raise ValueError('Invalid calibrated UR transform: '+name)
        origin = SE3.Trans(*[pose[k] for k in ('x','y','z')]) * SE3.RPY(
            [pose[k] for k in ('roll','pitch','yaw')], order='zyx')
        limit = math.radians(limits_deg[index])
        chain = chain * rtb.ET.SE3(origin) * rtb.ET.Rz(jindex=index, qlim=[-limit,limit])
        links.append(chain)
    # Fixed transforms from the installed ur_description ur_macro.xacro.
    tool = (SE3.RPY((0.,-math.pi/2,-math.pi/2), order='zyx') *
            SE3.RPY((math.pi/2,0.,math.pi/2), order='zyx'))
    chain = chain * rtb.ET.SE3(tool)
    links.append(chain)
    robot = rtb.ERobot(chain, name='UR16e calibrated')
    identity = {'kind':'ur_calibration', 'sha256':hashlib.sha256(raw).hexdigest(),
                'calibration_hash':values['hash']}
    return robot, chain, links, identity
