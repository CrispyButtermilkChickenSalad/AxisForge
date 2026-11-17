"""Bed XY coordinates anchored to a measured centre in the robot Base frame."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class BedFrame:
    center_base_mm: tuple
    size_mm: tuple
    yaw_deg: float = 0.

    def __post_init__(self):
        for name in ('center_base_mm', 'size_mm'):
            values=np.asarray(getattr(self,name),dtype=float)
            if values.shape!=(2,) or not np.all(np.isfinite(values)):
                raise ValueError('Bed frame '+name+' must contain two finite values')
            if name=='size_mm' and np.any(values<=0):
                raise ValueError('Bed frame dimensions must be positive')
        if type(self.yaw_deg) not in (int,float) or not math.isfinite(self.yaw_deg) or not -180<=self.yaw_deg<=180:
            raise ValueError('Bed direction must be -180..180 degrees')

    @property
    def rotation(self):
        angle=math.radians(self.yaw_deg);c,s=math.cos(angle),math.sin(angle)
        return np.array([[c,-s],[s,c]])

    def to_base(self, xy):
        return np.asarray(self.center_base_mm)+self.rotation@(np.asarray(xy)-np.asarray(self.size_mm)/2)

    def to_gcode(self, xy):
        return self.rotation.T@(np.asarray(xy)-np.asarray(self.center_base_mm))+np.asarray(self.size_mm)/2

    def as_dict(self):
        return {'source':'homing_center','center_base_mm':[float(v) for v in self.center_base_mm],
                'size_mm':[float(v) for v in self.size_mm],'yaw_deg':float(self.yaw_deg)}

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data,dict) or set(data)!={'source','center_base_mm','size_mm','yaw_deg'} or data['source']!='homing_center':
            raise ValueError('Invalid calibrated bed frame')
        return cls(tuple(data['center_base_mm']),tuple(data['size_mm']),data['yaw_deg'])
