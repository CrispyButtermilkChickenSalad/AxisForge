"""Dashboard probing options and cached metadata for the replaceable active mesh."""
import math
from pathlib import Path
import yaml


def validate_thermal(value):
    if value is None:return None
    if not isinstance(value,dict) or set(value)!={'nozzle_target_c','bed_target_c','soak_s'}:
        raise ValueError('Supply nozzle target, bed target and soak duration')
    for key,low,high in (('nozzle_target_c',50.,300.),('bed_target_c',30.,150.),('soak_s',5.,1800.)):
        number=value[key]
        if type(number) not in (int,float) or not math.isfinite(number) or not (low<=number<=high or key!='soak_s' and number==0):
            raise ValueError('Invalid probing temperature or soak: '+key)
    return {key:float(number) for key,number in value.items()}


class SavedMesh:
    def __init__(self,config):
        self.config=config
        self.config_key=self.map_key=self.path=None
        self.value={'available':False}

    @staticmethod
    def key(path):
        s=path.stat()
        return (s.st_mtime_ns,s.st_size)

    def status(self):
        try:
            if self.config is None:return {'available':False}
            key=self.key(self.config)
            if key!=self.config_key:
                if key[1]>1048576:raise ValueError('Configuration is too large')
                params=yaml.safe_load(self.config.read_text()).get('gcode_interpreter',{}).get('ros__parameters',{})
                self.path=Path(params['bed_probe_file']).expanduser() if params.get('bed_probe_file') else None
                self.config_key,self.map_key=key,None
            if self.path is None:return {'available':False}
            key=self.key(self.path)
            if key!=self.map_key:
                if key[1]>2097152:raise ValueError('Mesh is too large')
                data=yaml.safe_load(self.path.read_text())
                self.value={'available':True,'complete':data.get('complete',True) is True and not data.get('simulated',False),
                    'stamp':str(data.get('stamp','unknown')),'points':len(data.get('points',[])),
                    'temperature':data.get('probing_temperature'),
                    'message':'A new successful mesh replaces this map; the previous map is backed up automatically.'}
                self.map_key=key
            return dict(self.value)
        except (OSError,ValueError,TypeError,AttributeError,KeyError,yaml.YAMLError):
            return {'available':False,'message':'No readable saved mesh; start a new homing and mesh routine.'}
