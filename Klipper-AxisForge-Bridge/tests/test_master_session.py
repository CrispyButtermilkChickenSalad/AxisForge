"""Test the actual Klipper module with non-actuating printer/MCU doubles."""

import hashlib
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest

from klipper_extras.axisforge_sync import AxisForgeSync
from test_schedule import ENTRY


class Request:
    def __init__(self, **data): self.data,self.result=data,None
    def get(self,key,*default):
        return self.data.get(key,default[0]) if default else self.data[key]
    get_float=get_int=get_dict=get
    def error(self,message): return ValueError(message)
    def send(self,value): self.result=value


class Heater:
    min_temp,max_temp,min_extrude_temp=0.,300.,170.
    def __init__(self): self.target,self.actual=0.,25.; self.follows=True
    @property
    def can_extrude(self): return self.actual>=170.
    def set_temp(self,value):
        self.target=value
        if self.follows and value: self.actual=value
    def get_temp(self,now): return self.actual,self.target


class Printer:
    def __init__(self, root, clock=None):
        self.clock=clock or time.monotonic
        self.commands={}
        self.shutdowns=[]
        self.queued=[]
        self.heater=Heater()
        self.settings={'printer':{'kinematics':'none'},'virtual_sdcard':{'path':str(root)},
                       'extruder':{},'axisforge_sync':{}}
        self.reactor=SimpleNamespace(monotonic=self.clock,NOW=0.,
            register_timer=lambda *args:None)
        self.extruder=SimpleNamespace(heater=self.heater,last_position=0.,
            max_e_velocity=20.,max_e_accel=100.,max_e_dist=50.,max_extrude_ratio=5.,
            get_status=lambda now:{'pressure_advance':.04},
            check_move=self.check_move,process_move=self.process_move)
        self.objects={'webhooks':SimpleNamespace(register_endpoint=lambda *args:None),
            'toolhead':SimpleNamespace(get_max_velocity=lambda:(100.,100.),
                                      get_last_move_time=lambda:0.,_advance_move_time=lambda t:None,
                                      commanded_pos=[0.,0.,0.,0.]),
            'extruder':self.extruder,'mcu':SimpleNamespace(estimated_print_time=lambda t:t+1000.),
            'motion_queuing':SimpleNamespace(note_mcu_movequeue_activity=lambda t:None),
            'configfile':SimpleNamespace(get_status=lambda now:{'settings':self.settings}),
            'heaters':SimpleNamespace(lookup_heater=lambda name:self.heater),
            'virtual_sdcard':SimpleNamespace(is_active=lambda:False),
            'gcode':SimpleNamespace(register_command=self.register_command)}
        self.objects['gcode_move']=SimpleNamespace(reset_last_position=lambda:None)

    def register_command(self,name,callback):
        previous=self.commands.pop(name,None)
        if callback is not None: self.commands[name]=callback
        return previous
    def lookup_object(self,name,*default):
        return self.objects.get(name,default[0]) if default else self.objects[name]
    def register_event_handler(self,*args): pass
    def get_reactor(self): return self.reactor
    def invoke_shutdown(self,reason): self.shutdowns.append(reason)
    def check_move(self,move,index):
        if not self.heater.can_extrude: raise ValueError('Cold extruder')
    def process_move(self,stamp,move,index):
        self.queued.append((stamp,move))
        self.extruder.last_position=move.end_pos[3]


class Config:
    def __init__(self,printer): self.printer=printer
    def get_printer(self): return self.printer
    def getboolean(self,name,default): return True if name=='enabled' else default
    def getfloat(self,name,default,**kwargs): return default


def make_master(root,clock=None):
    printer=Printer(root,clock)
    master=AxisForgeSync(Config(printer))
    master._ready()
    return master,printer


def call(master,action,**data):
    request=Request(**data)
    getattr(master,'api_'+action)(request)
    return request.result


class MasterTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name)/'test.gcode'
        self.path.write_text('M109 S200\n'+ENTRY+'G1 X111 E.1\nM104 S0\n')
        self.now=100.
        self.master,self.printer=make_master(self.path.parent,lambda:self.now)
        self.prepared=call(self.master,'prepare',job=self.path.name,
                           sha256=hashlib.sha256(self.path.read_bytes()).hexdigest())
        self.session=self.prepared['session']
        call(self.master,'acquire',session=self.session)

    def test_reservation_never_queues_before_commit(self):
        call(self.master,'heat',session=self.session,targets={'extruder':200.})
        reservation=call(self.master,'reserve',session=self.session,index=0)
        self.assertFalse(self.printer.queued)
        call(self.master,'commit',session=self.session,token=reservation['token'])
        self.assertEqual(len(self.printer.queued),1)
        stamp,move=self.printer.queued[0]
        self.assertEqual(stamp,reservation['start_print_time'])
        self.assertAlmostEqual(move.end_pos[3]-move.start_pos[3],.1)
        with self.assertRaises(ValueError):
            call(self.master,'commit',session=self.session,token=reservation['token'])

    def test_expired_lease_switches_heaters_off_without_motion(self):
        call(self.master,'heat',session=self.session,targets={'extruder':200.})
        self.now+=2
        self.master._watchdog(self.now)
        self.assertEqual(self.printer.heater.target,0.)
        self.assertEqual(self.master.state,'fault')
        self.assertFalse(self.printer.queued)

    def test_committed_fault_shutdowns_mcu_instead_of_dropping_host_queue(self):
        call(self.master,'heat',session=self.session,targets={'extruder':200.})
        reservation=call(self.master,'reserve',session=self.session,index=0)
        call(self.master,'commit',session=self.session,token=reservation['token'])
        call(self.master,'abort',session=self.session,reason='robot lost')
        self.assertEqual(len(self.printer.shutdowns),1)
        self.assertEqual(self.printer.heater.target,0.)

    def test_late_commit_and_replayed_ack_are_rejected(self):
        call(self.master,'heat',session=self.session,targets={'extruder':200.})
        reservation=call(self.master,'reserve',session=self.session,index=0)
        self.now+=.8
        with self.assertRaisesRegex(ValueError,'deadline'):
            call(self.master,'commit',session=self.session,token=reservation['token'])
        self.assertFalse(self.printer.queued)

    def test_changed_file_invalidates_acquire(self):
        call(self.master,'release',session=self.session)
        self.path.write_text('changed')
        with self.assertRaisesRegex(ValueError,'changed'):
            call(self.master,'acquire',session=self.session)

    def test_unauthorized_target_and_early_ack_fail(self):
        with self.assertRaisesRegex(ValueError,'authorized'):
            call(self.master,'heat',session=self.session,targets={'extruder':250.})
        call(self.master,'heat',session=self.session,targets={'extruder':200.})
        reservation=call(self.master,'reserve',session=self.session,index=0)
        call(self.master,'commit',session=self.session,token=reservation['token'])
        with self.assertRaisesRegex(ValueError,'not finished'):
            call(self.master,'ack',session=self.session,token=reservation['token'])


if __name__=='__main__': unittest.main()
