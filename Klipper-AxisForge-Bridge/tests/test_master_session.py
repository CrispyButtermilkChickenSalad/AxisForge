"""Test the actual Klipper module with non-actuating printer/MCU doubles."""

import hashlib
from pathlib import Path
import tempfile
import time
import threading
from unittest.mock import patch
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
            register_timer=lambda *args:None,completion=Completion,
            async_complete=lambda completion,value:completion.complete(value))
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


class Completion:
    def __init__(self): self.event=threading.Event()
    def complete(self,value): self.value=value; self.event.set()
    def wait(self):
        if not self.event.wait(5.): raise RuntimeError('Test planner did not finish')
        return self.value


class Config:
    def __init__(self,printer): self.printer=printer
    def get_printer(self): return self.printer
    def getboolean(self,name,default): return True if name=='enabled' else default
    def getfloat(self,name,default,**kwargs): return default
    def get(self,name,default): return default


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

    def test_cold_preflight_extends_only_nonactuating_lease_and_cannot_revive_it(self):
        call(self.master,'lease_mode',session=self.session,mode='cold_preflight')
        self.assertEqual(self.master.deadline,self.now+3.)
        self.now+=1.5
        self.master._watchdog(self.now)
        self.assertTrue(self.master.leased)
        reply=call(self.master,'heartbeat',session=self.session)
        self.assertEqual(reply['watchdog_seconds'],3.)
        self.now+=3.01
        with self.assertRaisesRegex(ValueError,'expired'):
            call(self.master,'heartbeat',session=self.session)
        self.assertFalse(self.master.leased)
        self.assertEqual(self.master.state,'fault')

    def test_cold_mode_rejects_actuation_and_restores_strict_deadline(self):
        call(self.master,'lease_mode',session=self.session,mode='cold_preflight')
        for action,values in (('heat',{'targets':{'extruder':200.}}),('fan',{'speed':1.}),
                              ('reserve',{'index':0}),('commit',{'token':'invalid'})):
            with self.subTest(action=action),self.assertRaisesRegex(ValueError,'Cold preflight forbids'):
                call(self.master,action,session=self.session,**values)
        self.assertFalse(self.printer.queued)
        self.assertEqual(self.printer.heater.target,0.)
        reply=call(self.master,'lease_mode',session=self.session,mode='strict')
        self.assertEqual(reply['watchdog_seconds'],1.)
        self.assertEqual(self.master.deadline,self.now+1.)
        call(self.master,'heat',session=self.session,targets={'extruder':200.})
        self.now+=1.01
        self.master._watchdog(self.now)
        self.assertEqual(self.master.state,'fault')
        self.assertEqual(self.printer.heater.target,0.)

    def test_cold_entry_rejects_hot_targets_started_print_pending_motion_and_queue(self):
        for attribute,value in (('print_started',True),('reservation',{'committed':False}),('next_chunk',1)):
            original=getattr(self.master,attribute);setattr(self.master,attribute,value)
            with self.subTest(attribute=attribute),self.assertRaisesRegex(ValueError,'non-actuating'):
                call(self.master,'lease_mode',session=self.session,mode='cold_preflight')
            setattr(self.master,attribute,original)
        self.printer.heater.target=200.
        with self.assertRaisesRegex(ValueError,'non-actuating'):
            call(self.master,'lease_mode',session=self.session,mode='cold_preflight')
        self.printer.heater.target=0.
        with patch.object(self.master.toolhead,'get_last_move_time',return_value=9999.):
            with self.assertRaisesRegex(ValueError,'queue is busy'):
                call(self.master,'lease_mode',session=self.session,mode='cold_preflight')

    def test_cold_invariant_violation_aborts_and_turns_heaters_off(self):
        call(self.master,'lease_mode',session=self.session,mode='cold_preflight')
        self.printer.heater.target=200.
        self.master._watchdog(self.now)
        self.assertEqual(self.master.error,'Cold preflight invariant violated')
        self.assertEqual(self.printer.heater.target,0.)
        self.assertFalse(self.master.leased)

    def test_strict_transition_rechecks_configuration_and_file(self):
        call(self.master,'lease_mode',session=self.session,mode='cold_preflight')
        with patch.object(self.master,'_fingerprint',return_value='changed'):
            with self.assertRaisesRegex(ValueError,'settings changed'):
                call(self.master,'lease_mode',session=self.session,mode='strict')
        self.path.write_text('changed')
        with self.assertRaisesRegex(ValueError,'Job changed'):
            call(self.master,'lease_mode',session=self.session,mode='strict')
        self.assertEqual(self.master.lease_mode,'cold_preflight')

    def test_heartbeat_ack_has_no_full_status_or_configuration_work(self):
        with patch.object(self.master,'_status',side_effect=AssertionError('full status')),\
             patch.object(self.master,'_fingerprint',side_effect=AssertionError('fingerprint')),\
             patch.object(self.master,'_clock',side_effect=AssertionError('MCU clock')):
            reply=call(self.master,'heartbeat',session=self.session)
        self.assertEqual(set(reply),{'state','session','leased','lease_mode','watchdog_seconds'})

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

    def test_continuous_run_is_prequeued_with_adjacent_extrusion_timestamps(self):
        call(self.master,'release',session=self.session)
        self.master.continuous_motion=True
        self.path.write_text('M109 S200\n'+ENTRY+'G1 X111 E.1\nG1 X112 E.2\n')
        prepared=call(self.master,'prepare',job=self.path.name,
                      sha256=hashlib.sha256(self.path.read_bytes()).hexdigest())
        self.session=prepared['session']
        call(self.master,'acquire',session=self.session)
        call(self.master,'heat',session=self.session,targets={'extruder':200.})
        reservation=call(self.master,'reserve',session=self.session,index=0)
        call(self.master,'commit',session=self.session,token=reservation['token'])
        chunks=[op for op in prepared['plan']['operations'] if op['kind']=='chunk']
        self.assertEqual(len(chunks),1)
        self.assertEqual(len(self.printer.queued),2)
        (t0,left),(t1,right)=self.printer.queued
        self.assertAlmostEqual(t1,t0+left.accel_t+left.cruise_t+left.decel_t)
        self.assertGreater(right.start_v,0.)
        self.assertEqual(left.end_pos,right.start_pos)
        self.assertFalse(self.printer.shutdowns)

    def test_slow_prequeue_aborts_even_if_commit_started_before_deadline(self):
        original=self.printer.extruder.process_move
        def slow_queue(stamp,move,index):
            original(stamp,move,index)
            self.now+=.8
        self.printer.extruder.process_move=slow_queue
        call(self.master,'heat',session=self.session,targets={'extruder':200.})
        reservation=call(self.master,'reserve',session=self.session,index=0)
        with self.assertRaisesRegex(ValueError,'deadline'):
            call(self.master,'commit',session=self.session,token=reservation['token'])
        self.assertTrue(self.printer.shutdowns)
        self.assertEqual(self.master.state,'fault')

    def test_preparation_yields_blocks_concurrent_ownership_and_rejects_changed_limits(self):
        from klipper_extras.axisforge_schedule import compile_job
        call(self.master,'release',session=self.session)
        entered,finish=threading.Event(),threading.Event()
        errors=[]
        def slow(*args):
            entered.set()
            if not finish.wait(3.): raise RuntimeError('Test timed out')
            return compile_job(*args)
        def prepare():
            try:
                call(self.master,'prepare',job=self.path.name,
                     sha256=hashlib.sha256(self.path.read_bytes()).hexdigest())
            except Exception as error: errors.append(str(error))
        with patch('klipper_extras.axisforge_sync.compile_job',side_effect=slow):
            thread=threading.Thread(target=prepare)
            thread.start()
            try:
                self.assertTrue(entered.wait(2.))
                self.assertTrue(self.master.preparing)
                self.assertIn('print_time',call(self.master,'clock'))
                with self.assertRaisesRegex(ValueError,'being prepared'):
                    call(self.master,'prepare')
                with self.assertRaisesRegex(ValueError,'being prepared'):
                    call(self.master,'acquire',session=self.session)
                self.printer.objects['toolhead'].get_max_velocity=lambda:(10.,10.)
            finally:
                finish.set(); thread.join(3.)
        self.assertEqual(len(errors),1)
        self.assertIn('settings changed during preparation',errors[0])
        self.assertFalse(self.master.preparing)
        self.assertFalse(self.printer.queued)


if __name__=='__main__': unittest.main()
