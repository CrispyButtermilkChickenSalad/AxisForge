"""Klipper master for acknowledged, timestamped robot/extruder chunks.

Requires kinematics:none and the motion_queuing/extra-axis Klipper API. This
module owns the G-code job; virtual_sdcard must not execute a second copy.
"""

import hashlib
import inspect
import math
from pathlib import Path
import secrets

from .axisforge_schedule import PROTOCOL, compile_job, digest


class ScheduledExtrusion:
    """Present one preplanned trapezoid to Klipper's extruder implementation."""

    def __init__(self, record, e_origin):
        p = record['profile']
        xyz = [b-a for a, b in zip(record['start'][:3], record['end'][:3])]
        distance = math.sqrt(sum(x*x for x in xyz)) or abs(record['delta_e']) or 1.
        self.axes_d = xyz + [record['delta_e']]
        self.axes_r = [x/distance for x in self.axes_d]
        self.start_pos = record['start'][:3] + [record['e_start']+e_origin]
        self.end_pos = record['end'][:3] + [record['e_start']+record['delta_e']+e_origin]
        self.move_d = distance
        self.start_v, self.cruise_v = 0., p['velocity']*distance
        self.accel = p['acceleration']*distance
        self.accel_t = self.decel_t = p['ramp_s']
        self.cruise_t = p['cruise_s']

    def limit_speed(self, speed, accel):
        if self.cruise_v > speed+1e-8 or self.accel > accel+1e-8:
            raise ValueError('Klipper extrusion limits changed after preflight')


class AxisForgeSync:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.enabled = config.getboolean('enabled', False)
        self.linear_v = config.getfloat('max_velocity', 20., above=0.)
        self.linear_a = config.getfloat('max_accel', 20., above=0.)
        self.angular_v = config.getfloat('max_angular_velocity', 10., above=0.)
        self.angular_a = config.getfloat('max_angular_accel', 20., above=0.)
        self.chunk_seconds = config.getfloat('chunk_seconds', 2., minval=.1, maxval=10.)
        self.lease_s = config.getfloat('watchdog_seconds', 1., minval=.3, maxval=2.)
        self.lead_s = config.getfloat('start_lead_seconds', 1., minval=.5, maxval=5.)
        self.max_heat_s = config.getfloat('heater_timeout_seconds', 300., minval=10., maxval=900.)
        self.session = self.plan = self.reservation = None
        self.state, self.error = 'idle', ''
        self.leased = False
        self.deadline = self.heat_deadline = 0.
        self.next_chunk = 0
        self.e_origin = 0.
        self.toolhead = self.extruder = self.mcu = self.mq = None
        self.heaters = {}
        self.heater_config = {}
        self.config_fingerprint = None
        self.runtime_baseline = None
        self.job = self.file_hash = None
        self.print_started = False
        hooks = self.printer.lookup_object('webhooks')
        for name in ('capabilities', 'clock', 'prepare', 'acquire', 'heartbeat',
                     'heat', 'heaters_off', 'fan', 'reserve', 'commit', 'ack',
                     'release', 'complete', 'abort', 'status'):
            hooks.register_endpoint('axisforge_sync/'+name, getattr(self, 'api_'+name))
        self.printer.register_event_handler('klippy:ready', self._ready)
        self.printer.register_event_handler('klippy:shutdown', self._shutdown)
        self.printer.register_event_handler('gcode:request_restart', self._restart)
        self.reactor.register_timer(self._watchdog, self.reactor.NOW)

    def _ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.extruder = self.printer.lookup_object('extruder', None)
        self.mcu = self.printer.lookup_object('mcu')
        self.mq = self.printer.lookup_object('motion_queuing', None)
        cfg = self.printer.lookup_object('configfile').get_status(self.reactor.monotonic())['settings']
        self.kinematics = cfg.get('printer', {}).get('kinematics')
        self.sdroot = Path(cfg['virtual_sdcard']['path']).expanduser().resolve()
        manager = self.printer.lookup_object('heaters')
        for name in ('extruder', 'heater_bed'):
            if name in cfg:
                heater = manager.lookup_heater(name)
                self.heaters[name] = heater
                self.heater_config[name] = {'min_temp':heater.min_temp, 'max_temp':heater.max_temp}
        self.config_fingerprint = digest({
            'printer':cfg.get('printer'), 'extruder':cfg.get('extruder'),
            'heater_bed':cfg.get('heater_bed'), 'sync':cfg.get('axisforge_sync'),
        })
        self.runtime_baseline = self.config_fingerprint
        gcode = self.printer.lookup_object('gcode')
        # Do not permit a second planner or heater owner while this job holds the lease.
        for name in ('G0', 'G1', 'G28', 'G92', 'M82', 'M83', 'M84', 'M104', 'M109',
                     'M140', 'M190', 'M106', 'M107', 'M220', 'M221', 'M23', 'M24',
                     'SDCARD_PRINT_FILE', 'SET_HEATER_TEMPERATURE', 'SET_PRESSURE_ADVANCE',
                     'G2', 'G3', 'G10', 'G11', 'G90', 'G91', 'SET_VELOCITY_LIMIT',
                     'SET_GCODE_OFFSET', 'RESTORE_GCODE_STATE', 'SET_KINEMATIC_POSITION',
                     'MANUAL_STEPPER', 'FORCE_MOVE', 'STEPPER_BUZZ', 'SET_STEPPER_ENABLE',
                     'SYNC_EXTRUDER_MOTION', 'ACTIVATE_EXTRUDER', 'SET_EXTRUDER_ROTATION_DISTANCE'):
            original = gcode.register_command(name, None)
            if original is not None:
                def guarded(gcmd, handler=original):
                    if self.leased:
                        raise gcmd.error('AxisForge owns this job; abort it before manual changes')
                    return handler(gcmd)
                gcode.register_command(name, guarded)
        for name in ('CANCEL_PRINT', 'PAUSE', 'TURN_OFF_HEATERS'):
            original = gcode.register_command(name, None)
            if original is not None:
                def stopping(gcmd, handler=original, command=name):
                    if self.leased:
                        self._abort('External '+command)
                        return
                    return handler(gcmd)
                gcode.register_command(name, stopping)

    def _compatible(self):
        return (self.enabled and self.toolhead is not None and self.kinematics == 'none'
                and self.mq is not None and self.extruder is not None
                and callable(getattr(self.extruder, 'process_move', None))
                and len(inspect.signature(self.extruder.process_move).parameters) == 3
                and callable(getattr(self.toolhead, '_advance_move_time', None)))

    def _fingerprint(self):
        if self.toolhead is None: return None
        return digest({'config':self.runtime_baseline,
                       'motion_limits':self.toolhead.get_max_velocity(),
                       'extruder':{key:self.extruder.get_status(self.reactor.monotonic()).get(key,0.)
                                   for key in ('pressure_advance','smooth_time')},
                       'extrusion_limits':[self.extruder.max_e_velocity,self.extruder.max_e_accel,
                                           self.extruder.max_e_dist,self.extruder.max_extrude_ratio]})

    def _require(self, request, lease=True):
        if not self._compatible():
            raise request.error('Enable axisforge_sync with kinematics:none and a compatible Klipper')
        if not self.session or request.get('session') != self.session:
            raise request.error('Unknown or expired job session')
        if lease and (not self.leased or self.reactor.monotonic() >= self.deadline):
            self._abort('Control lease expired')
            raise request.error('Control lease expired; rerun preflight')
        if self.state == 'fault':
            raise request.error(self.error)

    def _off(self):
        for heater in self.heaters.values():
            heater.set_temp(0.)
        self.heat_deadline = 0.

    def _abort(self, reason):
        committed = self.reservation is not None and self.reservation.get('committed', False)
        self._off()
        self.leased, self.state, self.error = False, 'fault', reason
        self.reservation = None
        if self.print_started:
            self.print_started = False
            stats = self.printer.lookup_object('print_stats', None)
            if stats is not None: stats.note_error(reason)
        # Already queued MCU steps cannot be reliably recalled by deleting host trapqs.
        # A Klipper shutdown is required for an immediate abort of committed extrusion.
        if committed:
            self.printer.invoke_shutdown('AxisForge synchronized job aborted: '+reason)

    def _shutdown(self):
        self.leased, self.state, self.error = False, 'fault', 'Klipper shutdown'

    def _restart(self, *args):
        if self.leased:
            self._abort('Klipper restart requested')

    def _watchdog(self, eventtime):
        if self.leased and eventtime >= self.deadline:
            self._abort('Bridge heartbeat lost')
        elif self.leased and self.heat_deadline and eventtime >= self.heat_deadline:
            self._abort('Heater operation timed out')
        return eventtime + .05

    def _clock(self):
        now = self.reactor.monotonic()
        return {'eventtime':now, 'print_time':self.mcu.estimated_print_time(now)}

    def _status(self):
        now = self.reactor.monotonic()
        temperatures = {name:{'temperature':heater.get_temp(now)[0],
                              'target':heater.get_temp(now)[1],
                              'can_extrude':getattr(heater, 'can_extrude', True)}
                        for name, heater in self.heaters.items()}
        return {'protocol':PROTOCOL, 'state':self.state, 'error':self.error,
                'session':self.session, 'job':self.job, 'leased':self.leased,
                'plan_hash':None if self.plan is None else self.plan['plan_hash'],
                'machine_config':self._fingerprint(), 'next_chunk':self.next_chunk,
                'temperatures':temperatures, 'reservation':self.reservation,
                'clock':self._clock() if self.mcu else None}

    def api_status(self, request):
        request.send(self._status())

    def get_status(self, eventtime):
        return self._status()

    def api_capabilities(self, request):
        request.send({'protocol':PROTOCOL, 'control_ready':self._compatible(),
                      'heater_check_options':True,
                      'machine_config':self._fingerprint(), 'heaters':self.heater_config,
                      'watchdog_seconds':self.lease_s, 'start_lead_seconds':self.lead_s,
                      'execution':'acknowledged_stop_to_stop_chunks'})

    def api_clock(self, request):
        if not self.mcu: raise request.error('Klipper is not ready')
        request.send(self._clock())

    def api_prepare(self, request):
        if not self._compatible():
            raise request.error('axisforge_sync unavailable: enable module, kinematics:none and supported Klipper')
        if self.leased or self.reservation:
            raise request.error('A job already owns the motion/heater lease')
        sd = self.printer.lookup_object('virtual_sdcard')
        if sd.is_active(): raise request.error('A virtual SD print is active')
        if self.toolhead.get_last_move_time() > self._clock()['print_time'] + .5:
            raise request.error('Klipper motion queue is busy')
        name = request.get('job')
        if not isinstance(name, str) or Path(name).name != name or not name.lower().endswith('.gcode'):
            raise request.error('Expected a G-code basename')
        path = (self.sdroot/name).resolve()
        if path.parent != self.sdroot or not path.is_file():
            raise request.error('Job is outside the G-code directory or missing')
        if path.stat().st_size > 32*1024*1024: raise request.error('G-code exceeds 32 MiB')
        content = path.read_bytes()
        file_hash = hashlib.sha256(content).hexdigest()
        if file_hash != request.get('sha256'): raise request.error('G-code copies differ')
        scale = request.get_float('scale', 1.)
        if not .01 <= scale <= 1.: raise request.error('Invalid schedule speed scale')
        linear_v, linear_a = self.toolhead.get_max_velocity()
        limits = {'linear_velocity':min(linear_v, self.linear_v)*scale,
                  'linear_acceleration':min(linear_a, self.linear_a)*scale*scale,
                  'angular_velocity':self.angular_v*scale,
                  'angular_acceleration':self.angular_a*scale*scale,
                  'extrusion_velocity':self.extruder.max_e_velocity*scale,
                  'extrusion_acceleration':self.extruder.max_e_accel*scale*scale,
                  'max_extrude_ratio':self.extruder.max_extrude_ratio,
                  'max_extrude_distance':self.extruder.max_e_dist,
                  'min_extrude_temp':self.extruder.heater.min_extrude_temp,
                  'pressure_advance':self.extruder.get_status(self.reactor.monotonic()).get('pressure_advance', 0.),
                  'chunk_seconds':self.chunk_seconds}
        try:
            plan = compile_job(content.decode('utf-8'), limits, self.heater_config,
                               request.get_dict('heater_checks', {}))
            if (any(op['kind']=='fan' and op['speed'] for op in plan['operations'])
                    and self.printer.lookup_object('fan', None) is None):
                raise ValueError('G-code requests a primary fan, but none is configured')
        except (ValueError, UnicodeError) as error:
            raise request.error(str(error))
        self.plan, self.job, self.file_hash = plan, name, file_hash
        self.prepared_fingerprint = self._fingerprint()
        self.session = secrets.token_hex(24)
        self.state, self.error, self.next_chunk = 'prepared', '', 0
        self.e_origin = self.extruder.last_position
        request.send({'session':self.session, 'plan':plan, 'sha256':file_hash,
                      'machine_config':self.prepared_fingerprint})

    def api_acquire(self, request):
        self._require(request, lease=False)
        if self.leased: raise request.error('Session is already acquired')
        if self.state != 'prepared' or self.next_chunk:
            raise request.error('A new preflight is required')
        if self.printer.lookup_object('virtual_sdcard').is_active():
            raise request.error('A virtual SD print is active')
        if self.toolhead.get_last_move_time() > self._clock()['print_time'] + .5:
            raise request.error('Klipper motion queue is busy')
        if self._fingerprint() != self.prepared_fingerprint:
            raise request.error('Klipper settings changed; rerun preflight')
        if hashlib.sha256((self.sdroot/self.job).read_bytes()).hexdigest() != self.file_hash:
            raise request.error('Job changed since preflight')
        self.leased = True
        self.e_origin = self.extruder.last_position
        if request.get('run',False):
            self.print_started = True
            stats=self.printer.lookup_object('print_stats',None)
            if stats is not None:
                stats.set_current_file(self.job)
                stats.note_start()
        self.deadline = self.reactor.monotonic()+self.lease_s
        request.send(self._status())

    def api_heartbeat(self, request):
        self._require(request)
        self.deadline = self.reactor.monotonic()+self.lease_s
        request.send(self._status())

    def api_heat(self, request):
        self._require(request)
        if self.reservation: raise request.error('Cannot change heaters during a motion chunk')
        targets = request.get_dict('targets')
        for name, target in targets.items():
            if name not in self.heaters or type(target) not in (float, int) or not math.isfinite(target):
                raise request.error('Invalid heater target')
            if target < 0 or target > self.plan['heater_targets'].get(name, 0):
                raise request.error('Target was not authorized by the G-code preflight')
        for name, target in targets.items(): self.heaters[name].set_temp(target)
        self.heat_deadline = self.reactor.monotonic()+self.max_heat_s if any(targets.values()) else 0.
        request.send(self._status())

    def api_heaters_off(self, request):
        self._require(request, lease=False)
        if self.reservation:
            self._abort('Heaters switched off during motion')
        else:
            self._off()
        request.send(self._status())

    def api_fan(self, request):
        self._require(request)
        if self.reservation: raise request.error('Fan changes require a chunk boundary')
        speed = request.get_float('speed')
        if not 0 <= speed <= 1: raise request.error('Fan speed must be 0..1')
        fan = self.printer.lookup_object('fan', None)
        if fan is None and speed: raise request.error('Primary fan is not configured')
        if fan is not None: fan.fan.set_speed_from_command(speed)
        request.send({'ok':True})

    def api_reserve(self, request):
        self._require(request)
        index = request.get_int('index')
        if self.reservation or index != self.next_chunk:
            raise request.error('Unexpected chunk or previous chunk not acknowledged')
        chunks = [op for op in self.plan['operations'] if op['kind']=='chunk']
        if not 0 <= index < len(chunks): raise request.error('Invalid chunk index')
        chunk = chunks[index]
        moves = [ScheduledExtrusion(m, self.e_origin) for m in chunk['moves'] if m['delta_e']]
        try:
            for move in moves: self.extruder.check_move(move, 3)
        except Exception as error:
            raise request.error(str(error))
        clock = self._clock()
        start = clock['print_time']+self.lead_s
        self.reservation = {'token':secrets.token_hex(24), 'index':index,
                            'start_print_time':start, 'end_print_time':start+chunk['duration_s'],
                            'committed':False}
        self.state = 'reserved'
        request.send(dict(self.reservation, clock=clock))

    def api_commit(self, request):
        self._require(request)
        reservation = self.reservation
        if not reservation or request.get('token') != reservation['token'] or reservation['committed']:
            raise request.error('Invalid or already committed reservation')
        if reservation['start_print_time']-self._clock()['print_time'] < .25:
            self._abort('Commit arrived too late')
            raise request.error('Missed commit deadline')
        chunk = next(op for op in self.plan['operations']
                     if op['kind']=='chunk' and op['index']==reservation['index'])
        try:
            # Mark before the first side effect; a partial queue write requires shutdown.
            reservation['committed'] = True
            for record in chunk['moves']:
                if record['delta_e']:
                    move = ScheduledExtrusion(record, self.e_origin)
                    self.extruder.check_move(move, 3)
                    self.extruder.process_move(reservation['start_print_time']+record['offset_s'], move, 3)
            self.mq.note_mcu_movequeue_activity(reservation['end_print_time'])
            self.toolhead._advance_move_time(reservation['end_print_time'])
        except Exception as error:
            self._abort('Failed to queue extrusion: '+str(error))
            raise request.error(str(error))
        self.heat_deadline = 0.  # Streaming heartbeat now owns the heater lease.
        self.state = 'running'
        request.send(self._status())

    def api_ack(self, request):
        self._require(request)
        reservation = self.reservation
        if not reservation or request.get('token') != reservation['token'] or not reservation['committed']:
            raise request.error('No committed chunk to acknowledge')
        if self._clock()['print_time'] < reservation['end_print_time']:
            raise request.error('Chunk has not finished on the Klipper clock')
        self.next_chunk += 1
        chunk = next(op for op in self.plan['operations']
                     if op['kind']=='chunk' and op['index']==reservation['index'])
        last = chunk['moves'][-1]
        # Reconcile ordinary Klipper G-code state before releasing ownership later.
        # This does not append a second XYZ or E move.
        self.toolhead.commanded_pos[:3] = last['end'][:3]
        self.toolhead.commanded_pos[3] = last['e_start']+last['delta_e']+self.e_origin
        self.printer.lookup_object('gcode_move').reset_last_position()
        self.reservation, self.state = None, 'prepared'
        request.send(self._status())

    def api_release(self, request):
        self._require(request)
        if self.reservation: raise request.error('Cannot release a pending motion chunk')
        self._off()
        self.leased = False
        request.send(self._status())

    def api_complete(self, request):
        self._require(request)
        count = sum(op['kind']=='chunk' for op in self.plan['operations'])
        if self.reservation or self.next_chunk != count:
            raise request.error('Not all motion chunks were acknowledged')
        self._off()
        self.leased, self.state = False, 'complete'
        self.print_started = False
        stats=self.printer.lookup_object('print_stats',None)
        if stats is not None: stats.note_complete()
        request.send(self._status())

    def api_abort(self, request):
        if request.get('session') != self.session: raise request.error('Unknown session')
        self._abort(request.get('reason', 'Operator abort'))
        request.send(self._status())


def load_config(config):
    return AxisForgeSync(config)
