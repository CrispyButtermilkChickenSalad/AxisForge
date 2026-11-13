"""Upload -> leased heater check -> cold robot preflight -> master-timed job."""

import asyncio
import contextlib
import time

from bridge_dashboard.gcode_check import sha256_file
from bridge_dashboard.sync_clients import PROTOCOL, ParticipantError, ParticipantTransportError, sample_clock
from bridge_dashboard.clock_settings import clock_options, profiles, speed_guidance, GUIDANCE_DISTANCE_MM
from klipper_extras.axisforge_corners import options as corner_options

HEARTBEAT_REPLY_S = .5
COLD_REPLY_WINDOW_S = 2.
COLD_LEASE_S = 3.


class Workflow:
    def __init__(self, master, robot, axisforge, *, heat_timeout=300., poll_s=.1,
                 clock=time.monotonic, clock_sampler=sample_clock, robot_stop=None,
                 heat_stable_s=1.):
        self.master, self.robot, self.axisforge = master, robot, axisforge
        self.heat_timeout, self.poll_s, self.clock = heat_timeout, poll_s, clock
        self.clock_sampler = clock_sampler
        self.robot_stop = robot_stop
        self.heat_stable_s = heat_stable_s
        self.motion_started = False
        self.lock = asyncio.Lock()
        self.task = self.heartbeat_task = None
        self.session = self.plan = self.record = None
        self.master_leased = self.robot_leased = False
        self.robot_prepare_attempted = False
        self.heartbeat_error = None
        self.master_heartbeat_lock = asyncio.Lock()
        self.master_lease_mode = 'strict'
        self.strict_lease_s = 1.
        self.last_master_confirmed_send = 0.
        self.heartbeat_stats = {}
        self.robot_heartbeat_stats = {}
        self.state, self.message, self.job = 'idle', '', None
        self.stage, self.progress, self.checks = '', 0., []
        self.capabilities = {'control_ready':False, 'reason':'Checking synchronized services'}
        self.last_capability_check = 0.
        self.temperature = {}
        self.heater_checks = {'extruder':True, 'heater_bed':True}
        self.motion_options = None
        self.clock_options = clock_options()
        self.clock_measurement = None
        self.clock_speed_guidance = None

    def status(self):
        return {'state':self.state,'message':self.message,'job':self.job,
                'stage':self.stage,'progress':self.progress,'checks':self.checks,
                'temperatures':self.temperature,'busy':self.busy(),
                'heater_checks':dict(self.heater_checks),
                'motion_options':self.motion_options,
                'clock_options':dict(self.clock_options),
                'clock_defaults':clock_options(), 'clock_profiles':profiles(),
                'clock_guidance_distance_mm':GUIDANCE_DISTANCE_MM,
                'clock_measurement':(dict(self.clock_measurement,
                    age_s=max(0.,self.clock()-self.clock_measurement['sampled_at']))
                    if self.clock_measurement else None),
                'clock_speed_guidance':self.clock_speed_guidance,
                'heartbeat':dict(self.heartbeat_stats, active=self.master_leased,
                    lease_mode=self.master_lease_mode,
                    last_success_age_s=(max(0.,self.clock()-self.heartbeat_stats['last_success_at'])
                        if 'last_success_at' in self.heartbeat_stats else None)),
                'robot_heartbeat':dict(self.robot_heartbeat_stats, active=self.robot_leased),
                'armed':False,'armed_expires_in_s':None,
                **self.capabilities}

    def busy(self): return self.task is not None and not self.task.done()

    def invalidate_preflight(self, reason):
        """Discard saved motion validation when a standalone calibration routine runs."""
        if self.busy(): raise ParticipantError('A synchronized operation is still running')
        self.record = self.plan = self.session = None
        self.robot_prepare_attempted = False
        self.state, self.stage, self.progress = 'idle', '', 0.
        self.message, self.checks = reason, []
        self.last_capability_check = 0.

    async def refresh_capabilities(self):
        if self.busy() or self.clock()-self.last_capability_check < 2.: return
        self.last_capability_check = self.clock()
        try:
            master,robot = await asyncio.gather(self.master.call('capabilities'),
                                               self.robot.call('capabilities'))
            compatible = master.get('protocol') == robot.get('protocol') == PROTOCOL
            # A stopped External Control program can still allow nonmoving preflight.
            supported = master.get('heater_check_options') is True
            clock_supported = robot.get('clock_options_supported') is True
            heartbeat_supported = master.get('heartbeat_protocol') == 2
            external_busy = robot.get('external_control_busy',False)
            ready = compatible and supported and clock_supported and heartbeat_supported and not external_busy and master.get('control_ready') and robot.get('enabled')
            self.capabilities = {'control_ready':bool(ready),
                'clock_options_supported':clock_supported,
                'configured_heaters':sorted(master.get('heaters', {})),
                'corner_policies':sorted(set(master.get('corner_policies',['exact'])) &
                                         set(robot.get('corner_policies',['exact']))),
                'motion_defaults':master.get('motion_defaults',corner_options()),
                'pressure_advance':master.get('pressure_advance'),
                'robot_ready':bool(robot.get('control_ready')),
                'reason':('' if ready else 'Robot is owned by bed probing or another AxisForge controller'
                          if external_busy else 'Update the Klipper sync module to support heater check options'
                          if not supported else 'Update the AxisForge worker to support clock timing options'
                          if not clock_supported else 'Update the Klipper module for cold-preflight heartbeat support'
                          if not heartbeat_supported else 'Install/enable matching Klipper and AxisForge sync services'),
                'robot_reason':robot.get('feedback',{}).get('reason','')}
        except Exception as error:
            self.capabilities = {'control_ready':False,'reason':str(error)}

    def view(self, job):
        path = self.axisforge.job_path(job)
        valid = (self.record and self.record['job']==job and
                 self.record['sha256']==sha256_file(path) and
                 self.clock()-self.record['checked_at'] < 900.)
        result = {'job':job,'state':'preflight_passed' if valid else 'staged',
                  'sha256':self.record['sha256'] if valid else None,'klipper':None,'robot':None,
                  'heater_checks':dict(self.record['heater_checks']) if valid else None,
                  'clock_options':dict(self.record['clock_options']) if valid else None,
                  'motion_options':self.record.get('motion_options') if valid else None}
        if valid:
            for kind in ('klipper','robot'):
                result[kind] = {'passed':True,'checks':[c for c in self.checks if c['participant']==kind]}
        if self.job==job and self.state in ('preflighting','failed'):
            result['state'] = self.state
        return result

    def ready_jobs(self, names):
        return [{'job':name,'sha256':self.record['sha256']} for name in names
                if self.view(name)['state']=='preflight_passed']

    def _check(self, participant, name, detail, status='pass'):
        self.checks.append({'participant':participant,'name':name,'status':status,'detail':detail})

    def _healthy(self):
        if self.heartbeat_error: raise ParticipantError(self.heartbeat_error)

    def _validate_master_ack(self, result, mode):
        if result.get('state') == 'fault':
            raise ParticipantError(result.get('error','Klipper session fault'))
        expected = COLD_LEASE_S if mode=='cold_preflight' else self.strict_lease_s
        if (result.get('session') != self.session or result.get('leased') is not True
                or result.get('lease_mode') != mode or result.get('watchdog_seconds') != expected):
            raise ParticipantError('Klipper did not acknowledge the expected heartbeat lease')

    async def _set_master_lease_mode(self, mode):
        # Serialize transitions with heartbeats. A late heartbeat can only renew
        # the server's current mode; it cannot restore an old, longer lease.
        async with self.master_heartbeat_lock:
            self._healthy()
            sent = self.clock()
            try:
                result = await asyncio.wait_for(
                    self.master.call('lease_mode',session=self.session,mode=mode),HEARTBEAT_REPLY_S)
                if self.clock()-sent > HEARTBEAT_REPLY_S: raise asyncio.TimeoutError()
            except asyncio.TimeoutError as error:
                raise ParticipantError('Klipper did not acknowledge '+mode+' lease within 500 ms') from error
            self._validate_master_ack(result,mode)
            self.master_lease_mode = mode
            self.last_master_confirmed_send = sent
            self.heartbeat_stats['retrying'] = False

    async def _renew_master_heartbeat(self):
        async with self.master_heartbeat_lock:
            mode = self.master_lease_mode
            sent = self.clock()
            remaining = COLD_REPLY_WINDOW_S-(sent-self.last_master_confirmed_send)
            if mode=='cold_preflight' and remaining <= 0:
                raise ParticipantError('Klipper cold-preflight heartbeat recovery exceeded 2 s')
            timeout = min(HEARTBEAT_REPLY_S,remaining) if mode=='cold_preflight' else HEARTBEAT_REPLY_S
            try:
                result = await asyncio.wait_for(
                    self.master.call('heartbeat',session=self.session),timeout)
                if self.clock()-sent > timeout:
                    raise ParticipantTransportError('Klipper heartbeat response timed out')
            except (asyncio.TimeoutError, ParticipantTransportError) as error:
                detail = str(error) or 'Klipper heartbeat response timed out'
                self.heartbeat_stats['last_transport_error'] = detail
                if mode=='cold_preflight' and self.clock()-self.last_master_confirmed_send < COLD_REPLY_WINDOW_S:
                    self.heartbeat_stats['cold_retries'] = self.heartbeat_stats.get('cold_retries',0)+1
                    self.heartbeat_stats['retrying'] = True
                    return
                raise ParticipantError(detail+(' (cold-preflight recovery exceeded 2 s)'
                                              if mode=='cold_preflight' else '')) from error
            finally:
                elapsed = max(0.,self.clock()-sent)*1000
                self.heartbeat_stats['last_reply_ms'] = elapsed
                self.heartbeat_stats['max_reply_ms'] = max(elapsed,self.heartbeat_stats.get('max_reply_ms',0.))
            self._validate_master_ack(result,mode)
            # Count the bounded window from the acknowledged request's SEND time,
            # conservatively leaving margin inside the remote three-second lease.
            self.last_master_confirmed_send = sent
            self.heartbeat_stats.update(last_success_at=self.clock(),retrying=False)

    async def _heartbeat(self):
        async def renew(participant, lease_flag, label):
            while True:
                if getattr(self,lease_flag):
                    if label=='Klipper':
                        await self._renew_master_heartbeat()
                    else:
                        sent = self.clock()
                        try:
                            result = await asyncio.wait_for(
                                participant.call('heartbeat',session=self.session),HEARTBEAT_REPLY_S)
                            if self.clock()-sent > HEARTBEAT_REPLY_S: raise asyncio.TimeoutError()
                        except asyncio.TimeoutError as error:
                            raise ParticipantError(label+' heartbeat response timed out') from error
                        finally:
                            elapsed = max(0., self.clock()-sent)*1000
                            self.robot_heartbeat_stats['last_reply_ms'] = elapsed
                            self.robot_heartbeat_stats['max_reply_ms'] = max(
                                elapsed,self.robot_heartbeat_stats.get('max_reply_ms',0.))
                        if result.get('state') == 'fault':
                            raise ParticipantError(result.get('error','Participant fault'))
                await asyncio.sleep(min(.15,self.poll_s))

        async def monitor_loop():
            while True:
                due = self.clock()+.05
                await asyncio.sleep(.05)
                lag = max(0.,self.clock()-due)*1000
                self.heartbeat_stats['max_event_loop_delay_ms'] = max(
                    lag,self.heartbeat_stats.get('max_event_loop_delay_ms',0.))

        # A slow Klipper reply must not delay renewal of the loopback robot lease.
        tasks = [asyncio.create_task(renew(self.master,'master_leased','Klipper')),
                 asyncio.create_task(renew(self.robot,'robot_leased','Robot')),
                 asyncio.create_task(monitor_loop())]
        try:
            await asyncio.gather(*tasks)
        except asyncio.CancelledError:
            raise
        except Exception as error:
            self.heartbeat_error = str(error)
            if self.task and not self.task.done(): self.task.cancel()
            else: await self._stop_participants(str(error))
        finally:
            for task in tasks: task.cancel()
            await asyncio.gather(*tasks,return_exceptions=True)

    async def _begin_heartbeat(self):
        self.heartbeat_error = None
        self.master_lease_mode = 'strict'
        self.last_master_confirmed_send = self.clock()
        self.heartbeat_stats = {'cold_retries':0,'retrying':False,'max_reply_ms':0.,
                                'max_event_loop_delay_ms':0.}
        self.robot_heartbeat_stats = {'max_reply_ms':0.}
        self.heartbeat_task = asyncio.create_task(self._heartbeat())

    async def _end_heartbeat(self):
        task, self.heartbeat_task = self.heartbeat_task, None
        if task and task is not asyncio.current_task():
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError): await task
        self.heartbeat_stats['retrying'] = False

    async def _stop_participants(self, reason):
        # Stop monitoring before intentional faults from abort responses can
        # cancel this cleanup and interrupt the other participant's stop RPC.
        await self._end_heartbeat()
        if not self.session: return []
        calls = [self.master.call('abort',session=self.session,reason=reason)]
        # Host/clock/heater failures happen before the robot knows this session.
        # Once prepare has been sent, abort even if its acknowledgement was lost.
        if self.robot_prepare_attempted:
            calls.append(self.robot.call('abort',session=self.session,reason=reason))
        if self.motion_started and self.robot_stop: calls.append(self.robot_stop())
        results = await asyncio.gather(*calls,return_exceptions=True)
        self.master_leased = self.robot_leased = False
        return [str(r) for r in results if isinstance(r,Exception)]

    async def _wait(self, predicate, timeout, detail):
        deadline = self.clock()+timeout
        while True:
            self._healthy()
            result = await predicate()
            if result: return result
            if self.clock() >= deadline: raise ParticipantError(detail+' timed out')
            await asyncio.sleep(self.poll_s)

    async def _heat(self, targets):
        await self.master.call('heat',session=self.session,targets=targets)
        stable_since = None

        async def ready():
            nonlocal stable_since
            status = await self.master.call('status')
            self.temperature = status['temperatures']
            if status['state']=='fault': raise ParticipantError(status['error'])
            reached = True
            for name,target in targets.items():
                sensor = self.temperature.get(name,{})
                actual = sensor.get('temperature')
                if sensor.get('target') != target:
                    raise ParticipantError('Heater target changed outside the synchronized job')
                if target:
                    reached &= actual is not None and abs(actual-target)<=3.
            if reached:
                stable_since = self.clock() if stable_since is None else stable_since
                return self.clock()-stable_since >= self.heat_stable_s
            stable_since = None
            return False
        await self._wait(ready,self.heat_timeout,'Heater verification')

    async def preflight(self, job, heater_checks=None, motion_options=None, clock_settings=None):
        try:
            selected=corner_options(motion_options) if motion_options is not None else None
            timing=clock_options(clock_settings)
        except ValueError as error: raise ParticipantError(str(error)) from error
        checks = {'extruder':True, 'heater_bed':True}
        if heater_checks is not None:
            if (not isinstance(heater_checks,dict) or set(heater_checks)-set(checks)
                    or any(type(v) is not bool for v in heater_checks.values())):
                raise ParticipantError('Invalid heater check options')
            checks.update(heater_checks)
        async with self.lock:
            if self.busy(): raise ParticipantError('A preflight or synchronized job is already running')
            if self.axisforge.status()['running'] or self.axisforge.status().get('planning_check_running'):
                raise ParticipantError('Standalone AxisForge is busy')
            path = self.axisforge.job_path(job)
            self.record = None
            self.session = self.plan = None
            self.motion_started = False
            self.robot_prepare_attempted = False
            self.heater_checks = checks
            self.motion_options = selected
            self.clock_options = timing
            self.clock_measurement = self.clock_speed_guidance = None
            self.job,self.state,self.stage,self.progress,self.message = job,'preflighting','host',0.,'Checking Klipper'
            self.checks, self.temperature = [], {}
            self.task = asyncio.create_task(self._preflight(job,sha256_file(path)))
            return self.status()

    async def _preflight(self, job, sha):
        try:
            master_caps = await self.master.call('capabilities')
            if (master_caps.get('heartbeat_protocol') != 2 or
                    master_caps.get('cold_preflight_watchdog_seconds') != COLD_LEASE_S):
                raise ParticipantError('Update the Klipper module for cold-preflight heartbeat support')
            self.strict_lease_s = master_caps['watchdog_seconds']
            robot_caps = await self.robot.call('capabilities')
            if robot_caps.get('external_control_busy'):
                raise ParticipantError('Robot is owned by bed probing or another AxisForge controller')
            if robot_caps.get('clock_options_supported') is not True:
                raise ParticipantError('Update the AxisForge worker to support clock timing options')
            extra={'motion_options':self.motion_options} if self.motion_options is not None else {}
            prepared = await self.master.call('prepare',job=job,sha256=sha,
                                               heater_checks=dict(self.heater_checks),**extra)
            self.session,self.plan = prepared['session'],prepared['plan']
            if self.plan.get('heater_checks') != self.heater_checks:
                raise ParticipantError('Klipper did not apply the heater check options; update the sync module')
            planned_options=self.plan.get('motion_options',corner_options())
            if self.motion_options is not None and planned_options!=self.motion_options:
                raise ParticipantError('Klipper did not apply the corner policy; update the sync module')
            self.motion_options=planned_options
            self.clock_speed_guidance = speed_guidance(self.plan,self.clock_options)
            if planned_options['policy']!='exact':
                robot_caps=await self.robot.call('capabilities')
                if planned_options['policy'] not in robot_caps.get('corner_policies',[]):
                    raise ParticipantError('Update the AxisForge worker for the selected corner policy')
            acquired = await self.master.call('acquire',session=self.session)
            self.master_leased = True
            self._validate_master_ack(acquired,'strict')
            await self._begin_heartbeat()
            # Close the race with a probe starting after the initial capability
            # check. Probing takes the robot lock, then freshly checks Klipper's
            # lease; preflight takes Klipper's lease, then checks the robot lock.
            if (await self.robot.call('capabilities')).get('external_control_busy'):
                raise ParticipantError('Robot is owned by bed probing or another AxisForge controller')
            self._check('klipper','File and motion plan','Klipper verified the file and generated the master timeline')
            self._check('klipper','Corner policy',planned_options['policy']+'; native Klipper pressure advance retained')
            mapping = await self._sample_clock()
            self._check('klipper','Clock mapping',
                'Measured uncertainty %.2f ms; limit %g ms; running alignment limit %g ms' %
                (mapping.uncertainty*1000,self.clock_options['max_uncertainty_ms'],
                 self.clock_options['max_alignment_error_ms']))
            guidance = self.clock_speed_guidance
            self._check('bridge','Clock / speed guidance',
                'Planned peak XYZ speed %.2f mm/s corresponds to %.3f mm travel at the alignment limit. '
                'Illustrative %.2f mm allowance suggests at most %.2f mm/s; speeds are unchanged. '
                'This is not measured tracking error or a certified safe speed.' %
                (guidance['peak_xyz_speed_mm_s'],guidance['timing_displacement_mm'],
                 guidance['guidance_distance_mm'],guidance['suggested_max_speed_mm_s']),
                'warn' if guidance['exceeds_guidance'] else 'pass')
            for warning in self.plan.get('warnings', []):
                self.checks.append({'participant':'klipper','name':'G-code interpretation',
                                    'status':'warn','detail':warning})
            self.stage,self.progress,self.message = 'heating',.15,'Checking the G-code heater targets'
            try:
                targets = {k:v for k,v in self.plan['heater_targets'].items()
                           if v>0 and self.heater_checks[k]}
                if targets: await self._heat(targets)
                requested = self.plan.get('requested_heater_targets', {})
                for name,label in (('extruder','Extruder heater'),('heater_bed','Bed heater')):
                    if not self.heater_checks[name]:
                        detail = ('Check disabled; unconfigured heater commands are ignored for this job'
                                  if name in self.plan.get('ignored_heaters', []) else
                                  'Check disabled; configured heater commands still run at Start')
                        self._check('klipper',label,detail,'skip')
                    elif name in targets:
                        self._check('klipper',label,'Reached and held %g C from G-code' % targets[name])
                    else:
                        detail = 'G-code target is off (0 C)' if name in requested else 'No heating requested in G-code'
                        self._check('klipper',label,detail,'skip')
            finally:
                # Never leave a nozzle at temperature while expensive IK runs.
                off = await self.master.call('heaters_off',session=self.session)
                if any(s['target'] != 0 for s in off['temperatures'].values()):
                    raise ParticipantError('Klipper did not confirm all heater targets are zero')
                self.temperature = off['temperatures']
            self._check('klipper','Heaters off','All heater targets are zero for robot preflight')
            await self._set_master_lease_mode('cold_preflight')
            self.stage,self.progress,self.message = 'robot',.35,'Validating the Klipper timeline on the robot model'
            self.robot_prepare_attempted = True
            robot = await self.robot.call('prepare',session=self.session,plan=self.plan,
                                         clock_options=dict(self.clock_options))
            self.robot_leased = True
            if robot.get('clock_options') != self.clock_options:
                raise ParticipantError('Robot did not apply the selected clock timing options')

            async def planned():
                robot = await self.robot.call('status')
                self.progress = .35+.6*robot.get('progress',0)
                if robot['state']=='fault': raise ParticipantError(robot['error'])
                return robot if robot['state']=='ready' else None
            robot = await self._wait(planned,1800.,'Robot preflight')
            if robot['plan_hash'] != self.plan['plan_hash']:
                raise ParticipantError('Robot validated a different master schedule')
            if sha256_file(self.axisforge.job_path(job)) != sha:
                raise ParticipantError('File changed during preflight')
            # Return to strict supervision and obtain an actual heartbeat reply
            # before releasing preflight ownership. No actuator command precedes
            # this acknowledgement. The master rechecks its file/configuration.
            await self._set_master_lease_mode('strict')
            await self._renew_master_heartbeat()
            await self._sample_clock()
            self._check('robot','Timed trajectory','IK, interpolation and joint limits passed without retiming')
            self.record = {'job':job,'sha256':sha,'checked_at':self.clock(),
                           'heater_checks':dict(self.heater_checks),
                           'motion_options':dict(self.motion_options),
                           'clock_options':dict(self.clock_options),
                           'machine_config':prepared['machine_config'],'robot_config':robot['config_hash'],
                           'plan_hash':self.plan['plan_hash']}
            # Stop the heartbeat before releasing to avoid a request racing the release.
            await self._end_heartbeat()
            await self.robot.call('release',session=self.session)
            self.robot_leased = False
            await self.master.call('release',session=self.session)
            self.master_leased = False
            self.state,self.stage,self.progress,self.message = 'ready','ready',1.,'Both preflights passed; heaters are off'
        except asyncio.CancelledError:
            self.record = None
            await self._stop_participants(self.heartbeat_error or 'Preflight cancelled')
            self.state,self.message = 'failed' if self.heartbeat_error else 'aborted', self.heartbeat_error or 'Preflight cancelled'
        except Exception as error:
            self.record = None
            failures = await self._stop_participants(str(error))
            self.state,self.message = 'failed',str(error)+(('; stop errors: '+ '; '.join(failures)) if failures else '')
        finally:
            await self._end_heartbeat()

    async def start(self, job, heater_checks=None, motion_options=None, clock_settings=None):
        async with self.lock:
            if self.busy(): raise ParticipantError('A job is already running')
            if self.view(job)['state'] != 'preflight_passed':
                raise ParticipantError('Run both preflights on the current G-code first')
            if heater_checks is not None and heater_checks != self.record['heater_checks']:
                raise ParticipantError('Heater check options changed; rerun preflight')
            if motion_options is not None and corner_options(motion_options)!=self.record.get('motion_options'):
                raise ParticipantError('Corner options changed; rerun preflight')
            if clock_settings is not None and clock_options(clock_settings)!=self.record['clock_options']:
                raise ParticipantError('Clock timing options changed; rerun preflight')
            master,robot = await asyncio.gather(self.master.call('capabilities'),self.robot.call('status'))
            if not master.get('control_ready') or not robot.get('control_ready'):
                raise ParticipantError(robot.get('feedback',{}).get('reason') or 'Synchronized services are not ready')
            if robot.get('clock_options') != self.record['clock_options']:
                raise ParticipantError('Robot clock timing options changed; rerun preflight')
            if (master['machine_config'] != self.record['machine_config'] or
                robot['config_hash'] != self.record['robot_config'] or
                robot['plan_hash'] != self.record['plan_hash']):
                self.record = None
                raise ParticipantError('Preflight inputs changed; run preflight again')
            self.state,self.stage,self.progress,self.message = 'starting','heating',0.,'Preparing synchronized job'
            self.task = asyncio.create_task(self._run())
            return self.status()

    async def _run(self):
        try:
            acquired = await self.master.call('acquire',session=self.session,run=True)
            self.master_leased = True
            self._validate_master_ack(acquired,'strict')
            await self._begin_heartbeat()
            await self._renew_master_heartbeat()
            await self.robot.call('acquire',session=self.session,clock_options=dict(self.clock_options))
            self.robot_leased = True
            # Reject network timing problems before heating or the entry approach.
            await self._sample_clock()
            approached = False
            operations = self.plan['operations']
            for index,op in enumerate(operations):
                self._healthy()
                self.progress = index/len(operations)
                if op['kind']=='heater':
                    self.stage,self.message = 'heating','Setting '+op['heater']+' to '+str(op['target'])+' °C'
                    # Always await positive targets; this adds a deterministic barrier for M104/M140 too.
                    if op['target']>0: await self._heat({op['heater']:op['target']})
                    else: await self.master.call('heat',session=self.session,targets={op['heater']:0})
                elif op['kind']=='fan':
                    await self.master.call('fan',session=self.session,speed=op['speed'])
                elif op['kind']=='dwell':
                    self.stage,self.message = 'dwell','Waiting at a motion boundary'
                    await asyncio.sleep(op['duration_s'])
                elif op['kind']=='chunk':
                    if not approached:
                        self.stage,self.message = 'approach','Moving the robot to the entry point'
                        self.motion_started = True
                        await self.robot.call('approach',session=self.session)
                        async def at_entry():
                            status = await self.robot.call('status')
                            if status['state']=='fault': raise ParticipantError(status['error'])
                            return status['state']=='at_boundary'
                        await self._wait(at_entry,180.,'Robot entry approach')
                        approached = True
                    label = 'run' if self.plan.get('motion_mode')=='continuous_runs' else 'chunk'
                    self.state,self.stage,self.message = 'running','motion','Executing synchronized %s %d' % (label,op['index']+1)
                    mapping = await self._sample_clock()
                    reservation = await self.master.call('reserve',session=self.session,index=op['index'])
                    start = mapping.local_time(reservation['start_print_time'])
                    accepted = await self.robot.call('schedule',session=self.session,index=op['index'],
                        start_monotonic=start,clock_uncertainty_s=mapping.uncertainty,
                        clock_options=dict(self.clock_options))
                    if not accepted.get('accepted'): raise ParticipantError('Robot did not accept the timed chunk')
                    await self.master.call('commit',session=self.session,token=reservation['token'])
                    last_clock_check = self.clock()

                    async def chunk_done():
                        nonlocal last_clock_check
                        # A long prequeued run keeps its original time mapping.
                        # Check drift while moving; never slide one participant's
                        # timestamps to hide disagreement with the other clock.
                        if self.clock()-last_clock_check >= 2.:
                            await self._check_running_clock(mapping)
                            last_clock_check = self.clock()
                        robot,master = await asyncio.gather(self.robot.call('status'),self.master.call('status'))
                        for status in (robot,master):
                            if status['state']=='fault': raise ParticipantError(status['error'])
                        fraction = min(1.,max(0.,(master['clock']['print_time']-
                                                  reservation['start_print_time'])/op['duration_s']))
                        self.progress = (index+fraction)/len(operations)
                        return (robot['state']=='at_boundary' and robot['next_chunk']==op['index']+1
                                and master['clock']['print_time']>=reservation['end_print_time'])
                    await self._wait(chunk_done,op['duration_s']+10.,'Synchronized chunk')
                    await self.master.call('ack',session=self.session,token=reservation['token'])
            await self._end_heartbeat()
            await self.robot.call('complete',session=self.session)
            self.robot_leased = False
            await self.master.call('complete',session=self.session)
            self.master_leased = False
            self.state,self.stage,self.progress,self.message = 'complete','complete',1.,'Both systems completed; heaters are off'
        except asyncio.CancelledError:
            await self._stop_participants(self.heartbeat_error or 'Operator abort')
            self.state,self.message = 'failed' if self.heartbeat_error else 'aborted',self.heartbeat_error or 'Job aborted'
        except Exception as error:
            failures = await self._stop_participants(str(error))
            self.state,self.message = 'failed',str(error)+(('; stop errors: '+'; '.join(failures)) if failures else '')
        finally:
            self.record = None  # Every run needs fresh preflight and a new session.
            await self._end_heartbeat()

    async def _sample_clock(self):
        current = await self.clock_sampler(self.master,
            max_uncertainty_s=self.clock_options['max_uncertainty_ms']/1000)
        self.clock_measurement = {'uncertainty_ms':current.uncertainty*1000,
            'alignment_error_ms':None, 'sampled_at':current.sampled_at}
        return current

    async def _check_running_clock(self, mapping):
        current = await self._sample_clock()
        if self.clock()-current.sampled_at > 5.:
            raise ParticipantError('Clock mapping is stale during synchronized motion')
        error = abs(current.offset-mapping.offset)+current.uncertainty
        self.clock_measurement['alignment_error_ms'] = error*1000
        limit = self.clock_options['max_alignment_error_ms']
        if error > limit/1000:
            raise ParticipantError('Clock alignment error %.2f ms exceeds %g ms during motion' % (error*1000,limit))

    async def abort(self, reason='Operator abort'):
        task = self.task
        if task and not task.done():
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError): await task
        else:
            await self._stop_participants(reason)
            self.state,self.message = 'aborted',reason
        self.record = None
        return self.status()

    async def close(self):
        if self.busy(): await self.abort('Dashboard shutting down')
        await self._end_heartbeat()
        await self.robot.close()
        await self.master.close()
