"""Upload -> leased heater check -> cold robot preflight -> master-timed job."""

import asyncio
import contextlib
import time

from bridge_dashboard.gcode_check import sha256_file
from bridge_dashboard.sync_clients import PROTOCOL, ParticipantError, sample_clock


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
        self.state, self.message, self.job = 'idle', '', None
        self.stage, self.progress, self.checks = '', 0., []
        self.capabilities = {'control_ready':False, 'reason':'Checking synchronized services'}
        self.last_capability_check = 0.
        self.temperature = {}
        self.heater_checks = {'extruder':True, 'heater_bed':True}

    def status(self):
        return {'state':self.state,'message':self.message,'job':self.job,
                'stage':self.stage,'progress':self.progress,'checks':self.checks,
                'temperatures':self.temperature,'busy':self.busy(),
                'heater_checks':dict(self.heater_checks),
                'armed':False,'armed_expires_in_s':None,
                **self.capabilities}

    def busy(self): return self.task is not None and not self.task.done()

    async def refresh_capabilities(self):
        if self.busy() or self.clock()-self.last_capability_check < 2.: return
        self.last_capability_check = self.clock()
        try:
            master,robot = await asyncio.gather(self.master.call('capabilities'),
                                               self.robot.call('capabilities'))
            compatible = master.get('protocol') == robot.get('protocol') == PROTOCOL
            # A stopped External Control program can still allow nonmoving preflight.
            supported = master.get('heater_check_options') is True
            ready = compatible and supported and master.get('control_ready') and robot.get('enabled')
            self.capabilities = {'control_ready':bool(ready),
                'configured_heaters':sorted(master.get('heaters', {})),
                'robot_ready':bool(robot.get('control_ready')),
                'reason':('' if ready else 'Update the Klipper sync module to support heater check options'
                          if not supported else 'Install/enable matching Klipper and AxisForge sync services'),
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
                  'heater_checks':dict(self.record['heater_checks']) if valid else None}
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

    async def _heartbeat(self):
        async def renew(participant, lease_flag, label):
            while True:
                if getattr(self,lease_flag):
                    try:
                        result = await asyncio.wait_for(
                            participant.call('heartbeat',session=self.session),.5)
                    except asyncio.TimeoutError as error:
                        raise ParticipantError(label+' heartbeat response timed out') from error
                    if result.get('state') == 'fault':
                        raise ParticipantError(result.get('error','Participant fault'))
                await asyncio.sleep(min(.15,self.poll_s))

        # A slow Klipper reply must not delay renewal of the loopback robot lease.
        tasks = [asyncio.create_task(renew(self.master,'master_leased','Klipper')),
                 asyncio.create_task(renew(self.robot,'robot_leased','Robot'))]
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
        self.heartbeat_task = asyncio.create_task(self._heartbeat())

    async def _end_heartbeat(self):
        task, self.heartbeat_task = self.heartbeat_task, None
        if task and task is not asyncio.current_task():
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError): await task

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

    async def preflight(self, job, heater_checks=None):
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
            self.job,self.state,self.stage,self.progress,self.message = job,'preflighting','host',0.,'Checking Klipper'
            self.checks, self.temperature = [], {}
            self.task = asyncio.create_task(self._preflight(job,sha256_file(path)))
            return self.status()

    async def _preflight(self, job, sha):
        try:
            prepared = await self.master.call('prepare',job=job,sha256=sha,
                                               heater_checks=dict(self.heater_checks))
            self.session,self.plan = prepared['session'],prepared['plan']
            if self.plan.get('heater_checks') != self.heater_checks:
                raise ParticipantError('Klipper did not apply the heater check options; update the sync module')
            await self.master.call('acquire',session=self.session)
            self.master_leased = True
            await self._begin_heartbeat()
            self._check('klipper','File and motion plan','Klipper verified the file and generated the master timeline')
            mapping = await self.clock_sampler(self.master)
            self._check('klipper','Clock mapping','Measured uncertainty %.2f ms' % (mapping.uncertainty*1000))
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
            self.stage,self.progress,self.message = 'robot',.35,'Validating the Klipper timeline on the robot model'
            self.robot_prepare_attempted = True
            await self.robot.call('prepare',session=self.session,plan=self.plan)
            self.robot_leased = True

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
            self._check('robot','Timed trajectory','IK, interpolation and joint limits passed without retiming')
            self.record = {'job':job,'sha256':sha,'checked_at':self.clock(),
                           'heater_checks':dict(self.heater_checks),
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

    async def start(self, job, heater_checks=None):
        async with self.lock:
            if self.busy(): raise ParticipantError('A job is already running')
            if self.view(job)['state'] != 'preflight_passed':
                raise ParticipantError('Run both preflights on the current G-code first')
            if heater_checks is not None and heater_checks != self.record['heater_checks']:
                raise ParticipantError('Heater check options changed; rerun preflight')
            master,robot = await asyncio.gather(self.master.call('capabilities'),self.robot.call('status'))
            if not master.get('control_ready') or not robot.get('control_ready'):
                raise ParticipantError(robot.get('feedback',{}).get('reason') or 'Synchronized services are not ready')
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
            await self.master.call('acquire',session=self.session,run=True)
            self.master_leased = True
            await self._begin_heartbeat()
            await self.robot.call('acquire',session=self.session)
            self.robot_leased = True
            # Reject network timing problems before heating or the entry approach.
            await self.clock_sampler(self.master)
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
                    self.state,self.stage,self.message = 'running','motion','Executing synchronized chunk %d' % (op['index']+1)
                    mapping = await self.clock_sampler(self.master)
                    reservation = await self.master.call('reserve',session=self.session,index=op['index'])
                    start = mapping.local_time(reservation['start_print_time'])
                    accepted = await self.robot.call('schedule',session=self.session,index=op['index'],
                        start_monotonic=start,clock_uncertainty_s=mapping.uncertainty)
                    if not accepted.get('accepted'): raise ParticipantError('Robot did not accept the timed chunk')
                    await self.master.call('commit',session=self.session,token=reservation['token'])

                    async def chunk_done():
                        robot,master = await asyncio.gather(self.robot.call('status'),self.master.call('status'))
                        for status in (robot,master):
                            if status['state']=='fault': raise ParticipantError(status['error'])
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
