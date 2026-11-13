"""Loopback robot worker for the Klipper master protocol; never drives extrusion."""

import argparse
from concurrent.futures import Future
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import threading
import time

import numpy as np

from print_control.motion_planner import JOINT_NAMES, cubic_joint_state
from print_control.sync_path import PROTOCOL, TimedPath, configuration
from print_control.sync_clock import clock_options
from print_control.robot_ownership import RobotControlLock


class RobotWorker:
    """One preflight/session/action at a time, with a local heartbeat watchdog."""

    def __init__(self, backend, config_path, enabled=False, clock=time.monotonic,
                 compiler=TimedPath, lease_s=1., control_lock=None):
        self.backend, self.config_path, self.enabled = backend, config_path, enabled
        self.clock, self.compiler, self.lease_s = clock, compiler, lease_s
        self.lock = threading.RLock()
        self.cancelled = threading.Event()
        self.compiled = self.session = self.goal = None
        self.clock_options = clock_options()
        self.state, self.error = 'idle', ''
        self.progress, self.next_chunk = 0., 0
        self.lease = False
        self.deadline = 0.
        self.generation = 0
        self.heartbeat_stats = {'count':0, 'max_lock_wait_ms':0., 'max_gap_ms':0.}
        self.last_heartbeat = None
        self.validation_deadline = None
        self.control_lock = control_lock
        self.stopping = False
        self.stop_unconfirmed = False
        self.thread = None
        self.stop = threading.Event()
        self.watchdog = threading.Thread(target=self._watchdog, daemon=True)
        self.watchdog.start()

    def status(self):
        feedback = self.backend.feedback()
        config_hash = configuration(self.config_path)[2]
        external_busy = self.control_lock.external_busy() if self.control_lock is not None else False
        with self.lock:
            external_busy = external_busy or self.stop_unconfirmed
            return {'protocol':PROTOCOL, 'enabled':self.enabled,
                    'corner_policies':['exact','klipper','marlin','blend'],
                    'clock_options_supported':True,
                    'clock_options':dict(self.clock_options),
                    'control_ready':self.enabled and feedback['ready'] and not external_busy,
                    'state':self.state, 'error':self.error, 'session':self.session,
                    'progress':self.progress, 'next_chunk':self.next_chunk,
                    'plan_hash':None if not self.compiled else self.compiled.plan['plan_hash'],
                    'config_hash':config_hash,
                    'compiled_config_hash':None if not self.compiled else self.compiled.config_hash,
                    'feedback':feedback, 'leased':self.lease,
                    'external_control_busy':external_busy,
                    'heartbeat':dict(self.heartbeat_stats)}

    def _require(self, data, motion=False):
        if not self.session or data.get('session') != self.session:
            raise ValueError('Unknown robot session')
        if self.cancelled.is_set() or self.state == 'fault':
            raise ValueError(self.error or 'Robot session cancelled')
        if motion:
            if not self.enabled: raise ValueError('Robot motion is disabled in worker configuration')
            if not self.lease or self.clock() >= self.deadline:
                raise ValueError('Robot control lease expired')
            if not self.compiled: raise ValueError('Robot has no validated trajectory')

    def _motion_context(self, data):
        # ROS and filesystem calls must never hold the lease/watchdog lock.
        # Callers revalidate this snapshot under the lock before changing state.
        with self.lock:
            self._require(data, motion=True)
            generation, compiled = self.generation, self.compiled
        feedback = self.backend.feedback()
        if not feedback['ready']: raise ValueError(feedback['reason'])
        if configuration(self.config_path)[2] != compiled.config_hash:
            raise ValueError('Robot calibration changed; rerun preflight')
        return generation, compiled

    def _require_context(self, data, generation, compiled):
        self._require(data, motion=True)
        if generation != self.generation or compiled is not self.compiled:
            raise ValueError('Robot session changed during motion validation')

    def prepare(self, data):
        feedback = self.backend.feedback()
        if not feedback['joints_fresh']: raise ValueError('Fresh robot joint feedback is required')
        seed = self.backend.joints()
        with self.lock:
            if self.lease or self.stopping or self.stop_unconfirmed or (self.thread and self.thread.is_alive()):
                raise ValueError('Robot worker is busy')
            session = data.get('session')
            if not isinstance(session, str) or not 16 <= len(session) <= 128:
                raise ValueError('Invalid session identifier')
            plan = data['plan']
            timing = clock_options(data.get('clock_options'))
            if self.control_lock is not None:self.control_lock.acquire()
            self.cancelled.clear()
            self.session, self.compiled, self.goal = session, None, None
            self.clock_options = timing
            self.state, self.error, self.progress, self.next_chunk = 'preflighting', '', 0., 0
            self.lease, self.deadline = True, self.clock()+self.lease_s
            self.generation += 1
            generation = self.generation

            def run():
                try:
                    def report(done, total):
                        with self.lock: self.progress = done/max(1,total)
                    compiled = self.compiler(self.config_path, plan, seed, report, self.cancelled.is_set)
                    if hasattr(self.backend, 'prepare_motions'):
                        self.backend.prepare_motions(compiled.chunks, self.cancelled.is_set)
                    with self.lock:
                        if generation != self.generation or self.cancelled.is_set(): return
                        self.compiled, self.state, self.progress = compiled, 'ready', 1.
                except Exception as error:
                    self.abort({'session':session, 'reason':str(error)}, _generation=generation)
            self.thread = threading.Thread(target=run, daemon=True)
            self.thread.start()
        return self.status()

    def acquire(self, data):
        with self.lock:
            self._require(data)
            if clock_options(data.get('clock_options')) != self.clock_options:
                raise ValueError('Clock timing options differ from robot preflight')
            if self.lease: raise ValueError('Robot lease already acquired')
            if self.stopping: raise ValueError('Robot cancellation is still in progress')
            if self.state != 'ready': raise ValueError('Robot preflight is not ready')
            if self.control_lock is not None:self.control_lock.acquire()
            self.lease, self.deadline = True, self.clock()+self.lease_s
            self.heartbeat_stats = {'count':0, 'max_lock_wait_ms':0., 'max_gap_ms':0.}
            self.last_heartbeat = None
        return self.status()

    def heartbeat(self, data):
        received = self.clock()
        with self.lock:
            self._require(data)
            if not self.lease or self.clock() >= self.deadline:
                raise ValueError('Robot heartbeat arrived after the lease expired')
            now = self.clock()
            self.deadline = now+self.lease_s
            self.heartbeat_stats['count'] += 1
            self.heartbeat_stats['max_lock_wait_ms'] = max(
                self.heartbeat_stats['max_lock_wait_ms'], (now-received)*1000)
            if self.last_heartbeat is not None:
                self.heartbeat_stats['max_gap_ms'] = max(
                    self.heartbeat_stats['max_gap_ms'], (now-self.last_heartbeat)*1000)
            self.last_heartbeat = now
            # Lease renewal must not perform disk/configuration or ROS queries.
            # Those checks still run for every motion request and execution tick.
            return {'state':self.state, 'session':self.session, 'leased':self.lease}

    def release(self, data):
        with self.lock:
            self._require(data)
            if self.state not in ('ready','complete'): raise ValueError('Robot is still busy')
            self.lease = False
            if self.control_lock is not None:self.control_lock.release()
        return self.status()

    def approach(self, data):
        generation, compiled = self._motion_context(data)
        with self.lock:
            self._require_context(data, generation, compiled)
            if self.state != 'ready' or self.next_chunk != 0:
                raise ValueError('Approach is only allowed before the first chunk')
            self.state = 'approaching'

            def run():
                try:
                    motion = compiled.approach(self.backend.joints())
                    self._execute(motion, self.clock()+.6, generation, None)
                except Exception as error:
                    self.abort({'session':data['session'], 'reason':str(error)}, _generation=generation)
            self.thread = threading.Thread(target=run, daemon=True)
            self.thread.start()
        return self.status()

    def schedule(self, data):
        generation, compiled = self._motion_context(data)
        joints = self.backend.joints()
        with self.lock:
            self._require_context(data, generation, compiled)
            if clock_options(data.get('clock_options')) != self.clock_options:
                raise ValueError('Clock timing options differ from robot preflight')
            if self.state != 'at_boundary': raise ValueError('Robot has not reached a chunk boundary')
            index, start = data.get('index'), data.get('start_monotonic')
            uncertainty = data.get('clock_uncertainty_s')
            if type(index) is not int or index != self.next_chunk or index >= len(self.compiled.chunks):
                raise ValueError('Unexpected robot chunk index')
            if (type(start) not in (float,int) or not math.isfinite(start) or
                    not .35 <= start-self.clock() <= 5.):
                raise ValueError('Robot scheduling deadline is too close or invalid')
            limit = self.clock_options['max_uncertainty_ms']
            if type(uncertainty) not in (float,int) or not 0 <= uncertainty <= limit/1000:
                raise ValueError('Clock uncertainty exceeds %g ms' % limit)
            motion = self.compiled.chunks[index]
            if np.max(abs(joints-motion.positions[0])) > math.radians(1):
                raise ValueError('Robot moved away from the preflighted chunk start')
            self.state = 'arming'
            generation = self.generation
        # Wait for controller acceptance, not for trajectory completion.
        accepted = Future()

        def run():
            try:
                self._execute(motion, start, generation, index, accepted)
            except Exception as error:
                if not accepted.done(): accepted.set_exception(error)
                self.abort({'session':data['session'], 'reason':str(error)}, _generation=generation)
        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()
        accepted.result(timeout=max(.1, start-self.clock()-.25))
        return {'accepted':True, 'index':index, 'start_monotonic':start}

    def _execute(self, motion, start, generation, chunk_index, accepted=None):
        data = {'session':self.session}
        checked_generation, compiled = self._motion_context(data)
        with self.lock:
            self._require_context(data, generation, compiled)
            if checked_generation != generation: raise RuntimeError('Motion session changed')
        goal = self.backend.send(motion, start)
        rejection = None
        with self.lock:
            if self.cancelled.is_set() or generation != self.generation:
                rejection = 'Cancelled while controller was accepting the goal'
            elif not self.lease or self.clock() >= self.deadline:
                rejection = 'Robot control lease expired while accepting the goal'
            elif self.clock() >= start-.25:
                rejection = 'Controller accepted the goal after the commit deadline'
            else:
                self.goal, self.state = goal, 'armed' if chunk_index is not None else 'approaching'
                self.validation_deadline = self.clock()+.25
                if accepted is not None: accepted.set_result(True)
        if rejection:
            self._cancel_goal(goal)
            raise RuntimeError(rejection)
        end = start+motion.times[-1]
        while not self.backend.done(goal):
            checked_generation, compiled = self._motion_context(data)
            with self.lock:
                self._require_context(data, generation, compiled)
                if checked_generation != generation: raise RuntimeError('Motion session changed')
                if self.clock() >= self.validation_deadline:
                    raise RuntimeError('Robot motion supervision stalled for 250 ms')
                self.validation_deadline = self.clock()+.25
                if self.clock() >= start and chunk_index is not None: self.state = 'running'
            if self.clock() > end+2.: raise RuntimeError('Robot missed the chunk completion deadline')
            elapsed = self.clock()-start
            if 0 <= elapsed <= motion.times[-1]:
                index = min(len(motion.times)-2, max(0, np.searchsorted(motion.times, elapsed)-1))
                dt = motion.times[index+1]-motion.times[index]
                expected = cubic_joint_state(*motion.positions[index:index+2],
                    *motion.velocities[index:index+2], dt, (elapsed-motion.times[index])/dt)[0]
                if np.max(abs(self.backend.joints()-expected)) > math.radians(2.):
                    raise RuntimeError('Robot tracking error exceeds two degrees')
            time.sleep(.02)
        self.backend.check_result(goal)
        if self.clock() < end-.02: raise RuntimeError('Robot completed earlier than its master timeline')
        if np.max(abs(self.backend.joints()-motion.positions[-1])) > math.radians(.2):
            raise RuntimeError('Robot did not reach the chunk endpoint')
        with self.lock:
            if self.cancelled.is_set() or generation != self.generation: return
            if chunk_index is not None: self.next_chunk += 1
            self.state, self.goal = 'at_boundary', None
            self.validation_deadline = None

    def _cancel_goal(self, goal):
        try: self.backend.cancel(goal)
        except Exception as error:
            with self.lock:self.stop_unconfirmed = True
            if hasattr(self.backend, 'stop_program'):
                try: self.backend.stop_program()
                except Exception as stop_error:
                    raise RuntimeError(str(error)+'; UR stop failed: '+str(stop_error)) from error
            raise

    def complete(self, data):
        with self.lock:
            self._require(data)
            if self.state != 'at_boundary' or self.next_chunk != len(self.compiled.chunks):
                raise ValueError('Robot has unfinished chunks')
            self.state, self.lease = 'complete', False
            if self.control_lock is not None:self.control_lock.release()
        return self.status()

    def abort(self, data, *, _generation=None):
        with self.lock:
            if _generation is not None and _generation != self.generation:
                return {'state':self.state, 'error':self.error}
            if self.session and data.get('session') != self.session:
                raise ValueError('Unknown robot session')
            if self.state == 'fault' and not self.lease and self.goal is None:
                return {'state':'fault', 'error':self.error}
            self.cancelled.set()
            self.generation += 1
            self.lease, self.state, self.error = False, 'fault', str(data.get('reason','Operator abort'))
            goal, self.goal = self.goal, None
            self.validation_deadline = None
            self.stopping = goal is not None
        try:
            if goal is not None:
                try:self._cancel_goal(goal)
                except Exception as error:
                    with self.lock:
                        self.error += '; cancellation failed: '+str(error)
                        self.stop_unconfirmed = True
        finally:
            with self.lock:
                self.stopping = False
                self._release_control_if_idle()
        return {'state':'fault', 'error':self.error}

    def _release_control_if_idle(self):
        # A send may still be awaiting acceptance when abort runs. Keep ownership
        # until that execution thread has cancelled any late accepted goal.
        if (self.control_lock is not None and not self.lease and not self.stopping and
                not self.stop_unconfirmed and not (self.thread and self.thread.is_alive())):
            self.control_lock.release()

    def _watchdog(self):
        while not self.stop.wait(.05):
            with self.lock:
                self._release_control_if_idle()
                expired = self.lease and self.clock() >= self.deadline
                stalled = (self.lease and self.goal is not None and self.validation_deadline is not None
                           and self.clock() >= self.validation_deadline)
                session = self.session
                generation = self.generation
            if expired: self.abort({'session':session, 'reason':'Bridge heartbeat lost'}, _generation=generation)
            elif stalled: self.abort({'session':session, 'reason':'Robot motion supervision stalled for 250 ms'}, _generation=generation)

    def close(self):
        self.stop.set()
        if self.session: self.abort({'session':self.session, 'reason':'Robot worker shutting down'})
        if self.thread and self.thread is not threading.current_thread():self.thread.join(1.)
        with self.lock:self._release_control_if_idle()


class ROSRobot:
    """ROS I/O only: stamped joint actions, joint feedback and UR interlocks."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float64
        from ur_dashboard_msgs.msg import SafetyMode
        from control_msgs.action import FollowJointTrajectory
        from std_srvs.srv import Trigger
        rclpy.init()
        self.node = Node('axisforge_sync_worker')
        if self.node.get_parameter('use_sim_time').value:
            raise ValueError('Live worker requires a system ROS clock, not use_sim_time')
        self.action = ActionClient(self.node, FollowJointTrajectory,
                                  '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.stop_client = self.node.create_client(Trigger,'/dashboard_client/stop')
        self.active_clock_offset = None
        self.prepared_motions = {}
        self.lock = threading.RLock()
        self.values, self.joint_time = None, 0.
        self.signals = {}

        def joints(msg):
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if abs(self.node.get_clock().now().nanoseconds/1e9-stamp) > .25: return
            if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name): return
            mapping = dict(zip(msg.name,msg.position))
            if not all(n in mapping and math.isfinite(mapping[n]) for n in JOINT_NAMES): return
            with self.lock:
                self.values = np.array([mapping[n] for n in JOINT_NAMES])
                self.joint_time = time.monotonic()

        def signal(name, value):
            with self.lock: self.signals[name] = (value,time.monotonic())

        self.node.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data)
        self.node.create_subscription(Float64,'/speed_scaling_state_broadcaster/speed_scaling',
                                      lambda m:signal('speed',m.data),qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Bool,'/io_and_status_controller/robot_program_running',
                                      lambda m:signal('program',m.data),latched)
        self.node.create_subscription(SafetyMode,'/io_and_status_controller/safety_mode',
                                      lambda m:signal('safety',m.mode),latched)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def joints(self):
        with self.lock:
            if self.values is None: raise ValueError('No robot joint state')
            return self.values.copy()

    def feedback(self):
        with self.lock:
            now = time.monotonic()
            fresh = self.values is not None and now-self.joint_time < .25
            signals = dict(self.signals)
        # UR SpeedScalingStateBroadcaster publishes percent (RTDE factor * 100),
        # not the 0..1 factor consumed by the trajectory controller. Convert the
        # declared topic units explicitly: a received 1.0 means 1%, never 100%.
        speed_percent = signals.get('speed',(None,0))[0]
        if not isinstance(speed_percent, (int,float)) or not math.isfinite(speed_percent):
            speed_percent = None
        speed_scaling = None if speed_percent is None else speed_percent/100.
        reason = ''
        if not fresh: reason = 'Robot joint feedback is missing or older than 250 ms'
        elif not self.action.server_is_ready(): reason = 'UR trajectory controller is unavailable'
        elif 'speed' not in signals or now-signals['speed'][1] > .25:
            reason = 'UR speed-scaling feedback is stale'
        elif not signals.get('program',(False,0))[0]: reason = 'Start the UR External Control program'
        elif signals.get('safety',(None,0))[0] != 1: reason = 'UR safety mode must be NORMAL'
        elif speed_scaling is None:
            reason = 'UR speed-scaling feedback is invalid'
        elif not .999 <= speed_scaling <= 1.001:
            reason = ('UR speed scaling is %g%%; set it to 100%%. '
                      'Synchronized motion cannot be independently slowed' % speed_percent)
        elif (self.active_clock_offset is not None and
              abs(self.node.get_clock().now().nanoseconds/1e9-now-self.active_clock_offset)>.005):
            reason = 'ROS clock stepped relative to monotonic time'
        return {'ready':not reason, 'reason':reason, 'joints_fresh':fresh,
                'joint_age_s':round(now-self.joint_time,3) if self.values is not None else None,
                'robot_program_running':signals.get('program',(False,0))[0],
                'safety_mode':signals.get('safety',(None,0))[0],
                'speed_scaling':speed_scaling, 'speed_scaling_percent':speed_percent}

    @staticmethod
    def _wait(future, timeout):
        event = threading.Event()
        future.add_done_callback(lambda _:event.set())
        if not event.wait(timeout): raise RuntimeError('ROS controller response timed out')
        return future.result()

    @staticmethod
    def _build_goal(motion, cancelled=lambda:False):
        from control_msgs.action import FollowJointTrajectory
        from control_msgs.msg import JointTolerance
        from trajectory_msgs.msg import JointTrajectoryPoint
        from print_control.motion_planner import duration_parts
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        for index,(q,v,t) in enumerate(zip(motion.positions,motion.velocities,motion.times)):
            if index % 32 == 0 and cancelled(): raise RuntimeError('Planning cancelled')
            point = JointTrajectoryPoint(positions=q.tolist(),velocities=v.tolist())
            point.time_from_start.sec, point.time_from_start.nanosec = duration_parts(float(t))
            goal.trajectory.points.append(point)
        goal.path_tolerance = [JointTolerance(name=n,position=math.radians(2)) for n in JOINT_NAMES]
        goal.goal_tolerance = [JointTolerance(name=n,position=math.radians(.2)) for n in JOINT_NAMES]
        goal.goal_time_tolerance.sec = 1
        return goal

    def prepare_motions(self, motions, cancelled=lambda:False):
        # Build large messages during cold preflight. Stamps are assigned only
        # when scheduled; positions, velocities and times remain unchanged.
        prepared = {}
        for motion in motions:
            prepared[id(motion)] = (motion, self._build_goal(motion, cancelled))
        self.prepared_motions = prepared

    def send(self, motion, start):
        from builtin_interfaces.msg import Time
        prepared = getattr(self, 'prepared_motions', {}).get(id(motion))
        goal = prepared[1] if prepared is not None and prepared[0] is motion else self._build_goal(motion)
        self.active_clock_offset = self.node.get_clock().now().nanoseconds/1e9-time.monotonic()
        stamp = self.node.get_clock().now().nanoseconds + round((start-time.monotonic())*1e9)
        goal.trajectory.header.stamp = Time(sec=stamp//1000000000, nanosec=stamp%1000000000)
        future = self.action.send_goal_async(goal)
        try:
            handle = self._wait(future,max(.05,start-time.monotonic()-.25))
        except Exception:
            # A timed-out goal may still execute. Issue an independent stop now;
            # cancelling a late response alone is not a sufficient guarantee.
            from std_srvs.srv import Trigger
            self.stop_client.call_async(Trigger.Request())
            # A late acceptance must still be cancelled after the caller has timed out.
            def cancel_late(done):
                try:
                    handle = done.result()
                    if handle.accepted: handle.cancel_goal_async()
                except Exception: pass
            future.add_done_callback(cancel_late)
            raise
        if not handle.accepted: raise RuntimeError('UR controller rejected the trajectory')
        return (handle,handle.get_result_async())

    def done(self, goal): return goal[1].done()

    def check_result(self, goal):
        result = goal[1].result()
        self.active_clock_offset = None
        if result.status != 4 or result.result.error_code != 0:
            raise RuntimeError('UR trajectory failed: '+result.result.error_string)

    def cancel(self, goal):
        result = self._wait(goal[0].cancel_goal_async(),2.)
        self.active_clock_offset = None
        if not result.goals_canceling and not goal[1].done():
            raise RuntimeError('UR controller did not acknowledge cancellation')
        # A cancellation acknowledgement is not a stopped robot. Do not release
        # ownership to the probe until the action reports a terminal state.
        terminal = self._wait(goal[1],2.)
        if terminal.status not in (4,5,6):
            raise RuntimeError('UR trajectory termination is unconfirmed')

    def stop_program(self):
        from std_srvs.srv import Trigger
        if not self.stop_client.service_is_ready(): raise RuntimeError('UR stop service is unavailable')
        result = self._wait(self.stop_client.call_async(Trigger.Request()),2.)
        if not result.success: raise RuntimeError(result.message)

    def close(self):
        import rclpy
        self.executor.shutdown(timeout_sec=2)
        self.node.destroy_node()
        rclpy.shutdown()


def serve(worker, port):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def setup(self):
            super().setup()
            self.connection.settimeout(10.)

        def log_message(self, *args): pass

        def respond(self, code, value):
            raw = json.dumps(value, allow_nan=False).encode()
            try:
                self.send_response(code)
                self.send_header('Content-Type','application/json')
                self.send_header('Content-Length',str(len(raw)))
                if code >= 400:
                    self.send_header('Connection','close')
                    self.close_connection = True
                self.end_headers()
                self.wfile.write(raw)
            except (BrokenPipeError, ConnectionResetError): self.close_connection = True

        def do_GET(self):
            if self.path not in ('/status','/capabilities'):
                return self.respond(404,{'error':'Unknown endpoint'})
            self.respond(200,worker.status())

        def do_POST(self):
            actions = ('prepare','acquire','heartbeat','release','approach','schedule','complete','abort')
            if self.path[1:] not in actions: return self.respond(404,{'error':'Unknown endpoint'})
            if self.headers.get('Content-Type','').split(';')[0] != 'application/json':
                return self.respond(415,{'error':'application/json required'})
            if self.headers.get('Origin'):
                return self.respond(403,{'error':'Browser requests must use the dashboard'})
            try:
                length = int(self.headers.get('Content-Length',0))
                if not 0 < length <= 32*1024*1024: raise ValueError('Request body exceeds limit')
                data = json.loads(self.rfile.read(length))
                if not isinstance(data,dict): raise ValueError('Expected an object')
                result = getattr(worker,self.path[1:])(data)
                self.respond(200,result)
            except Exception as error:
                self.respond(409,{'error':str(error)})
    class Server(ThreadingHTTPServer):
        request_queue_size = 64
        daemon_threads = True
    return Server(('127.0.0.1',port),Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True)
    parser.add_argument('--port',type=int,default=8081)
    parser.add_argument('--enable-motion',action='store_true')
    args = parser.parse_args()
    backend = ROSRobot()
    worker = RobotWorker(backend,args.config,args.enable_motion,control_lock=RobotControlLock())
    server = serve(worker,args.port)
    import signal
    signal.signal(signal.SIGTERM,lambda *_:threading.Thread(target=server.shutdown,daemon=True).start())
    try:
        server.serve_forever(poll_interval=.1)
    finally:
        worker.close()
        backend.close()
        server.server_close()


if __name__ == '__main__': main()
