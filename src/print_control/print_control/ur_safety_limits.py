"""Non-moving UR controller checks before sending setup trajectories.

URScript is_within_safety_limits uses the active installation's safety planes,
tool orientation and joint limits. Small secondary programs return results via
Primary Interface text messages; no main program, motion, socket command, I/O,
TCP/payload change or safety-configuration change is uploaded. This sampled
planning check supplements, and cannot replace, the controller's safety system.
"""
import errno
import ipaddress
import math
import os
import re
import secrets
import select
import socket
import struct
import time

import numpy as np

from print_control.motion_planner import cubic_joint_state


class RobotSafetyError(RuntimeError):
    """No motion may be sent when controller validation fails or is unavailable."""


def trajectory_samples(motion, *, pump=lambda: None):
    """Include every waypoint, interval midpoints and joint-position extrema."""
    q, v, t = (np.asarray(x, dtype=float) for x in
               (motion.positions, motion.velocities, motion.times))
    if (len(t) < 2 or q.shape != (len(t), 6) or v.shape != q.shape or
            not all(np.all(np.isfinite(x)) for x in (q, v, t)) or np.any(np.diff(t) <= 0)):
        raise RobotSafetyError('Invalid trajectory for robot safety-limit checking')
    result = [q[0].copy()]
    for index, dt in enumerate(np.diff(t)):
        pump()
        q0, q1 = q[index:index+2]
        v0, v1 = v[index:index+2]
        # Bound subdivision using the cubic Bezier control-polygon variation;
        # equal endpoints must not conceal an interpolation excursion.
        control = np.array([q0, q0+dt*v0/3., q1-dt*v1/3., q1])
        variation = float(np.max(np.sum(np.abs(np.diff(control, axis=0)), axis=0)))
        count = max(2, math.ceil(variation/.002))
        if count > 30000 or len(result)+count > 30000:
            raise RobotSafetyError('Setup trajectory exceeds the safety-query sample budget; reduce its travel')
        fractions = set(np.linspace(0., 1., count+1)[1:])
        a = 2*q0-2*q1+dt*(v0+v1)
        b = -3*q0+3*q1-dt*(2*v0+v1)
        c = dt*v0
        for aj, bj, cj in zip(a, b, c):
            roots = np.roots([3*aj, 2*bj, cj]) if abs(aj) > 1e-14 else (
                [-cj/(2*bj)] if abs(bj) > 1e-14 else [])
            for root in roots:
                if np.isreal(root) and 0 < float(np.real(root)) < 1:
                    fractions.add(float(np.real(root)))
        if len(result)+len(fractions) > 30000:
            raise RobotSafetyError('Setup trajectory exceeds the safety-query sample budget; reduce its travel')
        result.extend(cubic_joint_state(q0, q1, v0, v1, dt, fraction)[0]
                      for fraction in sorted(fractions))
    return result


def query_script(token, joints, *, self_test=False):
    if not re.fullmatch('[a-f0-9]{32}', token):
        raise ValueError('Invalid safety-query token')
    if not 1 <= len(joints) <= (7 if self_test else 8):
        raise ValueError('Safety queries require a small bounded batch')
    expressions = ['is_within_safety_limits(p[100,100,100,0,0,0])'] if self_test else []
    for values in joints:
        q = np.asarray(values, dtype=float)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError('Six finite joints are required for safety checking')
        expressions.append('is_within_safety_limits(['+','.join(format(x, '.17g') for x in q)+'])')
    return ('sec axisforge_safety_query():\n  textmsg("AXISFORGE_SAFETY_'+token+'=", ['+
            ','.join(expressions)+'])\nend\n').encode('ascii')


class PrimaryReplies:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        messages = []
        while len(self.buffer) >= 5:
            size = struct.unpack('!I', self.buffer[:4])[0]
            if not 5 <= size <= 1048576:
                raise RobotSafetyError('Invalid packet from robot safety-query interface')
            if len(self.buffer) < size:
                break
            packet = bytes(self.buffer[:size]); del self.buffer[:size]
            # Primary RobotMessage: type, timestamp, source, subtype, text.
            if len(packet) >= 15 and packet[4] == 20 and packet[14] == 0:
                messages.append(packet[15:].decode('ascii', errors='replace').rstrip('\0'))
        return messages


class RobotSafetyChecker:
    def __init__(self, robot_ip=None, *, timeout_s=2., clock=time.monotonic):
        self.robot_ip = str(ipaddress.IPv4Address(robot_ip or os.environ.get('ROBOT_IP', '192.168.0.100')))
        self.timeout_s, self.clock = timeout_s, clock

    def _wait(self, sock, *, write, deadline, pump):
        while self.clock() < deadline:
            pump()  # ROS, ALPS, operator abort and heartbeat guards stay active.
            readable, writable, errors = select.select(
                [] if write else [sock], [sock] if write else [], [sock], .005)
            if errors:
                raise RobotSafetyError('Robot safety-query connection failed')
            if readable or writable:
                return
        raise RobotSafetyError('Robot safety-limit query timed out; no motion sent. Check Remote Control and Ethernet')

    def _connect(self, sock, pump):
        sock.setblocking(False)
        status = sock.connect_ex((self.robot_ip, 30001))
        if status not in (0, errno.EINPROGRESS, errno.EWOULDBLOCK):
            raise RobotSafetyError('Robot safety-query interface unavailable; no motion sent. Check Remote Control and Ethernet')
        self._wait(sock, write=True, deadline=self.clock()+self.timeout_s, pump=pump)
        if sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR):
            raise RobotSafetyError('Cannot connect to the robot safety-query interface; no motion sent')

    def _query(self, sock, replies, joints, pump, *, self_test=False):
        token = secrets.token_hex(16)
        prefix = 'AXISFORGE_SAFETY_'+token+'='
        pending = query_script(token, joints, self_test=self_test)
        deadline = self.clock()+self.timeout_s
        while pending:
            self._wait(sock, write=True, deadline=deadline, pump=pump)
            try: written = sock.send(pending)
            except BlockingIOError: continue
            if not written: raise RobotSafetyError('Robot closed the safety-query connection')
            pending = pending[written:]
        while True:
            self._wait(sock, write=False, deadline=deadline, pump=pump)
            try: data = sock.recv(65536)
            except BlockingIOError: continue
            if not data: raise RobotSafetyError('Robot closed the safety-query connection; no motion sent')
            for message in replies.feed(data):
                if not message.startswith(prefix):
                    continue
                payload = message[len(prefix):]
                if not re.fullmatch(r'\[(?:True|False)(?:,(?:True|False))*\]', payload):
                    raise RobotSafetyError('Invalid robot safety-limit response; no motion sent')
                result = [value == 'True' for value in payload[1:-1].split(',')]
                if len(result) != len(joints)+int(self_test):
                    raise RobotSafetyError('Incomplete robot safety-limit response; no motion sent')
                if self_test:
                    if result.pop(0):
                        raise RobotSafetyError('Robot safety checker failed its unreachable-pose self-test; no motion sent')
                return result

    def validate(self, motion, *, pump, nozzle_fk, progress=lambda checked,total,elapsed: None):
        samples = trajectory_samples(motion, pump=pump)
        began = self.clock()
        reported = began
        progress(0,len(samples),0.)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                self._connect(sock, pump)
                replies, index = PrimaryReplies(), 0
                while index < len(samples):
                    if self.clock()-began > 420.:
                        raise RobotSafetyError('Robot safety path check exceeded seven minutes; reduce the search distance')
                    batch = samples[index:index+(7 if index == 0 else 8)]
                    result = self._query(sock, replies, batch, pump, self_test=index == 0)
                    for offset, accepted in enumerate(result):
                        if not accepted:
                            xyz = nozzle_fk(batch[offset])
                            raise RobotSafetyError(
                                '%s: UR controller rejected path sample %d/%d at planned nozzle '
                                'Base XYZ [%.3f, %.3f, %.3f] mm. Active safety planes, joint limits '
                                'or tool-orientation limits would be exceeded. No motion sent; '
                                'reduce Maximum Z search or reposition within the robot limits.' %
                                (motion.name, index+offset+1, len(samples), *xyz))
                    index += len(batch)
                    if self.clock()-reported >= 1. or index==len(samples):
                        reported=self.clock()
                        progress(index,len(samples),reported-began)
                pump()
        except OSError as error:
            raise RobotSafetyError('Robot safety-limit connection failed; no motion sent: '+str(error)) from error
        return len(samples)
