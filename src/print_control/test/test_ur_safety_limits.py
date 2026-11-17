"""Controller limit checks with a fake Primary Interface; no robot access."""
import json
import re
import struct
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from print_control.bed_probe import BedProbe
from print_control.bridge_bed_probe import BridgeBedProbe
from print_control.motion_planner import PlannedMotion
from print_control.ur_safety_limits import (
    PrimaryReplies, RobotSafetyChecker, RobotSafetyError, query_script, trajectory_samples,
)


def packet(text, kind=20, subtype=0):
    body = bytes([kind])+struct.pack('!QbB', 123, 1, subtype)+text.encode()
    return struct.pack('!I', len(body)+4)+body


def motion():
    return PlannedMotion('Z search', np.array([[0.]*6, [.01]+[0.]*5]),
                         np.zeros((2, 6)), np.array([0., 1.]))


class FakePrimary:
    def __init__(self, *, reject=False, broken_self_test=False, stale=False):
        self.sent, self.response = b'', b''
        self.scripts = []
        self.reject, self.broken_self_test, self.stale = reject, broken_self_test, stale
    def __enter__(self): return self
    def __exit__(self, *args): pass
    def setblocking(self, flag): pass
    def connect_ex(self, address): return 0
    def getsockopt(self, *args): return 0
    def send(self, data):
        size = min(23, len(data))  # Force partial sends.
        self.sent += data[:size]
        if self.sent.endswith(b'end\n'):
            script = self.sent.decode(); self.scripts.append(script); self.sent = b''
            prefix = re.search('AXISFORGE_SAFETY_[a-f0-9]{32}=', script)[0]
            if self.stale: prefix = 'AXISFORGE_SAFETY_'+('f'*32)+'='
            result = [self.broken_self_test] if 'p[100,100,100,0,0,0]' in script else []
            joints = [json.loads(q) for q in re.findall(r'is_within_safety_limits\((\[[^]]*\])\)', script)]
            result += [not self.reject or q[0] < .005 for q in joints]
            self.response += packet(prefix+'['+','.join(str(x) for x in result)+']')
        return size
    def recv(self, size):
        if not self.response: raise BlockingIOError()
        result, self.response = self.response[:11], self.response[11:]
        return result


class SafetyTests(unittest.TestCase):
    def validate(self, peer):
        now = [100.]
        def pump(): now[0] += .001
        checker = RobotSafetyChecker('127.0.0.1', timeout_s=.5, clock=lambda: now[0])
        with patch('print_control.ur_safety_limits.socket.socket', return_value=peer), \
             patch('print_control.ur_safety_limits.select.select', side_effect=lambda r,w,e,t:(r,w,[])):
            return checker.validate(motion(), pump=pump, nozzle_fk=lambda q: 1000*q[:3])

    def test_fragmented_response_and_small_batches_accept_valid_full_path(self):
        peer = FakePrimary()
        count = self.validate(peer)
        self.assertGreaterEqual(count, 6)
        self.assertTrue(all(s.startswith('sec axisforge_safety_query():\n') for s in peer.scripts))
        self.assertTrue(all(s.count('is_within_safety_limits(') <= 8 for s in peer.scripts))
        for command in ('movej(', 'movel(', 'servoj(', 'set_tcp(', 'socket_open(', 'sleep(', 'def '):
            self.assertNotIn(command, ''.join(peer.scripts))

    def test_rejected_path_identifies_point_and_never_returns_success(self):
        with self.assertRaisesRegex(RobotSafetyError, 'UR controller rejected path sample.*Base XYZ'):
            self.validate(FakePrimary(reject=True))

    def test_unreachable_self_test_and_stale_replies_fail_closed(self):
        with self.assertRaisesRegex(RobotSafetyError, 'self-test'):
            self.validate(FakePrimary(broken_self_test=True))
        with self.assertRaisesRegex(RobotSafetyError, 'timed out'):
            self.validate(FakePrimary(stale=True))

    def test_parser_handles_coalesced_frames_and_ignores_unrelated_types(self):
        parser = PrimaryReplies()
        data = packet('ignored', subtype=3)+packet('first')+packet('second')
        self.assertEqual(parser.feed(data[:3]), [])
        self.assertEqual(parser.feed(data[3:]), ['first','second'])
        with self.assertRaises(RobotSafetyError): parser.feed(struct.pack('!IB', 4, 20))

    def test_no_invalid_values_or_injected_script_can_be_sent(self):
        for q in ([float('nan')]*6, [0.]*5, ['movej([0])']*6):
            with self.assertRaises(ValueError): query_script('a'*32,[q])
        with self.assertRaises(ValueError): query_script('";movej()', [[0.]*6])

    def test_equal_endpoints_do_not_hide_cubic_excursion(self):
        m = PlannedMotion('loop', np.zeros((2,6)),
                          np.array([[.02]+[0.]*5,[-.02]+[0.]*5]), np.array([0.,1.]))
        samples = trajectory_samples(m)
        self.assertGreaterEqual(max(q[0] for q in samples), .005-1e-10)
        np.testing.assert_array_equal(samples[0],m.positions[0])
        np.testing.assert_array_equal(samples[-1],m.positions[-1])

    def test_bad_trajectory_and_large_query_budget_are_rejected(self):
        for times in ([0.,0.], [0.,float('nan')]):
            m=motion(); m.times=np.array(times)
            with self.assertRaises(RobotSafetyError):trajectory_samples(m)
        m=motion();m.velocities[:]=1e6
        with self.assertRaisesRegex(RobotSafetyError,'sample budget'):trajectory_samples(m)

    def test_abort_is_pumped_during_query_and_prevents_success(self):
        with patch('print_control.ur_safety_limits.socket.socket', return_value=FakePrimary()):
            with self.assertRaisesRegex(RuntimeError,'Operator stopped'):
                RobotSafetyChecker('127.0.0.1').validate(motion(),
                    pump=Mock(side_effect=RuntimeError('Operator stopped')), nozzle_fk=lambda q:q[:3])

    def test_setup_action_is_never_sent_before_controller_approval(self):
        node=object.__new__(BridgeBedProbe)
        node.config=SimpleNamespace(dry_run=False)
        node.control=SimpleNamespace(state='homing_fast',emit=Mock())
        node.planner=SimpleNamespace(nozzle_position=lambda q:q[:3])
        node.pump=Mock()
        node.robot_safety_checker=Mock()
        with patch.object(BedProbe,'run_motion') as send:
            node.robot_safety_checker.validate.side_effect=RobotSafetyError('unsafe plane')
            with self.assertRaises(RobotSafetyError):node.run_motion(motion())
            send.assert_not_called()
            node.robot_safety_checker.validate.side_effect=None
            node.run_motion(motion())
            send.assert_called_once()

    def test_dry_run_never_queries_robot(self):
        node=object.__new__(BridgeBedProbe)
        node.config=SimpleNamespace(dry_run=True)
        node.robot_safety_checker=Mock()
        with patch.object(BedProbe,'run_motion') as send:
            node.run_motion(motion())
            send.assert_called_once()
        node.robot_safety_checker.validate.assert_not_called()


if __name__ == '__main__': unittest.main()
