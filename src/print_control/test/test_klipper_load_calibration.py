"""Stock native calibration and SAVE_CONFIG without hardware or robot imports."""
import copy
import json
import threading
import time
import unittest
from urllib.parse import urlsplit

from print_control.klipper_load_calibration import (
    KlipperLoadCalibration, KlipperLoadCalibrationError,
)


class Response:
    def __init__(self, result):
        self.data = json.dumps({'result': result}).encode()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def read(self, size):
        return self.data[:size]


class NativeHost:
    DESCRIPTIONS = {'TARE': 'Tare the load cell',
                    'CALIBRATE': 'Enter the load cell value in grams',
                    'ACCEPT': 'Accept calibration results and apply to load cell',
                    'ABORT': 'Abort load cell calibration tool'}

    def __init__(self):
        self.commands = {name: {} for name in ('ECHO', 'LOAD_CELL_CALIBRATE', 'SAVE_CONFIG', 'TURN_OFF_HEATERS')}
        self.config = {'load_cell_probe': {'sensor_type': 'ads131m02', 'counts_per_gram': '138'},
                       'extruder': {'max_temp': '300'}}
        self.pending = {}
        self.sensor = {'is_calibrated': False, 'counts_per_gram': 138.,
                       'reference_tare_counts': None, 'errors': 0, 'overflows': 0}
        self.raw_tare = -217754
        self.native_cpg = 246.912345
        self.target = 210.
        self.rows = []
        self.scripts = []
        self.exec_commands = []
        self.state = 'ready'
        self.print_state = 'standby'
        self.restart_delay = .15
        self.persist = True
        self.disconnect_save = False
        self.failure_command = None
        self.failure_after_execution = False
        self.command_delay = {}
        self.on_accept = None
        self.on_save = None
        self.omit_marker = False
        self.observations = []

    def helper(self):
        self.commands.update({name: {'help': help_text} for name, help_text in self.DESCRIPTIONS.items()})

    def remove_helper(self):
        for name in self.DESCRIPTIONS:
            self.commands.pop(name, None)

    def status(self):
        return {'webhooks': {'state': self.state}, 'gcode': {'commands': copy.deepcopy(self.commands)},
                'configfile': {'config': copy.deepcopy(self.config), 'save_config_pending': bool(self.pending),
                               'save_config_pending_items': copy.deepcopy(self.pending)},
                'load_cell_probe': copy.deepcopy(self.sensor), 'virtual_sdcard': {'is_active': False},
                'print_stats': {'state': self.print_state}, 'axisforge_sync': {},
                'extruder': {'temperature': self.target, 'target': self.target},
                'heater_bed': {'temperature': 25., 'target': 0.}}

    def respond(self, message):
        self.rows.append({'type': 'response', 'message': '// ' + message, 'time': time.time()})

    def execute(self, command):
        self.exec_commands.append(command)
        time.sleep(self.command_delay.get(command.split()[0], 0))
        if self.failure_command and command.startswith(self.failure_command) and not self.failure_after_execution:
            raise RuntimeError('Native command rejected')
        if command.startswith('ECHO '):
            if not command.startswith('ECHO MSG=') or not command[len('ECHO MSG='):]:
                raise RuntimeError("Malformed command '%s'" % command)
            if not (self.omit_marker and command.endswith('_END')):
                self.respond(command)
        elif command.startswith('LOAD_CELL_CALIBRATE'):
            if 'TARE' in self.commands:
                raise RuntimeError('Already calibrating a load cell')
            self.helper()
            self.respond('Starting load cell calibration.\n1.) Remove all load and run TARE.')
        elif command == 'TARE':
            self.respond('Load cell tare value: -2.60%% (%i)' % self.raw_tare)
        elif command.startswith('CALIBRATE GRAMS='):
            self.respond('Calibration value: -1.00%% (100000), Counts/gram: %.5f, Total capacity: +/- 10.000Kg' % self.native_cpg)
        elif command == 'ACCEPT':
            self.remove_helper()
            self.sensor.update(is_calibrated=True, counts_per_gram=self.native_cpg,
                               reference_tare_counts=self.raw_tare)
            self.pending = {'load_cell_probe': {'counts_per_gram': '%.5f' % self.native_cpg,
                                                'reference_tare_counts': str(self.raw_tare)}}
            self.respond('Load cell calibration settings:\ncounts_per_gram: %.6f' % self.native_cpg)
            if self.on_accept:
                self.on_accept()
        elif command == 'ABORT':
            self.remove_helper()
            self.respond('Load cell calibration aborted')
        elif command == 'TURN_OFF_HEATERS':
            self.target = 0.
        elif command == 'SAVE_CONFIG':
            if self.restart_delay:
                self.state = 'startup'
                time.sleep(self.restart_delay)
            if self.persist:
                self.config['load_cell_probe'].update(self.pending['load_cell_probe'])
                self.pending = {}
            if self.on_save:
                self.on_save()
            self.state = 'ready'
            if self.disconnect_save:
                raise OSError('Connection closed during restart')
        else:
            raise AssertionError('Unexpected command: ' + command)
        if self.failure_command and command.startswith(self.failure_command) and self.failure_after_execution:
            raise OSError('HTTP response lost after execution')

    def __call__(self, request, timeout):
        path = urlsplit(request.full_url).path
        if path == '/printer/objects/query':
            return Response({'status': self.status()})
        if path == '/server/gcode_store':
            return Response({'gcode_store': copy.deepcopy(self.rows[-1000:])})
        if path == '/server/info':
            self.observations.append(self.state)
            return Response({'klippy_connected': True, 'klippy_state': self.state})
        if path == '/printer/gcode/script':
            script = json.loads(request.data)['script']
            self.scripts.append(script)
            self.rows.append({'type': 'command', 'message': script, 'time': time.time()})
            for command in script.splitlines():
                self.execute(command)
            return Response('ok')
        raise AssertionError(path)


class NativeCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.host = NativeHost()
        self.client = self.make_client()

    def make_client(self, **kwargs):
        return KlipperLoadCalibration('http://printer:7125', opener=self.host,
                                      request_timeout_s=.2, restart_timeout_s=1., **kwargs)

    def record(self):
        self.client.begin_unloaded()
        return self.client.record_load(500.)

    def accept(self):
        self.record()
        return self.client.accept()

    def test_stock_tare_record_accept_uses_native_values_without_saving(self):
        self.assertEqual(self.client.prepare(), {'prepared': True})
        result = self.client.begin_unloaded()
        self.assertEqual(result, {'reference_tare_counts': -217754})
        result = self.client.record_load(500.)
        self.assertEqual(result['counts_per_gram'], float('%.5f' % self.host.native_cpg))
        receipt = self.client.accept()
        self.assertTrue(receipt['accepted'])
        self.assertFalse(receipt['saved'])
        self.assertFalse(self.client.owned)
        self.assertNotIn('SAVE_CONFIG', self.host.exec_commands)
        self.assertNotIn('TURN_OFF_HEATERS', self.host.exec_commands)
        self.assertTrue(all(not command.startswith(('G0', 'G1', 'G28', 'PROBE'))
                            for command in self.host.exec_commands))

    def test_echo_markers_use_valid_extended_gcode_parameter_grammar(self):
        with self.assertRaisesRegex(RuntimeError, 'Malformed command'):
            self.host.execute('ECHO AXISFORGE_BARE_ARGUMENT')
        response = self.client._command('ECHO MSG=AXISFORGE_CALIBRATION_TRANSPORT_CHECK', lambda: None)
        self.assertEqual(response, 'ECHO MSG=AXISFORGE_CALIBRATION_TRANSPORT_CHECK')
        for command in self.host.exec_commands[1:]:
            self.assertTrue(command.startswith('ECHO MSG='))

    def test_external_helper_is_never_adopted_or_automatically_aborted(self):
        self.host.helper()
        with self.assertRaisesRegex(KlipperLoadCalibrationError, 'Another native'):
            self.client.prepare()
        self.client.abort()
        self.assertIn('TARE', self.host.commands)
        self.assertEqual(self.host.scripts, [])

    def test_preexisting_pending_changes_and_paused_print_refuse_prepare(self):
        self.host.pending = {'extruder': {'pid_kp': '1'}}
        with self.assertRaisesRegex(KlipperLoadCalibrationError, 'pending'):
            self.client.prepare()
        self.host.pending = {}
        self.host.print_state = 'paused'
        with self.assertRaisesRegex(KlipperLoadCalibrationError, 'paused'):
            self.client.prepare()
        self.assertEqual(self.host.scripts, [])

    def test_mass_validation_prevents_any_record_mutation(self):
        self.client.begin_unloaded()
        before = len(self.host.scripts)
        for value in (True, None, float('nan'), float('inf'), 49., 25001.):
            with self.assertRaises(ValueError):
                self.client.record_load(value)
        self.assertEqual(len(self.host.scripts), before)

    def test_record_is_single_use_and_requires_native_tare(self):
        with self.assertRaises(KlipperLoadCalibrationError):
            self.client.record_load(500.)
        self.record()
        with self.assertRaises(KlipperLoadCalibrationError):
            self.client.record_load(500.)

    def test_missing_response_markers_do_not_claim_tare_success(self):
        self.host.omit_marker = True
        with self.assertRaisesRegex(KlipperLoadCalibrationError, 'markers'):
            self.client.begin_unloaded()
        self.assertIsNone(self.client.tare)
        with self.assertRaises(KlipperLoadCalibrationError):
            self.client.abort()
        self.assertNotIn('ABORT', self.host.exec_commands)

    def test_native_record_rejection_does_not_retry_or_accept(self):
        self.client.begin_unloaded()
        self.host.failure_command = 'CALIBRATE'
        with self.assertRaises(KlipperLoadCalibrationError):
            self.client.record_load(500.)
        self.assertEqual(sum(c.startswith('CALIBRATE') for c in self.host.exec_commands), 1)
        with self.assertRaises(KlipperLoadCalibrationError):
            self.client.accept()
        self.host.failure_command = None
        self.assertTrue(self.client.abort()['aborted'])

    def test_foreign_command_invalidates_ownership_and_prevents_abort(self):
        self.client.begin_unloaded()
        self.host.rows.append({'type': 'command', 'message': 'CALIBRATE GRAMS=123', 'time': time.time()})
        with self.assertRaisesRegex(KlipperLoadCalibrationError, 'Another console'):
            self.client.record_load(500.)
        with self.assertRaises(KlipperLoadCalibrationError):
            self.client.abort()
        self.assertNotIn('ABORT', self.host.exec_commands)

    def test_truncated_history_prevents_owned_mutations(self):
        self.client.begin_unloaded()
        self.host.rows = self.host.rows[1:]
        with self.assertRaisesRegex(KlipperLoadCalibrationError, 'history'):
            self.client.record_load(500.)

    def test_object_or_pending_mismatch_after_accept_never_saves(self):
        self.record()
        self.host.on_accept = lambda: self.host.pending.update(extruder={'pid_kp': '99'})
        with self.assertRaisesRegex(KlipperLoadCalibrationError, 'pending'):
            self.client.accept()
        self.assertFalse(self.client.owned)
        result = self.client.abort()
        self.assertTrue(result['accepted_unsaved'])
        self.assertNotIn('SAVE_CONFIG', self.host.exec_commands)

    def test_accept_response_loss_reports_possible_applied_unsaved_state(self):
        self.record()
        self.host.failure_command = 'ACCEPT'
        self.host.failure_after_execution = True
        with self.assertRaisesRegex(KlipperLoadCalibrationError, 'may have applied.*SAVE_CONFIG was not sent'):
            self.client.accept()
        cleanup = self.client.abort()
        self.assertTrue(cleanup['accepted_unsaved'])
        self.assertNotIn('ABORT', self.host.exec_commands)
        self.assertNotIn('SAVE_CONFIG', self.host.exec_commands)

    def test_loaded_config_change_during_session_refuses_accept(self):
        self.record()
        self.host.config['extruder']['max_temp'] = '290'
        with self.assertRaisesRegex(KlipperLoadCalibrationError, 'configuration changed'):
            self.client.accept()
        self.assertNotIn('ACCEPT', self.host.exec_commands)

    def test_owned_abort_removes_only_helper_and_does_not_save(self):
        self.client.begin_unloaded()
        result = self.client.abort()
        self.assertTrue(result['cleanup_confirmed'])
        self.assertTrue(result['aborted'])
        self.assertNotIn('TARE', self.host.commands)
        self.assertFalse(self.host.pending)

    def test_pump_runs_during_native_collection_and_cancellation_waits_for_cleanup(self):
        self.client.begin_unloaded()
        self.host.command_delay['CALIBRATE'] = .04
        pumps = []
        self.client.record_load(500., lambda: pumps.append(time.monotonic()))
        self.assertGreater(len(pumps), 3)
        self.client.abort()

    def test_pump_failure_during_mutation_is_not_retried_and_owned_abort_waits(self):
        self.client.begin_unloaded()
        self.host.command_delay['CALIBRATE'] = .04
        def pump():
            if any(c.startswith('CALIBRATE GRAMS=') for c in self.host.exec_commands):
                raise RuntimeError('Robot guard failed')
        with self.assertRaisesRegex(RuntimeError, 'Robot guard'):
            self.client.record_load(500., pump)
        self.host.command_delay = {}
        self.assertTrue(self.client.abort()['aborted'])
        self.assertEqual(sum(c.startswith('CALIBRATE') for c in self.host.exec_commands), 1)

    def test_fresh_backend_saves_once_after_heat_off_and_observed_restart(self):
        receipt = self.accept()
        states = []
        backend = self.make_client(on_state=states.append)
        result = backend.verify_and_save(receipt)
        self.assertTrue(result['saved'])
        self.assertTrue(result['restart_confirmed'])
        self.assertEqual(states, ['saving_config', 'verifying_config'])
        self.assertLess(self.host.exec_commands.index('TURN_OFF_HEATERS'), self.host.exec_commands.index('SAVE_CONFIG'))
        self.assertIn('startup', self.host.observations)
        self.assertFalse(self.host.pending)
        self.assertEqual(self.host.config['load_cell_probe']['reference_tare_counts'], '-217754')
        with self.assertRaisesRegex(KlipperLoadCalibrationError, 'already submitted'):
            backend.verify_and_save(receipt)
        self.assertEqual(self.host.exec_commands.count('SAVE_CONFIG'), 1)

    def test_save_http_disconnect_requires_and_can_pass_independent_restart_verification(self):
        receipt = self.accept()
        self.host.disconnect_save = True
        self.assertTrue(self.make_client().verify_and_save(receipt)['saved'])
        self.assertEqual(self.host.exec_commands.count('SAVE_CONFIG'), 1)

    def test_unconfigured_empty_bed_object_does_not_block_saving(self):
        original = self.host.status
        def without_bed():
            result = original()
            result['heater_bed'] = {}
            return result
        self.host.status = without_bed
        receipt = self.accept()
        self.assertTrue(self.make_client().verify_and_save(receipt)['saved'])

    def test_configured_bed_requires_zero_target_feedback(self):
        self.host.config['heater_bed'] = {'max_temp': '120'}
        original = self.host.status
        def missing_bed():
            result = original()
            result['heater_bed'] = {}
            return result
        self.host.status = missing_bed
        receipt = self.accept()
        with self.assertRaisesRegex(KlipperLoadCalibrationError, 'shutoff'):
            self.make_client().verify_and_save(receipt)
        self.assertNotIn('SAVE_CONFIG', self.host.exec_commands)

    def test_same_persisted_values_without_observed_restart_are_not_success(self):
        receipt = self.accept()
        self.host.restart_delay = 0.
        with self.assertRaisesRegex(KlipperLoadCalibrationError, 'restart'):
            self.make_client().verify_and_save(receipt)
        self.assertEqual(self.host.exec_commands.count('SAVE_CONFIG'), 1)

    def test_save_disconnect_without_persistence_never_reports_success_or_retries(self):
        receipt = self.accept()
        self.host.persist = False
        self.host.disconnect_save = True
        with self.assertRaisesRegex(KlipperLoadCalibrationError, 'unverified'):
            self.make_client().verify_and_save(receipt)
        self.assertEqual(self.host.exec_commands.count('SAVE_CONFIG'), 1)

    def test_unrelated_pending_added_before_save_prevents_heat_off_and_save(self):
        receipt = self.accept()
        self.host.pending['extruder'] = {'pid_kp': '123'}
        with self.assertRaisesRegex(KlipperLoadCalibrationError, 'pending'):
            self.make_client().verify_and_save(receipt)
        self.assertNotIn('TURN_OFF_HEATERS', self.host.exec_commands)
        self.assertNotIn('SAVE_CONFIG', self.host.exec_commands)

    def test_unrelated_config_after_restart_is_not_accepted(self):
        receipt = self.accept()
        self.host.on_save = lambda: self.host.config['extruder'].update(max_temp='310')
        with self.assertRaisesRegex(KlipperLoadCalibrationError, 'Unrelated configuration'):
            self.make_client().verify_and_save(receipt)

    def test_explicit_clear_identifies_native_helper_and_keeps_config_unchanged(self):
        self.host.helper()
        before = copy.deepcopy(self.host.config)
        result = self.client.cancel_existing()
        self.assertTrue(result['cleared'])
        self.assertEqual(self.host.config, before)
        self.assertNotIn('TARE', self.host.commands)
        self.assertNotIn('SAVE_CONFIG', self.host.exec_commands)

    def test_clear_no_helper_is_noop_and_other_abort_tool_is_not_touched(self):
        self.assertFalse(self.client.cancel_existing()['cleared'])
        self.host.commands['ABORT'] = {'help': 'Abort manual probing'}
        with self.assertRaisesRegex(KlipperLoadCalibrationError, 'not an identified'):
            self.client.cancel_existing()
        self.assertEqual(self.host.scripts, [])

    def test_clear_refuses_pending_configuration(self):
        self.host.helper()
        self.host.pending = {'load_cell_probe': {'counts_per_gram': '200'}}
        with self.assertRaisesRegex(KlipperLoadCalibrationError, 'Pending'):
            self.client.cancel_existing()
        self.assertEqual(self.host.scripts, [])


if __name__ == '__main__':
    unittest.main()
