"""Early approach contacts locate a new mesh point; only precision contacts count."""
from types import SimpleNamespace
from unittest.mock import Mock
import unittest
import numpy as np
from print_control.bridge_bed_probe import BridgeBedProbe


def contact(z, method):
    return dict(nozzle=np.array([0.,0.,z]),joints=np.zeros(6),load=20000.,
                tare_counts=-200000.,contact_method=method)


class MeshPrecisionTests(unittest.TestCase):
    def node(self, readings, reference=None):
        node=SimpleNamespace(config=SimpleNamespace(dry_run=False),
            homing=SimpleNamespace(max_retest_difference_mm=.5,mesh_search_step_mm=.1),
            contact_options=SimpleNamespace(speed_mm_s=.2),
            homed_z=100.,points=[(30.,30.)],index=0,_point_reference_z=reference,
            args=SimpleNamespace(floor_mm=1.),bed_point=lambda xy:np.array([0.,0.,100.]),
            homing_touch=Mock(side_effect=readings),
            retract_contact=Mock(),set_baseline=Mock())
        return node

    def test_early_contact_is_refined_once_and_is_not_a_precision_sample(self):
        locator=contact(100.4,'continuous_cancel')
        precise=contact(100.36,'bounded_precision_steps')
        node=self.node([locator,precise])
        result=BridgeBedProbe.mesh_touch(node)
        self.assertIs(result,precise)
        self.assertEqual(node.homing_touch.call_count,2)
        self.assertTrue(node.homing_touch.call_args_list[0].kwargs['allow_approach_locator'])
        self.assertEqual(node.homing_touch.call_args_list[0].kwargs['near_contact_step_mm'],.1)
        self.assertNotIn('allow_approach_locator',node.homing_touch.call_args_list[1].kwargs)
        self.assertNotIn('near_contact_step_mm',node.homing_touch.call_args_list[1].kwargs)
        node.retract_contact.assert_called_once_with()
        node.set_baseline.assert_called_once_with()
        self.assertEqual(node.homing_touch.call_args.kwargs['precision_reference_z'],100.4)
        self.assertAlmostEqual(node.homing_touch.call_args.kwargs['floor_z'],99.9)
        self.assertEqual(result['locator_contact']['nozzle_base_mm'],[0.,0.,100.4])

    def test_precision_contact_does_not_add_another_pass(self):
        precise=contact(100.,'bounded_precision_steps');node=self.node([precise])
        self.assertIs(BridgeBedProbe.mesh_touch(node),precise)
        node.retract_contact.assert_not_called()
        self.assertEqual(node.homing_touch.call_count,1)

    def test_repeat_does_not_allow_an_unloaded_approach_locator(self):
        node=self.node([contact(100.,'bounded_precision_steps')],reference=100.)
        BridgeBedProbe.mesh_touch(node)
        self.assertFalse(node.homing_touch.call_args.kwargs['allow_approach_locator'])
        self.assertIsNone(node.homing_touch.call_args.kwargs['near_contact_step_mm'])

    def test_larger_search_contact_is_refined_before_any_saved_measurement(self):
        node=self.node([contact(100.4,'stepped_locator_cancel'),contact(100.36,'bounded_precision_steps')])
        result=BridgeBedProbe.mesh_touch(node)
        self.assertEqual(result['contact_method'],'bounded_precision_steps')
        self.assertEqual(result['locator_contact']['contact_method'],'stepped_locator_cancel')
        self.assertEqual(node.homing_touch.call_count,2)

    def test_early_contact_on_repeat_is_rejected_without_retry(self):
        node=self.node([contact(100.4,'continuous_cancel')],reference=100.)
        with self.assertRaisesRegex(RuntimeError,'on a repeat'):
            BridgeBedProbe.mesh_touch(node)
        self.assertEqual(node.homing_touch.call_count,1)

    def test_changed_surface_during_refinement_cannot_loop(self):
        node=self.node([contact(100.4,'continuous_cancel'),contact(100.8,'continuous_cancel')])
        with self.assertRaisesRegex(RuntimeError,'Surface changed'):
            BridgeBedProbe.mesh_touch(node)
        self.assertEqual(node.homing_touch.call_count,2)

    def test_failed_release_blocks_next_contact(self):
        node=self.node([contact(100.4,'continuous_cancel')])
        node.retract_contact.side_effect=RuntimeError('Load did not clear')
        with self.assertRaisesRegex(RuntimeError,'did not clear'):
            BridgeBedProbe.mesh_touch(node)
        self.assertEqual(node.homing_touch.call_count,1)
        node.set_baseline.assert_not_called()
