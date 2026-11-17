"""Bounded natural stops retain contact without weakening fault cancellation."""
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

from print_control.continuous_probe import ContinuousHoming
from print_control.load_cell_contact import ContactDetected
from print_control.motion_planner import PlannedMotion
from test_continuous_probe import ContactHarness, Clock


class Executor:
    def execute_motion(self, motion, extrusion=None):
        self.sent.append(motion)
        self.during_motion()
        self.joints = motion.positions[-1].copy()


class Landing(ContactHarness, Executor):
    execute_motion = ContinuousHoming.execute_motion

    def __init__(self, clock):
        super().__init__(clock)
        self.sent = []
        self.during_motion = lambda: None
        self.target[2] = 100.1
        self.seed[:3] = self.target
        self.joints = self.seed.copy()
        self._continuous_homing = dict(floor_z=99.5, stop_travel_mm=0.)
        self.planner.sampled_line = self.plan

    def plan(self, name, start, end, seed, speed, *, pump):
        pump()
        q = seed.copy()
        q[:3] = end
        return PlannedMotion(name, np.array([seed, q]), np.zeros((2, 6)), np.array([0., .08]))

    def run_motion(self, motion):
        self.check_interrupted()  # Includes all planning/controller queries.
        self.execute_motion(motion)

    def check_interrupted(self):
        self.check_continuous_homing()
        if self.detector.update(self.source.snapshot()) and self.seeking:
            if not self.finish_precision_contact_step():
                self.capture_continuous_trigger()
                raise ContactDetected('contact')

    def window(self):
        self.check_interrupted()
        return self.source.snapshot()


@pytest.fixture
def landing():
    clock = Clock()
    node = Landing(clock)
    node.seeking = True
    with patch('print_control.continuous_probe.time.monotonic', clock):
        yield node


def test_contact_finishes_only_active_small_segment_then_prevents_next(landing):
    def contact():
        landing.source.counts = 25000.
        landing.check_interrupted()
    landing.during_motion = contact
    with pytest.raises(ContactDetected):
        landing.run_precision_descent(floor_z=99.5, reference_z=100., speed_mm_s=.2)
    assert len(landing.sent) == 1
    assert landing.sent[0].positions[0, 2]-landing.sent[0].positions[-1, 2] <= .010001
    assert landing._continuous_homing['trigger_at'] is not None
    assert not landing._precision_landing_executing
    assert landing._precision_landing_motion is None


def test_trigger_during_safety_queries_does_not_start_next_landing(landing):
    def query(motion):
        landing.source.counts = 25000.
        landing.check_interrupted()
        landing.execute_motion(motion)
    landing.run_motion = query
    with pytest.raises(ContactDetected):
        landing.run_precision_descent(floor_z=99.5, reference_z=100., speed_mm_s=.2)
    assert not landing.sent


def test_tenth_mm_locator_cancels_instead_of_finishing_contact_step(landing):
    original = landing.joints.copy()
    def contact():
        landing.source.counts = 25000.
        landing.check_interrupted()
    landing.during_motion = contact
    with pytest.raises(ContactDetected):
        landing.run_precision_descent(floor_z=99.5,reference_z=100.,speed_mm_s=.5,search_step_mm=.1)
    assert len(landing.sent) == 1
    assert landing.sent[0].positions[0,2]-landing.sent[0].positions[-1,2] == pytest.approx(.1)
    np.testing.assert_array_equal(landing.joints,original)
    assert landing._continuous_homing['stepped_locator'] is True
    assert not landing._continuous_homing.get('precision_landing_started')
    assert landing._precision_landing_motion is None


def test_locator_search_stops_at_floor_and_keeps_fault_guards(landing):
    with pytest.raises(RuntimeError,match='No contact'):
        landing.run_precision_descent(floor_z=99.85,reference_z=100.,speed_mm_s=.5,search_step_mm=.1)
    assert len(landing.sent) == 3
    assert min(m.positions[-1,2] for m in landing.sent) >= 99.85


@pytest.mark.parametrize('counts,age',[(120001.,0.),(0.,.151)])
def test_locator_step_faults_interrupt_without_another_step(landing,counts,age):
    def fault():
        landing.source.counts=counts
        landing.source.age=age
        landing.check_interrupted()
    landing.during_motion=fault
    with pytest.raises(RuntimeError) as error:
        landing.run_precision_descent(floor_z=99.5,reference_z=100.,speed_mm_s=.5,search_step_mm=.1)
    assert not isinstance(error.value,ContactDetected)
    assert len(landing.sent)==1


@pytest.mark.parametrize('fault', ['overload', 'stale', 'deadline', 'travel'])
def test_all_faults_interrupt_active_landing(landing, fault):
    def fail():
        landing.source.counts = 25000.
        landing.check_interrupted()
        if fault == 'overload':
            landing.source.counts = landing.contact_options.maximum+1
        elif fault == 'stale':
            landing.source.age = .151
        elif fault == 'deadline':
            landing.clock.advance(.201)
        else:
            landing.joints[2] -= .51
        landing.check_interrupted()
    landing.during_motion = fail
    with pytest.raises(RuntimeError) as error:
        landing.run_precision_descent(floor_z=99.5, reference_z=100., speed_mm_s=.2)
    assert not isinstance(error.value, ContactDetected)
    assert not landing._precision_landing_executing
    assert len(landing.sent) == 1


def test_retimed_segment_exceeding_stop_budget_never_executes(landing):
    plan = landing.plan
    def long_plan(*args, **kwargs):
        return replace(plan(*args, **kwargs), times=np.array([0., .151]))
    landing.planner.sampled_line = long_plan
    with pytest.raises(ValueError, match='stop-time budget'):
        landing.run_precision_descent(floor_z=99.5, reference_z=100., speed_mm_s=.2)
    assert not landing.sent


def test_no_trigger_stops_at_floor_without_creating_measurement(landing):
    with pytest.raises(RuntimeError, match='No contact'):
        landing.run_precision_descent(floor_z=100.08, reference_z=100.09, speed_mm_s=.2)
    assert min(m.positions[-1, 2] for m in landing.sent) >= 100.08


def test_unbounded_reference_is_rejected_before_any_motion(landing):
    for reference in (float('nan'), 99., 101.):
        with pytest.raises(ValueError, match='surface reference'):
            landing.run_precision_descent(floor_z=99.5, reference_z=reference, speed_mm_s=.2)
    assert not landing.sent
