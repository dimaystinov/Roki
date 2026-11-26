import pytest
from roki_ng.body.step_adaptation import StepAdaptation


def test_cycle_peak_is_retained_even_if_last_sample_is_level():
    a = StepAdaptation(-2, 0, 8)
    for pitch in (-2, 18, -2):
        a.observe(pitch, 0)
    assert a.next_cycle() == .4
    for _ in range(3):
        a.observe(-2, 0)
    assert a.next_cycle() == pytest.approx(.45)


def test_sparse_samples_do_not_increase_stride():
    a = StepAdaptation(0, 0, 8)
    for _ in range(3):
        a.observe(0, 16)
    assert a.next_cycle() == .5
    a.observe(0, 0)
    assert a.next_cycle() == .5


def test_stable_gait_keeps_requested_stride():
    a = StepAdaptation(-3, 1, 8)
    for pitch in (-5, -1, 0):
        a.observe(pitch, 2)
    assert a.next_cycle() == 1
