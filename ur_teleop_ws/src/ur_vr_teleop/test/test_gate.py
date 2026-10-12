import numpy as np
import pytest
from ur_vr_teleop.gate import Gate
from ur_vr_teleop.validation import JOINTS


def command(gate, **changes):
    values = dict(names=list(JOINTS), positions=[.001]*6, velocities=[], accelerations=[], effort=[],
                  horizon=.06, source_ns=1_010_000_000, now_ns=1_020_000_000,
                  measured=[0.]*6, now=10.)
    values.update(changes)
    return gate.validate(**values)


@pytest.fixture
def gate():
    g = Gate()
    assert g.arm([], False, True)
    g.accept_after_ns = 1_000_000_000
    return g


def test_arm_requires_health_release_and_standstill():
    g = Gate()
    assert not g.armed
    assert not g.arm(['optical missing'], False, True)
    assert not g.arm([], True, True)
    assert not g.arm([], False, False)
    assert g.arm([], False, True)
    g.disarm('lost optics')
    assert not g.armed
    with pytest.raises(ValueError): command(g)


@pytest.mark.parametrize('change', [
    {'source_ns': 0}, {'source_ns': 1_000_000_000}, {'source_ns': 1_030_000_000},
    {'now_ns': 1_200_000_000}, {'horizon': .2}, {'horizon': float('nan')},
    {'names': list(JOINTS[:-1]) + [JOINTS[0]]}, {'positions': [0.]*5},
    {'positions': [float('nan')]*6}, {'positions': [6.28]*6},
    {'positions': [.011]*6}, {'positions': [.007]*6},
    {'velocities': [.11]*6}, {'velocities': [0.]}, {'accelerations': [.6]*6},
    {'effort': [0.]*6}, {'measured': [float('nan')]*6},
])
def test_rejects_malformed_stale_and_unbounded_targets(gate, change):
    with pytest.raises(ValueError): command(gate, **change)


def test_orders_joints_and_bounds_stream_rate(gate):
    positions = [.001, .002, .003, .004, .005, .006]
    q = command(gate, names=list(reversed(JOINTS)), positions=list(reversed(positions)))
    assert np.allclose(q, positions)
    with pytest.raises(ValueError): command(gate, positions=[.009]*6, now=10.02, source_ns=1_015_000_000)


def test_stalled_stream_cannot_resume(gate):
    command(gate)
    with pytest.raises(ValueError): command(gate, now=10.2, source_ns=1_015_000_000)


def test_duplicate_source_cannot_refresh_gate_watchdog(gate):
    command(gate)
    with pytest.raises(ValueError, match='Duplicate'): command(gate, now=10.02)


def test_joint_speed_setting_scales_rate_and_position_lead(gate):
    gate.max_joint_speed = .5
    command(gate, positions=[.02]*6)
    with pytest.raises(ValueError, match='configured 0.5 rad/s'):
        command(gate, positions=[.029]*6, now=10.01, source_ns=1_015_000_000)
    with pytest.raises(ValueError, match='measured-position lead'):
        command(gate, positions=[.051]*6, now=10.02, source_ns=1_015_000_000)
    with pytest.raises(ValueError, match='timestamp'):
        command(gate, now_ns=1_500_000_000, source_ns=1_015_000_000)


def test_position_lead_is_bounded_by_actual_execution_horizon(gate):
    # A 3 mrad step passes the old 100 ms lead allowance but would exceed
    # the chosen 0.1 rad/s if executed in Servo's actual 20 ms horizon.
    with pytest.raises(ValueError, match='measured-position lead'):
        command(gate, positions=[.003]*6, horizon=.02)
    assert np.allclose(command(gate, positions=[.001]*6, horizon=.02), [.001]*6)


def test_verified_native_ramp_accepts_lag_without_treating_target_as_actuator_step(gate):
    gate.max_joint_speed = .5
    gate.native_smoothing = True
    assert np.allclose(command(gate, positions=[.012]*6, horizon=.01), [.012]*6)
    command(gate, positions=[.025]*6, horizon=.01, source_ns=1_015_000_000, now=10.005)
    # Timestamp/watchdog and large-discontinuity checks still apply.
    with pytest.raises(ValueError, match='lead'):
        command(gate, positions=[.051]*6, source_ns=1_016_000_000, now=10.01)
    with pytest.raises(ValueError, match='rate'):
        command(gate, positions=[-.04]*6, source_ns=1_016_000_000, now=10.01)
    with pytest.raises(ValueError, match='timestamp'):
        command(gate, source_ns=1_016_000_000, now_ns=1_500_000_000)
    gate.native_smoothing = False
    with pytest.raises(ValueError, match='lead'):
        command(gate, positions=[.012]*6, horizon=.01, source_ns=1_016_000_000, now=10.01)
