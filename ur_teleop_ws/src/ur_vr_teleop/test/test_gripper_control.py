"""Stop-dominant gripper authorization without ROS or hardware."""
import pytest

from ur_vr_teleop.gripper_control import GripperControl


def status(session=False, authorized=False):
    return {'session_enabled': session, 'authorized': authorized}


def test_enable_succeeds_normally():
    control = GripperControl()
    generation = control.request_enable(0)
    assert generation == 1 and control.state == 'ENABLING'
    assert control.heartbeat_allowed
    assert control.acknowledge(generation, 'enable', True, '', .1)
    assert control.state == 'ENABLED' and control.heartbeat_allowed


def test_disable_requires_status_readback():
    control = GripperControl()
    control.request_enable(0)
    control.acknowledge(1, 'enable', True, '', .1)
    generation = control.request_disable(.2)
    assert control.state == 'STOPPING' and not control.heartbeat_allowed
    control.acknowledge(generation, 'disable', True, '', .3)
    assert control.state == 'STOPPING'            # acknowledgment is not proof
    control.observe(status(False, False), True, .4)
    assert control.state == 'DISABLED' and control.confirmed_disabled
    assert not control.heartbeat_allowed


def test_stop_beats_pending_enable_and_late_ack_is_ignored():
    control = GripperControl()
    enable = control.request_enable(0)
    stop = control.request_disable(.1)
    assert stop > enable
    control.acknowledge(enable, 'enable', True, 'late success', .2)   # ignored
    assert control.state == 'STOPPING'
    assert not control.heartbeat_allowed
    control.acknowledge(stop, 'disable', True, '', .3)
    control.observe(status(False, False), True, .4)
    assert control.confirmed_disabled


def test_late_enable_failure_cannot_undo_stop():
    control = GripperControl()
    enable = control.request_enable(0)
    stop = control.request_disable(.1)
    control.acknowledge(enable, 'enable', False, 'late failure', .2)
    assert control.state == 'STOPPING'
    control.acknowledge(stop, 'disable', True, '', .3)
    control.observe(status(False, False), True, .4)
    assert control.confirmed_disabled


def test_disable_timeout_is_unconfirmed_and_heartbeat_stays_withdrawn():
    control = GripperControl(timeout=.5)
    control.request_enable(0)
    control.acknowledge(1, 'enable', True, '', .1)
    generation = control.request_disable(.2)
    control.observe(status(True, True), True, .8)
    assert generation == 1 or generation == control.generation
    assert control.state == 'UNCONFIRMED'
    assert not control.heartbeat_allowed
    assert not control.allow_enable
    assert not control.confirmed_disabled


def test_node_unavailable_is_distinct_from_confirmed_disable():
    control = GripperControl()
    control.request_disable(0)
    control.mark_unavailable('process exited')
    assert control.state == 'UNCONFIRMED'
    assert 'unavailable' in control.message
    assert not control.heartbeat_allowed


def test_double_stop_uses_latest_generation():
    control = GripperControl()
    first = control.request_disable(0)
    second = control.request_disable(.1)
    assert second > first
    control.acknowledge(first, 'disable', True, 'stale', .2)
    assert control.state == 'STOPPING'
    control.acknowledge(second, 'disable', True, '', .3)
    control.observe(status(False, False), True, .4)
    assert control.confirmed_disabled


def test_repeated_enable_while_stopping_is_rejected():
    control = GripperControl()
    control.request_disable(0)
    assert control.request_enable(.1) is None
    assert not control.allow_enable


def test_enable_after_confirmed_stop_requires_new_request():
    control = GripperControl()
    control.request_disable(0)
    control.acknowledge(1, 'disable', True, '', .1)
    control.observe(status(False, False), True, .2)
    assert control.confirmed_disabled and not control.heartbeat_allowed
    generation = control.request_enable(.3)
    assert generation is not None
    assert control.heartbeat_allowed
    control.acknowledge(generation, 'enable', True, '', .4)
    assert control.state == 'ENABLED'


def test_enable_timeout_fails_closed():
    control = GripperControl(timeout=.5)
    generation = control.request_enable(0)
    control.observe({}, False, .6)
    assert control.state == 'UNCONFIRMED'
    assert not control.heartbeat_allowed
    assert control.request_enable(.7) is None


def test_status_confirmation_not_regressed_by_late_disable_ack():
    control = GripperControl()
    generation = control.request_disable(0)
    control.observe(status(False, False), True, .1, status_at=.1)
    assert control.state == 'DISABLED'
    control.acknowledge(generation, 'disable', True, 'late ack', .2)
    assert control.state == 'DISABLED'
    assert control.confirmed_disabled


def test_external_session_loss_is_reported_disabled():
    control = GripperControl()
    control.request_enable(0)
    control.acknowledge(1, 'enable', True, '', .1)
    control.observe(status(False, False), True, .2)
    assert control.state == 'DISABLED'
    assert not control.heartbeat_allowed


def test_temperature_pause_retains_enabled_control_state():
    control = GripperControl()
    control.request_enable(0)
    control.acknowledge(1, 'enable', True, '', .1)
    paused = status(True, False)
    paused['temperature'] = {'state': 'TEMPERATURE_UNVERIFIED'}
    control.observe(paused, True, .2)
    assert control.state == 'ENABLED'
    assert control.heartbeat_allowed
