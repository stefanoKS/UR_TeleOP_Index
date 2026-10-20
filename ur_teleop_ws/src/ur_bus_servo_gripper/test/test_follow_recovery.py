"""V6.3 continuous-follow, transient-signal and serial-recovery regressions.

All observations use a fake driver and controllable monotonic time. No serial
adapter or physical motor is opened.
"""
from dataclasses import replace
import time

import pytest

from test_gripper import (FakeDriver, config, drive, drive_inputs, engage,
                          worker)
from ur_bus_servo_gripper.faults import MechanicalFault, MotorFault
from ur_bus_servo_gripper.gripper_mapping import Follow, target_ticks
from ur_bus_servo_gripper.serial_diagnostics import TransportError
from ur_bus_servo_gripper.st3215_driver import Feedback


def _step_without_heartbeat(worker, raw=1.):
    """Deliver fresh pose/optical/trigger but no heartbeat update."""
    worker.now[0] += .04
    worker.update('pose', True)
    worker.update('optical', True)
    worker.update('joy', raw)
    worker.step()


def test_continuous_follow_needs_no_repeated_pickup(worker):
    engage(worker)
    count = len(FakeDriver.writes)
    for raw in (.6, .7, .8, .7, .6, .5):
        drive(worker, raw)
    status = worker.snapshot()
    assert status['follow_state'] == 'FOLLOWING'
    assert status['recovery_count'] == 0
    assert len(FakeDriver.writes) >= count


@pytest.mark.parametrize('key', ['pose', 'optical'])
def test_short_tracking_interruption_holds_then_resumes(worker, key):
    engage(worker)
    drive(worker, 1.)
    count = len(FakeDriver.writes)
    drive(worker, 1., **{key: False})
    status = worker.snapshot()
    assert status['follow_state'] == 'SIGNAL_PAUSED'
    assert worker.follow.active and status['resume_pending'] is True
    assert len(FakeDriver.writes) == count
    for _ in range(6):
        drive(worker, 1.)
    status = worker.snapshot()
    assert status['follow_state'] == 'FOLLOWING'
    assert status['recovery_count'] == 1
    assert status['resume_pending'] is False


def test_no_commands_during_verification_interval(worker):
    engage(worker)
    drive(worker, 1.)
    count = len(FakeDriver.writes)
    drive(worker, 1., pose=False)
    # follow_verify_s is 0.15 s; three cycles (0.12 s) must not write.
    for _ in range(3):
        drive(worker, 1.)
    assert len(FakeDriver.writes) == count
    assert worker.snapshot()['follow_state'] == 'RECOVERY_VERIFYING'


def test_false_true_during_feedback_read_blocks_that_cycle(worker):
    engage(worker)
    drive(worker, 1.)
    count = len(FakeDriver.writes)
    original = worker.driver.read

    def read():
        worker.update('pose', False)
        worker.update('pose', True)
        return original()

    worker.driver.read = read
    drive(worker, 1.)
    assert len(FakeDriver.writes) == count
    assert worker.snapshot()['follow_state'] == 'RECOVERY_VERIFYING'
    assert worker.follow.active


def test_verification_restarts_after_another_invalid_sample(worker):
    engage(worker)
    drive(worker, 1.)
    drive(worker, 1., pose=False)
    for _ in range(3):
        drive(worker, 1.)
    assert worker.snapshot()['follow_state'] == 'RECOVERY_VERIFYING'
    drive(worker, 1., pose=False)  # must restart continuous verification
    for _ in range(3):
        drive(worker, 1.)
    assert worker.snapshot()['follow_state'] == 'RECOVERY_VERIFYING'
    for _ in range(2):
        drive(worker, 1.)
    assert worker.snapshot()['follow_state'] == 'FOLLOWING'


def test_temporary_heartbeat_delivery_delay_recovers(worker):
    engage(worker)
    drive(worker, 1.)
    count = len(FakeDriver.writes)
    worker.now[0] += .3
    worker.update('pose', True)
    worker.update('optical', True)
    worker.update('joy', 1.)
    worker.step()
    status = worker.snapshot()
    assert status['follow_state'] == 'SIGNAL_PAUSED'
    assert 'heartbeat' in status['last_signal_loss_reason'].lower()
    assert status['session_enabled'] is True
    assert len(FakeDriver.writes) == count
    for _ in range(6):
        drive(worker, 1.)
    status = worker.snapshot()
    assert status['follow_state'] == 'FOLLOWING'
    assert status['recovery_count'] == 1


def test_explicit_false_heartbeat_never_auto_recovers(worker):
    engage(worker)
    drive(worker, 1.)
    count = len(FakeDriver.writes)
    worker.update('heartbeat', False)
    for _ in range(12):
        worker.now[0] += .04
        worker.update('pose', True)
        worker.update('optical', True)
        worker.update('joy', 1.)
        worker.update('heartbeat', True)  # delivery resumes, but STOP is latched
        worker.step()
    status = worker.snapshot()
    assert status['follow_state'] == 'DISABLED'
    assert len(FakeDriver.writes) == count
    # A new explicit enable is required and starts from deliberate pickup.
    worker.update('session_enabled', False)
    worker.step()
    worker.update('session_enabled', True)
    worker.update('heartbeat', True)
    for _ in range(3):
        drive(worker, .5)
    assert worker.snapshot()['follow_state'] == 'PICKUP_REQUIRED'
    assert len(FakeDriver.writes) == count


def test_stop_during_recovery_cancels_resume_permission(worker):
    engage(worker)
    drive(worker, 1.)
    count = len(FakeDriver.writes)
    drive(worker, 1., pose=False)
    assert worker.snapshot()['follow_state'] == 'SIGNAL_PAUSED'
    worker.update('session_enabled', False)
    worker.update('heartbeat', False)
    worker.now[0] += .04
    worker.step()
    assert worker.snapshot()['follow_state'] == 'DISABLED'
    worker.update('session_enabled', True)
    worker.update('heartbeat', True)
    for _ in range(6):
        drive(worker, 1.)
    status = worker.snapshot()
    assert status['follow_state'] == 'PICKUP_REQUIRED'
    assert status['recovery_count'] == 0
    assert len(FakeDriver.writes) == count


def test_prolonged_interruption_requires_deliberate_pickup(worker):
    engage(worker)
    drive(worker, 1.)
    count = len(FakeDriver.writes)
    for _ in range(60):  # 2.4 s, beyond the 2 s default resume window
        drive(worker, 1., pose=False)
    assert worker.follow.active is False
    for _ in range(6):
        drive(worker, 1.)
    status = worker.snapshot()
    assert status['follow_state'] == 'PICKUP_REQUIRED'
    assert status['pickup_required'] is True
    assert status['recovery_count'] == 0
    assert len(FakeDriver.writes) == count
    # Deliberate pickup still restores following without a restart.
    drive(worker, .6)
    drive(worker, .5)
    drive(worker, 1.)
    assert worker.snapshot()['follow_state'] == 'FOLLOWING'
    assert len(FakeDriver.writes) > count


def test_trigger_changes_during_loss_are_not_replayed(worker):
    engage(worker)
    drive(worker, 1.)
    count = len(FakeDriver.writes)
    for raw in (0., 1., 0., 1.):
        drive(worker, raw, pose=False)
    assert len(FakeDriver.writes) == count
    for _ in range(6):
        drive(worker, 1.)
    assert worker.snapshot()['follow_state'] == 'FOLLOWING'
    drive(worker, 1.)
    recovered = FakeDriver.writes[count:][0]
    # Fresh measured baseline (1650), bounded by 100 ticks/s * 40 ms.
    assert recovered - 1650 <= 4
    assert recovered - 1650 <= worker.config.max_target_lead_ticks


def test_rapid_alternating_dropouts_stay_bounded_and_stop_wins(worker):
    engage(worker)
    drive(worker, 1.)
    count = len(FakeDriver.writes)
    for _ in range(10):
        drive(worker, 1., pose=False)
        drive(worker, 1.)
    assert worker.follow.active
    assert worker.snapshot()['follow_state'] in ('SIGNAL_PAUSED',
                                                 'RECOVERY_VERIFYING')
    assert len(FakeDriver.writes) == count
    worker.update('session_enabled', False)
    worker.update('heartbeat', False)
    worker.now[0] += .04
    worker.step()
    assert worker.snapshot()['follow_state'] == 'DISABLED'
    assert len(FakeDriver.writes) == count


def test_threshold_suppression_does_not_freeze_convergence(config):
    c = replace(config, threshold_ticks=5, rate_ticks_s=100.,
                max_target_lead_ticks=60)
    f = Follow(c)
    f.step(.6, 1650, .04)
    f.step(.5, 1650, .04)
    assert f.active
    targets = []
    for _ in range(5):
        target = f.step(1, 1650, .04)
        targets.append(target)
        if target is not None:
            f.commit(target)
    assert None in targets and any(t is not None for t in targets)
    assert target_ticks(1, c) == 2300


def test_successful_write_updates_acknowledged_state(worker):
    engage(worker)
    drive(worker, 1.)
    status = worker.snapshot()
    assert status['write_acknowledged'] is True
    assert status['commanded_ticks'] == status['last_acknowledged_ticks']
    assert worker.follow.last_sent == status['commanded_ticks']


def test_prewrite_rejection_does_not_commit_acknowledged_state(worker):
    engage(worker)
    drive(worker, 1.)
    old_sent = worker.follow.last_sent
    old_command = worker.snapshot()['commanded_ticks']
    old_writes = len(FakeDriver.writes)
    original = worker.driver.read

    def read():
        # Quit is checked only at the pre-write gate; planning still happens.
        worker.quit.set()
        return original()

    worker.driver.read = read
    drive(worker, 1.)
    status = worker.snapshot()
    assert status['planned_target_ticks'] is not None
    assert len(FakeDriver.writes) == old_writes
    assert worker.follow.last_sent == old_sent
    assert status['last_acknowledged_ticks'] == old_command


def test_ambiguous_write_is_never_replayed(worker):
    engage(worker)
    drive(worker, 1.)
    old_writes = list(FakeDriver.writes)

    def write(*args, **kwargs):
        raise TransportError('rx_timeouts', 'write acknowledgment timeout', -6)

    worker.driver.write = write
    with pytest.raises(IOError) as caught:
        drive(worker, 1.)
    worker.handle_fault(caught.value)
    assert FakeDriver.writes == old_writes
    assert worker.driver is None
    assert worker.session_enabled is False
    worker.request('connect')
    worker.step()
    worker.update('heartbeat', True)
    for _ in range(5):
        drive_inputs(worker, 1., session=False, heartbeat=True)
    assert FakeDriver.writes == old_writes
    assert worker.snapshot()['follow_state'] in ('DISABLED', 'PICKUP_REQUIRED')


def test_single_feedback_timeout_recovers_read_only(worker):
    engage(worker)
    drive(worker, 1.)
    count = len(FakeDriver.writes)
    original = worker.driver.read
    calls = [0]

    def read():
        calls[0] += 1
        if calls[0] == 1:
            raise TransportError('rx_timeouts', 'isolated feedback timeout', -6)
        return original()

    worker.driver.read = read
    drive(worker, 1.)
    status = worker.snapshot()
    assert status['communication_state'] == 'RETRYING'
    assert status['communication_retry_count'] == 1
    assert status['feedback_valid'] is False
    assert len(FakeDriver.writes) == count
    for _ in range(10):
        drive(worker, 1.)
    status = worker.snapshot()
    assert status['communication_state'] == 'OK'
    assert status['communication_retry_count'] == 0
    assert status['communication_recovery_count'] == 1
    assert status['follow_state'] == 'FOLLOWING'
    assert status['recovery_count'] == 1
    assert len(FakeDriver.writes) > count


def test_genuine_disconnect_does_not_auto_enable_after_reconnect(worker):
    engage(worker)
    drive(worker, 1.)
    count = len(FakeDriver.writes)
    worker.driver.fail = True
    for _ in range(3):
        drive(worker, 1.)
    with pytest.raises(IOError) as caught:
        drive(worker, 1.)
    worker.handle_fault(caught.value)
    assert worker.driver is None and not worker.session_enabled
    worker.request('connect')
    worker.step()
    assert worker.session_enabled is False
    assert worker.follow_state == 'DISABLED'
    for _ in range(5):
        drive_inputs(worker, 1., session=False, heartbeat=True)
    assert worker.snapshot()['follow_state'] == 'DISABLED'
    assert len(FakeDriver.writes) == count


def test_unsafe_encoder_and_motor_faults_remain_latched(worker):
    engage(worker)
    worker.driver.read = lambda: Feedback(
        2500, 0, 0, 12., 25, 0, 0, False, time.monotonic())
    with pytest.raises(ValueError) as caught:
        drive(worker, 1.)
    worker.handle_fault(caught.value)
    assert worker.snapshot()['fault_category'] == 'MECHANICAL'
    assert worker.driver is None and not worker.session_enabled
    assert worker.snapshot()['follow_state'] == 'FAULT'


def test_motor_protocol_error_is_not_retried(worker):
    engage(worker)
    worker.driver.read = lambda: (_ for _ in ()).throw(
        TransportError('motor_errors', 'motor error header', 0, 0x04))
    with pytest.raises(IOError) as caught:
        drive(worker, 1.)
    assert worker.comm_retry_count == 0
    worker.handle_fault(caught.value)
    assert worker.snapshot()['fault_category'] == 'MOTOR'
    assert worker.driver is None


def test_motor_status_fault_remains_latched(worker):
    engage(worker)
    worker.driver.read = lambda: Feedback(
        1650, 0, 0, 12., 25, 0, 0x04, False, time.monotonic())
    with pytest.raises(MotorFault) as caught:
        drive(worker, 1.)
    worker.handle_fault(caught.value)
    assert worker.snapshot()['fault_category'] == 'MOTOR'
    assert worker.driver is None and not worker.session_enabled
    assert worker.snapshot()['follow_state'] == 'FAULT'
