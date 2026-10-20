"""All injected temperatures/commands use fake hardware, never a serial adapter."""
from dataclasses import replace
import time
import pytest
from test_gripper import config, worker, drive, engage, FakeDriver
from ur_bus_servo_gripper.temperature import Temperature
from ur_bus_servo_gripper.st3215_driver import Feedback
from ur_bus_servo_gripper.serial_diagnostics import TransportError


def sample(w, temperature, status=0, age=0):
    w.driver.read = lambda: Feedback(1650, 0, 2, 12., temperature, 3, status,
                                    False, time.monotonic()-age)
    drive(w, 1.)


def confirm(w, value=None, error=None, age=0):
    def read():
        if error:
            raise error
        return value, time.monotonic()-age
    w.driver.read_temperature = read
    drive(w, 1.)


@pytest.mark.parametrize('active', [False, True])
def test_isolated_spike_pauses_without_replay_or_session_loss(worker, active):
    if active:
        engage(worker)
    sample(worker, 35); sample(worker, 36)
    before = list(FakeDriver.writes)
    sample(worker, 107)
    assert worker.temperature.state == 'TEMPERATURE_UNVERIFIED'
    assert worker.session_enabled and not worker.follow.active
    assert worker.snapshot()['authorized'] is False
    assert worker.update('session_enabled', True) is False
    confirm(worker, 35); sample(worker, 36)
    assert worker.temperature.state == 'NORMAL'
    assert worker.session_enabled
    assert worker.temperature.confirmations == [35, 36]
    assert worker.last_temperature_event['feedback']['temperature_c'] == 107
    for _ in range(10):
        drive(worker, 1.)
    assert FakeDriver.writes == before
    drive(worker, .5);drive(worker, .6);drive(worker, .5);drive(worker, 1.)
    assert FakeDriver.writes[-1] == 1654


@pytest.mark.parametrize('value', [60, 70, 74, 79])
def test_confirmed_high_disconnects_latched(worker, value):
    engage(worker);sample(worker, 35)
    before = list(FakeDriver.writes)
    sample(worker, value)
    confirm(worker, value+1)
    assert worker.temperature.state == 'TEMPERATURE_CONFIRMED_HIGH'
    assert worker.driver is None
    assert worker.update('session_enabled', True) is False
    assert worker.diagnostics.counters['confirmed_high_events'] == 1
    assert worker.temperature.verified == value+1
    assert FakeDriver.writes == before


@pytest.mark.parametrize('error', [TransportError('rx_timeouts', 'timeout', -6),
    TransportError('rx_corruption', 'checksum', -7),
    TransportError('invalid_responses', 'incomplete response'), IOError('motor_error=0x04')])
def test_failed_confirmation_requires_reconnect(worker, error):
    sample(worker, 35);sample(worker, 107)
    confirm(worker, error=error)
    assert worker.temperature.state == 'SENSOR_OR_COMMUNICATION_FAULT'
    assert worker.temperature.confirmations == []
    assert worker.driver is None and not FakeDriver.writes
    worker.request('connect');worker.step()
    assert worker.temperature.state == 'NORMAL' and not worker.session_enabled
    engage(worker);drive(worker, 1.)
    assert FakeDriver.writes


@pytest.mark.parametrize('status', [1, 4, 32])
def test_motor_fault_is_immediate(worker, status):
    engage(worker)
    with pytest.raises(IOError, match='Motor fault'):
        sample(worker, 70, status)
    assert not worker.session_enabled and not FakeDriver.writes
    assert worker.temperature.state != 'TEMPERATURE_UNVERIFIED'


def test_motor_fault_during_confirmation_is_not_temperature_fault(worker):
    sample(worker,35);sample(worker,107)
    worker.driver.read_temperature=lambda:(35,time.monotonic())
    drive(worker,1.)
    worker.driver.read=lambda: Feedback(1650,0,2,12.,36,3,4,False,time.monotonic())
    with pytest.raises(IOError,match='Motor fault') as caught:
        drive(worker,1.)
    worker.handle_fault(caught.value)
    assert worker.snapshot()['fault_category']=='MOTOR'
    assert worker.temperature.state=='TEMPERATURE_UNVERIFIED'
    assert not FakeDriver.writes
    worker.request('connect');worker.step()
    assert worker.temperature.state=='NORMAL'
    assert worker.snapshot()['fault_category']=='MOTOR'  # Historical until fresh status.
    drive(worker,.5)
    assert worker.snapshot()['fault_category']=='NONE' and not FakeDriver.writes


def test_hundreds_normal_cycles(worker):
    for _ in range(500):
        sample(worker, 35)
    assert worker.temperature.state == 'NORMAL'
    assert worker.diagnostics.counters['suspicious_temperatures'] == 0
    assert not FakeDriver.writes


@pytest.mark.parametrize('count', [1, 2, 4])
def test_isolated_spikes_within_policy_recover(worker, count):
    for index in range(count):
        sample(worker, 35);sample(worker, 107);confirm(worker, 35);sample(worker, 36)
        assert worker.temperature.state == 'NORMAL'
    assert worker.diagnostics.counters['suspicious_temperatures'] == count
    assert worker.temperature.status()['recent_anomaly_count'] == count
    assert worker.session_enabled


def test_five_spikes_require_manual_recovery(worker):
    for index in range(5):
        sample(worker, 35);sample(worker, 107);confirm(worker, 35);sample(worker, 36)
        assert worker.temperature.state == ('RECOVERY_REQUIRED' if index == 4 else 'NORMAL')
    assert worker.diagnostics.counters['suspicious_temperatures'] == 5
    assert not worker.session_enabled
    assert not FakeDriver.writes
    assert worker.update('session_enabled', True) is False


def test_anomalies_outside_the_window_do_not_accumulate():
    t = Temperature(60)
    for index in range(6):
        at = 100. + index * 61.
        t.sample(60, at, at)
        t.sample(35, at + .05, at + .05, independent=True)
        t.sample(36, at + .1, at + .1)
        assert t.state == 'NORMAL'
    assert t.status()['recent_anomaly_count'] == 1


def test_anomaly_policy_is_configurable():
    t = Temperature(60, anomaly_limit=2, anomaly_window_s=10.)
    for index in range(2):
        at = float(index * 3 + 1)
        t.sample(60, at, at)
        t.sample(35, at + .05, at + .05, independent=True)
        t.sample(36, at + .1, at + .1)
    assert t.state == 'RECOVERY_REQUIRED'
    assert t.status()['anomaly_limit'] == 2
    with pytest.raises(ValueError):
        Temperature(60, anomaly_limit=0)
    with pytest.raises(ValueError):
        Temperature(60, anomaly_window_s=0.)


@pytest.mark.parametrize('phase', ['register', 'block', 'deadline'])
def test_stale_confirmation_never_healthy(worker, phase):
    sample(worker, 35);sample(worker, 79)
    if phase == 'register':
        confirm(worker, 35, age=.3)
    elif phase == 'block':
        confirm(worker, 35);sample(worker, 35, age=.3)
    else:
        worker.now[0] += .3
        confirm(worker, 35)
    assert worker.temperature.state == 'SENSOR_OR_COMMUNICATION_FAULT'
    assert not FakeDriver.writes


@pytest.mark.parametrize('stop', ['gripper', 'all', 'close', 'heartbeat', 'tracking'])
def test_stop_during_confirmation_late_result_cannot_reauthorize(worker, stop):
    engage(worker);sample(worker, 107)
    def read():
        if stop == 'close':
            worker.quit.set()
        elif stop == 'tracking':
            worker.update('pose', False)
        else:
            worker.update('session_enabled', False)
            worker.update('heartbeat', False)
        return 35, time.monotonic()
    worker.driver.read_temperature = read
    drive(worker, 1.)
    if stop != 'close':
        sample(worker, 36)
    assert worker.session_enabled is (stop == 'tracking')
    assert not worker.follow.active
    assert worker.snapshot().get('authorized') is not True
    assert not FakeDriver.writes


def test_reconnect_still_hot_no_enable(worker):
    sample(worker, 79);confirm(worker, 80)
    class Hot(FakeDriver):
        def read(self):
            return replace(super().read(), temperature_c=79)
        def read_temperature(self):
            return 79, time.monotonic()
    worker.factory=Hot
    worker.request('connect');worker.step()
    assert worker.temperature.state == 'TEMPERATURE_UNVERIFIED'
    assert worker.update('session_enabled', True) is False
    drive(worker, 1.)
    assert worker.temperature.state == 'TEMPERATURE_CONFIRMED_HIGH'
    assert not FakeDriver.writes


def test_reconnect_cannot_skip_pending_confirmation(worker):
    sample(worker, 107)
    worker.driver.read_temperature=lambda:(35,time.monotonic())
    worker.now[0]+=.04
    worker.request('connect');worker.step()
    assert worker.temperature.confirmations == [35]
    assert worker.temperature.state == 'TEMPERATURE_UNVERIFIED'


def test_calibration_retired_during_verification(worker):
    worker.update('calibration',True)
    worker.request('jog_close');sample(worker,107)
    assert worker.calibration_target is None
    confirm(worker,35);sample(worker,36)
    before = list(FakeDriver.writes)
    drive(worker, .5)
    assert not FakeDriver.writes
    assert FakeDriver.writes == before
    assert worker.calibration_target is None
    assert worker.setup_lockout


@pytest.mark.parametrize('value,expected', [(59,'NORMAL'),(60,'TEMPERATURE_UNVERIFIED'),(61,'TEMPERATURE_UNVERIFIED')])
def test_threshold_boundary(value, expected):
    t=Temperature(60);t.sample(value,1.,1.)
    assert t.state == expected


def test_confirmed_fault_is_latched_even_with_later_normal_or_stale_samples():
    guard=Temperature(60)
    guard.sample(70,1.,1.)
    guard.sample(74,1.04,1.04,independent=True)
    guard.sample(79,1.08,1.08)
    guard.sample(35,1.12,1.12)
    guard.sample(35,1.,2.)
    assert guard.state=='TEMPERATURE_CONFIRMED_HIGH'


def test_duplicate_confirmation_timestamp_is_not_independent():
    guard=Temperature(60)
    guard.sample(107,1.,1.)
    guard.sample(35,1.04,1.04,independent=True)
    guard.sample(36,1.04,1.08)
    assert guard.state=='SENSOR_OR_COMMUNICATION_FAULT'
