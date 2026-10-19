"""Pure Dashboard program workflow and shutdown ordering tests."""
import time

import pytest

from ur_vr_teleop.program_control import (ProgramConfig, ProgramStartup, ProgramStopResult,
                                          ShutdownCoordinator, load_program_config,
                                          same_program, save_program_config)


def config(**changes):
    return ProgramConfig(enabled=True, program='/programs/external_control.urp', **changes)


def obs(**changes):
    return dict(dict(dashboard_ready=True, driver_ready=False, driver_status_fresh=True,
                     driver_fault=False, a_held=False, armed=False,
                     enable_requested=False), **changes)


def started():
    workflow = ProgramStartup(config())
    assert workflow.start(0.)
    return workflow


def ack(workflow, action, now, value=None, success=True, detail='ok'):
    assert workflow.pending == action
    workflow.acknowledge(action, success, detail, now, value)


def reach_loaded(workflow, loaded='/programs/external_control.urp', running=False, now=0.):
    assert workflow.step(obs(), now) == 'remote'
    ack(workflow, 'remote', now, True)
    assert workflow.step(obs(), now) == 'safety'
    ack(workflow, 'safety', now, 1)
    assert workflow.step(obs(), now) == 'mode'
    ack(workflow, 'mode', now, 7)
    assert workflow.step(obs(), now) == 'loaded'
    ack(workflow, 'loaded', now, loaded)
    assert workflow.step(obs(), now) == 'running'
    ack(workflow, 'running', now, running)
    return now


def test_remote_start_play_then_driver_readiness():
    workflow = started()
    now = reach_loaded(workflow)
    assert workflow.step(obs(), now) == 'play'
    ack(workflow, 'play', now, True)
    assert workflow.step(obs(), now + .1) is None          # bounded readback wait
    assert workflow.step(obs(), now + .3) == 'running'
    ack(workflow, 'running', now + .3, True)
    assert workflow.state != 'READY'
    workflow.step(obs(driver_ready=True), now + .4)
    assert workflow.state == 'READY'
    assert not workflow.busy
    assert 'running' in workflow.message.lower()


def test_local_control_refuses_play():
    workflow = started()
    assert workflow.step(obs(), 0) == 'remote'
    ack(workflow, 'remote', 0, False)
    workflow.step(obs(), 0)
    assert workflow.state == 'FAILED'
    assert 'Local Control' in workflow.message
    assert 'Remote Control' in workflow.message


def test_wrong_loaded_program_loads_then_verifies():
    workflow = started()
    now = reach_loaded(workflow, loaded='old.urp')
    assert workflow.step(obs(), now) == 'load'
    ack(workflow, 'load', now, True)
    assert workflow.step(obs(), now) == 'loaded'            # independent readback
    ack(workflow, 'loaded', now, '/programs/external_control.urp')
    assert workflow.step(obs(), now) == 'play'              # already known not running


def test_load_acknowledged_but_identity_unchanged_fails():
    workflow = started()
    now = reach_loaded(workflow, loaded='old.urp')
    assert workflow.step(obs(), now) == 'load'
    ack(workflow, 'load', now, True)
    assert workflow.step(obs(), now) == 'loaded'
    ack(workflow, 'loaded', now, 'old.urp')
    workflow.step(obs(), now)
    assert workflow.state == 'FAILED'
    assert 'Load acknowledged' in workflow.message


def test_different_running_program_is_never_replaced():
    workflow = started()
    now = reach_loaded(workflow, loaded='other.urp', running=True)
    assert workflow.step(obs(), now) is None
    assert workflow.state == 'FAILED'
    assert 'different robot program' in workflow.message


def test_correct_running_program_skips_play():
    workflow = started()
    now = reach_loaded(workflow, running=True)
    assert workflow.step(obs(), now) is None
    assert 'play' not in workflow.results
    assert workflow.state == 'WAITING_FOR_DRIVER_CONNECTION'


@pytest.mark.parametrize('safety', [3, 4, 5, 6, 7, 9])
def test_safety_mode_blocks_play(safety):
    workflow = started()
    assert workflow.step(obs(), 0) == 'remote'
    ack(workflow, 'remote', 0, True)
    assert workflow.step(obs(), 0) == 'safety'
    ack(workflow, 'safety', 0, safety)
    workflow.step(obs(), 0)
    assert workflow.state == 'FAILED'
    assert 'safety mode' in workflow.message


def test_play_acknowledged_without_running_readback_fails():
    workflow = started()
    now = reach_loaded(workflow)
    assert workflow.step(obs(), now) == 'play'
    ack(workflow, 'play', now, True)
    # Each poll returns false; Play is never repeated.
    for tick in range(1, 80):
        at = now + .3 + tick * .25
        action = workflow.step(obs(), at)
        if action == 'running':
            assert workflow.pending == 'running'
            workflow.acknowledge('running', True, '', at, False)
        elif workflow.state == 'FAILED':
            break
    assert workflow.state == 'FAILED'
    assert 'acknowledged' in workflow.message.lower()


def test_failed_play_is_terminal():
    workflow = started()
    now = reach_loaded(workflow)
    assert workflow.step(obs(), now) == 'play'
    ack(workflow, 'play', now, success=False, detail='program rejected')
    assert workflow.state == 'FAILED'


def test_stop_retires_late_acknowledgement():
    workflow = started()
    assert workflow.step(obs(), 0) == 'remote'
    generation = workflow.generation
    workflow.stop()
    workflow.acknowledge('remote', True, 'late', 1, True)
    assert workflow.results == {}
    assert workflow.step(obs(), 2) is None
    assert generation != workflow.generation


def test_duplicate_start_is_rejected():
    workflow = started()
    assert not workflow.start(0)
    assert not workflow.start(10)


def test_dashboard_missing_times_out_without_service_calls():
    workflow = ProgramStartup(config(timeouts={'dashboard': 1.}))
    assert workflow.start(0)
    assert workflow.step(obs(dashboard_ready=False), .5) is None
    assert workflow.state != 'FAILED'
    assert workflow.step(obs(dashboard_ready=False), 1.1) is None
    assert workflow.state == 'FAILED'
    assert 'Dashboard' in workflow.message


def test_driver_never_ready_is_bounded():
    workflow = started()
    now = reach_loaded(workflow, running=True, now=0)
    workflow.step(obs(), now)
    assert workflow.state == 'WAITING_FOR_DRIVER_CONNECTION'
    workflow.config = config(timeouts={'driver': 1.})
    assert workflow.step(obs(), 1.2) is None
    assert workflow.state == 'FAILED'
    assert 'driver did not become ready' in workflow.message


def test_held_a_waits_without_play():
    workflow = started()
    assert workflow.step(obs(), 0) == 'remote'
    ack(workflow, 'remote', 0, True)
    assert workflow.step(obs(), 0) == 'safety'
    ack(workflow, 'safety', 0, 1)
    assert workflow.step(obs(), 0) == 'mode'
    ack(workflow, 'mode', 0, 7)
    assert workflow.step(obs(a_held=True), 0) is None
    assert 'Release A' in workflow.message
    assert 'loaded' not in workflow.results


def test_existing_enable_intent_blocks_program_start():
    workflow = started()
    assert workflow.step(obs(), 0) == 'remote'
    ack(workflow, 'remote', 0, True)
    assert workflow.step(obs(), 0) == 'safety'
    ack(workflow, 'safety', 0, 1)
    assert workflow.step(obs(), 0) == 'mode'
    ack(workflow, 'mode', 0, 7)
    assert workflow.step(obs(enable_requested=True), 0) is None
    assert workflow.state == 'FAILED'


@pytest.mark.parametrize('loaded,configured,expected', [
    ('/programs/external_control.urp', '/programs/external_control.urp', True),
    ('/programs/external_control.urp/', '/programs/external_control.urp', True),
    ('external_control.urp', '/programs/external_control.urp', False),
    ('/programs/external_control.urp', 'external_control.urp', False),
    ('/other/external_control.urp', '/programs/external_control.urp', False),
    ('External_Control.urp', 'external_control.urp', False),
    ('', 'external_control.urp', False),
    (None, 'external_control.urp', False),
])
def test_program_identity(loaded, configured, expected):
    assert same_program(loaded, configured) is expected


def test_ready_requires_fresh_dashboard_verification():
    workflow = started()
    now = reach_loaded(workflow)
    workflow.step(obs(), now)
    workflow.acknowledge('play', True, '', now)
    workflow.step(obs(), now+.3)
    workflow.acknowledge('running', True, '', now+.3, True)
    workflow.step(obs(driver_ready=True), now+.4)
    assert workflow.state == 'READY'
    assert workflow.verified(now+.4)
    assert not workflow.verified(now+3.0)      # display must not claim RUNNING
    assert workflow.monitor_action(now+1.5) == 'running'
    workflow.acknowledge_monitor('running', True, '', now+1.6, True)
    assert workflow.verified(now+1.6)
    assert workflow.monitor_action(now+1.7) is None  # bounded interval


def test_dashboard_running_false_moves_program_to_stopped():
    workflow = started()
    now = reach_loaded(workflow, running=True)
    workflow.step(obs(), now)
    workflow.step(obs(driver_ready=True), now)
    assert workflow.state == 'READY'
    action = workflow.monitor_action(now+1.1)
    assert action == 'running'
    workflow.acknowledge_monitor(action, True, '', now+1.2, False)
    assert workflow.state == 'STOPPED'
    assert not workflow.verified(now+1.3)


def test_monitor_identity_change_is_a_fault():
    workflow = started()
    now = reach_loaded(workflow, running=True)
    workflow.step(obs(), now)
    workflow.step(obs(driver_ready=True), now)
    assert workflow.state == 'READY'
    workflow.last_loaded_check_at = -float('inf')
    action = workflow.monitor_action(now+1.1)
    assert action == 'loaded'
    workflow.acknowledge_monitor(action, True, '', now+1.2, '/other/program.urp')
    assert workflow.state == 'FAILED'
    assert 'identity changed' in workflow.message


def test_monitor_timeout_marks_unverified():
    workflow = started()
    now = reach_loaded(workflow, running=True)
    workflow.step(obs(), now)
    workflow.step(obs(driver_ready=True), now)
    assert workflow.monitor_action(now+1.1) == 'running'
    workflow.acknowledge_monitor('running', False, 'timeout', now+3.2)
    assert workflow.state == 'UNVERIFIED'


def test_config_requires_program_when_enabled(tmp_path):
    with pytest.raises(ValueError):
        ProgramConfig(enabled=True, program='').validated()
    with pytest.raises(ValueError):
        ProgramConfig(enabled=True, program='bad\nname').validated()


def test_program_config_override_round_trip(tmp_path):
    path = tmp_path/'real_teleop.yaml'
    path.write_text('dashboard:\n  remote_program_control_enabled: false\n  external_control_program: ""\n')
    config = ProgramConfig(enabled=True, program='external_control.urp').validated()
    save_program_config(path, config)
    loaded = load_program_config(path)
    assert loaded.enabled is True
    assert loaded.program == 'external_control.urp'


def test_stop_result_is_confirmed_only_by_readback():
    assert ProgramStopResult('stopped').confirmed
    assert ProgramStopResult('already_stopped').confirmed
    for status in ('unavailable', 'failed', 'timeout', 'unknown'):
        assert not ProgramStopResult(status).confirmed


def test_shutdown_coordinator_stops_program_before_processes():
    events = []
    coordinator = ShutdownCoordinator(
        lambda: events.append('program') or ProgramStopResult('stopped'),
        lambda: events.append('signal'))
    assert coordinator.run().confirmed
    assert events == ['program', 'signal']


def test_shutdown_coordinator_reports_but_never_skips_process_stop():
    events = []
    def fail():
        events.append('program')
        raise RuntimeError('service crashed')
    result = ShutdownCoordinator(fail, lambda: events.append('signal')).run()
    assert not result.confirmed
    assert events == ['program', 'signal']
