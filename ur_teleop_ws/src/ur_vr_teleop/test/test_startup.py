"""One-button workflow: no ROS graph or robot is involved."""
from copy import deepcopy
from concurrent.futures import Future
from unittest.mock import Mock
from types import SimpleNamespace
import pytest
from ur_vr_teleop.startup import Startup


def ready(**changes):
    return dict(dict(session_id='one', mode='LIVE OUTPUT', operator_heartbeat_fresh=True,
                     a_held=False, armed=False, enable_requested=False,
                     engineering_verified=True, robot_preparation_ready=True,
                     tracking_valid=True, observations_healthy=True, stationary=True,
                     controller_state_fresh=True, controller_ready=False,
                     controller_preparation_allowed=True, servo_status_fresh=False,
                     motion_ready=False), **changes)


def workflow():
    s=Startup();assert s.start('one');return s


def test_sequence_requires_ack_and_authoritative_readback():
    s=workflow();d=ready()
    assert s.step(d,True,0)=='prepare'
    assert s.step(d,True,1) is None
    s.acknowledge('prepare',True,'',1)
    assert s.step(d,True,2) is None  # An ack alone never proves activation.
    d['controller_ready']=True
    assert s.step(d,True,2)=='servo'
    s.acknowledge('servo',True,'',2)
    assert s.step(d,True,3) is None
    d.update(servo_status_fresh=True,motion_ready=True)
    assert s.step(d,True,3)=='enable'
    s.acknowledge('enable',True,'',3)
    assert s.state!='READY'
    d.update(enable_requested=True,armed=True)
    s.step(d,True,4)
    assert s.state=='READY'


@pytest.mark.parametrize('controller,servo,expected',[(True,False,'servo'),(True,True,'enable'),(False,True,'prepare')])
def test_already_prepared_skips_redundant_calls(controller,servo,expected):
    s=workflow();d=ready(controller_ready=controller,servo_status_fresh=servo,motion_ready=True)
    assert s.step(d,True,0)==expected


@pytest.mark.parametrize('field', ['operator_heartbeat_fresh','engineering_verified',
    'robot_preparation_ready','tracking_valid','observations_healthy','stationary',
    'controller_state_fresh','controller_preparation_allowed'])
def test_missing_prerequisite_waits_without_service_or_retry(field):
    s=workflow();d=ready(**{field:False})
    for now in (0,100,1000):assert s.step(d,True,now) is None
    assert s.requested and s.state!='READY'
    d[field]=True
    assert s.step(d,True,1001)=='prepare'


def test_pendant_instruction_and_held_a():
    s=workflow();d=ready(robot_preparation_ready=False,robot_preparation_reason='Start External Control on pendant')
    s.step(d,True,0);assert 'pendant' in s.message
    d.update(robot_preparation_ready=True,a_held=True)
    s.step(d,True,1);assert 'Release A' in s.message


@pytest.mark.parametrize('action', ['prepare','servo','enable'])
@pytest.mark.parametrize('failure', ['reject','timeout','stop','restart'])
def test_failures_cannot_continue_to_enable(action,failure):
    s=workflow();d=ready(controller_ready=action!='prepare',servo_status_fresh=action=='enable',motion_ready=True)
    assert s.step(d,True,0)==action
    assert not s.start('one')
    if failure=='reject':s.acknowledge(action,False,'service unavailable',1)
    elif failure=='timeout':s.step(d,True,6)
    elif failure=='stop':s.stop()
    else:d['session_id']='two';s.step(d,True,1)
    s.acknowledge(action,True,'late success',7)
    assert s.step(d,True,8) is None
    assert not s.requested and s.state!='READY'


def test_tracking_recovery_preserves_preparation_but_requires_backend_arm():
    s=workflow();d=ready(controller_ready=True,servo_status_fresh=True,motion_ready=True)
    assert s.step(d,True,0)=='enable';s.acknowledge('enable',True,'',0)
    d.update(enable_requested=True,armed=True);s.step(d,True,1)
    d.update(tracking_valid=False,armed=False,motion_ready=False,a_held=True)
    assert s.step(d,True,2) is None
    assert s.message=='TRACKING LOST — RELEASE A'
    d.update(tracking_valid=True)
    assert s.step(d,True,3) is None and s.state!='READY'
    d.update(a_held=False,armed=True,motion_ready=True)
    assert s.step(d,True,4) is None and s.state=='READY'
    s.stop();assert s.step(d,True,5) is None


def test_heartbeat_or_status_loss_retires_intent():
    for fresh in (True,False):
        s=workflow();d=ready(controller_ready=True,servo_status_fresh=True,motion_ready=True)
        s.step(d,True,0);s.acknowledge('enable',True,'',0)
        d.update(operator_heartbeat_fresh=False)
        s.step(d,fresh,1)
        assert not s.requested and s.state=='ERROR'


def test_gui_stop_retains_inflight_enable_then_disarms_again():
    from ur_vr_teleop.gui import OperatorGUI
    g=OperatorGUI.__new__(OperatorGUI)
    g.startup=workflow();g.startup.enabled=True
    f=Future();client=Mock()
    g.startup_request=('enable',f,client,0,1)
    g.generation=1;g.session='one';g.heartbeat=Mock();g.call=Mock()
    g.last_status=ready();g.stale=lambda:False;g.heartbeat_allowed=True
    g.stop_teleop()
    assert g.retiring_startup and not g.heartbeat_allowed
    assert g.startup_request is not None
    f.set_result(SimpleNamespace(success=True,message='enabled'))
    g.poll_startup(1)
    assert g.call.call_count==2 and g.startup_request is None
    assert not g.startup.requested and not g.heartbeat_allowed


def test_unknown_enable_outcome_requires_new_session():
    from ur_vr_teleop.gui import OperatorGUI
    g=OperatorGUI.__new__(OperatorGUI)
    g.startup=workflow();g.startup.pending='enable';g.startup.state='ENABLING_TELEOP'
    g.startup_request=('enable',Future(),Mock(),0,1)
    g.generation=1;g.session='one';g.heartbeat=Mock();g.call=Mock()
    g.last_status=ready();g.stale=lambda:False;g.heartbeat_allowed=True;g.retiring_startup=False
    g.poll_startup(6)
    assert g.uncertain_enable_session=='one' and not g.heartbeat_allowed
    assert not g.startup.requested


def test_servo_previously_started_but_stale_is_not_started_twice():
    s=workflow()
    d=ready(controller_ready=True,servo_started=True,servo_status_fresh=False)
    assert s.step(d,True,0) is None
    assert 'status lost' in s.message
