"""Worker and request failures without ROS processes or physical hardware."""
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock
import threading
import time
import pytest
from ur_vr_teleop.dashboard_process import ManagerWorker
from ur_vr_teleop.dashboard_state import ParameterState, Notifications
from ur_vr_teleop.gui import OperatorGUI


def until(predicate):
    end=time.monotonic()+3
    while not predicate() and time.monotonic()<end:time.sleep(.01)
    assert predicate()


class Manager:
    def __init__(self):
        self.children={};self.live=False;self.stopping=False;self.message='Stopped'
        self.entered=threading.Event();self.release=threading.Event()
        self.actions=[];self.threads=set();self.closed=0
    def start(self,live):
        self.threads.add(threading.get_ident());self.actions.append(('start',live))
        self.entered.set();assert self.release.wait(3)
        self.children={'test':1};self.live=live
    def stop(self,restart=None):
        self.threads.add(threading.get_ident());self.actions.append(('stop',restart))
        self.children={};self.stopping=False
    def poll(self):self.threads.add(threading.get_ident())
    def open_rviz(self):self.threads.add(threading.get_ident());raise RuntimeError('RViz unavailable')
    def close(self):self.threads.add(threading.get_ident());self.closed+=1;self.children={}


@pytest.mark.parametrize('last_action',['stop','restart','close'])
def test_worker_serializes_start_stop_restart_and_close(last_action):
    m=Manager();w=ManagerWorker(m)
    try:
        assert w.submit('start',False);assert m.entered.wait(1)
        assert not w.submit('start',False)
        if last_action=='close':w.close()
        else:
            assert w.submit('restart',True)
            if last_action=='stop':
                assert w.submit('stop')
                assert not w.submit('restart',True)
        m.release.set()
        if last_action=='close':w.join()
        else:until(lambda:len(m.actions)==2)
        assert len(m.threads)==1
        if last_action=='stop':assert m.actions[-1]==('stop',None)
        if last_action=='restart':assert m.actions[-1]==('stop',True)
    finally:
        m.release.set();w.close();w.join()
    assert m.closed==1 and not m.children


def test_worker_optional_failure_published_without_cross_thread_mutation():
    m=Manager();w=ManagerWorker(m)
    try:
        assert w.submit('rviz')
        until(lambda:'RViz unavailable' in m.message)
        assert 'failed' in w.events.get(timeout=1)[0]
    finally:w.close();w.join()


def panel():
    g=OperatorGUI.__new__(OperatorGUI)
    g.pending=[];g.request_meta={};g.generation=1
    g.parameters=ParameterState();g.notifications=Notifications();g.message=Mock()
    g.clients={'disarm':Mock()};g.enable_client=Mock();g.switch_client=Mock();g.settings=Mock()
    g.stale=lambda:False
    return g


@pytest.mark.parametrize('kind',['timeout','exception','old-session','stale'])
def test_requests_timeout_exception_and_old_session_never_succeed(kind):
    g=panel();f=Future();g.track_request('settings',f)
    g.parameters.begin({}, {}, False)
    if kind=='timeout':g.request_meta[f]=(time.monotonic()-6,1,None)
    elif kind=='exception':f.set_exception(RuntimeError('service disappeared'))
    elif kind=='old-session':g.generation=2;f.set_result(SimpleNamespace(result=SimpleNamespace(successful=True)))
    else:g.stale=lambda:True;f.set_result(SimpleNamespace(result=SimpleNamespace(successful=True)))
    g.finish_requests()
    assert not g.pending
    assert 'confirmed' not in g.notifications.current
    if kind=='timeout':assert f.cancelled()
    g.settings.remove_pending_request.assert_called_once_with(f)


def test_duplicate_requests_and_disarm_independence():
    g=panel();g.clients.update(servo=Mock())
    g.clients['servo'].call_async.return_value=Future()
    g.clients['disarm'].call_async.return_value=Future()
    g.call('servo');g.call('servo');g.call('disarm')
    g.clients['servo'].call_async.assert_called_once()
    g.clients['disarm'].call_async.assert_called_once()
    g.cancel_requests();assert not g.pending


def test_readback_timeout_does_not_leave_form_permanently_locked():
    g=panel();g.parameters.begin({}, {}, False);g.parameters.phase='readback'
    g.confirmation_pending={'alignment_verified':True};g.readback_started=time.monotonic()-6
    g.settings_result='Do not show success'
    g.check_readback(time.monotonic())
    assert not g.parameters.pending and not g.parameters.confirmed
    assert not g.settings_result and 'awaiting readback' in g.notifications.current
