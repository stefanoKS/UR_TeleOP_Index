"""GUI integration boundaries require no robot or physical serial device."""
import json
import time
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from ur_vr_teleop.dashboard_ros import RosDataBridge
from ur_vr_teleop.gui import OperatorGUI


@pytest.mark.parametrize('data',[[],{'feedback':[]},{'identity':'bad'}, {'normalized_trigger':float('nan')}])
def test_malformed_gripper_status_cannot_break_dashboard(data):
    gui=OperatorGUI.__new__(OperatorGUI)
    gui.ros_data=RosDataBridge()
    gui.on_gripper(SimpleNamespace(data=json.dumps(data)))
    assert not gui.ros_data.drain()


def test_valid_gripper_status_is_coalesced_for_tk_thread():
    gui=OperatorGUI.__new__(OperatorGUI)
    gui.ros_data=RosDataBridge()
    gui.on_gripper(SimpleNamespace(data=json.dumps({
        'connection':'Disconnected','feedback':{},'identity':{'servo_id':1},
        'normalized_trigger':.5,
    })))
    items=gui.ros_data.drain()
    assert len(items)==1 and items[0][1]=='gripper'


def test_stop_withdraws_calibration_consent_before_disarm():
    gui=OperatorGUI.__new__(OperatorGUI)
    gui.startup=Mock();gui.startup_request=None;gui.heartbeat=Mock();gui.call=Mock()
    gui.gripper_panel=SimpleNamespace(closed=False,authorized=Mock(),publisher=Mock())
    gui.stop_teleop()
    gui.gripper_panel.authorized.set.assert_called_once_with(False)
    assert gui.gripper_panel.publisher.publish.call_args.args[0].data is False
    assert gui.heartbeat_allowed is False


def test_stop_all_withdraws_arm_and_gripper_independently():
    from concurrent.futures import Future
    from ur_vr_teleop.gripper_control import GripperControl
    gui=OperatorGUI.__new__(OperatorGUI)
    gui.startup=Mock();gui.startup_request=None;gui.program=Mock();gui.program_request=None
    gui.heartbeat=Mock();gui.gripper_heartbeat=Mock();gui.call=Mock();gui.notify=Mock()
    gui.gripper_control=GripperControl();gui.gripper_requests=[];gui.gripper_enable_at=None
    gui.gripper_enable_client=Mock();gui.gripper_enable_client.service_is_ready.return_value=True
    future=Future();future.set_result(SimpleNamespace(success=True,message='disabled'))
    gui.gripper_enable_client.call_async.return_value=future
    gui.gripper_panel=SimpleNamespace(closed=False,authorized=Mock(),publisher=Mock())
    gui.stop_all()
    gui.program.stop.assert_called_once()
    gui.call.assert_called_with('disarm')
    assert gui.gripper_enable_client.call_async.call_args.args[0].data is False
    assert gui.gripper_panel.publisher.publish.call_args.args[0].data is False
    assert gui.gripper_heartbeat_allowed is False
    assert gui.gripper_control.state == 'STOPPING'


def test_stop_teleop_does_not_touch_gripper_session():
    gui=OperatorGUI.__new__(OperatorGUI)
    gui.startup=Mock();gui.startup_request=None;gui.program=Mock();gui.program_request=None
    gui.heartbeat=Mock();gui.gripper_heartbeat=Mock();gui.call=Mock();gui.notify=Mock()
    gui.gripper_control=__import__('ur_vr_teleop.gripper_control',fromlist=['GripperControl']).GripperControl()
    gui.gripper_requests=[];gui.gripper_enable_at=None
    gui.gripper_enable_client=Mock();gui.gripper_enable_client.service_is_ready.return_value=True
    gui.stop_teleop()
    gui.gripper_enable_client.call_async.assert_not_called()
    assert gui.gripper_control.state == 'READY'


def test_stop_gripper_does_not_disarm_arm():
    gui=OperatorGUI.__new__(OperatorGUI)
    gui.heartbeat=Mock();gui.gripper_heartbeat=Mock();gui.call=Mock();gui.notify=Mock()
    gui.gripper_control=__import__('ur_vr_teleop.gripper_control',fromlist=['GripperControl']).GripperControl()
    gui.gripper_requests=[];gui.gripper_enable_at=None
    gui.gripper_enable_client=Mock();gui.gripper_enable_client.service_is_ready.return_value=True
    from concurrent.futures import Future
    future=Future();future.set_result(SimpleNamespace(success=True,message='disabled'))
    gui.gripper_enable_client.call_async.return_value=future
    gui.set_gripper_enabled(False)
    gui.call.assert_not_called()
    assert gui.gripper_control.state == 'STOPPING'


def test_stop_all_cancels_scheduled_enable_before_service_call():
    from ur_vr_teleop.gripper_control import GripperControl
    gui=OperatorGUI.__new__(OperatorGUI)
    gui.startup=Mock();gui.startup_request=None;gui.program=Mock();gui.program_request=None
    gui.heartbeat=Mock();gui.gripper_heartbeat=Mock();gui.call=Mock();gui.notify=Mock()
    gui.gripper_control=GripperControl();gui.gripper_requests=[];gui.gripper_enable_at=None
    gui.gripper_panel=SimpleNamespace(closed=False,authorized=Mock(),publisher=Mock())
    gui.gripper_enable_client=Mock();gui.gripper_enable_client.service_is_ready.return_value=True
    from concurrent.futures import Future
    future=Future();future.set_result(SimpleNamespace(success=True,message='disabled'))
    gui.gripper_enable_client.call_async.return_value=future
    gui.set_gripper_enabled(True)
    assert gui.gripper_enable_at is not None and gui.gripper_control.state=='ENABLING'
    gui.stop_all()
    assert gui.gripper_enable_at is None
    assert gui.gripper_control.state == 'STOPPING'
    assert gui.gripper_enable_client.call_async.call_args.args[0].data is False


def test_stop_all_beats_pending_enable_and_late_ack_is_ignored():
    from concurrent.futures import Future
    from ur_vr_teleop.gripper_control import GripperControl
    gui=OperatorGUI.__new__(OperatorGUI)
    gui.startup=Mock();gui.startup_request=None;gui.program=Mock();gui.program_request=None
    gui.heartbeat=Mock();gui.gripper_heartbeat=Mock();gui.call=Mock();gui.notify=Mock()
    gui.gripper_control=GripperControl();gui.gripper_requests=[];gui.gripper_enable_at=None
    gui.gripper_panel=SimpleNamespace(closed=False,authorized=Mock(),publisher=Mock())
    gui.gripper_enable_client=Mock();gui.gripper_enable_client.service_is_ready.return_value=True
    enable=Future();disable=Future();disable.set_result(SimpleNamespace(success=True,message='disabled'))
    gui.gripper_enable_client.call_async.side_effect=[disable]
    gui.gripper_control.state='ENABLING';gui.gripper_control.generation=1
    gui.gripper_control.pending='enable';gui.gripper_control.deadline=time.monotonic()+2
    gui.gripper_requests=[('enable',enable,gui.gripper_enable_client,1,time.monotonic())]
    gui.stop_all()
    assert gui.gripper_control.generation == 2
    assert gui.gripper_enable_client.call_async.call_args.args[0].data is False
    enable.set_result(SimpleNamespace(success=True,message='enabled late'))
    gui.poll_gripper_requests(time.monotonic())
    assert gui.gripper_control.state == 'STOPPING'
    assert gui.gripper_heartbeat_allowed is False
