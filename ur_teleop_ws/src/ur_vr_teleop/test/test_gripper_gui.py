"""GUI integration boundaries require no robot or physical serial device."""
import json
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
    gui=OperatorGUI.__new__(OperatorGUI)
    gui.startup=Mock();gui.startup_request=None;gui.program=Mock();gui.program_request=None
    gui.heartbeat=Mock();gui.call=Mock();gui.notify=Mock();gui.gripper_request=None
    gui.gripper_enable_client=Mock();gui.gripper_enable_client.service_is_ready.return_value=True
    future=Future();future.set_result(SimpleNamespace(success=True,message='disabled'))
    gui.gripper_enable_client.call_async.return_value=future
    gui.gripper_panel=SimpleNamespace(closed=False,authorized=Mock(),publisher=Mock())
    gui.stop_all()
    gui.program.stop.assert_called_once()
    gui.call.assert_called_with('disarm')
    assert gui.gripper_enable_client.call_async.call_args.args[0].data is False
    assert gui.gripper_panel.publisher.publish.call_args.args[0].data is False
