"""GUI integration boundaries require no robot or physical serial device."""
import json
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from ur_vr_teleop.gui import OperatorGUI


@pytest.mark.parametrize('data',[[],{'feedback':[]},{'identity':'bad'}, {'normalized_trigger':float('nan')}])
def test_malformed_gripper_status_cannot_break_dashboard(data):
    gui=OperatorGUI.__new__(OperatorGUI)
    gui.gripper_data={};gui.gripper_at=0
    gui.on_gripper(SimpleNamespace(data=json.dumps(data)))
    assert gui.gripper_data=={} and gui.gripper_at==0


def test_stop_withdraws_calibration_consent_before_disarm():
    gui=OperatorGUI.__new__(OperatorGUI)
    gui.startup=Mock();gui.startup_request=None;gui.heartbeat=Mock();gui.call=Mock()
    gui.gripper_panel=SimpleNamespace(closed=False,authorized=Mock(),publisher=Mock())
    gui.stop_teleop()
    gui.gripper_panel.authorized.set.assert_called_once_with(False)
    assert gui.gripper_panel.publisher.publish.call_args.args[0].data is False
    assert gui.heartbeat_allowed is False
