"""Controller preparation protocol without Tk, ROS nodes, or hardware calls."""
import time
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from ur_vr_teleop.gui import OperatorGUI


def test_close_destroys_window_even_if_ros_publish_fails(monkeypatch):
    monkeypatch.setattr('ur_vr_teleop.gui.rclpy.ok', lambda: True)
    p = SimpleNamespace(closed=False, heartbeat=Mock(), root=Mock())
    p.heartbeat.publish.side_effect = RuntimeError('ROS context shutting down')
    with pytest.raises(RuntimeError):
        OperatorGUI.close(p)
    p.root.destroy.assert_called_once()
    OperatorGUI.close(p)
    p.root.destroy.assert_called_once()


def test_closed_ros_context_still_closes_window(monkeypatch):
    monkeypatch.setattr('ur_vr_teleop.gui.rclpy.ok', lambda: False)
    p = SimpleNamespace(closed=False, heartbeat=Mock(), root=Mock())
    OperatorGUI.close(p)
    p.heartbeat.publish.assert_not_called()
    p.root.destroy.assert_called_once()


def panel():
    return SimpleNamespace(last_status_at=time.monotonic(), last_status={
        'mode': 'LIVE OUTPUT', 'armed': False, 'a_held': False, 'enable_requested': False,
        'observations_healthy': True, 'stationary': True, 'robot_ready': True, 'controller_ready': False},
        message=Mock(), switch_client=Mock(), pending=[], call=Mock())


@pytest.mark.parametrize('key,value', [('armed', True), ('a_held', True), ('enable_requested', True),
    ('observations_healthy', False), ('stationary', False), ('robot_ready', False), ('mode', 'DRY RUN')])
def test_prepare_never_activates_when_preconditions_fail(key, value):
    p = panel()
    p.last_status[key] = value
    OperatorGUI.prepare_controller(p)
    p.switch_client.call_async.assert_not_called()


def test_stale_status_prevents_preparation():
    p = panel()
    p.last_status_at -= 2.
    OperatorGUI.prepare_controller(p)
    p.switch_client.call_async.assert_not_called()


def test_explicit_prepare_only_activates_scaled_and_never_resets_robot():
    p = panel()
    OperatorGUI.prepare_controller(p)
    request = p.switch_client.call_async.call_args.args[0]
    assert request.activate_controllers == ['scaled_joint_trajectory_controller']
    assert request.deactivate_controllers == []
    assert request.strictness == request.STRICT
    assert request.timeout.sec == 3
    p.call.assert_not_called()
