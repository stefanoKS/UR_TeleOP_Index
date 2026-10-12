"""Exercise the preview callbacks with ROS messages and mocked clock/TF/outputs."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from ur_vr_sim_teleop.core import Clutch
from ur_vr_teleop.preview import Preview


@pytest.fixture
def preview(monkeypatch):
    monkeypatch.setattr('ur_vr_teleop.preview.time.monotonic', lambda: 10.)
    transform = TransformStamped()
    transform.header.stamp = Time(seconds=10).to_msg()
    transform.transform.translation.x = .4
    transform.transform.rotation.w = 1.
    return SimpleNamespace(
        core=Clutch(np.eye(3)), records={k: (10., 10_000_000_000) for k in ('pose', 'joy', 'joints')},
        hand=np.zeros(3), hand_q=[0., 0., 0., 1.],
        pressed=False, pose_ok=True, joy_ok=True, joints_ok=True, flag=True,
        flag_at=10., last_tick=10., last_status=None, base='base_link', tool='tool0',
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=10)),
        get_logger=lambda: Mock(), tf_buffer=SimpleNamespace(lookup_transform=lambda *args: transform),
        status=Mock(), markers=Mock(),
    )


def test_capture_relative_target_and_release(preview):
    Preview.tick(preview)
    assert preview.core.state == 'READY'
    preview.pressed = True
    Preview.tick(preview)
    assert preview.core.state == 'ACTIVE'
    assert np.allclose(preview.core.target, [.4, 0., 0.])
    preview.hand[0] = .025
    Preview.tick(preview)
    sphere = preview.markers.publish.call_args.args[0].markers[0]
    assert sphere.pose.position.x == pytest.approx(.42)
    preview.pressed = False
    Preview.tick(preview)
    sphere = preview.markers.publish.call_args.args[0].markers[0]
    assert sphere.pose.position.x == pytest.approx(.4)


def test_stale_feedback_deletes_target_and_requires_release(preview):
    Preview.tick(preview)
    preview.pressed = True
    Preview.tick(preview)
    preview.records['joints'] = (9., 9_000_000_000)
    Preview.tick(preview)
    assert preview.core.state == 'WAIT_RELEASE'
    assert preview.markers.publish.call_args.args[0].markers[0].action == Marker.DELETEALL
    preview.records['joints'] = (10., 10_000_000_000)
    Preview.tick(preview)
    assert preview.core.state == 'WAIT_RELEASE'
    preview.pressed = False
    Preview.tick(preview)
    assert preview.core.state == 'READY'


def test_held_at_start_and_invalid_quaternion_hide_target(preview):
    preview.pressed = True
    Preview.tick(preview)
    assert preview.core.state == 'WAIT_RELEASE'
    preview.pressed = False
    Preview.tick(preview)
    preview.hand_q = [0., 0., 0., 0.]
    Preview.tick(preview)
    assert preview.markers.publish.call_args.args[0].markers[0].action == Marker.DELETEALL


def test_timeout_diagnostic_identifies_source_and_recovery_requires_release(preview):
    preview.observation_timeout = .25
    Preview.tick(preview)
    preview.pressed = True
    Preview.tick(preview)
    preview.records['pose'] = (9.78, 9_780_000_000)
    Preview.tick(preview)
    assert preview.observation_healthy
    preview.records['pose'] = (9.74, 9_740_000_000)
    Preview.tick(preview)
    assert not preview.observation_healthy
    assert 'pose: receipt age 260 ms, source age 260 ms' in preview.observation_reason
    assert 'observation limit 250 ms' in preview.observation_reason
    preview.records['pose'] = (10., 10_000_000_000)
    Preview.tick(preview)
    assert preview.core.state == 'WAIT_RELEASE'
    preview.pressed = False
    Preview.tick(preview)
    assert preview.core.state == 'READY'
