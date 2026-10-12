import numpy as np
import pytest
from ur_vr_sim_teleop.core import Clutch, require_mock_urdf, rotation_matrix


def active(**kwargs):
    c = Clutch(np.eye(3), **kwargs)
    c.step([0, 0, 0], [0, 0, 0], False, True)
    c.step([0, 0, 0], [0, 0, 0], True, True)
    return c


def test_held_at_start_cannot_arm():
    c = Clutch(np.eye(3))
    for x in np.linspace(0, .1, 20):
        assert not c.step([x, 0, 0], [0, 0, 0], True, True).any()
    assert c.state == 'WAIT_RELEASE'


def test_scale_release_and_reanchor():
    c = active()
    c.step([.03, 0, 0], [0, 0, 0], True, True)
    np.testing.assert_allclose(c.target, [.024, 0, 0])
    assert not c.step([.03, 0, 0], [.004, 0, 0], False, True).any()
    assert not c.step([.4, 0, 0], [.004, 0, 0], True, True).any()
    np.testing.assert_allclose(c.target, [.004, 0, 0])


def test_tracking_recovery_requires_release():
    c = active()
    c.step([.03, 0, 0], [0, 0, 0], True, True)
    assert not c.step([.03, 0, 0], [0, 0, 0], True, False).any()
    for _ in range(4):
        assert not c.step([.04, 0, 0], [0, 0, 0], True, True).any()
    assert c.state == 'WAIT_RELEASE'
    c.step([.04, 0, 0], [0, 0, 0], False, True)
    c.step([.04, 0, 0], [0, 0, 0], True, True)
    assert c.step([.05, 0, 0], [0, 0, 0], True, True)[0] > 0


def test_jump_latches_stop():
    c = active()
    assert not c.step([.051, 0, 0], [0, 0, 0], True, True).any()
    assert c.state == 'WAIT_RELEASE'


def test_workspace_and_speed_limits():
    c = active()
    for x in np.linspace(0, .7, 71):
        velocity = c.step([x, 0, 0], [0, 0, 0], True, True)
        assert np.linalg.norm(velocity) <= .04 + 1e-12
        assert np.linalg.norm(c.target) <= .20
    assert c.state == 'WAIT_RELEASE'
    assert not velocity.any()


def test_rotation_maps_direction():
    c = Clutch([0, -1, 0, 1, 0, 0, 0, 0, 1])
    c.step([0, 0, 0], [0, 0, 0], False, True)
    c.step([0, 0, 0], [0, 0, 0], True, True)
    c.step([.02, 0, 0], [0, 0, 0], True, True)
    np.testing.assert_allclose(c.target, [0, .016, 0])


@pytest.mark.parametrize('value', [np.diag([-1, 1, 1]), np.eye(3)*2, np.full((3,3), np.nan)])
def test_bad_axis_map_rejected(value):
    with pytest.raises(ValueError):
        rotation_matrix(value)


def test_nonfinite_pose_stops():
    c = active()
    assert not c.step([np.nan, 0, 0], [0, 0, 0], True, True).any()
    assert c.state == 'WAIT_RELEASE'


@pytest.mark.parametrize('plugin', ['mock_components/GenericSystem', 'fake_components/GenericSystem'])
def test_mock_model_accepted(plugin):
    require_mock_urdf('<robot><ros2_control><hardware><plugin>' + plugin + '</plugin></hardware></ros2_control></robot>')


@pytest.mark.parametrize('xml', [
    '<robot/>',
    '<robot><ros2_control><hardware><plugin>ur_robot_driver/URPositionHardwareInterface</plugin></hardware></ros2_control></robot>',
    '<robot>' + '<ros2_control><hardware><plugin>mock_components/GenericSystem</plugin></hardware></ros2_control>'*2 + '</robot>',
])
def test_other_models_rejected(xml):
    with pytest.raises(ValueError):
        require_mock_urdf(xml)
