import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R
from ur_vr_sim_teleop.core import Clutch

Z = [0, 0, 0]
I = R.identity()


def step(c, hand=I, tcp=I, pressed=True, healthy=True, position=Z):
    return c.step(position, Z, pressed, healthy, hand.as_quat(), tcp.as_quat())


def armed(hand=I, tcp=I, mapping=np.eye(3)):
    c = Clutch(mapping)
    step(c, hand, tcp, pressed=False)
    assert not step(c, hand, tcp).any()
    return c


@pytest.mark.parametrize('axis', ['x', 'y', 'z'])
def test_each_rotation_axis_and_speed_cap(axis):
    c = armed()
    out = step(c, R.from_euler(axis, 20, degrees=True))
    index = 'xyz'.index(axis)
    np.testing.assert_allclose(out[:3], 0, atol=1e-12)
    expected = np.zeros(3)
    expected[index] = np.deg2rad(15)
    np.testing.assert_allclose(out[3:], expected, atol=1e-12)
    np.testing.assert_allclose(c.target_rotation.as_rotvec()[index], np.deg2rad(20))


def test_arbitrary_anchor_and_axis_mapping():
    a = R.from_euler('zyx', [70, -20, 15], degrees=True)
    h0 = R.from_euler('xyz', [35, -40, 120], degrees=True)
    t0 = R.from_euler('zyx', [-25, 80, 65], degrees=True)
    d = R.from_rotvec([.08, -.11, .05])
    c = armed(h0, t0, a.as_matrix())
    out = step(c, d*h0, t0)
    # Independently compute with matrices; rotation increments are spatial.
    expected = a.as_matrix() @ d.as_matrix() @ a.as_matrix().T @ t0.as_matrix()
    np.testing.assert_allclose(c.target_rotation.as_matrix(), expected, atol=1e-12)
    assert np.dot(out[3:], a.apply(d.as_rotvec())) > 0
    assert np.linalg.norm(out[3:]) <= np.deg2rad(15) + 1e-12


def test_quaternion_sign_change_is_not_motion_or_jump():
    h = R.from_euler('xyz', [40, 70, 100], degrees=True)
    c = armed(h)
    out = c.step(Z, Z, True, True, -h.as_quat(), I.as_quat())
    np.testing.assert_allclose(out, 0, atol=1e-12)
    assert c.state == 'ACTIVE'


def test_crossing_180_degrees_uses_small_relative_rotation():
    c = armed(R.from_euler('z', 179, degrees=True))
    out = step(c, R.from_euler('z', -179, degrees=True))
    np.testing.assert_allclose(c.target_rotation.as_rotvec(), [0, 0, np.deg2rad(2)], atol=1e-12)
    assert 0 < out[5] < np.deg2rad(15)


def test_release_and_regrip_recaptures_actual_tool():
    c = armed()
    step(c, R.from_euler('x', 25, degrees=True))
    actual = R.from_euler('x', 8, degrees=True)
    repositioned_hand = R.from_euler('zyx', [130, 20, 60], degrees=True)
    assert not step(c, repositioned_hand, actual, pressed=False).any()
    assert not step(c, repositioned_hand, actual).any()
    np.testing.assert_allclose(c.target_rotation.as_matrix(), actual.as_matrix(), atol=1e-12)
    assert not step(c, repositioned_hand, actual).any()


def test_fault_stops_both_axes_until_release():
    c = armed()
    out = step(c, R.from_euler('y', 15, degrees=True), position=[.02, 0, 0])
    assert np.linalg.norm(out[:3]) > 0 and np.linalg.norm(out[3:]) > 0
    assert not step(c, healthy=False).any()
    assert not step(c, R.from_euler('y', 20, degrees=True)).any()
    assert c.state == 'WAIT_RELEASE'
    step(c, pressed=False)
    assert not step(c, R.from_euler('y', 20, degrees=True)).any()


@pytest.mark.parametrize('which', ['hand', 'tcp'])
@pytest.mark.parametrize('bad', [[0,0,0,0], [np.nan,0,0,1], [0,0,0,2]])
def test_bad_orientation_stops_all_motion(which, bad):
    c = armed()
    h, t = (bad, I.as_quat()) if which == 'hand' else (I.as_quat(), bad)
    assert not c.step([.02,0,0], Z, True, True, h, t).any()
    assert c.state == 'WAIT_RELEASE'


def test_angular_jump_stops_translation_too():
    c = armed()
    assert not step(c, R.from_euler('z', 46, degrees=True), position=[.02, 0, 0]).any()
    assert c.state == 'WAIT_RELEASE'


def test_combined_motion_obeys_separate_speed_limits():
    c = armed()
    for angle in np.linspace(0, 35, 100):
        out = step(c, R.from_euler('xyz', [angle]*3, degrees=True), position=[angle*.001]*3)
        assert np.linalg.norm(out[:3]) <= .04 + 1e-12
        assert np.linalg.norm(out[3:]) <= np.deg2rad(15) + 1e-12


def test_closed_loop_rotation_converges_in_base_frame():
    actual = R.from_euler('xyz', [30, -25, 80], degrees=True)
    start = actual
    hand = R.from_euler('z', 20, degrees=True)
    c = armed(tcp=actual)
    for _ in range(400):
        out = step(c, hand, actual)
        actual = R.from_rotvec(out[3:]*.02) * actual
    expected = hand * start
    assert (expected * actual.inv()).magnitude() <= np.deg2rad(.31)
