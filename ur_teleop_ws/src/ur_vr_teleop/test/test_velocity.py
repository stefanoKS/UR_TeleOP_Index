"""Deterministic source-time velocity, clutch and SO(3) regressions."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from ur_vr_teleop.tracking import TrackingClutch
from ur_vr_teleop.tracking_quality import PoseQuality, optical_ready


def controller(**values):
    c = TrackingClutch(np.eye(3), scale=1., max_speed=.5, radius=float('inf'))
    c.control_mode = 'velocity'
    c.translation_filter_alpha = c.rotation_filter_alpha = 1.
    c.translation_deadband = c.rotation_deadband = 0.
    c.linear_velocity_filter_s = c.angular_velocity_filter_s = 0.
    c.linear_acceleration = c.angular_acceleration = 100.
    for k, v in values.items(): setattr(c, k, v)
    c.step([0]*3, [0]*3, False, True, sample_id=1_000_000_000)
    c.step([0]*3, [0]*3, True, True, sample_id=1_000_000_000)
    return c


def step(c, x=0., stamp=1_010_000_000, **kwargs):
    return c.step([x, 0, 0], [0, 0, 0], True, True, sample_id=stamp, **kwargs)


def test_stationary_does_not_chase_robot_position_error():
    c = controller()
    assert step(c, .001)[0] == pytest.approx(.1)
    assert np.allclose(step(c, .001, 1_020_000_000), 0)
    c.response_gain = 999.
    assert np.allclose(step(c, .001, 1_030_000_000), 0)


@pytest.mark.parametrize('dt', [.005, .01, .02])
def test_actual_sample_time_gives_equal_physical_velocity(dt):
    c = controller()
    assert step(c, .1*dt, int(1e9+dt*1e9), dt=dt)[0] == pytest.approx(.1)


def test_repeated_sample_is_not_differentiated_and_missing_samples_latch_stop():
    c = controller(linear_acceleration=.5)
    step(c, .001)
    raw = c.raw_hand_velocity.copy()
    for _ in range(2): step(c, .001)
    assert c.raw_hand_velocity == pytest.approx(raw)
    for _ in range(12): v = step(c, .001)
    assert np.allclose(v, 0) and c.state == 'WAIT_RELEASE'
    assert np.allclose(step(c, .002, 1_200_000_000), 0)


def test_bounded_jitter_does_not_accumulate_motion():
    c = controller(translation_deadband=.001, rotation_deadband=.01)
    total = np.zeros(6)
    for i in range(1, 1000):
        total += step(c, .0002*np.sin(i), int(1e9+i*1e7))*.01
    assert np.allclose(total, 0)


def test_speed_acceleration_and_reversal_bounds():
    c = controller(linear_acceleration=.25, angular_acceleration=.5)
    previous = np.zeros(6)
    for i in range(1, 101):
        x = .003*i if i < 50 else .003*(100-i)
        q = Rotation.from_rotvec([0, 0, x]).as_quat()
        v = step(c, x, int(1e9+i*1e7), hand_q=q)
        assert np.linalg.norm(v[:3]) <= c.max_speed
        assert np.linalg.norm(v[3:]) <= c.max_angular_speed
        assert np.linalg.norm(v[:3]-previous[:3]) <= .0025+1e-12
        assert np.linalg.norm(v[3:]-previous[3:]) <= .005+1e-12
        previous = v
    assert v[0] < 0


def test_release_and_reengagement_at_new_location():
    c = controller()
    step(c, .001)
    assert np.allclose(c.step([.001, 0, 0], [0]*3, False, True, sample_id=1_020_000_000), 0)
    assert c.velocity_sample is None
    assert np.allclose(step(c, 1., 1_030_000_000), 0)
    assert np.allclose(step(c, 1., 1_040_000_000), 0)


@pytest.mark.parametrize('stamp,x', [(0, 0), (999_000_000, 0), (1_000_010_000, .001), (1_200_000_000, .001), (1_010_000_000, float('nan'))])
def test_invalid_samples_fault(stamp, x):
    c = controller()
    assert np.allclose(step(c, x, stamp), 0)
    assert c.state == 'WAIT_RELEASE'


def test_sign_boundary_mapping_and_simultaneous_rotation():
    c = controller()
    c.axis_rotation = Rotation.from_euler('z', 90, degrees=True)
    c.rotation = c.axis_rotation.as_matrix()
    # Fresh clutch to initialize mapped history.
    c.step([0]*3, [0]*3, False, True, sample_id=1_000_000_000)
    q = Rotation.from_euler('z', 179, degrees=True)
    c.step([0]*3, [0]*3, True, True, q.as_quat(), sample_id=1_000_000_000)
    v = step(c, .001, hand_q=-Rotation.from_euler('z', 181, degrees=True).as_quat())
    assert v[0] == pytest.approx(0, abs=1e-10) and v[1] == pytest.approx(.1)
    assert v[5] > 0 and v[5] <= c.max_angular_speed
    v = step(c, .001, 1_020_000_000, hand_q=Rotation.from_euler('z', 181, degrees=True).as_quat())
    assert np.allclose(v, 0)


def test_pose_and_optical_are_independent_requirements():
    assert not optical_ready({}, 'single_station_experimental')
    assert optical_ready({1: 1}, 'single_station_experimental')
    assert not optical_ready({1: 1}, 'two_station_strict')
    assert optical_ready({0: 4, 1: 4}, 'two_station_strict')
    q = PoseQuality()
    assert not q.accept(1, [0]*3, [0]*4)
    assert q.accept(1, [0]*3, [0, 0, 0, 1])
    assert not q.accept(1, [0]*3, [0, 0, 0, 1])
    assert not q.accept(10_000_001, [.1, 0, 0], [0, 0, 0, 1])
    assert 'jump' in q.reason


def test_exact_frozen_pose_detected_even_if_timestamps_advance():
    q = PoseQuality()
    for i in range(51): assert q.accept(1+10_000_000*i, [0]*3, [0, 0, 0, 1])
    assert not q.accept(510_000_001, [0]*3, [0, 0, 0, 1])
    assert 'frozen' in q.reason
