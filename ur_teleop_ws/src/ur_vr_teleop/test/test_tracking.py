import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from ur_vr_teleop.tracking import TrackingClutch, smooth_velocity


def tracking():
    c = TrackingClutch(np.eye(3), scale=1., max_speed=.5, radius=float('inf'))
    c.max_angular_speed = 2.
    c.translation_filter_alpha = c.rotation_filter_alpha = 1.
    c.translation_deadband = c.rotation_deadband = 0.
    c.step([0., 0., 0.], [0., 0., 0.], False, True)
    c.step([0., 0., 0.], [0., 0., 0.], True, True)
    return c


def test_one_to_one_target_response_gain_and_speed_bound():
    c = tracking()
    v = c.step([.01, 0., 0.], [0., 0., 0.], True, True)
    assert c.target == pytest.approx([.01, 0., 0.])
    assert v == pytest.approx([.0025, 0., 0., 0., 0., 0.])
    for _ in range(30):
        v = c.step([.01, 0., 0.], [0., 0., 0.], True, True)
    assert v[0] == pytest.approx(.018)
    for x in np.arange(.04, 1.01, .04):
        v = c.step([x, 0., 0.], [0., 0., 0.], True, True)
    assert c.state == 'ACTIVE'
    assert c.target[0] == pytest.approx(1.)
    assert 0 < v[0] <= .5


def test_one_to_one_orientation_and_release_reanchor_without_jump():
    c = tracking()
    hand_q = Rotation.from_euler('z', 10, degrees=True).as_quat()
    v = c.step([.01, 0., 0.], [0., 0., 0.], True, True, hand_q)
    assert c.target_rotation.as_rotvec()[2] == pytest.approx(np.deg2rad(10))
    assert v[5] == pytest.approx(np.deg2rad(90)*.01)
    assert np.allclose(c.step([.01, 0., 0.], [.002, 0., 0.], False, True), 0)
    assert np.allclose(c.step([.5, 0., 0.], [.002, 0., 0.], True, True), 0)
    assert c.target[0] == pytest.approx(.002)


@pytest.mark.parametrize('fault', ['stale', 'jump', 'quaternion'])
def test_tracking_fault_stops_and_requires_release(fault):
    c = tracking()
    hand = [.2, 0., 0.] if fault == 'jump' else [.01, 0., 0.]
    q = [0., 0., 0., 0.] if fault == 'quaternion' else [0., 0., 0., 1.]
    assert np.allclose(c.step(hand, [0., 0., 0.], True, fault != 'stale', q), 0)
    assert c.state == 'WAIT_RELEASE'
    assert np.allclose(c.step([.01, 0., 0.], [0., 0., 0.], True, True), 0)
    assert c.state == 'WAIT_RELEASE'


@pytest.mark.parametrize('axis', np.eye(3).tolist())
def test_mapping_preserves_distance_with_nonzero_anchors_and_alignment(axis):
    alignment = Rotation.from_euler('xyz', [13, -27, -90], degrees=True)
    c = TrackingClutch(alignment.as_matrix(), scale=1., radius=float('inf'))
    c.translation_filter_alpha = c.rotation_filter_alpha = 1.
    hand0, tool0 = np.array([.7, -.3, 1.2]), np.array([.4, .2, .5])
    c.translation_deadband = c.rotation_deadband = 0.
    hq = Rotation.from_euler('xyz', [25, 10, 45], degrees=True)
    tq = Rotation.from_euler('xyz', [-30, 20, -15], degrees=True)
    c.step(hand0, tool0, False, True, hq.as_quat(), tq.as_quat())
    c.step(hand0, tool0, True, True, hq.as_quat(), tq.as_quat())
    for distance in np.linspace(0, .2, 21):
        hand = hand0 + distance * np.asarray(axis)
        rotation = Rotation.from_rotvec(np.asarray(axis)*distance) * hq
        c.step(hand, tool0, True, True, rotation.as_quat(), tq.as_quat())
        assert c.target-tool0 == pytest.approx(alignment.apply(hand-hand0))
        metrics = c.displacement_metrics(hand, tool0, rotation.as_quat(), tq.as_quat())
        assert metrics['hand_m'] == pytest.approx(metrics['target_m'])
        assert metrics['hand_deg'] == pytest.approx(metrics['target_deg'])
    original = c.target.copy()
    for _ in range(500):
        c.step(hand, tool0, True, True, rotation.as_quat(), tq.as_quat())
    assert c.target == pytest.approx(original)  # No integration/double-counting.


def test_velocity_ramp_and_release_clear_all_history():
    c = tracking()
    last = np.zeros(6)
    for _ in range(40):
        v = c.step([.04, 0., 0.], [0., 0., 0.], True, True)
        assert np.linalg.norm(v[:3]-last[:3]) <= c.linear_acceleration*.01 + 1e-12
        last = v
    assert np.allclose(c.step([.04, 0., 0.], [0., 0., 0.], False, True), 0)
    assert np.allclose(c.velocity, 0)
    assert np.allclose(c.step([.04, 0., 0.], [.02, 0., 0.], True, True), 0)
    assert c.target == pytest.approx([.02, 0., 0.])


def test_held_target_settles_with_simple_lagged_plant():
    # Numerical regression only: this is not a UR dynamics or safety model.
    c = tracking()
    tcp, velocity = np.zeros(3), np.zeros(3)
    positions = []
    for i in range(800):
        hand = [min(.1, i*.001), 0., 0.]
        command = c.step(hand, tcp, True, True)[:3]
        velocity += (1-np.exp(-.01/.03))*(command-velocity)
        tcp += velocity*.01
        positions.append(tcp[0])
    assert abs(tcp[0]-.1) < .002
    assert max(positions) <= .102
    assert c.target == pytest.approx([.1, 0., 0.])


@pytest.mark.parametrize('dt', [0., -1., float('nan'), .5])
def test_invalid_tick_interval_stops(dt):
    c = tracking()
    assert np.allclose(c.step([.01, 0., 0.], [0., 0., 0.], True, True, dt=dt), 0)
    assert c.state == 'WAIT_RELEASE'
    assert np.allclose(c.step([.01, 0., 0.], [0., 0., 0.], True, True), 0)
    assert c.state == 'WAIT_RELEASE'


def test_norm_acceleration_bound_including_braking_and_reversal():
    previous = np.zeros(3)
    acceleration = .25
    errors = [[.1, .1, .1]]*80 + [[0., 0., 0.]]*10 + [[-.1, .05, 0.]]*60
    for i, error in enumerate(errors):
        dt = [.007, .01, .015][i % 3]
        v = smooth_velocity(np.array(error), previous, .1, acceleration, .001, 2., dt)
        assert np.linalg.norm(v-previous) <= acceleration*dt + 1e-12
        assert np.linalg.norm(v) <= .1 + 1e-12
        previous = v


def test_pose_filters_advance_only_on_new_samples_and_can_be_disabled():
    c = tracking()
    c.translation_filter_alpha = c.rotation_filter_alpha = .8
    rotation = Rotation.from_euler('z', 10, degrees=True).as_quat()
    c.step([.02, 0., 0.], [0., 0., 0.], True, True, rotation, sample_id=1)
    assert c.target[0] == pytest.approx(.016)
    assert c.target_rotation.magnitude() == pytest.approx(np.deg2rad(8))
    for _ in range(20):
        c.step([.02, 0., 0.], [0., 0., 0.], True, True, rotation, sample_id=1)
    assert c.target[0] == pytest.approx(.016)
    for sample in range(2, 14):
        c.step([.02, 0., 0.], [0., 0., 0.], True, True, rotation, sample_id=sample)
    assert c.target[0] == pytest.approx(.02, abs=1e-10)
    c.translation_filter_alpha = c.rotation_filter_alpha = 1.
    c.step([.03, 0., 0.], [0., 0., 0.], True, True, rotation, sample_id=14)
    assert c.target[0] == pytest.approx(.03)


def test_quaternion_sign_changes_and_180_crossing_do_not_spike_commands():
    c = tracking()
    last = np.zeros(6)
    for i, angle in enumerate(np.arange(1., 362.)):
        q = Rotation.from_euler('z', angle, degrees=True).as_quat() * (-1 if i % 2 else 1)
        v = c.step([0., 0., 0.], [0., 0., 0.], True, True, q)
        assert np.linalg.norm(v[3:]-last[3:]) <= c.angular_acceleration*.01 + 1e-12
        expected = Rotation.from_euler('z', angle, degrees=True)
        assert (c.target_rotation * expected.inv()).magnitude() < 1e-10
        last = v


def test_raw_jump_is_not_hidden_by_filter():
    c = tracking()
    c.translation_filter_alpha = .01
    assert np.allclose(c.step([.1, 0., 0.], [0., 0., 0.], True, True), 0)
    assert c.state == 'WAIT_RELEASE'
    assert np.allclose(c.step([.01, 0., 0.], [0., 0., 0.], True, True), 0)
    assert c.state == 'WAIT_RELEASE'


def test_default_jitter_bands_hold_target_without_integrating_noise():
    c = tracking()
    c.translation_deadband = .001
    c.rotation_deadband = np.deg2rad(.3)
    rng = np.random.default_rng(42)
    for sample in range(2000):
        hand = rng.uniform(-.0003, .0003, 3)
        q = Rotation.from_rotvec(rng.uniform(-.001, .001, 3)).as_quat()
        c.step(hand, np.zeros(3), True, True, q, sample_id=sample)
        assert c.target == pytest.approx(np.zeros(3), abs=1e-12)
        assert c.target_rotation.magnitude() < 1e-12
        assert np.linalg.norm(c.velocity) < 1e-12


def test_slow_translation_crosses_band_without_staircase_or_scale_drift():
    c = tracking()
    c.scale = 1./1.2
    c.translation_deadband = .001
    last = 0.
    for x in np.linspace(0, .12, 1201):
        c.step([x, 0., 0.], [0., 0., 0.], True, True)
        assert -1e-12 <= c.target[0]-last <= .0001/1.2+1e-12
        last = c.target[0]
    assert c.target[0] == pytest.approx((.12-.001)/1.2)
    for _ in range(200):
        c.step([.12, 0., 0.], [0., 0., 0.], True, True)
    assert c.target[0] == last


def test_rotation_mapping_remains_continuous_across_180_degrees():
    c = tracking()
    c.rotation_scale = .5
    for angle in np.arange(1., 361.):
        q = Rotation.from_euler('z', angle, degrees=True).as_quat()
        c.step([0., 0., 0.], [0., 0., 0.], True, True, q)
        expected = Rotation.from_euler('z', angle*.5, degrees=True)
        assert (c.target_rotation*expected.inv()).magnitude() < 1e-10


def test_settling_velocity_is_continuous_at_deadband():
    from ur_vr_teleop.tracking import tracking_velocity
    for x in [.000999999, .001, .001000001]:
        assert np.linalg.norm(tracking_velocity(np.array([x, 0., 0.]), 1., .001, 2.)) < 3e-9


def test_damping_opposes_measured_motion_and_zero_restores_previous_control():
    error = np.array([.02, 0., 0.])
    velocity = np.array([.03, 0., 0.])
    plain = smooth_velocity(error, np.zeros(3), .5, 100., .001, 2., .01)
    damped = smooth_velocity(error, np.zeros(3), .5, 100., .001, 2., .01, velocity, 1.)
    disabled = smooth_velocity(error, np.zeros(3), .5, 100., .001, 2., .01, velocity, 0.)
    assert damped == pytest.approx(plain-velocity)
    assert disabled == pytest.approx(plain)
    # Target changes without actual tool motion must not produce derivative kick.
    assert smooth_velocity(error, np.zeros(3), .5, 100., .001, 2., .01, np.zeros(3), 1.) == pytest.approx(plain)


def test_feedback_uses_tf_time_ignores_duplicates_and_handles_quaternion_signs():
    c = tracking()
    c.reset_feedback()
    c.update_feedback([0., 0., 0.], [0, 0, 0, 1], .01, 1_000_000_000)
    q = Rotation.from_rotvec([0, 0, .002]).as_quat()
    assert c.update_feedback([.001, 0, 0], q, .001, 1_020_000_000)
    alpha = 1-np.exp(-.02/.05)
    assert c.measured_velocity == pytest.approx([.05*alpha, 0, 0, 0, 0, .1*alpha])
    initial = c.measured_velocity.copy()
    for _ in range(20):
        c.update_feedback([.001, 0, 0], -q, .01, 1_020_000_000)
    assert c.measured_velocity == pytest.approx(initial)
    c.update_feedback([.001, 0, 0], -q, .01, 1_030_000_000)
    assert c.measured_velocity == pytest.approx(initial*np.exp(-.01/.05))
    assert not c.update_feedback([.001, 0, 0], q, .01, 1_025_000_000)
    assert c.state == 'WAIT_RELEASE'
    assert np.allclose(c.measured_velocity, 0)


@pytest.mark.parametrize('reset', ['release', 'fault', 'disabled'])
def test_damping_history_is_cleared_before_reclutch(reset):
    c = tracking()
    c.step([.01, 0, 0], [.005, 0, 0], True, True)
    assert c.measured_velocity[0] > 0
    if reset == 'release': c.step([.01, 0, 0], [.005, 0, 0], False, True)
    elif reset == 'fault': c.step([.01, 0, 0], [.005, 0, 0], True, False)
    else: c.reset_command()
    assert c.feedback_pose is None and np.allclose(c.measured_velocity, 0)
    c.step([.01, 0, 0], [.005, 0, 0], False, True)
    v = c.step([.2, 0, 0], [.005, 0, 0], True, True)
    assert np.allclose(v, 0) and np.allclose(c.measured_velocity, 0)


def test_damping_still_respects_vector_speed_and_acceleration_limits():
    previous = np.zeros(3)
    for i in range(400):
        measured = np.array([1., -2., 3.])*(-1 if i%7 else 1)
        command = smooth_velocity(np.array([.05]*3), previous, .1, .25, .001, 10., .01, measured, 1.)
        assert np.linalg.norm(command) <= .1+1e-12
        assert np.linalg.norm(command-previous) <= .0025+1e-12
        previous = command


def test_damping_reduces_sustained_sway_in_delayed_mock_without_changing_mapping():
    # Deliberately demanding outer loop, matching saved user gain/speed/ramp.
    # 50 ms transport + 120 ms first-order velocity lag, NOT a UR dynamics model.
    from collections import deque
    results = {}
    for damping in (0., 1.):
        c = tracking()
        c.response_gain, c.max_speed, c.linear_acceleration = 10., .8, .5
        c.translation_damping_gain = damping
        c.translation_filter_alpha, c.translation_deadband = .5, .001
        delay = deque([0.]*5)
        x = velocity = 0.
        positions = []
        for sample in range(1500):
            command = c.step([min(.1, sample*.002), 0, 0], [x, 0, 0], True, True)[0]
            delay.append(command)
            velocity += (1-np.exp(-.01/.12))*(delay.popleft()-velocity)
            x += .01*velocity
            positions.append(x)
        results[damping] = (np.ptp(positions[-200:]), abs(x-c.target[0]))
        assert c.target[0] == pytest.approx(.099)
    assert results[1.][0] < .1*results[0.][0]
    assert results[1.][1] < .002
    print('Delayed mock, late position span/error (m), damping off/on:', results)
