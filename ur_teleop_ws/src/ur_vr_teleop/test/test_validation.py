from ur_vr_teleop.validation import JOINTS, fresh, valid_joints


def test_feedback_requires_complete_unique_finite_positions():
    assert valid_joints(list(reversed(JOINTS)), [0.] * 6)
    assert not valid_joints(JOINTS[:-1], [0.] * 5)
    assert not valid_joints(JOINTS, [0.] * 5)
    assert not valid_joints(JOINTS + (JOINTS[0],), [0.] * 7)
    assert not valid_joints(JOINTS, [0.] * 5 + [float('nan')])


def test_both_receipt_and_source_must_be_fresh():
    assert fresh(10., 1_000_000_000, 10.1, 1_100_000_000)
    assert not fresh(9., 1_000_000_000, 10.1, 1_100_000_000)
    assert not fresh(10., 500_000_000, 10.1, 1_100_000_000)
    assert not fresh(10., 2_000_000_000, 10.1, 1_100_000_000)
    assert not fresh(11., 1_000_000_000, 10.1, 1_100_000_000)
    assert not fresh(10., 0, 10.1, 100_000_000)
    assert not fresh(10., None, 10.1, 1_100_000_000)


def test_configured_freshness_accepts_small_delay_but_rejects_stale_and_future():
    assert not fresh(10., 10_000_000_000, 10.22, 10_220_000_000)
    assert fresh(10., 10_000_000_000, 10.22, 10_220_000_000, .25)
    assert not fresh(10., 10_000_000_000, 10.25, 10_250_000_000, .25)
    assert not fresh(10.24, 10_000_000_000, 10.26, 10_260_000_000, .25)
    assert not fresh(10., 10_260_000_000, 10.1, 10_100_000_000, .25)
    assert not fresh(10., 0, 10.1, 10_100_000_000, .25)
