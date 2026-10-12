import numpy as np
import pytest
from ur_vr_teleop.joint_diagnostics import JointDiagnostics
from ur_vr_teleop.validation import JOINTS


def test_timestamp_derivatives_joint_order_and_monitoring_only():
    d = JointDiagnostics()
    names = list(reversed(JOINTS))
    for i, angle in enumerate([0., .001, .003]):
        q = [angle if name == 'wrist_3_joint' else 0. for name in names]
        before = list(q)
        data = d.observe(names, q, 1_000_000_000+i*10_000_000,
                         1_005_000_000+i*10_000_000, 5., .5)
        assert q == before
    index = list(JOINTS).index('wrist_3_joint')
    assert data['dq_rad_s'][index] == pytest.approx(.2)
    assert data['ddq_rad_s2'][index] == pytest.approx(10.)
    assert data['warning_joints'] == ['wrist_3_joint']
    d.reset(keep_latest=True)
    assert d.latest == data
    assert d.dq is None


@pytest.mark.parametrize('stamp,now', [(0, 10), (999, 1001), (1_000_000_000, 1_000_000_000),
                                      (2_000_000_000, 1_000_000_000)])
def test_invalid_or_reordered_sample_resets_baseline(stamp, now):
    d = JointDiagnostics()
    d.observe(list(JOINTS), [0.]*6, 1_000_000_000, 1_000_000_000, 5., .5)
    d.observe(list(JOINTS), [1.]*6, stamp, now, 5., .5)
    assert d.q is None


def test_does_not_wrap_large_joint_discontinuities():
    d = JointDiagnostics()
    d.observe(list(JOINTS), [0.]*6, 1_000_000_000, 1_000_000_000, 5., .5)
    data = d.observe(list(JOINTS), [0.]*5+[2*np.pi], 1_010_000_000, 1_010_000_000, 5., .5)
    assert max(data['dq_rad_s']) > 600
