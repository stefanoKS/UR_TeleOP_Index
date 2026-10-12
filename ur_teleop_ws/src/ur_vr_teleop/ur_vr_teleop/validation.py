"""Validation for preview observations, not a motion safety gate."""
import math

JOINTS = (
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
)


def valid_joints(names, positions):
    return (len(names) == len(positions) and len(set(names)) == len(names)
            and set(JOINTS).issubset(names)
            and all(math.isfinite(p) for p in positions))


def fresh(received_at, stamp_ns, steady_now, ros_now_ns, timeout=.20):
    return (stamp_ns is not None and stamp_ns > 0
            and 0 <= steady_now - received_at < timeout
            and -0.05 <= (ros_now_ns - stamp_ns) / 1e9 < timeout)
