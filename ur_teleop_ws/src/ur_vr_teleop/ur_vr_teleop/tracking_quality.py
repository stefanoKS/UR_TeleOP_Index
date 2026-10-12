"""Independent conservative pose continuity checks; not an accuracy certificate."""
import numpy as np
from ur_vr_sim_teleop.core import orientation


def optical_ready(counts, policy):
    if policy == 'two_station_strict':
        return sum(v >= 4 for v in counts.values()) >= 2
    if policy == 'single_station_experimental':
        return any(v >= 1 for v in counts.values())
    return False


class PoseQuality:
    def __init__(self):
        self.stamp = None
        self.position = self.rotation = None
        self.unchanged_since = None
        self.reason = 'No accepted pose'
        self.continuous_since = None
        self.accepted = 0

    @property
    def continuous(self):
        return (self.accepted >= 3 and self.continuous_since is not None and
                (self.stamp-self.continuous_since)/1e9 >= .1)

    def accept(self, stamp, position, quaternion):
        try:
            p = np.asarray(position, dtype=float)
            q = orientation(quaternion)
            if p.shape != (3,) or not np.all(np.isfinite(p)):
                raise ValueError('Nonfinite or malformed pose')
            if not np.isfinite(stamp) or stamp <= 0 or (self.stamp is not None and stamp <= self.stamp):
                raise ValueError('Duplicate or out-of-order pose timestamp')
            if self.stamp is not None:
                displacement = np.linalg.norm(p-self.position)
                angle = (q*self.rotation.inv()).magnitude()
                gap = (stamp-self.stamp)/1e9
                # Rebase after discontinuities while reporting the fault. The
                # clutch/gate latch prevents recovery while A remains held.
                if gap > .35 or displacement > .05 or angle > np.deg2rad(45):
                    self.stamp, self.position, self.rotation = stamp, p.copy(), q
                    self.unchanged_since = stamp
                    raise ValueError('Pose gap or excessive translation/orientation jump; release A')
                if displacement > 1e-10 or angle > 1e-10:
                    self.unchanged_since = stamp
                elif (stamp-self.unchanged_since)/1e9 > .5:
                    self.stamp = stamp
                    raise ValueError('Exactly frozen solver pose for 500 ms; release A')
            else:
                self.unchanged_since = stamp
                self.reason = 'none'
            self.stamp, self.position, self.rotation = stamp, p.copy(), q
            if self.continuous_since is None:
                self.continuous_since = stamp
            self.accepted += 1
            return True
        except (ValueError, TypeError) as exc:
            self.reason = str(exc)
            self.continuous_since = None
            self.accepted = 0
            return False
