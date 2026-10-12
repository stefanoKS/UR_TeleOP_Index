"""Finite differences of staged position targets. Never edits or rejects commands.

These are command-stream estimates, not measured joint acceleration or the UR
internal controller's sanity-check signal. Source timestamps avoid DDS burst jitter.
"""
import math
import numpy as np
from .validation import JOINTS


class JointDiagnostics:
    def __init__(self):
        self.reset()

    def reset(self, keep_latest=False):
        self.q = self.dq = self.stamp = None
        if not keep_latest:
            self.latest = {'state': 'Waiting for three ordered target samples'}

    def observe(self, names, positions, stamp_ns, now_ns, acceleration_warning, velocity_warning):
        if (len(names) != 6 or set(names) != set(JOINTS) or len(positions) != 6
                or stamp_ns <= 0 or not 0 <= (now_ns-stamp_ns)/1e9 < .1
                or not all(math.isfinite(x) for x in positions)):
            self.reset()
            self.latest = {'state': 'Invalid/stale target sample; diagnostic baseline reset'}
            return self.latest
        q = np.array([positions[names.index(j)] for j in JOINTS])
        dt = None if self.stamp is None else (stamp_ns-self.stamp)/1e9
        if dt is not None and not 1e-6 <= dt <= .1:
            self.reset()
            self.latest = {'state': 'Target timestamp discontinuity; diagnostic baseline reset'}
            return self.latest
        if dt is not None:
            dq = (q-self.q)/dt
            ddq = None if self.dq is None else (dq-self.dq)/dt
            warnings = [j for i, j in enumerate(JOINTS) if abs(dq[i]) > velocity_warning
                        or (ddq is not None and abs(ddq[i]) > acceleration_warning)]
            self.latest = {'state': 'Command estimates only', 'dt_s': dt,
                           'joint_names': list(JOINTS), 'dq_rad_s': dq.tolist(),
                           'ddq_rad_s2': None if ddq is None else ddq.tolist(),
                           'warning_joints': warnings}
            self.dq = dq
        self.q, self.stamp = q, stamp_ns
        return self.latest
