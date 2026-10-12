"""Delay/lag regression plant, not a UR dynamics or safety model."""
from collections import deque
import numpy as np
from .tracking import TrackingClutch


class Plant:
    def __init__(self, dt=.01, delay=.08, lag=.10, acceleration=.3):
        self.dt, self.lag, self.acceleration = dt, lag, acceleration
        self.queue = deque([np.zeros(3) for _ in range(round(delay/dt))])
        self.position, self.velocity = np.zeros(3), np.zeros(3)

    def step(self, command):
        self.queue.append(np.asarray(command).copy())
        target = self.queue.popleft()
        change = (target-self.velocity)*(1. if self.lag == 0 else -np.expm1(-self.dt/self.lag))
        self.velocity += change*min(1., self.acceleration*self.dt/max(np.linalg.norm(change), 1e-12))
        self.position += self.velocity*self.dt
        return self.position.copy()


def simulate(mode, scenario='hold', delay=.08, lag=.1, acceleration=.3):
    dt = .01
    c = TrackingClutch(np.eye(3), scale=1., max_speed=.1, radius=float('inf'))
    c.control_mode = mode
    plant = Plant(dt, delay, lag, acceleration)
    c.step([0]*3, plant.position, False, True, sample_id=1_000_000_000)
    c.step([0]*3, plant.position, True, True, sample_id=1_000_000_000)
    rows = []
    hand = 0.
    for i in range(1, 801):
        t = i*dt
        speed = (.05 if t <= 1 else 0.)
        if scenario == 'reversal': speed = .08 if t <= 1 else (-.08 if t <= 2 else 0.)
        if scenario == 'small': speed = .01 if t <= 4 and int(t/.25) % 2 == 0 else 0.
        hand += speed*dt
        v = c.step([hand, 0, 0], plant.position, True, True, dt=dt,
                   sample_id=int(1e9+i*1e7), feedback_sample_id=int(1e9+i*1e7))
        plant.step(v[:3])
        rows.append({'t': t, 'hand': hand, 'hand_speed': speed, 'command': v[0],
                     'position': plant.position[0], 'velocity': plant.velocity[0]})
    return rows


def metrics(rows):
    t = np.array([r['t'] for r in rows])
    v = np.array([r['velocity'] for r in rows])
    x = np.array([r['position'] for r in rows])
    moving = np.array([abs(r['hand_speed']) > 1e-6 for r in rows])
    last = np.flatnonzero(moving)
    end = int(last[-1]+1) if len(last) else 0
    tail = slice(min(end, len(rows)-1), None)
    settled = next((i for i in range(end, len(v)) if np.all(np.abs(v[i:]) < .001)), None)
    return {'post_hand_stop_travel_m': float(np.sum(np.abs(np.diff(x[tail])))),
            'post_hand_stop_excursion_m': float(np.ptp(x[tail])),
            'stop_latency_s': None if settled is None else float(t[settled]-t[min(end, len(t)-1)]),
            'acceleration_rms_m_s2': float(np.sqrt(np.mean((np.diff(v)/np.diff(t))**2))),
            'final_position_m': float(x[-1])}
