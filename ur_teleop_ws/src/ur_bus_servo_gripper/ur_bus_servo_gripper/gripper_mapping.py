"""Pure mapping and pickup logic. A is deliberately absent from this module."""
import math
from .faults import EncoderFault


def normalize(raw, config):
    if not math.isfinite(raw):
        raise ValueError('Nonfinite trigger')
    # Reject grossly out-of-profile input; tolerate small calibration margins.
    span = config.trigger_max - config.trigger_min
    if not config.trigger_min - .1 * span <= raw <= config.trigger_max + .1 * span:
        raise ValueError('Trigger outside calibrated range')
    value = max(0., min(1., (raw - config.trigger_min) / span))
    if config.invert:
        value = 1. - value
    if value <= config.deadzone:
        return 0.
    if value >= 1. - config.deadzone:
        return 1.
    return (value - config.deadzone) / (1. - 2. * config.deadzone)


def target_ticks(fraction, config):
    if not math.isfinite(fraction):
        raise ValueError('Invalid closure fraction')
    return max(config.min_ticks, min(config.max_ticks, round(
        config.open_ticks + max(0., min(1., fraction)) * (config.closed_ticks - config.open_ticks))))


class Follow:
    """After every withdrawal, move trigger >=5%, then match measured closure.

    No writes occur during pickup. Output changes are bounded in encoder ticks
    per monotonic second, independent of vendor speed interpretation.
    """
    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.initial = None
        self.moved = self.active = False
        self.reference = self.last_sent = None

    def step(self, fraction, measured, dt):
        c = self.config
        if not c.min_ticks <= measured <= c.max_ticks:
            self.reset()
            raise EncoderFault(
                'Measured encoder outside reviewed travel; inspect calibration before reconnect')
        if not self.active:
            if self.initial is None:
                self.initial = fraction
            self.moved |= abs(fraction - self.initial) >= .05
            expected = (measured - c.open_ticks) / (c.closed_ticks - c.open_ticks)
            if self.moved and abs(fraction - expected) <= .03:
                self.active = True
                self.reference = float(measured)
                self.last_sent = measured
            return None
        if abs(measured - self.last_sent) > 20:
            raise EncoderFault(
                'Encoder moved outside commanded lead; inspect mechanism and reconnect')
        desired = target_ticks(fraction, c)
        bound = c.rate_ticks_s * max(0., min(.04, dt))
        self.reference += max(-bound, min(bound, desired - self.reference))
        # Limit outstanding travel even when the motor is obstructed.
        self.reference = max(measured - 20, min(measured + 20, self.reference))
        target = max(c.min_ticks, min(c.max_ticks, round(self.reference)))
        if abs(target - self.last_sent) >= c.threshold_ticks or (target == desired and target != self.last_sent):
            self.last_sent = target
            return target
        return None
