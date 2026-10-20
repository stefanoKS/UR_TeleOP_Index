"""Bounded, read-only temperature verification; never grants motion permission."""
import math


class Temperature:
    def __init__(self, limit):
        self.limit = limit
        self.state = 'NORMAL'
        self.verified = self.verified_at = None
        self.suspicious = None
        self.confirmations = []
        self.started = self.last_confirmation_at = None
        self.reason = ''

    def sample(self, value, received_at, now, independent=False):
        if self.state not in ('NORMAL', 'TEMPERATURE_UNVERIFIED'):
            return
        if (type(value) is not int or not 0 <= value <= 255 or
                not math.isfinite(received_at) or not 0 <= now - received_at < .25 or
                (self.state == 'TEMPERATURE_UNVERIFIED' and
                 (now - self.started >= .25 or received_at <=
                  (self.last_confirmation_at if self.last_confirmation_at is not None else self.started)))):
            self.fail('Stale/invalid temperature confirmation')
            return
        if self.state == 'NORMAL':
            if value < self.limit:
                self.verified, self.verified_at = value, received_at
                return
            self.suspicious, self.started = value, received_at
            self.confirmations = []
            self.state = 'TEMPERATURE_UNVERIFIED'
            self.reason = f'Temperature unverified: received {value} C; checking sensor feedback; movement inhibited'
        elif self.state == 'TEMPERATURE_UNVERIFIED':
            # Exactly one register read followed by one full block if normal.
            if independent != (len(self.confirmations) == 0):
                self.fail('Unexpected temperature confirmation order')
                return
            self.confirmations.append(value)
            self.last_confirmation_at = received_at
            if value >= self.limit:
                self.verified, self.verified_at = value, received_at
                self.state = 'TEMPERATURE_CONFIRMED_HIGH'
                self.reason = f'Confirmed high temperature: {self.suspicious}, {value} C; cool, Connect read-only, then explicitly enable'
            elif len(self.confirmations) == 2:
                self.verified, self.verified_at = value, received_at
                self.state = 'RECOVERY_REQUIRED'
                self.reason = f'Temperature spike suspected: {self.suspicious} C; subsequent readings normal. Review, Connect read-only, then re-enable gripper'

    def fail(self, reason):
        self.state = 'SENSOR_OR_COMMUNICATION_FAULT'
        self.reason = 'Temperature feedback error: ' + reason + '; check USB/power, then Connect read-only'

    def status(self):
        return dict(state=self.state, verified_temperature_c=self.verified,
                    verified_at=self.verified_at, suspicious_temperature_c=self.suspicious,
                    confirmation_readings=list(self.confirmations), reason=self.reason)
