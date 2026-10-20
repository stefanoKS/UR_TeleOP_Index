"""Fixed-memory serial statistics and rare-event evidence (single worker owner)."""
from collections import deque
import time


class TransportError(IOError):
    def __init__(self, category, message, communication=None, motor_error=0):
        super().__init__(message)
        self.category, self.communication, self.motor_error = category, communication, motor_error


class Diagnostics:
    def __init__(self):
        self.counters = dict.fromkeys(('feedback_reads', 'rx_timeouts', 'rx_corruption',
            'invalid_responses', 'motor_errors', 'other_errors', 'suspicious_temperatures',
            'confirmed_high_events', 'connections', 'reconnects', 'disconnects',
            'consecutive_failures'), 0)
        self.timings = {}
        self.events = deque(maxlen=32)
        self.last = {}

    def transaction(self, kind, elapsed, communication, motor_error, category=None):
        stats = self.timings.setdefault(kind, dict(count=0, total_ms=0., max_ms=0., last_ms=0.))
        ms = elapsed * 1000.
        stats.update(count=stats['count'] + 1, total_ms=stats['total_ms'] + ms,
                     max_ms=max(stats['max_ms'], ms), last_ms=ms)
        self.last = dict(kind=kind, duration_ms=ms, communication=communication,
                         motor_error=motor_error, wall_ns=time.time_ns(), category=category)
        if category:
            self.counters[category] += 1
            self.counters['consecutive_failures'] += 1
            self.event('serial_error', **self.last)
        else:
            self.counters['consecutive_failures'] = 0
            if kind == 'feedback':
                self.counters['feedback_reads'] += 1

    def event(self, event, **context):
        self.events.append(dict(event=event, **{'wall_ns': time.time_ns(), **context}))

    def snapshot(self):
        return dict(counters=dict(self.counters), timings={k: dict(count=v['count'],
            mean_ms=v['total_ms']/v['count'], max_ms=v['max_ms'], last_ms=v['last_ms'])
            for k, v in self.timings.items()}, last_transaction=dict(self.last),
            last_event=dict(self.events[-1]) if self.events else None)
