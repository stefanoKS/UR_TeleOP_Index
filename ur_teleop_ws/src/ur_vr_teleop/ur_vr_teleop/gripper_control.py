"""Stop-dominant gripper authorization coordinator (pure, no ROS/Tk).

STOP always wins over an in-flight ENABLE.  The coordinator tracks desired
permission, request generations and authoritative status separately.  A late
enable acknowledgment from an older generation can never restore UI permission
or the gripper heartbeat.
"""

import time

READY = 'READY'
ENABLING = 'ENABLING'
ENABLED = 'ENABLED'
STOPPING = 'STOPPING'
DISABLED = 'DISABLED'
UNCONFIRMED = 'UNCONFIRMED'


class GripperControl:
    def __init__(self, timeout=2.0):
        self.timeout = timeout
        self.state = READY
        self.generation = 0
        self.pending = None
        self.deadline = None
        self.changed_at = -float('inf')
        self.message = 'Gripper not enabled'

    @property
    def heartbeat_allowed(self):
        # Heartbeat is retained only while enabling/enabled.  A stop or an
        # unknown enable outcome withdraws it until a new explicit ENABLE.
        return self.state in (READY, ENABLING, ENABLED)

    @property
    def allow_enable(self):
        return self.state in (READY, DISABLED)

    @property
    def confirmed_disabled(self):
        return self.state == DISABLED

    @property
    def busy(self):
        return self.state in (ENABLING, STOPPING)

    @property
    def unconfirmed(self):
        return self.state == UNCONFIRMED

    def request_enable(self, now):
        if not self.allow_enable:
            return None
        self.generation += 1
        self.state = ENABLING
        self.changed_at = now
        self.pending = 'enable'
        self.deadline = now + self.timeout
        self.message = 'GRIPPER ENABLING — waiting for node confirmation'
        return self.generation

    def request_disable(self, now, reason='STOP GRIPPER'):
        # Stop-dominant: no pending request can block this.
        self.generation += 1
        self.state = STOPPING
        self.changed_at = now
        self.pending = 'disable'
        self.deadline = now + self.timeout
        self.message = 'GRIPPER STOPPING — awaiting status confirmation'
        self.stop_reason = reason
        return self.generation

    def acknowledge(self, generation, action, success, detail, now):
        if generation != self.generation:
            return False
        self.pending = None
        self.changed_at = now
        if action == 'enable':
            if success:
                self.state = ENABLED
                self.deadline = None
                self.message = 'GRIPPER ENABLED'
            else:
                self.state = DISABLED
                self.deadline = None
                self.message = 'GRIPPER ENABLE FAILED — heartbeat withdrawn: ' + str(detail)
        else:
            if not success:
                self.state = UNCONFIRMED
                self.deadline = None
                self.message = 'GRIPPER STOP UNCONFIRMED — disable rejected: ' + str(detail)
            elif self.state != DISABLED:
                self.state = STOPPING
                self.message = 'GRIPPER STOPPING — disable acknowledged; verifying status'
        return True

    def expire(self, generation, action, now):
        if generation != self.generation:
            return False
        self.pending = None
        self.deadline = None
        self.changed_at = now
        if action == 'enable':
            self.state = UNCONFIRMED
            self.message = 'GRIPPER ENABLE UNCONFIRMED — heartbeat withdrawn; STOP before retrying'
        else:
            self.state = UNCONFIRMED
            self.message = 'GRIPPER STOP UNCONFIRMED — service timeout; heartbeat withheld'
        return True

    def mark_unavailable(self, detail):
        self.generation += 1
        self.pending = None
        self.deadline = None
        self.changed_at = time.monotonic()
        self.state = UNCONFIRMED
        self.message = 'GRIPPER STOP UNCONFIRMED — process unavailable: ' + str(detail)

    def observe(self, status, fresh, now, status_at=None):
        """Apply authoritative gripper status; return True if the state changed.

        ``status_at`` is the status receipt time.  A snapshot older than the
        latest authorization acknowledgment cannot prove the new state, so its
        transitions are ignored; the deadline logic still applies.
        """
        old = self.state
        current_snapshot = (status_at is None or status_at >= self.changed_at)
        if fresh and current_snapshot and isinstance(status, dict):
            session = status.get('session_enabled') is True
            authorized = status.get('authorized') is True
            if self.state == STOPPING and not session and not authorized:
                self.state = DISABLED
                self.deadline = None
                self.message = 'GRIPPER STOPPED (confirmed by status)'
            elif self.state == UNCONFIRMED and not session and not authorized:
                self.state = DISABLED
                self.deadline = None
                self.message = 'GRIPPER STOPPED (confirmed after uncertainty)'
            elif self.state == ENABLING and session:
                self.state = ENABLED
                self.deadline = None
                self.message = 'GRIPPER ENABLED (confirmed by status)'
            elif self.state == ENABLED and not session:
                self.state = DISABLED
                self.message = 'GRIPPER DISABLED (reported by node)'
        if self.state in (STOPPING, ENABLING) and self.deadline is not None and now >= self.deadline:
            self.expire(self.generation, self.pending or
                        ('enable' if self.state == ENABLING else 'disable'), now)
        return self.state != old
