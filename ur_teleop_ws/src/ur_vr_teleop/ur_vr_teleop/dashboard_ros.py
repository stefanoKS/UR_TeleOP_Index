"""Thread boundary between the ROS executor and Tk dashboard."""
import threading
import time
from collections import deque

import rclpy
from rclpy.executors import SingleThreadedExecutor


_STATUS_EVENT_FIELDS = (
    'session_id', 'mode', 'armed', 'enable_requested', 'robot_fault',
    'configuration_ready', 'configuration_reason', 'controller_ready',
    'servo_status_fresh', 'servo_state', 'motion_ready', 'tracking_valid',
    'observations_healthy', 'operator_heartbeat_fresh', 'reason',
)


class RosDataBridge:
    """Latest telemetry plus ordered status transitions for the Tk thread."""
    def __init__(self):
        self._lock = threading.Lock()
        self._latest = {}
        self._events = deque()
        self._sequence = 0
        self._status_signature = None

    def put_latest(self, kind, value, received_at=None):
        received_at = time.monotonic() if received_at is None else received_at
        with self._lock:
            self._sequence += 1
            self._latest[kind] = (self._sequence, kind, value, received_at)

    def put_status(self, value, received_at=None):
        received_at = time.monotonic() if received_at is None else received_at
        blockers = value.get('blockers')
        signature = tuple(value.get(field) for field in _STATUS_EVENT_FIELDS) + (
            tuple(blockers) if isinstance(blockers, list) else None,
        )
        with self._lock:
            self._sequence += 1
            item = (self._sequence, 'status', value, received_at)
            self._latest['status'] = item
            if signature != self._status_signature:
                self._events.append(item)
                self._status_signature = signature

    def drain(self):
        with self._lock:
            items = list(self._events)
            self._events.clear()
            items.extend(self._latest.values())
            self._latest.clear()
        return sorted({item[0]: item for item in items}.values())

    def peek(self, kind):
        """Latest value without consuming it (safe for bounded close waits)."""
        with self._lock:
            item = self._latest.get(kind)
            return (item[2], item[3]) if item is not None else (None, -float('inf'))


class LatestValue:
    """Bounded one-value mailbox for high-rate optional telemetry."""
    def __init__(self):
        self._lock = threading.Lock()
        self._value = None

    def put(self, value):
        with self._lock:
            self._value = value

    def get(self):
        with self._lock:
            return self._value

    def clear(self):
        with self._lock:
            self._value = None


class RosExecutorThread:
    """Continuously service one GUI node independently of Tk rendering."""
    def __init__(self, node):
        self.node = node
        self.closing = threading.Event()
        self.ready = threading.Event()
        self.failure = None
        self.executor = None
        self.thread = threading.Thread(target=self._run, name='teleop-gui-ros', daemon=False)
        self.thread.start()
        self.ready.wait(2.)
        if self.failure is not None:
            raise RuntimeError('GUI ROS executor failed to start') from self.failure
        if not self.ready.is_set():
            raise RuntimeError('GUI ROS executor did not start')

    def _run(self):
        executor = SingleThreadedExecutor(context=self.node.context)
        self.executor = executor
        try:
            executor.add_node(self.node)
            self.ready.set()
            while not self.closing.is_set() and rclpy.ok(context=self.node.context):
                executor.spin_once(timeout_sec=.02)
        except Exception as error:
            self.failure = error
            self.ready.set()
        finally:
            executor.shutdown(timeout_sec=1.)
            executor.remove_node(self.node)

    def close(self):
        self.closing.set()
        if self.executor is not None:
            self.executor.wake()

    def join(self, timeout=2.):
        self.thread.join(timeout)
        return not self.thread.is_alive()
