"""Bounded UR Dashboard access for graceful program shutdown.

ROS imports are intentionally lazy so pure state-machine tests do not require a
ROS environment.  Every call has an explicit deadline and every answer field is
checked; a successful transport call is never treated as program state.
"""
import os
import time

from .program_control import ProgramStopResult, ShutdownCoordinator


def _spin_call(node, client, request, deadline):
    """Run one service request without exceeding the caller's deadline."""
    import rclpy
    try:
        future = client.call_async(request)
    except Exception as error:
        return None, str(error)
    while not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            future.cancel()
            return None, 'service timeout'
        try:
            rclpy.spin_until_future_complete(node, future, timeout_sec=min(.05, remaining))
        except Exception as error:
            return None, str(error)
    try:
        return future.result(), None
    except Exception as error:
        return None, str(error)


def stop_program(timeout=2.0):
    """Request Dashboard Stop only while the services are still alive.

    Returns a confirmed result only when ``program_running`` readback proves the
    program is not running.  A Stop acknowledgment by itself is not proof.
    """
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger
    from ur_dashboard_msgs.srv import IsProgramRunning

    if not rclpy.ok():
        return ProgramStopResult('unavailable',
                                 'ROS context is not running; graceful Dashboard stop could not be requested')
    deadline = time.monotonic() + max(.1, float(timeout))
    node = None
    try:
        node = Node(f'teleop_program_shutdown_{os.getpid()}')
        running_client = node.create_client(IsProgramRunning, '/dashboard_client/program_running')
        stop_client = node.create_client(Trigger, '/dashboard_client/stop')
        wait = min(max(0., deadline - time.monotonic()), .5)
        if not running_client.wait_for_service(timeout_sec=wait):
            return ProgramStopResult(
                'unavailable',
                'Dashboard program-running service is unavailable; graceful program shutdown '
                'could not be confirmed. A pendant socket-disconnect popup may still occur.')
        result, error = _spin_call(node, running_client, IsProgramRunning.Request(), deadline)
        if result is None or not result.success:
            return ProgramStopResult(
                'unavailable',
                'Dashboard could not confirm program state' + (f' ({error})' if error else '')
                + '; graceful program shutdown could not be confirmed.')
        if not result.program_running:
            return ProgramStopResult(
                'already_stopped',
                'Dashboard confirmed no robot program was running; no Stop was requested.')
        wait = min(max(0., deadline - time.monotonic()), .5)
        if not stop_client.wait_for_service(timeout_sec=wait):
            return ProgramStopResult(
                'unavailable',
                'Program is running but the Dashboard stop service is unavailable; '
                'graceful shutdown could not be requested.')
        stop_result, error = _spin_call(node, stop_client, Trigger.Request(), deadline)
        if stop_result is None or not stop_result.success:
            detail = error or getattr(stop_result, 'message', '') or 'Dashboard rejected the stop request'
            return ProgramStopResult('failed', f'Dashboard Stop failed: {detail}')
        # Require authoritative readback, not the Stop acknowledgment.
        while time.monotonic() < deadline:
            result, error = _spin_call(node, running_client, IsProgramRunning.Request(), deadline)
            if result is not None and result.success and not result.program_running:
                return ProgramStopResult(
                    'stopped', 'Dashboard Stop confirmed; no robot program is running.')
            time.sleep(.1)
        return ProgramStopResult(
            'timeout',
            'Dashboard Stop was acknowledged but program-running readback did not confirm '
            'shutdown before the timeout. A pendant socket-disconnect popup may still occur.')
    except Exception as error:
        return ProgramStopResult('unknown', f'Graceful Dashboard stop failed: {error}')
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass


def coordinated_stop(signal_processes, timeout=2.0):
    """Run the bounded program stop before signaling owned processes."""
    coordinator = ShutdownCoordinator(lambda: stop_program(timeout), signal_processes)
    return coordinator.run()
