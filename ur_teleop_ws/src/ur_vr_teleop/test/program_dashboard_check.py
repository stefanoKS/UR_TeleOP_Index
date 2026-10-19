"""Mocked Dashboard services plus the pure program workflow; no robot is used.

Run explicitly with an isolated domain:
    ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=147 /usr/bin/python3 \
      ur_teleop_ws/src/ur_vr_teleop/test/program_dashboard_check.py

The fake server implements the installed Humble interface fields.  It never
launches the UR driver, plays a real program or commands motion.
"""
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import (GetLoadedProgram, GetRobotMode, GetSafetyMode,
                                   IsInRemoteControl, IsProgramRunning, Load)

from ur_vr_teleop.dashboard_services import stop_program
from ur_vr_teleop.program_control import ProgramConfig, ProgramStartup


class FakeDashboard(Node):
    def __init__(self):
        super().__init__('fake_dashboard_services')
        self.remote = True
        self.safety = 1
        self.mode = 7
        self.loaded = 'external_control.urp'
        self.running = False
        self.play_ack_running = True
        self.stop_ack = True
        self.play_calls = 0
        self.stop_calls = 0
        self.create_service(IsInRemoteControl, '/dashboard_client/is_in_remote_control', self.on_remote)
        self.create_service(GetSafetyMode, '/dashboard_client/get_safety_mode', self.on_safety)
        self.create_service(GetRobotMode, '/dashboard_client/get_robot_mode', self.on_mode)
        self.create_service(GetLoadedProgram, '/dashboard_client/get_loaded_program', self.on_loaded)
        self.create_service(IsProgramRunning, '/dashboard_client/program_running', self.on_running)
        self.create_service(Load, '/dashboard_client/load_program', self.on_load)
        self.create_service(Trigger, '/dashboard_client/play', self.on_play)
        self.create_service(Trigger, '/dashboard_client/stop', self.on_stop)

    def on_remote(self, request, response):
        response.success = True
        response.remote_control = self.remote
        response.answer = 'remote' if self.remote else 'local'
        return response

    def on_safety(self, request, response):
        response.success = True
        response.safety_mode.mode = self.safety
        response.answer = 'safety'
        return response

    def on_mode(self, request, response):
        response.success = True
        response.robot_mode.mode = self.mode
        response.answer = 'mode'
        return response

    def on_loaded(self, request, response):
        response.success = True
        response.program_name = self.loaded
        response.answer = 'loaded'
        return response

    def on_running(self, request, response):
        response.success = True
        response.program_running = self.running
        response.answer = 'running'
        return response

    def on_load(self, request, response):
        self.loaded = request.filename
        response.success = True
        response.answer = 'loaded ' + request.filename
        return response

    def on_play(self, request, response):
        self.play_calls += 1
        self.running = bool(self.play_ack_running)
        response.success = True
        response.message = 'playing'
        return response

    def on_stop(self, request, response):
        self.stop_calls += 1
        if self.stop_ack:
            self.running = False
        response.success = self.stop_ack
        response.message = 'stopped' if self.stop_ack else 'rejected'
        return response


def call(node, client, request, timeout=2.0):
    future = client.call_async(request)
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(.01)
    if not future.done():
        future.cancel()
        raise TimeoutError('mock dashboard service timeout')
    return future.result()


def clients(node):
    return {
        'remote': node.create_client(IsInRemoteControl, '/dashboard_client/is_in_remote_control'),
        'safety': node.create_client(GetSafetyMode, '/dashboard_client/get_safety_mode'),
        'mode': node.create_client(GetRobotMode, '/dashboard_client/get_robot_mode'),
        'loaded': node.create_client(GetLoadedProgram, '/dashboard_client/get_loaded_program'),
        'running': node.create_client(IsProgramRunning, '/dashboard_client/program_running'),
        'load': node.create_client(Load, '/dashboard_client/load_program'),
        'play': node.create_client(Trigger, '/dashboard_client/play'),
    }


def drive(workflow, node, service_clients, driver_ready):
    """Run the pure workflow against the mocked service answers."""
    while True:
        observations = dict(dashboard_ready=all(c.service_is_ready() for c in service_clients.values()),
                            driver_ready=driver_ready, driver_status_fresh=True, driver_fault=False,
                            a_held=False, armed=False, enable_requested=False)
        action = workflow.step(observations, time.monotonic())
        if action:
            if action == 'remote':
                result = call(node, service_clients[action], IsInRemoteControl.Request())
                ok, value = result.success, result.remote_control
            elif action == 'safety':
                result = call(node, service_clients[action], GetSafetyMode.Request())
                ok, value = result.success, result.safety_mode.mode
            elif action == 'mode':
                result = call(node, service_clients[action], GetRobotMode.Request())
                ok, value = result.success, result.robot_mode.mode
            elif action == 'loaded':
                result = call(node, service_clients[action], GetLoadedProgram.Request())
                ok, value = result.success, result.program_name
            elif action == 'running':
                result = call(node, service_clients[action], IsProgramRunning.Request())
                ok, value = result.success, result.program_running
            elif action == 'load':
                result = call(node, service_clients[action], Load.Request(
                    filename=workflow.config.program))
                ok, value = result.success, True
            else:
                result = call(node, service_clients[action], Trigger.Request())
                ok, value = result.success, True
            workflow.acknowledge(action, ok, 'mock', time.monotonic(), value)
            if workflow.state == 'READY':
                return 'READY'
            if workflow.state == 'FAILED':
                return 'FAILED'
            continue
        if workflow.state in ('READY', 'FAILED'):
            return 'READY' if workflow.state == 'READY' else 'FAILED'
        time.sleep(.02)


def main():
    rclpy.init(domain_id=147)
    server = FakeDashboard()
    node = Node('program_dashboard_check')
    service_clients = clients(node)
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(node)
    closing = threading.Event()
    def spin():
        while not closing.is_set():
            executor.spin_once(timeout_sec=.02)
    thread = threading.Thread(target=spin, name='mock-dashboard-executor', daemon=True)
    thread.start()
    try:
        for client in service_clients.values():
            assert client.wait_for_service(timeout_sec=2.), 'mock dashboard service missing'
        config = ProgramConfig(enabled=True, program='external_control.urp',
                               timeouts={'dashboard': 1., 'load': 1., 'play': 1.,
                                         'running_wait': 1., 'driver': 1.})
        # Remote mode + correct loaded program -> Play.
        server.remote, server.loaded, server.running = True, 'external_control.urp', False
        workflow = ProgramStartup(config); assert workflow.start(0)
        assert drive(workflow, node, service_clients, True) == 'READY'
        assert server.play_calls == 1, server.play_calls

        # Wrong loaded program -> Load, verify identity, then Play.
        server.play_calls = 0; server.loaded = 'old.urp'; server.running = False
        workflow = ProgramStartup(config); assert workflow.start(0)
        assert drive(workflow, node, service_clients, True) == 'READY'
        assert server.loaded == 'external_control.urp'
        assert server.play_calls == 1

        # Already running -> no redundant Play.
        server.play_calls = 0; server.loaded = 'external_control.urp'; server.running = True
        workflow = ProgramStartup(config); assert workflow.start(0)
        assert drive(workflow, node, service_clients, True) == 'READY'
        assert server.play_calls == 0

        # Local Control -> refusal, no Play.
        server.play_calls = 0; server.remote = False; server.running = False
        workflow = ProgramStartup(config); assert workflow.start(0)
        assert drive(workflow, node, service_clients, True) == 'FAILED'
        assert 'Local Control' in workflow.message
        assert server.play_calls == 0

        # Protective stop -> no Play.
        server.remote = True; server.safety = 3; server.play_calls = 0
        workflow = ProgramStartup(config); assert workflow.start(0)
        assert drive(workflow, node, service_clients, True) == 'FAILED'
        assert server.play_calls == 0

        # Play acknowledgment without running readback is a bounded failure.
        server.safety = 1; server.play_ack_running = False; server.play_calls = 0
        short = ProgramConfig(enabled=True, program='external_control.urp',
                              timeouts={'dashboard': 1., 'play': 1., 'running_wait': .5, 'driver': 1.})
        workflow = ProgramStartup(short); assert workflow.start(0)
        assert drive(workflow, node, service_clients, True) == 'FAILED'
        assert 'acknowledged' in workflow.message.lower()

        # Graceful stop requires readback confirmation, not just an acknowledgment.
        server.running = True; server.stop_ack = True; server.stop_calls = 0
        result = stop_program(timeout=2.)
        assert result.confirmed and result.status == 'stopped', result
        assert server.stop_calls == 1 and not server.running
        result = stop_program(timeout=1.)
        assert result.confirmed and result.status == 'already_stopped', result
        assert server.stop_calls == 1

        # A Stop acknowledgment that does not change state is a timeout, not success.
        server.running = True; server.stop_ack = False
        result = stop_program(timeout=.8)
        assert not result.confirmed and result.status == 'failed', result
        print('PASS: mocked Dashboard start/refusal/readback and confirmed shutdown ordering')
    finally:
        closing.set()
        thread.join(timeout=1.)
        executor.shutdown(timeout_sec=1.)
        node.destroy_node()
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
