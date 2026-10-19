"""Single-window launcher. Owns only its child process groups; never resets UR faults."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET
import yaml
from .process_ownership import SESSION_KEY, identity, is_same, tagged_processes, send_signal


def tuned_description(description_share, destination, gain, lookahead, tcp_offset=None):
    """Local copy of three installed xacros; leave /opt and robot geometry intact."""
    if type(gain) is not int or not 100 <= gain <= 2000:
        raise ValueError('servoj_gain must be an integer from 100 to 2000')
    if not math.isfinite(lookahead) or not .03 <= lookahead <= .2:
        raise ValueError('servoj_lookahead_time must be between .03 and .2 s')
    source, destination = Path(description_share)/'urdf', Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    replacements = {
        'ur.urdf.xacro': ('$(find ur_description)/urdf/ur_macro.xacro', str(destination/'ur_macro.xacro')),
        'ur_macro.xacro': ('$(find ur_description)/urdf/ur.ros2_control.xacro', str(destination/'ur.ros2_control.xacro')),
    }
    for name, (old, new) in replacements.items():
        content = (source/name).read_text()
        if content.count(old) != 1:
            raise RuntimeError('Installed UR xacro layout changed: '+name)
        (destination/name).write_text(content.replace(old, new))
    content = (source/'ur.ros2_control.xacro').read_text()
    import re
    for key, value in [('servoj_gain', gain), ('servoj_lookahead_time', lookahead)]:
        content, count = re.subn(r'(<param name="'+key+r'">)[^<]*(</param>)',
                                lambda m: m[1]+str(value)+m[2], content)
        if count != 1:
            raise RuntimeError('Cannot configure UR hardware parameter '+key)
    ET.fromstring(content)
    (destination/'ur.ros2_control.xacro').write_text(content)
    if tcp_offset is not None:
        from .tool_tcp import tool_xml
        path = destination/'ur.urdf.xacro'
        path.write_text(path.read_text().replace('</robot>', tool_xml(tcp_offset)+'</robot>'))
    return destination/'ur.urdf.xacro'


def foreign_processes():
    """Detect local competing stacks without ever signalling them."""
    found = []
    binaries = {'controller_bridge', 'servo_node_main', 'ur_ros2_control_node', 'commissioning', 'servo_stamp_relay'}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            args = (entry/'cmdline').read_bytes().decode(errors='replace').split('\0')
            # Executable may be a Python console script (second argv element).
            if any(Path(a).name in binaries for a in args[:2]):
                found.append((int(entry.name), ' '.join(args[:3])))
            elif any(Path(a).name == 'gui' and '/ur_vr_teleop/' in a for a in args[:2]):
                found.append((int(entry.name), ' '.join(args[:3])))
            elif len(args) > 4 and args[1].endswith('/ros2') and args[2:4] == ['launch', 'ur_robot_driver']:
                found.append((int(entry.name), ' '.join(args[1:5])))
        except (OSError, ProcessLookupError):
            pass
    return found


def tuned_controllers(source, destination, settings):
    data = yaml.safe_load(Path(source).read_text())
    data['controller_manager']['ros__parameters']['scaled_joint_trajectory_controller']['type'] = (
        'ur_teleop_controllers/SmoothScaledJointTrajectoryController')
    parameters = data['scaled_joint_trajectory_controller']['ros__parameters']
    limits = {'smoothing.max_velocity': settings['teleop']['max_joint_speed_rad_s'],
              'smoothing.max_acceleration': settings['robot'].get('joint_command_acceleration_rad_s2', 1.),
              'smoothing.max_jerk': settings['robot'].get('joint_command_jerk_rad_s3', 10.)}
    if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or v <= 0 for v in limits.values()):
        raise ValueError('Final joint command limits must be finite positive numbers')
    parameters.update({k: float(v) for k, v in limits.items()})
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    # PyYAML removes the quotes around ROS substitutions such as:
    #   tf_prefix: "$(var tf_prefix)"
    #
    # An empty tf_prefix would then become:
    #   tf_prefix:
    #
    # which ROS interprets as null instead of an empty string.
    text = yaml.safe_dump(data, sort_keys=False)

    text = text.replace(
        'tf_prefix: $(var tf_prefix)',
        'tf_prefix: "$(var tf_prefix)"'
    )

    # Defensive check: never generate an unquoted tf_prefix substitution.
    if 'tf_prefix: $(var tf_prefix)' in text:
        raise RuntimeError(
            'Generated controller YAML contains an unquoted tf_prefix substitution'
        )

    destination.write_text(text)
    validate_controller_file(destination)
    return destination


def validate_controller_file(path):
    """Check the exact post-substitution file with ROS, before launching children."""
    from launch import LaunchContext
    from launch_ros.parameter_descriptions import ParameterFile
    from rclpy.utilities import remove_ros_args
    context = LaunchContext()
    context.launch_configurations.update(tf_prefix='', update_rate='500')
    parameters = ParameterFile(str(path), allow_substs=True)
    try:
        resolved = parameters.evaluate(context)
        remove_ros_args(['validate', '--ros-args', '--params-file', str(resolved)])
    finally:
        parameters.cleanup()

class ProcessManager:
    def __init__(self, config, root=None, conflict_check=foreign_processes, program_stop=None):
        self.config = Path(config)
        self.root = Path(root or Path.home()/'Documents/Teleop/ur_teleop')
        self.conflict_check = conflict_check
        self.program_stop = program_stop
        self.children = {}
        self.log_files = {}
        self.logs = []
        self.stopping = False
        self.stop_scope = None
        self.program_stop_result = None
        self.restart_mode = None
        self.live = False
        self.message = 'Stopped'
        self.directory = None
        self.stop_started = 0.
        self.escalation = 0
        self.token = None
        self.owned = []
        self.last_scan = -float('inf')
        self.manifest = self.root/'.managed_session.json'
        self.environment = None
        self.log_offsets = {}
        self.log_fragments = {}
        self.last_log_check = -float('inf')
        self.crash_counts = {}

    def commands(self, live):
        from ament_index_python.packages import get_package_share_directory
        from .preferences import write_effective_snapshot
        settings, snapshot = write_effective_snapshot(self.config, self.directory)
        robot = settings['robot']
        generated = tuned_description(get_package_share_directory('ur_description'),
            self.directory/'description', robot.get('servoj_gain', 500),
            robot.get('servoj_lookahead_time', .08), robot.get('tcp_offset'))
        controller_config = tuned_controllers(self.root/'config/commissioning_controllers.yaml',
                                              self.directory/'controllers.yaml', settings)
        driver = ['ros2', 'launch', 'ur_robot_driver', 'ur_control.launch.py', 'ur_type:=ur10e',
            'robot_ip:='+robot['ip'], 'kinematics_params_file:='+robot['calibration_file'],
            'controllers_file:='+str(controller_config),
            'description_file:='+str(generated), 'script_sender_port:=50002', 'headless_mode:=false',
            'initial_joint_controller:=scaled_joint_trajectory_controller',
            'activate_joint_controller:=false', 'launch_rviz:=false']
        commands = {
            'driver': driver,
            'tracking': ['ros2', 'run', 'ur_vr_bridge', 'controller_bridge', '--ros-args',
                         '-p', 'device_serial:=LHR-4632363E'],
            'teleop': ['ros2', 'launch', 'ur_vr_teleop', 'commissioning.launch.py',
                        'config_file:='+str(snapshot), 'motion_output_enabled:='+str(live).lower(),
                        'launch_rviz:=false', 'engineering_session:='+str(getattr(self, 'engineering_session', False)).lower()],
            'rviz': self.rviz_command(),
        }
        if settings.get('gripper', {}).get('process_enabled', True):
            commands['gripper'] = ['ros2', 'run', 'ur_bus_servo_gripper', 'gripper', '--ros-args',
                '-p', 'config_file:='+str(self.config.parent/'gripper.json')]
        camera = settings.get('camera', {})
        if camera.get('enabled', True):
            commands['camera'] = ['ros2', 'run', 'ur_d405_camera', 'd405_camera', '--ros-args',
                '-p', "serial:='" + str(camera.get('serial', '230322276078')) + "'"]
        return commands

    def rviz_command(self):
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        return [str(Path(get_package_prefix('rviz2'))/'lib/rviz2/rviz2'), '-d',
                str(Path(get_package_share_directory('ur_vr_teleop'))/'rviz/commissioning.rviz'),
                '--ros-args', '-r', '__node:=teleop_rviz']

    def scan_owned(self, force=False):
        now = time.monotonic()
        if force or now-self.last_scan >= .25:
            self.owned = tagged_processes(self.token)
            self.last_scan = now
        return [p for p in self.owned if is_same(p)]

    def recover_abandoned(self, live):
        if not self.manifest.exists():
            return False
        saved = json.loads(self.manifest.read_text())
        if is_same(saved.get('owner')) and saved['owner'] != identity(os.getpid()):
            raise RuntimeError('The recorded launcher is still running; use its window.')
        self.token = saved['token']
        self.directory = Path(saved['directory'])
        if self.scan_owned(force=True):
            self.message = 'Cleaning up the abandoned launcher session, including RViz; then starting disabled.'
            self.stop(restart=bool(live))
            return True
        self.manifest.unlink(missing_ok=True)
        self.token = None
        return False

    def clear_manifest(self):
        if self.token and self.manifest.exists():
            saved = json.loads(self.manifest.read_text())
            if saved.get('token') == self.token:
                self.manifest.unlink(missing_ok=True)

    def start(self, live=False):
        if self.stopping:
            raise RuntimeError('Stop the managed stack before starting another')
        if self.children:
            # Retained shared/peripheral processes (tracking, gripper) must not
            # be duplicated.  Perform one reliable full stop/start instead.
            self.message = ('Retained shared processes present; stopping the full owned stack '
                            'before a fresh restart.')
            self.stop(restart=bool(live))
            return
        if self.recover_abandoned(live):
            return
        conflicts = self.conflict_check()
        if conflicts:
            raise RuntimeError('Existing ROS stack detected. Stop its original terminals first; nothing was killed. PIDs: '
                               + ', '.join(str(pid) for pid, _ in conflicts))
        self.live = bool(live)
        self.token = str(uuid.uuid4())
        self.directory = self.root/'logs'/(time.strftime('%Y%m%d_%H%M%S')+'_'+self.token[:8])
        self.directory.mkdir(parents=True, exist_ok=True)
        self.log_offsets.clear()
        self.log_fragments.clear()
        commands = self.commands(live)
        (self.directory/'commands.json').write_text(json.dumps(commands, indent=2))
        environment = os.environ.copy()
        environment['ROS_DOMAIN_ID'] = '43'
        environment['ROS_LOG_DIR'] = str(self.directory/'ros')
        environment['PYTHONUNBUFFERED'] = '1'
        environment[SESSION_KEY] = self.token
        self.environment = environment
        temporary = self.manifest.with_suffix('.tmp')
        temporary.write_text(json.dumps({'owner': identity(os.getpid()), 'token': self.token,
                                         'directory': str(self.directory)}))
        temporary.chmod(0o600)
        temporary.replace(self.manifest)
        try:
            for name, command in commands.items():
                self.spawn(name, command)
        except Exception:
            self.stop()
            raise
        self.message = ('LIVE' if live else 'PREVIEW')+' stack started, motion disabled. Logs: '+str(self.directory)

    def spawn(self, name, command):
        if hasattr(self, 'optional_failed'):
            self.optional_failed.discard(name)
        log = (self.directory/(name+'.log')).open('a')
        self.logs.append(log)
        self.log_files[name] = log
        wrapper = [sys.executable, '-m', 'ur_vr_teleop.process_ownership', str(os.getpid())]
        self.children[name] = subprocess.Popen(wrapper+command, stdout=log, stderr=subprocess.STDOUT,
            env=self.environment, start_new_session=True)

    def open_rviz(self):
        if not self.children or self.stopping:
            raise RuntimeError('Start Preview or Live setup before opening RViz.')
        current = self.children.get('rviz')
        if current is not None and current.poll() is None:
            self.message = 'RViz is already running; select its window in the taskbar.'
            return
        self.spawn('rviz', self.rviz_command())
        self.message = 'RViz opened with the commissioning marker display.'

    def stop(self, restart=None, scope=None, program_timeout=2.0):
        """Stop all owned groups, or only ``scope`` while retaining the rest.

        For any stop that includes the driver, the confirmed Dashboard program
        stop happens before the driver receives a signal.  Its bounded failure
        is reported but never blocks the existing fail-safe termination.
        """
        self.restart_mode = restart
        if self.stopping:
            return
        self.stop_scope = set(scope) if scope else None
        self.stopping, self.stop_started, self.escalation = True, time.monotonic(), 0
        self.program_stop_result = self.graceful_program_stop(program_timeout)
        self.signal_groups(signal.SIGINT, self.stop_scope)

    def driver_alive(self):
        driver = self.children.get('driver')
        return driver is not None and driver.poll() is None

    def graceful_program_stop(self, timeout):
        from .program_control import ProgramStopResult
        if not self.driver_alive():
            return ProgramStopResult(
                'unavailable',
                'UR driver is not running; no Dashboard program stop was possible.')
        try:
            if self.program_stop is not None:
                return self.program_stop(timeout)
            from .dashboard_services import stop_program
            return stop_program(timeout)
        except Exception as error:
            return ProgramStopResult('unknown', str(error))

    def signal_groups(self, sig, scope=None):
        if scope is None:
            # Descendants may leave their original process group. Match inherited
            # ownership tags plus immutable process identities, never executable names.
            for saved in self.scan_owned(force=True):
                send_signal(saved, sig)
            for process in self.children.values():
                if process.poll() is None:
                    try:
                        os.killpg(process.pid, sig)
                    except (ProcessLookupError, PermissionError):
                        pass
            return
        # Partial stop is deliberately limited to named launch groups.  Optional
        # peripherals (gripper/camera) keep their serial/process ownership.
        for name in scope:
            process = self.children.get(name)
            if process is not None and process.poll() is None:
                try:
                    os.killpg(process.pid, sig)
                except (ProcessLookupError, PermissionError):
                    pass

    def groups_alive(self, scope=None):
        if scope is None:
            for process in self.children.values():
                process.poll()  # reap leaders
            return bool(self.scan_owned(force=True))
        alive = False
        for name in scope:
            process = self.children.get(name)
            if process is None:
                continue
            if process.poll() is None:
                alive = True
                continue
            try:
                os.killpg(process.pid, 0)
                alive = True
            except (ProcessLookupError, PermissionError):
                pass
        return alive

    def child_log_failure(self):
        # ros2 launch may survive a critical child crash. Read only new log
        # bytes, and only match launch's own fatal process/launch reports.
        now = time.monotonic()
        if self.directory is None or now-self.last_log_check < .25:
            return None
        self.last_log_check = now
        for name in ('driver', 'teleop', 'tracking'):
            path = self.directory/(name+'.log')
            if not path.exists():
                continue
            with path.open('rb') as stream:
                stream.seek(self.log_offsets.get(name, 0))
                chunk = stream.read(65536)
                self.log_offsets[name] = stream.tell()
            text = self.log_fragments.get(name, '') + chunk.decode('utf-8', errors='replace')
            lines = text.split('\n')
            self.log_fragments[name] = lines.pop()[-4096:]
            for line in lines:
                if ('[ERROR]' in line and ('process has died' in line or 'Caught exception in launch' in line)):
                    return name + ': ' + line[-600:] + ' (log: ' + str(path) + ')'
        return None

    def record_crash(self, names):
        # Expose the process name, exit reason and log path; flag repeated failures.
        for name in names:
            self.crash_counts[name] = self.crash_counts.get(name, 0) + 1
        repeats = ', '.join(f'{name} x{count}' for name, count in self.crash_counts.items() if count > 1)
        return ' Repeated crashes: ' + repeats + '.' if repeats else ''

    def poll(self):
        self.scan_owned()
        if not self.stopping:
            failed = [name for name, p in self.children.items() if name not in ('rviz', 'camera', 'gripper') and p.poll() is not None]
            if failed:
                reasons = ', '.join(f'{name}(exit {self.children[name].returncode})' for name in failed)
                scope = {'driver', 'teleop', 'rviz'}
                tracking_failed = 'tracking' in failed
                if tracking_failed:
                    scope.add('tracking')
                detail = ('Tracking also stopped; gripper retained for read-only diagnostics.'
                          if tracking_failed else
                          'Tracking and gripper retained; verify fresh GRIPPER status in the dashboard.')
                self.message = ('ROBOT FAULT — ' + detail + ' Process exited: ' + reasons
                                + '. Logs: ' + str(self.directory)
                                + '.' + self.record_crash(failed) + ' Inspect the failing component, then use Restart ROS.')
                # The arm subsystem is stopped.  The shared tracking bridge is
                # preserved on a driver/teleop fault so independent gripper
                # trigger following can continue with valid tracking.
                self.stop(scope=scope, program_timeout=1.0)
            else:
                failure = self.child_log_failure()
                if failure:
                    failed_name = failure.split(':', 1)[0]
                    scope = {'driver', 'teleop', 'rviz'}
                    if failed_name == 'tracking':
                        scope.add('tracking')
                    self.message = ('ROS child failed; arm stack stopped, gripper retained. ' + failure
                                    + '.' + self.record_crash([failed_name]))
                    self.stop(scope=scope, program_timeout=1.0)
                else:
                    for name in ('gripper', 'camera', 'rviz'):
                        process = self.children.get(name)
                        if (process is not None and process.poll() is not None
                                and name not in getattr(self, 'optional_failed', set())):
                            self.optional_failed = getattr(self, 'optional_failed', set()) | {name}
                            if name == 'gripper':
                                self.message = ('Gripper process exited; arm teleoperation is unaffected. '
                                                'Inspect the gripper log, then use Restart ROS to reconnect.')
                            else:
                                self.message = f'{name} process exited; arm operation is unaffected.'
        if self.stopping:
            scope = self.stop_scope
            age = time.monotonic()-self.stop_started
            if age > 3 and self.escalation == 0:
                self.signal_groups(signal.SIGTERM, scope)
                self.escalation = 1
            if age > 5 and self.escalation == 1:
                self.signal_groups(signal.SIGKILL, scope)
                self.escalation = 2
            if not self.groups_alive(scope):
                if scope is None:
                    self.children.clear()
                    for log in self.logs:
                        log.close()
                    self.logs.clear()
                    self.log_files.clear()
                    self.clear_manifest()
                    self.token = None
                    restart, self.restart_mode = self.restart_mode, None
                    self.stopping = self.stop_scope = None
                    self.escalation = 0
                    if restart is not None:
                        try:
                            self.start(restart)
                        except Exception as error:
                            self.message = 'Restart failed: '+str(error)
                else:
                    for name in scope:
                        self.children.pop(name, None)
                        log = self.log_files.pop(name, None)
                        if log is not None:
                            try:
                                log.close()
                            except OSError:
                                pass
                            if log in self.logs:
                                self.logs.remove(log)
                    self.stopping = self.stop_scope = None
                    self.escalation = 0
                    if not self.children:
                        self.clear_manifest()
                        self.token = None

    def close(self):
        self.stop()
        until = time.monotonic()+7
        while self.stopping and time.monotonic() < until:
            self.poll()
            time.sleep(.05)
        # A close that arrived during a partial arm stop must still terminate the
        # retained gripper (and release its serial device).
        if self.children and not self.stopping:
            self.stop()
            until = time.monotonic()+7
            while self.stopping and time.monotonic() < until:
                self.poll()
                time.sleep(.05)
        if not self.stopping and not self.children and self.token:
            self.clear_manifest()
            self.token = None
        self.signal_groups(signal.SIGKILL)


def parse_options(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--live', dest='live', action='store_true', default=True, help='Live stack (default), still disarmed/inactive')
    mode.add_argument('--preview', dest='live', action='store_false', help='Engineering Preview, no final motion publisher')
    parser.add_argument('--engineering-session', action='store_true', help='Deprecated compatibility option; configuration and runtime protections always apply')
    parser.add_argument('--config', default=str(Path.home()/'Documents/Teleop/ur_teleop/config/real_teleop.yaml'))
    return parser.parse_args(args)


def main(args=None):
    options = parse_options(args)
    root = Path.home()/'Documents/Teleop/ur_teleop'
    lock = (root/'.desktop.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        from tkinter import messagebox
        messagebox.showerror('UR teleop', 'The teleop launcher is already running.')
        return
    import rclpy
    from .gui import OperatorGUI
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    manager = ProcessManager(options.config)
    manager.engineering_session = options.engineering_session
    gui = None
    try:
        gui = OperatorGUI(manager=manager)
        # Do not let terminal closure skip the manager's finally/cleanup path.
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, lambda *_: setattr(gui, 'shutdown_requested', True))
        gui.root.after(200, lambda: gui.stack_action('start', options.live))
        gui.root.mainloop()
    except KeyboardInterrupt:
        if gui:
            gui.close()
    finally:
        if gui and gui.process_worker:
            gui.process_worker.close()
            gui.process_worker.join()
        else:
            manager.close()
        if gui:
            gui.node.destroy_node()
        rclpy.shutdown()
        lock.close()
