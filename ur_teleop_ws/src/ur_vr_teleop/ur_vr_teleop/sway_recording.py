"""Owned, read-only rosbag recording. No motion service or publisher exists here."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

TOPICS = (
    '/vr/controller/source_time', '/vr/controller/pose', '/vr/controller/joy',
    '/vr/controller/pose_fresh', '/vr/controller/optical_fresh', '/vr/controller/optical_status',
    '/vr_real/preview_markers', '/vr_real/preview_state', '/vr_real/operator_status',
    '/vr_real/servo_node/delta_twist_cmds', '/vr_real/servo_node/status',
    '/vr_real/servo_unstamped_targets', '/vr_real/servo_joint_targets', '/vr_real/armed',
    '/parameter_events', '/scaled_joint_trajectory_controller/joint_trajectory',
    '/scaled_joint_trajectory_controller/controller_state',
    '/scaled_joint_trajectory_controller/smoothed_command',
    '/scaled_joint_trajectory_controller/control_diagnostics', '/joint_states', '/tf', '/tf_static',
    '/speed_scaling_state_broadcaster/speed_scaling', '/io_and_status_controller/robot_mode',
    '/io_and_status_controller/safety_mode', '/io_and_status_controller/robot_program_running')
NODES = ('/vr_real/teleop', '/vr_real/servo_node', '/scaled_joint_trajectory_controller', '/controller_manager')


class Recorder:
    """Tk adapter: nonblocking start/stop/poll, and only signal its own process."""
    def __init__(self, root, config, snapshot=None):
        self.root, self.config, self.snapshot = Path(root), Path(config), snapshot
        self.process = self.log = None
        self.stopping = False
        self.message = 'Recording idle'

    def start(self):
        if self.process is not None:
            return False
        directory = self.root/'ur_teleop/diagnostics'
        directory.mkdir(parents=True, exist_ok=True)
        self.log_path = directory/('recorder_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.log')
        self.log = self.log_path.open('w')
        command = [str(self.root/'ur_teleop/scripts/record_sway.sh'), '--config', str(self.config)]
        if self.snapshot:
            command += ['--snapshot', str(self.snapshot)]
        try:
            self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                            start_new_session=True)
        except OSError:
            self.log.close(); self.log = None
            raise
        self.stopping = False
        self.log_offset = 0
        self.log_fragment = ''
        self.message = 'Starting recording; ' + str(self.log_path)
        return True

    def stop(self):
        if self.process is not None and self.process.poll() is None and not self.stopping:
            # The wrapper owns and finalizes its bag child; never signal unrelated ROS processes.
            self.process.send_signal(signal.SIGINT)
            self.stopping = True
            self.message = 'Stopping recording and finalizing bag…'

    def poll(self):
        if self.process is None:
            return self.message
        code = self.process.poll()
        if code is None:
            if not self.stopping and not self.message.startswith('RECORDING:'):
                try:
                    with self.log_path.open(errors='replace') as stream:
                        stream.seek(self.log_offset)
                        content = self.log_fragment + stream.read(8192)
                        self.log_offset = stream.tell()
                except OSError as error:
                    self.message = 'Recorder running; log unavailable: '+str(error)
                    return self.message
                lines = content.split('\n')
                self.log_fragment = lines.pop()[-8192:]
                for line in lines:
                    if line.startswith('RECORDING:'):
                        self.message = line
            return self.message
        self.log.close(); self.log = None; self.process = None
        self.message = ('Recording saved. ' if code == 0 else 'Recording FAILED; inspect ') + str(self.log_path)
        self.stopping = False
        return self.message

    def close(self):
        self.stop()
        if self.process is not None:
            # Called from a shutdown worker, not the Tk thread.
            self.process.wait(timeout=30)
            self.poll()


def parameter_dump(node):
    try:
        result = subprocess.run(['ros2', 'param', 'dump', node], text=True,
                                capture_output=True, timeout=5)
        return node, dict(returncode=result.returncode, yaml=result.stdout, error=result.stderr)
    except (OSError, subprocess.TimeoutExpired) as error:
        return node, dict(error=str(error))


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--snapshot', help='Managed effective_config.yaml, when available')
    options = parser.parse_args(args)
    directory = Path(options.output_root); directory.mkdir(parents=True, exist_ok=True)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    # Lock is shared by GUI and CLI and held until rosbag has finished writing.
    with (directory/'.sway_recording.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('Recording FAILED: another sway recording is active', flush=True)
            return 2
        output = directory/('sway_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        output.mkdir()
        metadata = {'clock': 'bag receive time (ROS wall time); preserve source stamps separately',
                    'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'), 'topics': TOPICS,
                    'started_wall_ns': time.time_ns(), 'configuration': {}}
        from .preferences import load_settings_with_sources
        try:
            settings, sources = load_settings_with_sources(options.snapshot or options.config)
            metadata['configuration'] = {'startup_or_selected_settings': settings, 'sources': sources,
                'note': 'Selected files are not runtime proof. Runtime dumps and operator_status are authoritative.'}
        except (OSError, ValueError, TypeError, KeyError) as error:
            metadata['configuration_error'] = str(error)
        bag = None
        try:
            # QoS overrides preserve best-effort driver samples and latched TF/state.
            qos = output/'qos.yaml'
            qos.write_text('/tf_static:\n  history: keep_last\n  depth: 1\n  durability: transient_local\n  reliability: reliable\n'
                           '/joint_states:\n  history: keep_last\n  depth: 100\n  reliability: best_effort\n')
            if stop.is_set():
                return 0
            bag = subprocess.Popen(['ros2', 'bag', 'record', '--qos-profile-overrides-path', str(qos),
                                    '-o', str(output/'bag'), *TOPICS], start_new_session=True)
            with ThreadPoolExecutor(max_workers=4) as pool:
                metadata['runtime_parameters_start'] = dict(pool.map(parameter_dump, NODES))
            if bag.poll() is not None:
                raise RuntimeError('rosbag exited during startup: '+str(bag.returncode))
            print('RECORDING: '+str(output), flush=True)
            while not stop.wait(.1):
                if bag.poll() is not None:
                    raise RuntimeError('rosbag exited unexpectedly: '+str(bag.returncode))
            with ThreadPoolExecutor(max_workers=4) as pool:
                metadata['runtime_parameters_stop'] = dict(pool.map(parameter_dump, NODES))
        except (OSError, RuntimeError) as error:
            metadata['error'] = str(error)
        finally:
            if bag is not None and bag.poll() is None:
                # Owned process group includes the ros2 CLI's recorder executable.
                os.killpg(bag.pid, signal.SIGINT)
                try:
                    bag.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(bag.pid, signal.SIGTERM)
                    try:
                        bag.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(bag.pid, signal.SIGKILL); bag.wait()
                    metadata['error'] = 'rosbag did not finalize on SIGINT; bag may be incomplete'
            metadata['bag_returncode'] = bag.returncode if bag else None
            if bag and bag.returncode not in (0, -signal.SIGINT):
                metadata.setdefault('error', 'rosbag exited with an error')
            metadata['stopped_wall_ns'] = time.time_ns()
            (output/'recording.json').write_text(json.dumps(metadata, indent=2, allow_nan=False))
        print(('Recording FAILED: ' + metadata['error'] if 'error' in metadata else 'Recording saved: '+str(output)), flush=True)
        return 1 if 'error' in metadata else 0


if __name__ == '__main__':
    raise SystemExit(main())
