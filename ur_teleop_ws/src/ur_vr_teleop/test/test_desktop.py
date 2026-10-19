import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET
import pytest
import json
import signal
import subprocess
from ur_vr_teleop.desktop import ProcessManager, tuned_description
from ur_vr_teleop.process_ownership import identity, is_same, send_signal, SESSION_KEY


def pump(manager, condition, timeout=8):
    deadline = time.monotonic()+timeout
    while not condition() and time.monotonic() < deadline:
        manager.poll()
        time.sleep(.02)
    assert condition(), manager.message


class ToyManager(ProcessManager):
    """Only local sleeping Python children; no ROS or network access."""
    def commands(self, live):
        return {name: [sys.executable, '-c', 'import time; time.sleep(60)'] for name in ('driver', 'tracking', 'teleop')}

    def rviz_command(self):
        return [sys.executable, '-c', 'import time; time.sleep(60)']


@pytest.mark.parametrize('optional', ['rviz', 'camera', 'gripper'])
def test_rviz_can_close_and_reopen_without_stopping_stack(tmp_path, optional):
    manager = ToyManager(tmp_path/'unused', root=tmp_path, conflict_check=lambda: [])
    try:
        manager.start()
        manager.open_rviz()
        if optional in ('camera', 'gripper'):
            manager.spawn(optional, [sys.executable, '-c', 'import time; time.sleep(60)'])
        old = manager.children[optional]
        old.terminate()
        old.wait(timeout=3)
        manager.poll()
        assert not manager.stopping
        manager.open_rviz()
        assert manager.children['rviz'].pid != old.pid
        rviz = manager.children['rviz']
        manager.close()
        assert rviz.poll() is not None
        assert not manager.manifest.exists()
    finally:
        manager.close()


def test_abandoned_session_recovered_without_touching_unowned_child(tmp_path):
    environment = dict(os.environ, **{SESSION_KEY: 'abandoned-test'})
    owned = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'], env=environment)
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    manager = ToyManager(tmp_path/'unused', root=tmp_path, conflict_check=lambda: [])
    manager.manifest.write_text(json.dumps({'owner': None, 'token': 'abandoned-test', 'directory': str(tmp_path)}))
    try:
        manager.start()
        assert manager.stopping
        pump(manager, lambda: bool(manager.children) and not manager.stopping)
        owned.wait(timeout=3)
        assert unrelated.poll() is None
        assert not manager.live
    finally:
        manager.close()
        for child in (owned, unrelated):
            if child.poll() is None:
                child.kill()
            child.wait()


def test_new_session_descendant_is_cleaned_up(tmp_path):
    manager = ToyManager(tmp_path/'unused', root=tmp_path, conflict_check=lambda: [])
    escaped = None
    try:
        manager.start()
        escaped = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                                   env=manager.environment, start_new_session=True)
        manager.close()
        escaped.wait(timeout=3)
    finally:
        manager.close()
        if escaped and escaped.poll() is None:
            escaped.kill()
            escaped.wait()


def test_pid_identity_mismatch_is_not_signalled():
    saved = identity(os.getpid())
    saved['start'] += 1
    assert not is_same(saved)
    send_signal(saved, signal.SIGKILL)  # Must leave this test process alive.


def test_other_launcher_manifest_survives_close(tmp_path):
    manager = ToyManager(tmp_path/'unused', root=tmp_path, conflict_check=lambda: [])
    saved = {'owner': identity(os.getppid()), 'token': 'another-owner', 'directory': str(tmp_path)}
    manager.manifest.write_text(json.dumps(saved))
    with pytest.raises(RuntimeError, match='still running'):
        manager.start()
    manager.close()
    assert json.loads(manager.manifest.read_text()) == saved


def test_parent_death_interrupts_managed_child(tmp_path):
    pidfile = tmp_path/'child.pid'
    parent_code = '''
import os, subprocess, sys, time
child = subprocess.Popen([sys.executable, '-m', 'ur_vr_teleop.process_ownership', str(os.getpid()),
    sys.executable, '-c', 'import os,time; from pathlib import Path; Path('+repr(sys.argv[1])+').write_text(str(os.getpid())); time.sleep(60)'])
time.sleep(60)
'''
    parent = subprocess.Popen([sys.executable, '-c', parent_code, str(pidfile)])
    saved = None
    try:
        deadline = time.monotonic()+3
        while not pidfile.exists() and time.monotonic() < deadline:
            time.sleep(.02)
        assert pidfile.exists()
        saved = identity(int(pidfile.read_text()))
        parent.kill()
        parent.wait()
        deadline = time.monotonic()+3
        while is_same(saved) and time.monotonic() < deadline:
            time.sleep(.02)
        assert not is_same(saved)
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait()
        if saved:
            send_signal(saved, signal.SIGKILL)


def test_managed_start_restart_and_stop_do_not_touch_unowned_processes(tmp_path):
    manager = ToyManager(tmp_path/'unused.yaml', root=tmp_path, conflict_check=lambda: [])
    try:
        manager.start(True)
        first = {p.pid for p in manager.children.values()}
        assert len(first) == 3
        manager.stop(restart=False)
        pump(manager, lambda: not manager.stopping and bool(manager.children))
        assert not manager.live
        assert not first.intersection(p.pid for p in manager.children.values())
        manager.stop()
        pump(manager, lambda: not manager.stopping)
        assert manager.children == {}
    finally:
        manager.close()


def test_driver_failure_stops_arm_but_keeps_shared_tracking(tmp_path):
    manager = ToyManager(tmp_path/'unused.yaml', root=tmp_path, conflict_check=lambda: [])
    try:
        manager.start()
        tracker = manager.children['tracking']
        manager.children['driver'].terminate()
        pump(manager, lambda: 'driver' not in manager.children and not manager.stopping)
        assert manager.children.get('tracking') is tracker
        assert tracker.poll() is None
        assert 'ROBOT FAULT' in manager.message
        assert manager.restart_mode is None
    finally:
        manager.close()


def test_start_with_retained_children_does_not_duplicate_stack(tmp_path):
    manager = ToyManager(tmp_path/'unused.yaml', root=tmp_path, conflict_check=lambda: [])
    try:
        manager.start()
        old = {name: p.pid for name, p in manager.children.items()}
        manager.spawn('gripper', [sys.executable, '-c', 'import time; time.sleep(60)'])
        manager.start()  # retained children -> one full stop/start
        pump(manager, lambda: not manager.stopping and manager.children
             and old['driver'] not in {p.pid for p in manager.children.values()})
        assert set(manager.children) == set(old)
        assert old['driver'] not in {p.pid for p in manager.children.values()}
        assert old['tracking'] not in {p.pid for p in manager.children.values()}
    finally:
        manager.close()


def test_foreign_stack_is_not_adopted_or_killed(tmp_path):
    manager = ToyManager(tmp_path/'unused.yaml', root=tmp_path, conflict_check=lambda: [(123, 'foreign')])
    with pytest.raises(RuntimeError, match='nothing was killed'):
        manager.start()
    assert manager.children == {}


def test_program_stop_is_requested_before_process_signals(tmp_path):
    from ur_vr_teleop.program_control import ProgramStopResult
    events = []
    manager = ToyManager(tmp_path/'unused.yaml', root=tmp_path, conflict_check=lambda: [],
                         program_stop=lambda timeout: events.append('program') or ProgramStopResult('stopped'))
    original = manager.signal_groups
    manager.signal_groups = lambda sig, scope=None: (events.append('signal'), original(sig, scope))
    try:
        manager.start()
        manager.stop()
        assert events[:2] == ['program', 'signal']
        pump(manager, lambda: not manager.stopping)
        assert manager.program_stop_result.status == 'stopped'
    finally:
        manager.close()


def test_driver_failure_keeps_independent_gripper(tmp_path):
    manager = ToyManager(tmp_path/'unused.yaml', root=tmp_path, conflict_check=lambda: [])
    try:
        manager.start()
        manager.spawn('gripper', [sys.executable, '-c', 'import time; time.sleep(60)'])
        gripper = manager.children['gripper']
        manager.children['driver'].terminate()
        pump(manager, lambda: 'driver' not in manager.children and not manager.stopping)
        assert gripper.poll() is None
        assert manager.children.get('gripper') is gripper
        assert 'gripper' in manager.message.lower()
    finally:
        manager.close()
        if gripper.poll() is None:
            gripper.kill()
            gripper.wait()


def test_tuned_xacro_changes_only_servoj_parameters(tmp_path):
    import xacro
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('ur_description'))
    tuned = tuned_description(share, tmp_path/'description', 500, .08)
    mappings = {'name': 'ur10e', 'ur_type': 'ur10e', 'robot_ip': '192.168.1.251',
                'kinematics_params': '/home/ubuntu-ros/Documents/Teleop/ur_teleop/config/ur10e_calibration.yaml'}
    original = ET.fromstring(xacro.process_file(str(share/'urdf/ur.urdf.xacro'), mappings=mappings).toxml())
    modified = ET.fromstring(xacro.process_file(str(tuned), mappings=mappings).toxml())
    for tree in (original, modified):
        for name, value in [('servoj_gain', '500'), ('servoj_lookahead_time', '0.08')]:
            found = tree.findall(f'.//ros2_control/hardware/param[@name="{name}"]')
            assert len(found) == 1
            if tree is modified:
                assert found[0].text == value
            found[0].text = value
    assert ET.tostring(original) == ET.tostring(modified)


def test_generated_driver_command_stays_inactive(tmp_path):
    import shutil
    import yaml
    (tmp_path/'config').mkdir()
    shutil.copy('/home/ubuntu-ros/Documents/Teleop/ur_teleop/config/commissioning_controllers.yaml', tmp_path/'config/commissioning_controllers.yaml')
    source=Path('/home/ubuntu-ros/Documents/Teleop/ur_teleop/config/real_teleop.yaml')
    config=tmp_path/'config/real_teleop.yaml'
    settings=yaml.safe_load(source.read_text())
    settings['gripper']['process_enabled']=True
    config.write_text(yaml.safe_dump(settings))
    m = ProcessManager(config, root=tmp_path)
    m.directory = tmp_path/'logs'
    commands = m.commands(True)
    assert 'activate_joint_controller:=false' in commands['driver']
    assert 'headless_mode:=false' in commands['driver']
    assert 'motion_output_enabled:=true' in commands['teleop']
    assert commands['gripper'][:4] == ['ros2', 'run', 'ur_bus_servo_gripper', 'gripper']
    assert 'config_file:='+str(m.config.parent/'gripper.json') in commands['gripper']
    assert any(arg.startswith('description_file:/') or arg.startswith('description_file:=/') for arg in commands['driver'])
    controller_file = next(arg.split(':=', 1)[1] for arg in commands['driver'] if arg.startswith('controllers_file:='))
    data = yaml.safe_load(Path(controller_file).read_text())
    assert data['controller_manager']['ros__parameters']['scaled_joint_trajectory_controller']['type'] == 'ur_teleop_controllers/SmoothScaledJointTrajectoryController'
    assert data['scaled_joint_trajectory_controller']['ros__parameters']['smoothing.max_acceleration'] == 1.
    assert data['scaled_joint_trajectory_controller']['ros__parameters']['smoothing.max_jerk'] == 10.
    settings['gripper']['process_enabled']=False
    config.write_text(yaml.safe_dump(settings))
    disabled=ProcessManager(config,root=tmp_path);disabled.directory=tmp_path/'disabled-logs'
    assert 'gripper' not in disabled.commands(True)


def test_ros_parser_catches_unquoted_empty_prefix(tmp_path):
    from ur_vr_teleop.desktop import validate_controller_file
    broken = tmp_path/'broken.yaml'
    broken.write_text('test:\n  ros__parameters:\n    tf_prefix: $(var tf_prefix)\n')
    with pytest.raises(Exception, match='parse|value|Value'):
        validate_controller_file(broken)
    broken.write_text('test:\n  ros__parameters:\n    tf_prefix: "$(var tf_prefix)"\n')
    validate_controller_file(broken)


def test_nested_launch_child_failure_stops_even_when_launch_parent_survives(tmp_path):
    manager = ToyManager(tmp_path/'unused', root=tmp_path, conflict_check=lambda: [])
    try:
        manager.start()
        with (manager.directory/'driver.log').open('a') as stream:
            stream.write('[ERROR] [ur_ros2_control_node-1]: process has died [pid 12, exit code -6]\n')
        assert manager.children['driver'].poll() is None
        pump(manager, lambda: 'driver' not in manager.children and not manager.stopping)
        assert 'tracking' in manager.children
        assert 'ROS child failed' in manager.message
        assert 'ur_ros2_control_node' in manager.message
    finally:
        manager.close()


def test_optional_camera_exit_does_not_stop_robot_stack(tmp_path):
    manager = ToyManager(tmp_path/'unused', root=tmp_path, conflict_check=lambda: [])
    try:
        manager.start()
        manager.spawn('camera', [sys.executable, '-c', 'raise SystemExit(1)'])
        manager.children['camera'].wait(timeout=3)
        manager.poll()
        assert not manager.stopping
        assert manager.children['driver'].poll() is None
        assert manager.children['teleop'].poll() is None
    finally:
        manager.close()


def test_v4_default_is_live_but_preview_and_supervised_rollback_are_explicit():
    from ur_vr_teleop.desktop import parse_options
    assert parse_options([]).live
    assert not parse_options([]).engineering_session
    assert parse_options(['--live']).live
    assert not parse_options(['--preview']).live
    assert parse_options(['--preview','--engineering-session']).engineering_session
    with pytest.raises(SystemExit):parse_options(['--live','--preview'])
