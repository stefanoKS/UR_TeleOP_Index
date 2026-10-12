"""Isolated GUI render check: localhost domain 147, synthetic status, no robot."""
import os
import json
import time
from pathlib import Path
from types import SimpleNamespace
from PIL import ImageGrab
import rclpy
from std_msgs.msg import String
from ur_vr_teleop.gui import OperatorGUI

os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_DOMAIN_ID'] = '43'
rclpy.init(domain_id=147)
gui = OperatorGUI(manager=SimpleNamespace(message='TEST: managed launcher; no robot processes started', poll=lambda: None, directory=None))
try:
    gui.root.title('TEST — UR10e GUI — isolated domain 147')
    gui.on_status(String(data=json.dumps({
        'enable_requested': True, 'mode': 'DRY RUN — ROBOT OUTPUT DISABLED', 'armed': False, 'a_held': False,
        'optical': False, 'servo': 0, 'accepted_targets': 0,
        'reason': 'Disarmed at startup',
        'session_id': 'gui-render-test',
        'teleop_control_mode': 'velocity', 'tracking_station_policy': 'single_station_experimental',
        'linear_velocity_filter_s': .04, 'angular_velocity_filter_s': .06,
        'single_station_motion_scale': .25, 'optical_stations': {'0': 0, '1': 1},
        'active_station_count': 1, 'pose_fresh': True, 'motion_ready': False,
        'tracking_rejection': 'none',
        'workspace_radius_m': 0., 'observation_timeout_s': .35, 'linear': .1, 'angular': 30., 'joint_speed': .5, 'tracking_gain': 2.,
        'translation_scale': 1./1.2, 'rotation_scale': 1., 'control_period_s': .01, 'alignment_rpy_deg': [0., 0., -90.],
        'hardware_smoothing': {'velocity': .5, 'acceleration': 1., 'jerk': 10.},
        'linear_acceleration': .25, 'angular_acceleration_deg': 90.,
        'translation_damping_gain': .7, 'rotation_damping_gain': .5, 'damping_velocity_filter_s': .08,
        'motion_metrics_current': False,
        'motion_metrics': {'hand_m': .1, 'target_m': .1, 'tool_m': .098, 'error_m': .002,
                           'hand_deg': 10., 'target_deg': 10., 'tool_deg': 9.8},
        'optical_detail': 'Gen2 accepted paired sensors (250 ms): LH0=12 LH1=0; require >=4 on each of >=2 stations; optical_fresh=false',
        'blockers': ['Need stable optical observations from two stations',
                     'Perform optical-loss test and confirm in GUI'],
    })))
    gui.poll()
    gui.root.attributes('-topmost', True)
    gui.root.lift()
    # Give the window manager/compositor time to map the actual test window.
    for _ in range(15):
        gui.last_status_at = time.monotonic()
        gui.root.update()
        time.sleep(.05)
    assert 'Workspace: OFF' in gui.applied_limits.get()
    assert 'VR:robot translation 1.2:1, rotation 1:1' in gui.applied_limits.get()
    assert '100 Hz' in gui.applied_limits.get()
    assert '350 ms' in gui.applied_limits.get()
    assert 'Applied damping: translation 0.7, rotation 0.5' in gui.applied_limits.get()
    assert gui.fields[14].get() == '0.7' and gui.fields[15].get() == '0.5'
    assert gui.fields[16].get() == '80'
    assert '1 rad/s²' in gui.applied_limits.get() and '10 rad/s³' in gui.applied_limits.get()
    assert 'hand 10.0 / target 10.0 / tool 9.8 cm' in gui.motion.get()
    assert gui.fields[17].get() == 'velocity'
    assert gui.fields[18].get() == 'single_station_experimental'
    assert gui.fields[20].get() == '0.06'
    assert 'Lighthouse 1: detected (1 paired sensors)' in gui.detail.get()
    assert gui.enabled.get()
    assert 'disabled' not in gui.enable_button.state()
    x, y = gui.root.winfo_rootx(), gui.root.winfo_rooty()
    w, h = gui.root.winfo_width(), gui.root.winfo_height()
    target = Path('/tmp/ur_teleop_gui_check.png')
    ImageGrab.grab(bbox=(x, y, x+w, y+h)).save(target)
    gui.canvas.yview_moveto(1.)
    for _ in range(10):
        gui.last_status_at = time.monotonic()
        gui.root.update()
        time.sleep(.05)
    ImageGrab.grab(bbox=(x, y, x+w, y+h)).save('/tmp/ur_teleop_gui_check_bottom.png')
    assert gui.apply_button.winfo_rooty() < y+h
    assert gui.save_button.winfo_rooty() < y+h
    assert gui.reset_button.winfo_rooty() < y+h
    assert gui.fields[10].get() == '1.2'
    from concurrent.futures import Future
    pending = Future()
    gui.pending.append(('enable', pending))
    changed = dict(gui.last_status, session_id='new-backend-session', enable_requested=False, tracking_gain=2.)
    gui.alignment.set(True)
    gui.on_status(String(data=json.dumps(changed)))
    assert pending.cancelled() and not gui.pending
    assert not gui.alignment.get()
    gui.last_status_at = time.monotonic()-1.
    gui.poll()
    assert 'CONNECTION LOST' in gui.mode.get()
    assert 'disabled' in gui.enable_button.state()
    print('GUI rendered; enable stays checked on fault; stale control disabled. Screenshot:', target)
finally:
    gui.close()
    gui.node.destroy_node()
    rclpy.shutdown()
