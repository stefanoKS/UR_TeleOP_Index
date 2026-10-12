"""Local Tk operator panel. No shell execution or controller activation."""
import json
import os
import time
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool
from controller_manager_msgs.srv import SwitchController
from pathlib import Path
from .preferences import DEFAULTS, FIELD_PARAMETERS, validate, save_settings


class OperatorGUI:
    def __init__(self, manager=None):
        if os.environ.get('ROS_DOMAIN_ID') != '43':
            raise RuntimeError('GUI requires ROS_DOMAIN_ID=43')
        self.node = Node('teleop_operator_gui', namespace='vr_real')
        self.manager = manager
        self.config_path = Path(getattr(manager, 'config', Path.home()/'Documents/Teleop/ur_teleop/config/real_teleop.yaml'))
        self.pending_settings = {}
        self.closed = self.shutdown_requested = False
        self.root = tk.Tk()
        self.root.title('UR10e • Teleop commissioning')
        self.root.geometry('900x960')
        self.root.minsize(760, 650)
        self.root.configure(bg='#eef2f6')
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TFrame', background='#eef2f6')
        style.configure('TLabel', background='#eef2f6', font=('Sans', 11))
        style.configure('TButton', font=('Sans', 11), padding=9)
        canvas = tk.Canvas(self.root, bg='#eef2f6', highlightthickness=0)
        self.canvas = canvas
        scrollbar = ttk.Scrollbar(self.root, orient='vertical', command=canvas.yview)
        scrollbar.pack(side='right', fill='y')
        canvas.pack(side='left', fill='both', expand=True)
        canvas.configure(yscrollcommand=scrollbar.set)
        self.frame = ttk.Frame(canvas, padding=24)
        window = canvas.create_window((0, 0), window=self.frame, anchor='nw')
        self.frame.bind('<Configure>', lambda event: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.bind('<Configure>', lambda event: canvas.itemconfigure(window, width=event.width))
        canvas.bind_all('<Button-4>', lambda event: canvas.yview_scroll(-3, 'units'))
        canvas.bind_all('<Button-5>', lambda event: canvas.yview_scroll(3, 'units'))
        ttk.Label(self.frame, text='UR10e teleoperation', font=('Sans', 22, 'bold')).pack(anchor='w')
        if self.manager is not None:
            controls = ttk.Frame(self.frame)
            controls.pack(fill='x', pady=6)
            for label, action, mode in [('Preview', 'start', False), ('Live setup', 'start', True),
                                        ('Restart ROS', 'restart', None), ('Stop ROS', 'stop', None)]:
                ttk.Button(controls, text=label, command=lambda a=action, m=mode: self.stack_action(a, m)).pack(side='left', padx=3)
            ttk.Button(controls, text='Open logs', command=self.open_logs).pack(side='left', padx=3)
            ttk.Button(controls, text='Open RViz', command=self.open_rviz).pack(side='left', padx=3)
            self.stack_status = tk.StringVar(value='Stopped')
            ttk.Label(self.frame, textvariable=self.stack_status, wraplength=800).pack(anchor='w')
        self.camera_window = None
        self.camera_at = -float('inf')
        self.camera_text = 'Camera not connected'
        self.camera_status = tk.StringVar(value=self.camera_text)
        camera_row = ttk.Frame(self.frame)
        camera_row.pack(fill='x', pady=5)
        ttk.Button(camera_row, text='Open D405 camera', command=self.open_camera).pack(side='left', padx=(0, 8))
        ttk.Label(camera_row, textvariable=self.camera_status, wraplength=590).pack(side='left')
        self.node.create_subscription(String, '/wrist_camera/status', self.on_camera_status, 1)
        self.mode = tk.StringVar(value='WAITING FOR GATE — ARM DISABLED')
        ttk.Label(self.frame, textvariable=self.mode, font=('Sans', 13, 'bold')).pack(anchor='w', pady=(8, 12))
        self.visual_help = tk.StringVar(value='RViz: robot mesh = measured joints; orange marker = requested tool target.')
        ttk.Label(self.frame, textvariable=self.visual_help, wraplength=760).pack(anchor='w', pady=(0, 10))
        self.summary = tk.StringVar(value='No commissioning node connected')
        ttk.Label(self.frame, textvariable=self.summary, wraplength=760).pack(anchor='w')
        self.applied_limits = tk.StringVar(value='Waiting for applied limits')
        ttk.Label(self.frame, textvariable=self.applied_limits, wraplength=760).pack(anchor='w', pady=(6, 0))
        self.motion = tk.StringVar(value='Displacement check: hold A in preview to measure hand and target movement.')
        ttk.Label(self.frame, textvariable=self.motion, wraplength=820).pack(anchor='w', pady=(6, 0))
        self.detail = tk.StringVar(value='')
        ttk.Label(self.frame, textvariable=self.detail, wraplength=760).pack(anchor='w', pady=8)
        row = ttk.Frame(self.frame)
        row.pack(fill='x', pady=10)
        self.servo_button = ttk.Button(row, text='Start Servo solver', command=lambda: self.call('servo'))
        self.servo_button.pack(side='left', padx=(0, 8))
        self.prepare_button = ttk.Button(row, text='Prepare controller', command=self.prepare_controller)
        self.prepare_button.pack(side='left', padx=4)
        self.enabled = tk.BooleanVar(value=False)
        self.enable_button = ttk.Checkbutton(row, text='Enable teleop', variable=self.enabled, command=self.toggle_enabled)
        self.enable_button.pack(side='left', padx=8)
        ttk.Button(row, text='DISARM TELEOP', command=lambda: self.call('disarm')).pack(side='right')
        ttk.Label(self.frame, text='Disarm requests a ramped software stop. Use the pendant for an emergency stop.',
                  wraplength=760).pack(anchor='w', pady=(0, 15))
        controls = ttk.Frame(self.frame)
        controls.pack(fill='x')
        self.fields = {}
        for i, (label, value) in enumerate([('Linear speed limit (m/s)', '0.1'),
                                          ('Angular speed limit (deg/s)', '30.0'),
                                          ('Alignment RPY (degrees)', '0, 0, 0'),
                                          ('Joint speed (rad/s; save + restart)', '0.5'),
                                          ('Position-mode response (1/s)', '2.0'),
                                          ('Linear acceleration (m/s²)', '0.25'),
                                          ('Angular acceleration (rad/s²)', '1.5708'),
                                          ('Translation filter alpha (1 = off)', '0.8'),
                                          ('Rotation filter alpha (1 = off)', '0.8'),
                                          ('Joint acceleration warning (rad/s²)', '5.0'),
                                          ('Translation: VR distance per 1 robot', '1.2'),
                                          ('Rotation: VR angle per 1 robot', '1.0'),
                                          ('Translation jitter deadband (mm; 0 = off)', '1.0'),
                                          ('Rotation jitter deadband (deg; 0 = off)', '0.3'),
                                          ('Translation damping (0–1; 0 = off)', '0.0'),
                                          ('Rotation damping (0–1; 0 = off)', '0.0'),
                                          ('Damping velocity filter (ms)', '50'),
                                          ('Control mode (position / velocity)', 'velocity'),
                                          ('Station policy', 'two_station_strict'),
                                          ('Linear velocity filter (seconds; 0 = off)', '0.04'),
                                          ('Angular velocity filter (seconds; 0 = off)', '0.04'),
                                          ('Experimental motion fraction (0.01–1)', '0.25')]):
            ttk.Label(controls, text=label).grid(row=i, column=0, sticky='w', pady=4)
            v = tk.StringVar(value=value)
            if i in (17, 18):
                choices = ('position', 'velocity') if i == 17 else ('two_station_strict', 'single_station_experimental')
                ttk.Combobox(controls, textvariable=v, values=choices, state='readonly', width=30).grid(row=i, column=1, padx=16, pady=4)
            else:
                ttk.Entry(controls, textvariable=v, width=26).grid(row=i, column=1, padx=16, pady=4)
            self.fields[i] = v
        self.alignment = tk.BooleanVar(value=False)
        self.optical = tk.BooleanVar(value=False)
        ttk.Checkbutton(self.frame, text='I verified all three movement directions and tool0 in preview',
                        variable=self.alignment).pack(anchor='w', pady=(14, 4))
        ttk.Checkbutton(self.frame, text='I verified optical loss while the robot motion controllers were inactive',
                        variable=self.optical).pack(anchor='w', pady=4)
        self.apply_button = ttk.Button(self.frame, text='Apply settings / confirmations (disarmed)', command=self.apply)
        self.apply_button.pack(anchor='w', pady=8)
        saved_row = ttk.Frame(self.frame)
        saved_row.pack(anchor='w')
        self.save_button = ttk.Button(saved_row, text='Apply and save parameters', command=lambda: self.apply(save=True))
        self.save_button.pack(side='left', padx=(0, 8))
        self.reset_button = ttk.Button(saved_row, text='Reset to default parameters', command=self.reset_defaults)
        self.reset_button.pack(side='left')
        ttk.Label(self.frame, text='Mapping 1.2 means 12 cm of hand movement requests 10 cm at the robot.\n'
                  'Deadbands ignore small tracking jitter. Filter alpha: lower = smoother/slower; 1 = off.\n'
                  'Velocity mode uses hand speed; response gain and damping apply only in position mode.\n'
                  'The damping filter smooths measured velocity; more filtering adds delay.\n'
                  'Save restores tuning next launch. Enable and verification checkmarks start off.', wraplength=800).pack(anchor='w', pady=6)
        self.message = tk.StringVar(value='When enabled: faults stop motion. Release A, wait for readiness, then press A again.')
        ttk.Label(self.frame, textvariable=self.message, wraplength=760).pack(anchor='w', pady=8)
        self.blockers = tk.Text(self.frame, height=7, wrap='word', font=('Sans', 10), relief='flat', bg='#ffffff')
        self.blockers.pack(fill='both', expand=True)
        self.clients = {key: self.node.create_client(Trigger, name) for key, name in {
            'arm': '/vr_real/arm', 'disarm': '/vr_real/disarm', 'servo': '/vr_real/servo_node/start_servo'}.items()}
        self.enable_client = self.node.create_client(SetBool, '/vr_real/set_enabled')
        self.switch_client = self.node.create_client(SwitchController, '/controller_manager/switch_controller')
        self.settings = self.node.create_client(SetParametersAtomically, '/vr_real/teleop/set_parameters_atomically')
        self.heartbeat = self.node.create_publisher(Bool, '/vr_real/operator_present', 1)
        self.node.create_subscription(String, '/vr_real/operator_status', self.on_status, 1)
        self.last_status_at = -float('inf')
        self.last_status = {}
        self.pending = []
        self.heartbeat_at = -float('inf')
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after(20, self.poll)

    def on_camera_status(self, msg):
        self.camera_at = time.monotonic()
        self.camera_text = msg.data

    def open_camera(self):
        if self.camera_window is None or self.camera_window.closed:
            from .camera_view import CameraView
            self.camera_window = CameraView(self.root, self.node)
        else:
            self.camera_window.window.lift()

    def on_status(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict): return
            if not self.last_status or value.get('session_id') != self.last_status.get('session_id'):
                for _, future in self.pending:
                    future.cancel()
                self.pending.clear()
                self.pending_settings.clear()
                self.alignment.set(False)
                self.optical.set(False)
                for i, key in ((0, 'linear'), (1, 'angular'), (3, 'joint_speed'), (4, 'tracking_gain'),
                               (5, 'max_linear_acceleration_m_s2'), (6, 'max_angular_acceleration_rad_s2'),
                               (7, 'vr_translation_filter_alpha'), (8, 'vr_rotation_filter_alpha'),
                               (9, 'joint_acceleration_warning_rad_s2')):
                    if key in value:
                        self.fields[i].set(f'{value[key]:g}')
                for i, key in ((10, 'translation_scale'), (11, 'rotation_scale'),
                               (12, 'vr_translation_deadband_m'), (13, 'vr_rotation_deadband_deg'),
                               (14, 'translation_damping_gain'), (15, 'rotation_damping_gain'),
                               (16, 'damping_velocity_filter_s')):
                    if key in value:
                        v = 1./value[key] if i in (10, 11) else value[key] * (1000 if i in (12, 16) else 1)
                        self.fields[i].set(f'{v:.12g}')
                for i in range(17, len(FIELD_PARAMETERS)):
                    key = FIELD_PARAMETERS[i]
                    if key in value:
                        self.fields[i].set(str(value[key]))
                if 'alignment_rpy_deg' in value:
                    self.fields[2].set(', '.join(str(x) for x in value['alignment_rpy_deg']))
            self.last_status, self.last_status_at = value, time.monotonic()
        except (ValueError, TypeError): pass

    def stack_action(self, action, mode=None):
        if self.manager is None:
            return
        for _, future in self.pending:
            future.cancel()
        self.pending.clear()
        self.pending_settings.clear()
        self.heartbeat.publish(Bool(data=False))
        if self.enable_client.service_is_ready():
            request = SetBool.Request()
            request.data = False
            self.enable_client.call_async(request)
        try:
            if action == 'stop':
                self.manager.message = 'Stopping owned ROS processes; robot faults must be handled on the pendant.'
                self.manager.stop()
            elif action == 'restart' or self.manager.children:
                self.manager.message = 'Restarting ROS disabled; acknowledge any robot fault on the pendant first.'
                self.manager.stop(restart=self.manager.live if mode is None else mode)
            else:
                self.manager.start(bool(mode))
        except Exception as error:
            self.manager.message = str(error)

    def open_logs(self):
        if self.manager and self.manager.directory:
            import subprocess
            subprocess.Popen(['xdg-open', str(self.manager.directory)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def open_rviz(self):
        try:
            self.manager.open_rviz()
        except Exception as error:
            self.manager.message = str(error)

    def prepare_controller(self):
        data = self.last_status
        if (time.monotonic()-self.last_status_at >= .5 or data.get('armed') or data.get('a_held')
                or data.get('enable_requested')):
            self.message.set('Disable teleop and release A before preparing the controller.')
            return
        if not str(data.get('mode', '')).startswith('LIVE'):
            self.message.set('Preview mode does not activate robot motion controllers. Use Start Servo solver for preview.')
            return
        if not data.get('observations_healthy') or not data.get('stationary') or not data.get('robot_ready'):
            self.message.set('Fresh tracking/joints/TF, robot standstill and External Control are required; acknowledge faults on the pendant.')
            return
        if data.get('controller_ready'):
            self.call('servo')
            return
        if not self.switch_client.service_is_ready():
            self.message.set('Controller manager unavailable; use Restart ROS after resolving the robot fault.')
            return
        request = SwitchController.Request()
        request.activate_controllers = ['scaled_joint_trajectory_controller']
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 3
        self.pending.append(('prepare_controller', self.switch_client.call_async(request)))
        self.message.set('Preparing scaled controller; teleop stays disabled.')

    def call(self, name):
        if not self.clients[name].service_is_ready():
            self.message.set(f'{name}: service unavailable')
            return
        if name == 'arm' and time.monotonic()-self.last_status_at >= .5:
            self.message.set('Gate status is stale; cannot arm')
            return
        self.pending.append((name, self.clients[name].call_async(Trigger.Request())))
        self.message.set(name + ': request sent')

    def toggle_enabled(self):
        if time.monotonic()-self.last_status_at >= .5 or not self.enable_client.service_is_ready():
            self.enabled.set(bool(self.last_status.get('enable_requested', False)))
            self.message.set('Enable service/status unavailable')
            return
        request = SetBool.Request()
        request.data = self.enabled.get()
        self.pending.append(('enable', self.enable_client.call_async(request)))
        self.message.set('Enable setting requested')

    def field_values(self):
        values = {}
        for i, name in enumerate(FIELD_PARAMETERS):
            if isinstance(DEFAULTS[name], str):
                value = self.fields[i].get()
            elif i == 2:
                value = [float(x.strip()) for x in self.fields[i].get().split(',')]
            else:
                value = float(self.fields[i].get())
                if i in (10, 11):
                    if value <= 0:
                        raise ValueError('VR:robot mapping must be positive')
                    value = 1./value
                elif i in (12, 16):
                    value /= 1000.
            values[name] = value
        return validate(values)

    def reset_defaults(self):
        if self.last_status.get('armed') or self.last_status.get('a_held'):
            self.message.set('Disable teleop and release A before resetting parameters.')
            return
        for i, name in enumerate(FIELD_PARAMETERS):
            value = DEFAULTS[name]
            if isinstance(value, str):
                text = value
            elif i == 2:
                text = ', '.join(str(v) for v in value)
            else:
                value = 1./value if i in (10, 11) else value * (1000 if i in (12, 16) else 1)
                text = f'{value:.12g}'
            self.fields[i].set(text)
        self.alignment.set(False)
        self.optical.set(False)
        self.apply(save=True)

    def apply(self, save=False):
        try:
            values = self.field_values()
            if not self.settings.service_is_ready(): raise ValueError('Settings service unavailable')
            if any(label == 'settings' for label, _ in self.pending):
                raise ValueError('Waiting for the previous settings request')
            request = SetParametersAtomically.Request()
            applied = dict(values, alignment_verified=self.alignment.get(), optical_loss_test_verified=self.optical.get())
            hardware = self.last_status.get('hardware_smoothing')
            if hardware and abs(values['max_joint_speed_rad_s']-hardware['velocity']) > 1e-9:
                if not save:
                    raise ValueError('Joint speed needs Apply and save parameters, then Restart ROS.')
                # Save the requested startup limit, but leave the running gate
                # consistent with the native controller until that restart.
                applied.pop('max_joint_speed_rad_s')
            request.parameters = [Parameter(name, value=value).to_parameter_msg() for name, value in applied.items()]
            future = self.settings.call_async(request)
            self.pending_settings[future] = values if save else None
            self.pending.append(('settings', future))
        except (ValueError, TypeError) as exc:
            self.message.set(str(exc))

    def complete_settings(self, future, result):
        values = self.pending_settings.pop(future, None)
        if not result.successful:
            self.message.set(result.reason)
        elif values is None:
            self.message.set('Settings applied (session only)')
        else:
            try:
                path = save_settings(self.config_path, values)
            except (OSError, ValueError) as exc:
                self.message.set('Settings applied but NOT saved: ' + str(exc))
                return
            hardware = self.last_status.get('hardware_smoothing')
            restart = hardware and abs(values['max_joint_speed_rad_s']-hardware['velocity']) > 1e-9
            self.message.set(('Parameters saved; Restart ROS to apply the joint speed. Other tuning applied. '
                              if restart else 'Parameters applied and saved: ') + str(path))

    def poll(self):
        if self.closed:
            return
        if self.shutdown_requested or not rclpy.ok():
            self.close()
            return
        rclpy.spin_once(self.node, timeout_sec=0)
        if self.manager is not None:
            self.manager.poll()
            self.stack_status.set(self.manager.message)
        now = time.monotonic()
        if now-self.heartbeat_at >= .1:
            self.heartbeat.publish(Bool(data=True))
            self.heartbeat_at = now
        self.camera_status.set(self.camera_text if now-self.camera_at < 4. else 'D405 unavailable / no recent status (teleop unaffected)')
        stale = now-self.last_status_at >= .5
        data = self.last_status
        self.mode.set('CONNECTION LOST — ARM DISABLED' if stale else str(data.get('mode', 'Unknown')))
        self.visual_help.set(
            'RViz: robot mesh = measured joints; orange marker = requested tool target.\n'
            + ('Dry run computes targets only. The physical arm and robot mesh do not follow your hand.'
               if not stale and str(data.get('mode', '')).startswith('DRY RUN')
               else 'The robot mesh changes only when measured robot joints change.'))
        if data.get('teleop_control_mode') == 'velocity':
            self.visual_help.set('Velocity-following: hand speed requests TCP speed. RViz marker shows measured TCP; there is no positional target to chase.')
        if stale:
            self.summary.set('No fresh gate status. Do not infer that the robot has stopped; check the pendant.')
        else:
            self.summary.set(f"Enabled: {'ON' if data.get('enable_requested') else 'OFF'}  |  Gate: {'ARMED' if data.get('armed') else 'DISARMED'}  |  A: {'held' if data.get('a_held') else 'released'}  |  Optical: {'fresh' if data.get('optical') else 'LOST'}  |  Servo: {data.get('servo_detail', data.get('servo'))}\nAccepted targets: {data.get('accepted_targets')}  |  {data.get('reason')}")
            stations = data.get('optical_stations', {})
            station_text = ' | '.join(f"Lighthouse {i}: {'detected' if stations.get(str(i), 0) else 'not detected'} ({stations.get(str(i), 0)} paired sensors)" for i in (0, 1))
            diagnostic = data.get('velocity_diagnostics', {})
            def vector(value):
                return 'unavailable' if value is None else ', '.join(f'{v:.4g}' for v in value)
            self.detail.set(station_text + f"\nActive stations: {data.get('active_station_count', 0)} | Pose fresh: {data.get('pose_fresh', False)} | Motion ready: {data.get('motion_ready', False)}"
                + f"\nPolicy: {data.get('tracking_station_policy')} | Control: {data.get('teleop_control_mode')} | Last rejection: {data.get('tracking_rejection')}"
                + f"\nHand velocity [m/s, rad/s]: {vector(diagnostic.get('observed_hand_velocity', diagnostic.get('filtered_hand_velocity')))}"
                + f"\nCommand TCP velocity: {vector(diagnostic.get('published_twist'))}"
                + f"\nMeasured TCP velocity: {vector(diagnostic.get('measured_tcp_velocity'))}"
                + f"\nScale: {diagnostic.get('motion_scale')} | Clamped: {diagnostic.get('clamped')} | Sample-to-command age (s): {diagnostic.get('command_age_s')}"
                + '\n' + str(data.get('optical_detail', '')))
        if stale:
            self.applied_limits.set('Applied limits unavailable: gate status stale')
        elif any(key not in data for key in ('workspace_radius_m', 'observation_timeout_s', 'joint_speed', 'tracking_gain', 'translation_scale', 'control_period_s')):
            self.applied_limits.set('Older gate version: restart commissioning to load updated limits/diagnostics')
        else:
            workspace = 'OFF' if data['workspace_radius_m'] == 0 else f"{data['workspace_radius_m']*100:g} cm radius"
            self.applied_limits.set(f"Applied: {data.get('linear', 0):g} m/s, {data.get('angular', 0):g} deg/s | "
                f"Joint: {data['joint_speed']:g} rad/s\n"
                f"Workspace: {workspace} | VR:robot translation {1/data['translation_scale']:g}:1, rotation {1/data.get('rotation_scale', 1.):g}:1 | Response: {data['tracking_gain']:g}/s | "
                f"{1/data['control_period_s']:g} Hz\n"
                f"Observation timeout: {data['observation_timeout_s']*1000:g} ms")
            if 'linear_acceleration' in data:
                self.applied_limits.set(self.applied_limits.get() +
                    f" | Command ramp: {data['linear_acceleration']:g} m/s², {data['angular_acceleration_deg']:g} deg/s²")
            if 'translation_damping_gain' in data:
                self.applied_limits.set(self.applied_limits.get() +
                    f"\nApplied damping: translation {data['translation_damping_gain']:g}, rotation {data['rotation_damping_gain']:g} | "
                    f"Velocity filter: {1000*data['damping_velocity_filter_s']:g} ms")
            smoothing = data.get('hardware_smoothing')
            self.applied_limits.set(self.applied_limits.get() + (
                f"\nFinal joint command: {smoothing['velocity']:g} rad/s | {smoothing['acceleration']:g} rad/s² | "
                f"{smoothing['jerk']:g} rad/s³ (restart to change)" if smoothing else
                '\nFinal joint command smoothing: waiting for controller settings'))
        enabling = any(label in ('enable', 'prepare_controller') for label, _ in self.pending)
        metrics = data.get('motion_metrics')
        if stale:
            self.motion.set('Displacement measurements unavailable: status stale')
        elif metrics:
            label = 'Current clutch' if data.get('motion_metrics_current') else 'Last clutch sample'
            self.motion.set(f"{label} — from press-A anchor (tool0; target is lightly filtered):\n"
                f"Translation: hand {metrics['hand_m']*100:.1f} / target {metrics['target_m']*100:.1f} / "
                f"tool {metrics['tool_m']*100:.1f} cm | target–tool error {metrics['error_m']*1000:.1f} mm\n"
                f"Rotation: hand {metrics['hand_deg']:.1f} / target {metrics['target_deg']:.1f} / "
                f"tool {metrics['tool_deg']:.1f} degrees")
        else:
            self.motion.set('Displacement check: hold A in preview to measure hand and target movement.')
        if not enabling:
            self.enabled.set(bool(data.get('enable_requested', False)))
        self.enable_button.state(['disabled'] if stale or enabling or 'enable_requested' not in data else ['!disabled'])
        self.apply_button.state(['disabled'] if stale or data.get('armed') or data.get('a_held') else ['!disabled'])
        for button in (self.save_button, self.reset_button):
            button.state(['disabled'] if stale or data.get('armed') or data.get('a_held') else ['!disabled'])
        self.servo_button.state(['disabled'] if stale or data.get('armed') else ['!disabled'])
        self.prepare_button.state(['disabled'] if stale or data.get('armed') or data.get('a_held') or data.get('enable_requested') or enabling else ['!disabled'])
        self.blockers.configure(state='normal')
        self.blockers.delete('1.0', 'end')
        diagnostic = data.get('joint_diagnostics', {})
        detail = '\n'.join(data.get('blockers', [])) or 'No reported readiness blockers'
        if diagnostic.get('warning_joints'):
            detail += '\nUpstream Servo warnings (before final smoothing): '+', '.join(diagnostic['warning_joints'])+' (see teleop log)'
        self.blockers.insert('end', detail)
        self.blockers.configure(state='disabled')
        remaining = []
        for label, future in self.pending:
            if not future.done():
                remaining.append((label, future))
                continue
            try:
                result = future.result()
                if label == 'settings':
                    self.complete_settings(future, result.result)
                elif label == 'prepare_controller':
                    self.message.set('Controller prepared; start Servo solver next.' if result.ok else
                                     'Controller activation failed; inspect driver log. No other controller was stopped.')
                else: self.message.set(f'Last {label} response: {result.message}.')
            except Exception as exc: self.message.set(f'{label}: {exc}')
        self.pending = remaining
        self.root.after(20, self.poll)

    def close(self):
        if self.closed:
            return
        self.closed = True
        if getattr(self, 'camera_window', None) is not None:
            self.camera_window.close()
        try:
            if rclpy.ok():
                self.heartbeat.publish(Bool(data=False))
        finally:
            self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    gui = None
    try:
        gui = OperatorGUI()
        gui.root.mainloop()
    finally:
        if gui: gui.node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
