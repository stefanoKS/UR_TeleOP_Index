"""Tk-only calibration editor; ROS requests and persistence stay asynchronous."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
from .gripper_diagnostics import temperature_text, serial_text
import time
import tkinter as tk
from tkinter import ttk
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from ur_bus_servo_gripper.gripper_config import Config, load, save
from ur_bus_servo_gripper.st3215_driver import ports


class GripperPanel:
    def __init__(self, gui):
        self.gui = gui
        self.path = gui.config_path.parent / 'gripper.json'
        self.window = tk.Toplevel(gui.root)
        self.window.title('Gripper Setup')
        self.window.geometry('680x700')
        self.closed = False
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='gripper-config')
        self.future = None
        self.ros_future = None
        self.clients = {}
        self.fields = {}
        self.authorized = tk.BooleanVar(value=False)
        self.enabled = tk.BooleanVar(value=False)
        self.invert = tk.BooleanVar(value=False)
        self.message = tk.StringVar(value='Connect is read-only. Review safe travel before any motion.')
        self.telemetry = tk.StringVar(value='Disconnected')
        canvas = tk.Canvas(self.window, highlightthickness=0)
        scrollbar = ttk.Scrollbar(self.window, orient='vertical', command=canvas.yview)
        scrollbar.pack(side='right', fill='y')
        canvas.pack(side='left', fill='both', expand=True)
        canvas.configure(yscrollcommand=scrollbar.set)
        body = ttk.Frame(canvas, padding=12)
        item = canvas.create_window((0, 0), window=body, anchor='nw')
        body.bind('<Configure>', lambda event: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.bind('<Configure>', lambda event: canvas.itemconfigure(item, width=event.width))
        self.window.bind('<Button-4>', lambda event: canvas.yview_scroll(-3, 'units'))
        self.window.bind('<Button-5>', lambda event: canvas.yview_scroll(3, 'units'))
        self.canvas = canvas
        ttk.Label(body, text='GRIPPER CONFIGURATION', font=('', 13, 'bold')).grid(row=0, columnspan=2, sticky='w')
        ttk.Label(body, textvariable=self.telemetry, wraplength=580).grid(row=1, columnspan=2, sticky='w', pady=5)
        labels = [('port','USB port (manual fallback allowed)'), ('servo_id','Servo ID (-1 discovers all IDs)'),
            ('open_ticks','OPEN encoder ticks'), ('closed_ticks','CLOSED encoder ticks'),
            ('min_ticks','Reviewed minimum ticks'), ('max_ticks','Reviewed maximum ticks'),
            ('speed','Maximum speed (SDK raw)'), ('acceleration','Acceleration (SDK raw)'),
            ('rate_ticks_s','Software target rate (ticks/s)'), ('jog_ticks','Jog increment (ticks)'),
            ('trigger_min','Trigger minimum'), ('trigger_max','Trigger maximum'),
            ('deadzone','Trigger deadzone (fraction)'), ('threshold_ticks','Change threshold (ticks)'),
            ('temperature_limit','Temperature fault threshold (°C)'),
            ('stall_error_ticks','Persistent error threshold (ticks)'), ('stall_seconds','No-progress timeout (s)')]
        for row, (key, label) in enumerate(labels, 2):
            ttk.Label(body, text=label).grid(row=row, column=0, sticky='w')
            variable = tk.StringVar()
            self.fields[key] = variable
            if key == 'port':
                widget = ttk.Combobox(body, textvariable=variable, values=ports(), width=35)
            else:
                widget = ttk.Entry(body, textvariable=variable, width=36)
            widget.grid(row=row, column=1, sticky='ew', pady=1)
        row = len(labels) + 2
        for label, variable in [('Enable trigger follow on future sessions', self.enabled),
                                ('Invert trigger', self.invert),
                                ('Authorize low-speed calibration (mechanism clear and supervised)', self.authorized)]:
            ttk.Checkbutton(body, text=label, variable=variable).grid(row=row, columnspan=2, sticky='w')
            row += 1
        actions = [('Connect / Read Servo', lambda: self.request('connect')),
            ('Read Current Position', lambda: self.request('read')),
            ('Set Current as OPEN', lambda: self.capture('open_ticks')),
            ('Set Current as CLOSED', lambda: self.capture('closed_ticks')),
            ('Jog Open Direction', lambda: self.request('jog_open')),
            ('Jog Close Direction', lambda: self.request('jog_close')),
            ('Save Settings', self.save), ('Restore Settings', self.restore),
            ('Test Open', lambda: self.request('open')), ('Test Close', lambda: self.request('close')),
            ('Test trigger (arm disarmed)', lambda: self.request('follow'))]
        for i, (label, action) in enumerate(actions):
            ttk.Button(body, text=label, command=action).grid(row=row+i//2, column=i%2, sticky='ew', pady=2)
        ttk.Label(body, textvariable=self.message, wraplength=580).grid(row=row+6, columnspan=2, sticky='w', pady=5)
        ttk.Label(body, text='Motion uses saved settings. Save does not move the motor.\n'
            'Pickup: move trigger ≥5%, then match actual closure.\n'
            'External servo supply required; holding is not a guaranteed grasp.', wraplength=580).grid(row=row+7, columnspan=2, sticky='w')
        body.columnconfigure(1, weight=1)
        self.publisher = gui.node.create_publisher(Bool, '/gripper/calibration_present', 1)
        self.restore()
        self.window.protocol('WM_DELETE_WINDOW', self.close)
        self.window.after(100, self.poll)

    def request(self, name):
        # Connect, read and reload are read-only and independent of the arm.
        # Motion services need explicit gripper calibration consent only.
        if name in ('jog_open', 'jog_close', 'open', 'close', 'follow') and not self.authorized.get():
            self.message.set('Explicitly authorize low-speed calibration first '
                             '(mechanism clear and supervised)')
            return
        if self.ros_future is not None:
            self.message.set('Request pending')
            return
        client = self.clients.setdefault(name, self.gui.node.create_client(Trigger, '/gripper/'+name)) if name not in self.clients else self.clients[name]
        if not client.service_is_ready():
            self.message.set('Gripper process unavailable; start the managed stack or inspect its log')
            return
        self.ros_future = client.call_async(Trigger.Request())
        self.message.set('Request pending; serial work runs outside the GUI')

    def capture(self, key):
        data = self.gui.gripper_data
        fb = data.get('feedback', {})
        if not data.get('feedback_valid') or time.monotonic()-self.gui.gripper_at >= .25:
            self.message.set('Fresh measured feedback required')
            return
        self.fields[key].set(str(fb['ticks']))
        self.fields['servo_id'].set(str(data['identity']['servo_id']))
        self.message.set('Captured actual encoder position; review and save')

    def restore(self):
        if self.future is None:
            self.future = ('load', self.executor.submit(load, self.path))

    def save(self):
        try:
            if self.future is not None:
                raise ValueError('A settings file operation is already pending')
            base = asdict(Config())
            for key, var in self.fields.items():
                base[key] = str(var.get()) if key == 'port' else type(base[key])(var.get())
            base.update(enabled=self.enabled.get(), invert=self.invert.get())
            config = Config(**base).validate()
            self.authorized.set(False)
            self.publisher.publish(Bool(data=False))
            self.future = ('save', self.executor.submit(save, self.path, config))
        except (ValueError, TypeError) as exc:
            self.message.set(str(exc))

    def poll(self):
        if self.closed:
            return
        # Calibration consent is gripper-owned.  It does not depend on the arm
        # being connected, disarmed or stationary.
        self.publisher.publish(Bool(data=self.authorized.get()))
        data = self.gui.gripper_data
        fresh = time.monotonic() - self.gui.gripper_at < .25
        fb = data.get('feedback', {}) if fresh and data.get('feedback_valid') else {}
        if data.get('temperature', {}).get('state', 'NORMAL') != 'NORMAL':
            self.authorized.set(False)
        self.telemetry.set(f"Connection: {data.get('connection', 'Disconnected') if fresh else 'Disconnected'}  "
            f"Servo ID: {data.get('identity', {}).get('servo_id', '—')}  "
            f"Session enabled: {data.get('session_enabled', False) if fresh else False}\n"
            f"Current Position: {fb.get('ticks', 'unavailable')} ticks\n"
            f"{temperature_text(data, fresh)}\n{serial_text(data)}\n"
            f"Last fault/warning: {data.get('last_fault_reason', 'none')}\n"
            f"Raw trigger: {data.get('raw_trigger')} · Observed range: {data.get('observed_trigger_range')}\n{data.get('reason', '')}")
        if self.future and self.future[1].done():
            action, future = self.future
            self.future = None
            try:
                config = future.result()
                if action == 'load':
                    for key, var in self.fields.items():
                        var.set(str(getattr(config, key)))
                    self.enabled.set(config.enabled)
                    self.invert.set(config.invert)
                else:
                    self.message.set('Saved atomically. Reloading read-only; pickup required.')
                    self.request('reload')
            except Exception as exc:
                self.message.set(str(exc))
        if self.ros_future is not None and self.ros_future.done():
            try:
                self.message.set(self.ros_future.result().message)
            except Exception as exc:
                self.message.set(str(exc))
            self.ros_future = None
        self.window.after(100, self.poll)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.publisher.publish(Bool(data=False))
        self.gui.node.destroy_publisher(self.publisher)
        for client in self.clients.values():
            self.gui.node.destroy_client(client)
        self.executor.shutdown(wait=True, cancel_futures=True)
        self.window.destroy()
