"""Disarmed tool profile editor; no robot commands and no runtime geometry edits."""
from concurrent.futures import ThreadPoolExecutor
import tkinter as tk
from tkinter import ttk
import yaml
from .preferences import load_settings
from .tool_tcp import read_offset, save_profile, validate_offset


class ToolTCPPanel:
    def __init__(self, gui):
        self.gui = gui
        try:
            self.check_disarmed()
            self.settings = load_settings(gui.config_path)
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
            gui.notify(str(error), True)
            return
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='tcp-read')
        self.future = None
        self.closed = False
        self.window = tk.Toplevel(gui.root)
        self.window.title('Tool tip / rotation centre')
        self.window.transient(gui.root)
        self.window.grab_set()
        self.window.protocol('WM_DELETE_WINDOW', self.close)
        self.window.bind('<Destroy>', self.destroyed)
        frame = ttk.Frame(self.window, padding=16)
        frame.pack(fill='both', expand=True)
        ttk.Label(frame, text='Tool tip relative to tool0. Read the active TCP from the UR, then review and save.\n'
                  'Changes take effect after Restart ROS. The pendant TCP must match the saved profile.\n'
                  'This sets a reference point; it does not add tool collision geometry.', wraplength=650).pack(anchor='w')
        self.values = []
        offset = self.settings['robot'].get('tcp_offset', [0.]*6)
        for name, value in zip(('X (mm)', 'Y (mm)', 'Z (mm)', 'Rx (rad)', 'Ry (rad)', 'Rz (rad)'),
                               [v*1000 for v in offset[:3]]+offset[3:]):
            row = ttk.Frame(frame); row.pack(fill='x', pady=3)
            ttk.Label(row, text=name, width=15).pack(side='left')
            var = tk.StringVar(value=f'{value:.12g}')
            ttk.Entry(row, textvariable=var, width=32).pack(side='left')
            self.values.append(var)
        ttk.Label(frame, text='Rx/Ry/Rz are a rotation vector in radians, not roll/pitch/yaw.').pack(anchor='w')
        self.message = tk.StringVar(value='Existing sessions keep their current TCP until restart.')
        ttk.Label(frame, textvariable=self.message, wraplength=650).pack(anchor='w', pady=8)
        self.read_button = ttk.Button(frame, text='Read TCP from robot', command=self.read)
        self.read_button.pack(side='left')
        self.save_button = ttk.Button(frame, text='Save for restart', command=self.save)
        self.save_button.pack(side='left')
        ttk.Button(frame, text='Close', command=self.close).pack(side='right')

    def check_disarmed(self):
        d = self.gui.last_status
        if (self.gui.stale() or d.get('armed') is not False or d.get('a_held') is not False
                or d.get('enable_requested') is not False or self.gui.startup.requested):
            raise ValueError('Press STOP TELEOP and release A; fresh disarmed backend status is required.')

    def read(self):
        try:
            self.check_disarmed()
            if self.future is not None:
                return
            self.read_session = self.gui.session
            self.future = self.pool.submit(read_offset, self.settings['robot']['ip'])
            self.read_button.state(['disabled']); self.save_button.state(['disabled'])
            self.message.set('Reading active pendant TCP…')
            self.window.after(50, self.poll)
        except (ValueError, KeyError) as error:
            self.message.set(str(error))

    def poll(self):
        if self.closed:
            return
        if not self.future.done():
            self.window.after(50, self.poll)
            return
        try:
            result = self.future.result()
            self.check_disarmed()
            if self.gui.session != self.read_session:
                raise ValueError('Backend session changed; read TCP again.')
            for var, value in zip(self.values, [v*1000 for v in result[:3]]+result[3:]):
                var.set(f'{value:.12g}')
            self.message.set('Read complete. Review the tool tip, save, then Restart ROS while disarmed.')
        except (OSError, ValueError) as error:
            self.message.set('TCP read failed: '+str(error))
        finally:
            self.future = None
            self.read_button.state(['!disabled']); self.save_button.state(['!disabled'])

    def save(self):
        try:
            self.check_disarmed()
            numbers = [float(var.get()) for var in self.values]
            offset = validate_offset([v/1000 for v in numbers[:3]]+numbers[3:])
            save_profile(self.gui.config_path, self.settings['robot']['ip'], offset)
            self.message.set('Saved. Restart ROS to apply. Check tip pose and validate at low speed under supervision.')
            self.gui.notify('Tool tip saved; Restart ROS required. Current session geometry is unchanged.')
        except (OSError, ValueError) as error:
            self.message.set(str(error))

    def destroyed(self, event):
        if event.widget == self.window and not self.closed:
            self.closed = True
            self.pool.shutdown(wait=True, cancel_futures=True)

    def close(self):
        self.window.destroy()
