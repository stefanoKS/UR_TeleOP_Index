"""Tk-only reviewed External Control program identity editor."""
import tkinter as tk
from tkinter import ttk
from .program_control import ProgramConfig, save_program_config, same_program


class ProgramPanel:
    """Configure and verify the approved .urp; never loads or plays by itself."""
    def __init__(self, gui):
        self.gui = gui
        self.window = tk.Toplevel(gui.root)
        self.window.title('External Control Program')
        self.window.geometry('620x280')
        self.closed = False
        self.config = gui.program_config
        self.enabled = tk.BooleanVar(value=self.config.enabled)
        self.program = tk.StringVar(value=self.config.program)
        self.status = tk.StringVar(value=(
            'Configure the approved External Control .urp. START TELEOP will then check '
            'Remote Control, load/play it, and wait for driver confirmation.'
            if not getattr(gui, 'program_config_error', '')
            else 'Configuration error: ' + gui.program_config_error))
        body = ttk.Frame(self.window, padding=14)
        body.pack(fill='both', expand=True)
        body.columnconfigure(0, weight=1)
        ttk.Label(body, text='EXTERNAL CONTROL PROGRAM', font=('', 12, 'bold')).grid(
            row=0, column=0, columnspan=2, sticky='w')
        ttk.Checkbutton(body, text='Automatic Dashboard program start on START TELEOP',
                        variable=self.enabled).grid(row=1, column=0, columnspan=2, sticky='w', pady=8)
        ttk.Label(body, text='Approved program name or path').grid(row=2, column=0, sticky='w')
        ttk.Entry(body, textvariable=self.program, width=56).grid(row=3, column=0, columnspan=2,
                                                                 sticky='ew', pady=3)
        row = ttk.Frame(body)
        row.grid(row=4, column=0, columnspan=2, sticky='ew', pady=8)
        ttk.Button(row, text='Verify loaded program', command=self.verify).pack(side='left')
        ttk.Button(row, text='Save reviewed program', command=self.save).pack(side='left', padx=6)
        ttk.Label(body, textvariable=self.status, wraplength=570, justify='left').grid(
            row=5, column=0, columnspan=2, sticky='w')
        ttk.Label(body, text='This panel never loads an unrelated program, unlocks protective stops, '
                            'changes Remote Control mode, or commands motion. Enter the exact '
                            'path/name Dashboard reports; a bare filename is not inferred to match '
                            'a configured directory.',
                  wraplength=570).grid(row=6, column=0, columnspan=2, sticky='w', pady=8)
        self.future = None
        self.window.protocol('WM_DELETE_WINDOW', self.close)
        self.window.after(100, self.poll)

    def verify(self):
        client = self.gui.dashboard.get('loaded')
        if client is None or not client.service_is_ready():
            self.status.set('Dashboard get_loaded_program service unavailable; check the UR driver.')
            return
        from ur_dashboard_msgs.srv import GetLoadedProgram
        self.future = client.call_async(GetLoadedProgram.Request())
        self.status.set('Checking the program loaded on the robot…')

    def save(self):
        try:
            config = ProgramConfig(enabled=self.enabled.get(), program=self.program.get()).validated()
        except ValueError as error:
            self.status.set(str(error))
            return
        try:
            path = save_program_config(self.gui.config_path, config)
        except OSError as error:
            self.status.set('Save failed: ' + str(error))
            return
        self.status.set('Saved reviewed program to ' + str(path)
                        + '. Restart the managed stack only if the program identity changes.')
        if not self.gui.program.busy:
            self.gui.program_config = config
            self.gui.program.config = config

    def poll(self):
        if self.closed:
            return
        if self.future is not None and self.future.done():
            try:
                result = self.future.result()
                loaded = result.program_name if result.success else 'Unavailable'
                match = same_program(loaded, self.program.get())
                self.status.set(f'Dashboard loaded program: {loaded}. '
                                + ('Matches the configured identity.' if match else
                                   'Does not match the configured identity; review before enabling automatic start.'))
            except Exception as error:
                self.status.set('Verification failed: ' + str(error))
            self.future = None
        self.window.after(100, self.poll)

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.future is not None:
            self.future.cancel()
            self.future = None
        self.window.destroy()
