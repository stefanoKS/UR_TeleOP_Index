"""Tk setup adapter: requests existing services; records observed, supervised checks."""
import json
import time
import tkinter as tk
from tkinter import ttk, messagebox
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from .engineering import CHECKS, identity, save_setup, migrate_legacy, invalidate_setup
from .preferences import save_settings
from .setup_checks import SetupChecks, finite_vector
from scipy.spatial.transform import Rotation
import numpy as np

LABELS = dict(mapping_x='Move hand along robot +X', mapping_y='Move hand along robot +Y',
    mapping_z='Move hand along robot +Z', orientation_x='Rotate hand around robot +X',
    orientation_y='Rotate hand around robot +Y', orientation_z='Rotate hand around robot +Z',
    preview_loss='Cover tracking, then restore (Preview)', a_stop='A release stop (Live)',
    tracking_stop='Tracking-loss stop (Live)', fresh_clutch='Held-A recovery / fresh press (Live)',
    software_stop='STOP TELEOP (Live)')


class SetupWizard:
    def __init__(self, gui):
        self.gui = gui
        self.checks = SetupChecks()
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='setup-files')
        self.future = None
        self.baseline = None
        self.baseline_parameters = None
        self.closed = False
        self.capture_enabled = False
        self.awaiting_preview_session = None
        from .calibration_observer import CalibrationObserver
        self.observer = CalibrationObserver(context=gui.node.context)
        self.window = tk.Toplevel(gui.root)
        self.window.title('Initial Setup — supervised verification')
        self.window.geometry('1000x680')
        self.window.minsize(900, 650)
        self.window.protocol('WM_DELETE_WINDOW', self.close)
        body = ttk.Frame(self.window, padding=14)
        body.pack(fill='both', expand=True)
        ttk.Label(body, text='Initial Setup', font=('DejaVu Sans', 18, 'bold')).pack(anchor='w')
        ttk.Label(body, text='Configure once. Live monitoring remains active on every launch. No motion starts automatically.').pack(anchor='w', pady=5)
        self.status = tk.StringVar(value='Waiting for backend connection…')
        ttk.Label(body, textvariable=self.status, wraplength=950).pack(anchor='w')
        # Reserve the stop/footer before allocating expanding tab content.
        ttk.Button(body, text='STOP TELEOP', style='Danger.TButton', command=gui.stop_teleop).pack(side='bottom', fill='x')
        self.message = tk.StringVar(value='Begin Preview to capture the installation identity and perform the checks.')
        ttk.Label(body, textvariable=self.message, wraplength=950).pack(side='bottom', anchor='w', pady=5)
        self.tabs = tabs = ttk.Notebook(body)
        tabs.pack(fill='both', expand=True, pady=8)
        preview, live, save = [ttk.Frame(tabs, padding=12) for _ in range(3)]
        for frame, title in ((preview, '1  Connection & mapping'), (live, '2  Supervised stop tests'), (save, '3  Save Setup')):
            tabs.add(frame, text=title)
        ttk.Label(preview, text='Verify the configured robot-base mapping using known physical axes. No automatic RPY estimation.\nIf the physical axes are uncertain, stop and establish a trusted reference. Advanced tuning retains manual RPY.', wraplength=910).pack(anchor='w')
        self.axes_canvas = canvas = tk.Canvas(preview, height=85, bg='white', highlightthickness=0)
        canvas.pack(fill='x', pady=5)
        self.telemetry = tk.StringVar(value='Measured tool TF and controller pose: waiting')

        ttk.Button(preview, text='Begin / restart supervised Preview', command=self.begin_preview).pack(anchor='w', pady=5)
        self.selection = tk.StringVar(value=LABELS['preview_loss'])
        from .calibration_panel import CalibrationPanel
        self.calibration_panel = CalibrationPanel(preview, self)
        ttk.Label(live, text='Authorized commissioning only. Review payload, TCP, mounting, collision model, workspace and the\nACTUAL motion limits below. Select site-approved test limits in the dashboard and save/restart Preview\nbefore testing. Keep the pendant available and clear the workspace. Each test requires observed motion.\n\nAfter Preview checks: enter supervised Live, apply Preview results, then press Start supervised tests.\nSelect a test and click Observe test BEFORE holding A and moving slowly. For A stop: release A.\nFor tracking loss: keep A held while covering tracking; wait for standstill, restore tracking while held,\nthen release A and press again. Verify the actual stopping distance and recovery before accepting.\nFor software stop: while moving, press the always-available STOP TELEOP button.\nThe observer requires zero upstream demand and measured standstill for 0.5 s; this is not a stopping-distance limit.', wraplength=910).pack(anchor='w')
        row = ttk.Frame(live); row.pack(fill='x', pady=10)
        for title, action in (('Enter supervised Live', self.begin_live), ('Apply Preview results', self.apply_preview),
                              ('Start supervised tests', self.start_live)):
            ttk.Button(row, text=title, command=action).pack(side='left', padx=3)
        self.live_selection = tk.StringVar(value=LABELS['a_stop'])
        ttk.Combobox(live, state='readonly', textvariable=self.live_selection,
                     values=[LABELS[k] for k in CHECKS[7:]], width=28).pack(anchor='w')
        ttk.Button(live, text='Observe selected test', command=lambda:self.begin_check(live=True)).pack(anchor='w', pady=5)
        ttk.Button(live, text='I verified the physical stop / recovery', command=lambda:self.attest(live=True)).pack(anchor='w')
        self.results_text = tk.StringVar()
        self.results_remaining = tk.StringVar()
        checks = ttk.Frame(save); checks.pack(fill='x')
        ttk.Label(checks, textvariable=self.results_text, wraplength=430).pack(side='left', anchor='n')
        ttk.Label(checks, textvariable=self.results_remaining, wraplength=430).pack(side='left', anchor='n', padx=14)
        ttk.Label(save, text='Record who performed the supervised checks, tool/mounting identity, and the observed physical\nstopping/recovery results. Confirm TCP/payload, workspace, collision model and displayed limits\nare appropriate. This is your physical assessment, not an automatic certification.', wraplength=910).pack(anchor='w', pady=10)
        self.assessment = tk.Text(save, height=3, wrap='word')
        self.assessment.pack(fill='x')
        row = ttk.Frame(save); row.pack(fill='x', pady=8)
        self.save_button = ttk.Button(row, text='Save Setup', command=self.save)
        self.save_button.pack(side='left')
        ttk.Button(row, text='Import existing valid V4 record', command=self.migrate).pack(side='left', padx=8)
        ttk.Label(save, text='Save while stopped, disarmed and A released. The app returns to routine Live, disabled.\nFuture launches skip this wizard while the saved configuration remains valid.\nReopen Settings → Recalibrate / Initial Setup after physical tool or Lighthouse changes.', wraplength=910).pack(anchor='w')
        self.observe(gui.last_status, gui.last_status_at)

    def guard(self):
        d = self.gui.last_status
        if self.gui.stale():
            raise ValueError('Waiting for a fresh backend connection.')
        if self.future is not None:
            raise ValueError('Wait for setup file verification to finish.')
        return d

    def invoke(self, action):
        try:
            action()
        except (ValueError, TypeError, KeyError) as error:
            self.message.set(str(error))

    def submit(self, purpose, fn, *args):
        if self.future is not None:
            raise ValueError('A setup operation is already running.')
        self.future = (purpose, self.pool.submit(fn, *args))
        self.message.set('Checking configuration… STOP remains available.')

    def begin_preview(self):
        def begin():
            d = self.guard()
            if self.gui.process_worker is None:
                raise ValueError('Open the managed application to run supervised setup.')
            self.gui.stop_teleop()
            self.checks = SetupChecks()
            self.baseline = None
            self.baseline_parameters = deepcopy(d['effective_parameters'])
            c = json.loads(d['setup_context'])
            settings = dict(c['settings'], teleop=dict(c['settings']['teleop'], **self.baseline_parameters))
            def begin_files():
                baseline = identity(settings, c['root'], c['tracking'])
                invalidate_setup(d['setup_context'])
                return baseline
            self.submit('preview', begin_files)
        self.invoke(begin)

    def preview_complete(self):
        return all(self.checks.results.get(k, {}).get('physical_pass') is True for k in CHECKS[:7])

    def begin_live(self):
        def begin():
            self.guard()
            if not self.preview_complete() or self.baseline is None:
                raise ValueError('Complete and physically verify all seven Preview checks first.')
            if not messagebox.askyesno('Supervised Live commissioning',
                'Are you authorized to perform physical robot tests? Have you reviewed the displayed limits, '
                'TCP/payload and clear workspace, with the pendant available?\n\nLive starts disarmed. '
                'Motion still requires Start supervised tests and a fresh A press.', parent=self.window):
                return
            self.cancel_capture()
            self.gui.stack_action('setup_live')
        self.invoke(begin)

    def begin_check(self, live=False):
        def begin():
            d = self.guard()
            if self.baseline is None or self.baseline_parameters != d.get('effective_parameters'):
                raise ValueError('Begin Preview again after settings changes.')
            name = self.selected(live)
            if name == 'fresh_clutch':
                raise ValueError('This check follows tracking_stop automatically: restore while A held, release, then press.')
            self.checks.begin(name, d)
            self.message.set(LABELS[name]+' — observing; no motion commanded by the wizard.')
        self.invoke(begin)

    def capture(self):
        self.invoke(lambda:self.checks.capture(self.guard()))

    def selected(self, live):
        label = self.live_selection.get() if live else self.selection.get()
        return next(k for k, v in LABELS.items() if v == label)

    def attest(self, live=False):
        def accept():
            self.guard()
            self.checks.attest(self.selected(live))
            self.message.set('Physical result recorded. Continue with the next check.')
        self.invoke(accept)

    def apply_preview(self):
        def apply():
            d = self.guard()
            if not self.preview_complete() or d.get('engineering_required') is not False:
                raise ValueError('Complete Preview checks and enter supervised Live first.')
            if self.baseline_parameters != d.get('effective_parameters') or self.gui.parameters.changed():
                raise ValueError('Settings changed. Save settings and repeat Preview verification.')
            self.gui.alignment.set(True)
            self.gui.optical.set(True)
            self.gui.apply()
            self.message.set('Preview results submitted; waiting for backend confirmation.')
        self.invoke(apply)

    def start_live(self):
        def start():
            d = self.guard()
            if (not self.preview_complete() or d.get('engineering_required') is not False
                    or d.get('alignment_verified') is not True or d.get('optical_loss_test_verified') is not True):
                raise ValueError('Apply Preview results and wait for backend confirmation first.')
            self.gui.start_teleop(supervised=True)
        self.invoke(start)

    def stopped(self):
        d = self.guard()
        if any(d.get(k) is not False for k in ('armed', 'enable_requested', 'a_held')) or d.get('stationary') is not True:
            raise ValueError('Press STOP TELEOP, release A and wait for confirmed standstill before saving.')
        if d.get('observations_healthy') is not True or d.get('tracking_valid') is not True:
            raise ValueError('Restore fresh tracking, joints and tool TF before saving.')
        if self.gui.parameters.pending or self.gui.parameters.changed() or self.gui.parameters.restart:
            raise ValueError('Resolve pending settings and restart changes before saving setup.')
        return d

    def save(self):
        def save():
            d = self.stopped()
            if self.baseline_parameters != d['effective_parameters']:
                raise ValueError('Settings changed; repeat Preview and physical verification.')
            assessment = self.assessment.get('1.0', 'end').strip()
            if len(assessment) < 20:
                raise ValueError('Describe the authorized physical assessment, tool identity and observed results.')
            if not all(self.checks.results.get(k, {}).get('physical_pass') is True for k in CHECKS):
                raise ValueError('Complete all observed checks and physical assessments before saving.')
            encoded, params, baseline, results = d['setup_context'], deepcopy(d['effective_parameters']), self.baseline, deepcopy(self.checks.results)
            def write():
                # Existing validated preference writer; no enable/confirmation flags.
                save_settings(self.gui.config_path, params)
                return save_setup(encoded, params, baseline, results, assessment)
            self.gui.stop_teleop()
            self.submit('save', write)
        self.invoke(save)

    def migrate(self):
        def migrate():
            d = self.stopped()
            self.submit('save', migrate_legacy, d['setup_context'], deepcopy(d['effective_parameters']))
        self.invoke(migrate)

    def observe(self, d, now):
        if self.closed:
            return
        self.checks.update(d, now)
        if self.awaiting_preview_session is not None and d.get('session_id') != self.awaiting_preview_session:
            self.awaiting_preview_session = None
            self.capture_enabled = True

    def cancel_capture(self):
        self.awaiting_preview_session = None
        self.capture_enabled = False
        self.observer.configure(None, False)
        self.observer.command('cancel')
        panel = getattr(self, 'calibration_panel', None)
        if panel and panel.snapshot:
            panel.update(dict(panel.snapshot, state='CANCELLED',
                reason='Capture stopped. Release A and use Retry / resume to continue.',
                samples=0, progress=None, isolated=False))

    def draw_axes(self, v, d):
        canvas = self.axes_canvas
        canvas.delete('all')
        matrices = [np.eye(3), None, None]
        if not self.gui.stale():
            q = v.get('measured_tcp_orientation')
            raw = v.get('raw_pose')
            try:
                if finite_vector(q, 4) and d.get('setup_feedback_fresh') is True:
                    matrices[1] = Rotation.from_quat(q).as_matrix()
                if finite_vector(raw, 7) and d.get('tracking_valid') is True:
                    mapping = Rotation.from_euler('xyz', d['effective_parameters']['tracking_to_robot_rpy_deg'], degrees=True)
                    matrices[2] = (mapping*Rotation.from_quat(raw[3:])).as_matrix()
            except (ValueError, KeyError):
                pass
        for i, (title, matrix) in enumerate(zip(('Robot base axes', 'Measured tool axes', 'Mapped controller axes'), matrices)):
            x, y = 140+i*300, 65
            canvas.create_text(x, 12, text=title)
            if matrix is None:
                canvas.create_text(x, 55, text='Waiting for fresh pose / TF', fill='#64748B')
                continue
            for j, (axis, color) in enumerate((('X', '#DC2626'), ('Y', '#15803D'), ('Z', '#2563EB'))):
                delta = matrix[:, j]*40
                end = (x+delta[0]-.6*delta[1], y-.4*delta[1]-delta[2])
                canvas.create_line(x, y, *end, fill=color, width=2, arrow=tk.LAST)
                canvas.create_text(end[0]+10, end[1]-5, text='+'+axis, fill=color)

    def poll(self):
        if self.closed:
            return
        d = self.gui.last_status
        stale = self.gui.stale()
        token, managed_preview = self.gui.process_worker.calibration_snapshot() if self.gui.process_worker else (None, False)
        permitted = (self.capture_enabled and managed_preview and not stale and self.baseline is not None
            and self.baseline_parameters == d.get('effective_parameters') and not self.gui.startup.requested
            and not self.gui.startup_request and not self.gui.pending and not self.gui.parameters.pending)
        self.observer.configure(token, permitted)
        snapshot = self.observer.latest()
        if snapshot is not None and self.capture_enabled:self.calibration_panel.update(snapshot)
        if self.observer.failure:
            self.gui.setup_technical = self.observer.failure
            self.message.set('Calibration observer unavailable. Close and reopen Setup; details in Advanced Diagnostics.')
        if stale:
            self.checks.cancel()
        self.status.set('Backend unavailable — checks paused' if stale else
            f"Robot: {'connected' if d.get('robot_ready') else 'waiting / inspect pendant'}  |  "
            f"VR 6-DoF: {'valid' if d.get('tracking_valid') else 'waiting / invalid'}  |  "
            f"Lighthouses: {d.get('active_station_count', '?')} (either calibrated station is sufficient)\n"+
            self.gui.active_limit.get())
        v = d.get('velocity_diagnostics', {})
        self.draw_axes(v, d)
        self.telemetry.set(f"Controller pose (lighthouse): {v.get('raw_pose', 'unavailable')}\n"
            f"Measured {d.get('base_frame', '?')} → {d.get('tool_frame', '?')}: {v.get('measured_tcp_position', 'unavailable')}\n"
            f"Tool quaternion x,y,z,w: {v.get('measured_tcp_orientation', 'unavailable')}")
        rows = [('✓ Verified' if self.checks.results.get(k, {}).get('physical_pass') else
            'Observed — physical review needed' if k in self.checks.results else 'Pending')+' — '+LABELS[k] for k in CHECKS]
        self.results_text.set('\n'.join(rows[:6]))
        self.results_remaining.set('\n'.join(rows[6:]))
        self.save_button.state(['!disabled'] if not stale and self.future is None and
            all(self.checks.results.get(k, {}).get('physical_pass') for k in CHECKS) else ['disabled'])
        if self.checks.test:
            self.message.set(LABELS.get(self.checks.test, self.checks.test)+' — '+{'capture':'Move along the selected axis, then Capture end', 'loss':'Cover tracking', 'restore':'Restore tracking',
                'move':'Hold A and make a small supervised movement', 'trigger':'Perform the selected stop',
                'stop':'Waiting for measured standstill', 'held_recovery':'Restore tracking with A held',
                'release':'Release A', 'press':'Press A again and move slowly'}.get(self.checks.phase, self.checks.phase))
        if self.future is not None and self.future[1].done():
            purpose, future = self.future
            self.future = None
            try:
                result = future.result()
                if purpose == 'preview':
                    self.baseline = result
                    self.awaiting_preview_session = self.gui.session
                    self.gui.stack_action('setup_preview')
                    self.message.set('Preview starting. Wait for fresh robot, VR and tool TF, then perform each check.')
                else:
                    self.gui.notify('Setup saved. Routine Live will start disarmed; press START TELEOP when ready.')
                    self.gui.stack_action('routine_live')
                    self.close(stop=False)
            except Exception as error:
                self.gui.setup_technical = repr(error) + ('; caused by '+repr(error.__cause__) if error.__cause__ else '')
                self.message.set(str(error) if type(error) is ValueError else
                    'Setup could not be saved or checked. Check configuration access in Advanced Diagnostics.')

    def close(self, stop=True):
        if self.closed:
            return
        self.closed = True
        self.observer.close()
        if stop:
            self.gui.stop_teleop()
            # Never leave an abandoned wizard in the legacy session path.
            if self.gui.last_status.get('engineering_required') is False and self.gui.process_worker:
                self.gui.stack_action('routine_live')
        self.pool.shutdown(wait=False, cancel_futures=True)
        self.window.destroy()
        self.gui.setup_wizard = None
