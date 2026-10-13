"""Tk-only presentation and review of observer snapshots; no ROS callbacks."""
import json
import math
import tkinter as tk
from tkinter import ttk
from copy import deepcopy
from .calibration_capture import STEPS

TITLES = ('Translation +X', 'Translation +Y', 'Translation +Z', 'Rotation +X', 'Rotation +Y', 'Rotation +Z')


class CalibrationPanel:
    def __init__(self, parent, wizard):
        self.wizard = wizard
        self.snapshot = {}
        self.generation = None
        self.last_pass = None
        self.last_results = {}
        self.last_state = None
        self.notebook = ttk.Notebook(parent)
        self.notebook.pack(fill='both', expand=True)
        panels = []
        for text in ('Controller capture', 'Review six directions', 'Tracking-loss check'):
            host = ttk.Frame(self.notebook)
            self.notebook.add(host, text=text)
            canvas = tk.Canvas(host, highlightthickness=0, height=260)
            scrollbar = ttk.Scrollbar(host, orient='vertical', command=canvas.yview)
            scrollbar.pack(side='right', fill='y')
            canvas.pack(side='left', fill='both', expand=True)
            canvas.configure(yscrollcommand=scrollbar.set)
            frame = ttk.Frame(canvas, padding=5)
            item = canvas.create_window((0, 0), window=frame, anchor='nw')
            frame.bind('<Configure>', lambda event, c=canvas: c.configure(scrollregion=c.bbox('all')))
            canvas.bind('<Configure>', lambda event, c=canvas, i=item: c.itemconfigure(i, width=event.width))
            panels.append(frame)
        capture, review, optical = panels
        self.title = tk.StringVar(value='Step 1 of 6 — Translation +X')
        ttk.Label(capture,textvariable=self.title,font=('DejaVu Sans',12,'bold')).pack(anchor='w')
        ttk.Label(capture,text='Hold still and press A. Wait for “Move”, move along the indicated physical robot axis,\nthen hold still until “release A”. Minimum: 20 mm / 10°. No mouse needed between captures.',wraplength=900).pack(anchor='w')
        self.detail = tk.StringVar(value='Begin Preview to connect the read-only capture observer.')
        self.indicator = ttk.Label(capture,textvariable=self.detail,wraplength=900)
        self.indicator.pack(anchor='w',pady=4)
        self.progress = ttk.Progressbar(capture,maximum=6)
        self.progress.pack(fill='x',pady=3)
        row = ttk.Frame(capture);row.pack(fill='x')
        ttk.Button(row,text='Retry / resume capture',command=self.retry).pack(side='left')
        self.return_step = tk.StringVar(value=TITLES[0])
        ttk.Combobox(row,textvariable=self.return_step,values=TITLES,state='readonly',width=19).pack(side='left',padx=5)
        ttk.Button(row,text='Return to this step',command=lambda:self.retry(TITLES.index(self.return_step.get()))).pack(side='left')
        ttk.Button(row,text='Cannot identify physical axes',command=self.cannot_identify).pack(side='left',padx=5)
        fallback=ttk.Frame(capture);fallback.pack(fill='x',pady=2)
        ttk.Label(fallback,text='Mouse fallback (A released):').pack(side='left')
        ttk.Button(fallback,text='Record',command=lambda:wizard.observer.command('mouse_start')).pack(side='left')
        ttk.Button(fallback,text='Finish stable endpoint',command=lambda:wizard.observer.command('mouse_finish')).pack(side='left',padx=5)
        columns=('expected','observed','magnitude','quality','result')
        self.table=ttk.Treeview(review,columns=columns,show='tree headings',height=6,selectmode='browse')
        self.table.heading('#0',text='Test');self.table.column('#0',width=130)
        for key,width in zip(columns,(90,100,100,240,70)):
            self.table.heading(key,text=key.title());self.table.column(key,width=width)
        for i,name in enumerate(STEPS):self.table.insert('', 'end',iid=name,text=TITLES[i],values=('+'+name[-1].upper(),'—','—','Not captured','Pending'))
        self.table.pack(fill='x')
        self.review_button=ttk.Button(review,text='I reviewed the six direction measurements against the physical robot frame.',command=self.review)
        self.review_button.pack(fill='x',pady=5)
        ttk.Button(review,text='Retry selected measurement',command=self.retry_selected).pack(anchor='w')
        ttk.Label(review,text='This verifies the configured mapping. Free-hand movement cannot establish absolute robot-base alignment.\nUse a trusted physical reference; review measured tool axes against the actual tool. Stopping is tested separately.',wraplength=900).pack(anchor='w')
        ttk.Label(optical,text='Separate inactive Preview test. Release A. Observe tracking loss, restore it, then assess the result.\nDirection measurements alone do not validate stopping or resolve the reported robot sway.',wraplength=900).pack(anchor='w',pady=10)
        ttk.Button(optical,text='Observe Preview tracking loss',command=self.optical_begin).pack(anchor='w',pady=4)
        ttk.Button(optical,text='I verified Preview tracking loss / recovery',command=self.optical_review).pack(anchor='w')

    def retry(self, step=None):
        w=self.wizard
        if w.baseline is None or w.baseline_parameters != w.gui.last_status.get('effective_parameters'):
            w.message.set('Begin Preview after saving the current configuration.');return
        w.capture_enabled=True
        w.observer.command('retry',step)
        # Invalidate review immediately, before the worker acknowledges the retry.
        if step is None: step=self.snapshot.get('step',0)
        w.checks.results.pop(STEPS[step],None)
        for name in STEPS:
            if name in w.checks.results:w.checks.results[name]['physical_pass']=False
        self.notebook.select(0)

    def cannot_identify(self):
        self.wizard.cancel_capture()
        for name in STEPS:
            if name in self.wizard.checks.results:
                self.wizard.checks.results[name]['physical_pass'] = False
        self.wizard.message.set('Mapping cannot be verified confidently without known physical robot-base directions. Establish a trusted reference first.')

    def retry_selected(self):
        selected=self.table.selection()
        if selected:self.retry(STEPS.index(selected[0]))

    def optical_begin(self):
        self.wizard.cancel_capture()
        self.wizard.selection.set('Cover tracking, then restore (Preview)')
        self.wizard.begin_check()

    def optical_review(self):
        self.wizard.selection.set('Cover tracking, then restore (Preview)')
        self.wizard.attest()

    def review(self):
        w=self.wizard
        try:
            w.guard()
            if self.snapshot.get('session_id') != w.gui.session or self.snapshot.get('parameters') != w.baseline_parameters:
                raise ValueError('Session or configuration changed; repeat direction measurements.')
            if not all(w.checks.results.get(k,{}).get('observed') is True and
                       w.checks.results[k].get('details',{}).get('passed') is True for k in STEPS):
                raise ValueError('Six genuinely passed measurements are required before review.')
            if w.gui.last_status.get('a_held') is not False:
                raise ValueError('Release A before confirming the review.')
            for name in STEPS:w.checks.attest(name)
            w.message.set('Six directions reviewed. Complete the separate Preview tracking-loss check next.')
            self.notebook.select(2)
        except ValueError as error:w.message.set(str(error))

    def update(self, snapshot):
        self.snapshot=snapshot
        w=self.wizard
        if (snapshot.get('session_id') != w.gui.session or
                snapshot.get('parameters') != w.baseline_parameters):return
        reports=snapshot['results']
        if snapshot['generation'] != self.generation:
            for name in STEPS:w.checks.results.pop(name,None)
            self.generation=snapshot['generation']
        for name in STEPS:
            if name not in reports:
                w.checks.results.pop(name,None)
            elif w.checks.results.get(name,{}).get('details') != reports[name]:
                w.checks.results[name]=dict(observed=True,physical_pass=False,details=deepcopy(reports[name]))
        step=snapshot['step'];state=snapshot['state']
        self.title.set(f'Step {step+1} of 6 — {TITLES[step]}')
        report=snapshot.get('report') or {}
        progress=snapshot.get('progress') or {}
        current_report = report if report.get('step') == STEPS[step] else {}
        magnitude=progress.get('magnitude',current_report.get('magnitude',0))
        units='mm' if step<3 else '°'
        value=magnitude*(1000 if step<3 else 180/math.pi)
        self.detail.set(f"A: {'held' if snapshot['button'] else 'released'} | {state} | Samples: {snapshot['samples']}\n"
            f"{snapshot['reason']}\nMeasured: {value:.1f} {units}; dominant axis: {progress.get('dominant_axis',current_report.get('dominant_axis','—'))} | "
            f"Accepted / received: {snapshot['accepted']} / {snapshot['received']}; rejected: {snapshot['rejected']}")
        self.indicator.configure(foreground='#15803D' if state in ('PASSED','REVIEW') else '#B45309' if state=='FAILED_RETRY' else '#1F2937')
        self.progress['value']=len(reports)
        for i,name in enumerate(STEPS):
            r=reports.get(name) or (report if report.get('step')==name else {})
            magnitude=r.get('magnitude',0)*(1000 if i<3 else 180/math.pi)
            jitter = (f"{max(r.get('position_jitter_m',[0]))*1000:.1f} mm" if i<3 else
                      f"{math.degrees(max(r.get('orientation_jitter_rad',[0]))):.1f}°")
            quality=(f"{r.get('accepted_samples',0)} samples; {r.get('source_hz',0):.0f} Hz; jitter {jitter}") if r else 'Not captured'
            self.table.item(name,values=('+'+name[-1].upper(),r.get('direction_sign','')+r.get('dominant_axis','—'),
                f"{magnitude:.1f} {'mm' if i<3 else '°'}" if r else '—',quality,'Pass' if r.get('passed') else 'Retry' if r else 'Pending'))
        self.review_button.state(['!disabled'] if len(reports)==6 else ['disabled'])
        pass_id=json.dumps(reports,sort_keys=True)
        if reports and pass_id != self.last_pass:
            self.last_pass=pass_id
            w.window.bell()
        if state=='REVIEW' and self.last_state != state:self.notebook.select(1)
        self.last_results=deepcopy(reports)
        self.last_state=state
