"""Responsive single-window Tk dashboard. It requests supervision; it never commands motion."""
import json
import math
import os
import queue
import time
import tkinter as tk
from collections import deque
from pathlib import Path
from tkinter import ttk

import rclpy
from controller_manager_msgs.srv import SwitchController
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from .preferences import (DEFAULTS, PARAMETER_REGISTRY, PARAMETERS_BY_KEY,
                          save_settings, stored_value, validate, preference_path)
from .dashboard_state import (ParameterState, Notifications, equivalent, boolean, display_text,
                              vector, vector_text, station_text, motion_text)
from .dashboard_process import ManagerWorker
from .startup import Startup

COLORS = {'navy': '#17263D', 'blue': '#2563EB', 'green': '#15803D', 'amber': '#D97706',
          'red': '#DC2626', 'bg': '#F3F6FA', 'card': '#FFFFFF', 'text': '#1F2937', 'muted': '#64748B'}


class OperatorGUI:
    """Dashboard using named parameters and async ROS requests only."""
    def __init__(self, manager=None):
        if os.environ.get('ROS_DOMAIN_ID') != '43': raise RuntimeError('GUI requires ROS_DOMAIN_ID=43')
        self.node, self.manager = Node('teleop_operator_gui', namespace='vr_real'), manager
        self.config_path = Path(getattr(manager, 'config', Path.home()/'Documents/Teleop/ur_teleop/config/real_teleop.yaml'))
        self.closed = self.shutdown_requested = False
        self.pending, self.request_meta = [], {}
        self.parameters = ParameterState()
        self.notifications = Notifications()
        self.startup = Startup()
        self.startup_request = None
        self.retiring_startup = False
        self.uncertain_enable_session = None
        self.uncertain_servo_session = None
        self.heartbeat_allowed = False
        self.session = None
        self.generation = 0
        # Backend session identity: distinct IDs and change rate expose duplicate backends.
        self.session_ids = []
        self.session_changes = 0
        self.session_events = deque(maxlen=20)
        self.session_restart = False
        self.confirmation_rejected = False
        self.confirmation_pending = None
        self.settings_result = ''
        self.process_worker = None
        self.setup_wizard = None
        self.setup_prompted = False
        self.setup_technical = ''
        from .sway_recording import Recorder
        from .preferences import installation_root
        directory = getattr(manager, 'directory', None)
        self.recorder = Recorder(installation_root(self.config_path), self.config_path,
                                 Path(directory)/'effective_config.yaml' if directory else None)
        try:
            saved = json.loads(preference_path(self.config_path).read_text())
            if saved.get('version') == 1:
                self.parameters.saved = validate(saved['parameters'])
                if 'max_joint_speed_rad_s' in self.parameters.saved:
                    self.parameters.restart = {'max_joint_speed_rad_s': self.parameters.saved['max_joint_speed_rad_s']}
        except (OSError, ValueError, TypeError, KeyError):
            pass
        self.running_values, self.last_status = {}, {}
        self.native_reference = {}
        self.native_reference_at = -float('inf')
        self.last_status_at = self.heartbeat_at = -float('inf')
        self.camera_at, self.camera_text, self.camera_window = -float('inf'), 'Camera not connected', None
        self.fields, self.field_rows, self.field_widgets, self._loading_fields = {}, {}, {}, False
        self.root = tk.Tk(); self.root.title('UR10e Teleoperation Dashboard'); self.root.geometry('1440x900'); self.root.minsize(1280, 700)
        self.root.configure(bg=COLORS['bg']); self._style(); self._build(); self._interfaces()
        self.process_worker = ManagerWorker(manager) if manager else None
        self.root.protocol('WM_DELETE_WINDOW', self.close); self.root.after(20, self.poll)

    def _style(self):
        s = ttk.Style(); s.theme_use('clam')
        s.configure('App.TFrame', background=COLORS['bg']); s.configure('Card.TFrame', background=COLORS['card'], relief='solid', borderwidth=1)
        s.configure('Card.TLabel', background=COLORS['card'], foreground=COLORS['text'], font=('DejaVu Sans', 10))
        s.configure('Muted.TLabel', background=COLORS['card'], foreground=COLORS['muted'], font=('DejaVu Sans', 9))
        s.configure('Section.TLabel', background=COLORS['card'], foreground=COLORS['text'], font=('DejaVu Sans', 12, 'bold'))
        s.configure('Primary.TButton', foreground='white', background=COLORS['blue'], font=('DejaVu Sans', 10, 'bold'), padding=(7, 5))
        s.configure('Danger.TButton', foreground='white', background=COLORS['red'], font=('DejaVu Sans', 10, 'bold'), padding=(7, 5))
        s.configure('Invalid.TEntry', fieldbackground='#FEE2E2')
        s.map('TButton', foreground=[('disabled', '#7B8491')], background=[('disabled', '#E2E8F0')])
        s.configure('TButton', font=('DejaVu Sans', 10), padding=(5, 3))

    def card(self, parent, title):
        f = ttk.Frame(parent, style='Card.TFrame', padding=7); ttk.Label(f, text=title, style='Section.TLabel').pack(anchor='w', pady=(0, 7)); return f

    def _build(self):
        menu = tk.Menu(self.root)
        settings_menu = tk.Menu(menu, tearoff=False)
        settings_menu.add_command(label='Advanced tuning', command=lambda: (self.advanced_open.set(True), self.toggle_advanced()))
        menu.add_cascade(label='Settings', menu=settings_menu)
        self.root.configure(menu=menu)
        self.root.grid_rowconfigure(2, weight=1); self.root.grid_columnconfigure(0, weight=1)
        header = tk.Frame(self.root, bg=COLORS['navy'], height=64); header.grid(row=0, column=0, sticky='ew'); header.grid_propagate(False); header.grid_columnconfigure(1, weight=1)
        tk.Label(header, text='UR10e TELEOPERATION', bg=COLORS['navy'], fg='white', font=('DejaVu Sans', 19, 'bold')).grid(row=0, column=0, padx=20, pady=12)
        self.header_mode, self.header_state = tk.StringVar(value='DISCONNECTED'), tk.StringVar(value='WAITING FOR GATE')
        tk.Label(header, textvariable=self.header_mode, bg=COLORS['navy'], fg='#BFDBFE', font=('DejaVu Sans', 11, 'bold')).grid(row=0, column=1, sticky='e', padx=12)
        self.header_badge = tk.Label(header, textvariable=self.header_state, bg=COLORS['navy'], fg=COLORS['muted'], font=('DejaVu Sans', 11, 'bold'))
        self.header_badge.grid(row=0, column=2, sticky='e', padx=20)
        # Prominent startup banner: current stage, blocker and any previous failure.
        self.banner = tk.Frame(self.root, bg=COLORS['amber'])
        self.banner.grid(row=1, column=0, sticky='ew')
        self.startup_stage = tk.StringVar(value='DISARMED — PRESS START TELEOP')
        self.startup_detail = tk.StringVar(value='')
        self.banner_label = tk.Label(self.banner, textvariable=self.startup_stage, bg=COLORS['amber'],
                                     fg='white', font=('DejaVu Sans', 13, 'bold'), anchor='w')
        self.banner_label.pack(side='left', padx=15, pady=5)
        self.banner_detail = tk.Label(self.banner, textvariable=self.startup_detail, bg=COLORS['amber'],
                                      fg='white', font=('DejaVu Sans', 10), anchor='w', justify='left',
                                      wraplength=1000)
        self.banner_detail.pack(side='left', fill='x', expand=True, padx=8)
        content = ttk.Frame(self.root, style='App.TFrame', padding=12); content.grid(row=2, column=0, sticky='nsew')
        for c, weight, minimum in ((0,25,285),(1,35,365),(2,40,490)): content.grid_columnconfigure(c, weight=weight, minsize=minimum)
        content.grid_rowconfigure(0, weight=1)
        self.operations(content).grid(row=0,column=0,sticky='nsew',padx=(0,8)); self.monitoring(content).grid(row=0,column=1,sticky='nsew',padx=4); self.tuning(content).grid(row=0,column=2,sticky='nsew',padx=(8,0))
        strip = tk.Frame(self.root, bg='#E8EEF6'); strip.grid(row=3,column=0,sticky='ew'); strip.grid_columnconfigure(0,weight=1)
        self.message = tk.StringVar(value='')
        self.live_message = tk.StringVar(value='Waiting for a commissioning backend.')
        tk.Label(strip,textvariable=self.message,bg='#E8EEF6',fg=COLORS['text'],anchor='w',wraplength=1050,font=('DejaVu Sans',10)).grid(row=0,column=0,sticky='ew',padx=15,pady=7)
        ttk.Button(strip, text='Dismiss', command=self.dismiss_notification).grid(row=0,column=1)
        ttk.Button(strip, text='Action history', command=self.show_history).grid(row=0,column=2)
        tk.Label(strip,textvariable=self.live_message,bg='#E8EEF6',anchor='w',wraplength=1180).grid(row=1,column=0,columnspan=3,sticky='ew',padx=15)
        self.blockers = tk.Text(strip,height=1,wrap='word',font=('DejaVu Sans',9),relief='flat',bg='#E8EEF6'); self.blockers.grid(row=2,column=0,columnspan=3,sticky='ew',padx=12,pady=(0,7)); self.blockers.configure(state='disabled');self.blockers.grid_remove()

    def operations(self, parent):
        outer=ttk.Frame(parent,style='App.TFrame')
        motion=self.card(outer,'Teleoperation'); motion.pack(fill='x',pady=(0,8))
        self.enable_button=ttk.Button(motion,text='START TELEOP',style='Primary.TButton',command=self.start_teleop)
        self.disarm_button=ttk.Button(motion,text='STOP TELEOP',style='Danger.TButton',command=self.stop_teleop)
        for button in (self.enable_button,self.disarm_button):button.pack(fill='x',pady=4)
        self.stop_notice=ttk.Label(motion,text='Software stop; not an emergency stop. Use the pendant for emergencies.',style='Muted.TLabel',wraplength=270)
        self.stop_notice.pack(anchor='w',pady=4)
        self.stack_status=tk.StringVar(value='Connecting — motion disarmed')
        ttk.Label(motion,textvariable=self.stack_status,style='Muted.TLabel',wraplength=270).pack(anchor='w')
        ready=self.card(outer,'Current status');ready.pack(fill='x',pady=(0,8));self.readiness={}
        for key in ('robot','tracking'):
            value=tk.StringVar(value='Waiting')
            widget=ttk.Label(ready,textvariable=value,style='Card.TLabel',wraplength=270)
            widget.pack(anchor='w',pady=5);self.readiness[key]=(value,widget)
        self.engineering_open=tk.BooleanVar(value=False)
        ttk.Checkbutton(outer,text='Engineering / diagnostics',variable=self.engineering_open,
                        command=self.toggle_engineering).pack(anchor='w')
        self.engineering_panel=self.card(outer,'Engineering tools')
        ttk.Button(self.engineering_panel,text='Advanced diagnostics / sway recording',command=self.show_history).pack(fill='x')
        self.record_status = tk.StringVar(value='Recording idle — independent of motion enable')
        ttk.Label(self.engineering_panel,text='Authorized commissioning personnel only. Review physical configuration before operating a changed workcell.',style='Muted.TLabel',wraplength=270).pack(anchor='w')
        buttons=ttk.Frame(self.engineering_panel,style='Card.TFrame');buttons.pack(fill='x')
        buttons.columnconfigure((0,1),weight=1)
        for i,(label,action,mode) in enumerate([('Preview','start',False),('Live stack','start',True),('Restart ROS','restart',None),('Stop ROS','stop',None)]):
            ttk.Button(buttons,text=label,command=lambda a=action,m=mode:self.stack_action(a,m)).grid(row=i//2,column=i%2,sticky='ew')
        ttk.Button(self.engineering_panel, text='Optional commissioning observations', command=self.open_setup).pack(fill='x')
        self.servo_button=ttk.Button(self.engineering_panel,text='Start Servo (engineering)',command=lambda:self.call('servo'))
        self.prepare_button=ttk.Button(self.engineering_panel,text='Prepare controller (engineering)',command=self.prepare_controller)
        self.servo_button.pack(fill='x');self.prepare_button.pack(fill='x')
        # Legacy Preview verification readback remains available to engineers.
        for key in ('controller','joints','pose','optical','alignment','heartbeat','confirmations'):
            value=tk.StringVar(value='Waiting');widget=ttk.Label(self.engineering_panel,textvariable=value,style='Muted.TLabel',wraplength=270)
            self.readiness[key]=(value,widget)
        return outer

    def start_recording(self):
        if self.recorder.process is not None:
            return
        try:
            directory = getattr(self.manager, 'directory', None)
            self.recorder.snapshot = Path(directory)/'effective_config.yaml' if directory else None
            self.recorder.start()
        except (OSError, ValueError) as error:
            self.notify('Recording failed: '+str(error), True)

    def toggle_recording(self):
        if self.recorder.process is None:
            self.start_recording()
        else:
            self.recorder.stop()

    def toggle_engineering(self):
        if self.engineering_open.get():self.engineering_panel.pack(fill='x',pady=4)
        else:self.engineering_panel.pack_forget()

    def monitoring(self,parent):
        outer=ttk.Frame(parent,style='App.TFrame'); camera=self.card(outer,'Camera / visualization'); camera.pack(fill='x',pady=(0,8))
        ttk.Label(camera,text='Optional camera; separate from motion supervision.',style='Muted.TLabel',wraplength=350).pack(anchor='w'); ttk.Button(camera,text='Open D405 camera',command=self.open_camera).pack(anchor='w',pady=(7,2)); self.camera_status=tk.StringVar(value='D405 unavailable / no recent status (teleop unaffected)'); ttk.Label(camera,textvariable=self.camera_status,style='Muted.TLabel',wraplength=350).pack(anchor='w')
        if self.manager: ttk.Button(camera,text='Open RViz',command=self.open_rviz).pack(anchor='w',pady=(5,0))
        live=self.card(outer,'Live robot diagnostics'); live.pack(fill='x',pady=(0,8)); self.diagnostics=tk.StringVar(value='Waiting for telemetry…'); ttk.Label(live,textvariable=self.diagnostics,style='Card.TLabel',justify='left',wraplength=350).pack(anchor='w')
        lighthouse=self.card(outer,'Lighthouse tracking'); lighthouse.pack(fill='both',expand=True); self.lighthouse=tk.StringVar(value='Lighthouse 0: waiting\nLighthouse 1: waiting\nValid pose: waiting\nLast rejection: waiting'); ttk.Label(lighthouse,textvariable=self.lighthouse,style='Card.TLabel',justify='left',wraplength=350).pack(anchor='w'); self.motion=tk.StringVar(value='Hold A in Preview to inspect clutch displacement.'); ttk.Label(lighthouse,textvariable=self.motion,style='Muted.TLabel',wraplength=350).pack(anchor='w',pady=(10,0)); return outer

    def tuning(self,parent):
        outer=ttk.Frame(parent,style='App.TFrame'); quick=self.card(outer,'Quick tuning'); quick.pack(fill='x',pady=(0,8)); self.add_fields(quick,'quick')
        self.unsaved=tk.StringVar(value='Running values load when backend connects.'); ttk.Label(quick,textvariable=self.unsaved,style='Muted.TLabel',wraplength=305).pack(anchor='w',pady=(7,4))
        self.apply_button=ttk.Button(quick,text='Apply changes',style='Primary.TButton',command=self.apply); self.save_button=ttk.Button(quick,text='Apply + save',command=lambda:self.apply(save=True)); self.revert_button=ttk.Button(quick,text='Revert edits',command=self.revert_edits); self.reset_button=ttk.Button(quick,text='Restore defaults',command=self.reset_defaults)
        actions=ttk.Frame(quick,style='Card.TFrame'); actions.pack(fill='x')
        for i,b in enumerate((self.apply_button,self.save_button,self.revert_button,self.reset_button)):
            b.grid(in_=actions,row=i//2,column=i%2,sticky='ew',padx=2,pady=2);b.lift()
        actions.columnconfigure((0,1),weight=1)
        self.active_limit=tk.StringVar(value='Final joint limit: Unavailable')
        ttk.Label(quick,textvariable=self.active_limit,style='Muted.TLabel',wraplength=460).pack(anchor='w')
        self.advanced_open=tk.BooleanVar(value=False); ttk.Checkbutton(outer,text='Show advanced tuning',variable=self.advanced_open,command=self.toggle_advanced).pack(anchor='w',pady=4)
        self.advanced=ttk.Frame(outer,style='Card.TFrame')
        self.canvas=tk.Canvas(self.advanced,bg=COLORS['card'],highlightthickness=0,height=160)
        self.scrollbar=ttk.Scrollbar(self.advanced,orient='vertical',command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.pack(side='left',fill='both',expand=True)
        self.advanced_body=ttk.Frame(self.canvas,style='Card.TFrame',padding=6)
        self.canvas_window=self.canvas.create_window((0,0),window=self.advanced_body,anchor='nw')
        self.canvas.bind('<Configure>',self.resize_advanced)
        self.advanced_body.bind('<Configure>',self.resize_advanced)
        for category,title in [('motion','Motion'),('tracking','Tracking'),('position','Position-mode tuning'),('engineering','Engineering / calibration')]:
            section=ttk.Frame(self.advanced_body,style='Card.TFrame')
            ttk.Label(section,text=title,style='Section.TLabel').pack(anchor='w',pady=(4,1))
            self.add_fields(section,category)
            section.pack(fill='x',pady=2)
        self.alignment=tk.BooleanVar(value=False)
        self.optical=tk.BooleanVar(value=False)
        self.bind_scroll(self.advanced_body)
        self.bind_scroll(self.canvas)
        return outer

    def resize_advanced(self,event=None):
        self.canvas.itemconfigure(self.canvas_window,width=self.canvas.winfo_width())
        self.canvas.configure(scrollregion=self.canvas.bbox('all'))
        if self.advanced_body.winfo_reqheight()>self.canvas.winfo_height():
            self.scrollbar.pack(side='right',fill='y')
        else:
            self.scrollbar.pack_forget()

    def bind_scroll(self,widget):
        def wheel(event):
            self.canvas.yview_scroll(-1 if getattr(event,'num',None)==4 or getattr(event,'delta',0)>0 else 1,'units')
            return 'break'
        for event in ('<Button-4>','<Button-5>','<MouseWheel>'):
            widget.bind(event,wheel)
        widget.bind('<FocusIn>',self.reveal_focus,add='+')
        for child in widget.winfo_children():
            self.bind_scroll(child)

    def reveal_focus(self,event):
        self.root.update_idletasks()
        top=event.widget.winfo_rooty()-self.advanced_body.winfo_rooty()
        height=self.advanced_body.winfo_height()
        visible=self.canvas.canvasy(0)
        if top<visible or top+event.widget.winfo_height()>visible+self.canvas.winfo_height():
            self.canvas.yview_moveto(max(0,top-10)/max(1,height))

    def add_fields(self,parent,category):
        for spec in PARAMETER_REGISTRY:
            if spec.category!=category or not spec.editable: continue
            row=ttk.Frame(parent,style='Card.TFrame'); row.pack(fill='x',pady=2); ttk.Label(row,text=spec.label+(f' ({spec.units})' if spec.units else ''),style='Card.TLabel',wraplength=315).pack(side='left')
            v=tk.StringVar(value=display_text(spec,DEFAULTS[spec.key])); v.trace_add('write',self.mark_dirty); self.fields[spec.key]=v
            w=ttk.Combobox(row,textvariable=v,values=spec.choices,state='readonly',width=13) if spec.choices else ttk.Entry(row,textvariable=v,width=13)
            if spec.key=='teleop_control_mode': w.bind('<<ComboboxSelected>>',lambda _event:self.update_mode_visibility())
            w.pack(side='right'); self.field_rows[spec.key]=row; self.field_widgets[spec.key]=w

    def _interfaces(self):
        from trajectory_msgs.msg import JointTrajectory
        self.node.create_subscription(JointTrajectory, '/scaled_joint_trajectory_controller/smoothed_command', self.on_native_reference, 1)
        self.clients={key:self.node.create_client(Trigger,name) for key,name in {'arm':'/vr_real/arm','disarm':'/vr_real/disarm','servo':'/vr_real/servo_node/start_servo'}.items()}; self.enable_client=self.node.create_client(SetBool,'/vr_real/set_enabled'); self.switch_client=self.node.create_client(SwitchController,'/controller_manager/switch_controller'); self.settings=self.node.create_client(SetParametersAtomically,'/vr_real/teleop/set_parameters_atomically'); self.heartbeat=self.node.create_publisher(Bool,'/vr_real/operator_present',1); self.node.create_subscription(String,'/vr_real/operator_status',self.on_status,1); self.node.create_subscription(String,'/wrist_camera/status',self.on_camera_status,1)

    def on_native_reference(self, msg):
        from .validation import JOINTS
        if len(msg.points) != 1 or len(msg.joint_names) != 6 or set(msg.joint_names) != set(JOINTS):
            return
        p = msg.points[0]
        if len(p.positions) != 6 or len(p.velocities) != 6 or not all(math.isfinite(v) for v in [*p.positions, *p.velocities]):
            return
        age = (self.node.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        if not 0 <= age < .25:
            return
        order = [msg.joint_names.index(j) for j in JOINTS]
        self.native_reference = {'position': [p.positions[j] for j in order], 'velocity': [p.velocities[j] for j in order]}
        self.native_reference_at = time.monotonic()

    def on_camera_status(self,msg): self.camera_at,self.camera_text=time.monotonic(),msg.data
    def open_camera(self):
        if self.camera_window is None or self.camera_window.closed:
            from .camera_view import CameraView; self.camera_window=CameraView(self.root,self.node)
        else: self.camera_window.window.lift()
    def on_status(self,msg):
        if self.closed:
            return
        try:
            value=json.loads(msg.data)
        except (ValueError,TypeError):
            return
        if not isinstance(value,dict) or not isinstance(value.get('session_id'),str):
            return
        new=value['session_id']!=self.session
        if new:
            if self.session is not None:
                # A genuine backend restart: disarm, retire in-flight requests, clear old
                # session state, then wait for fresh backend status before a new Start.
                self.stop_teleop()
                self.startup.retire()
                self.session_changes+=1
                self.session_events.append(time.monotonic())
                if value['session_id'] not in self.session_ids:
                    self.session_ids.append(value['session_id'])
                self.session_restart=True
                self.notify('ROS backend session restarted — previous session retired. '
                            'Wait for fresh status, then press START TELEOP.')
            self.uncertain_enable_session=None
            self.uncertain_servo_session=None
            self.cancel_requests()
            self.generation+=1
            self.session=value['session_id']
            self.parameters.confirmed=False
            self.parameters.pending=None
            self.parameters.phase=''
            self.confirmation_pending=None
            self.settings_result=''
            self.alignment.set(False)
            self.optical.set(False)
            self.confirmation_rejected=False
        if value.get('servo_status_fresh') is True:
            # A fresh Servo status resolves any previously uncertain start outcome.
            self.uncertain_servo_session=None
        for key,var in [('alignment_verified',self.alignment),('optical_loss_test_verified',self.optical)]:
            if boolean(self.last_status,key) is True and boolean(value,key) is False:
                var.set(False)
        self.last_status,self.last_status_at=value,time.monotonic()
        self.load_running(value)
        if self.setup_wizard is not None:
            self.setup_wizard.observe(value, self.last_status_at)
        if self.confirmation_pending is not None:
            if all(boolean(value,k)==v for k,v in self.confirmation_pending.items()):
                self.confirmation_pending=None
        if not self.parameters.pending and self.settings_result and self.confirmation_pending is None:
            self.notify(self.settings_result,error='failed' in self.settings_result)
            self.settings_result=''

    def sync_draft(self):
        self.parameters.draft={k:v.get() for k,v in self.fields.items()}

    def show_draft(self):
        self._loading_fields=True
        try:
            for key,text in self.parameters.draft.items():
                if self.fields[key].get()!=text:
                    self.fields[key].set(text)
        finally:
            self._loading_fields=False
        self.update_mode_visibility()
        self.mark_dirty()

    def load_running(self,status):
        self.sync_draft()
        hardware=status.get('hardware_smoothing')
        active=hardware.get('velocity') if isinstance(hardware,dict) else None
        self.parameters.receive(status.get('effective_parameters'),active)
        self.running_values=self.parameters.effective
        self.show_draft()

    def mark_dirty(self,*_args):
        if self._loading_fields:
            return
        self.sync_draft()
        # A previous check cannot approve a newly edited mapping.
        mapping=self.fields.get('tracking_to_robot_rpy_deg')
        text=mapping.get() if mapping else None
        if hasattr(self,'_mapping_text') and text!=self._mapping_text:
            self.alignment.set(False)
        self._mapping_text=text
        self.unsaved.set(self.parameters.label(not self.stale()))
        for key,widget in self.field_widgets.items():
            if PARAMETERS_BY_KEY[key].choices:
                continue
            try:
                validate({key:stored_value(PARAMETERS_BY_KEY[key],self.fields[key].get())})
                widget.configure(style='TEntry')
            except (ValueError,TypeError,OverflowError):
                widget.configure(style='Invalid.TEntry')

    def update_mode_visibility(self):
        mode=self.fields['teleop_control_mode'].get()
        for spec in PARAMETER_REGISTRY:
            if spec.key in self.field_rows and spec.mode=='position':
                row=self.field_rows[spec.key]
                if mode=='position': row.pack(fill='x',pady=2)
                else: row.pack_forget()

    def toggle_advanced(self):
        if self.advanced_open.get():
            self.advanced.pack(fill='both',expand=True,pady=(0,4))
            self.update_mode_visibility()
        else:
            self.advanced.pack_forget()

    def stale(self):
        return time.monotonic()-self.last_status_at>=.5

    def notify(self,text,error=False):
        self.notifications.post(text,time.monotonic(),error)
        self.message.set(self.notifications.text(time.monotonic()))

    def dismiss_notification(self):
        self.notifications.expires=0
        self.message.set('')

    def show_history(self):
        window=tk.Toplevel(self.root)
        window.title('Advanced diagnostics / recent operator actions')
        row = ttk.Frame(window); row.pack(fill='x')
        ttk.Button(row, text='RECORD SWAY DIAGNOSTICS', command=self.start_recording).pack(side='left')
        ttk.Button(row, text='STOP RECORDING', command=self.recorder.stop).pack(side='left')
        ttk.Label(window, textvariable=self.record_status, wraplength=850).pack(anchor='w')
        ttk.Label(window, text=str(self.last_status.get('setup_technical', ''))+'\n'+self.setup_technical, wraplength=850).pack()
        text=tk.Text(window,width=100,height=20,wrap='word')
        text.pack(fill='both',expand=True)
        text.insert('end','Backend status:\n'+json.dumps(self.last_status,indent=2)+'\n\nCurrent blockers:\n'+str(self.last_status.get('blockers','Unavailable'))+'\n\nRecent actions:\n'+'\n'.join(self.notifications.history))
        text.configure(state='disabled')

    def stack_action(self,action,mode=None):
        if not self.process_worker or self.closed:
            return
        if action != 'rviz':self.stop_teleop()
        if self.process_worker.submit(action,mode):
            self.heartbeat.publish(Bool(data=False))
            self.notify('ROS '+action+' requested; waiting for process manager')

    def open_rviz(self):
        self.stack_action('rviz')

    def track_request(self,label,future,desired=None):
        self.pending.append((label,future))
        self.request_meta[future]=(time.monotonic(),self.generation,desired)

    def request_busy(self,label):
        return any(name==label for name,_ in self.pending)

    def prepare_controller(self):
        if getattr(self,'startup',None) and self.startup.requested:return
        d=self.last_status
        if self.stale() or d.get('armed') is not False or d.get('a_held') is not False or d.get('enable_requested') is not False:
            self.notify('Disable teleop and release A before preparing the controller.',True);return
        if not str(d.get('mode','')).startswith('LIVE'):
            self.notify('Preview does not activate motion controllers. Start Servo solver for Preview.',True);return
        if not d.get('observations_healthy') or not d.get('stationary') or not d.get('robot_ready'):
            self.notify('Fresh tracking/joints/TF, standstill, and External Control are required.',True);return
        if d.get('controller_ready'):
            self.call('servo');return
        if any(name=='prepare_controller' for name,_ in self.pending):return
        if not self.switch_client.service_is_ready():
            self.notify('Controller manager unavailable; resolve the condition then restart ROS.',True);return
        r=SwitchController.Request();r.activate_controllers=['scaled_joint_trajectory_controller'];r.strictness=r.STRICT;r.timeout.sec=3
        try:
            future=self.switch_client.call_async(r)
        except Exception as error:
            self.notify('Prepare controller: '+str(error),True)
            return
        self.track_request('prepare_controller',future)
        self.notify('Preparing scaled controller; teleop remains disabled.')

    def call(self,name):
        if name=='servo' and getattr(self,'startup',None) and self.startup.requested:return
        if self.request_busy(name):return
        if not self.clients[name].service_is_ready():
            self.notify(f'{name}: service unavailable',True);return
        try:
            self.track_request(name,self.clients[name].call_async(Trigger.Request()))
            self.notify(name+': request sent')
        except Exception as error:
            self.notify(f'{name}: {error}',True)

    def open_setup(self):
        from .setup_wizard import SetupWizard
        self.setup_prompted = True
        if self.setup_wizard is None:
            self.stop_teleop()
            self.setup_wizard = SetupWizard(self)
        else:
            self.setup_wizard.window.lift()

    def start_teleop(self):
        if self.retiring_startup or self.startup_request or self.uncertain_enable_session == self.session and self.session is not None:
            self.notify('Waiting for canceled startup to retire. An enable timeout requires Restart ROS.',True)
            return
        if self.uncertain_servo_session == self.session and self.session is not None:
            self.notify('Servo start outcome is unknown. Wait for fresh Servo status, or Restart ROS.',True)
            return
        if self.pending or self.parameters.pending:
            self.notify('Wait for the current engineering/settings request to finish.',True)
            return
        if self.startup.start(self.session):
            self.heartbeat_allowed=True
            self.session_restart=False

    def stop_teleop(self):
        if getattr(self, 'setup_wizard', None) is not None:
            self.setup_wizard.cancel_capture()
        self.startup.stop()
        # A canceled Future cannot cancel a request already executing at the server.
        # Withhold heartbeat until it retires, then send disarm again.
        self.heartbeat_allowed=False
        self.heartbeat.publish(Bool(data=False))
        self.retiring_startup=self.startup_request is not None
        self.call('disarm')

    def poll_startup(self, now):
        request=self.startup_request
        if request is not None:
            action,future,client,started,generation=request
            if future.done() or now-started>=5:
                success=False;detail='service timeout; outcome unknown'
                try:
                    if future.done():
                        result=future.result()
                        success=bool(result.ok if action=='prepare' else result.success)
                        detail=getattr(result,'message','Controller activation rejected')
                    elif action=='enable' and generation==self.generation:
                        self.uncertain_enable_session=self.session
                    elif action=='servo' and generation==self.generation:
                        self.uncertain_servo_session=self.session
                except Exception as error:
                    detail=str(error)
                    if action=='enable' and generation==self.generation:self.uncertain_enable_session=self.session
                    if action=='servo' and generation==self.generation:self.uncertain_servo_session=self.session
                if not future.done():future.cancel()
                client.remove_pending_request(future)
                self.startup_request=None
                if generation==self.generation and not self.retiring_startup:
                    self.startup.acknowledge(action,success,detail,now)
                else:
                    self.retiring_startup=False
                    self.call('disarm')
        action=self.startup.step(self.last_status,not self.stale(),now)
        if self.startup.state=='ERROR':
            if self.heartbeat_allowed:
                self.heartbeat_allowed=False
                self.heartbeat.publish(Bool(data=False))
                self.call('disarm')
            return
        if action:
            client={'prepare':self.switch_client,'servo':self.clients['servo'],'enable':self.enable_client}[action]
            if not client.service_is_ready():
                self.startup.acknowledge(action,False,action+' service unavailable',now)
                self.heartbeat_allowed=False
                self.heartbeat.publish(Bool(data=False))
                self.call('disarm')
                return
            if action=='prepare':
                req=SwitchController.Request();req.activate_controllers=['scaled_joint_trajectory_controller']
                req.strictness=req.STRICT;req.timeout.sec=3
            elif action=='enable':req=SetBool.Request(data=True)
            else:req=Trigger.Request()
            try:
                self.startup_request=(action,client.call_async(req),client,now,self.generation)
            except Exception as error:
                self.startup.acknowledge(action,False,str(error),now)

    def field_values(self):
        self.sync_draft()
        return self.parameters.values()

    def revert_edits(self):
        try:
            if self.stale():raise ValueError('Unable to verify running values')
            confirmations=(self.alignment.get(),self.optical.get())
            self.parameters.revert()
            self.show_draft()
            self.alignment.set(confirmations[0]);self.optical.set(confirmations[1])
            self.notify('Edits reverted to confirmed running values; saved restart settings retained.')
        except ValueError as error:
            self.notify(str(error),True)

    def reset_defaults(self):
        self.parameters.defaults()
        self.show_draft()
        self.notify('Defaults staged — review before applying')

    def apply(self,save=False):
        try:
            values=self.field_values()
            if self.stale() or not self.parameters.confirmed or self.last_status.get('armed') is not False or self.last_status.get('a_held') is not False:
                raise ValueError('Settings require verified fresh values, disarmed status and A released')
            if self.parameters.conflicts:raise ValueError('External parameter changes detected: Revert edits before applying')
            if self.parameters.pending:raise ValueError('Waiting for previous settings request/readback')
            if not self.settings.service_is_ready():raise ValueError('Settings service unavailable')
            applied=self.parameters.changed()
            # Never send restart-only parameters to the runtime service.
            restart=not equivalent(values['max_joint_speed_rad_s'],self.parameters.effective['max_joint_speed_rad_s'])
            if restart and not save:raise ValueError('Joint speed requires Apply + save, then Restart ROS.')
            applied.pop('max_joint_speed_rad_s',None)
            r=SetParametersAtomically.Request()
            r.parameters=[Parameter(name,value=value).to_parameter_msg() for name,value in applied.items()]
            future=self.settings.call_async(r)
            self.parameters.begin(applied,values,save)
            self.confirmation_rejected=False
            self.track_request('settings',future)
            self.notify('Apply: waiting for backend acceptance')
            self.mark_dirty()
        except (ValueError,TypeError,RuntimeError) as error:
            self.notify('Apply: '+str(error),True)

    def complete_settings(self,future,result):
        request=self.parameters.pending
        if not request:return
        if not result.successful:
            self.parameters.pending=None;self.parameters.phase=''
            self.confirmation_rejected=True
            self.notify('Apply rejected: '+result.reason,True)
            return
        self.parameters.phase='readback'
        self.readback_started=time.monotonic()
        self.confirmation_pending={k:v for k,v in request['applied'].items() if k not in DEFAULTS}
        self.settings_result='Apply confirmed by backend readback'
        if request['save']:
            try:
                save_settings(self.config_path,request['values'])
                self.parameters.saved=dict(request['values'])
                joint=request['values']['max_joint_speed_rad_s']
                if not equivalent(joint,self.parameters.effective['max_joint_speed_rad_s']):
                    self.parameters.restart={'max_joint_speed_rad_s':joint}
                else:
                    self.parameters.restart={}
                self.settings_result='Apply + Save confirmed'+ ('; Restart ROS to apply saved joint limit' if self.parameters.restart else '')
            except (OSError,ValueError) as error:
                self.settings_result='Applied to current session, but saving failed: '+str(error)
                self.notify(self.settings_result,True)
                return
        self.notify('Applied — awaiting readback',True)

    def check_readback(self,now):
        if (self.parameters.phase=='readback' or self.confirmation_pending is not None) and now-getattr(self,'readback_started',now)>5:
            self.parameters.pending=None
            self.parameters.phase=''
            self.parameters.confirmed=False
            self.confirmation_pending=None
            self.settings_result=''
            self.notify('Applied — awaiting readback: confirmation timed out. Review current values and Revert when available.',True)

    def cancel_requests(self):
        for _,future in self.pending:
            future.cancel()
            for client in list(self.clients.values())+[self.enable_client,self.switch_client,self.settings]:
                client.remove_pending_request(future)
        self.pending.clear()
        self.request_meta.clear()

    def finish_requests(self):
        now=time.monotonic()
        for label,future in list(self.pending):
            started,generation,desired=self.request_meta.get(future,(now,self.generation,None))
            expired=now-started>5
            if not future.done() and not expired:continue
            self.pending.remove((label,future));self.request_meta.pop(future,None)
            if generation!=self.generation:
                future.cancel()
                for client in list(self.clients.values())+[self.enable_client,self.switch_client,self.settings]:
                    client.remove_pending_request(future)
                continue
            try:
                if expired or self.stale():
                    raise TimeoutError('request timed out / backend status unavailable; outcome unknown')
                result=future.result()
                if label=='settings':self.complete_settings(future,result.result)
                elif label=='prepare_controller':
                    self.notify('Controller prepared; start Servo solver next.' if result.ok else 'Controller activation rejected; inspect driver log.',not result.ok)
                else:
                    self.notify(f'{label}: {result.message}',not result.success)
            except Exception as error:
                self.notify(f'{label}: {error}',True)
                if label=='settings':
                    self.parameters.pending=None;self.parameters.phase='';self.parameters.confirmed=False
                    self.confirmation_rejected=True
            finally:
                if not future.done():future.cancel()
                for client in list(self.clients.values())+[self.enable_client,self.switch_client,self.settings]:
                    client.remove_pending_request(future)

    def set_ready(self,key,ok,text):
        value,w=self.readiness[key]
        value.set(('✓ ' if ok is True else '? ' if ok is None else '● ')+text)
        w.configure(foreground=COLORS['green'] if ok is True else COLORS['muted'] if ok is None else COLORS['amber'])

    def session_warning(self):
        recent=sum(1 for t in self.session_events if time.monotonic()-t<60.)
        if len(self.session_ids)>1 and recent>=2:
            return (f'{len(self.session_ids)} backend session IDs, {recent} restarts in 60 s — '
                    'check for duplicate commissioning backends on /vr_real/operator_status.')
        if self.session_restart:
            return 'Waiting for fresh backend status; press START TELEOP when the work area is ready.'
        return ''

    def set_banner(self,stage,detail='',color=None):
        if color is None:
            color=(COLORS['red'] if stage.startswith('FAULT') else COLORS['green']
                   if stage.startswith(('READY','TELEOP ACTIVE')) else COLORS['amber'])
        self.startup_stage.set(stage)
        self.startup_detail.set(detail)
        for widget in (self.banner,self.banner_label,self.banner_detail):
            widget.configure(bg=color)

    def clutch_text(self,armed,d):
        raw=d.get('clutch','Unavailable')
        if armed is None:
            return 'Unknown — backend motion authorization unavailable'
        if armed is not True:
            return 'DISARMED' + (' — A held, no motion authorization' if boolean(d,'a_held') is True else '')
        if raw=='ACTIVE' and boolean(d,'motion_ready') is True:
            return 'ACTIVE'
        return 'ARMED — ready for a fresh A press' if raw!='ACTIVE' else 'ARMED'

    def servo_text(self,d):
        if boolean(d,'servo_status_fresh') is True:
            return str(d.get('servo_detail','Unavailable'))+' (running)'
        state=d.get('servo_state')
        if state=='stale': return 'Stale — started but no fresh status (treated as not healthy)'
        if state=='stopped': return 'Stopped — process no longer publishing status'
        if state=='absent': return 'Absent — Servo process not running'
        if state=='unstarted': return 'Not started'
        return 'Stale / Unavailable'

    def update_dashboard(self,stale,d):
        if stale:
            self.header_mode.set('DISCONNECTED');self.header_state.set('CONNECTING')
            self.stack_status.set('Waiting for backend / motion status unavailable')
            self.header_badge.configure(fg=COLORS['muted'])
            self.live_message.set('No fresh backend status. Do not infer robot state; check the pendant.')
            self.diagnostics.set('Diagnostics: Stale / Unavailable')
            self.lighthouse.set('Lighthouse 0: Unknown\nLighthouse 1: Unknown\nValid pose: Unknown')
            self.motion.set('Clutch displacement: Stale / not current')
            self.set_banner('DISCONNECTED — WAITING FOR BACKEND',
                            'No fresh backend status. Do not infer robot state; check the pendant.',COLORS['muted'])
            for key in self.readiness:self.set_ready(key,None,'Unknown / status unavailable')
            return
        mode=str(d.get('mode','UNKNOWN'))
        self.stack_status.set('Live stack connected' if mode.startswith('LIVE') else 'Engineering Preview connected')
        self.header_mode.set('LIVE' if mode.startswith('LIVE') else 'PREVIEW' if 'DRY RUN' in mode else mode)
        ready=boolean(d,'motion_ready');armed=boolean(d,'armed')
        if any(boolean(d,key) is None for key in ('armed','enable_requested','configuration_ready','operator_heartbeat_fresh')):
            ready=armed=None
        self.header_state.set('TELEOP ACTIVE' if armed is True and d.get('clutch')=='ACTIVE' else 'READY' if armed is True and ready is True else 'WAITING')
        fault=boolean(d,'servo_status_fresh') is True and type(d.get('servo')) is int and d['servo'] in (2,4,5)
        if fault:self.header_state.set('FAULT')
        self.header_badge.configure(fg=COLORS['red'] if fault else COLORS['amber'] if self.pending else COLORS['green'] if ready is True else COLORS['blue'] if 'DRY RUN' in mode else COLORS['amber'] if ready is False else COLORS['muted'])
        blockers=d.get('blockers')
        blockers=blockers if isinstance(blockers,list) and all(isinstance(x,str) for x in blockers) else []
        self.live_message.set(blockers[0] if blockers else str(d.get('reason','Readiness not yet confirmed')))
        for key,source,title in [('robot','robot_ready','Robot / External Control'),('controller','controller_ready','Controller ownership'),('joints','observations_healthy','Joint / TF observations'),('pose','pose_fresh','Controller pose fresh'),('optical','optical','Optical signal (not pose proof)'),('heartbeat','operator_heartbeat_fresh','Backend heartbeat')]:
            ok=boolean(d,source)
            if key=='controller' and boolean(d,'controller_state_fresh') is not True:
                ok=None
            self.set_ready(key,ok,title+(': confirmed' if ok is True else ': unavailable' if ok is None else ': not ready'))
        for key,source,draft in [('alignment','alignment_verified',self.alignment.get()),('confirmations','optical_loss_test_verified',self.optical.get())]:
            ok=boolean(d,source)
            label='Alignment' if key=='alignment' else 'Optical-loss test'
            text='Verified' if ok is True else 'Unknown / status unavailable' if ok is None else 'Not verified — request rejected' if self.confirmation_rejected else 'Pending application' if draft else 'Not verified / revoked'
            self.set_ready(key,ok,label+': '+text)
        diag=d.get('velocity_diagnostics')
        a_text={True:'held',False:'released',None:'Unknown'}[boolean(d,'a_held')]
        servo=self.servo_text(d)
        authorization={True:'ARMED (motion permitted once A is held)',False:'disarmed (no motion authorization)',None:'Unknown'}[armed]
        clutch=self.clutch_text(armed,d)
        self.diagnostics.set('Published TCP command: '+vector_text(vector(diag,'published_twist'))+
            '\nMeasured TCP estimate (TF): '+vector_text(vector(diag,'measured_tcp_velocity'))+
            f"\nServo: {servo}\nA button (VR input): {a_text}\nBackend motion authorization: {authorization}\nClutch (effective): {clutch}")
        native = self.native_reference if time.monotonic()-self.native_reference_at < .25 else {}
        measured = vector(d, 'measured_joint_velocity')
        actual_q = vector(d, 'measured_joint_position')
        reference_q = vector(native, 'position')
        error = [a-b for a,b in zip(reference_q, actual_q)] if reference_q is not None and actual_q is not None else None
        joint_text = lambda v: 'Unavailable' if v is None else ', '.join(f'{x:.3g}' for x in v)
        self.diagnostics.set(self.diagnostics.get()+'\nFinal joint command (rad/s): '+joint_text(vector(native, 'velocity'))+
                             '\nMeasured joints (rad/s): '+joint_text(measured)+'\nJoint tracking error (rad): '+joint_text(error))
        tracking=boolean(d,'tracking_valid')
        count=d.get('active_station_count')
        tracking_text=('TRACKING VALID — '+str(count)+' LIGHTHOUSE'+('S' if count!=1 else '')) if tracking is True and type(count) is int and count>0 else 'TRACKING: LOST' if self.startup.enabled else 'TRACKING: WAITING'
        self.lighthouse.set(tracking_text)
        self.set_ready('tracking',tracking,tracking_text)
        robot=boolean(d,'robot_preparation_ready')
        robot_fault=boolean(d,'robot_fault') is True
        self.set_ready('robot',robot,'ROBOT: FAULT' if robot_fault else 'ROBOT: READY' if robot is True else 'ROBOT: WAITING')
        if robot_fault:self.header_state.set('FAULT')
        if self.startup.requested or self.startup.state=='ERROR' or not self.heartbeat_allowed:
            self.live_message.set(self.startup.message)
        if self.startup.state=='ERROR':
            failure=self.startup.last_failure or {}
            detail=str(failure.get('message','Startup failed'))
            hint=str(failure.get('hint',''))
            self.set_banner('FAULT — '+str(failure.get('stage','STARTUP')),
                            detail+(' — '+hint if hint else ''),COLORS['red'])
            self.header_state.set('FAULT')
        elif self.startup.requested and self.startup.state!='READY':
            self.set_banner(self.startup.stage,self.startup.message)
            self.header_state.set(self.startup.stage)
        elif self.startup.state=='READY':
            warning=self.session_warning()
            self.set_banner(self.startup.stage,warning)
        elif self.session_restart:
            self.set_banner('SESSION CHANGED — PRESS START TELEOP',self.session_warning())
        else:
            self.set_banner('DISARMED — PRESS START TELEOP','')
        if not fault and not robot_fault and self.startup.enabled and tracking is False:
            self.header_state.set('TRACKING LOST')
        self.motion.set(motion_text(d))

    def poll(self):
        if self.closed:return
        if self.shutdown_requested or not rclpy.ok():self.close();return
        rclpy.spin_once(self.node,timeout_sec=0)
        if self.process_worker:
            try:
                message,busy=self.process_worker.events.get_nowait()
                self.stack_status.set(message if len(message)<=95 else message[:92]+'…')
                if message!=getattr(self,'_manager_message',None):
                    self._manager_message=message
                    self.notify('ROS: '+message,any(word in message.lower() for word in ('failed','exited','error')))
            except queue.Empty:pass
        self.record_status.set(self.recorder.poll())
        now=time.monotonic()
        if now-self.heartbeat_at>=.1:
            self.heartbeat.publish(Bool(data=self.heartbeat_allowed));self.heartbeat_at=now
        self.finish_requests()
        self.check_readback(now)
        self.poll_startup(now)
        if self.setup_wizard is not None:
            self.setup_wizard.poll()
        self.camera_status.set(self.camera_text if now-self.camera_at<4 else 'D405 unavailable / no recent status (teleop unaffected)')
        stale,d=self.stale(),self.last_status
        self.update_dashboard(stale,d)
        self.mark_dirty()
        self.message.set(self.notifications.text(now))
        busy=self.request_busy('enable') or self.request_busy('prepare_controller')
        disabled=stale or d.get('armed') is not False or d.get('a_held') is not False or self.startup.requested
        for b in (self.apply_button,self.save_button):b.state(['disabled'] if disabled or self.parameters.pending else ['!disabled'])
        self.revert_button.state(['disabled'] if stale or not self.parameters.confirmed or self.parameters.pending else ['!disabled'])
        self.reset_button.state(['disabled'] if self.parameters.pending else ['!disabled'])
        self.enable_button.state(['disabled'] if self.startup.requested or self.retiring_startup or self.startup_request else ['!disabled'])
        self.enable_button.configure(text='START TELEOP')
        self.servo_button.state(['disabled'] if stale or d.get('armed') or self.request_busy('servo') or self.startup.requested else ['!disabled'])
        self.prepare_button.state(['disabled'] if disabled or d.get('enable_requested') or busy or self.startup.requested else ['!disabled'])
        hardware=d.get('hardware_smoothing') if not stale else None
        limit=hardware.get('velocity') if isinstance(hardware,dict) else None
        effective=d.get('effective_parameters',{}) if not stale else {}
        self.active_limit.set('Effective TCP caps: '+str(effective.get('max_linear_speed_m_s','?'))+' m/s; '+str(effective.get('max_angular_speed_deg_s','?'))+' deg/s\nActive final joint limit: '+(f'{limit:g} rad/s' if type(limit) in (float,int) and math.isfinite(limit) else 'Unavailable')+('\nSaved for restart: '+str(self.parameters.restart['max_joint_speed_rad_s'])+' rad/s' if self.parameters.restart else ''))
        blockers=d.get('blockers',[])
        text='Status unavailable' if stale else '\n'.join(str(x) for x in blockers) if isinstance(blockers,list) else 'Blocker details unavailable'
        if self.blockers.get('1.0','end-1c')!=text:
            self.blockers.configure(state='normal');self.blockers.delete('1.0','end');self.blockers.insert('end',text);self.blockers.configure(state='disabled')
        self.root.after(100,self.poll)

    def close(self):
        if self.closed:return
        self.closed=True
        try:
            if getattr(self,'setup_wizard',None) is not None:self.setup_wizard.close(stop=False)
            if getattr(self, 'recorder', None):
                import threading
                threading.Thread(target=self.recorder.close, name='recorder-shutdown', daemon=False).start()
            if getattr(self,'startup',None):self.startup.stop()
            if getattr(self,'startup_request',None):
                _,future,client,_,_=self.startup_request
                future.cancel();client.remove_pending_request(future)
            if getattr(self,'process_worker',None):self.process_worker.close()
            if hasattr(self,'request_meta'):self.cancel_requests()
            if getattr(self,'camera_window',None) is not None:self.camera_window.close()
            if rclpy.ok():self.heartbeat.publish(Bool(data=False))
        finally:self.root.destroy()


def main(args=None):
    rclpy.init(args=args);gui=None
    try:gui=OperatorGUI();gui.root.mainloop()
    finally:
        if gui:gui.node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
