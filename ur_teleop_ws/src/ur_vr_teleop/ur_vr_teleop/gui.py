"""Responsive single-window Tk dashboard. It requests supervision; it never commands motion."""
import json
import os
import threading
import time
import tkinter as tk
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
                          display_value, save_settings, stored_value, validate)

COLORS = {'navy': '#17263D', 'blue': '#2563EB', 'green': '#15803D', 'amber': '#D97706',
          'red': '#DC2626', 'bg': '#F3F6FA', 'card': '#FFFFFF', 'text': '#1F2937', 'muted': '#64748B'}


class OperatorGUI:
    """Dashboard using named parameters and async ROS requests only."""
    def __init__(self, manager=None):
        if os.environ.get('ROS_DOMAIN_ID') != '43': raise RuntimeError('GUI requires ROS_DOMAIN_ID=43')
        self.node, self.manager = Node('teleop_operator_gui', namespace='vr_real'), manager
        self.config_path = Path(getattr(manager, 'config', Path.home()/'Documents/Teleop/ur_teleop/config/real_teleop.yaml'))
        self.closed = self.shutdown_requested = False
        self.pending, self.pending_settings, self.workers = [], {}, []
        self.running_values, self.last_status = {}, {}
        self.last_status_at = self.heartbeat_at = -float('inf')
        self.camera_at, self.camera_text, self.camera_window = -float('inf'), 'Camera not connected', None
        self.fields, self.field_rows, self._loading_fields = {}, {}, False
        self.root = tk.Tk(); self.root.title('UR10e Teleoperation Dashboard'); self.root.geometry('1440x900'); self.root.minsize(1280, 700)
        self.root.configure(bg=COLORS['bg']); self._style(); self._build(); self._interfaces()
        self.root.protocol('WM_DELETE_WINDOW', self.close); self.root.after(20, self.poll)

    def _style(self):
        s = ttk.Style(); s.theme_use('clam')
        s.configure('App.TFrame', background=COLORS['bg']); s.configure('Card.TFrame', background=COLORS['card'], relief='solid', borderwidth=1)
        s.configure('Card.TLabel', background=COLORS['card'], foreground=COLORS['text'], font=('DejaVu Sans', 10))
        s.configure('Muted.TLabel', background=COLORS['card'], foreground=COLORS['muted'], font=('DejaVu Sans', 9))
        s.configure('Section.TLabel', background=COLORS['card'], foreground=COLORS['text'], font=('DejaVu Sans', 12, 'bold'))
        s.configure('Primary.TButton', foreground='white', background=COLORS['blue'], font=('DejaVu Sans', 10, 'bold'), padding=(9, 7))
        s.configure('Danger.TButton', foreground='white', background=COLORS['red'], font=('DejaVu Sans', 10, 'bold'), padding=(9, 7))
        s.configure('TButton', font=('DejaVu Sans', 10), padding=(7, 5))

    def card(self, parent, title):
        f = ttk.Frame(parent, style='Card.TFrame', padding=11); ttk.Label(f, text=title, style='Section.TLabel').pack(anchor='w', pady=(0, 7)); return f

    def _build(self):
        self.root.grid_rowconfigure(1, weight=1); self.root.grid_columnconfigure(0, weight=1)
        header = tk.Frame(self.root, bg=COLORS['navy'], height=76); header.grid(row=0, column=0, sticky='ew'); header.grid_propagate(False); header.grid_columnconfigure(1, weight=1)
        tk.Label(header, text='UR10e TELEOPERATION', bg=COLORS['navy'], fg='white', font=('DejaVu Sans', 19, 'bold')).grid(row=0, column=0, padx=20, pady=18)
        self.header_mode, self.header_state = tk.StringVar(value='DISCONNECTED'), tk.StringVar(value='WAITING FOR GATE')
        tk.Label(header, textvariable=self.header_mode, bg=COLORS['navy'], fg='#BFDBFE', font=('DejaVu Sans', 11, 'bold')).grid(row=0, column=1, sticky='e', padx=12)
        tk.Label(header, textvariable=self.header_state, bg=COLORS['navy'], fg='#FDE68A', font=('DejaVu Sans', 11, 'bold')).grid(row=0, column=2, sticky='e', padx=20)
        content = ttk.Frame(self.root, style='App.TFrame', padding=12); content.grid(row=1, column=0, sticky='nsew')
        for c, weight, minimum in ((0,25,290),(1,44,430),(2,31,330)): content.grid_columnconfigure(c, weight=weight, minsize=minimum)
        content.grid_rowconfigure(0, weight=1)
        self.operations(content).grid(row=0,column=0,sticky='nsew',padx=(0,8)); self.monitoring(content).grid(row=0,column=1,sticky='nsew',padx=4); self.tuning(content).grid(row=0,column=2,sticky='nsew',padx=(8,0))
        strip = tk.Frame(self.root, bg='#E8EEF6'); strip.grid(row=2,column=0,sticky='ew'); strip.grid_columnconfigure(0,weight=1)
        self.message = tk.StringVar(value='Waiting for a commissioning backend.')
        tk.Label(strip,textvariable=self.message,bg='#E8EEF6',fg=COLORS['text'],anchor='w',font=('DejaVu Sans',10)).grid(row=0,column=0,sticky='ew',padx=15,pady=7)
        self.blockers = tk.Text(strip,height=3,wrap='word',font=('DejaVu Sans',9),relief='flat',bg='#E8EEF6'); self.blockers.grid(row=1,column=0,sticky='ew',padx=12,pady=(0,7)); self.blockers.configure(state='disabled')

    def operations(self, parent):
        outer=ttk.Frame(parent,style='App.TFrame'); session=self.card(outer,'Session controls'); session.pack(fill='x',pady=(0,8))
        for text, action, mode, style in [('Start Preview','start',False,'TButton'),('Start Live Setup','start',True,'Primary.TButton'),('Restart ROS','restart',None,'TButton'),('Stop ROS','stop',None,'Danger.TButton')]:
            ttk.Button(session,text=text,style=style,command=lambda a=action,m=mode:self.stack_action(a,m)).pack(fill='x',pady=2)
        self.stack_status=tk.StringVar(value='Stopped'); ttk.Label(session,textvariable=self.stack_status,style='Muted.TLabel',wraplength=270).pack(anchor='w',pady=(6,0))
        ready=self.card(outer,'Readiness checklist'); ready.pack(fill='both',expand=True,pady=(0,8)); self.readiness={}
        for key,label in [('robot','Robot / External Control'),('controller','Controller ownership'),('joints','Joint feedback valid'),('pose','VR pose fresh'),('optical','Lighthouse optical signal'),('alignment','Alignment verified'),('heartbeat','Operator heartbeat'),('confirmations','Session confirmations')]:
            value=tk.StringVar(value='● Waiting'); w=ttk.Label(ready,textvariable=value,style='Muted.TLabel',wraplength=270); w.pack(anchor='w',pady=2); self.readiness[key]=(value,w)
        motion=self.card(outer,'Motion controls'); motion.pack(fill='x')
        self.servo_button=ttk.Button(motion,text='Start Servo solver',command=lambda:self.call('servo')); self.prepare_button=ttk.Button(motion,text='Prepare controller',command=self.prepare_controller); self.enable_button=ttk.Button(motion,text='ENABLE TELEOP',style='Primary.TButton',command=self.toggle_enabled); self.disarm_button=ttk.Button(motion,text='DISARM',style='Danger.TButton',command=lambda:self.call('disarm'))
        for b in (self.servo_button,self.prepare_button,self.enable_button,self.disarm_button): b.pack(fill='x',pady=2)
        ttk.Label(motion,text='DISARM requests a software stop; it is not a safety-rated emergency stop. Use the pendant for an emergency.',style='Muted.TLabel',wraplength=270).pack(anchor='w',pady=(7,0)); return outer

    def monitoring(self,parent):
        outer=ttk.Frame(parent,style='App.TFrame'); camera=self.card(outer,'Camera / visualization'); camera.pack(fill='x',pady=(0,8))
        ttk.Label(camera,text='Optional camera is isolated from motion supervision.',style='Muted.TLabel').pack(anchor='w'); ttk.Button(camera,text='Open D405 camera',command=self.open_camera).pack(anchor='w',pady=(7,2)); self.camera_status=tk.StringVar(value='D405 unavailable / no recent status (teleop unaffected)'); ttk.Label(camera,textvariable=self.camera_status,style='Muted.TLabel',wraplength=420).pack(anchor='w')
        if self.manager: ttk.Button(camera,text='Open RViz',command=self.open_rviz).pack(anchor='w',pady=(5,0))
        live=self.card(outer,'Live robot diagnostics'); live.pack(fill='x',pady=(0,8)); self.diagnostics=tk.StringVar(value='Waiting for telemetry…'); ttk.Label(live,textvariable=self.diagnostics,style='Card.TLabel',justify='left',wraplength=430).pack(anchor='w')
        lighthouse=self.card(outer,'Lighthouse tracking'); lighthouse.pack(fill='both',expand=True); self.lighthouse=tk.StringVar(value='Lighthouse 0: waiting\nLighthouse 1: waiting\nValid pose: waiting\nLast rejection: waiting'); ttk.Label(lighthouse,textvariable=self.lighthouse,style='Card.TLabel',justify='left',wraplength=430).pack(anchor='w'); self.motion=tk.StringVar(value='Hold A in Preview to inspect clutch displacement.'); ttk.Label(lighthouse,textvariable=self.motion,style='Muted.TLabel',wraplength=430).pack(anchor='w',pady=(10,0)); return outer

    def tuning(self,parent):
        outer=ttk.Frame(parent,style='App.TFrame'); quick=self.card(outer,'Quick tuning'); quick.pack(fill='x',pady=(0,8)); self.add_fields(quick,'quick')
        self.unsaved=tk.StringVar(value='Running values load when backend connects.'); ttk.Label(quick,textvariable=self.unsaved,style='Muted.TLabel',wraplength=305).pack(anchor='w',pady=(7,4))
        self.apply_button=ttk.Button(quick,text='Apply changes',style='Primary.TButton',command=self.apply); self.save_button=ttk.Button(quick,text='Apply + save',command=lambda:self.apply(save=True)); self.revert_button=ttk.Button(quick,text='Revert edits',command=self.revert_edits); self.reset_button=ttk.Button(quick,text='Restore defaults',command=self.reset_defaults)
        for b in (self.apply_button,self.save_button,self.revert_button,self.reset_button): b.pack(fill='x',pady=2)
        self.advanced_open=tk.BooleanVar(value=False); ttk.Checkbutton(outer,text='Show advanced tuning',variable=self.advanced_open,command=self.toggle_advanced).pack(anchor='w',pady=4)
        self.advanced=self.card(outer,'Advanced tuning')
        for category,title in [('motion','Motion settings'),('tracking','Tracking settings'),('position','Position-mode tuning'),('engineering','Engineering / calibration')]:
            s=ttk.Frame(self.advanced,style='Card.TFrame'); ttk.Label(s,text=title,style='Section.TLabel').pack(anchor='w',pady=(4,1)); self.add_fields(s,category); s.pack(fill='x',pady=2)
        self.alignment=tk.BooleanVar(value=False); self.optical=tk.BooleanVar(value=False); ttk.Checkbutton(self.advanced,text='I verified mapping directions and tool0 in Preview',variable=self.alignment,command=self.mark_dirty).pack(anchor='w',pady=(5,1)); ttk.Checkbutton(self.advanced,text='I verified optical loss with controllers inactive',variable=self.optical,command=self.mark_dirty).pack(anchor='w'); return outer

    def add_fields(self,parent,category):
        for spec in PARAMETER_REGISTRY:
            if spec.category!=category or not spec.editable: continue
            row=ttk.Frame(parent,style='Card.TFrame'); row.pack(fill='x',pady=2); ttk.Label(row,text=spec.label+(f' ({spec.units})' if spec.units else ''),style='Card.TLabel').pack(side='left')
            v=tk.StringVar(value=str(display_value(spec,DEFAULTS[spec.key]))); v.trace_add('write',self.mark_dirty); self.fields[spec.key]=v
            w=ttk.Combobox(row,textvariable=v,values=spec.choices,state='readonly',width=13) if spec.choices else ttk.Entry(row,textvariable=v,width=13)
            if spec.key=='teleop_control_mode': w.bind('<<ComboboxSelected>>',lambda _event:self.update_mode_visibility())
            w.pack(side='right'); self.field_rows[spec.key]=row

    def _interfaces(self):
        self.clients={key:self.node.create_client(Trigger,name) for key,name in {'arm':'/vr_real/arm','disarm':'/vr_real/disarm','servo':'/vr_real/servo_node/start_servo'}.items()}; self.enable_client=self.node.create_client(SetBool,'/vr_real/set_enabled'); self.switch_client=self.node.create_client(SwitchController,'/controller_manager/switch_controller'); self.settings=self.node.create_client(SetParametersAtomically,'/vr_real/teleop/set_parameters_atomically'); self.heartbeat=self.node.create_publisher(Bool,'/vr_real/operator_present',1); self.node.create_subscription(String,'/vr_real/operator_status',self.on_status,1); self.node.create_subscription(String,'/wrist_camera/status',self.on_camera_status,1)

    def on_camera_status(self,msg): self.camera_at,self.camera_text=time.monotonic(),msg.data
    def open_camera(self):
        if self.camera_window is None or self.camera_window.closed:
            from .camera_view import CameraView; self.camera_window=CameraView(self.root,self.node)
        else: self.camera_window.window.lift()
    def on_status(self,msg):
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict): return
            new=not self.last_status or value.get('session_id')!=self.last_status.get('session_id'); self.last_status,self.last_status_at=value,time.monotonic()
            if new:
                for _,future in self.pending: future.cancel()
                self.pending.clear(); self.pending_settings.clear(); self.alignment.set(False); self.optical.set(False); self.load_running(value)
        except (ValueError,TypeError): pass
    def status_value(self,key,status): return status.get({'max_linear_speed_m_s':'linear','max_angular_speed_deg_s':'angular','max_joint_speed_rad_s':'joint_speed','tracking_gain_s_inv':'tracking_gain','tracking_to_robot_rpy_deg':'alignment_rpy_deg'}.get(key,key))
    def load_running(self,status):
        self._loading_fields=True
        try:
            for spec in PARAMETER_REGISTRY:
                value=self.status_value(spec.key,status)
                if value is not None:
                    self.running_values[spec.key]=value
                    if spec.key in self.fields: self.fields[spec.key].set(str(display_value(spec,value)))
        finally: self._loading_fields=False
        self.update_mode_visibility(); self.mark_dirty()
    def mark_dirty(self,*_args):
        if self._loading_fields:return
        try: changed=self.field_values()!={key:self.running_values.get(key,DEFAULTS[key]) for key in self.fields}
        except (ValueError,TypeError): changed=True
        self.unsaved.set('Unsaved edits — apply while disarmed with A released.' if changed else 'Editing current running values.')
    def update_mode_visibility(self):
        mode=self.fields.get('teleop_control_mode',tk.StringVar(value='velocity')).get()
        for spec in PARAMETER_REGISTRY:
            if spec.key in self.field_rows and spec.mode:
                (self.field_rows[spec.key].pack if spec.mode==mode else self.field_rows[spec.key].pack_forget)(**({'fill':'x','pady':2} if spec.mode==mode else {}))
    def toggle_advanced(self):
        if self.advanced_open.get(): self.advanced.pack(fill='both',expand=True,pady=(0,4)); self.update_mode_visibility()
        else:self.advanced.pack_forget()
    def stale(self): return time.monotonic()-self.last_status_at>=.5

    def stack_action(self,action,mode=None):
        if self.manager is None or any(w.is_alive() for w in self.workers):return
        self.heartbeat.publish(Bool(data=False))
        def run():
            try:
                if action=='stop': self.manager.message='Stopping owned ROS processes; robot faults remain a pendant procedure.'; self.manager.stop()
                elif action=='restart' or self.manager.children: self.manager.message='Restarting owned ROS processes; acknowledge pendant faults first.'; self.manager.stop(restart=self.manager.live if mode is None else mode)
                else:self.manager.start(bool(mode))
            except Exception as error:self.manager.message=str(error)
        w=threading.Thread(target=run,daemon=True); self.workers=[w]; w.start()
    def open_rviz(self):
        try:self.manager.open_rviz()
        except Exception as error:self.stack_status.set(str(error))
    def prepare_controller(self):
        d=self.last_status
        if self.stale() or d.get('armed') or d.get('a_held') or d.get('enable_requested'):self.message.set('Disable teleop and release A before preparing the controller.');return
        if not str(d.get('mode','')).startswith('LIVE'):self.message.set('Preview does not activate motion controllers. Start Servo solver for Preview.');return
        if not d.get('observations_healthy') or not d.get('stationary') or not d.get('robot_ready'):self.message.set('Fresh tracking/joints/TF, standstill, and External Control are required.');return
        if d.get('controller_ready'):self.call('servo');return
        if not self.switch_client.service_is_ready():self.message.set('Controller manager unavailable; resolve the condition then restart ROS.');return
        r=SwitchController.Request();r.activate_controllers=['scaled_joint_trajectory_controller'];r.strictness=SwitchController.Request.STRICT;r.timeout.sec=3;self.pending.append(('prepare_controller',self.switch_client.call_async(r)));self.message.set('Preparing scaled controller; teleop remains disabled.')
    def call(self,name):
        if not self.clients[name].service_is_ready():self.message.set(f'{name}: service unavailable');return
        self.pending.append((name,self.clients[name].call_async(Trigger.Request())));self.message.set(name+': request sent')
    def toggle_enabled(self):
        if self.stale() or not self.enable_client.service_is_ready():self.message.set('Enable service/status unavailable');return
        r=SetBool.Request();r.data=not bool(self.last_status.get('enable_requested',False));self.pending.append(('enable',self.enable_client.call_async(r)));self.message.set('Enable setting requested')
    def field_values(self):
        values=dict(self.running_values)
        for key,v in self.fields.items(): values[key]=stored_value(PARAMETERS_BY_KEY[key],v.get())
        return validate({key:values.get(key,DEFAULTS[key]) for key in DEFAULTS})
    def revert_edits(self):self.load_running(self.last_status);self.message.set('Edits reverted to current running values.')
    def reset_defaults(self):
        if self.last_status.get('armed') or self.last_status.get('a_held'):self.message.set('Disable teleop and release A before restoring defaults.');return
        self._loading_fields=True
        try:
            for key,v in self.fields.items():v.set(str(display_value(PARAMETERS_BY_KEY[key],DEFAULTS[key])))
        finally:self._loading_fields=False
        self.alignment.set(False);self.optical.set(False);self.update_mode_visibility();self.mark_dirty();self.message.set('Defaults are staged only. Review and Apply + save to persist.')
    def apply(self,save=False):
        try:
            values=self.field_values()
            if self.stale() or self.last_status.get('armed') or self.last_status.get('a_held'):raise ValueError('Settings require a fresh, disarmed status with A released')
            if not self.settings.service_is_ready():raise ValueError('Settings service unavailable')
            if any(l=='settings' for l,_ in self.pending):raise ValueError('Waiting for previous settings request')
            applied={key:values[key] for key in self.fields};applied.update(alignment_verified=self.alignment.get(),optical_loss_test_verified=self.optical.get());hardware=self.last_status.get('hardware_smoothing');restart=hardware and abs(values['max_joint_speed_rad_s']-hardware['velocity'])>1e-9
            if restart:
                if not save:raise ValueError('Joint speed requires Apply + save, then Restart ROS.')
                applied.pop('max_joint_speed_rad_s')
            r=SetParametersAtomically.Request();r.parameters=[Parameter(name,value=value).to_parameter_msg() for name,value in applied.items()];f=self.settings.call_async(r);self.pending_settings[f]=values if save else None;self.pending.append(('settings',f))
        except (ValueError,TypeError) as error:self.message.set(str(error))
    def complete_settings(self,future,result):
        values=self.pending_settings.pop(future,None)
        if not result.successful:self.message.set('Settings rejected: '+result.reason);return
        if values is None:self.message.set('Settings applied to this running session.');return
        try:path=save_settings(self.config_path,values)
        except (OSError,ValueError) as error:self.message.set('Settings applied but not saved: '+str(error));return
        hardware=self.last_status.get('hardware_smoothing');restart=hardware and abs(values['max_joint_speed_rad_s']-hardware['velocity'])>1e-9;self.message.set(('Settings saved; Restart ROS to apply final joint-speed limit. ' if restart else 'Settings applied and saved: ')+str(path))

    def set_ready(self,key,ok,text):
        value,w=self.readiness[key];value.set('● '+text);w.configure(foreground=COLORS['green'] if ok else COLORS['amber'])
    def update_dashboard(self,stale,d):
        if stale:
            self.header_mode.set('DISCONNECTED');self.header_state.set('NOT READY');self.message.set('No fresh backend status. Do not infer robot state; check the pendant.');self.diagnostics.set('No fresh backend status.');self.lighthouse.set('Lighthouse 0: unknown\nLighthouse 1: unknown\nValid pose: unknown\nLast rejection: status stale')
            for key in self.readiness:self.set_ready(key,False,'Status unavailable')
            return
        mode=str(d.get('mode','UNKNOWN'));self.header_mode.set('LIVE' if mode.startswith('LIVE') else 'PREVIEW' if 'DRY RUN' in mode else mode);self.header_state.set('ARMED' if d.get('armed') else 'READY TO ENABLE' if d.get('motion_ready') else 'DISARMED / NOT READY'); blockers=d.get('blockers',[]);self.message.set(blockers[0] if blockers else str(d.get('reason','Waiting for readiness')))
        self.set_ready('robot',bool(d.get('robot_ready')),'Robot and External Control ready' if d.get('robot_ready') else 'Robot / External Control not ready');self.set_ready('controller',bool(d.get('controller_ready')),'Controller ownership ready' if d.get('controller_ready') else 'Controller not ready');self.set_ready('joints',bool(d.get('observations_healthy')),'Joint / TF feedback fresh' if d.get('observations_healthy') else 'Joint / TF feedback not valid');self.set_ready('pose',bool(d.get('pose_fresh')),'Valid controller pose fresh' if d.get('pose_fresh') else 'Valid pose not fresh');self.set_ready('optical',bool(d.get('optical')),'Optical signal fresh (not pose proof)' if d.get('optical') else 'Optical signal lost');self.set_ready('alignment',self.alignment.get(),'Alignment confirmed this session' if self.alignment.get() else 'Alignment confirmation required');self.set_ready('heartbeat',True,'Operator dashboard connected');self.set_ready('confirmations',self.alignment.get() and self.optical.get(),'Required confirmations applied' if self.alignment.get() and self.optical.get() else 'Confirmations pending')
        diag=d.get('velocity_diagnostics',{});vector=lambda v:'unavailable' if v is None else ', '.join(f'{x:.3g}' for x in v[:3]);self.diagnostics.set(f"Requested TCP velocity: {vector(diag.get('published_twist'))} m/s\nMeasured TCP velocity: {vector(diag.get('measured_tcp_velocity'))} m/s\nServo: {d.get('servo_detail',d.get('servo','unknown'))} | Clutch: {d.get('clutch','unknown')}\nTracking: {'valid pose' if d.get('pose_fresh') else 'not valid'} | A: {'held' if d.get('a_held') else 'released'}")
        stations=d.get('optical_stations',{});station=lambda i:stations.get(str(i),stations.get(i,0));self.lighthouse.set(f"Lighthouse 0: {'signal' if station(0) else 'no recent signal'} ({station(0)} paired sensors)\nLighthouse 1: {'signal' if station(1) else 'no recent signal'} ({station(1)} paired sensors)\nValid pose: {'yes' if d.get('pose_fresh') else 'no'} — one station can be sufficient\nLast rejection: {d.get('tracking_rejection','none reported')}")
    def poll(self):
        if self.closed:return
        if self.shutdown_requested or not rclpy.ok():self.close();return
        rclpy.spin_once(self.node,timeout_sec=0) # Tk thread owns all widget updates.
        if self.manager:self.manager.poll();self.stack_status.set(self.manager.message)
        now=time.monotonic()
        if now-self.heartbeat_at>=.1:self.heartbeat.publish(Bool(data=True));self.heartbeat_at=now
        self.camera_status.set(self.camera_text if now-self.camera_at<4 else 'D405 unavailable / no recent status (teleop unaffected)');stale,d=self.stale(),self.last_status;self.update_dashboard(stale,d)
        busy=any(label in ('enable','prepare_controller') for label,_ in self.pending);disabled=stale or d.get('armed') or d.get('a_held')
        for b in (self.apply_button,self.save_button,self.revert_button,self.reset_button):b.state(['disabled'] if disabled else ['!disabled'])
        self.enable_button.state(['disabled'] if stale or busy else ['!disabled']);self.servo_button.state(['disabled'] if stale or d.get('armed') else ['!disabled']);self.prepare_button.state(['disabled'] if disabled or d.get('enable_requested') or busy else ['!disabled'])
        self.blockers.configure(state='normal');self.blockers.delete('1.0','end');self.blockers.insert('end','\n'.join(d.get('blockers',[])) or 'No reported readiness blockers');self.blockers.configure(state='disabled');remaining=[]
        for label,future in self.pending:
            if not future.done():remaining.append((label,future));continue
            try:
                result=future.result()
                if label=='settings':self.complete_settings(future,result.result)
                elif label=='prepare_controller':self.message.set('Controller prepared; start Servo solver next.' if result.ok else 'Controller activation failed; inspect driver log.')
                else:self.message.set(f'Last {label} response: {result.message}.')
            except Exception as error:self.message.set(f'{label}: {error}')
        self.pending=remaining;self.root.after(100,self.poll)
    def close(self):
        if self.closed:return
        self.closed=True
        if getattr(self,'camera_window',None) is not None:self.camera_window.close()
        try:
            if rclpy.ok():self.heartbeat.publish(Bool(data=False))
        finally:self.root.destroy()

def main(args=None):
    rclpy.init(args=args);gui=None
    try:gui=OperatorGUI();gui.root.mainloop()
    finally:
        if gui:gui.node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
