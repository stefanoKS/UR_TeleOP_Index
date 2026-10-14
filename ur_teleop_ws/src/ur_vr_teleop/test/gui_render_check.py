"""Actual Tk/ROS integration on localhost domain 147; no driver or robot nodes."""
import json
import os
from pathlib import Path
import tempfile
import time
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['ROS_DOMAIN_ID']='43'  # GUI launch guard; context below explicitly uses isolated 147.
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from std_msgs.msg import String
from PIL import ImageGrab
from ur_vr_teleop.gui import OperatorGUI
from ur_vr_teleop.preferences import DEFAULTS


def run():
    rclpy.init(domain_id=147)
    backend=rclpy.create_node('teleop',namespace='vr_real')
    for key,value in DEFAULTS.items():backend.declare_parameter(key,value)
    for key in ('alignment_verified','optical_loss_test_verified'):backend.declare_parameter(key,False)
    rejected=[False]
    backend.add_on_set_parameters_callback(lambda params: SetParametersResult(
        successful=not rejected[0],reason='Test rejection: A held' if rejected[0] else ''))
    gui=OperatorGUI()
    temporary=tempfile.TemporaryDirectory()
    gui.config_path=Path(temporary.name)/'config.yaml'
    gui.parameters.saved={};gui.parameters.restart={}
    publisher=backend.create_publisher(String,'/vr_real/operator_status',1)
    session=['integration-1']
    def status():
        return dict(session_id=session[0],mode='DRY RUN — ROBOT OUTPUT DISABLED',
            armed=False,a_held=False,enable_requested=False,observations_healthy=True,
            robot_ready=True,controller_ready=True,controller_state_fresh=True,servo_status_fresh=True,
            motion_ready=False,operator_heartbeat_fresh=True,configuration_ready=True,
            alignment_verified=backend.get_parameter('alignment_verified').value,
            optical_loss_test_verified=backend.get_parameter('optical_loss_test_verified').value,
            effective_parameters={k:list(backend.get_parameter(k).value) if isinstance(v,list) else backend.get_parameter(k).value for k,v in DEFAULTS.items()},
            hardware_smoothing={'velocity':.5},optical=True,optical_stations={'0':0,'1':1},pose_fresh=True,
            tracking_rejection='none',servo_detail='Ready',servo_state='running',servo_process_present=True,
            servo_status_seen=True,clutch='READY',teleop_control_mode='velocity',
            velocity_diagnostics={'published_twist':[0.]*6,'measured_tcp_velocity':None},
            motion_metrics_current=False,blockers=['External Control program is not running'],reason='Disarmed')
    def pump(seconds=.4,publish=True):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            if publish:publisher.publish(String(data=json.dumps(status())))
            rclpy.spin_once(backend,timeout_sec=0)
            gui.root.update()
            time.sleep(.01)
    try:
        pump(1.5)
        gui.open_camera();pump(.1)
        assert not gui.camera_window.closed
        gui.camera_window.close()
        assert gui.camera_window.closed
        assert gui.parameters.confirmed
        assert gui.unsaved.get()=='No pending edits'
        gui.alignment.set(True);gui.mark_dirty();pump()
        assert 'Pending application' in gui.readiness['alignment'][0].get()
        assert 'Verified' not in gui.readiness['alignment'][0].get()
        gui.fields['max_linear_speed_m_s'].set('.2')
        gui.apply(save=True)
        assert gui.parameters.effective['max_linear_speed_m_s']==.1
        pump()
        assert gui.parameters.effective['max_linear_speed_m_s']==.2
        assert gui.unsaved.get()=='No pending edits'
        assert not backend.get_parameter('alignment_verified').value  # Tuning no longer sends approvals.
        assert json.loads((Path(temporary.name)/'operator_settings.json').read_text())['parameters']['max_linear_speed_m_s']==.2
        result=gui.message.get();pump()
        assert result==gui.message.get() and 'confirmed' in result
        rejected[0]=True
        gui.fields['max_linear_speed_m_s'].set('.3');gui.apply();pump()
        assert gui.fields['max_linear_speed_m_s'].get()=='.3'
        assert gui.parameters.effective['max_linear_speed_m_s']==.2
        assert 'Test rejection' in gui.message.get()
        gui.revert_edits();assert float(gui.fields['max_linear_speed_m_s'].get())==.2
        rejected[0]=False
        # External update preserves a conflicting draft and blocks accidental overwrite.
        gui.fields['max_linear_speed_m_s'].set('.4')
        backend.set_parameters_atomically([Parameter('max_linear_speed_m_s',value=.25)])
        pump()
        assert gui.parameters.conflicts and gui.fields['max_linear_speed_m_s'].get()=='.4'
        gui.apply();assert 'External' in gui.message.get()
        gui.alignment.set(True);gui.optical.set(True)
        gui.revert_edits();assert float(gui.fields['max_linear_speed_m_s'].get())==.25
        assert gui.alignment.get() and gui.optical.get()
        # File failure after acceptance retains the applied value and previous saved file.
        import ur_vr_teleop.gui as gui_module
        original_save=gui_module.save_settings
        def fail_save(*args):raise OSError('Test disk full')
        try:
            gui_module.save_settings=fail_save
            gui.fields['max_linear_speed_m_s'].set('.26');gui.apply(save=True);pump()
            assert gui.parameters.effective['max_linear_speed_m_s']==.26
            assert float(gui.fields['max_linear_speed_m_s'].get())==.26
            assert 'Applied to current session, but saving failed' in gui.message.get()
            assert json.loads((Path(temporary.name)/'operator_settings.json').read_text())['parameters']['max_linear_speed_m_s']==.2
        finally:
            gui_module.save_settings=original_save
        gui.fields['max_linear_speed_m_s'].set('.25');gui.apply(save=True);pump()
        # Mode hiding preserves text; defaults stage only.
        gui.fields['tracking_gain_s_inv'].set('3.1')
        gui.fields['teleop_control_mode'].set('position');gui.update_mode_visibility()
        gui.fields['teleop_control_mode'].set('velocity');gui.update_mode_visibility()
        assert gui.fields['tracking_gain_s_inv'].get()=='3.1'
        gui.reset_defaults()
        assert backend.get_parameter('max_linear_speed_m_s').value==.25
        gui.revert_edits()
        # Restart settings stay separate from active controller values.
        gui.fields['max_joint_speed_rad_s'].set('.7');gui.apply(save=True);pump()
        assert gui.parameters.effective['max_joint_speed_rad_s']==.5
        assert gui.parameters.restart=={'max_joint_speed_rad_s':.7}
        assert 'Restart required' in gui.unsaved.get()
        gui.revert_edits();assert gui.parameters.restart
        # Session changes cancel old futures and clear manual confirmation drafts.
        late=Future();gui.track_request('enable',late,True)
        gui.settings_result='Old session success must not appear'
        session[0]='integration-2'
        backend.set_parameters_atomically([Parameter('alignment_verified',value=False)])
        pump()
        assert late.cancelled() and not gui.alignment.get()
        assert 'Old session success' not in gui.message.get()
        rejected[0]=True;gui.alignment.set(True);gui.apply();pump()
        assert 'Not verified — request rejected' in gui.readiness['alignment'][0].get()
        rejected[0]=False;gui.alignment.set(False)
        # A held with the gate disarmed must never be shown as an active robot clutch.
        gui.update_dashboard(False,dict(status(),armed=False,a_held=True,clutch='ACTIVE',motion_ready=False))
        assert 'Clutch (effective): DISARMED' in gui.diagnostics.get()
        assert 'A held, no motion authorization' in gui.diagnostics.get()
        assert 'A button (VR input): held' in gui.diagnostics.get()
        assert 'Backend motion authorization: disarmed' in gui.diagnostics.get()
        gui.update_dashboard(False,dict(status(),armed=True,a_held=True,clutch='ACTIVE',motion_ready=True))
        assert 'Clutch (effective): ACTIVE' in gui.diagnostics.get()
        # Servo lifecycle is displayed explicitly and a stale status is never healthy.
        gui.update_dashboard(False,dict(status(),servo_status_fresh=False,servo_state='stale'))
        assert 'Stale — started but no fresh status' in gui.diagnostics.get()
        gui.update_dashboard(False,dict(status(),servo_status_fresh=False,servo_state='stopped'))
        assert 'Stopped — process no longer publishing status' in gui.diagnostics.get()
        gui.update_dashboard(False,dict(status(),servo_status_fresh=False,servo_state='absent'))
        assert 'Absent — Servo process not running' in gui.diagnostics.get()
        gui.update_dashboard(False,status())
        # Distinct backend session IDs are counted and surfaced as a duplicate-backend risk.
        session[0]='integration-restart'
        backend.set_parameters_atomically([Parameter('alignment_verified',value=False)])
        pump()
        assert 'integration-restart' in gui.session_ids and gui.session_changes>=1
        assert gui.startup_stage.get()=='SESSION CHANGED — PRESS START TELEOP'
        assert not gui.startup.requested
        # Frequent session changes are reported as a duplicate-backend risk.
        session[0]='integration-restart-2'
        backend.set_parameters_atomically([Parameter('alignment_verified',value=False)])
        pump()
        assert len(set(gui.session_ids))>=2 and gui.session_changes>=2
        assert 'backend session IDs' in gui.session_warning()
        # Malformed optional diagnostics remain unknown, never zero or green.
        malformed=dict(status(),velocity_diagnostics=['bad'],optical_stations=None,pose_fresh=None,alignment_verified=None)
        gui.update_dashboard(False,malformed)
        assert 'Unavailable' in gui.diagnostics.get() and 'WAITING' in gui.lighthouse.get()
        assert 'Unknown' in gui.readiness['alignment'][0].get()
        gui.update_dashboard(False,dict(status(),controller_state_fresh=False,servo_status_fresh=False))
        assert 'unavailable' in gui.readiness['controller'][0].get()
        assert 'Stale' in gui.diagnostics.get()
        gui.last_status_at=time.monotonic()-1;gui.update_dashboard(True,status())
        assert 'Stale' in gui.motion.get()
        assert all('Unknown' in value.get() for value,_ in gui.readiness.values())
        pump()
        # Exercise one-button preparation using real async ROS services, no robot.
        from controller_manager_msgs.srv import SwitchController
        from std_srvs.srv import Trigger, SetBool
        backend.set_parameters_atomically([Parameter('max_joint_speed_rad_s',value=.7)])
        state=dict(hardware_smoothing={'velocity':.7,'acceleration':1.,'jerk':10.},controller_ready=False,controller_preparation_allowed=True,
                   servo_status_fresh=False,servo_state='unstarted',servo_process_present=True,servo_status_seen=False,
                   motion_ready=False,enable_requested=False,armed=False)
        calls=[]
        original_status=status
        def status():
            return dict(original_status(),**dict(state,mode='LIVE OUTPUT',tracking_valid=True,
                active_station_count=1,configuration_ready=True,robot_preparation_ready=True,
                stationary=True,a_held=False,blockers=[],reason='Synthetic backend — no robot connected'))
        def prepare(req,res):
            assert req.activate_controllers==['scaled_joint_trajectory_controller'] and not req.deactivate_controllers
            calls.append('prepare');state['controller_ready']=True;res.ok=True;return res
        def servo(req,res):
            calls.append('servo');state.update(servo_status_fresh=True,motion_ready=True)
            res.success=True;res.message='Synthetic Servo started';return res
        def enable(req,res):
            calls.append('enable');state.update(enable_requested=req.data,armed=req.data)
            res.success=True;res.message='Synthetic backend enabled';return res
        def disarm(req,res):
            calls.append('disarm');state.update(enable_requested=False,armed=False)
            res.success=True;res.message='Synthetic backend disarmed';return res
        services=[backend.create_service(SwitchController,'/controller_manager/switch_controller',prepare),
                  backend.create_service(Trigger,'/vr_real/servo_node/start_servo',servo),
                  backend.create_service(SetBool,'/vr_real/set_enabled',enable),
                  backend.create_service(Trigger,'/vr_real/disarm',disarm)]
        pump(.8);assert not gui.parameters.restart
        gui.start_teleop();gui.start_teleop();pump(2)
        assert calls==['prepare','servo','enable'],calls
        assert gui.startup.state=='READY' and gui.header_state.get()=='READY'
        assert not gui.servo_button.winfo_viewable() and not gui.prepare_button.winfo_viewable()
        gui.stop_teleop();pump(.5)
        assert not state['enable_requested'] and not gui.startup.requested
        calls.clear();gui.start_teleop();pump(1)
        assert calls==['enable'],calls
        output=Path('/tmp/ur_teleop_ui_v4');output.mkdir(exist_ok=True)
        gui.dismiss_notification()
        gui.notify('ISOLATED GUI TEST — synthetic status, no robot connected')
        gui.notifications.expires=float('inf')
        for width,height in ((1440,900),(1366,768),(1280,720)):
            gui.root.geometry(f'{width}x{height}+0+100')
            gui.advanced_open.set(True);gui.toggle_advanced();pump()
            gui.root.lift();gui.root.update()
            for button in (gui.enable_button,gui.disarm_button,gui.apply_button,gui.save_button,gui.revert_button,gui.reset_button):
                assert button.winfo_viewable()
                assert gui.root.winfo_containing(button.winfo_rootx()+5,button.winfo_rooty()+5)==button
                top=button.winfo_rooty()-gui.root.winfo_rooty()
                assert 0<=top and top+button.winfo_height()<=height,(width,height,button.cget('text'),top,button.winfo_height())
            assert gui.stop_notice.winfo_height()>=gui.stop_notice.winfo_reqheight()
            assert gui.stop_notice.winfo_rooty()+gui.stop_notice.winfo_height()<=gui.root.winfo_rooty()+height
            assert gui.canvas.winfo_height()>60,(width,height,gui.canvas.winfo_height())
            before=gui.disarm_button.winfo_rooty()
            gui.canvas.yview_moveto(1);pump(.1)
            assert gui.disarm_button.winfo_rooty()==before
            gui.advanced_open.set(False);gui.toggle_advanced();pump(.1)
            gui.advanced_open.set(True);gui.toggle_advanced();pump(.1)
            last=gui.field_widgets['tracking_to_robot_rpy_deg']
            last.focus_force();pump(.1)
            assert gui.canvas.winfo_rooty()<=last.winfo_rooty()<gui.canvas.winfo_rooty()+gui.canvas.winfo_height()
            if width==1280:
                x,y=gui.root.winfo_rootx(),gui.root.winfo_rooty()
                ImageGrab.grab(bbox=(x,y,x+width,y+height)).save(output/'dashboard-1280x720-advanced-bottom.png')
            gui.canvas.yview_moveto(0);pump(.1)
            x,y=gui.root.winfo_rootx(),gui.root.winfo_rooty()
            ImageGrab.grab(bbox=(x,y,x+width,y+height)).save(output/f'dashboard-{width}x{height}-advanced.png')
            gui.advanced_open.set(False);gui.toggle_advanced();pump(.1)
            ImageGrab.grab(bbox=(x,y,x+width,y+height)).save(output/f'dashboard-{width}x{height}.png')
        # Recording controls are read-only and available while the main STOP stays visible.
        gui.engineering_open.set(True);gui.toggle_engineering();pump(.2)
        gui.show_history();pump(.2)
        windows=[w for w in gui.root.winfo_children() if w.winfo_class()=='Toplevel']
        diagnostics=next(w for w in windows if w.title()=='Advanced diagnostics / recent operator actions')
        x,y=diagnostics.winfo_rootx(),diagnostics.winfo_rooty()
        ImageGrab.grab(bbox=(x,y,x+diagnostics.winfo_width(),y+diagnostics.winfo_height())).save(output/'sway-diagnostics.png')
        diagnostics.destroy()
        gui.engineering_open.set(False);gui.toggle_engineering()
        # Missing historical setup never opens the optional wizard.
        assert gui.setup_wizard is None
        previous_status = status
        setup_state = dict(engineering_required=True, engineering_verified=True,
                           setup_checked=True, setup_context='{}', setup_feedback_fresh=True,
                           base_frame='base_link', tool_frame='tool0',
                           velocity_diagnostics=dict(previous_status()['velocity_diagnostics'],
                               raw_pose=[.1,.2,1.,0.,0.,0.,1.], measured_tcp_position=[.4,.1,.5],
                               measured_tcp_orientation=[0.,0.,0.,1.]))
        def status():
            return dict(previous_status(), **setup_state)
        pump(.3)
        assert gui.setup_wizard is None
        setup_state.update(engineering_verified=False, setup_checked=False)
        pump(.3)
        assert gui.setup_wizard is None
        setup_state['setup_checked'] = True
        pump(.3)
        assert gui.setup_wizard is None  # Missing history never opens a wizard.
        gui.open_setup()  # Optional engineering tool is explicit.
        assert gui.setup_wizard is not None
        wizard = gui.setup_wizard
        assert not wizard.checks.results
        wizard.save()
        assert not (gui.config_path.parent/'verified_setup.json').exists()
        # Actual Tk windows, synthetic telemetry, no automatic physical checks.
        for index, name in enumerate(('connection-mapping', 'supervised-stops', 'save-setup')):
            wizard.tabs.select(index)
            wizard.window.geometry('1000x680+50+50')
            wizard.window.lift();pump(.2)
            def children(widget):
                for child in widget.winfo_children():
                    yield child
                    yield from children(child)
            stops=[w for w in children(wizard.window) if w.winfo_class()=='TButton' and w.cget('text')=='STOP TELEOP']
            assert len(stops)==1 and stops[0].winfo_viewable()
            assert stops[0].winfo_rooty()+stops[0].winfo_height() <= wizard.window.winfo_rooty()+680
            x,y=wizard.window.winfo_rootx(),wizard.window.winfo_rooty()
            ImageGrab.grab(bbox=(x,y,x+1000,y+680)).save(output/('setup-'+name+'.png'))
        # Feed genuine synthetic multi-sample measurements into the Tk-only adapter.
        # The real observer remains isolated and cannot activate a command path.
        from test_calibration_capture import Stream
        from ur_vr_teleop.calibration_capture import STEPS
        import numpy as np
        synthetic=Stream()
        wizard.baseline={}
        wizard.baseline_parameters=status()['effective_parameters']
        panel=wizard.calibration_panel
        for i in range(6):
            axis=np.eye(3)[i%3]
            synthetic.capture(i,delta=axis*.04 if i<3 else None,angle=axis*.3 if i>=3 else None)
            synthetic.run(.7)
            snapshot=synthetic.c.snapshot()
            snapshot.update(session_id=gui.session,parameters=wizard.baseline_parameters)
            panel.update(snapshot)
        assert len(wizard.checks.results)==6
        assert not any(r['physical_pass'] for r in wizard.checks.results.values())
        assert panel.notebook.index(panel.notebook.select())==1
        assert all(panel.table.item(name,'values')[-1]=='Pass' for name in STEPS)
        panel.review()
        assert all(wizard.checks.results[name]['physical_pass'] for name in STEPS)
        assert not wizard.preview_complete()  # Separate optical and Live tests remain required.
        panel.cannot_identify()
        assert not any(wizard.checks.results[name]['physical_pass'] for name in STEPS)
        panel.review()  # A subsequent explicit review is still required.
        wizard.save()
        assert not (gui.config_path.parent/'verified_setup.json').exists()
        wizard.tabs.select(0)
        wizard.message.set('Synthetic capture demonstration — physical commissioning remains required.')
        review_snapshot=snapshot
        in_progress=Stream();in_progress.held=True;in_progress.run(.4);in_progress.move([.04,0,0])
        guided_snapshot=in_progress.c.snapshot()
        guided_snapshot.update(session_id=gui.session,parameters=wizard.baseline_parameters)
        for width,height in ((1000,680),(900,650)):
            wizard.window.geometry(f'{width}x{height}+50+50')
            for tab,label in ((0,'guided'),(1,'review'),(2,'optical')):
                panel.update(guided_snapshot if tab==0 else review_snapshot)
                panel.notebook.select(tab);wizard.window.lift();pump(.2)
                assert stops[0].winfo_rooty()+stops[0].winfo_height() <= wizard.window.winfo_rooty()+height
                x,y=wizard.window.winfo_rootx(),wizard.window.winfo_rooty()
                ImageGrab.grab(bbox=(x,y,x+width,y+height)).save(output/f'calibration-{label}-{width}x{height}.png')
        panel.table.selection_set('mapping_x');panel.retry_selected()
        assert 'mapping_x' not in wizard.checks.results
        assert not any(r['physical_pass'] for r in wizard.checks.results.values())
        gui.stop_teleop();assert not wizard.capture_enabled
        assert panel.snapshot['state']=='CANCELLED'
        old_observer=wizard.observer
        wizard.close();pump(.2)
        assert not old_observer.thread.is_alive()
        gui.open_setup();new_observer=gui.setup_wizard.observer
        assert new_observer is not old_observer
        gui.setup_wizard.close();assert not new_observer.thread.is_alive()
        assert gui.setup_wizard is None  # Closing does not reopen on every status tick.
        # Close cancels futures and prevents late widget callbacks.
        pending=Future();gui.track_request('servo',pending)
        gui.close();assert pending.cancelled()
        gui.poll()
        print('PASS: Tk/ROS Apply, Save, rejection, readback, conflict, defaults, modes, restart, stale status, close, and three layouts;',output)
    finally:
        gui.close();gui.node.destroy_node();backend.destroy_node();rclpy.shutdown();temporary.cleanup()


if __name__=='__main__':run()
