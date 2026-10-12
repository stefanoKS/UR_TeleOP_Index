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
            motion_ready=False,operator_heartbeat_fresh=True,
            alignment_verified=backend.get_parameter('alignment_verified').value,
            optical_loss_test_verified=backend.get_parameter('optical_loss_test_verified').value,
            effective_parameters={k:list(backend.get_parameter(k).value) if isinstance(v,list) else backend.get_parameter(k).value for k,v in DEFAULTS.items()},
            hardware_smoothing={'velocity':.5},optical=True,optical_stations={'0':0,'1':1},pose_fresh=True,
            tracking_rejection='none',servo_detail='Ready',clutch='READY',teleop_control_mode='velocity',
            velocity_diagnostics={'published_twist':[0.]*6,'measured_tcp_velocity':None},
            motion_metrics_current=False,blockers=['Confirm mapping and optical-loss test'],reason='Disarmed')
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
        assert 'Verified' in gui.readiness['alignment'][0].get()
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
        # Malformed optional diagnostics remain unknown, never zero or green.
        malformed=dict(status(),velocity_diagnostics=['bad'],optical_stations=None,pose_fresh=None,alignment_verified=None)
        gui.update_dashboard(False,malformed)
        assert 'Unavailable' in gui.diagnostics.get() and 'Unavailable' in gui.lighthouse.get()
        assert 'Unknown' in gui.readiness['alignment'][0].get()
        gui.update_dashboard(False,dict(status(),controller_state_fresh=False,servo_status_fresh=False))
        assert 'unavailable' in gui.readiness['controller'][0].get()
        assert 'Stale' in gui.diagnostics.get()
        gui.last_status_at=time.monotonic()-1;gui.update_dashboard(True,status())
        assert 'Stale' in gui.motion.get()
        assert all('Unknown' in value.get() for value,_ in gui.readiness.values())
        pump()
        output=Path('/tmp/ur_teleop_ui_v3_1');output.mkdir(exist_ok=True)
        gui.dismiss_notification()
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
            ImageGrab.grab(bbox=(x,y,x+width,y+height)).save(output/f'dashboard-{width}x{height}.png')
        # Close cancels futures and prevents late widget callbacks.
        pending=Future();gui.track_request('servo',pending)
        gui.close();assert pending.cancelled()
        gui.poll()
        print('PASS: Tk/ROS Apply, Save, rejection, readback, conflict, defaults, modes, restart, stale status, close, and three layouts;',output)
    finally:
        gui.close();gui.node.destroy_node();backend.destroy_node();rclpy.shutdown();temporary.cleanup()


if __name__=='__main__':run()
