"""Actual observer + managed Preview backend on localhost 147; synthetic inputs only."""
import json
import os
from pathlib import Path
import tempfile
import time
from unittest.mock import Mock
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Bool, String
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from ur_vr_teleop.calibration_observer import CalibrationObserver, COMMAND
from ur_vr_teleop.commissioning import Commissioning
from ur_vr_teleop.validation import JOINTS
from ur_vr_teleop.desktop import ProcessManager


def run():
    os.environ.update(ROS_DOMAIN_ID='43',ROS_LOCALHOST_ONLY='1',UR_TELEOP_MANAGED_SESSION='isolated-calibration-test')
    rclpy.init(domain_id=147)
    backend=Commissioning();fixture=Node('calibration_fixture')
    ex=SingleThreadedExecutor();ex.add_node(backend);ex.add_node(fixture)
    observer=None
    # Inspect the real managed Preview command generation, without starting its processes.
    with tempfile.TemporaryDirectory() as directory:
        manager=ProcessManager(str(Path(__file__).resolve().parents[4]/'ur_teleop/config/real_teleop.yaml'))
        manager.directory=Path(directory);manager.engineering_session=True
        commands=manager.commands(False)
        assert 'motion_output_enabled:=false' in commands['teleop'] and 'engineering_session:=true' in commands['teleop']
        assert 'activate_joint_controller:=false' in commands['driver']
    assert backend.command_pub is None and backend.get_parameter('motion_output_enabled').value is False
    def transform(*args):
        msg=TransformStamped();msg.header.stamp=fixture.get_clock().now().to_msg()
        msg.transform.translation.x=.4;msg.transform.rotation.w=1.;return msg
    backend.tf_buffer.lookup_transform=Mock(side_effect=transform)
    state={'held':False,'p':np.zeros(3),'valid':True,'pose':True,'active_controller':False}
    counts={'pose':0,'joy':0,'nonzero_twist':0}
    def list_controllers(req,res):
        c=ControllerState(name='scaled_joint_trajectory_controller',state='active' if state['active_controller'] else 'inactive')
        c.claimed_interfaces=['shoulder_pan_joint/position'] if state['active_controller'] else []
        res.controller=[c];return res
    fixture.create_service(ListControllers,'/controller_manager/list_controllers',list_controllers)
    posepub=fixture.create_publisher(PoseStamped,'/vr/controller/pose',64)
    joypub=fixture.create_publisher(Joy,'/vr/controller/joy',32)
    jointpub=fixture.create_publisher(JointState,'/joint_states',10)
    flagpub=fixture.create_publisher(Bool,'/vr/controller/pose_fresh',32)
    opticalpub=fixture.create_publisher(Bool,'/vr/controller/optical_fresh',32)
    detailpub=fixture.create_publisher(String,'/vr/controller/optical_status',10)
    def twist(msg):
        if any(abs(x)>1e-12 for x in (msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z,msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z)):
            counts['nonzero_twist']+=1
    fixture.create_subscription(TwistStamped,'/vr_real/servo_node/delta_twist_cmds',twist,10)
    def publish():
        now=fixture.get_clock().now().to_msg()
        flagpub.publish(Bool(data=state['valid']));opticalpub.publish(Bool(data=state['valid']))
        detailpub.publish(String(data='Gen2 accepted paired sensors (250 ms): LH0=0 LH1=1'))
        msg=Joy();msg.header.stamp=now;msg.buttons=[0]*16;msg.axes=[0.]*12
        msg.buttons[4]=int(state['held'] and state['valid']);joypub.publish(msg);counts['joy']+=1
        if state['pose']:
            msg=PoseStamped();msg.header.stamp=now;msg.header.frame_id='lighthouse'
            p=state['p']+np.array([np.sin(time.monotonic()*83)*1e-5,0,0])
            msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=map(float,p)
            msg.pose.orientation.w=1.;posepub.publish(msg);counts['pose']+=1
        msg=JointState();msg.header.stamp=now;msg.name=list(JOINTS);msg.position=[0.]*6;msg.velocity=[0.]*6;jointpub.publish(msg)
    fixture.create_timer(.01,publish)
    latest={}
    def pump(seconds=0,until=None,timeout=5):
        end=time.monotonic()+(timeout if until else seconds)
        while time.monotonic()<end:
            ex.spin_once(timeout_sec=.002)
            snapshot=observer.latest() if observer else None
            if snapshot:latest.clear();latest.update(snapshot)
            if observer and observer.failure:raise AssertionError(observer.failure)
            if until and until():return
        if until:raise AssertionError(latest)
    try:
        observer=CalibrationObserver();observer.configure('isolated-calibration-test',True)
        pump(until=lambda:latest.get('state')=='READY_FOR_CAPTURE')
        start_count=latest['received'];sent=counts['pose'];t0=time.monotonic()
        state['held']=True;pump(until=lambda:latest.get('state')=='RECORDING')
        for i in range(60):state['p'][0]=.05*(i+1)/60;pump(.01)
        pump(until=lambda:latest.get('state')=='WAITING_FOR_END_STABILITY')
        state['held']=False;pump(until=lambda:bool(latest.get('results')))
        report=latest['results']['mapping_x'];assert report['passed'] and report['accepted_samples']>50
        elapsed=time.monotonic()-t0;received=latest['received']-start_count
        throughput={'published_pose_samples':counts['pose']-sent,'received_pose_samples':received,
                    'elapsed_s':elapsed,'receipt_hz':received/elapsed}
        assert throughput['receipt_hz']>60,throughput
        assert received >= .9*(counts['pose']-sent)-3,throughput
        assert len(observer.capture.samples)<=observer.capture.cfg.maximum_samples
        # A synthetic bridge zero on loss cannot complete a second result.
        pump(.7);state['held']=True;pump(until=lambda:latest.get('state')=='RECORDING')
        state['valid']=False;pump(.25);state['valid']=True;state['held']=False;pump(.7)
        assert len(latest['results'])==1
        # Missing source samples abort even while validity flags keep arriving.
        state['held']=True;pump(until=lambda:latest.get('state')=='RECORDING')
        state['pose']=False;pump(.25)
        assert latest['state'] not in ('RECORDING','WAITING_FOR_END_STABILITY','PASSED')
        state['pose']=True;pump(.6)
        assert latest['state']=='WAITING_FOR_BUTTON_RELEASE' and len(latest['results'])==1
        state['held']=False;pump(.2)
        # Graph detection rejects even an idle conflicting command publisher.
        conflict=fixture.create_publisher(JointTrajectory,COMMAND,1)
        pump(until=lambda:not latest.get('isolated'))
        assert not observer.capture.capturing
        fixture.destroy_publisher(conflict)
        pump(until=lambda:latest.get('state')=='READY_FOR_CAPTURE')
        # Losing controller isolation and resuming with A held never starts capture.
        state['active_controller']=True;pump(until=lambda:not latest.get('isolated'))
        state['held']=True;state['active_controller']=False;pump(.8)
        assert latest['state']=='WAITING_FOR_BUTTON_RELEASE'
        assert not backend.gate.armed and not backend.enable_requested
        assert fixture.count_publishers(COMMAND)==0 and counts['nonzero_twist']==0
        assert not any('set_enabled' in name or 'switch_controller' in name for name,_ in observer.node.get_client_names_and_types_by_node('preview_calibration_observer','/vr_real'))
        threads=[]
        for _ in range(2):
            observer.close();threads.append(observer.thread)
            assert not observer.thread.is_alive()
            observer=CalibrationObserver();observer.configure('isolated-calibration-test',True)
            state['held']=False;pump(until=lambda:latest.get('state')=='READY_FOR_CAPTURE')
            pump(.15)
            assert fixture.count_subscribers('/vr/controller/pose')==2  # backend + one observer
        observer.close();assert all(not t.is_alive() for t in threads)
        output=Path('/tmp/ur_teleop_calibration_v51');output.mkdir(exist_ok=True)
        (output/'synthetic-capture.json').write_text(json.dumps(dict(synthetic_only=True,quality=report,throughput=throughput),indent=2))
        print('PASS: real Preview parameter, managed command generation, read-only observer, A capture, loss, isolation, cleanup; '+json.dumps(throughput))
    finally:
        if observer:observer.close()
        ex.shutdown();backend.destroy_node();fixture.destroy_node();rclpy.shutdown()

if __name__=='__main__':run()
