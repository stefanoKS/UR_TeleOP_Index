"""Isolated ROS lifecycle integration with fake serial, never a hardware driver."""
import json
import tempfile
import time
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool,String
from std_srvs.srv import SetBool, Trigger
from ur_bus_servo_gripper.gripper_node import GripperNode, MetadataExecutor
from ur_bus_servo_gripper.gripper_worker import Worker
from ur_bus_servo_gripper.gripper_config import Config,save
from ur_bus_servo_gripper.st3215_driver import Feedback


class FakeDriver:
    writes=[]
    temperature=25
    def __init__(self,config):
        self.config=config;self.servo_id=7;self.identity={'servo_id':7}
    def connect(self,cancelled):return self.read()
    def read(self):return Feedback(1650,0,0,12.,self.temperature,0,0,False,time.monotonic())
    def read_temperature(self):return 35,time.monotonic()
    def write(self,ticks,calibration=False):self.writes.append(ticks)
    def close(self):pass


def run():
    rclpy.init(domain_id=147)
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory)/'g.json'
        save(path,Config(enabled=True,servo_id=7,open_ticks=1000,closed_ticks=2300,min_ticks=990,max_ticks=2310,deadzone=0.))
        fixture=rclpy.create_node('gripper_fixture')
        node=GripperNode(lambda unused:Worker(path,FakeDriver))
        ex=MetadataExecutor();ex.add_node(node);ex.add_node(fixture)
        pubs={key:fixture.create_publisher(typ,topic,1) for key,typ,topic in [
            ('joy',Joy,'/vr/controller/joy'),('pose',Bool,'/vr/controller/pose_fresh'),
            ('optical',Bool,'/vr/controller/optical_fresh'),
            ('heartbeat',Bool,'/gripper/operator_present')]}
        enable=fixture.create_client(SetBool,'/gripper/set_enabled')
        state=dict(raw=.5,enabled=False,valid=True,stale=False,heartbeat=True)
        latest={}
        fixture.create_subscription(String,'/gripper/status',lambda msg:latest.update(json.loads(msg.data)),1)
        def set_enabled(value,expect=True):
            deadline=time.monotonic()+2
            while not enable.service_is_ready() and time.monotonic()<deadline:ex.spin_once(timeout_sec=.01)
            future=enable.call_async(SetBool.Request(data=value))
            while not future.done() and time.monotonic()<deadline:ex.spin_once(timeout_sec=.01)
            assert future.done(), latest
            result=future.result()
            assert result.success is expect, (result.message, latest)
            return result
        def publish():
            pubs['pose'].publish(Bool(data=state['valid']));pubs['optical'].publish(Bool(data=state['valid']))
            pubs['heartbeat'].publish(Bool(data=state['heartbeat']))
            msg=Joy();msg.header.stamp=fixture.get_clock().now().to_msg()
            if state['stale']:msg.header.stamp.sec-=1
            msg.axes=[0.,state['raw'] if state['valid'] else 0.];msg.buttons=[0]*16
            pubs['joy'].publish(msg)
        fixture.create_timer(.01,publish)
        def pump(seconds):
            until=time.monotonic()+seconds
            while time.monotonic()<until:ex.spin_once(timeout_sec=.002)
        try:
            pump(.6)
            assert latest.get('connected'),latest
            assert not FakeDriver.writes
            set_enabled(True);state['enabled']=True;pump(.3);state['raw']=.6;pump(.2);state['raw']=.5;pump(.2)
            assert latest.get('following'),latest
            state['raw']=1.;pump(.12)
            assert FakeDriver.writes and max(FakeDriver.writes)<=1670
            state['valid']=False;pump(.15);count=len(FakeDriver.writes);pump(.15)
            assert len(FakeDriver.writes)==count
            state.update(valid=True,raw=0.);pump(.2)
            assert not latest.get('following') and len(FakeDriver.writes)==count
            state['raw']=.5;pump(.2);state['raw']=1.;pump(.1)
            assert latest.get('following')
            state['stale']=True;pump(.15);count=len(FakeDriver.writes);pump(.15)
            assert len(FakeDriver.writes)==count and not latest.get('input_fresh')
            set_enabled(False);state.update(stale=False,enabled=False);pump(.2)
            assert len(FakeDriver.writes)==count
            # A late ENABLE after the heartbeat withdrawal must be rejected.
            state['heartbeat']=False;pump(.35)
            rejected=set_enabled(True,expect=False)
            assert 'heartbeat' in rejected.message.lower(), rejected.message
            state['heartbeat']=True;pump(.25)
            set_enabled(True);set_enabled(False)
            assert latest['schema_version']==2 and 'serial_diagnostics' in latest
            assert latest['fault_category']=='NONE'
            assert node.count_publishers('/gripper/status')==1
            FakeDriver.temperature=107
            deadline=time.monotonic()+1
            while latest.get('temperature',{}).get('state')!='TEMPERATURE_UNVERIFIED' and time.monotonic()<deadline:
                ex.spin_once(timeout_sec=.002)
            assert latest['temperature']['state']=='TEMPERATURE_UNVERIFIED',latest
            count=len(FakeDriver.writes)
            FakeDriver.temperature=36
            pump(.2)
            assert latest['temperature']['state']=='RECOVERY_REQUIRED',latest
            assert not latest['session_enabled'] and not latest['authorized']
            assert latest['last_temperature_event']['feedback']['temperature_c']==107
            assert len(FakeDriver.writes)==count
            set_enabled(True,expect=False)
            connect=fixture.create_client(Trigger,'/gripper/connect')
            assert connect.wait_for_service(timeout_sec=1.)
            connect.call_async(Trigger.Request());pump(.2)
            assert latest['temperature']['state']=='NORMAL'
            state['raw']=.5;set_enabled(True);pump(.2)
            assert len(FakeDriver.writes)==count
            state['raw']=.6;pump(.2);state['raw']=.5;pump(.2)
            assert latest['following'],latest
            state['raw']=1.;pump(.12)
            assert len(FakeDriver.writes)>count
            set_enabled(False)
            assert not node.get_publishers_info_by_topic('/scaled_joint_trajectory_controller/joint_trajectory')
            print('ROS gripper lifecycle passed: independent enable, pickup, loss, recovery, stale stamps, STOP, heartbeat-gated enable, thermal status/inhibition/reconnect/pickup; no arm publisher')
        finally:
            ex.remove_node(node);node.destroy_node();fixture.destroy_node();ex.shutdown();rclpy.shutdown()


if __name__=='__main__':run()
