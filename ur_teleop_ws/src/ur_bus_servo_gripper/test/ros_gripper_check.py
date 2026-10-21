"""Isolated ROS lifecycle integration with fake serial, never a hardware driver."""
import json
import tempfile
import subprocess
import os
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
    bridge = subprocess.Popen([str(Path(__file__).resolve().parents[3] / 'build/ur_vr_bridge/trigger_input_fixture')],
                              env=dict(os.environ, ROS_DOMAIN_ID='147', ROS_LOCALHOST_ONLY='1'))
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory)/'g.json'
        save(path,Config(enabled=True,servo_id=7,open_ticks=1000,closed_ticks=2300,min_ticks=990,max_ticks=2310,deadzone=0.))
        fixture=rclpy.create_node('gripper_fixture')
        node=GripperNode(lambda unused:Worker(path,FakeDriver))
        ex=MetadataExecutor();ex.add_node(node);ex.add_node(fixture)
        pubs={key:fixture.create_publisher(String if key == 'events' else Bool,topic,10) for key,topic in [
            ('events','/test/trigger_events'),('heartbeat','/gripper/operator_present')]}
        sent = {}
        enable=fixture.create_client(SetBool,'/gripper/set_enabled')
        state=dict(raw=.5,enabled=False,valid=True,stale=False,heartbeat=True,imu=True)
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
            pubs['heartbeat'].publish(Bool(data=state['heartbeat']))
            # Emit only physical changes. Held input is classified by the same
            # C++ cache and liveness guard used in controller_bridge, not fake Joy.
            if fixture.count_subscribers('/test/trigger_events'):
                for command, value in (('axis',state['raw']),('pose',int(state['valid'])),
                                       ('stale',int(state['stale'])),('imu',int(state['imu']))):
                    if sent.get(command) != value:
                        pubs['events'].publish(String(data=f'{command} {value}'))
                        sent[command] = value
        fixture.create_timer(.01,publish)
        def pump(seconds):
            until=time.monotonic()+seconds
            while time.monotonic()<until:ex.spin_once(timeout_sec=.002)
        try:
            pump(.6)
            assert latest.get('connected'),latest
            assert not FakeDriver.writes
            set_enabled(True);state['enabled']=True;pump(.2);state['raw']=.35;pump(.12);state['raw']=.55;pump(.12)
            assert latest['pickup_method']=='crossing',latest
            assert latest.get('following'),latest
            state['raw']=1.;pump(.12)
            assert FakeDriver.writes and max(FakeDriver.writes)<=1670
            state['valid']=False;pump(.15);count=len(FakeDriver.writes);pump(.15)
            assert len(FakeDriver.writes)==count
            assert latest['follow_state']=='SIGNAL_PAUSED',latest
            assert latest['resume_pending'] is True
            state.update(valid=True);pump(.35)
            # Short verified interruption resumes without a new pickup.
            assert latest['follow_state']=='FOLLOWING',latest
            assert latest['recovery_count']>=1
            assert len(FakeDriver.writes)>count
            assert FakeDriver.writes[count]==1654  # held trigger stays closed, fresh baseline
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
            set_enabled(True)
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
            assert latest['temperature']['state']=='NORMAL',latest
            assert latest['session_enabled'] and not latest['following']
            assert latest['last_temperature_event']['feedback']['temperature_c']==107
            assert len(FakeDriver.writes)==count
            state['raw']=.5;pump(.2)
            assert len(FakeDriver.writes)==count
            state['raw']=.6;pump(.2);state['raw']=.5;pump(.2)
            assert latest['following'],latest
            state['raw']=1.;pump(.12)
            assert len(FakeDriver.writes)>count
            state['imu']=False;pump(.4)
            count=len(FakeDriver.writes)
            state['imu']=True;pump(.3)
            assert len(FakeDriver.writes)==count
            assert latest['trigger_input']['state']=='TEMPORARILY_UNVERIFIED',latest
            state['raw']=.7;pump(.35)
            assert latest['follow_state']=='FOLLOWING',latest
            state['valid']=False;pump(2.1)
            count=len(FakeDriver.writes)
            state['valid']=True;pump(.3)
            assert len(FakeDriver.writes)==count and not latest['following'],latest
            state['raw']=.35;pump(.1);state['raw']=.55;pump(.1)
            assert latest['pickup_method']=='crossing' and latest['following'],latest
            set_enabled(False)
            count=len(FakeDriver.writes);state['raw']=0.;pump(.3)
            assert len(FakeDriver.writes)==count
            assert not node.get_publishers_info_by_topic('/scaled_joint_trajectory_controller/joint_trajectory')
            print('ROS gripper lifecycle passed: independent enable, pickup, loss, recovery, stale stamps, STOP, heartbeat-gated enable, thermal pause/session retention/pickup; no arm publisher')
        finally:
            bridge.terminate();bridge.wait(timeout=5)
            ex.remove_node(node);node.destroy_node();fixture.destroy_node();ex.shutdown();rclpy.shutdown()


if __name__=='__main__':run()
