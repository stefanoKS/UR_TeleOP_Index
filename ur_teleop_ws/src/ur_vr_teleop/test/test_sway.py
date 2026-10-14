import numpy as np
from ur_vr_teleop.sway_analysis import analyze, frequency, interpolate, lag, stops, timing
from ur_vr_teleop.sway_recording import Recorder, TOPICS

def test_synthetic_two_hz_feedback_oscillation_and_stable_command():
    t=np.arange(0,10,.01);wave=.05*np.sin(4*np.pi*t)
    streams={'actual_v':[(x,[v]*6) for x,v in zip(t,wave)],
             'native_v':[(x,[0.]*6) for x in t], 'servo_v':[(x,[0.]*6) for x in t]}
    report=analyze(streams)
    for row in report['joints'].values():
        assert abs(row['stages']['actual']['oscillation_frequency_hz']-2)<.02
        assert row['stages']['actual']['sign_reversals']==39
        assert row['stages']['native']['sign_reversals']==0
        assert row['stages']['native']['oscillation_frequency_hz'] is None
        assert row['native_feedback_offset'] is None
    assert 'actual_q' in report['unavailable']

def test_nonoscillatory_stop_delay_overshoot_and_settling():
    t=np.arange(0,4,.01);command=np.where(t<1,.1,0);actual=np.where(t<1.2,.1,0)
    position=np.minimum(t,1.2)*.1;reference=np.full(len(t),.1)
    events=stops(t,command,actual,reference,position)
    assert len(events)==1
    assert abs(events[0]['measured_standstill_s']-.2)<1e-9
    assert abs(events[0]['peak_overshoot_rad']-.02)<1e-9
    assert events[0]['settling_s'] is None
    assert frequency(t,np.zeros(len(t))) is None

def test_feedback_lag_and_no_gap_interpolation():
    t=np.arange(0,6,.01);a=np.exp(-((t-2)/.2)**2);b=np.exp(-((t-2.12)/.2)**2)
    assert abs(lag(t,a,b)['feedback_lag_s']-.12)<.011
    result=interpolate(np.array([0.,.01,1.,1.01]),np.zeros((4,6)),np.array([.005,.5,2.]))
    assert np.isfinite(result[0]).all() and np.isnan(result[1:]).all()
    assert timing(np.array([0.,.01,.02,.05]))['estimated_missing_updates']==2

def test_short_or_missing_stop_data_cannot_prove_settling():
    t=np.arange(0,1.3,.01);cmd=np.where(t<1,.1,0)
    assert stops(t,cmd,np.zeros(len(t)))[0]['measured_standstill_s'] is None
    assert analyze({})['unavailable']

def test_recorder_is_idempotent_and_only_signals_owned_process(tmp_path,monkeypatch):
    from unittest.mock import Mock
    from ur_vr_teleop import sway_recording as m
    process=Mock();process.poll.return_value=None
    launch=Mock(return_value=process);monkeypatch.setattr(m.subprocess,'Popen',launch)
    r=Recorder(tmp_path,tmp_path/'config.yaml')
    assert r.start();assert not r.start();assert launch.call_count==1
    r.log_path.unlink()
    assert 'log unavailable' in r.poll()  # Optional recorder trouble cannot stop Tk polling.
    r.stop();r.stop();assert process.send_signal.call_count==1
    process.poll.return_value=0;r.poll();assert r.process is None
    assert '/joint_states' in TOPICS and '/vr_real/operator_status' in TOPICS
    assert 'record_sway.sh' in launch.call_args.args[0][0]

def synthetic_bag(path):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from std_msgs.msg import Float64
    from ur_vr_teleop.validation import JOINTS
    from ur_vr_teleop.sway_analysis import PREFIX
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    types={'/joint_states':'sensor_msgs/msg/JointState', '/vr_real/servo_joint_targets':'trajectory_msgs/msg/JointTrajectory',
           PREFIX+'smoothed_command':'trajectory_msgs/msg/JointTrajectory','/vr/controller/source_time':'std_msgs/msg/Float64'}
    for topic,kind in types.items():writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type=kind,serialization_format='cdr'))
    for i in range(1000):
        t=i*.01;stamp=1700000000000000000+i*10000000
        joints=JointState();joints.header.stamp.sec=stamp//10**9;joints.header.stamp.nanosec=stamp%10**9
        joints.name=list(reversed(JOINTS))  # Prove named ordering rather than index assumptions.
        joints.velocity=[.05*np.sin(4*np.pi*t)]*6;joints.position=[-.05/(4*np.pi)*np.cos(4*np.pi*t)]*6
        writer.write('/joint_states',serialize_message(joints),stamp+1000000)
        for topic in ('/vr_real/servo_joint_targets',PREFIX+'smoothed_command'):
            msg=JointTrajectory();msg.header=joints.header;msg.joint_names=list(JOINTS)
            msg.points=[JointTrajectoryPoint(positions=[0.]*6,velocities=[0.]*6)]
            writer.write(topic,serialize_message(msg),stamp+2000000)
        writer.write('/vr/controller/source_time',serialize_message(Float64(data=t+1)),stamp+3000000)
    del writer


def test_real_bag_reader_and_report_without_ros_nodes(tmp_path):
    import json
    from ur_vr_teleop.sway_analysis import read_bag, main
    path=tmp_path/'bag';synthetic_bag(path)
    streams,metadata=read_bag(path)
    report=analyze(streams)
    assert abs(report['joints']['wrist_3_joint']['stages']['actual']['oscillation_frequency_hz']-2)<.02
    assert metadata['clocks']['/vr/controller/source_time']['source_nonincreasing']==0
    assert metadata['clocks']['/joint_states']['receive_minus_header_median_s']<.002
    output=tmp_path/'report';main([str(path),'--output',str(output)])
    assert all((output/f'joint_{i}.png').is_file() for i in range(1,7))
    assert json.loads((output/'report.json').read_text())['unavailable']


def test_cli_recording_lock_and_clean_stop_are_read_only(tmp_path):
    import os, signal, subprocess, sys, time
    fake=tmp_path/'ros2'
    fake.write_text('''#!/usr/bin/python3
import sys, time, signal
if sys.argv[1:3] == ['param', 'dump']:
    print('synthetic: {}')
elif sys.argv[1:3] == ['bag', 'record']:
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    while True: time.sleep(.05)
else:
    sys.exit(99)
''')
    fake.chmod(0o755)
    env=dict(os.environ,PATH=str(tmp_path)+':'+os.environ['PATH'],ROS_DOMAIN_ID='147',ROS_LOCALHOST_ONLY='1')
    cmd=[sys.executable,'-m','ur_vr_teleop.sway_recording','--output-root',str(tmp_path/'out'),'--config',str(tmp_path/'missing.yaml')]
    first=subprocess.Popen(cmd,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    try:
        assert first.stdout.readline().startswith('RECORDING:')
        second=subprocess.run(cmd,env=env,capture_output=True,text=True,timeout=5)
        assert second.returncode==2 and 'another sway recording' in second.stdout
        first.send_signal(signal.SIGINT)
        assert first.wait(timeout=10)==0
        import json
        record=next((tmp_path/'out').glob('sway_*/recording.json'))
        data=json.loads(record.read_text())
        assert 'configuration_error' in data and data['bag_returncode']==0
        import yaml
        from ros2bag.api import convert_yaml_to_qos_profile
        profiles=convert_yaml_to_qos_profile(yaml.safe_load(record.with_name('qos.yaml').read_text()))
        assert profiles['/joint_states'].depth==100 and profiles['/tf_static'].depth==1
    finally:
        if first.poll() is None:first.terminate();first.wait(timeout=10)


def test_gui_joint_monitor_requires_named_finite_fresh_reference():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from ur_vr_teleop.gui import OperatorGUI
    from ur_vr_teleop.validation import JOINTS
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    g=OperatorGUI.__new__(OperatorGUI);g.native_reference={};g.node=Mock()
    g.node.get_clock.return_value.now.return_value=SimpleNamespace(nanoseconds=2_000_000_000)
    msg=JointTrajectory();msg.header.stamp.sec=2
    msg.joint_names=list(reversed(JOINTS));msg.points=[JointTrajectoryPoint(positions=[0.]*6,velocities=[1.,2.,3.,4.,5.,6.])]
    g.on_native_reference(msg);assert g.native_reference['velocity']==[6.,5.,4.,3.,2.,1.]
    previous=g.native_reference_at
    msg.points[0].velocities=[float('nan')]*6;g.on_native_reference(msg)
    assert g.native_reference_at==previous
    msg.points[0].velocities=[0.]*6;msg.header.stamp.sec=1;g.on_native_reference(msg)
    assert g.native_reference_at==previous
