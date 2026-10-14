"""Evaluate launch parameters only. Never execute a launch or connect a robot."""
import importlib.util
import json
from pathlib import Path
import pytest
from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters


@pytest.mark.parametrize('live,engineering,required', [('true','false',True),('false','false',False),('true','true',False)])
def test_launch_record_policy_is_explicit_and_context_is_ros_string(monkeypatch,live,engineering,required):
    monkeypatch.setenv('ROS_DOMAIN_ID','43')
    path=Path(__file__).parents[1]/'launch/commissioning.launch.py'
    spec=importlib.util.spec_from_file_location('v4_launch_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    ctx=LaunchContext()
    config=Path(__file__).resolve().parents[4]/'ur_teleop/config/real_teleop.yaml'
    ctx.launch_configurations.update(config_file=str(config),motion_output_enabled=live,engineering_session=engineering)
    action=module.nodes(ctx)[1]
    params={}
    for entry in evaluate_parameters(ctx,action._Node__parameters):params.update(entry)
    assert 'engineering_required' not in params
    assert params['motion_output_enabled'] is (live=='true')
    assert params['alignment_verified'] is False and params['optical_loss_test_verified'] is False
    assert type(params['engineering_context']) is str
    assert json.loads(params['engineering_context'])['record']==str(config.with_name('engineering_record.json'))
    assert 'enable_requested' not in params
