import json
import math
import struct
import xml.etree.ElementTree as ET
from types import SimpleNamespace
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from ur_vr_teleop import tool_tcp as tcp
from ur_vr_teleop.preferences import load_settings, write_effective_snapshot
from ur_vr_teleop.tcp_panel import ToolTCPPanel


@pytest.mark.parametrize('value', [[0]*5, [0]*7, [True]*6, [float('nan')]*6, [0,0,3,0,0,0], [0,0,0,7,0,0]])
def test_invalid_offset(value):
    with pytest.raises(ValueError): tcp.validate_offset(value)


def test_geometry_rotation_vector_and_semantics():
    offset = [.05, -.02, .2, .3, -.4, .5]
    root = ET.fromstring(tcp.add_tool('<robot><link name="tool0"/></robot>', offset))
    joint = root.find('joint')
    assert joint.find('parent').get('link') == 'tool0'
    assert list(map(float, joint.find('origin').get('xyz').split())) == offset[:3]
    rpy = list(map(float, joint.find('origin').get('rpy').split()))
    np.testing.assert_allclose(Rotation.from_euler('xyz', rpy).as_matrix(), Rotation.from_rotvec(offset[3:]).as_matrix())
    semantic = tcp.tool_semantic('<robot><group name="ur10e_manipulator"><chain base_link="base_link" tip_link="tool0"/></group></robot>', offset)
    assert ET.fromstring(semantic).find('./group/chain').get('tip_link') == tcp.FRAME
    assert tcp.matches([0]*6, [0,0,0,0,0,2*math.pi])


def test_profile_identity_and_immutable_snapshot(tmp_path):
    config = tmp_path/'config.yaml'
    config.write_text('robot:\n  ip: 127.0.0.1\nteleop: {}\n')
    assert load_settings(config)['teleop'].get('tool_frame') is None
    first = [0,0,.2,0,0,0]
    tcp.save_profile(config, '127.0.0.1', first)
    settings, snapshot = write_effective_snapshot(config, tmp_path/'session')
    assert settings['teleop']['tool_frame'] == tcp.FRAME
    tcp.save_profile(config, '127.0.0.1', [0]*6)
    assert load_settings(snapshot)['robot']['tcp_offset'] == first
    config.write_text('robot:\n  ip: 127.0.0.2\nteleop: {}\n')
    with pytest.raises(ValueError, match='IP mismatch'): load_settings(config)


class Socket:
    def __init__(self, data): self.data, self.sent = data, []
    def __enter__(self): return self
    def __exit__(self, *args): pass
    def settimeout(self, value): pass
    def sendall(self, value): self.sent.append(value)
    def recv(self, count):
        # Exercise fragmented headers and payloads.
        result, self.data = self.data[:min(count,2)], self.data[min(count,2):]
        return result


def packet(kind, data): return struct.pack('!HB',len(data)+3,kind)+data


@pytest.mark.parametrize('fault', ['', 'protocol', 'recipe', 'size', 'nonfinite', 'closed'])
def test_output_only_rtde_and_rejections(monkeypatch, fault):
    offset = [.01,.02,.2,.3,.4,.5]
    stream = packet(86, b'\x00' if fault=='protocol' else b'\x01')
    stream += packet(79, b'\x00NOT_FOUND' if fault=='recipe' else b'\x01VECTOR6D')
    stream += packet(83,b'\x01')
    value = [float('nan')]*6 if fault=='nonfinite' else offset
    stream += packet(85, b'\x01'+struct.pack('!6d',*value)+(b'x' if fault=='size' else b''))
    sock = Socket(b'' if fault=='closed' else stream)
    monkeypatch.setattr(tcp.socket, 'create_connection', lambda *a, **kw: sock)
    if fault:
        with pytest.raises((ValueError,OSError)): tcp.read_offset('127.0.0.1')
    else:
        assert tcp.read_offset('127.0.0.1') == offset
        assert [p[2] for p in sock.sent] == [86,79,83]  # No inputs or commands.
        assert sock.sent[1][11:] == b'tcp_offset'


@pytest.mark.parametrize('fault', ['', 'armed', 'a_held', 'enable_requested', 'stale', 'starting'])
def test_editor_requires_disarmed_fresh_backend(fault):
    status = dict(armed=False,a_held=False,enable_requested=False)
    if fault in status: status[fault] = True
    panel = ToolTCPPanel.__new__(ToolTCPPanel)
    panel.gui = SimpleNamespace(last_status=status, stale=lambda:fault=='stale', startup=SimpleNamespace(requested=fault=='starting'))
    if fault:
        with pytest.raises(ValueError): panel.check_disarmed()
    else: panel.check_disarmed()
