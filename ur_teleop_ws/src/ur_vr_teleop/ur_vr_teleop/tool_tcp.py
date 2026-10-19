"""Fixed tool geometry and bounded, output-only UR RTDE TCP observation."""
import ipaddress
import json
import math
import os
from pathlib import Path
import socket
import struct
import tempfile
import time
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation

FRAME = 'teleop_tcp'


def validate_offset(value):
    if not isinstance(value, (list, tuple)) or len(value) != 6 or any(
            type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError('TCP requires six finite numbers: x, y, z metres; rx, ry, rz radians (rotation vector)')
    if math.sqrt(sum(v*v for v in value[:3])) > 2:
        raise ValueError('TCP offset must be within 2 metres of tool0')
    if math.hypot(*value[3:]) > 2*math.pi:
        raise ValueError('TCP rotation-vector magnitude must be at most 2π radians')
    return [float(v) for v in value]


def matches(a, b):
    a, b = validate_offset(a), validate_offset(b)
    return (math.dist(a[:3], b[:3]) <= 1e-5 and
            (Rotation.from_rotvec(a[3:]).inv()*Rotation.from_rotvec(b[3:])).magnitude() <= 1e-5)


def tool_xml(offset):
    offset = validate_offset(offset)
    rpy = Rotation.from_rotvec(offset[3:]).as_euler('xyz')
    xyz = ' '.join(map(str, offset[:3]))
    angles = ' '.join(map(str, rpy))
    return (f'<link name="{FRAME}"/><joint name="teleop_tcp_joint" type="fixed">'
            f'<parent link="tool0"/><child link="{FRAME}"/>'
            f'<origin xyz="{xyz}" rpy="{angles}"/></joint>')


def add_tool(xml, offset):
    if offset is None:
        return xml
    root = ET.fromstring(xml)
    if any(element.get('name') == FRAME for element in root.findall('link')):
        raise ValueError('Tool frame already exists')
    for element in ET.fromstring('<robot>'+tool_xml(offset)+'</robot>'):
        root.append(element)
    return ET.tostring(root, encoding='unicode')


def tool_semantic(xml, offset):
    if offset is None:
        return xml
    root = ET.fromstring(xml)
    chain = root.find("./group[@name='ur10e_manipulator']/chain")
    if chain is None or chain.get('tip_link') != 'tool0':
        raise ValueError('Unexpected MoveIt manipulator chain')
    chain.set('tip_link', FRAME)
    return ET.tostring(root, encoding='unicode')


def load_profile(config, robot):
    path = Path(config).with_name('tool_tcp.json')
    if not path.exists():
        return None
    value = json.loads(path.read_text())
    if value.get('version') != 1 or value.get('robot_ip') != robot['ip']:
        raise ValueError('TCP profile version or robot IP mismatch')
    return validate_offset(value['offset'])


def save_profile(config, robot_ip, offset):
    ipaddress.ip_address(robot_ip)
    value = {'version': 1, 'robot_ip': robot_ip, 'offset': validate_offset(offset)}
    path = Path(config).with_name('tool_tcp.json')
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
            name = stream.name
            json.dump(value, stream, indent=2, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if name and os.path.exists(name):
            os.unlink(name)


def read_offset(robot_ip, timeout=.6):
    """One bounded fresh connection; never configure inputs or send robot commands.

    Closing each observation avoids accepting a backlog of stale RTDE samples.
    Called exclusively on a worker, never on the ROS command executor/Tk thread.
    """
    ipaddress.ip_address(robot_ip)
    deadline = time.monotonic()+timeout
    with socket.create_connection((robot_ip, 30004), timeout=timeout) as sock:
        def exact(count):
            data = b''
            while len(data) < count:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('TCP read timed out')
                sock.settimeout(remaining)
                part = sock.recv(count-len(data))
                if not part:
                    raise OSError('TCP RTDE connection closed')
                data += part
            return data

        def receive(expected):
            for _ in range(16):
                size, kind = struct.unpack('!HB', exact(3))
                if not 3 <= size <= 4096:
                    raise ValueError('Invalid RTDE packet size')
                payload = exact(size-3)
                if kind == expected:
                    return payload
                if kind != 77:
                    raise ValueError('Unexpected RTDE packet')
            raise ValueError('Too many RTDE messages')

        def request(kind, payload=b''):
            sock.settimeout(max(.001, deadline-time.monotonic()))
            sock.sendall(struct.pack('!HB', len(payload)+3, kind)+payload)
            return receive(kind)

        if request(86, struct.pack('!H', 2)) != b'\x01':
            raise ValueError('UR does not accept RTDE protocol 2')
        recipe = request(79, struct.pack('!d', 10.)+b'tcp_offset')
        if len(recipe) < 2 or recipe[0] == 0 or recipe[1:] != b'VECTOR6D':
            raise ValueError('UR tcp_offset output is unavailable')
        if request(83) != b'\x01':
            raise ValueError('UR TCP observation stream refused')
        payload = receive(85)
        if len(payload) != 49 or payload[0] != recipe[0]:
            raise ValueError('Invalid TCP observation recipe or size')
        return validate_offset(struct.unpack('!6d', payload[1:]))
