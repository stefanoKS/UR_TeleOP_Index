#!/usr/bin/python3
"""ROS publisher for an isolated SDK capture child; no robot dependencies."""
import array
import json
import subprocess
import struct
import threading
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String


def read_exact(stream, count):
    data = bytearray()
    while len(data) < count:
        part = stream.read(count-len(data))
        if not part:
            raise EOFError('Camera capture stopped')
        data.extend(part)
    return bytes(data)


class Camera(Node):
    def __init__(self):
        super().__init__('d405_camera')
        self.serial = self.declare_parameter('serial', '230322276078').value
        self.stopped = threading.Event()
        self.latest = None
        self.child = None
        self.status_text = 'D405 connecting'
        self.last_frame = 0.
        self.last_published = None
        self.status_at = 0.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.rgb = self.create_publisher(Image, '/wrist_camera/color/image_raw', qos)
        self.depth = self.create_publisher(Image, '/wrist_camera/depth/image_raw', qos)
        self.rgb_info = self.create_publisher(CameraInfo, '/wrist_camera/color/camera_info', qos)
        self.depth_info = self.create_publisher(CameraInfo, '/wrist_camera/depth/camera_info', qos)
        self.status = self.create_publisher(String, '/wrist_camera/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.worker = threading.Thread(target=self.capture, daemon=True)
        self.worker.start()
        self.create_timer(.02, self.publish_latest)

    def capture(self):
        helper = Path(__file__).resolve().with_name('d405_capture')
        while not self.stopped.is_set():
            child = None
            try:
                self.status_text = 'D405 connecting: '+self.serial
                child = subprocess.Popen([str(helper), self.serial], stdout=subprocess.PIPE, bufsize=0)
                self.child = child
                while not self.stopped.is_set():
                    if read_exact(child.stdout, 4) != b'D405':
                        raise ValueError('Invalid camera pipe header')
                    sizes = struct.unpack('=III', read_exact(child.stdout, 12))
                    if not (0 < sizes[0] < 4096 and sizes[1] == 640*480*3 and sizes[2] == 640*480*2):
                        raise ValueError('Unexpected camera frame dimensions')
                    info = json.loads(read_exact(child.stdout, sizes[0]))
                    rgb = read_exact(child.stdout, sizes[1]); depth = read_exact(child.stdout, sizes[2])
                    self.latest = (info, rgb, depth)
                    self.last_frame = time.monotonic()
                    self.status_text = 'D405 RGB + depth (metres), '+self.serial+'; mount transform not calibrated'
            except (OSError, ValueError, EOFError) as error:
                self.status_text = 'D405 unavailable: '+str(error)+'; retrying (teleop unaffected)'
                if not self.stopped.is_set() and rclpy.ok():
                    self.get_logger().warning(self.status_text)
            finally:
                if child is not None:
                    if child.poll() is None:
                        child.terminate()
                    try: child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        child.kill(); child.wait()
                    child.stdout.close()
                self.child = None
            self.stopped.wait(2.)

    @staticmethod
    def calibration(info, key, stamp, frame):
        intr = info[key]
        msg = CameraInfo()
        msg.header.stamp = stamp; msg.header.frame_id = frame
        msg.width = info['width']; msg.height = info['height']
        fx, fy, x, y = [float(intr[k]) for k in ('fx', 'fy', 'ppx', 'ppy')]
        msg.k = [fx, 0., x, 0., fy, y, 0., 0., 1.]
        msg.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        msg.p = [fx, 0., x, 0., 0., fy, y, 0., 0., 0., 1., 0.]
        msg.distortion_model = intr['model']; msg.d = [float(v) for v in intr['d']]
        return msg

    def publish_latest(self):
        now = time.monotonic()
        if now-self.status_at > 1.:
            text = self.status_text if now-self.last_frame < 3. or self.last_frame == 0. else 'D405 frames stale; capture reconnecting (teleop unaffected)'
            self.status.publish(String(data=text)); self.status_at = now
        frame = self.latest
        if frame is None or frame is self.last_published or now-self.last_frame > 1.:
            return
        self.last_published = frame
        info, rgb, raw_depth = frame
        stamp = self.get_clock().now().to_msg()  # host receipt; not hardware-synchronized to UR
        color = Image(); color.header.stamp = stamp
        color.header.frame_id = 'wrist_camera_color_optical_frame'
        color.width = info['width']; color.height = info['height']
        color.encoding = 'rgb8'; color.step = info['color_stride']; color.data = array.array('B', rgb)
        self.rgb.publish(color)
        self.rgb_info.publish(self.calibration(info, 'color', stamp, color.header.frame_id))
        raw = np.ndarray((info['height'], info['width']), dtype='<u2', buffer=raw_depth,
                         strides=(info['depth_stride'], 2))
        metric = raw.astype('<f4')*info['depth_units']; metric[raw == 0] = np.nan
        depth = Image(); depth.header.stamp = stamp
        depth.header.frame_id = 'wrist_camera_depth_optical_frame'
        depth.width = info['width']; depth.height = info['height']
        depth.encoding = '32FC1'; depth.step = depth.width*4; depth.data = array.array('B', metric.tobytes())
        self.depth.publish(depth)
        self.depth_info.publish(self.calibration(info, 'depth', stamp, depth.header.frame_id))

    def stop(self):
        self.stopped.set()
        child = self.child
        if child is not None and child.poll() is None:
            child.terminate()
        self.worker.join(timeout=5.)


def main():
    rclpy.init(); node = None
    try:
        node = Camera(); rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
