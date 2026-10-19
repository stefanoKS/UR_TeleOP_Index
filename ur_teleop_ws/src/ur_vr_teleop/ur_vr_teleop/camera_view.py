"""Optional camera viewer: latest frame only, capped display rate, no motion access."""
import time
import tkinter as tk
from tkinter import ttk
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from .dashboard_ros import LatestValue


class CameraView:
    def __init__(self, root, node):
        self.node = node
        self.window = tk.Toplevel(root)
        self.window.title('Wrist D405 — RGB and depth')
        self.mode = tk.StringVar(value='rgb')
        bar = ttk.Frame(self.window)
        bar.pack(fill='x')
        for text, value in [('RGB', 'rgb'), ('Depth (0–1 metre)', 'depth')]:
            ttk.Radiobutton(bar, text=text, variable=self.mode, value=value).pack(side='left', padx=12)
        self.label = ttk.Label(self.window, text='Waiting for camera frames…')
        self.label.pack()
        ttk.Label(self.window, text='Depth is in metres. Camera-to-tool transform is not calibrated.').pack(pady=8)
        self.frames = {'rgb': LatestValue(), 'depth': LatestValue()}
        self.closed = False
        self.subscriptions = [node.create_subscription(Image, topic,
            lambda message, stream=stream: self.receive(stream, message), qos_profile_sensor_data)
            for stream, topic in [('rgb', '/wrist_camera/color/image_raw'),
                                  ('depth', '/wrist_camera/depth/image_raw')]]
        self.window.protocol('WM_DELETE_WINDOW', self.close)
        self.window.after(200, self.render)

    def receive(self, stream, message):
        self.frames[stream].put((time.monotonic(), message))

    def render(self):
        if self.closed:
            return
        from PIL import Image as PILImage, ImageTk
        import numpy as np
        frame = self.frames[self.mode.get()].get()
        if frame is None or time.monotonic()-frame[0] > 2.:
            self.label.configure(image='', text='No fresh camera frame — check camera status in the main window')
        else:
            message = frame[1]
            try:
                if message.encoding == 'rgb8':
                    picture = PILImage.frombytes('RGB', (message.width, message.height), bytes(message.data),
                                                  'raw', 'RGB', message.step)
                elif message.encoding == '32FC1':
                    values = np.ndarray((message.height, message.width),
                        dtype='>f4' if message.is_bigendian else '<f4', buffer=bytes(message.data),
                        strides=(message.step, 4))
                    intensity = (255*np.clip(np.nan_to_num(values, nan=0., posinf=0., neginf=0.), 0., 1.)).astype('uint8')
                    picture = PILImage.fromarray(intensity, 'L')
                else:
                    raise ValueError('Unsupported image encoding: ' + message.encoding)
                picture.thumbnail((640, 480))
                self.photo = ImageTk.PhotoImage(picture)
                self.label.configure(image=self.photo, text='')
            except Exception as error:
                self.label.configure(image='', text='Camera display: '+str(error))
        self.window.after(200, self.render)

    def close(self):
        if self.closed:
            return
        self.closed = True
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        for frame in self.frames.values():
            frame.clear()
        self.window.destroy()
