"""Read-only ROS observer on its own executor. Bounded mailboxes; no Tk calls."""
import json
import queue
import threading
import time
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from controller_manager_msgs.srv import ListControllers
from .calibration_capture import Capture

COMMAND = '/scaled_joint_trajectory_controller/joint_trajectory'


def capture_isolation(d, expected_token, permitted, parameter_off, controllers_off, command_count):
    """Independent, fail-closed capture predicate. Never checks a mode label or A."""
    return bool(permitted and expected_token and d.get('managed_session_id') == expected_token
        and isinstance(d.get('session_id'), str) and d['session_id']
        and d.get('motion_output_enabled') is False and d.get('command_publisher_present') is False
        and parameter_off and controllers_off and command_count == 0
        and d.get('armed') is False and d.get('enable_requested') is False
        and d.get('controller_state_fresh') is True and d.get('setup_controllers_inactive') is True
        and d.get('observations_healthy') is True and d.get('setup_feedback_fresh') is True
        and d.get('tracking_valid') is True)


class CalibrationObserver:
    def __init__(self, context=None, config=None):
        self.context = context
        self.capture = Capture(config)
        self.commands = queue.Queue(maxsize=8)
        self.results = queue.Queue(maxsize=1)  # complete cumulative snapshot, never partial lost result events
        self.lock = threading.Lock()
        self.request = (None, False)
        self.closing = threading.Event()
        self.ready = threading.Event()
        self.failure = None
        self.thread = threading.Thread(target=self.run, name='preview-calibration-observer', daemon=False)
        self.thread.start()

    def configure(self, token, permitted):
        with self.lock:
            self.request = (token, bool(permitted))

    def command(self, action, step=None):
        if self.closing.is_set(): return
        try:
            self.commands.put_nowait((action, step))
        except queue.Full:
            self.configure(None, False)  # overload must stop collecting, never drop a STOP

    def latest(self):
        try: return self.results.get_nowait()
        except queue.Empty: return None

    def run(self):
        node = executor = None
        try:
            node = Node('preview_calibration_observer', namespace='vr_real', context=self.context,
                        start_parameter_services=False, enable_rosout=False)
            self.node = node
            self.status, self.status_at, self.last_joy_stamp = {}, -float('inf'), 0
            self.param_at = self.controllers_at = -float('inf')
            self.parameter_off = self.controllers_off = False
            self.param_pending = self.controllers_pending = None
            self.param_client = node.create_client(GetParameters, '/vr_real/teleop/get_parameters')
            self.controllers_client = node.create_client(ListControllers, '/controller_manager/list_controllers')
            self.status_sub = node.create_subscription(String, '/vr_real/operator_status', self.on_status, 1)
            node.create_subscription(PoseStamped, '/vr/controller/pose', self.on_pose, 64)
            node.create_subscription(Joy, '/vr/controller/joy', self.on_joy, 32)
            node.create_subscription(Bool, '/vr/controller/pose_fresh', lambda m:self.capture.flag('pose', m.data, time.monotonic()), 32)
            node.create_subscription(Bool, '/vr/controller/optical_fresh', lambda m:self.capture.flag('optical', m.data, time.monotonic()), 32)
            node.create_timer(.02, self.tick)
            executor = SingleThreadedExecutor(context=node.context)
            executor.add_node(node)
            self.ready.set()
            while not self.closing.is_set() and rclpy.ok(context=node.context):
                executor.spin_once(timeout_sec=.02)
        except Exception as error:
            self.failure = repr(error)
            self.capture.cancel('Observer unavailable; reopen Initial Setup')
        finally:
            if executor is not None:
                executor.shutdown(timeout_sec=1.)
                if node is not None: executor.remove_node(node)
            if node is not None: node.destroy_node()
            self.ready.set()

    def on_status(self, msg):
        try:
            d = json.loads(msg.data)
            if not isinstance(d, dict): return
            if d.get('session_id') != self.status.get('session_id'):
                self.param_at = self.controllers_at = -float('inf')
                for field, client in (('param_pending',self.param_client),('controllers_pending',self.controllers_client)):
                    pending = getattr(self,field)
                    if pending:
                        client.remove_pending_request(pending[0]);pending[0].cancel()
                        setattr(self,field,None)
                self.last_joy_stamp = 0
                self.capture.context_update(d.get('session_id'),json.dumps(d.get('effective_parameters',{}),sort_keys=True),False,time.monotonic())
            self.status, self.status_at = d, time.monotonic()
        except (ValueError, TypeError):
            self.capture.cancel('Malformed backend status')

    def on_pose(self, msg):
        if self.closing.is_set(): return
        p,q = msg.pose.position,msg.pose.orientation
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if msg.header.frame_id != 'lighthouse':
            self.capture.cancel('Unexpected controller pose frame', 'FAILED_RETRY')
            return
        self.capture.pose(stamp,[p.x,p.y,p.z],[q.x,q.y,q.z,q.w],time.monotonic(),
                          (self.node.get_clock().now().nanoseconds-stamp)/1e9)

    def on_joy(self, msg):
        if self.closing.is_set(): return
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds-stamp)/1e9
        if (len(msg.buttons) != 16 or len(msg.axes) != 12 or msg.buttons[4] not in (0,1)
                or not 0 <= age <= self.capture.cfg.joy_timeout_s or stamp < self.last_joy_stamp):
            self.capture.cancel('Invalid or stale controller button message', 'FAILED_RETRY');return
        if stamp == self.last_joy_stamp: return
        self.last_joy_stamp = stamp
        self.capture.joy(bool(msg.buttons[4]),time.monotonic())

    def readback(self, name, client, request, now):
        pending = getattr(self,name+'_pending')
        if pending:
            future, sent = pending
            if future.done() or now-sent > .5:
                valid = False
                try:
                    if future.done():
                        result = future.result()
                        if name == 'param':
                            valid = len(result.values)==1 and result.values[0].type==ParameterType.PARAMETER_BOOL and not result.values[0].bool_value
                        else:
                            valid = (any(c.name=='scaled_joint_trajectory_controller' and c.state=='inactive' for c in result.controller)
                                and not any(c.state=='active' and any(i.endswith(('/position','/velocity','/effort')) for i in c.claimed_interfaces) for c in result.controller))
                except Exception:
                    valid = False
                setattr(self,'parameter_off' if name=='param' else 'controllers_off',valid)
                setattr(self,name+'_at',now)
                client.remove_pending_request(future)
                if not future.done(): future.cancel()
                setattr(self,name+'_pending',None)
        if getattr(self,name+'_pending') is None and now-getattr(self,name+'_at') >= .20 and client.service_is_ready():
            setattr(self,name+'_pending',(client.call_async(request),now))

    def tick(self):
        if self.closing.is_set(): return
        now = time.monotonic()
        with self.lock: token,permitted = self.request
        if token and permitted:
            self.readback('param',self.param_client,GetParameters.Request(names=['motion_output_enabled']),now)
            self.readback('controllers',self.controllers_client,ListControllers.Request(),now)
        else:
            self.param_at = self.controllers_at = -float('inf')
            for field, client in (('param_pending',self.param_client),('controllers_pending',self.controllers_client)):
                pending = getattr(self,field)
                if pending:
                    client.remove_pending_request(pending[0]); pending[0].cancel()
                    setattr(self,field,None)
        d = self.status
        isolated = capture_isolation(d,token,permitted and now-self.status_at < .4
            and all(self.node.count_publishers(topic)==1 for topic in ('/vr_real/operator_status',
                '/vr/controller/pose','/vr/controller/joy','/vr/controller/pose_fresh','/vr/controller/optical_fresh')),
            self.parameter_off and now-self.param_at < .5,
            self.controllers_off and now-self.controllers_at < .5, self.node.count_publishers(COMMAND))
        params = d.get('effective_parameters',{})
        self.capture.context_update(d.get('session_id'),json.dumps(params,sort_keys=True),isolated,now)
        mapping = params.get('tracking_to_robot_rpy_deg')
        if isinstance(mapping,list) and len(mapping)==3:
            self.capture.mapping = mapping
        else:
            self.capture.isolated = False
        while True:
            try: action,step = self.commands.get_nowait()
            except queue.Empty: break
            if action == 'retry': self.capture.retry(step)
            elif action == 'mouse_start': self.capture.mouse_start(now)
            elif action == 'mouse_finish': self.capture.mouse_finish(now)
            else: self.capture.cancel('Capture stopped; release A before retry')
        self.capture.tick(now)
        snapshot = deepcopy(self.capture.snapshot())
        snapshot.update(session_id=d.get('session_id'),parameters=params,isolated=isolated)
        try: self.results.get_nowait()
        except queue.Empty: pass
        self.results.put_nowait(snapshot)

    def close(self):
        self.configure(None,False)
        self.closing.set()
        self.thread.join(timeout=2.)
        if self.thread.is_alive():
            raise RuntimeError('Calibration observer did not shut down; keep setup closed')
        while True:
            try:self.results.get_nowait()
            except queue.Empty:break
