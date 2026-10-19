"""Serial owner with one pending action and latest-only input. Never runs in ROS/Tk.

Gripper authorization is deliberately independent of the UR arm.  It is owned by
this worker from an explicit session enable, the saved/calibrated profile, the
gripper operator heartbeat and the bridge's fresh Joy/pose/optical signals.  The
arm backend's enable/motion-ready/motion-output flags are never consulted.
"""
from dataclasses import asdict, replace
import threading
import time
from .gripper_config import Config, load
from .gripper_mapping import Follow, normalize, target_ticks
from .gripper_calibration import jog_target
from .st3215_driver import Driver



class Worker:
    def __init__(self, path, driver_factory=Driver):
        self.path, self.factory = path, driver_factory
        self.lock = threading.Lock()
        self.quit = threading.Event()
        self.inputs = {}
        # Session enable is latched until STOP GRIPPER; it is not a heartbeat.
        self.session_enabled = False
        self.action = None
        self.epoch = self.seen_epoch = 0
        self.output = {'connection': 'Disconnected', 'status': 'Holding', 'reason': 'Not connected'}
        self.driver = self.feedback = None
        self.config = Config()
        self.follow = Follow(self.config)
        self.commanded = self.command_at = None
        self.attempted_ticks = self.attempt_at = None
        self.write_acknowledged = None
        self.fraction = None
        self.calibration_target = None
        self.calibration_follow = False
        self.last_time = time.monotonic()
        self.stall_since = None
        self.last_progress = None
        self.session = None
        self.thread = threading.Thread(target=self.run, name='gripper-serial', daemon=False)

    def update(self, key, value):
        with self.lock:
            if key == 'session_enabled':
                value = bool(value)
                if value != self.session_enabled:
                    self.session_enabled = value
                    self.epoch += 1
                return
            now = time.monotonic()
            old, at = self.inputs.get(key, (None, -float('inf')))
            if (now - at >= .25 or ((value is False or value is None) and value != old) or
                    (key == 'duplicate' and bool(value) and not bool(old))):
                self.epoch += 1
            self.inputs[key] = (value, now)

    def request(self, name):
        with self.lock:
            if self.action is not None:
                return False
            self.action = (name, time.monotonic())
            return True

    def snapshot(self):
        with self.lock:
            return dict(self.output)

    def start(self):
        self.thread.start()

    def close(self):
        self.quit.set()
        self.thread.join(timeout=8.)
        if self.thread.is_alive():
            raise RuntimeError('Gripper serial worker failed to join')

    def disconnect(self):
        if self.driver:
            try:
                self.driver.close()
            except OSError:
                pass
        self.driver = self.feedback = None
        self.follow.reset()
        self.calibration_target = None
        self.commanded = self.command_at = None
        self.calibration_follow = False

    def step(self):
        now = time.monotonic()
        dt, self.last_time = now - self.last_time, now
        with self.lock:
            inputs, action, epoch = dict(self.inputs), self.action, self.epoch
            self.action = None
        if epoch != self.seen_epoch:
            self.follow.reset()
            self.calibration_target = None
            self.calibration_follow = False
            self.seen_epoch = epoch
        def recent(key, seconds=.25):
            value, at = inputs.get(key, (None, -float('inf')))
            return value if now - at < seconds else None
        setup = recent('calibration') is True
        session = self.session_enabled
        heartbeat = recent('heartbeat') is True
        duplicate = recent('duplicate') is True
        calibration = setup and not duplicate
        reason = 'Holding; authorization withdrawn'
        if action and now - action[1] < .5:
            name = action[0]
            if name in ('connect', 'reload'):
                # Read-only discovery is independent of the arm and external
                # program.  No target or torque command is sent here.
                self.disconnect()
                self.config = load(self.path)
                self.follow = Follow(self.config)
                self.driver = self.factory(self.config)
                self.feedback = self.driver.connect(self.quit.is_set)
                # Discovery is read-only and does not silently save an ID.
                self.config = replace(self.config, servo_id=self.driver.servo_id)
                self.driver.config = self.config
                self.follow = Follow(self.config)
                self.stall_since = self.last_progress = None
                self.attempted_ticks = self.attempt_at = self.write_acknowledged = None
                return
            if name != 'read':
                if not calibration or self.feedback is None:
                    raise ValueError(
                        'Authorize gripper calibration (mechanism clear, supervised) and '
                        'connect with fresh feedback before calibration movement')
                self.config.validate(calibrated=True)
                if name == 'follow':
                    self.calibration_follow = True
                    self.calibration_target = None
                    self.follow = Follow(replace(self.config, rate_ticks_s=min(self.config.rate_ticks_s, 25.)))
                else:
                    self.calibration_follow = False
                measured = self.feedback.ticks
                self.calibration_target = (self.config.open_ticks if name == 'open' else
                    self.config.closed_ticks if name == 'close' else
                    None if name == 'follow' else jog_target(measured, name == 'jog_close', self.config))
                # Calibration gets a fresh measured baseline, never a stale target.
                self.follow.reset()
                self.follow.reference = float(measured)
                self.follow.last_sent = measured
        if not self.driver:
            return
        self.feedback = self.driver.read()
        fb = self.feedback
        feedback_wall_ns = time.time_ns()
        if fb.motor_status or fb.temperature_c >= self.config.temperature_limit:
            raise IOError(f'Motor fault status=0x{fb.motor_status:02x}; temperature={fb.temperature_c} C')
        # Bridge signs: fresh accepted pose stream, fresh optical signal and a
        # fresh Joy.  Arm enable/motion flags deliberately do not appear here.
        valid_input = (recent('pose') is True and recent('optical') is True
                       and recent('joy') is not None and not duplicate)
        authorized = self.config.enabled and session and heartbeat and not duplicate
        fraction = None
        if valid_input:
            try:
                fraction = normalize(recent('joy'), self.config)
            except ValueError:
                valid_input = False
        self.fraction = fraction
        target = None
        if calibration and self.calibration_target is not None:
            if abs(fb.ticks - self.follow.last_sent) > 20:
                raise ValueError('Encoder moved outside calibration lead; inspect mechanism and reconnect')
            goal = self.calibration_target
            step = min(self.config.rate_ticks_s, 25.) * max(0., min(dt, .04))
            self.follow.reference += max(-step, min(step, goal - self.follow.reference))
            target = round(max(fb.ticks - 20, min(fb.ticks + 20, self.follow.reference)))
            if target == self.follow.last_sent:
                target = None
            else:
                self.follow.last_sent = target
            reason = 'Calibration; low-speed bounded travel'
        elif (authorized or (calibration and self.calibration_follow)) and valid_input:
            self.calibration_target = None
            self.config.validate(calibrated=True)
            target = self.follow.step(fraction, fb.ticks, dt)
            reason = 'Following trigger' if self.follow.active else 'Pickup: move trigger >=5%, then match actual closure'
        else:
            self.follow = Follow(self.config)
            self.calibration_target = None
            self.calibration_follow = False
            if duplicate:
                reason = 'Duplicate gripper node or serial owner detected; follow disabled'
            elif not self.config.enabled:
                reason = 'Gripper profile is not enabled; calibrate and enable it in Gripper Setup'
            elif not session:
                reason = 'Gripper session not enabled; press ENABLE GRIPPER'
            elif not heartbeat:
                reason = 'Gripper operator heartbeat missing; authorization withdrawn'
            elif not valid_input:
                reason = 'Controller pose/optical/trigger input stale or invalid; holding, fresh pickup required'
            else:
                reason = 'Holding; authorization withdrawn'
        # Error is diagnostic, never proof of grip. Halt target advancement on lack of progress.
        desired = (self.calibration_target if calibration and self.calibration_target is not None else
                   target_ticks(fraction, self.config) if (authorized or calibration) and self.follow.active and fraction is not None else None)
        if desired is not None and abs(desired - fb.ticks) >= self.config.stall_error_ticks:
            if self.last_progress is None or abs(fb.ticks - self.last_progress) >= 2:
                self.stall_since, self.last_progress = now, fb.ticks
            elif now - self.stall_since >= self.config.stall_seconds:
                raise IOError('Persistent position error without progress; object/obstruction/jam unknown')
        else:
            self.stall_since = self.last_progress = None
        # Recheck latest withdrawal after bounded reads and before issuing a packet.
        with self.lock:
            latest, current_epoch, live_session = dict(self.inputs), self.epoch, self.session_enabled
        current = time.monotonic()
        def live(key):
            value, at = latest.get(key, (None, -float('inf')))
            return value if current - at < .25 else None
        physical_calibration = calibration and self.calibration_target is not None
        if physical_calibration or self.calibration_follow:
            # Calibration consent is the only authorization for setup movement.
            safe = live('calibration') is True
        else:
            safe = (live_session and live('heartbeat') is True
                    and live('duplicate') is not True)
        if self.calibration_follow:
            safe = (safe and live('calibration') is True and live('pose') is True
                    and live('optical') is True and live('joy') is not None)
        elif target is not None and not physical_calibration:
            safe = safe and live('pose') is True and live('optical') is True and live('joy') is not None
        if target is not None and safe and epoch == current_epoch and not self.quit.is_set():
            self.attempted_ticks, self.attempt_at = target, time.time_ns()
            self.write_acknowledged = False
            self.driver.write(target, calibration=calibration)
            self.write_acknowledged = True
            self.commanded, self.command_at = target, time.time_ns()
        connected_with_fault = duplicate
        try:
            self.config.validate(calibrated=True)
            calibration_complete = True
        except ValueError:
            calibration_complete = False
        with self.lock:
            self.output = dict(connection='Fault' if connected_with_fault else 'Connected',
                status=('Fault' if connected_with_fault else 'Moving' if fb.moving else
                        'Ready' if authorized and valid_input and not self.follow.active else 'Holding'),
                reason=reason, connected=not connected_with_fault, feedback_valid=True,
                session_enabled=session, heartbeat_fresh=heartbeat, authorized=authorized,
                setup_authorized=setup, duplicate_owner=duplicate,
                profile_enabled=self.config.enabled, calibration_complete=calibration_complete,
                feedback=asdict(fb), feedback_wall_ns=feedback_wall_ns,
                commanded_ticks=self.commanded, command_wall_ns=self.command_at,
                attempted_ticks=self.attempted_ticks, write_started_wall_ns=self.attempt_at,
                write_acknowledged=self.write_acknowledged,
                raw_trigger=recent('joy') if valid_input else None,
                normalized_trigger=fraction, requested_fraction=fraction,
                input_fresh=valid_input, following=self.follow.active,
                identity=self.driver.identity, config=asdict(self.config))

    def run(self):
        try:
            while not self.quit.is_set():
                started = time.monotonic()
                try:
                    self.step()
                except Exception as exc:
                    self.disconnect()
                    with self.lock:
                        self.output = dict(connection='Fault', status='Fault', connected=False,
                                           feedback_valid=False, reason=str(exc), motor_fault=str(exc),
                                           session_enabled=self.session_enabled,
                                           authorized=False, profile_enabled=self.config.enabled,
                                           attempted_ticks=self.attempted_ticks, write_started_wall_ns=self.attempt_at,
                                           write_acknowledged=self.write_acknowledged)
                self.quit.wait(max(0., .04 - (time.monotonic() - started)))
        finally:
            self.disconnect()
