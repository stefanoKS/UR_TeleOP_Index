"""Pure, status-driven operator workflow. Actions are dispatched by the Tk adapter.

One request per stage/attempt. No retries until an explicit Start; service and
readback deadlines are distinct from indefinitely waiting for physical inputs.

Servo handling uses an authoritative lifecycle (process present, status seen, status
fresh) instead of a permanent historical observation, so a stale Servo can never wedge
startup indefinitely: at most one bounded recovery is attempted, then startup fails with
the exact instruction needed to recover.
"""

RESTART_ROS_HINT = 'Restart ROS (Engineering -> Restart ROS), then press START TELEOP.'


class Startup:
    def __init__(self):
        self.state = 'IDLE'
        self.message = 'LIVE STACK — MOTION DISARMED'
        self.stage = 'DISARMED'
        self.session = None
        self.pending = None
        self.deadline = None
        self.completed = set()
        self.requested = False
        self.enabled = False
        self.seen_status = False
        self.last_failure = None
        self.servo_attempts = 0

    def start(self, session):
        if self.requested or self.pending:
            return False
        self.session = session
        self.completed.clear()
        self.enabled = False
        self.seen_status = False
        self.last_failure = None
        self.servo_attempts = 0
        self.requested = True
        self.state, self.message = 'WAITING_FOR_STACK', 'Connecting to UR10e…'
        self.stage = 'CONNECTING'
        return True

    def stop(self):
        self.requested = self.enabled = False
        self.pending = None
        self.deadline = None
        self.state, self.message = 'IDLE', 'STOP TELEOP — motion disarmed requested'
        self.stage = 'DISARMED'

    def retire(self):
        """Backend session ended; keep the last failure text for the operator."""
        self.stop()
        if self.last_failure is None:
            self.stage = 'SESSION ENDED'

    def fail(self, message, hint='Resolve the condition, then press START TELEOP.'):
        stage = self.stage or self.state
        self.last_failure = {'stage': stage, 'message': message, 'hint': hint}
        self.stop()
        self.state, self.stage = 'ERROR', 'FAULT'
        self.message = f'{stage}: {message}. {hint}'

    def acknowledge(self, action, success, detail, now):
        if action != self.pending or not self.requested:
            return
        self.pending = None
        if not success:
            self.fail(detail, 'Correct the reported problem, then press START TELEOP.')
            return
        self.completed.add(action)
        self.deadline = now + 5.
        if action == 'enable':
            self.enabled = True

    def wait(self, state, message, stage=None):
        self.state, self.message = state, message
        if stage is not None:
            self.stage = stage

    def action(self, name, state, message, now, stage=None):
        self.wait(state, message, stage)
        self.pending, self.deadline = name, now + 5.
        return name

    def step(self, d, fresh, now):
        if not self.requested:
            return None
        if self.session is not None and d.get('session_id') != self.session:
            self.fail('ROS session changed; a fresh Start is required',
                      'Wait for fresh backend status, then press START TELEOP.')
            return None
        if self.session is None and fresh:
            self.session = d.get('session_id')
        if not fresh:
            if self.seen_status or self.enabled or self.pending:
                self.stage = 'BACKEND STATUS LOST'
                self.fail('Backend status lost; enable intent retired',
                          'Check the ROS stack, then press START TELEOP.')
            else:
                self.wait('WAITING_FOR_STACK', 'Connecting to UR10e…', 'CONNECTING')
            return None
        self.seen_status = True
        if d.get('operator_heartbeat_fresh') is not True:
            if self.enabled:
                self.fail('Operator heartbeat lost; a fresh Start is required',
                          'Reconnect the operator GUI, then press START TELEOP.')
            else:
                self.wait('WAITING_FOR_STACK', 'Waiting for operator heartbeat…',
                          'WAITING — OPERATOR HEARTBEAT')
            return None
        if self.pending:
            if now >= self.deadline:
                if self.pending == 'enable':
                    self.fail('Enable service timed out; outcome unknown', RESTART_ROS_HINT)
                elif self.pending == 'servo':
                    self.fail('Start Servo timed out; outcome unknown',
                              'Wait for fresh Servo status, or ' + RESTART_ROS_HINT)
                else:
                    self.fail('Controller activation timed out; outcome unknown',
                              'Inspect the controller manager, then press START TELEOP.')
            return None
        if not str(d.get('mode', '')).startswith('LIVE'):
            self.wait('WAITING_FOR_STACK',
                      'Engineering Preview selected; start the Live stack in Engineering tools',
                      'WAITING — LIVE STACK')
            return None
        if self.enabled:
            if d.get('enable_requested') is not True:
                if now >= self.deadline:
                    self.fail('Backend cleared enable intent; a fresh Start is required',
                              'Press START TELEOP to re-arm after checks.')
                return None
            if d.get('armed') is True and d.get('motion_ready') is True:
                if d.get('clutch') == 'ACTIVE':
                    self.wait('READY', 'TELEOP ACTIVE', 'TELEOP ACTIVE')
                else:
                    self.wait('READY', 'READY — PRESS A TO MOVE', 'READY — PRESS A TO MOVE')
            elif d.get('tracking_valid') is not True:
                self.wait('WAITING_FOR_READINESS', 'TRACKING LOST — RELEASE A', 'TRACKING LOST — RELEASE A')
            else:
                blocker = next(iter(d.get('blockers') or []), '')
                self.wait('WAITING_FOR_READINESS',
                          ('Waiting for readiness; release A. ' + blocker) if blocker
                          else 'Waiting for readiness; release A.', 'WAITING — READINESS')
            return None
        if d.get('enable_requested') is not False or d.get('armed') is not False:
            self.fail('Unexpected existing enable intent; press STOP TELEOP first',
                      'Press STOP TELEOP, then press START TELEOP.')
            return None
        if d.get('a_held') is not False:
            self.wait('WAITING_FOR_READINESS', 'Release A before startup', 'WAITING — RELEASE A')
            return None
        if d.get('configuration_ready') is not True:
            self.wait('WAITING_FOR_READINESS',
                      d.get('configuration_reason', 'Waiting for configuration readiness'),
                      'WAITING — CONFIGURATION')
            return None
        if d.get('robot_preparation_ready') is not True:
            reason = d.get('robot_preparation_reason', 'Waiting for robot / External Control program…')
            stage = 'WAITING — EXTERNAL CONTROL' if 'External Control' in reason else 'WAITING — ROBOT MODE'
            self.wait('WAITING_FOR_ROBOT', reason, stage)
            return None
        if d.get('tracking_valid') is not True or d.get('observations_healthy') is not True:
            self.wait('WAITING_FOR_TRACKING', 'Waiting for valid VR tracking, joints and TF…',
                      'WAITING — INVALID TRACKING')
            return None
        if d.get('stationary') is not True:
            self.wait('WAITING_FOR_ROBOT', 'Waiting for robot standstill…', 'WAITING — ROBOT STANDSTILL')
            return None
        if d.get('controller_state_fresh') is not True:
            self.wait('WAITING_FOR_ROBOT', 'Waiting for controller ownership status…',
                      'WAITING — ROBOT CONTROLLER')
            return None
        if d.get('controller_ready') is not True:
            if 'prepare' in self.completed:
                self.wait('WAITING_FOR_ROBOT',
                          'Controller activation acknowledged; waiting for active ownership readback…',
                          'WAITING — CONTROLLER READBACK')
                if now >= self.deadline:
                    self.fail('Controller activation acknowledged but active ownership not confirmed',
                              'Inspect the controller manager and driver log, then press START TELEOP.')
                return None
            if d.get('controller_preparation_allowed') is not True:
                self.wait('WAITING_FOR_ROBOT',
                          'Controller ownership conflict; another controller claims the hardware interfaces',
                          'WAITING — CONTROLLER CONFLICT')
                return None
            return self.action('prepare', 'PREPARING_CONTROLLER', 'Preparing robot controller…', now,
                               'PREPARING CONTROLLER')
        if d.get('servo_status_fresh') is not True:
            present = d.get('servo_process_present') is True
            seen = d.get('servo_status_seen') is True
            if 'servo' in self.completed:
                self.wait('WAITING_FOR_READINESS',
                          'Waiting for fresh Servo status after Start Servo…', 'WAITING — SERVO STATUS')
                if now >= self.deadline:
                    self.fail('Servo did not publish fresh status after Start Servo', RESTART_ROS_HINT)
                return None
            if not present:
                if seen:
                    self.fail('Servo process stopped publishing status', RESTART_ROS_HINT)
                else:
                    self.fail('Servo process is not running (no status publisher)', RESTART_ROS_HINT)
                return None
            if seen:
                if self.servo_attempts >= 1:
                    self.fail('Servo status still stale after one restart attempt', RESTART_ROS_HINT)
                    return None
                self.servo_attempts += 1
                return self.action('servo', 'STARTING_SERVO',
                                   'Servo status stale; restarting Servo…', now, 'RESTARTING SERVO')
            return self.action('servo', 'STARTING_SERVO', 'Starting Servo…', now, 'STARTING SERVO')
        if d.get('motion_ready') is not True:
            blocker = next(iter(d.get('blockers') or []), 'Waiting for backend readiness…')
            self.wait('WAITING_FOR_READINESS', blocker, 'WAITING — READINESS')
            return None
        return self.action('enable', 'ENABLING_TELEOP', 'Enabling teleoperation…', now, 'ENABLING TELEOP')
