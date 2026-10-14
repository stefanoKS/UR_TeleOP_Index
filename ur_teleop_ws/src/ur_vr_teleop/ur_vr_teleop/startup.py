"""Pure, status-driven operator workflow. Actions are dispatched by the Tk adapter.

One request per stage/attempt. No retries until an explicit Start; service and
readback deadlines are distinct from indefinitely waiting for physical inputs.
"""


class Startup:
    def __init__(self):
        self.state = 'IDLE'
        self.message = 'LIVE STACK — MOTION DISARMED'
        self.session = None
        self.pending = None
        self.deadline = None
        self.completed = set()
        self.requested = False
        self.enabled = False
        self.seen_status = False

    def start(self, session):
        if self.requested or self.pending:
            return False
        self.session = session
        self.completed.clear()
        self.enabled = False
        self.seen_status = False
        self.requested = True
        self.state, self.message = 'WAITING_FOR_STACK', 'Connecting to UR10e…'
        return True

    def stop(self):
        self.requested = self.enabled = False
        self.pending = None
        self.deadline = None
        self.state, self.message = 'IDLE', 'STOP TELEOP — motion disarmed requested'

    def fail(self, message):
        stage = self.state
        self.stop()
        self.state, self.message = 'ERROR', f'{stage}: {message}. Resolve and press START TELEOP to retry.'

    def acknowledge(self, action, success, detail, now):
        if action != self.pending or not self.requested:
            return
        self.pending = None
        if not success:
            self.fail(detail)
            return
        self.completed.add(action)
        self.deadline = now + 5.
        if action == 'enable':
            self.enabled = True

    def wait(self, state, message):
        self.state, self.message = state, message

    def action(self, name, state, message, now):
        self.wait(state, message)
        self.pending, self.deadline = name, now + 5.
        return name

    def step(self, d, fresh, now):
        if not self.requested:
            return None
        if self.session is not None and d.get('session_id') != self.session:
            self.fail('ROS session changed; a fresh Start is required')
            return None
        if self.session is None and fresh:
            self.session = d.get('session_id')
        if not fresh:
            if self.seen_status or self.enabled or self.pending:
                self.fail('Backend status lost; enable intent retired')
            else:
                self.wait('WAITING_FOR_STACK', 'Connecting to UR10e…')
            return None
        self.seen_status = True
        if d.get('operator_heartbeat_fresh') is not True:
            if self.enabled:
                self.fail('Operator heartbeat lost; a fresh Start is required')
            else:
                self.wait('WAITING_FOR_STACK', 'Waiting for operator heartbeat…')
            return None
        if self.pending:
            if now >= self.deadline:
                self.fail(self.pending + ' service timed out; outcome unknown')
            return None
        if not str(d.get('mode', '')).startswith('LIVE'):
            self.wait('WAITING_FOR_STACK', 'Engineering Preview selected; start the Live stack in Engineering tools')
            return None
        if self.enabled:
            if d.get('enable_requested') is not True:
                if now >= self.deadline:
                    self.fail('Backend cleared enable intent; a fresh Start is required')
                return None
            if d.get('armed') is True and d.get('motion_ready') is True:
                self.wait('READY', 'TELEOP ACTIVE' if d.get('clutch') == 'ACTIVE' else 'READY — PRESS A TO MOVE')
            else:
                self.wait('WAITING_FOR_READINESS', 'TRACKING LOST — RELEASE A' if d.get('tracking_valid') is not True else
                          'Waiting for readiness; release A. ' + next(iter(d.get('blockers') or []), ''))
            return None
        if d.get('enable_requested') is not False or d.get('armed') is not False:
            self.fail('Unexpected existing enable intent; press STOP TELEOP first')
            return None
        if d.get('a_held') is not False:
            self.wait('WAITING_FOR_READINESS', 'Release A before startup')
            return None
        if d.get('configuration_ready') is not True:
            self.wait('WAITING_FOR_READINESS', d.get('configuration_reason', 'Waiting for configuration readiness'))
            return None
        if d.get('robot_preparation_ready') is not True:
            self.wait('WAITING_FOR_ROBOT', d.get('robot_preparation_reason', 'Waiting for robot / External Control program…'))
            return None
        if d.get('tracking_valid') is not True or d.get('observations_healthy') is not True:
            self.wait('WAITING_FOR_TRACKING', 'Waiting for valid VR tracking, joints and TF…')
            return None
        if d.get('stationary') is not True:
            self.wait('WAITING_FOR_ROBOT', 'Waiting for robot standstill…')
            return None
        if d.get('controller_state_fresh') is not True:
            self.wait('WAITING_FOR_ROBOT', 'Waiting for controller ownership status…')
            return None
        if d.get('controller_ready') is not True:
            if 'prepare' in self.completed:
                if now >= self.deadline:
                    self.fail('Controller activation acknowledged but active ownership not confirmed')
                return None
            if d.get('controller_preparation_allowed') is not True:
                self.wait('WAITING_FOR_ROBOT', 'Controller ownership conflict; engineer intervention required')
                return None
            return self.action('prepare', 'PREPARING_CONTROLLER', 'Preparing robot controller…', now)
        if d.get('servo_status_fresh') is not True:
            if 'servo' in self.completed:
                if now >= self.deadline:
                    self.fail('Servo acknowledged but fresh status not received')
                return None
            if d.get('servo_started') is True:
                self.wait('WAITING_FOR_READINESS', 'Servo previously started but status lost; inspect Servo diagnostics')
                return None
            return self.action('servo', 'STARTING_SERVO', 'Starting Servo…', now)
        if d.get('motion_ready') is not True:
            self.wait('WAITING_FOR_READINESS', next(iter(d.get('blockers') or []), 'Waiting for backend readiness…'))
            return None
        return self.action('enable', 'ENABLING_TELEOP', 'Enabling teleoperation…', now)
