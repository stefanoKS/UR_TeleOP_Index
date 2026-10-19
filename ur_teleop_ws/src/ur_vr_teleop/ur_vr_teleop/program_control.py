"""Pure External Control program workflow and shutdown ordering.

No ROS, Tk, process or motion imports live here.  The GUI adapter performs the
service calls; this module decides which call is permitted next and enforces
bounded stage deadlines.  A successful Play acknowledgment alone never means
the program is running: Dashboard readback and fresh driver status are required.
"""
from dataclasses import dataclass, replace
import math
from pathlib import Path, PurePosixPath

# Dashboard service answers are checked explicitly, never inferred from transport.
LOCAL_CONTROL_TEXT = (
    'Robot is in Local Control. Select Remote Control on the UR pendant to enable '
    'PC program start.')

STAGES = (
    'WAITING_FOR_DASHBOARD', 'CHECKING_REMOTE_CONTROL', 'CHECKING_SAFETY_MODE',
    'CHECKING_ROBOT_MODE', 'CHECKING_LOADED_PROGRAM', 'LOADING_PROGRAM',
    'PLAYING_PROGRAM', 'WAITING_FOR_PROGRAM_RUNNING', 'WAITING_FOR_DRIVER_CONNECTION',
)

DEFAULT_TIMEOUTS = {
    'dashboard': 3.0, 'remote': 2.0, 'safety': 2.0, 'mode': 2.0, 'loaded': 2.0,
    'load': 10.0, 'play': 5.0, 'running': 2.0, 'running_wait': 10.0,
    'driver': 20.0,
}


@dataclass(frozen=True)
class ProgramConfig:
    enabled: bool = False
    program: str = ''
    timeouts: dict = None

    def validated(self):
        if type(self.enabled) is not bool:
            raise ValueError('remote_program_control_enabled must be boolean')
        if not isinstance(self.program, str):
            raise ValueError('external_control_program must be a string')
        program = self.program.strip()
        if program and (len(program) > 255 or any(ch in program for ch in '\r\n\t\0')):
            raise ValueError('external_control_program contains unsupported characters')
        if self.enabled and not program:
            raise ValueError(
                'remote_program_control_enabled requires external_control_program '
                '(the approved External Control .urp name or path)')
        timeouts = dict(DEFAULT_TIMEOUTS)
        if self.timeouts:
            if not isinstance(self.timeouts, dict):
                raise ValueError('dashboard timeouts must be a mapping')
            for key, value in self.timeouts.items():
                if key not in DEFAULT_TIMEOUTS:
                    raise ValueError('Unsupported dashboard timeout: ' + str(key))
                if type(value) not in (int, float) or not math.isfinite(value) or not .1 <= value <= 60.:
                    raise ValueError(f'dashboard timeout {key} must be 0.1-60 s')
                timeouts[key] = float(value)
        return replace(self, program=program, timeouts=timeouts)


def load_program_config(path):
    """Read YAML dashboard settings, then an optional adjacent operator override."""
    import json
    import yaml
    path = Path(path)
    document = yaml.safe_load(path.read_text()) if path.exists() else {}
    section = (document or {}).get('dashboard') if isinstance(document, dict) else None
    section = section if isinstance(section, dict) else {}
    config = ProgramConfig(
        enabled=section.get('remote_program_control_enabled', False),
        program=section.get('external_control_program', '') or '',
        timeouts=section.get('timeouts'))
    override = path.with_name('program_control.json')
    if override.exists():
        saved = json.loads(override.read_text())
        if not isinstance(saved, dict) or saved.get('version') != 1:
            raise ValueError('Unsupported program control settings: ' + str(override))
        config = replace(config, enabled=saved.get('enabled', config.enabled),
                         program=saved.get('program', config.program))
    return config.validated()


def save_program_config(path, config):
    """Atomically persist the reviewed program identity beside operator settings."""
    import json
    import os
    import tempfile
    config = config.validated()
    path = Path(path).with_name('program_control.json')
    path.parent.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
            name = stream.name
            json.dump({'version': 1, 'enabled': config.enabled, 'program': config.program},
                      stream, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if name and os.path.exists(name):
            os.unlink(name)
    return path


def same_program(loaded, configured):
    """Compare Dashboard-reported identity with the reviewed configured program.

    PolyScope may report a bare filename after a full path was loaded, so only
    the final path component is compared.  Both the exact name and a missing
    ``.urp`` suffix are accepted only for the configured component.
    """
    if not isinstance(loaded, str) or not isinstance(configured, str):
        return False
    loaded_name = PurePosixPath(loaded.strip().replace('\\', '/')).name
    configured_name = PurePosixPath(configured.strip().replace('\\', '/')).name
    return bool(loaded_name and configured_name) and loaded_name == configured_name


class ProgramStartup:
    """One bounded, non-retrying Dashboard start sequence.

    ``step`` returns a service action name (``remote``, ``safety``, ``mode``,
    ``loaded``, ``running``, ``load`` or ``play``) or ``None``.  The adapter asks
    this class again only after the matching :meth:`acknowledge`.  Timeouts are
    deliberate dead ends: an unknown Play outcome can never be retried into an
    uncontrolled enable.
    """

    def __init__(self, config):
        self.config = config if isinstance(config, ProgramConfig) else ProgramConfig(**config)
        self.config = self.config.validated()
        self.state = 'IDLE'
        self.stage = 'IDLE'
        self.message = ('External Control program control is not configured; start it manually on the pendant.'
                        if not self.config.enabled else 'PROGRAM: STOPPED')
        self.requested = False
        self.pending = None
        self.deadline = None
        self.overall_deadline = None
        self.generation = 0
        self.results = {}
        self.last_failure = None
        self.play_attempted = False
        self.play_wait_deadline = None
        self.running_checked_at = -float('inf')
        self.ready_since = None

    @property
    def enabled(self):
        return self.config.enabled

    @property
    def busy(self):
        return self.requested or self.pending is not None

    def start(self, now):
        if self.busy:
            return False
        if not self.config.enabled:
            self.state, self.stage = 'DISABLED', 'MANUAL'
            self.message = 'PROGRAM: MANUAL — external_control_program is not configured'
            return False
        self.requested = True
        self.pending = None
        self.deadline = None
        self.generation += 1
        self.results = {}
        self.last_failure = None
        self.play_attempted = False
        self.play_wait_deadline = None
        self.ready_since = None
        self.running_checked_at = -float('inf')
        self.overall_deadline = now + self.config.timeouts['dashboard']
        self.state, self.stage = 'WAITING_FOR_DASHBOARD', 'WAITING FOR DASHBOARD'
        self.message = 'Checking UR Dashboard services…'
        return True

    def stop(self):
        self.generation += 1
        self.requested = False
        self.pending = None
        self.deadline = None
        self.overall_deadline = None
        self.ready_since = None
        self.play_wait_deadline = None
        if self.state != 'FAILED':
            self.state, self.stage = 'IDLE', 'STOPPED'
            self.message = 'PROGRAM: STOPPED'

    def fail(self, detail, hint='Resolve the condition, then press START TELEOP.'):
        stage = self.stage or self.state
        self.last_failure = {'stage': stage, 'message': detail, 'hint': hint}
        self.generation += 1
        self.requested = False
        self.pending = None
        self.deadline = None
        self.overall_deadline = None
        self.state, self.stage = 'FAILED', 'PROGRAM FAULT'
        self.message = f'{stage}: {detail}' + (f' {hint}' if hint else '')

    def wait(self, state, message, stage=None):
        self.state, self.message = state, message
        if stage is not None:
            self.stage = stage

    def action(self, name, state, message, now, timeout=None, stage=None):
        self.wait(state, message, stage)
        self.pending = name
        self.deadline = now + self.config.timeouts[timeout or name]
        return name

    def acknowledge(self, action, success, detail, now, value=None):
        if action != self.pending or not self.requested:
            return
        self.pending = None
        if not success:
            self.fail(detail or 'Dashboard service rejected the request')
            return
        self.results[action] = value
        self.deadline = now + self.config.timeouts[
            'running_wait' if action == 'play' else 'running']
        if action == 'load':
            # Force an independent loaded-program readback before Play.
            self.results.pop('loaded', None)
            self.results['load_acknowledged'] = True
        if action == 'play':
            self.play_attempted = True
            self.play_wait_deadline = now + self.config.timeouts['running_wait']
            self.results['play_acknowledged'] = True
        if action == 'running':
            self.running_checked_at = now

    def _driver_ready(self, obs):
        return obs.get('driver_ready') is True and obs.get('driver_status_fresh') is True

    def step(self, obs, now):
        if not self.requested:
            return None
        if not isinstance(obs, dict):
            obs = {}
        if not obs.get('dashboard_ready'):
            self.wait('WAITING_FOR_DASHBOARD', 'Waiting for UR Dashboard services…',
                      'WAITING FOR DASHBOARD')
            deadline = self.deadline or self.overall_deadline
            if deadline is not None and now >= deadline:
                self.fail('UR Dashboard services are unavailable',
                          'Check the UR driver, robot network and Remote Control, then press START TELEOP.')
            return None
        self.overall_deadline = None
        if self.pending:
            if now >= self.deadline:
                if self.pending == 'play':
                    self.fail('Play acknowledgment timed out; program state is unknown',
                              'Do not retry automatically. Confirm the pendant program state, '
                              'then press STOP TELEOP and start again only after checking.')
                elif self.pending == 'load':
                    self.fail('Program load timed out; outcome unknown',
                              'Check the pendant program, then press START TELEOP.')
                else:
                    self.fail(f'Dashboard {self.pending} request timed out',
                              'Check the UR driver and network, then press START TELEOP.')
            return None

        if 'remote' not in self.results:
            return self.action('remote', 'CHECKING_REMOTE_CONTROL',
                               'Checking robot Remote Control state…', now)
        if self.results.get('remote') is not True:
            self.fail(LOCAL_CONTROL_TEXT, 'Select Remote Control on the pendant.')
            return None

        # Safety and mode checks are Dashboard facts, not transport success.
        if 'safety' not in self.results:
            return self.action('safety', 'CHECKING_SAFETY_MODE', 'Checking robot safety mode…', now)
        safety = self.results.get('safety')
        if safety not in (1, 2):
            self.fail(f'Robot safety mode {safety} blocks program start',
                      'Resolve the protective/emergency condition on the pendant. '
                      'This application never resets safety faults.')
            return None
        if 'mode' not in self.results:
            return self.action('mode', 'CHECKING_ROBOT_MODE', 'Checking robot mode…', now)
        mode = self.results.get('mode')
        if mode != 7:
            self.fail(f'Robot mode {mode}: RUNNING (7) is required before Play',
                      'Power on and release brakes on the pendant as a separate deliberate action.')
            return None
        if obs.get('a_held') is True:
            self.wait('WAITING_FOR_READINESS', 'Release A before starting the robot program',
                      'WAITING — RELEASE A')
            return None
        if obs.get('enable_requested') is True or obs.get('armed') is True:
            self.fail('Existing arm enable intent or armed clutch blocks program start',
                      'Press STOP TELEOP first, then press START TELEOP.')
            return None
        if 'loaded' not in self.results:
            return self.action('loaded', 'CHECKING_LOADED_PROGRAM',
                               'Checking the loaded robot program…', now)
        loaded = self.results.get('loaded')
        running = self.results.get('running')
        if (self.play_attempted and running is not True and self.play_wait_deadline is not None
                and now >= self.play_wait_deadline):
            self.fail('Play was acknowledged but the program is not running',
                      'Inspect the pendant and driver log. Do not retry automatically.')
            return None
        # A fresh program-running readback is required before any Play decision.
        if running is None or (self.play_attempted and not running and
                               now - self.running_checked_at >= .25):
            return self.action('running', 'CHECKING_LOADED_PROGRAM',
                               'Checking whether the approved program is running…', now,
                               timeout='running')

        if running is True:
            if not same_program(loaded, self.config.program):
                self.fail(f'A different robot program is running: {loaded or "unknown"}',
                          'Stop it on the pendant or load the approved program explicitly. '
                          'This application never replaces an unrelated running program.')
                return None
            return self._wait_for_driver(obs, now)

        if not same_program(loaded, self.config.program):
            if self.results.get('load_acknowledged') and 'loaded' in self.results:
                self.fail(f'Load acknowledged but loaded program is {loaded or "unknown"}',
                          'Confirm the configured External Control program name in Engineering tools.')
                return None
            return self.action('load', 'LOADING_PROGRAM',
                               f'Loading approved program {self.config.program}…', now,
                               timeout='load')

        if self.play_attempted:
            if now < (self.deadline or now):
                self.wait('WAITING_FOR_PROGRAM_RUNNING',
                          'Play acknowledged; waiting for program-running readback…',
                          'WAITING — PROGRAM RUNNING')
                return None
            self.fail('Play was acknowledged but the program is not running',
                      'Inspect the pendant and driver log. Do not retry automatically.')
            return None
        return self.action('play', 'PLAYING_PROGRAM', 'Starting the approved robot program…', now,
                           timeout='play')

    def _wait_for_driver(self, obs, now):
        if self._driver_ready(obs):
            if self.ready_since is None:
                self.ready_since = now
            self.state, self.stage = 'READY', 'PROGRAM RUNNING'
            self.message = 'External Control program running; preparing the arm.'
            self.requested = False
            self.pending = None
            return None
        if obs.get('driver_fault') is True:
            self.fail('Robot driver reported a fault after program start',
                      'Inspect the driver log and pendant; do not treat the program as ready.')
            return None
        self.wait('WAITING_FOR_DRIVER_CONNECTION',
                  'Program running; waiting for the driver connection and speed scaling…',
                  'WAITING — DRIVER CONNECTION')
        if self.ready_since is None:
            self.ready_since = now
        if now - self.ready_since >= self.config.timeouts['driver']:
            self.fail('Program is running but the UR driver did not become ready',
                      'Check the workstation IP, reverse interface and driver log, then restart ROS.')
        return None


@dataclass(frozen=True)
class ProgramStopResult:
    """Outcome of the graceful Dashboard stop, never inferred from a transport call."""
    status: str
    detail: str = ''

    @property
    def confirmed(self):
        return self.status in ('stopped', 'already_stopped')


class ShutdownCoordinator:
    """Order program stop before process signals; neither step blocks forever.

    ``program_stop`` must be bounded internally and return :class:`ProgramStopResult`.
    Its failure is reported but never prevents the existing fail-safe process stop.
    """
    def __init__(self, program_stop, signal_processes):
        self.program_stop = program_stop
        self.signal_processes = signal_processes
        self.result = None

    def run(self):
        try:
            self.result = self.program_stop()
        except Exception as error:  # Never let graceful shutdown wedge termination.
            self.result = ProgramStopResult('unknown', str(error))
        self.signal_processes()
        return self.result
