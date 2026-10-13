"""Local verified setup. No enable intent, external report or expiry is stored.

The record combines instrumented observations and an authorized person's physical
assessment. It is not a signature, authentication system or safety certification.
"""
import json
import os
import tempfile
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from .legacy_engineering import LIMITS, digest, context, main

VERSION = 5
CHECKS = ('mapping_x', 'mapping_y', 'mapping_z', 'orientation_x', 'orientation_y',
          'orientation_z', 'preview_loss', 'a_stop', 'tracking_stop', 'fresh_clutch', 'software_stop')
CONTROL_MODULES = ('tracking.py', 'tracking_quality.py', 'validation.py', 'gate.py',
                   'commissioning.py', 'preview.py', 'desktop.py', 'startup.py')


def motion_packages():
    packages = ('ros-humble-moveit-servo', 'ros-humble-ruckig',
                'ros-humble-ur-client-library', 'ros-humble-ur-controllers',
                'ros-humble-ur-robot-driver')
    try:
        output = subprocess.check_output(
            ['dpkg-query', '-W', '-f=${Package}=${Version}\n', *packages],
            text=True, stderr=subprocess.PIPE, timeout=2.)
        return dict(line.split('=', 1) for line in output.splitlines())
    except (OSError, subprocess.SubprocessError, ValueError) as error:
        raise ValueError('Motion software inventory unavailable. Open Advanced Diagnostics.') from error


def identity(settings, root, tracking):
    root = Path(root).resolve()
    robot = dict(settings['robot'])
    robot['calibration_sha256'] = digest(robot['calibration_file'])
    ignored = LIMITS + ('alignment_verified', 'optical_loss_test_verified',
        'motion_output_enabled', 'teleop_control_mode', 'engineering_context',
        'engineering_record', 'engineering_required', 'joint_acceleration_warning_rad_s2',
        'single_station_motion_scale')
    teleop = {k: v for k, v in settings['teleop'].items() if k not in ignored}
    paths = [root/'dependencies.repos', root/'ur_teleop/config/commissioning_controllers.yaml']
    package = root/'ur_teleop_ws/src/ur_vr_teleop'
    paths += [package/'ur_vr_teleop'/name for name in CONTROL_MODULES]
    paths += list((package/'launch').glob('*.py'))
    paths += [root/'ur_teleop_ws/src/ur_vr_sim_teleop/ur_vr_sim_teleop/core.py']
    for name in ('ur_vr_bridge', 'ur_teleop_controllers'):
        paths += [p for p in (root/'ur_teleop_ws/src'/name).rglob('*')
                  if p.suffix in ('.cpp', '.hpp', '.yaml') and 'test' not in p.parts]
    # Required files fail closed. Tests construct an explicit miniature installation.
    compatibility = json.loads(Path(__file__).with_name('setup_source_compatibility.json').read_text())
    software = {str(p.relative_to(root)): digest(p) for p in sorted(set(paths))}
    software = {name: compatibility.get(name, {}).get(value, value) for name, value in software.items()}
    return dict(version=VERSION, installation=str(root), host=digest('/etc/machine-id'),
                robot=robot, teleop=teleop, tracking_sha256=digest(tracking),
                controller_serial='LHR-4632363E', software=software, motion_packages=motion_packages())


def validate_record(record, settings, current):
    if record.get('version') != VERSION:
        raise ValueError('Saved setup is incompatible. Open Setup Wizard.')
    if record.get('identity') != current:
        raise ValueError('Robot, mapping, tool, tracking or control configuration changed. Open Setup Wizard.')
    results = record.get('results', {})
    if record.get('migration') != 'verified-v4':
        if not all(isinstance(results.get(k), dict) and results[k].get('observed') is True
                   and results[k].get('physical_pass') is True for k in CHECKS):
            raise ValueError('Setup checks are incomplete. Open Setup Wizard.')
        if not isinstance(record.get('physical_assessment'), str) or not record['physical_assessment'].strip():
            raise ValueError('Physical assessment is missing. Open Setup Wizard.')
    elif not record.get('legacy_record_sha256'):
        raise ValueError('Setup migration is incomplete. Open Setup Wizard.')
    modes = record['approved_modes']
    if not isinstance(modes, list) or not modes or any(m not in ('velocity', 'position') for m in modes):
        raise ValueError('Saved control modes are damaged. Open Setup Wizard.')
    if settings['teleop']['teleop_control_mode'] not in modes:
        raise ValueError('This control mode needs verification. Open Setup Wizard.')
    for key in LIMITS:
        value, cap = settings['teleop'][key], record['limits'][key]
        if isinstance(value, bool) or isinstance(cap, bool) or not 0 < value <= cap < float('inf'):
            raise ValueError('Motion limits exceed the verified setup. Open Setup Wizard.')


class Verification:
    def __init__(self, encoded):
        self.context = json.loads(encoded)
        self.valid = False
        self.reason = 'Initial setup has not been completed. Open Setup Wizard.'
        self.technical = ''

    def refresh(self, parameters):
        self.valid = False
        try:
            c = self.context
            settings = dict(c['settings'], teleop=dict(c['settings']['teleop'], **parameters))
            path = Path(c['record']).with_name('verified_setup.json')
            record = json.loads(path.read_text())
            validate_record(record, settings, identity(settings, c['root'], c['tracking']))
            self.valid, self.reason, self.technical = True, 'Saved setup verified', ''
        except FileNotFoundError as error:
            self.reason = 'Initial setup or a required configuration file is missing. Open Setup Wizard.'
            self.technical = repr(error) + ('; caused by '+repr(error.__cause__) if error.__cause__ else '')
        except (OSError, ValueError, TypeError, KeyError, AttributeError) as error:
            self.reason = (str(error) if type(error) is ValueError else
                           'Saved setup cannot be read or is damaged. Open Setup Wizard.')
            self.technical = repr(error) + ('; caused by '+repr(error.__cause__) if error.__cause__ else '')
        return self.valid


def atomic_save(target, record):
    target = Path(target)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=target.parent, delete=False) as stream:
            name = stream.name
            json.dump(record, stream, indent=2, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, target)
        directory = os.open(target.parent, os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if name and Path(name).exists():
            Path(name).unlink()


def invalidate_setup(encoded):
    """Explicitly beginning recalibration retires prior eligibility, preserving evidence."""
    c = json.loads(encoded)
    target = Path(c['record']).with_name('verified_setup.json')
    if target.exists():
        import uuid
        os.replace(target, target.with_name('verified_setup.previous-'+uuid.uuid4().hex+'.json'))


def save_setup(encoded, parameters, baseline, results, assessment):
    c = json.loads(encoded)
    settings = dict(c['settings'], teleop=dict(c['settings']['teleop'], **parameters))
    current = identity(settings, c['root'], c['tracking'])
    if current != baseline:
        raise ValueError('Configuration changed during setup. Repeat verification.')
    record = dict(version=VERSION, identity=current, results=results,
                  physical_assessment=assessment.strip(),
                  approved_modes=[settings['teleop']['teleop_control_mode']],
                  limits={k: settings['teleop'][k] for k in LIMITS},
                  recorded_at=datetime.now(timezone.utc).isoformat())
    validate_record(record, settings, current)
    atomic_save(Path(c['record']).with_name('verified_setup.json'), record)
    return record


def migrate_legacy(encoded, parameters):
    """Explicit GUI import: validate old evidence, envelope and control identity first."""
    from . import legacy_engineering as legacy
    c = json.loads(encoded)
    settings = dict(c['settings'], teleop=dict(c['settings']['teleop'], **parameters))
    path = Path(c['record'])
    old = json.loads(path.read_text())
    if old.get('version') != 4 or old.get('workcell_validated') is not True:
        raise ValueError('Legacy record is incomplete; use Initial Setup.')
    expiry = datetime.fromisoformat(old['valid_until'])
    if expiry.tzinfo is None or expiry <= datetime.now(timezone.utc):
        raise ValueError('Legacy record expired; use Initial Setup.')
    if not all(isinstance(old.get(k), str) and old[k].strip() for k in ('engineer', 'workcell', 'evidence')):
        raise ValueError('Legacy evidence is incomplete; use Initial Setup.')
    if digest(old['evidence']) != old['evidence_sha256']:
        raise ValueError('Legacy evidence changed; use Initial Setup.')
    previous = old['identity']
    expected = legacy.identity(settings, c['root'], c['tracking'])
    # UI-only changes may be ignored; control changes require an exact reviewed transition.
    transitions = json.loads(Path(__file__).with_name('setup_migration.json').read_text())
    current = identity(settings, c['root'], c['tracking'])
    for name, value in current['software'].items():
        before = previous['software'].get(name)
        if before != value and transitions.get(name) != [before, value]:
            raise ValueError('Control software changed since the legacy checks. Repeat Initial Setup.')
    if {k:v for k,v in previous.items() if k != 'software'} != {k:v for k,v in expected.items() if k != 'software'}:
        raise ValueError('Legacy robot, calibration or configuration differs. Repeat Initial Setup.')
    record = dict(version=VERSION, identity=current, migration='verified-v4',
                  legacy_record_sha256=digest(path), results={'legacy': old},
                  approved_modes=old['approved_modes'], limits=old['limits'],
                  recorded_at=datetime.now(timezone.utc).isoformat())
    validate_record(record, settings, current)
    atomic_save(path.with_name('verified_setup.json'), record)
    return record
