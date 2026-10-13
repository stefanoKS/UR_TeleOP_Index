"""Engineering evidence record, separate from operator preferences/enable intent.

Records are local administrative evidence, not signatures or sensor proofs.
Only an authorized engineer should have write access to their directory.
"""
import argparse
import hashlib
import json
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path

VERSION = 4
# Lower limits are allowed within the commissioned envelope. Other tuning is
# identity-bound: changing filters/mapping requires engineering revalidation.
LIMITS = ('max_linear_speed_m_s', 'max_angular_speed_deg_s', 'max_joint_speed_rad_s',
          'max_linear_acceleration_m_s2', 'max_angular_acceleration_rad_s2',
          'translation_scale', 'rotation_scale')


def digest(path):
    return hashlib.sha256(Path(path).expanduser().read_bytes()).hexdigest()


def identity(settings, root, tracking):
    root = Path(root).resolve()
    robot = dict(settings['robot'])
    robot['calibration_sha256'] = digest(robot['calibration_file'])
    teleop = {k: v for k, v in settings['teleop'].items()
              if k not in LIMITS + ('alignment_verified', 'optical_loss_test_verified',
                  'motion_output_enabled', 'teleop_control_mode', 'engineering_context', 'engineering_record',
                  'engineering_required')}
    # Source and configuration templates are part of the commissioned version.
    paths = [root/'dependencies.repos', root/'ur_teleop/config/commissioning_controllers.yaml']
    for package in ('ur_vr_teleop', 'ur_vr_sim_teleop', 'ur_vr_bridge', 'ur_teleop_controllers'):
        paths.extend(p for p in (root/'ur_teleop_ws/src'/package).rglob('*')
                     if p.suffix in ('.py', '.cpp', '.hpp', '.yaml') and 'test' not in p.parts)
    software = {str(p.relative_to(root)): digest(p) for p in sorted(paths)}
    return dict(version=VERSION, installation=str(root), host=digest('/etc/machine-id'),
                robot=robot, teleop=teleop, tracking_sha256=digest(tracking),
                controller_serial='LHR-4632363E', software=software)


def context(settings, root, record, tracking=None):
    return json.dumps(dict(settings=settings, root=str(root), record=str(record),
                          tracking=str(tracking or Path.home()/'.config/libsurvive/config.json')))


class Verification:
    def __init__(self, encoded):
        self.context = json.loads(encoded)
        self.valid = False
        self.reason = 'ENGINEERING SETUP REQUIRED'

    def refresh(self, parameters):
        self.valid = False
        try:
            c = self.context
            settings = dict(c['settings'], teleop=dict(c['settings']['teleop']))
            settings['teleop'].update(parameters)
            record = json.loads(Path(c['record']).read_text())
            if record.get('version') != VERSION or record.get('workcell_validated') is not True:
                raise ValueError('missing V4 workcell validation')
            if not all(isinstance(record.get(k), str) and record[k].strip() for k in ('engineer', 'workcell', 'evidence', 'valid_until')):
                raise ValueError('incomplete engineering evidence')
            expiry = datetime.fromisoformat(record['valid_until'])
            if expiry.tzinfo is None or expiry <= datetime.now(timezone.utc):
                raise ValueError('periodic engineering verification expired')
            if digest(record['evidence']) != record['evidence_sha256']:
                raise ValueError('engineering evidence changed')
            if identity(settings, c['root'], c['tracking']) != record['identity']:
                raise ValueError('installation, calibration, mapping, tuning or software changed')
            if settings['teleop']['teleop_control_mode'] not in record['approved_modes']:
                raise ValueError('control mode was not commissioned')
            for key in LIMITS:
                value, cap = settings['teleop'][key], record['limits'][key]
                if not 0 < value <= cap < float('inf'):
                    raise ValueError('effective limit exceeds engineering envelope: ' + key)
            self.valid, self.reason = True, 'Engineering installation and limits verified'
        except (OSError, ValueError, TypeError, KeyError, AttributeError) as error:
            self.reason = 'ENGINEERING SETUP REQUIRED: ' + str(error)
        return self.valid


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--root', default=str(Path.home()/'Documents/Teleop'))
    parser.add_argument('--tracking', default=str(Path.home()/'.config/libsurvive/config.json'))
    parser.add_argument('--approved-modes', nargs='+', choices=('velocity', 'position'),
                        help='Only modes physically validated in the evidence; defaults to the selected mode')
    parser.add_argument('--engineer', required=True)
    parser.add_argument('--workcell', required=True)
    parser.add_argument('--evidence', required=True, help='Existing signed-off commissioning report; never a unit-test result alone')
    parser.add_argument('--valid-until', required=True, help='Site-selected ISO timestamp with timezone')
    parser.add_argument('--attest-workcell-validated', action='store_true', required=True,
                        help='Attest V4 workflow, low-speed physical stops, TCP/payload, collision model and limits were reviewed and validated')
    options = parser.parse_args(args)
    from .preferences import load_settings
    settings = load_settings(options.config)
    target = Path(options.config).resolve().with_name('engineering_record.json')
    evidence = str(Path(options.evidence).resolve())
    record = dict(version=VERSION, engineer=options.engineer, workcell=options.workcell,
                  evidence=evidence, evidence_sha256=digest(evidence), valid_until=options.valid_until,
                  workcell_validated=True, identity=identity(settings, options.root, options.tracking),
                  approved_modes=options.approved_modes or [settings['teleop']['teleop_control_mode']],
                  limits={k: settings['teleop'][k] for k in LIMITS},
                  recorded_at=datetime.now(timezone.utc).isoformat())
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=target.parent, delete=False) as stream:
            name = stream.name
            json.dump(record, stream, indent=2, allow_nan=False)
            stream.flush(); os.fsync(stream.fileno())
        verifier = Verification(context(settings, options.root, name, options.tracking))
        if not verifier.refresh({}):
            raise ValueError(verifier.reason)
        os.replace(name, target)
    finally:
        if name and Path(name).exists():
            Path(name).unlink()
    print('Engineering record written: ' + str(target) + '; no motion authorized or commanded')


if __name__ == '__main__':
    main()
