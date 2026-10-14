"""Offline stage comparison. No node, publisher, service client or robot connection.

All comparisons use bag receive time. Header/source clocks are audited separately,
not subtracted from each other or silently realigned. This exposes transport jitter
but cannot establish absolute actuator latency or causality.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import numpy as np
from .validation import JOINTS

PREFIX = '/scaled_joint_trajectory_controller/'
TRAJECTORIES = {'/vr_real/servo_joint_targets': 'servo', PREFIX+'smoothed_command': 'native',
                PREFIX+'joint_trajectory': 'gate'}


def series(rows):
    if not rows:
        return np.empty(0), np.empty((0, 0))
    t = np.array([r[0] for r in rows], dtype=float)
    y = np.array([r[1] for r in rows], dtype=float)
    keep = np.r_[True, np.diff(t) > 0] & np.all(np.isfinite(y), axis=1)
    return t[keep], y[keep]


def interpolate(t, values, target, max_gap=.1):
    """No extrapolation or interpolation across missing observation intervals."""
    out = np.full((len(target), values.shape[1]), np.nan)
    if len(t) < 2:
        return out
    right = np.searchsorted(t, target, side='right').clip(1, len(t)-1)
    left = right-1
    ok = (target >= t[0]) & (target <= t[-1]) & (t[right]-t[left] <= max_gap)
    f = (target[ok]-t[left[ok]])/(t[right[ok]]-t[left[ok]])
    out[ok] = values[left[ok]]*(1-f[:, None])+values[right[ok]]*f[:, None]
    return out


def timing(t):
    dt = np.diff(t)
    good = dt[dt > 0]
    if not len(good):
        return {'samples': len(t), 'frequency_hz': None}
    period = float(np.median(good))
    return dict(samples=len(t), frequency_hz=1/period, median_period_s=period,
                max_gap_s=float(max(good)), nonincreasing_updates=int(sum(dt <= 0)),
                estimated_missing_updates=int(sum(np.maximum(0, np.rint(good/period)-1))),
                note='Missing-update estimate assumes the observed median period is nominal')


def reversals(t, y, threshold=.001):
    indices = np.flatnonzero(np.abs(y) > threshold)
    if len(indices) < 2:
        return 0
    # Ignore deadband samples, but never count across long observation gaps.
    gaps = np.r_[0, np.cumsum(np.diff(t) > .1)]
    return int(sum((np.sign(y[indices[1:]]) != np.sign(y[indices[:-1]])) &
                   (gaps[indices[1:]] == gaps[indices[:-1]])))


def frequency(t, y, threshold=.001):
    if len(t) < 20 or np.max(np.abs(y-np.mean(y))) < threshold:
        return None
    if np.any(np.diff(t) > .1):
        return None
    dt = float(np.median(np.diff(t)))
    grid = np.arange(t[0], t[-1], dt)
    z = np.interp(grid, t, y)
    z -= np.mean(z)
    spectrum = np.abs(np.fft.rfft(z*np.hanning(len(z))))**2
    f = np.fft.rfftfreq(len(z), dt)
    if len(f) < 4:
        return None
    peak = int(np.argmax(spectrum[1:])+1)
    # At least two cycles and concentrated energy; a single stop is not oscillation.
    if f[peak]*(t[-1]-t[0]) < 2 or spectrum[peak]/max(sum(spectrum[1:]), 1e-30) < .2:
        return None
    return float(f[peak])


def lag(t, command, feedback, max_lag=.5):
    if len(t) < 30 or np.any(np.diff(t) > .1):
        return None
    dt = float(np.median(np.diff(t)))
    grid = np.arange(t[0], t[-1], dt)
    a, b = np.interp(grid, t, command), np.interp(grid, t, feedback)
    if np.std(a) < .001 or np.std(b) < .001:
        return None
    scored = []
    for k in range(-min(int(max_lag/dt), len(a)//3), min(int(max_lag/dt), len(a)//3)+1):
        x, y = (a[:-k], b[k:]) if k > 0 else (a[-k:], b[:k]) if k < 0 else (a, b)
        if np.std(x) > .001 and np.std(y) > .001:
            scored.append((float(np.corrcoef(x, y)[0, 1]), k*dt))
    if not scored:
        return None
    score, delay = max(scored)
    return dict(feedback_lag_s=delay, correlation=score,
                note='Receive-time correlation estimate; periodic signals may have ambiguous lag')


def stops(t, command, measured, reference=None, position=None, speed=.001, tolerance=.002):
    moving = np.abs(command) > speed
    result = []
    for i in np.flatnonzero(moving[:-1] & ~moving[1:] & np.isfinite(command[1:]))+1:
        end = i+1
        while end < len(t) and np.isfinite(command[end]) and not moving[end] and t[end]-t[end-1] <= .1:
            end += 1
        row = dict(at_s=float(t[i]), measured_standstill_s=None, settling_s=None,
                   peak_overshoot_rad=None)
        if t[i]-t[i-1] > .1:
            continue
        def sustained(mask):
            # Require at least 0.5 s, with no later excursion in the observed stop window.
            bad = np.flatnonzero(~mask)
            j = int(bad[-1]+1) if len(bad) else 0
            return j if j < len(mask) and t[end-1]-t[i+j] >= .5 else None
        j = sustained(np.isfinite(measured[i:end]) & (np.abs(measured[i:end]) <= speed))
        if j is not None:
            row['measured_standstill_s'] = float(t[i+j]-t[i])
        if reference is not None and position is not None:
            ref, pos = reference[i:end], position[i:end]
            if np.all(np.isfinite(ref)) and np.all(np.isfinite(pos)) and np.ptp(ref) <= tolerance:
                final = float(ref[-1]); direction = np.sign(command[i-1])
                row['peak_overshoot_rad'] = float(max(0., np.max((pos-final)*direction)))
                j = sustained(np.abs(pos-final) <= tolerance)
                if j is not None:
                    row['settling_s'] = float(t[i+j]-t[i])
                row['final_reference_rad'] = final
        result.append(row)
    return result


def read_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import yaml
    path = Path(path)
    if (path/'bag/metadata.yaml').exists():
        path = path/'bag'
    metadata = yaml.safe_load((path/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=metadata['storage_identifier']),
                rosbag2_py.ConverterOptions('', ''))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    streams, clocks, received, errors = defaultdict(list), defaultdict(list), defaultdict(list), defaultdict(int)
    configurations = []
    start = None
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if start is None:
            start = stamp
        t = (stamp-start)/1e9
        received[topic].append(t)
        if topic not in TRAJECTORIES and topic not in (
            '/joint_states', '/vr_real/operator_status', '/vr_real/servo_node/delta_twist_cmds',
            '/vr/controller/pose', '/vr/controller/source_time', '/vr/controller/joy',
            '/speed_scaling_state_broadcaster/speed_scaling'):
            continue
        try:
            msg = deserialize_message(raw, get_message(types[topic]))
            if hasattr(msg, 'header'):
                header = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
                if header > 0:
                    clocks[topic].append((t, header, stamp/1e9-header))
            if topic in TRAJECTORIES or topic == '/joint_states':
                names = list(msg.name if topic == '/joint_states' else msg.joint_names)
                if len(names) != 6 or set(names) != set(JOINTS):
                    raise ValueError('Joint membership mismatch')
                order = [names.index(j) for j in JOINTS]
                stage = TRAJECTORIES.get(topic, 'actual')
                if stage != 'actual' and len(msg.points) != 1:
                    raise ValueError('Expected one short-horizon target')
                point = msg if stage == 'actual' else msg.points[0]
                for field, attribute in [('q', 'position' if stage == 'actual' else 'positions'),
                                         ('v', 'velocity' if stage == 'actual' else 'velocities')]:
                    values = getattr(point, attribute)
                    if len(values) == 6 and all(np.isfinite(values)):
                        streams[stage+'_'+field].append((t, [values[j] for j in order]))
            elif topic == '/vr_real/operator_status':
                status = json.loads(msg.data)
                diag = status.get('velocity_diagnostics') or {}
                for key in ('raw_hand_velocity', 'filtered_hand_velocity', 'observed_hand_velocity',
                            'measured_tcp_velocity', 'measured_tcp_position', 'published_twist'):
                    value = diag.get(key)
                    if isinstance(value, list) and len(value) in (3, 6) and all(np.isfinite(value)):
                        streams[key].append((t, value))
                cfg = dict(parameters=status.get('effective_parameters'), native=status.get('hardware_smoothing'))
                if not configurations or configurations[-1]['values'] != cfg:
                    configurations.append(dict(at_s=t, values=cfg))
            elif topic.endswith('delta_twist_cmds'):
                v = msg.twist
                streams['cartesian'].append((t, [v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z]))
            elif topic == '/vr/controller/pose':
                p = msg.pose
                streams['hand_pose'].append((t,[p.position.x,p.position.y,p.position.z,
                                               p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]))
            elif topic == '/vr/controller/joy':
                streams['a_button'].append((t,[msg.buttons[4]]))
            elif topic == '/vr/controller/source_time':
                clocks[topic].append((t, float(msg.data), None))
            else:
                streams['speed_scaling'].append((t,[msg.data]))
        except (ValueError, TypeError, KeyError, IndexError, AttributeError):
            errors[topic] += 1
    return streams, dict(topic_timing={k:timing(np.array(v)) for k,v in received.items()},
                         clocks={k:clock_report(v) for k,v in clocks.items()},
                         malformed_messages=dict(errors), effective_configuration_changes=configurations)


def clock_report(rows):
    source = np.array([r[1] for r in rows]); offsets = [r[2] for r in rows if r[2] is not None]
    result = dict(source_nonincreasing=int(sum(np.diff(source) <= 0)),
                  source_largest_gap_s=float(max(np.diff(source))) if len(source)>1 else None)
    if offsets:
        result.update(receive_minus_header_median_s=float(np.median(offsets)),
                      receive_minus_header_p95_s=float(np.quantile(offsets,.95)),
                      warning='Offsets include transport delay; large offsets may indicate different clock epochs')
    else:
        result['warning'] = 'Raw device clock; never directly compared with ROS epoch'
    return result


def analyze(streams, speed=.001, tolerance=.002):
    arrays = {k:series(v) for k,v in streams.items()}
    report = dict(time_basis='seconds since first bag receive timestamp', joints={},
                  thresholds=dict(standstill_rad_s=speed, position_rad=tolerance, dwell_s=.5),
                  unavailable=[k for k in ('servo_v','native_v','actual_v','native_q','actual_q',
                      'cartesian','measured_tcp_velocity','raw_hand_velocity') if k not in arrays],
                  interpretation='Stage metrics are evidence for investigation, not an identified root cause.')
    for j, name in enumerate(JOINTS):
        row = {'stages': {}}
        for stage in ('servo','native','actual'):
            t, v = arrays.get(stage+'_v', (np.empty(0), None))
            if len(t):
                row['stages'][stage] = dict(sign_reversals=reversals(t,v[:,j],speed),
                    oscillation_frequency_hz=frequency(t,v[:,j],speed), peak_speed_rad_s=float(max(abs(v[:,j]))))
        if 'actual_v' in arrays:
            t, v = arrays['actual_v']
            for stage in ('servo','native'):
                if stage+'_v' in arrays:
                    command = interpolate(*arrays[stage+'_v'], t)[:,j]
                    mask = np.isfinite(command)
                    row[stage+'_feedback_offset'] = lag(t[mask],command[mask],v[mask,j])
                    reference = interpolate(*arrays['native_q'],t)[:,j] if 'native_q' in arrays else None
                    position = interpolate(*arrays['actual_q'],t)[:,j] if 'actual_q' in arrays else None
                    row[stage+'_stops'] = stops(t,command,v[:,j],reference,position,speed,tolerance)
        for stage in ('servo', 'native', 'actual'):
            if stage+'_v' in arrays:
                st, sv = arrays[stage+'_v']
                dt = np.diff(st); good = (dt > 0) & (dt <= .1)
                acceleration = np.diff(sv[:,j])[good]/dt[good]
                row['stages'][stage]['peak_receive_time_acceleration_estimate_rad_s2'] = float(max(abs(acceleration))) if len(acceleration) else None
        if 'actual_q' in arrays and 'native_q' in arrays:
            t, q = arrays['actual_q']; ref = interpolate(*arrays['native_q'],t)[:,j]
            error = q[:,j]-ref; error = error[np.isfinite(error)]
            row['tracking_error_rad'] = dict(rms=float(np.sqrt(np.mean(error**2))), peak=float(max(abs(error)))) if len(error) else None
        report['joints'][name] = row
    if 'speed_scaling' in arrays:
        _, scale = arrays['speed_scaling']
        if len(scale): report['speed_scaling_percent'] = dict(min=float(scale.min()),max=float(scale.max()),std=float(scale.std()))
    return report


def plots(streams, output, configurations=()):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    arrays = {k:series(v) for k,v in streams.items()}
    for j,name in enumerate(JOINTS):
        fig, axes = plt.subplots(2,1,figsize=(11,6),sharex=True)
        for stage in ('servo','gate','native','actual'):
            for axis,field,unit in zip(axes,('v','q'),('rad/s','rad')):
                if stage+'_'+field in arrays:
                    t,y = arrays[stage+'_'+field]; axis.plot(t,y[:,j],label=stage,linewidth=1)
                axis.set_ylabel(unit); axis.grid(alpha=.25)
        limits = [(r['at_s'], (r['values'].get('native') or {}).get('velocity')) for r in configurations]
        limits = [(t,v) for t,v in limits if type(v) in (float,int) and np.isfinite(v)]
        if limits:
            lt,lv = map(np.array,zip(*limits))
            end = max((v[0][-1] for v in arrays.values() if len(v[0])),default=lt[-1])
            lt=np.r_[lt,end];lv=np.r_[lv,lv[-1]]
            axes[0].step(lt,lv,where='post',color='grey',linestyle='--',label='effective native speed cap')
            axes[0].step(lt,-lv,where='post',color='grey',linestyle='--')
        for axis in axes:
            if axis.lines:axis.legend()
            else:axis.text(.5,.5,'Data unavailable',transform=axis.transAxes,ha='center')
        axes[-1].set_xlabel('Bag receive time (s)');fig.suptitle(name)
        fig.tight_layout();fig.savefig(output/f'joint_{j+1}.png',dpi=140);plt.close(fig)
    for filename,keys in [('cartesian',('cartesian','measured_tcp_velocity','raw_hand_velocity','filtered_hand_velocity')),
                           ('hand_pose',('hand_pose',)), ('speed_scaling',('speed_scaling',))]:
        available = [k for k in keys if k in arrays and len(arrays[k][0])]
        if not available:continue
        dim = max(arrays[k][1].shape[1] for k in available)
        fig,axes=plt.subplots(dim,1,figsize=(11,2*dim),sharex=True,squeeze=False)
        for key in available:
            t,y=arrays[key]
            for j in range(y.shape[1]):axes[j,0].plot(t,y[:,j],label=key,linewidth=1)
        for j,axis in enumerate(axes[:,0]):
            axis.legend();axis.grid(alpha=.25)
            axis.set_ylabel(('m/s' if j<3 else 'rad/s') if filename=='cartesian' else ('m' if j<3 else 'quaternion') if filename=='hand_pose' else '%')
        axes[-1,0].set_xlabel('Bag receive time (s)');fig.tight_layout()
        fig.savefig(output/(filename+'.png'),dpi=140);plt.close(fig)


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',help='Existing bag or sway recording directory')
    parser.add_argument('--output',help='Report directory (default: analysis beside bag)')
    parser.add_argument('--standstill',type=float,default=.001,help='Analysis threshold rad/s, not a motion limit')
    parser.add_argument('--position-tolerance',type=float,default=.002)
    options=parser.parse_args(args)
    if not 0 < options.standstill < float('inf') or not 0 < options.position_tolerance < float('inf'):
        parser.error('Analysis thresholds must be positive and finite')
    output=Path(options.output or str(Path(options.bag)/'analysis'));output.mkdir(parents=True,exist_ok=True)
    streams,metadata=read_bag(options.bag)
    report=analyze(streams,options.standstill,options.position_tolerance);report.update(metadata)
    recording=Path(options.bag)/'recording.json'
    if not recording.exists():recording=Path(options.bag).parent/'recording.json'
    if recording.exists():report['recording_metadata']=json.loads(recording.read_text())
    report['limits_note'] = 'Effective configuration changes include Cartesian speed/acceleration and native velocity/acceleration/jerk. Estimated acceleration uses receive intervals and may include transport jitter.'
    (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    lines=['# Offline sway comparison','', 'Receive-time comparison; source clock offsets are audited in report.json.',
           'Missing data: '+(', '.join(report['unavailable']) or 'none of the primary streams'), '',
           '| Joint | Servo reversals | Native reversals | Feedback reversals | Feedback frequency Hz |',
           '|---|---:|---:|---:|---:|']
    for name,row in report['joints'].items():
        stages=row['stages']
        values=[stages.get(k,{}).get('sign_reversals','unavailable') for k in ('servo','native','actual')]
        lines.append('| '+name+' | '+' | '.join(map(str,values))+' | '+str(stages.get('actual',{}).get('oscillation_frequency_hz'))+' |')
    lines += ['', 'Inspect hand/Twist first, then Servo, native reference, and measured joints. Oscillation already in a command stream is command-source evidence. Feedback oscillation with a stationary final reference points downstream; it does not establish a UR, payload or mechanical root cause.',
              '', 'Correlation lag is not absolute actuator latency. Stop metrics require 0.5 s of observed standstill; overshoot and settling require a stationary final reference. Null means unavailable or insufficient evidence. See report.json for events, configuration, gaps and clock audits.']
    (output/'report.md').write_text('\n'.join(lines)+'\n')
    plots(streams,output,metadata['effective_configuration_changes'])
    print('Offline report: '+str(output/'report.md'))


if __name__=='__main__':main()
