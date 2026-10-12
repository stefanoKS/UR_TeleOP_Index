#!/usr/bin/env python3
"""Offline synthetic comparison, or metrics from an existing ROS2 SQLite bag.
Never publishes commands. Source the ROS workspace before running.
"""
import argparse
import json
import numpy as np
from ur_vr_teleop.comparison import simulate, metrics


def bag_report(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    records = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == '/vr_real/operator_status':
            row = json.loads(deserialize_message(data, String).data)
            row['t'] = stamp/1e9
            records.append(row)
    report = {}
    for mode in ('position', 'velocity'):
        selected = [r for r in records if r.get('teleop_control_mode') == mode and r.get('velocity_diagnostics')]
        if len(selected) < 2: continue
        t = np.array([r['t'] for r in selected])
        d = [r['velocity_diagnostics'] for r in selected]
        valid = [i for i, v in enumerate(d) if v.get('measured_tcp_velocity') is not None]
        if len(valid) < 2: continue
        t = t[valid]; d = [d[i] for i in valid]
        v = np.array([r['measured_tcp_velocity'][:3] for r in d])
        hand = np.array([r.get('observed_hand_velocity', r['filtered_hand_velocity'])[:3] for r in d])
        age = np.array([r['command_age_s'] for r in d if r['command_age_s'] is not None])
        dt = np.diff(t)
        continuous = (dt > 0) & (dt < .1)
        stationary = (np.linalg.norm(hand[:-1], axis=1) < .001) & continuous
        accel = np.diff(v, axis=0)[continuous]/dt[continuous, None]
        # Travel while the filtered hand is stationary includes the normal stop.
        travel = np.sum(np.linalg.norm(v[:-1][stationary], axis=1)*dt[stationary])
        stops = []
        stop_events = []
        reversal_latencies = []
        positions = np.array([r.get("measured_tcp_position", [np.nan]*3) for r in d])
        moving = np.linalg.norm(hand, axis=1) >= .001
        for i in np.flatnonzero(moving[:-1] & ~moving[1:])+1:
            for j in range(i, len(t)):
                if j > i and (t[j]-t[j-1] >= .1 or moving[j]): break
                if np.linalg.norm(v[j]) < .001:
                    stops.append(t[j]-t[i]); break
        for i in np.flatnonzero(moving[:-1] & ~moving[1:])+1:
            end = i+1
            while end < len(t) and not moving[end] and t[end]-t[end-1] < .1:
                end += 1
            direction = hand[i-1]/max(np.linalg.norm(hand[i-1]), 1e-12)
            projected = (positions[i:end]-positions[i]) @ direction
            if len(projected) and np.all(np.isfinite(projected)):
                stop_events.append({'at_s': float(t[i]-t[0]),
                    'overshoot_from_hand_stop_m': float(max(0., np.max(projected))),
                    'oscillation_peak_to_peak_m': float(np.ptp(projected[len(projected)//2:]))})
        for i in range(1, len(t)):
            if not moving[i]: continue
            previous = i-1
            while previous >= 0 and not moving[previous] and t[i]-t[previous] < .5:
                previous -= 1
            if previous < 0 or np.dot(hand[i], hand[previous]) >= 0: continue
            direction = hand[i]/np.linalg.norm(hand[i])
            for j in range(i, len(t)):
                if j > i and (t[j]-t[j-1] >= .1 or not moving[j]): break
                if np.dot(v[j], direction) > .001:
                    reversal_latencies.append(float(t[j]-t[i])); break
        report[mode] = {'samples': len(d), 'stationary_hand_travel_m': float(travel),
            'command_age_p50_s': float(np.median(age)) if len(age) else None,
            'command_age_p95_s': float(np.quantile(age, .95)) if len(age) else None,
            'command_period_jitter_std_s': float(np.std(dt[continuous])) if np.any(continuous) else None,
            'measured_acceleration_rms_m_s2': float(np.sqrt(np.mean(accel**2))) if len(accel) else None,
            'stop_latency_s': stops, 'post_stop_events': stop_events,
            'reversal_latency_s': reversal_latencies, 'clamping_samples': sum(r['clamped'] for r in d)}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', help='Existing SQLite ROS2 bag directory (read only)')
    parser.add_argument('--delay', type=float, default=.08)
    parser.add_argument('--lag', type=float, default=.1)
    parser.add_argument('--acceleration', type=float, default=.3)
    args = parser.parse_args()
    if args.delay < 0 or args.lag < 0 or args.acceleration <= 0:
        parser.error('delay/lag must be nonnegative and acceleration positive')
    result = bag_report(args.bag) if args.bag else {
        scenario: {mode: metrics(simulate(mode, scenario, args.delay, args.lag, args.acceleration))
                   for mode in ('position', 'velocity')} for scenario in ('hold', 'reversal', 'small')}
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__': main()
