"""Shared operator wording; historical measurements are explicitly labelled."""
import time


def temperature_text(data, fresh=True):
    if not fresh:
        return 'ST3215 feedback unavailable / stale'
    thermal = data.get('temperature', {})
    state = thermal.get('state', 'NORMAL')
    fb = data.get('feedback', {})
    current = (data.get('feedback_valid') and
               0 <= time.monotonic()-fb.get('received_at', -float('inf')) < .25)
    if state == 'NORMAL':
        if current:
            event = data.get('last_temperature_event') or {}
            previous = event.get('feedback', {}).get('temperature_c')
            warning = f'; previous suspect sample {previous} °C (history)' if previous is not None else ''
            return f"ST3215 temperature {fb.get('temperature_c', '—')} °C{warning}"
        return 'ST3215 temperature unavailable / stale'
    titles = {'TEMPERATURE_UNVERIFIED': 'Temperature verification — movement paused',
              'TEMPERATURE_CONFIRMED_HIGH': 'ST3215 OVER-TEMPERATURE — confirmed',
              'RECOVERY_REQUIRED': 'ST3215 suspected temperature spike — recovery required',
              'SENSOR_OR_COMMUNICATION_FAULT': 'ST3215 FEEDBACK ERROR'}
    return (f"{titles.get(state, 'ST3215 feedback unverified')} — movement inhibited\n"
            f"Last verified (historical): {thermal.get('verified_temperature_c', '—')} °C; "
            f"suspicious: {thermal.get('suspicious_temperature_c', '—')} °C; "
            f"confirmations: {thermal.get('confirmation_readings', [])}")


def serial_text(data):
    counters = data.get('serial_diagnostics', {}).get('counters', {})
    return ('Serial: ' + ' · '.join(f'{name}={counters.get(name, 0)}' for name in
            ('rx_timeouts', 'rx_corruption', 'invalid_responses', 'motor_errors', 'reconnects')))


def performance_text(data, fresh=True):
    """Explain which configured limit currently restricts target advancement."""
    if not fresh:
        return 'Performance: feedback unavailable / stale'
    def fmt(value):
        if value is None:
            return '—'
        return f'{value:g}' if isinstance(value, (int, float)) else str(value)
    desired = data.get('desired_ticks')
    measured = data.get('measured_ticks')
    outstanding = data.get('outstanding_error_ticks')
    lead = data.get('target_lead_ticks')
    rate = data.get('effective_rate_ticks_s')
    speed = data.get('sdk_speed')
    acceleration = data.get('sdk_acceleration')
    acknowledged = data.get('last_acknowledged_ticks')
    timing = []
    if data.get('feedback_ms') is not None:
        timing.append(f'feedback {data["feedback_ms"]:.1f} ms')
    if data.get('write_ms') is not None:
        timing.append(f'write {data["write_ms"]:.1f} ms')
    if data.get('motor_not_following'):
        limitation = ('Motor is not following acknowledged targets; outstanding error is '
                      'bounded, not accumulated. Inspect mechanism/load.')
    elif outstanding is not None and lead is not None and abs(outstanding) >= lead:
        limitation = ('Command lead is the active restriction: the target is at the '
                      'configured distance from measured feedback.')
    elif (outstanding is not None and desired is not None and measured is not None and
          abs(outstanding) >= 2):
        limitation = ('Software target rate is the active restriction while the target '
                      'advances toward the trigger endpoint.')
    else:
        limitation = 'No software limit is currently restricting movement.'
    return (f"Target {fmt(desired)} · measured {fmt(measured)} · acknowledged "
            f"{fmt(acknowledged)} ticks · outstanding {fmt(outstanding)} "
            f"(lead limit {fmt(lead)})\n"
            f"Software rate {fmt(rate)} ticks/s · SDK speed {fmt(speed)} · "
            f"acceleration {fmt(acceleration)} · motor "
            f"{'moving' if data.get('motor_moving') else 'stationary'}"
            f"{' · ' + ' / '.join(timing) if timing else ''}\n{limitation}")


def follow_state_text(data):
    """Specific follow label; never collapse every hold into a pickup prompt."""
    fault = data.get('fault_category')
    reason = str(data.get('recovery_reason') or data.get('last_signal_loss_reason') or '')
    if fault == 'COMMUNICATION':
        return 'COMMUNICATION FAULT' + (f' — {reason}' if reason else '')
    if fault == 'MOTOR':
        return 'MOTOR FAULT' + (f' — {reason}' if reason else '')
    if data.get('communication_state') == 'RETRYING':
        return 'RECOVERING SERIAL FEEDBACK — HOLDING'
    state = data.get('follow_state')
    if state == 'FOLLOWING':
        return 'FOLLOWING'
    if state == 'SIGNAL_PAUSED':
        return ('SIGNAL LOST — HOLDING (auto-resume pending)' if data.get('resume_pending')
                else 'SIGNAL LOST — HOLDING (pickup required after recovery)')
    if state == 'RECOVERY_VERIFYING':
        return 'RECOVERING SIGNAL — VERIFYING HEALTHY INPUT'
    if state == 'PICKUP_REQUIRED':
        return 'PICKUP REQUIRED' + (f' — {reason}' if reason else '')
    if state == 'DISABLED':
        return 'DISABLED' + (f' — {reason}' if reason else '')
    if state == 'FAULT':
        return 'FAULT' + (f' — {reason}' if reason else '')
    return 'WAITING FOR FRESH TRIGGER PICKUP'


def fault_text(data):
    category = data.get('fault_category', 'NONE')
    if category in ('NONE', 'TEMPERATURE'):
        return ''
    reason = str(data.get('last_fault_reason') or data.get('reason') or 'Reason unavailable')
    titles = {
        'COMMUNICATION': 'ST3215 COMMUNICATION FAULT',
        'MOTOR': 'ST3215 MOTOR FAULT',
        'MECHANICAL': 'ST3215 MECHANICAL FAULT',
        'CONFIGURATION': 'ST3215 CONFIGURATION ERROR',
        'UNKNOWN': 'ST3215 INTERNAL FAULT',
    }
    title = titles.get(category, titles['UNKNOWN'])
    suffix = '; movement inhibited; inspect application log' if category == 'UNKNOWN' else ''
    return f'{title} — {reason}{suffix}'
