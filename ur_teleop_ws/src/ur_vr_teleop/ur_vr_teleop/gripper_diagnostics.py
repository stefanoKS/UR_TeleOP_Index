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
