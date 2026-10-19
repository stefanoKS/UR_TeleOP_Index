"""Authorization shared by calibration UI and worker (backend checks are decisive)."""
import math


def stationary_disarmed(status):
    velocities = status.get('measured_joint_velocity')
    return (status.get('armed') is False and status.get('enable_requested') is False
            and status.get('a_held') is False and status.get('setup_feedback_fresh') is True
            and status.get('robot_fault') is False
            and isinstance(velocities, list) and len(velocities) == 6
            and all(type(v) in (float, int) and math.isfinite(v) and abs(v) < .02 for v in velocities))


def jog_target(measured, toward_closed, config):
    config.validate(calibrated=True)
    direction = 1 if config.closed_ticks > config.open_ticks else -1
    if not toward_closed:
        direction *= -1
    return max(config.min_ticks, min(config.max_ticks, measured + direction * config.jog_ticks))
