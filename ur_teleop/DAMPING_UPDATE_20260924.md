# Adjustable velocity-feedback damping — 24 September 2026

Added three operator controls, supported by Apply, Apply and save parameters,
Reset to default parameters, backend status and loading on next startup:

| Control | Default | Accepted range |
|---|---:|---:|
| Translation damping | 1.0 | 0–1; 0 disables |
| Rotation damping | 1.0 | 0–1; 0 disables |
| Damping velocity filter | 50 ms | 5–200 ms |

These are dimensionless gains in the Cartesian velocity-command loop, not UR
firmware torque/impedance constants or a claimed critically damped tuning.
For each translation/rotation vector, the controller first calculates its existing
bounded proportional command, subtracts damping gain times filtered measured tool
velocity, reapplies the vector speed limit, then applies the existing acceleration
ramp. Native joint velocity, acceleration and jerk limiting remain unchanged.
This opposes measured movement as the tool approaches its target. More damping
can slow following; arbitrary delay/high tracking response can still destabilize
the complete loop. It is not a guarantee of vibration elimination.

The velocity estimate uses changes in measured base_link-to-tool0 TF, with actual
TF timestamps and quaternion-relative rotation. It does not differentiate VR target
steps. Repeated TF timestamps are ignored rather than injecting false zero velocity.
A time-aware exponential filter suppresses derivative noise. More filtering adds
feedback delay, so this control is separate from the existing VR-pose alpha filters.
Feedback history resets on clutch release, disabled output, fault or setting changes;
a new anchor starts without an old damping impulse. Out-of-order or invalid velocity
samples stop the clutch instead of reusing invalid damping data.

The movement mapping ratios, existing saved speed and tracking-response preferences
were preserved. The existing saved profile has response gain 10/s (default is 2/s),
linear speed 0.8 m/s and acceleration 0.5 m/s². The saved translation ratio is 1.5:1.
New damping defaults are supplied by the base configuration when loading an older
profile that has no damping keys. The GUI reports applied damping separately from
editable fields. Changing damping while disarmed and A released does not require a
ROS restart; reopening the old application once is needed to load this implementation.

## Verification and limits

168 Python/isolated ROS checks passed, including new velocity-estimation,
quaternion-sign, repeated/older timestamp, release/fault reset, vector-bound,
zero-damping, parameter-validation, persistence and delayed-feedback tests.
The GUI rendering test verifies the actual applied damping values and ms conversion.
Rebuilt and installed ur_vr_teleop; native controller code was not changed in this update.

A numerical mock used 50 ms transport delay plus 120 ms first-order velocity lag,
with the saved response/speed/acceleration settings and a 10 cm target. Without damping,
the model retained large oscillations. With damping 1.0, it settled within 2 mm of the
target and the final two-second position span was below one tenth of the undamped
span. This is a stress-model regression, not a calibrated UR10e model or evidence
that physical robot sway has been eliminated. Damping 0.5 reduced the model's sway
but did not fully settle that case. This motivated the default of 1.0.

No real robot motion, restart, enabling or fault reset was performed by the agent.
The saved high tracking gain may still need to be reduced toward the 2/s default if
physical sway persists; adding damping alone cannot certify stability at every tuning.

Background: [Modern Robotics: velocity-input motion control](https://modernrobotics.northwestern.edu/nu-gm-book-resource/11-3-motion-control-with-velocity-inputs-part-1-of-3/)
describes proportional error-to-velocity control. [PD control with force/torque inputs](https://modernrobotics.northwestern.edu/nu-gm-book-resource/11-4-motion-control-with-torque-or-force-inputs-part-1-of-3/)
explains the damping role of derivative feedback; those force-control gain formulas
are not directly applied to this velocity-command interface.
