# Sway recording and offline comparison

The original startup simplification changed no motion algorithm. The subsequent October 15 sway correction changes Servo position processing as described below; no physical improvement is claimed from software tests alone.

## Record

1. Open Engineering / diagnostics → Advanced diagnostics → **RECORD SWAY DIAGNOSTICS**. Wait for the recording path.
2. Perform only a separately authorized supervised diagnostic session. Recording cannot enable a controller, request motion or bypass readiness. It can begin while disarmed.
3. Press **STOP RECORDING** and wait for completion. Use STOP TELEOP separately to disarm. Closing the application requests recorder shutdown and bag finalization.

The CLI uses the same duplicate lock and output directory:

```bash
./ur_teleop/scripts/record_sway.sh
# Ctrl+C finalizes the bag.
```

Each timestamped directory under `ur_teleop/diagnostics/` contains `bag/`, `recording.json` and recorder QoS. GUI feedback links the recorder log. Camera availability is irrelevant. The lock is held through finalization; an interrupted recorder releases its OS lock automatically. Do not delete unknown process/session locks.

Recorded topics include raw hand pose/source time, Joy/A, validity/optical status, raw/filtered hand velocity and measured TCP estimates in operator_status, actual published Twist, Servo targets before/after timestamp relay, gated trajectories, smoothed position/velocity, native control diagnostics, measured joints, TF, Servo state, speed scaling, robot mode/safety/program state and parameter events. Parameter reads at start/stop capture the teleop, Servo, native controller and controller-manager configuration. Missing reads are recorded as errors, not fabricated values. Operator status captures effective tuning changes throughout the session.

The selected/managed snapshot is also saved. It records driver-generation settings, including servoj tuning, but is not independent readback from the robot. Committed saved overrides are 0.5 m/s linear speed, 60 deg/s angular speed, 1.5 rad/s joint speed, −45° yaw and Position mode, versus YAML 0.10 m/s, 30 deg/s, 0.5 rad/s, −90° and Velocity. Configured servoj gain/lookahead are 500/0.03 s. Review runtime dumps/status before attributing those values to an actual session; the effective runtime values are authoritative. No limits are increased by these tools.

## Analyze an existing recording

```bash
source /opt/ros/humble/setup.bash
source ur_teleop_ws/install/local_setup.bash
/usr/bin/python3 ur_teleop/scripts/analyze_sway.py ur_teleop/diagnostics/sway_TIMESTAMP
```

This reads an existing bag without creating a ROS node or connecting to the robot. `--output` chooses a report directory. JSON and Markdown reports accompany six separate joint plots (including individual wrist 1/2/3 plots in `joint_4.png`, `joint_5.png`, `joint_6.png`), Cartesian/hand-velocity plots, raw hand pose and speed scaling when available.

The report includes stage sign reversals, dominant frequency where supported, receive-time correlation lag, position error, command-stop to measured standstill, directional overshoot relative to a stationary final reference, settling time, receive-time acceleration estimates, effective native speed-cap overlays, frequency/gaps, speed-scaling variation, effective parameters and clock audits. Analysis defaults are 0.001 rad/s standstill, 0.002 rad position tolerance and 0.5 s observed dwell. These are analysis thresholds, not actuator limits. Change thresholds explicitly if appropriate; missing/insufficient data produces null metrics.

All cross-stream comparisons use bag receive time. Actual header/source timestamps are retained in the bag and audited separately. Device time is never subtracted directly from ROS epoch. Receive-time lag includes transport and sampling; Servo targets also have a future trajectory horizon. Correlation on periodic data is ambiguous and is not absolute actuator latency. No interpolation crosses gaps larger than 100 ms or extrapolates beyond a stream. Frequency estimates need at least two cycles and concentrated spectral energy; they are candidates for inspection, not a root-cause verdict. Stop metrics require an observed stop transition and sufficient continuous feedback; a truncated tail cannot prove standstill or settling.

## Interpret the stages

| Observation | Investigate |
|---|---|
| Stationary hand but oscillating Cartesian demand | Tracking noise, sample timing, velocity estimation |
| Stable Cartesian demand but reversing Servo velocity | Servo, feedback timing, IK and singularity scaling |
| Stable Servo request but oscillating native reference | Trajectory replacement/interpolation, Ruckig state handling |
| Stationary final reference but oscillating feedback | UR servo response/timing, speed scaling, TCP/payload and mechanical behavior |
| Same-sign decaying reference and feedback | Continued commanded deceleration rather than repeated corrections |

Inspect the actual time plots and configuration together. Whole-record reversal counts also include deliberate operator reversals. An FFT peak alone does not establish unwanted oscillation. Physical payload/TCP, stopping distance and wrist behavior require supervised validation.

## October 15 Servo feedback correction

The 12:42 recording showed a roughly 2.6 Hz oscillation in Servo output, final reference and measured joints. In a representative two-second interval, the reconstructed shoulder-pan request stayed positive while Servo repeatedly requested reverse motion. Humble Servo's Butterworth position history fed measured motion back into its position-difference velocity calculation.

Production now selects `ur_teleop_controllers::ServoVelocityPassthrough`: it preserves Servo's joint increment and removes that position-history term. Native Ruckig remains responsible for final joint smoothing and derivative limits. This does not change hand filtering, mapping, saved limits, servoj tuning, collision/singularity handling, timestamps or gate checks. The standalone simulator is unchanged. Restart the managed stack after rebuilding both affected packages; an existing process keeps its loaded plugin.

The isolated real-Servo regression injects a 0.004 rad, 2.6 Hz feedback oscillation while requesting 0.001 m/s steady translation. The previous configuration failed at a peak joint request of 0.05327 rad/s; the corrected configuration produced 0.002139 rad/s. These are synthetic test command measurements, not a measured percentage reduction in physical robot sway. Zero-command output, stale-input publication suppression and recovery are checked in the same test. Native C++ tests check final derivative limits and reset behavior.

**Physical outcome:** the operator reports that the earlier wrist/arm oscillation disappeared following this Servo filtering correction. A separately documented post-fix physical measurement was not available in this audit. This is an operator-reported outcome, not a synthetic-test result or a certified hardware performance claim.

A new supervised recording is still required to assess physical sway, stop response and any remaining downstream servoj amplification. Keep the previous recording as the baseline and compare similar low-speed movements with the same payload, pose and tuning.

### Implementation validation (2026-10-15)

Executed with `/usr/bin/python3`, ROS Humble and the rebuilt local overlay:

| Command / check | Result |
|---|---|
| `colcon build --base-paths src --packages-select ur_teleop_controllers ur_vr_teleop --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3` (inside `ur_teleop_ws`) | Both packages built |
| `/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_vr_teleop/test ur_teleop_ws/src/ur_vr_sim_teleop/test -q` | 370 passed; one existing Python dependency warning |
| `/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_vr_teleop/test/ros_integration_checks.py -q` | 88 passed |
| `ctest --test-dir ur_teleop_ws/build/ur_teleop_controllers --output-on-failure` | 3 suites / 13 tests passed, including the real native plugin with local hardware-interface doubles |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/servo_pipeline_check.py` | Passed with the production launch and installed Servo/RSP/relay; previous configuration failed the new feedback regression |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/calibration_ros_check.py` | Passed |
| `git diff --check` | Passed |

ROS tests used isolated localhost domains; no physical driver was started. Driver-generation mock startup, GUI rendering and bridge guard suites were not rerun because those implementations were unchanged. Physical sway, payload/TCP response, and stopping distance remain unvalidated. No runtime robot settings were changed and the managed physical stack was not restarted by these tests.
