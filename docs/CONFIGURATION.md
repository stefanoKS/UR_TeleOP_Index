# Configuration reference

This reference covers application parameters, managed launch settings, and project-owned Servo/controller overrides. Standard ROS/UR/MoveIt packages expose additional parameters; inspect their installed configuration rather than assuming this is an exhaustive reference for those external packages. Units below describe stored values, not necessarily GUI display units.

## Configuration precedence

1. `preferences.DEFAULTS` supplies missing supported tuning keys.
2. `ur_teleop/config/real_teleop.yaml` supplies robot, camera, and teleop settings.
3. Valid version-1 `operator_settings.json` **beside the selected YAML** overrides supported teleop tuning. It does not override robot/camera sections or grant motion permissions.
4. The desktop writes a session `effective_config.yaml` with `effective_sources`. Loading this snapshot does not reread the adjacent preference file. Driver generation and commissioning use the same snapshot.
5. Accepted atomic GUI updates can change runtime teleop parameters. Startup-only native limits still require restarting the stack. Saving follows backend acceptance; a newly saved joint-speed request is retained for restart while the running value remains unchanged.

`alignment_verified` and `optical_loss_test_verified` reset to false at startup, even when a file says true. These are deprecated observational compatibility fields; neither is a readiness requirement. `motion_output_enabled` comes from launch mode. None of these is persisted operator permission. The GUI refreshes from effective status when the session ID changes.

Inspect the panel, `/vr_real/operator_status`, ROS parameters, and session `effective_settings.log`. A snapshot records startup, not every subsequent runtime update. `preferences.py` is authoritative for saved-key validation; `commissioning.py` validates runtime updates.

## Teleop tuning

All rows in this table are supported saved-preference keys. **D** means apply only while disarmed with A released; the change must pass atomic backend validation. **R** means stack/driver restart is required to change the effective native limit. Numeric values must be finite. YAML defaults below agree with `preferences.DEFAULTS` except that the legacy scale is omitted from YAML.

ROS runtime tuning uses floating-point parameters (and a double array for RPY); a valid numeric JSON value is not permission to change a ROS parameter's declared type. The GUI performs these conversions.

| Parameter | Meaning / unit | YAML or default | Committed saved override | Allowed values | Override / apply |
|---|---|---|---|---|---|
| `teleop_control_mode` | Control law / enum | `velocity` | None | `position`, `velocity` | Saved/runtime; D |
| `tracking_station_policy` | Station acceptance / enum | `any_station` | None | `any_station`; legacy `two_station_strict`, `single_station_experimental` normalize to it | Saved/runtime compatibility; D; hidden in GUI |
| `single_station_motion_scale` | Legacy multiplier / ratio | 1.0 (code) | None | Input 0.01–1; effective scale always 1 | Saved/runtime compatibility; ignored, hidden |
| `translation_scale` | Robot displacement / hand displacement | 0.8333333333333334 | 1.0 | 0.01–10 | Saved/runtime; D; GUI shows inverse ratio |
| `rotation_scale` | Robot rotation / hand rotation | 1.0 | 1.0 | 0.01–10 | Saved/runtime; D; GUI shows inverse ratio |
| `max_linear_speed_m_s` | Cartesian linear norm cap / m/s | 0.10 | **1.5** | >0 | Saved/runtime; D |
| `max_angular_speed_deg_s` | Cartesian angular norm cap / deg/s | 30 | **180** | >0 | Saved/runtime; D |
| `max_joint_speed_rad_s` | Joint envelope/native velocity limit / rad/s | 0.50 | **2.0** | >0 | Saved; D + R for changed native limit |
| `tracking_gain_s_inv` | Position-error gain / 1/s | 2.0 | 2.0 | >0 | Saved/runtime; D; position mode demand only |
| `max_linear_acceleration_m_s2` | Cartesian linear slew limit / m/s² | 0.25 | 0.15 | >0 | Saved/runtime; D |
| `max_angular_acceleration_rad_s2` | Cartesian angular slew limit / rad/s² | 1.5707963267948966 | 0.3 | >0 | Saved/runtime; D |
| `vr_translation_filter_alpha` | New-pose position EMA weight / ratio | 0.8 | 1.0 | >0 and ≤1; 1 disables | Saved/runtime; D |
| `vr_rotation_filter_alpha` | New-pose orientation filter weight / ratio | 0.8 | 1.0 | >0 and ≤1; 1 disables | Saved/runtime; D |
| `vr_translation_deadband_m` | Spatial hand deadband / m | 0.001 | 0.02 | 0–0.02 | Saved/runtime; D; GUI mm |
| `vr_rotation_deadband_deg` | Spatial angular deadband / deg | 0.3 | 10.0 | 0–10 | Saved/runtime; D |
| `translation_damping_gain` | Measured-tool linear velocity feedback / ratio | 0 | 0.5 | 0–1 | Saved/runtime; D; position mode only |
| `rotation_damping_gain` | Measured-tool angular velocity feedback / ratio | 0 | 0.5 | 0–1 | Saved/runtime; D; position mode only |
| `damping_velocity_filter_s` | Measured-velocity filter time / s | 0.05 | 0.2 | 0.005–0.2 | Saved/runtime; D; GUI ms |
| `linear_velocity_filter_s` | Hand linear velocity EMA time / s | 0.04 | None | 0–0.2; 0 disables | Saved/runtime; D; velocity mode |
| `angular_velocity_filter_s` | Hand angular velocity EMA time / s | 0.04 | None | 0–0.2; 0 disables | Saved/runtime; D; velocity mode |
| `joint_acceleration_warning_rad_s2` | Upstream diagnostic warning / rad/s² | 5.0 | 5.0 | >0 | Saved/runtime; D; not a command limiter |
| `tracking_to_robot_rpy_deg` | Tracking-to-base roll/pitch/yaw / deg | [0, 0, −90] | **[0, 0, −40]** | Three finite numbers | Saved/runtime; D; mapping change requires physical review and a fresh configuration check |

“None” means no saved key, so YAML/default supplies it. Existing high saved values are not commissioning recommendations. Do not silently replace them in documentation or assume YAML is active.

### Other commissioning and preview parameters

These are not persisted tuning keys. Directly instantiating a node can differ from the managed launch.

| Parameter | Meaning / unit | Source/default | Allowed values | Override / apply |
|---|---|---|---|---|
| `base_frame` | Measured/reference base / frame | `base_link` | Valid matching TF frame | Startup ROS/YAML; restart; Servo frames must agree |
| `tool_frame` | Measured tool / frame | `tool0`; `teleop_tcp` with configured TCP | Valid matching TF frame | Startup ROS/YAML; restart; verify actual tool |
| `axis_map` | Initial 3×3 tracking rotation / row-major matrix | Identity in Preview constructor | Proper finite orthonormal rotation | Startup ROS; restart; commissioning configures RPY mapping |
| `observation_timeout_s` | Pose/joy/joint/TF age bound / s | YAML/Commissioning 0.35; standalone Preview 0.20 | 0.10–0.50 | Startup ROS/YAML; restart |
| `workspace_radius_m` | Radius about initial robot reference / m | 0 | 0 disabled or 0.05–0.50 | Commissioning startup ROS/YAML; restart |
| `alignment_verified` | Legacy observed mapping / bool; no readiness effect | false, reset on load | true/false, subject to checks | GUI atomic update; D; session only |
| `optical_loss_test_verified` | Legacy loss observation / bool; no readiness effect | false, reset on load | true/false | GUI atomic update; D; session only |
| `motion_output_enabled` | Create final command publisher / bool | false | true/false | Launch/desktop; restart; not permission to move |
| `use_sim_time` | ROS clock source / bool | false | Production Preview rejects true | Keep false; startup |

Standalone `preview` uses `translation_scale=0.8`, read-only, and the original observation-only clutch. It does not perform the commissioning optical gate and cannot command motion. Managed Preview uses `commissioning` with output disabled. Direct `Commissioning` declares `tracking_to_robot_rpy_deg=[0,0,0]`; the normal config loader supplies −90° or the saved −40° instead. These entry points are not interchangeable.

## Robot and camera YAML

Only the managed desktop consumes driver-tuning keys. Editing them while ROS is running does not reconfigure the hardware plugin. Robot `ip` and `calibration_file` are also used by commissioning launch to build its model.

| YAML key | Meaning / unit | Committed value (fallback if different) | Allowed values | Override / apply |
|---|---|---|---|---|
| `robot.ip` | Robot network address | `192.168.1.251` | Reachable correct robot address | Config; restart; no saved override |
| `robot.calibration_file` | Calibrated kinematics / path | `/home/ubuntu-ros/Documents/Teleop/ur_teleop/config/ur10e_calibration.yaml` | Readable YAML with expected calibration hash | Config; restart and mapping review |
| `robot.tcp_offset` | Tool tip XYZ metres + rotation vector radians | Absent (legacy `tool0`) | Six finite numbers; translation norm ≤2 m, rotation norm ≤2π | Adjacent `tool_tcp.json` override; restart; pendant agreement required |
| `robot.servoj_gain` | UR servoj gain / dimensionless | 500 | Integer 100–2000 | Managed generated description; restart |
| `robot.servoj_lookahead_time` | UR servoj lookahead / s | 0.03 (desktop fallback 0.08) | 0.03–0.2 | Managed generated description; restart |
| `robot.joint_command_acceleration_rad_s2` | Native joint acceleration / rad/s² | 1.0 | Finite >0 | Generated controller YAML; restart |
| `robot.joint_command_jerk_rad_s3` | Native joint jerk / rad/s³ | 10.0 | Finite >0 | Generated controller YAML; restart |
| `camera.enabled` | Start camera child / bool | true | true/false | Managed config; restart |
| `camera.serial` | D405 device selection / string | `230322276078` | Connected device serial | Managed config; restart camera/stack |

Camera node `/d405_camera` declares `serial` with the same default. Its resolution/rate (640×480/15 fps) are capture implementation constants, not ROS parameters. Camera failure does not relax or add robot readiness conditions.

## Launch and bridge controls

| Control | Meaning / unit | Default | Allowed / overrides | Restart |
|---|---|---|---|---|
| Desktop `--config` | Runtime YAML path | `~/Documents/Teleop/ur_teleop/config/real_teleop.yaml` | Existing config; adjacent preferences overlay | New desktop/session |
| Desktop `--live` / `--preview` | Select initial output mode | Live | Flags; still starts disabled/inactive | New stack |
| Launch `config_file` | Commissioning YAML/snapshot | Same home-relative YAML path | Passed by desktop as effective snapshot | Yes |
| Launch `motion_output_enabled` | Commissioning output mode | `false` | Literal `true` or `false` | Yes |
| Launch `launch_rviz` | Start RViz child | `true`; desktop sets false and owns RViz separately | Boolean | Relaunch / managed Open RViz |
| Bridge `device_name` | libsurvive object name | `KN0` | Existing object string | Restart bridge |
| Bridge `device_serial` | Serial selection | Empty; desktop requests `LHR-4632363E` | Empty or matching serial | Restart bridge |
| Bridge `frame_id` | Published pose parent frame | `lighthouse` | String; teleop currently requires `lighthouse` | Coordinated restart/code review |
| Bridge `child_frame_id` | Published controller TF frame | `vr_controller` | Frame string | Restart bridge |
| Bridge `pose_timeout` | Bridge pose freshness / s | 0.35 | Numeric; no explicit bounded validator in bridge | Startup; keep consistent with downstream guards |
| Bridge `libsurvive_verbosity` | libsurvive logging level | 10 | Integer passed to library | Restart bridge |
| `ROS_DOMAIN_ID` | DDS domain | Scripts set 43 | Production launch requires 43; simulator requires 42 | All participating processes |
| `ROS_LOCALHOST_ONLY` | Limit discovery to host | Not forced by desktop | 1 for isolated mock tests; ROS environment semantics | All processes in test |

`servo_stamp_relay` declares no custom tuning parameters. Changing topic remappings must preserve timestamps and the gate's publisher ownership checks.

### Managed UR driver launch arguments

These arguments are assembled by `desktop.py`, not read from saved GUI preferences directly. All require a new driver process. Their allowed values are constrained by the installed UR driver and this application's fixed robot/control contract.

| Argument | Meaning / unit | Managed value / source | Override behavior |
|---|---|---|---|
| `ur_type` | Robot model | `ur10e` | Fixed in launcher; coordinated model/calibration change required |
| `robot_ip` | Network address | `robot.ip` | YAML, restart |
| `kinematics_params_file` | Calibration path | `robot.calibration_file` | YAML, matching hash/model required |
| `controllers_file` | Controller YAML path | Session-generated `controllers.yaml` | Generated from project template and effective limits |
| `description_file` | Driver xacro path | Session-generated description copy | Generated with validated servoj values |
| `script_sender_port` | External Control script port / TCP port | 50002 | Fixed in launcher; match network/robot setup |
| `headless_mode` | Automatic script handling / bool | false | Pendant External Control workflow |
| `initial_joint_controller` | Controller selected at startup | `scaled_joint_trajectory_controller` | Must preserve gate/plugin ownership contract |
| `activate_joint_controller` | Activate on startup / bool | false | START automatically prepares after prerequisites; starts inactive |
| `launch_rviz` | Driver RViz / bool | false | Desktop owns RViz separately |

## Native controller configuration

Source: `ur_teleop/config/commissioning_controllers.yaml`, specialized by `desktop.py`. All of these are startup/controller configuration; do not treat runtime parameter mutation as proof that the configured controller changed its internal watchdog. Standard UR controllers listed in the YAML are available definitions, not all active motion controllers.

| Parameter | Meaning / unit | Project value | Allowed / effective behavior | Apply |
|---|---|---|---|---|
| Controller `type` | Plugin class | `ur_teleop_controllers/SmoothScaledJointTrajectoryController` | Must match installed plugin | Restart |
| `smoothing.max_velocity` | Native joint speed / rad/s | 0.5 template/code; generated from effective `max_joint_speed_rad_s` (saved 2) | Finite >0 | Restart |
| `smoothing.max_acceleration` | Native acceleration / rad/s² | 1.0 | Finite >0; generated from robot YAML | Restart |
| `smoothing.max_jerk` | Native jerk / rad/s³ | 10.0 | Finite >0; generated from robot YAML | Restart |
| `smoothing.max_following_error` | Measured/reference deviation / rad | 0.10 | >0 and ≤0.20 | Restart |
| `cmd_timeout` | Joint command watchdog / s | 0.2 | Gate accepts 0.10–0.25 | Restart |
| `joints` | Controlled joint order | Six unprefixed UR joints | All six named joints required | Restart |
| `command_interfaces` | Hardware output | [position] | Native implementation requires position | Restart |
| `state_interfaces` | Hardware feedback | [position, velocity] | Both required | Restart |
| `state_publish_rate` | Standard controller state / Hz | 100 | Base controller validation | Restart |
| `action_monitor_rate` | Action monitor / Hz | 20 | Base controller validation | Restart |
| `allow_partial_joints_goal` | Partial action goals | false | Keep false for six-joint contract | Restart |
| `constraints.stopped_velocity_tolerance` | Standard goal stopped tolerance / rad/s | 0.2 | Base-controller constraint; distinct from native activation standstill | Restart |
| `constraints.goal_time` | Goal-time tolerance / s | 0 | Must be below command timeout for gate | Restart |
| `constraints.<joint>.trajectory` | Per-joint path tolerance / rad | 0.2 for each joint | Base controller validation | Restart |
| `constraints.<joint>.goal` | Per-joint goal tolerance / rad | 0.1 for each joint | Base controller validation | Restart |
| `speed_scaling_interface_name` | UR scaling state | `speed_scaling/speed_scaling_factor` with configured prefix | Must match hardware state interface | Restart |
| `update_rate` | Controller update / Hz | Substituted UR driver rate, nominal 500 | Plugin rejects invalid/excessive periods; not GUI tuning | Restart |
| Broadcaster `state_publish_rate` | Speed-scaling feedback / Hz | 100 | Driver broadcaster setting | Restart |
| `tf_prefix` | UR controller/joint frame prefix | Empty managed real path | Must match entire robot model | Restart |

Six joint names: `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`. Other standard controllers in this template retain their UR interface/sensor/topic settings; they must not become competing active position claimants. A duplicate `passthrough_trajectory_controller` declaration exists in the current template; this documentation task does not edit it.

The remaining project-template fields configure standard UR components, not teleop tuning. They are startup-only and must match the hardware interfaces/model; saved preferences do not override them:

| Component / parameter | Meaning / unit | Template value | Allowed / change rule |
|---|---|---|---|
| Force/torque `sensor_name` | Hardware sensor name | Prefixed `tcp_fts_sensor` | Must exist in hardware state interfaces |
| Force/torque `state_interface_names` | Wrench axes / N and N·m | `force.x/y/z`, `torque.x/y/z` | Match sensor interfaces |
| Force/torque `frame_id`, `topic_name` | Wrench frame/topic | Prefixed `tool0_controller`, `ft_data` | Match physical sensor/model |
| Forward velocity/effort `interface_name` | Command interface | `velocity` / `effort` respectively | Standard controller contract; not an alternate teleop route |
| TCP broadcaster `frame_id`, `pose_name` | Pose reference/interface | Prefixed `base`, `tcp_pose` | Match UR hardware |
| TCP broadcaster `tf.child_frame_id` | Tool TF frame | Prefixed `tool0_controller` | Distinct from teleop model's `tool0` |
| Standard controllers `type`, `joints`, `state_interfaces` | Plugin and joint/interface membership | Classes and six-joint lists in template | Must match installed UR plugins; retain inactive competing motion definitions |

## MoveIt Servo overrides

`commissioning.launch.py` loads installed `ur_moveit_config/config/ur_servo.yaml`, then applies the following `moveit_servo.*` values. They are source-level launch overrides, not persisted GUI settings. Treat changes as restart-and-revalidate work. External defaults not overridden here depend on the installed UR/MoveIt version.

| Parameter(s) | Meaning / unit | Effective project override | Allowed / change rule |
|---|---|---|---|
| `command_in_type` | Input units | `speed_units` | Must agree with Cartesian m/s and rad/s output |
| `publish_period` | Servo output period / s | 0.01 | Positive, coordinated with control period |
| `smoothing_filter_plugin_name` | Servo position processing | `ur_teleop_controllers::ServoVelocityPassthrough` | Restart; requires the supervised native Ruckig chain |
| `command_out_type` | Output message | `trajectory_msgs/JointTrajectory` | Gate requires this type |
| `publish_joint_positions`, `publish_joint_velocities`, `publish_joint_accelerations` | Fields emitted | true, true, false | Preserve native velocity-demand contract |
| `use_gazebo`, `is_primary_planning_scene_monitor` | Environment / scene ownership | false, true | Match launch topology |
| `move_group_name` | Semantic group | `ur10e_manipulator` | Must exist in SRDF |
| `planning_frame`, `robot_link_command_frame` | Cartesian frame | `base_link` | Must match mapping/TF |
| `ee_frame_name` | Tool frame | `tool0`; `teleop_tcp` with configured TCP | Must match robot model |
| `joint_topic` | Measured feedback | `/joint_states` | Matching six-joint feedback |
| `cartesian_command_in_topic` | Teleop input | `/vr_real/servo_node/delta_twist_cmds` | Single authorized source |
| `joint_command_in_topic` | Standard Servo joint-jog input | `/vr_real/servo_node/delta_joint_cmds` | Unused by teleop; do not inject a competing demand |
| `command_out_topic` | Staged trajectory | `/vr_real/servo_unstamped_targets` | Must pass relay/gate |
| `status_topic` | Servo health | `/vr_real/servo_node/status` | Must match gate |
| `incoming_command_timeout` | Cartesian input timeout / s | 0.10 | Preserve watchdog |
| `num_outgoing_halt_msgs_to_publish` | Halt messages / count | 10 | Preserve stop behavior |
| `check_collisions`, `collision_check_rate` | Model collision checking / Hz | true, 20 | Do not silently disable |
| `lower_singularity_threshold`, `hard_stop_singularity_threshold` | Singularity condition thresholds | 17, 30 | Ordered, installed Servo validation |

The production Servo plugin preserves the joint increment instead of filtering absolute joint positions before Servo computes velocity. Final Ruckig smoothing and all native limits remain active. `butterworth_filter_coeff` no longer affects production motion; no operator preference is added or migrated. The old `system_latency_compensation` override was removed because installed Humble Servo 2.5.10 does not expose it as an effective parameter. This change requires a managed-stack restart and supervised sway comparison; rebuilding does not change an already-running Servo process.

Generated real model xacro uses `safety_limits=true`, `safety_pos_margin=0.15`, and `safety_k_position=20`. The desktop copies UR description xacros to the session directory to apply servoj tuning; it does not edit installed UR package files. The commissioning model comes from installed `ur_description` with the same calibration, independently of the desktop's driver description copy.

## Separate simulation settings

`ur_vr_sim_teleop/launch/simulation.launch.py` uses domain 42 and wall time. These settings do not override production tuning or read `operator_settings.json`.

| Parameter / launch argument | Meaning / unit | Default | Allowed / override | Apply |
|---|---|---|---|---|
| `translation_scale` | Hand-to-target ratio | 0.8 | Finite positive scale, core validation | Startup launch/ROS |
| `axis_yaw_deg` | Launch-generated XY mapping rotation / deg | 0 | Finite angle | Relaunch |
| `axis_map` | Node mapping / 9-element matrix | Identity, generated from yaw | Proper rotation | Startup |
| `clutch_button` | Joy index | 4 | Integer 0–15 | Startup |
| `enable_motion` | Simulated motion permission | false | Boolean; only mutable simulation parameter | Runtime, simulation only |
| `robot_description` | Robot XML | Empty node default; launch supplies URDF | Must contain `mock_components/GenericSystem` | Startup |
| `use_sim_time` | Clock | false | Wall time for this stack | Startup |

Simulation uses `sim_` frames/joints, controller manager 100 Hz, Servo period 0.01 s, `std_msgs/Float64MultiArray` position output to `/vr_sim/forward_position_controller/commands`, and original clutch limits of 0.04 m/s, 15 deg/s, and 0.20 m workspace radius. These limits are implementation constants, not current production defaults.

Its controller YAML sets `joint_state_broadcaster.use_local_topics=false` and lists six `sim_` joints for the forward position controller. Initial mock positions are in [initial_positions.yaml](../ur_teleop_ws/src/ur_vr_sim_teleop/config/initial_positions.yaml); change them only with a simulation restart and model check. Its Servo topics use `/vr_sim`, its semantic group is `sim_ur10e_manipulator`, and its base/tool are `sim_base_link`/`sim_tool0`. It emits positions only, keeps collision checking at 20 Hz and a 0.10 s incoming timeout, and inherits other defaults from installed `ur_servo.yaml`.

## Fixed guards are not tuning parameters

The optical age limit (0.25 s), quality continuity window, pose-jump/freeze guards, velocity sample deadlines, command horizon/age checks, GUI heartbeat deadline, and recovery/hold timing are code constants. See [Architecture](ARCHITECTURE.md). Changing them requires code review and tests; there is no supported GUI switch to bypass them.

### Dashboard readback and drafts

V3.1 uses the complete `effective_parameters` status object for runtime readback;
see [operator-status contract](OPERATOR_STATUS.md). Saved, drafted and submitted
values are separate from this baseline. The GUI preserves hidden compatibility
inputs, formats display units through the named registry and only sends changed
runtime keys. It never sends `max_joint_speed_rad_s` as a runtime modification.
A saved joint-speed change remains pending until both teleop and native controller
report that value after restart. Session verification is never persisted.

The GUI does not infer per-value startup provenance from a numeric match. Consult
`effective_settings.log` for actual YAML/default/saved source information; that
startup record is not proof of later runtime values. No defaults, ranges, units,
persisted keys, or committed operator overrides changed in this stabilization.

## V4 startup and engineering configuration

Desktop defaults to Live, disabled/inactive. `--preview` selects Preview; direct launch defaults to motion output disabled. `--engineering-session` / `engineering_session` remain accepted for compatibility and grant no alternate readiness path. The old `engineering_required` parameter is accepted but ignored.

`engineering_context` retains its wire name for compatibility and carries the immutable effective configuration plus tracking calibration path. The backend checks this using `configuration.Configuration`, not `engineering.Verification`. Explicit mapping must come from YAML or saved preferences; a built-in fallback alone is insufficient. RPY/tuning must be typed and finite, robot IP valid, calibration readable with the installation's expected hash, and frames must match Servo (`base_link`, and `tool0` or configured `teleop_tcp`). The libsurvive file must be readable/nonempty once at backend startup; an initial missing/empty file is retried without requiring restart. Its native format is not parsed as strict JSON. Once available, this mutable persistence file is not monitored for byte changes: libsurvive rewrites metadata and station estimates during normal operation, including truncating writes. Live calibrated optical/pose guards remain authoritative. Robot kinematics file changes within a session still block until review/restart. Deliberate tracking recalibration or physical station movement still requires physical mapping review; a disk checksum cannot establish the running solver state.

Checks run on the file worker every 0.5 s; results older than 2 s or for different current parameters cannot grant readiness. Configuration checks do not prove physical alignment or calibration ownership. Before operating a new/modified workcell, physically assess mapping, tool/TCP/payload, mounting and stopping behavior. Existing explicit saved configuration is reused without requiring evidence files or repeated GUI approvals.

### Wizard setup persistence (schema 5)

`verified_setup.json`, `engineering_record_path`, legacy source hashes and migration records remain optional historical evidence. They are never consulted by routine readiness. The optional observations tool can still validate/save its historical record format, but a missing/incompatible record does not block ordinary Start. No numeric defaults, saved operator overrides or motion algorithms changed.

## V5.1 internal capture quality thresholds

`calibration_capture.CaptureConfig` is an internal, injectable immutable configuration for observational capture/tests. It adds no YAML/ROS tuning key and changes no saved preference, motion limit or filter. Stable endpoints require 250 ms and at least 8 distinct samples, with maximum positional spread 3 mm and orientation spread 2°. Required positive-axis displacement remains 20 mm / 10°; perpendicular magnitude must be at most 0.6 times the requested-axis component.

| Capture guard | Default |
|---|---|
| Pose / Joy / validity flag freshness | 150 / 200 / 200 ms |
| Isolation readback lifetime | 500 ms (backend status additionally <400 ms) |
| Button debounce / release verification | 40 / 120 ms |
| Continuous recovery / advance delay | 100 / 600 ms |
| Maximum capture / stored samples per buffer | 20 s / 3000 |
| Adjacent-pose jump | 50 mm / 45° |
| Exactly frozen source pose | 500 ms |
| Quaternion norm error / future source tolerance | 0.2 / 30 ms |

These limits validate observation quality; they are not robot stopping bounds. Changing them does not tune the actuator. The engine normalizes accepted quaternions and uses quaternion-relative world rotation. Neither GUI display ratios nor stored RPY are modified. Existing saved linear speed 1.5 m/s and yaw −40° remain distinct from YAML 0.10 m/s and −90°.

## Tool TCP profile

`robot.tcp_offset` is an optional six-number list `[x,y,z,rx,ry,rz]`: translation in metres relative to model `tool0`, followed by a rotation vector in radians. Translation norm must be ≤2 m and rotation-vector norm ≤2π; all values must be finite numbers (not booleans). This is startup-only and requires restart. These bounds validate input, not physical tool suitability.

A version-1 `tool_tcp.json` beside the selected YAML overrides this key. Its fields are `version`, `robot_ip` (must match selected robot), and `offset`. The GUI writes it atomically. Missing profiles preserve legacy behavior; malformed profiles block loading. This is separate from `operator_settings.json`, whose version and tuning keys remain unchanged. Effective snapshots embed the resolved offset and never reread the mutable profile. The original UR kinematic calibration remains unchanged.

The managed driver and Servo add the same fixed `tool0 → teleop_tcp` joint. Servo's semantic manipulator chain ends at this link, and teleop measures it. Cartesian limits/workspace checks therefore apply at the selected tip; native joint velocity/acceleration/jerk ownership is unchanged. Tool shape/payload are not inferred from the point.

Configured TCP consistency uses output-only RTDE protocol 2 on robot port 30004 (`tcp_offset`, VECTOR6D). Reads run on the configuration worker using a new bounded connection (0.6 s total deadline) per check, avoiding old queued samples. Checks are scheduled at the existing 0.5 s configuration interval; results expire after 2 s. Difference tolerances are 10 µm translation and 10 µrad orientation. Mismatch/read failure blocks configuration readiness; this is not instantaneous detection. No input recipes, TCP-setting scripts, or motion commands are sent. Without a configured offset, automatic reads are disabled; the editor can still request a read while disarmed.

## Gripper settings

`gripper.process_enabled` in the selected YAML defaults true and controls only optional child startup (restart required). Adjacent `gripper.json` is a separate version-1 calibration, initially absent/disabled, with explicit endpoint/travel validation and atomic saving. It does not change arm operator preferences. See the [gripper parameter table](GRIPPER.md#settings-json-version-1).
