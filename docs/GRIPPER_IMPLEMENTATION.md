# ST3215 implementation and validation — 2026-10-19

Implemented one independent ST3215 rack-and-pinion subsystem. The gripper is disabled without a reviewed local calibration. Hardware acceptance remains outstanding; this report describes software behavior and automated evidence.

## Changed files

- `ur_teleop_ws/src/ur_bus_servo_gripper/`: new ament Python package, manifest/setup/resource, `gripper_config.py` (validated atomic persistence), `gripper_mapping.py` (normalization/pickup/rate limits), `st3215_driver.py` (exclusive SDK transport and feedback), `gripper_worker.py` (serial ownership/authorization/faults), `gripper_calibration.py` (disarmed/jog checks), `gripper_node.py` (Humble metadata executor and ROS interfaces), unit/pseudo-terminal tests and explicit ROS lifecycle check.
- `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/gripper_panel.py`: asynchronous Tk setup/calibration panel.
- `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/{gui,desktop}.py`: Settings entry, compact status card, STOP consent withdrawal, optional managed child with existing ownership lifecycle.
- `ur_teleop_ws/src/ur_vr_teleop/package.xml`: package dependency.
- `ur_teleop_ws/src/ur_vr_teleop/test/{test_gripper_gui,test_desktop,gui_render_check}.py`: UI input robustness, STOP, optional-child ownership/failure and real Tk regression coverage.
- `ur_teleop/scripts/install_gripper_sdk.py`: checksum-pinned vendor SDK installer.
- `ur_teleop/config/real_teleop.yaml`: optional `gripper.process_enabled` process-start setting; no arm setting changed.
- `.gitignore`: excludes mechanism-specific `gripper.json` from shipped defaults.
- `README.md`, `docs/{ARCHITECTURE,CONFIGURATION,INSTALLATION,OPERATION,COMMISSIONING,GRIPPER,GRIPPER_IMPLEMENTATION}.md`: integration, settings, protocol/state contract, installation and validation procedures.

The pre-existing modifications to `ur_teleop/config/operator_settings.json` were preserved. No calibration file or enable intent was created. No changes were made to bridge tracking, commissioning/gate, ServoVelocityPassthrough, Ruckig, servoj tuning, alignment, TCP, A-button behavior or D405 source.

## Feature summary

1. **SDK:** compatible Waveshare `scservo_sdk.sms_sts`; installed signatures and actual packet behavior inspected. See [installation and pinned checksum](GRIPPER.md#install-and-identify). The SDK and Ubuntu pyserial 3.5 were installed into system Python 3.10's user site for this session; system apt installation was unavailable without administrator authentication. No vendor motion example was executed.
2. **Detection:** by-id preference, manual `/dev/...` selector, read-only ID discovery 0–253 or configured-ID ping, mode/resolution/angle-limit checks. No ID, EEPROM or mode rewrites.
3. **Calibration GUI:** actual feedback capture, both endpoint directions, reviewed bounds, speed/acceleration/rate limits, small deliberate jogs, explicit Open/Close tests, disarmed trigger test, save/restore. Serial I/O and file operations do not run on Tk.
4. **Persistence:** version-1 adjacent `gripper.json`, atomic replacement, previous file retained if saving fails; disabled incomplete drafts allowed, motion requires valid endpoints/ID/travel. Saved calibration is reusable.
5. **Trigger:** calibrated min/max/inversion, endpoint deadzones, change threshold, post-Joy pose/optical corroboration, timestamp/order checks and deliberate pickup. A press/release leaves gripper follow engaged.
6. **Commands:** separate at-most-25 Hz worker, latest-only input and bounded action mailbox, configurable software ticks/s limit (1–2000), configurable target lead (default 20, maximum 60 ticks), lower calibration limits. Unexpected encoder displacement faults instead of creating a large setpoint jump; a motor that does not progress is reported and faults rather than accumulating outstanding travel. SDK physical speed/acceleration behavior is unverified.
7. **Faults:** stale input holds without automatic opening; failed feedback/USB link closes and never auto-replays; motor errors/temperature and lack of progress fault. STOP retires follow and calibration consent without disabling holding torque. Unacknowledged attempts are explicitly distinguishable from acknowledged commands.
8. **ROS:** `/gripper/target_fraction`, `/gripper/target_ticks`, `/gripper/joint_states`, `/gripper/status`; timestamps, units, validity and service semantics are documented in the [observation contract](GRIPPER.md#ros-observation-contract). No LeRobot exporter.
9. **Dashboard:** optional status card and Settings → Gripper Setup. Faults, missing process/feedback and pickup instructions remain distinct from arm/camera status.

## Executed validation

Commands below ran with `/usr/bin/python3` and the rebuilt Humble overlay. ROS Python integration scripts used localhost domain 147; the GenericSystem startup check isolates itself on 148. No production driver/managed desktop was launched as a test, and no automated test sent physical serial traffic.

| Command | Result |
|---|---|
| `colcon build --base-paths src --packages-select ur_bus_servo_gripper ur_vr_teleop --executor sequential` (inside `ur_teleop_ws`, system PATH, Humble sourced) | Both packages built |
| `/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_bus_servo_gripper/test ur_teleop_ws/src/ur_vr_teleop/test ur_teleop_ws/src/ur_vr_sim_teleop/test -q` | 441 passed; one existing upstream xacro compatibility warning |
| `/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_vr_teleop/test/ros_integration_checks.py -q` | 91 passed |
| `/usr/bin/python3 ur_teleop_ws/src/ur_bus_servo_gripper/test/ros_gripper_check.py` | Passed: startup without writes, pickup, bounded follow, synthetic-zero loss, recovery, stale Joy, STOP and no arm publisher |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/gui_render_check.py` | Passed: gripper feedback capture, temporary save, stale feedback, unanswered-request responsiveness; existing camera panel and dashboard layouts |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/mock_startup_check.py` | Passed: generated parameters, GenericSystem, joints and tool0 TF |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/calibration_ros_check.py` | Passed: Preview capture, managed command generation, isolation/loss/cleanup |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/servo_pipeline_check.py` | Passed: 201 targets, 10 ms horizon, zero/stale-input behavior, oscillating-feedback peak 0.002139 rad/s |
| `ctest --test-dir ur_teleop_ws/build/ur_vr_bridge --output-on-failure` | 2/2 passed |
| `ctest --test-dir ur_teleop_ws/build/ur_teleop_controllers --output-on-failure` (overlay sourced) | 3/3 suites passed |
| Final targeted gripper suite after reviewed-travel fault refinement | 45 passed |
| Final GUI/desktop unit checks after status-card refinement | 22 passed; real Tk/ROS render check also passed again |
| Python syntax compilation, `git diff --check`, new local documentation links | Passed |

During development, checks found and resolved a GUI fixture compatibility issue and Humble's discarded DDS metadata. A transport assertion initially used an old installed package and passed after rebuilding. Native plugin-loading CTests initially lacked the workspace overlay and passed after sourcing it. These were not physical validation failures. Unchanged native packages were tested from their existing builds, not rebuilt. The separate TCP-offset variants and live D405 capture were not repeated; D405 code and command construction are unchanged, and the existing camera-panel regression passed.

## Physical acceptance outstanding

The adapter `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7B058089-if00` was visible. A separately scoped **read-only** discovery attempt failed before opening it with Linux `Permission denied`. No servo packet/motion was issued by that attempt; the actual ID, firmware mode, power supply, trigger range and mechanism endpoints remain unverified.

Follow the [supervised sequence](GRIPPER.md#supervised-physical-validation-still-required) after administrator-approved serial access. It covers a tiny slow jog, opening direction, safe endpoints, intermediate travel, trigger pickup/range, physical speed/force/current behavior, tracking/USB/STOP recovery, holding under load, and combined UR/gripper operation. No full-range sweep, hard-stop search, EEPROM change or physical grasp test was performed automatically.
