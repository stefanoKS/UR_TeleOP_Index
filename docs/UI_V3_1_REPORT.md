# UI V3.1 stabilization report

> **Historical values.** Numeric saved overrides quoted below were read at the time of this report. They do not describe the current committed profile (0.5 m/s, 60 deg/s, 1.5 rad/s, −45°, Position); see [Configuration](CONFIGURATION.md).

Reviewed branch: `main`. Starting commit:
`93adb5c780e72cf91d74e72e0403fa9c7052721e`
(`Add ParameterSpec dataclass for operator settings and dashboard presentation`).
The working tree was clean before this task. Changes remain uncommitted. Review
used the actual local checkout, root AGENTS.md, implementation, dependencies and
current operator/configuration/validation guides.

## Verified issues and fixes

Paths in this table are inside `ur_teleop_ws/src/ur_vr_teleop/`.

| Issue and verified original state | Implementation / root cause and fix | Regression evidence |
|---|---|---|
| False unsaved edits: confirmed | `gui.py:mark_dirty` compared full validated settings against editable-only keys. `dashboard_state.py` now compares matching editable keys after stored-unit conversion, finite/range validation and tolerant numeric/array comparison. Invalid text is retained/highlighted; manual confirmations are separate. | `test_dashboard_state.py`: initial/equivalent/changed/restored/invalid/array/unit/hidden/mode cases |
| Stale Apply baseline: confirmed | Original `load_running` ran only on a new session; acknowledgment did not update the baseline. `effective_parameters` is now a complete authoritative readback. Effective, draft, submitted, saved and restart states are separate. Pending edits survive reconciliation. | Pure readback tests and real Tk/ROS Apply/Save workflow |
| Save/restart ambiguity: confirmed | Service acceptance and persistence now have distinct outcomes. Atomic preference writing remains unchanged. The restart-only joint key is never sent at runtime; active final limit and saved limit are separate until both backend/controller match. | Updated `test_preferences.py`; GUI accepted-save, rejected-save, disk-full, restart/revert checks |
| Revert/defaults/conflicts: confirmed | Revert previously reused partial status; defaults also cleared checkboxes. Revert uses complete confirmed values and preserves checkbox drafts and saved restart state. Defaults stage only editable keys. Observed external changes to edited keys block Apply until reconciliation. | State and GUI external-change/default/revert checks |
| Local checks shown as verified: confirmed | `update_dashboard` used checkbox values. New read-only flags expose actual effective mapping verification and optical-loss confirmation. Local selections show pending; rejection, mapping invalidation, restart and stale status remain distinct. | `ros_integration_checks.py:test_dashboard_readback_and_confirmations_match_actual_backend`; GUI unapplied/rejected/session/stale checks |
| Reported missing pose/station/readiness/velocity keys: **already present** | The reviewed backend already published `pose_fresh`, `motion_ready`, `optical_stations`, `tracking_rejection`, and `velocity_diagnostics`. Preserved these, added defensive type/finite handling, and documented their meanings. Missing counts/velocities no longer appear as zero. | State malformed/missing diagnostics tests and GUI incomplete-status checks |
| Individual controller/Servo freshness: confirmed gap | A new status publication could contain an older ownership or Servo observation. New read-only freshness flags reuse the existing 0.6/0.25 s checks; GUI displays stale/unknown independently of message age. Robot mode/program values are documented as latched observations, not a heartbeat. | Backend fresh/expired flag assertions and GUI stale-source assertions |
| Clutch feedback: confirmed unused | `self.motion` was never updated. It now shows existing hand displacement and position-mode target displacement or velocity-mode request with units. Release/status loss makes retained metrics non-current. TF-derived velocity is labeled an estimate. | Pure position/velocity/release metrics tests and GUI status-loss check |
| Disappearing results: confirmed | Live readiness and actions wrote the same variable. Separate channels now retain successes for five seconds and errors until dismissal/correction. History is bounded at 40; current blockers remain independent. | Notification lifetime/history/relevance tests; GUI polling preserves confirmed result and save-failure message |
| Advanced clipping: confirmed | All categories were packed vertically without an inner scroll area. Right-column canvas scrolling, local wheel bindings and focus reveal retain quick controls and the three-column architecture. Compact session/preparation rows keep Disarm accessible. | Actual geometry, mapped-widget and hit-test checks at all three sizes; resize/open/close/focus/scroll checks and screenshots |
| Enable/static header: confirmed | Enable label never reflected intent and the header was always amber. Backend state now drives labels and semantic color; each operation has pending state and duplicate suppression. Disarm remains independently callable. | Request duplicate/Disarm tests and GUI readiness checks |
| Process races: confirmed | Tk polling/RViz and start/stop threads could mutate the same manager. `dashboard_process.py:ManagerWorker` is the sole owner of all manager operations; bounded queues publish results. Stop supersedes queued restart; desktop joins cleanup. Ownership implementation is unchanged. | New worker tests: rapid start, stop/restart ordering, close during start, optional failures; existing owned-process tests, now also camera exit |
| Async robustness: confirmed incomplete | Original requests lacked deadlines and generation metadata. Futures now track identity/session/time, clean up on exceptions/timeouts/session changes/close, and cannot announce an old session's success. Readback timeout releases the form lock without declaring synchronization. | `test_dashboard_async.py`; GUI late-future, new-session, close and real service rejection checks |
| Registry/units: architecture already present | Named registry retained. Backend conversions/validation remain unchanged; array formatting now retains enough precision to avoid round-trip false edits. Only position-specific fields hide; their draft values persist. | All-key round-trip tests; existing preference/migration tests; actual mode-switch checks |
| Camera: existing isolation retained | `camera_view.py` needed no changes. GUI opens/closes the existing viewer in the isolated test. Camera/RViz remain optional manager children. | GUI camera lifecycle and desktop optional-process tests |
| Old GUI render test: stale | It referenced removed indexed fields/widgets. Replaced those obsolete references with stronger named-field behavior, real service/readback assertions and actual visibility checks; existing acceptance/persistence tests were adapted to the new state object without removing their guarantees. | `gui_render_check.py` |

## Files and architecture

Production changes:

- `gui.py`: draft/readback binding, truthful indicators, notifications, bounded
  async requests, scrollable advanced pane and worker integration.
- `dashboard_state.py` (new): pure parameter/notification state and defensive
  telemetry formatting. No ROS, Tk or motion side effects.
- `dashboard_process.py` (new): one manager-owner worker; no alternate process
  ownership or robot communication mechanism.
- `desktop.py`: final cleanup joins that owner instead of racing another close.
- `preferences.py`: RPY display precision only; saved schema, numeric limits,
  defaults and atomic persistence are unchanged.
- `commissioning.py`: imports the parameter-key inventory and adds six read-only
  payload fields: complete parameters, two effective confirmations, heartbeat
  freshness, controller freshness and Servo freshness. No control branch changed.

Tests changed/added: `test_dashboard_state.py`, `test_dashboard_async.py`,
`test_preferences.py`, `test_gui_recovery.py`, `test_desktop.py`,
`ros_integration_checks.py`, and `gui_render_check.py`.

Documentation changed: README, Architecture, Operation, Configuration,
Troubleshooting, Commissioning, this report, and the new
[operator-status contract](OPERATOR_STATUS.md). Root AGENTS.md did not need changes.

The backend remains the authority. Status readback is complete and in ROS/stored
units; there is no ad hoc reconstruction from aliases and no optimistic baseline
update based only on sending a request. Saved preferences never become motion
permission. Per-value startup provenance is not fabricated: actual source
information remains in the existing effective-settings log. See the contract for
all displayed fields, units, validity and unknown behavior.

## Validation executed

The supported system Python and installed Humble workspace were used. The final
import check resolved `gui.py` from the rebuilt `install/ur_vr_teleop/...` overlay,
not an old source/installed mixture.

```bash
source /opt/ros/humble/setup.bash
source ur_teleop_ws/install/local_setup.bash
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
cd ur_teleop_ws
colcon build --base-paths src --packages-select ur_vr_teleop
```

Result: **1 package built successfully**. Rebuilt again after final source edits.
From the repository root, with the fresh overlay sourced:

```bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=147
/usr/bin/python3 -m compileall -q ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop
/usr/bin/python3 -c 'import ur_vr_teleop.gui as g; import ur_vr_teleop.commissioning; import ur_vr_teleop.desktop; print(g.__file__)'
/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_vr_teleop/test ur_teleop_ws/src/ur_vr_sim_teleop/test -q
/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_vr_teleop/test/ros_integration_checks.py -q
/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/gui_render_check.py
```

Results:

- Python compilation/import: passed.
- Full Python unit regression: **194 passed**, no skipped tests; one pre-existing
  xacro/Python compatibility warning.
- Explicit ROS integration: **79 passed**, no skipped tests; rerun after the final
  read-only freshness additions.
- Focused state/async tests after the final notification change: **28 passed**.
- Actual Tk/ROS integration/render script: passed; additionally rerun after final
  GUI changes. Tests use a fake backend on explicit localhost domain 147 and
  temporary preferences. Camera opens without requiring a device.
- `git diff --check`: passed. Local Markdown target paths were checked and passed.
- Scope diff: no changes to committed configuration, tracking, gate, Servo,
  native-controller code, calibration, or driver command generation.

Initial development runs exposed three tests coupled to the old state object,
then a 1280×720 Disarm overflow. Screenshot review also exposed sibling containers
covering buttons. These were corrected, tests were strengthened to check actual
widget hit targets, and the subsequent runs above passed.

C++ suites, `servo_pipeline_check.py`, `mock_startup_check.py`, the calibration
workspace and unrelated ROS packages were **not rerun/rebuilt**: their code and
interfaces were not modified. The existing process tests exercised owned local
sleeping children; no production driver or unattended managed real desktop was
launched. This is not a clean-machine installation test or a timing benchmark.

## Actual application screenshots

All images show the running Tk dashboard with explicitly isolated fake telemetry,
not a mockup and not a real-robot readiness claim. Advanced tuning is open.

- [1440 × 900](ui_v3_1/dashboard-1440x900.png)
- [1366 × 768](ui_v3_1/dashboard-1366x768.png)
- [1280 × 720](ui_v3_1/dashboard-1280x720.png)
- [1280 × 720, advanced pane scrolled to engineering](ui_v3_1/dashboard-1280x720-advanced-bottom.png)

## Scope and remaining validation

Only GUI/state/process-lifecycle integration, tests, documentation, numeric
display precision and read-only status publishing changed. **No motion-control
or safety-gating behavior changed.** The production chain and every tracking,
Cartesian, Servo, gate, native Ruckig and UR-driver limit remain owned by their
existing stages. At the time of this report, the committed 1.5 m/s speed and −40° alignment overrides were intact; the current saved profile differs as noted at the top.

Remaining supervised physical checks: actual dashboard/driver reconnects under
site network conditions, manual mapping and optical-loss procedures, each
Lighthouse's tracking/cold start, physical A-release/fresh-engagement recovery,
restart verification against the real native controller, camera hardware,
payload/TCP/collision-model validity and physical stopping behavior. None is
claimed verified by these software tests.

The existing atomic ROS service has no compare-and-swap revision, so an external
writer can race after the most recent status and before submission. Observed
conflicts are blocked and subsequent mismatching readback is reported; this does
not claim global transactional isolation. Service cancellation cannot undo a
request already executed by the backend. These limitations are documented, not
worked around by bypassing supervision.
