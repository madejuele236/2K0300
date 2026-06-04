## 0. Verification Contract

- Shared sequence:
  - `openspec/schemas/ai-enforced-workflow/verification-sequence.md#verify-sequence/default`
- Shared JSON verification contract:
  - `openspec/schemas/modules/verification-cycle/contracts/verification-cycle-core-v1.json`
  - `openspec/schemas/modules/verification-cycle/contracts/verification-cycle-openspec-adapter-v1.json`
  - `openspec/schemas/modules/verification-cycle/contracts/verification-cycle-agent-table-v1.json`
- Shared field groups:
  - `invocation_common_required`
  - `output_paths_required`
  - `verifier_evidence_required`
  - `valid_pass_requirements`
  - `partial_scope_rule`
  - `subject_required_any_of`
  - `findings_required`
  - `finding_object_required`
  - `finding_semantics`
  - `repair_routing_rules`
- Routing target for blocking findings:
  - `openspec-repair-change`
- Supported continuation overrides:
  - `verify-only`
  - `dry-run`
  - `manual_pause`
- Artifact-completion gate ownership:
  - when this task list completes the schema's `applyRequires` set under
    `ai-enforced-workflow`, the active artifact-creation caller runs docs-first
    review before implementation entry
  - `openspec-apply-change` does not own that docs-first artifact gate

## 1. Vertical Slice: Port Contracts And Default-Off Parameters

- [x] 1.1 Add `new/code/port/control_command_history_types.hpp` with a fixed
  capacity ring buffer for applied command facts.
- [x] 1.2 Add `new/code/port/vehicle_pose_delta_types.hpp` with measured,
  predicted, source-flag, timestamp, and SE(2) delta fields.
- [x] 1.3 Extend `ReferenceTimeAlignmentFacts` in
  `new/code/port/bev_reference_types.hpp` with control-effective timestamp,
  measured/predicted fields, forward/lateral/yaw deltas, and source flags using
  current names such as `delta_forward_m`; do not leave `delta_s_m` as a final
  active compatibility field.
- [x] 1.4 Extend `ReferenceTimeAlignmentParameters` in
  `new/code/port/runtime_parameter_types.hpp` for effective delay, prediction
  horizon, encoder/wheel calibration, yaw source toggles, command-yaw
  prediction, actuator tau, and max SE(2) deltas.
- [x] 1.5 Update `new/code/platform/param_store.cpp`,
  `new/config/default_params.json`, and `new/config/default_params.md` so the
  expanded `REFERENCE_TIME_ALIGNMENT` surface parses, validates, documents, and
  defaults to behavior-preserving disabled values.
- [x] 1.6 Run `rtk bash new/verification/tests/run_runtime_parameter_defaults_test.sh`
  and record the command result in implementation evidence.

## 2. Vertical Slice: Vehicle Pose Delta Estimator

- [x] 2.1 Add `new/code/estimation/vehicle_pose_delta_estimator.hpp/.cpp` so
  estimator code consumes motion history, command history, timestamps, and
  alignment parameters without including reference-path, tracking-geometry, yaw
  controller, wheel mixer, or actuator headers.
- [x] 2.2 Implement measured-window integration for IMU yaw, optional encoder
  forward distance, wheel-yaw fallback, integration-gap rejection,
  monotonic-history rejection, and finite/max-delta validation.
- [x] 2.3 Implement future-window prediction gated by
  `FUTURE_PREDICTION_ENABLED`, `FUTURE_PREDICTION_MAX_MS`, and optional usable
  applied command-yaw prediction.
- [x] 2.4 Add `new/verification/tests/vehicle_pose_delta_estimator_test.cpp` and
  `new/verification/tests/run_vehicle_pose_delta_estimator_test.sh` covering
  encoder forward, IMU yaw, disabled future prediction, constant-velocity
  prediction, and command filtering.
- [x] 2.5 Run `rtk bash new/verification/tests/run_vehicle_pose_delta_estimator_test.sh`
  and record the command result in implementation evidence.

## 3. Vertical Slice: SE2 Reference Time Alignment

- [x] 3.1 Replace the reference time-alignment API in
  `new/code/reference/reference_time_alignment.hpp/.cpp` with
  `AlignReferencePathToVehiclePoseDelta`.
- [x] 3.2 Remove motion-history dependency from reference alignment and
  implement inverse SE(2) transformation over the leading observed reference
  prefix with fail-closed age, pose-delta, max-delta, finite, behind-vehicle,
  and minimum-sample handling.
- [x] 3.3 Update `new/verification/tests/reference_time_alignment_test.cpp` to
  cover disabled pass-through, measured SE(2) alignment, forward shift, lateral
  shift, future effective-time facts, stale input, pose-delta failure, and
  insufficient samples.
- [x] 3.4 Run `rtk bash new/verification/tests/run_reference_time_alignment_test.sh`
  and record the command result in implementation evidence.

## 4. Vertical Slice: ControlLoop Orchestration And Command History

- [x] 4.1 Add `ControlCommandHistory` to `new/code/runtime/runtime_state.hpp`
  and clear it in control-loop start/reset/timer-failure reset paths.
- [x] 4.2 Update `BuildControlTimePerception` in
  `new/code/runtime/loops/control_loop.cpp` to compute
  `control_effective_time_ms`, call `EstimateVehiclePoseDelta`, call
  `AlignReferencePathToVehiclePoseDelta`, and recompute usability,
  lateral-error comparison, tracking geometry, and readiness only after valid
  alignment.
- [x] 4.3 Copy `state_.command_history` into the control tick and push a
  `ControlCommandHistorySample` after actuator application/debug snapshot
  assembly with raw/applied turn output, wheel targets, PWM command values, and
  command validity flags.
- [x] 4.4 Ensure alignment failure clears the control copy's usability,
  lateral-error, tracking geometry, and readiness facts with
  `reference_time_alignment_*` reasons so existing gate/readiness logic blocks
  control.
- [x] 4.5 Run `rtk env SKIP_UPLOAD=1 new/user/build.sh` as the required runtime
  no-upload build before any board interaction; do not upload or auto-start
  runtime from this task.
- [x] 4.6 Board smoke task for runtime/platform impact: if the user explicitly
  authorizes board upload/start in this implementation run, use the
  repo-supported board upload/runtime flow and capture a short steering debug
  evidence sample showing `reference_time_alignment.control_time_ms`,
  `control_effective_time_ms`, `delta_forward_m`, `delta_lateral_m`,
  `delta_yaw_rad`, and `reason`; if board execution is not authorized or board
  access is unavailable, record the no-run reason in implementation evidence and
  keep the local no-upload build as the completed safety gate.

## 5. Vertical Slice: Debug Evidence Surface

- [x] 5.1 Extend `new/code/observability/control_debug_snapshot.hpp` and
  `BuildControlDebugSnapshot` with the full expanded
  `ReferenceTimeAlignmentFacts` field set.
- [x] 5.2 Update debug reporter and any assistant/media selftests directly
  impacted by the new field set so runtime evidence can inspect capture time,
  control effective time, measured/predicted deltas, and source flags.
- [x] 5.3 Run `rtk bash new/verification/tests/run_assistant_telemetry_selftest.sh`
  and `rtk bash new/verification/tests/run_steering_media_selftest.sh` and
  record the command results in implementation evidence.

## 6. Regression Verification

- [x] 6.1 Run the required steering/control regression set:
  `rtk bash new/verification/tests/run_power_adapter_threshold_test.sh`,
  `rtk bash new/verification/tests/run_startup_low_voltage_order_test.sh`,
  `rtk bash new/verification/tests/run_bev_simple_perception_test.sh`,
  `rtk bash new/verification/tests/run_visual_reference_orchestration_test.sh`,
  `rtk bash new/verification/tests/run_reference_usability_lateral_error_test.sh`,
  `rtk bash new/verification/tests/run_reference_tracking_geometry_test.sh`,
  `rtk bash new/verification/tests/run_bev_simple_residual_check.sh`, and
  `rtk bash new/verification/tests/run_perf_counter_test.sh`.
- [x] 6.2 Run `rtk git diff --check`, `rtk openspec validate --all`, and
  `rtk codegraph sync`.
- [x] 6.3 Confirm dependency boundaries with source inspection: estimator does
  not include reference/control headers, reference alignment does not include
  motion/command/control headers, and yaw controller remains unaware of
  alignment and pose-delta types.
- [x] 6.4 Final compatibility cleanup before source-first review: remove
  temporary migration aliases from active port/debug/config/docs/tests,
  including `delta_s_m`; run an active-surface search proving no legacy
  compatibility field remains outside archive/superseded historical material.

## 7. Verification And Review

- [x] 7.1 [Checkpoint] Run docs-first verifier-subagent review for
  `openspec/changes/implement-time-aligned-path-tracking/{proposal.md,design.md,tasks.md,specs/**/*.md}`
  using `verify-sequence/default`. Use the verification contract above for
  field groups in `verification-cycle-core-v1.json` and
  `verification-cycle-openspec-adapter-v1.json`. Require
  `review_goal=implementation_correctness`. Write authoritative findings JSON
  to
  `review/review-runs/implement-time-aligned-path-tracking/docs-first-findings.json`
  and verifier evidence JSON to
  `review/review-runs/implement-time-aligned-path-tracking/docs-first-verifier-evidence.json`;
  the caller/orchestrator reconciles and writes
  `review/review-runs/implement-time-aligned-path-tracking/agent-table.json`.
  Follow `cycle_rules` for agent lifecycle. Require fields from
  `verifier_evidence_required`, enforce valid-pass requirements, require subject
  binding through `subject_required_any_of`, carry findings routing semantics
  from `findings_required / finding_object_required / finding_semantics /
  repair_routing_rules`, and require explicit `scope` for any partial
  verification.
- [x] 7.2 Repair any docs-first blocking finding through
  `openspec-repair-change`, then rerun the same docs-first checkpoint until the
  active verifier returns a valid pass.
- [x] 7.3 [Checkpoint] Run source-first verifier-subagent review for changed
  implementation, tests, config, OpenSpec artifacts, and directly impacted code
  using `verify-sequence/default`. Use the same shared field groups, require
  authoritative findings JSON at
  `review/review-runs/implement-time-aligned-path-tracking/source-first-findings.json`,
  verifier evidence JSON at
  `review/review-runs/implement-time-aligned-path-tracking/source-first-verifier-evidence.json`,
  current-state-only `agent-table.json`, subject binding through
  `subject_required_any_of`, and findings routing semantics from
  `findings_required / finding_object_required / finding_semantics /
  repair_routing_rules`.
- [x] 7.4 Repair any source-first blocking finding through
  `openspec-repair-change`, then rerun source-first review until the active
  verifier returns a valid pass.
- [x] 7.5 Run an additional source-first verification pass over the same final
  implementation surface and continue repair/rerun until there are two
  consecutive valid source-first passes.

## 8. Sync, Archive, And Git

- [x] 8.1 Mark completed task checkboxes only after implementation evidence
  exists for each task.
- [x] 8.2 Sync
  `openspec/changes/implement-time-aligned-path-tracking/specs/time-aligned-path-tracking/spec.md`
  into `openspec/specs/time-aligned-path-tracking/spec.md`.
- [x] 8.3 Archive `implement-time-aligned-path-tracking` to
  `openspec/changes/archive/2026-06-04-implement-time-aligned-path-tracking/`
  after specs are synced and verification passes.
- [x] 8.4 Run final `rtk openspec validate --all`, focused tests needed for any
  post-archive path movement, and `rtk git diff --check`.
- [x] 8.5 Commit the scoped implementation, OpenSpec, tests, config, and review
  evidence changes with a conventional commit, then push to the remote branch.
