## Context

`new/docs/time_aligned_path_tracking_implementation.md` identifies the current
gap: `reference_time_alignment` aligns `Path(t_camera)` to the control tick with
yaw-only history integration. The control loop already has the correct
orchestration point, `BuildControlTimePerception`, and recomputes usability,
lateral-error comparison, tracking geometry, and reference-control readiness
after alignment. The missing pieces are a clean vehicle pose-delta contract,
command-history facts for future prediction, and an SE(2) reference transform
that does not make reference code read motion history directly.

## Goals / Non-Goals

**Goals:**

- Preserve perception output semantics: `PerceptionResult.reference_path`
  remains `Path(t_camera)` when published by perception.
- Make `runtime::ControlLoop` the only owner that combines perception,
  `MotionHistory`, `ControlCommandHistory`, runtime parameters, pose-delta
  estimation, reference alignment, tracking geometry, readiness, and control.
- Add `VehiclePoseDeltaEstimator` as an estimation module that consumes sensor
  and applied-command facts and outputs a body-frame delta from capture time to
  control-effective time.
- Make `ReferenceTimeAlignment` consume only `BEVReferencePath`,
  `VehiclePoseDelta`, timestamps, and runtime parameters.
- Fail closed on stale references, unavailable history, uncalibrated encoder
  scale, excessive delta, future-prediction violations, and insufficient aligned
  samples.
- Keep default runtime behavior unchanged by leaving alignment and all new
  prediction/integration paths disabled by default.

**Non-Goals:**

- Do not change visual reference generation, hold selection, CircleV2/cross
  evidence, or ordinary BEV reference semantics.
- Do not change `SteeringYawController` inputs; it continues to consume only
  `ReferenceTrackingGeometry`.
- Do not tune board-specific `encoder_ticks_to_meter`,
  `effective_delay_ms`, or `turn_output_to_yaw_rate_gain` in this change.
- Do not change safety-gate ownership, wheel mixer APIs, wheel PID, PWM output,
  actuator adapter APIs, or camera capture.

## Decisions

### Decision 1: Split Pose Delta From Reference Alignment

Problem: the current alignment helper reads `MotionHistory` directly, making the
reference layer aware of motion estimation details.

Alternatives considered:
- Keep yaw integration inside `reference_time_alignment.cpp`; smallest diff but
  preserves the boundary leak.
- Move all alignment into control loop; keeps reference pure but inlines
  estimator math into orchestration.
- Add `VehiclePoseDeltaEstimator` and make alignment consume the resulting
  `VehiclePoseDelta`.

Chosen option: add `new/code/estimation/vehicle_pose_delta_estimator.*` and
`new/code/port/vehicle_pose_delta_types.hpp`.

- Stack Equivalent: state estimation produces a relative SE(2) odometry delta;
  mapping/reference alignment consumes that delta without knowing the sensors.
- Named Deliverables: `VehiclePoseDelta`, `EstimateVehiclePoseDelta`, estimator
  unit test, and runtime orchestration call.
- Failure Semantics: invalid timestamps, missing history, non-monotonic history,
  missing yaw source, missing encoder scale, prediction horizon excess, and
  excessive deltas return `valid=false` with a reason string.
- Boundary Examples: estimator may include motion/command history and runtime
  alignment parameters; it must not include BEV reference, tracking geometry,
  yaw controller, wheel mixer, or actuator headers.
- Contrast Structure: estimator outputs vehicle motion facts; reference
  alignment transforms path facts.
- Verification Hook: `run_vehicle_pose_delta_estimator_test.sh` covers measured
  encoder forward, IMU yaw, disabled future prediction, and bounded future
  constant-velocity prediction. On-board hook: debug output exposes
  `measured_forward_mps`, `measured_yaw_rate_radps`, `predicted_ms`, and source
  flags so a board run can compare estimator facts against encoder/IMU traces.
- Feedback Loop: run the focused estimator test before integrating it into
  `ControlLoop`, then include it in the no-upload build/test regression.

### Decision 2: Align To Control-Effective Time With SE(2), Not Yaw-Only

Problem: yaw-only alignment leaves forward and lateral body-frame displacement
out of the path consumed by tracking geometry.

Alternatives considered:
- Add only `delta_s_m` to the existing transform; simpler but cannot represent
  lateral displacement or clean future prediction facts.
- Keep old API and add more `MotionHistory` parameters; worsens layer coupling.
- Replace the public API with `AlignReferencePathToVehiclePoseDelta`.

Chosen option: replace the active alignment call with
`AlignReferencePathToVehiclePoseDelta(path, capture, now, effective, delta,
params)` and transform the leading observed prefix by inverse SE(2).

- Stack Equivalent: a map/reference path is transformed into the future vehicle
  body frame using the inverse of estimated vehicle motion.
- Named Deliverables: expanded `ReferenceTimeAlignmentFacts`, new alignment API,
  SE(2) transform implementation, updated alignment unit test.
- Failure Semantics: disabled alignment is valid pass-through; enabled alignment
  fails closed with an empty reference if timestamps are invalid, age exceeds
  limits, pose delta is invalid, deltas exceed limits, or aligned samples are
  insufficient.
- Boundary Examples: reference alignment may include BEV reference and
  vehicle-pose-delta types; it must not include `motion_history_types.hpp`,
  command-history types, or control/yaw headers.
- Contrast Structure: `delta_forward_m` is the current forward-displacement
  fact. Any transient compatibility bridge for the old `delta_s_m` name must be
  removed before closure so active contracts do not retain legacy aliases.
- Verification Hook: `run_reference_time_alignment_test.sh` verifies disabled
  pass-through, measured SE(2) alignment, forward/lateral shift, future
  effective-time facts, stale input, excessive deltas, and insufficient samples.
  On-board hook: `control_debug_reporter` prints control/effective timestamps
  and SE(2) deltas for captured steering runs.
- Feedback Loop: update the alignment test first, then compile the control-loop
  build to catch any stale API callers.

### Decision 3: Command History Records Applied Facts, Not Controller Strategy

Problem: future yaw prediction needs a command fact surface, but putting command
fields in `MotionHistorySample` violates its sensor-facts contract.

Alternatives considered:
- Extend `MotionHistorySample`; convenient but mixes sensor and actuator-command
  facts.
- Read `last_command` only; misses whether a command was diagnostic-only,
  emergency-stop, hold-disarmed, or actually applied.
- Add `ControlCommandHistory` as a separate port contract.

Chosen option: add `ControlCommandHistorySample` and `ControlCommandHistory` and
push one sample per control tick after actuator application and debug snapshot
assembly.

- Stack Equivalent: a controller command/event log separate from odometry.
- Named Deliverables: command-history port type, runtime-state field,
  control-loop push/reset points, estimator lookup of latest usable command.
- Failure Semantics: diagnostic-only, hold-disarmed, emergency-stop, invalid, or
  unapplied commands cannot drive command-yaw prediction.
- Boundary Examples: command history can carry raw/applied turn output, wheel
  targets, and PWM commands; it does not describe reference geometry or yaw
  controller internals.
- Contrast Structure: `MotionHistory` remains sensor facts; `CommandHistory`
  remains applied-command facts.
- Verification Hook: estimator tests cover command-filter behavior where
  command prediction is enabled. On-board hook: command-history-derived
  prediction can be checked against debug `used_command_prediction` and
  `predicted_yaw_rate_radps`.
- Feedback Loop: unit test command filtering before enabling any command-yaw
  prediction in configuration.

### Decision 4: Default-Off Parameter Surface With Explicit Calibration Gates

Problem: encoder scale, future delay, and turn-output-to-yaw-rate gain are
board-calibrated values. Enabling them by default would create unproven runtime
behavior.

Alternatives considered:
- Turn on full compensation immediately; highest behavior risk.
- Keep only hidden constants; easier to code but not inspectable or tunable.
- Add explicit parameters with default-off behavior and validation.

Chosen option: expand `REFERENCE_TIME_ALIGNMENT` with effective delay,
prediction horizon, encoder scale, wheel track, source toggles, command-yaw
prediction gain/tau, and max delta gates. Defaults keep `ENABLED=0`,
`USE_ENCODER_FORWARD=0`, `FUTURE_PREDICTION_ENABLED=0`, and
`COMMAND_YAW_PREDICTION_ENABLED=0`.

- Stack Equivalent: a feature gate plus calibration parameter group.
- Named Deliverables: `RuntimeParameters`, `param_store.cpp`,
  `default_params.json`, and `default_params.md` updates.
- Failure Semantics: malformed or out-of-range parameters reject the parameter
  set; enabled estimator paths still fail closed if required calibration values
  are absent.
- Boundary Examples: parameter validation can check ranges; formulas must not
  silently clamp semantic failures.
- Contrast Structure: runtime defaults are behavior-preserving, while board
  tuning can opt in phase by phase.
- Verification Hook: `run_runtime_parameter_defaults_test.sh` validates parsing,
  defaults, and range rejection. On-board hook: steering media/config snapshots
  expose the active parameter values used during a run.
- Feedback Loop: run parameter-default tests before and after build.

### Decision 5: Runtime Orchestrates, Business Modules Stay Unaware

Problem: control-effective alignment crosses perception, motion facts, command
facts, reference facts, tracking geometry, and control timing. Putting global
knowledge into any business module would recreate coupling.

Alternatives considered:
- Put estimator calls in perception pipeline; violates perception's ignorance of
  control delay and commands.
- Put time alignment into the yaw controller; violates the controller's
  tracking-geometry-only contract.
- Keep orchestration in `ControlLoop::BuildControlTimePerception`.

Chosen option: `ControlLoop` computes the effective time, calls estimator,
calls reference alignment, and recomputes usability/lateral-error comparison,
tracking geometry, and readiness on the aligned control copy.

- Stack Equivalent: application service composes domain services.
- Named Deliverables: updated `BuildControlTimePerception` signature/callers,
  runtime-state command history, reset clearing, and control debug copies.
- Failure Semantics: invalid alignment zeros reference usability, lateral-error,
  tracking geometry, and readiness with a `reference_time_alignment_*` reason so
  the existing readiness/gate chain blocks control.
- Boundary Examples: yaw controller still only receives
  `ReferenceTrackingGeometry`; safety gate remains the owner of low voltage,
  stale, perception invalid, IMU, and encoder vetoes.
- Contrast Structure: orchestration knows all modules; modules know only their
  input/output contracts.
- Verification Hook: build/no-upload regression plus focused control-loop
  tests where available. On-board hook: a no-upload build is mandatory before
  board upload; later board smoke can inspect debug reporter and media output
  without starting runtime automatically.
- Feedback Loop: after focused unit tests pass, run the required steering
  regression scripts and `rtk env SKIP_UPLOAD=1 new/user/build.sh`.

## Engineering Discipline

- Principles reference:
  `openspec/schemas/ai-enforced-workflow/engineering-principles.md`
- Domain language / ADRs consulted:
  `README.md`, `new/docs/time_aligned_path_tracking_implementation.md`,
  `new/config/default_params.md`, existing `reference-tracking-geometry` spec,
  and active code under `new/code/{port,reference,runtime,platform}`.
- Primary feedback loop:
  focused estimator/alignment/default tests first, then required steering
  regression scripts and no-upload build, then source-first verifier pass.
- Prototype question, if any:
  no throwaway prototype is planned; the implementation document already
  resolves the required decomposition.
- Hard dependencies:
  `verify-sequence/default`, authoritative findings/evidence, valid-pass
  requirements, subject binding, and current-state `agent-table.json`
- Soft dependencies:
  glossary, ADRs, architecture heuristics, and prototype notes; use them when
  present, but do not create auxiliary review gates for them

## Independent Verification Plan (STANDARD/STRICT)

Document verification using shared sequence `verify-sequence/default` from:
`openspec/schemas/ai-enforced-workflow/verification-sequence.md`
and shared verification-cycle contracts:

- `openspec/schemas/modules/verification-cycle/contracts/verification-cycle-core-v1.json`
- `openspec/schemas/modules/verification-cycle/contracts/verification-cycle-openspec-adapter-v1.json`
- `openspec/schemas/modules/verification-cycle/contracts/verification-cycle-agent-table-v1.json`

Stage A flow:

- checkpoints use the same `active/non_active` verification cycle
- docs-first checkpoints use changed `proposal/specs/design/tasks` as the
  primary surface
- source-first checkpoints use changed code, tests, and directly impacted code
  as the primary surface
- approved docs remain reference material when source-first review runs
- verification continues a usable `active` agent first
- callers prefer `send_input` while that same `active` agent is still open
- callers use `continuation_probe` to distinguish resume from recovery spawn
- if no usable `active` agent exists, the orchestrator spawns one
- only `block -> pass` marks an agent `non_active`
- termination depends only on a valid `active` pass

Runtime profile policy:

- Use verifier runtime profile from
  `openspec/schemas/ai-enforced-workflow/agents/verify-reviewer.toml`.

Loop rule:

- an `active` agent that reports `block` stays authoritative until that same
  agent returns `pass`
- `agent-table.json` stays current-state-only; recovery lives in
  `continuation_probe`
- valid `pass` requires
  `review_coverage.coverage_status=complete` and
  `review_coverage.exhaustive=true`
- partial verification requires explicit `review_scope.scope`
- only the main orchestrator may authorize resume/spawn/repair/terminate, and
  it must not substitute its own judgment for verifier output

Shared field groups from `verification-cycle-core-v1.json` and
`verification-cycle-openspec-adapter-v1.json`:

- `invocation_common_required`
- `output_paths_required`
- `verifier_evidence_required`
- `valid_pass_requirements`
- `partial_scope_rule`

Review completion contract:

- execution evidence MUST record:
  - `review_goal`
  - `review_phase`
  - `review_scope`
  - `review_coverage`
  - `reviewed_paths`
  - `skipped_paths`
  - `reviewed_axes`
  - `unreviewed_axes`
- each checkpoint MUST maintain `agent-table.json`

### Review Checkpoints

- Shared sequence reference: `verify-sequence/default`
- Review goal: `implementation_correctness`
- Verifier agent path:
  `openspec/schemas/ai-enforced-workflow/agents/verify-reviewer.toml`
- Invocation template id: `verify-reviewer-inline-v3`
- Default loop behavior:
  - resume `active` first
  - prefer `send_input` while that same `active` agent is still open
  - use `continuation_probe` to distinguish resume from dedicated recovery
    spawn
  - spawn when no usable `active` agent exists
  - repair follows `block`
  - only `block -> pass` marks `non_active`
  - final termination requires a valid `active` pass
- Authoritative verifier-subagent findings JSON path:
  `review/review-runs/implement-time-aligned-path-tracking/docs-first-findings.json`
  and
  `review/review-runs/implement-time-aligned-path-tracking/source-first-findings.json`
- Verifier execution evidence JSON path:
  `review/review-runs/implement-time-aligned-path-tracking/docs-first-verifier-evidence.json`
  and
  `review/review-runs/implement-time-aligned-path-tracking/source-first-verifier-evidence.json`
- Agent table path:
  `review/review-runs/implement-time-aligned-path-tracking/agent-table.json`
- Continuation target on pass:
  docs-first pass unlocks implementation; source-first pass unlocks extra
  repeated verification, spec sync, archive, and git push.

Checkpoint-specific primary surfaces:

- artifact-completion docs-first review: changed `proposal/specs/design/tasks`
- active-change source-first review: changed code, changed tests, directly
  impacted code

## Migration Plan

1. Add port contracts and parameter fields with defaults preserving behavior.
2. Add estimator and reference-alignment SE(2) tests before runtime wiring.
3. Wire runtime orchestration while keeping `REFERENCE_TIME_ALIGNMENT.ENABLED=0`.
4. Extend debug/report fields and parameter docs.
5. Clean up migration compatibility fields after code tasks are complete:
   active port/debug/config/docs/tests must use current field names, and legacy
   aliases such as `delta_s_m` must not remain in active contracts unless a
   separate explicit external-protocol requirement is created.
6. Run local focused tests, required steering regressions, no-upload build,
   OpenSpec source-first verification, and one extra source-first pass until two
   consecutive passes are achieved.
7. Sync the delta spec into `openspec/specs/`, archive the change, and commit
   only the scoped implementation/artifact changes.

Rollback is disabling `REFERENCE_TIME_ALIGNMENT.ENABLED`, which returns to the
existing pass-through path. Because defaults stay disabled, a config rollback is
not needed for existing deployments.

## Open Questions

- Board-specific calibration values for `encoder_ticks_to_meter`,
  `effective_delay_ms`, and `turn_output_to_yaw_rate_gain` remain unknown. This
  change exposes the gates and evidence but does not select final tuned values.
- Full board smoke evidence is useful after implementation, but the required
  final local gate is a no-upload build unless the user separately asks to
  upload/start runtime.

## Risks / Trade-offs

- The estimator can be correct by contract but still ineffective until board
  calibration values are measured. The default-off rollout limits this risk.
- SE(2) alignment can drop samples behind the future vehicle frame; this is
  intentional fail-closed behavior, but it may reduce aligned sample counts
  under aggressive delay settings.
- Command-yaw prediction uses a simple first-order response model. It is
  exposed and gated so future evidence can replace or refine it without changing
  yaw-controller semantics.
