## Why

The active steering stack already has a `reference_time_alignment` hook in the control loop, but the current implementation only rotates the selected BEV reference by integrated yaw and still aligns to the control tick timestamp instead of the time at which the new turn command can affect vehicle motion. This leaves the controller consuming a path that is visually correct for capture time but not geometrically aligned to the control-effective vehicle frame.

## What Changes

- Upgrade reference time alignment from yaw-only `Path(t_camera) -> Path(t_control_now)` into a control-effective reference alignment boundary: `Path(t_camera) -> Path(t_control_effective)`.
- Keep perception, visual-reference selection, scene modules, tracking geometry, yaw control, wheel mixing, and actuator application mutually unaware of the compensation internals; only a control-local copy of `PerceptionResult` is transformed.
- Extend the reference alignment model to estimate a bounded vehicle pose delta from motion history and, in later vertical slices, command history / differential-drive response facts.
- Transform BEV reference samples into the future vehicle frame before reference usability, tracking-geometry fitting, and reference-control readiness are recomputed.
- Preserve the existing `ReferenceTrackingGeometry -> SteeringYawController -> turn_output -> wheel_target_mixer -> wheel PID -> PWM` chain; the yaw controller consumes the same geometry contract and does not learn whether the reference was raw, held, or time-aligned.
- Add runtime parameters and observability fields for effective delay, measured/predicted interval split, forward/lateral/yaw deltas, input/aligned sample counts, fail-closed reasons, and model source.
- Add focused tests and review evidence for yaw-only compatibility, forward encoder compensation, effective-time horizon behavior, SE(2) transform sign, fail-closed motion-history gaps, and controller obliviousness.

## Capabilities

### New Capabilities

- `control-effective-reference-alignment`: Defines the control-effective reference alignment boundary, pose-delta estimation contract, SE(2) reference transformation, fail-closed semantics, observability, and mutual-unawareness constraints.

### Modified Capabilities

- `reference-tracking-geometry`: Updates the tracking geometry contract so it fits the final control-effective reference path while remaining unaware of alignment internals, scene source, control gates, wheel mixing, or actuator state.

## Risk Tier

- `STRICT`: This change alters the timing semantics of steering control input, runtime parameters, diagnostic facts, control-loop preprocessing, and reference-path geometry before yaw control. A sign error or stale/overconfident motion estimate can directly degrade steering behavior even if builds and ordinary perception still pass, so the change requires docs-first and source-first verification-cycle checkpoints, focused local tests, and board/replay evidence before enabling by default.

## Impact

- Affected code and contracts:
  - `new/code/reference/reference_time_alignment.*`
  - `new/code/port/bev_reference_types.hpp`
  - `new/code/port/runtime_parameter_types.hpp`
  - `new/code/port/motion_history_types.hpp`
  - future command-history / differential-drive response contract under `port/` or `control/`
  - `new/code/runtime/loops/control_loop.cpp`
  - `new/code/observability/control_debug_snapshot.hpp`
  - `new/code/observability/control_debug_reporter.cpp`
  - `new/code/transport/steering_media_protocol.cpp` and user-side snapshot parsers if serialized fields change
  - `new/config/default_params.json` and `new/config/default_params.md`
  - `new/verification/tests/reference_time_alignment_test.cpp` and new focused tests for control-effective alignment
- Affected behavior:
  - selected/held reference paths are transformed only inside the control loop's control-local perception copy
  - reference usability, tracking geometry, and reference-control readiness are recomputed from the aligned path
  - yaw controller and wheel mixer keep their current public inputs and formulas
- Non-goal impact:
  - no servo terminal, servo naming, or direct actuator terminal restoration
  - no change to visual-reference selection, CircleV2 scene ownership, cross evidence, BEV sparse sampling, or wheel PID formulas
- Participating workflow and skills:
  - `openspec-align` for source alignment against existing reference/control code
  - `openspec-architect` for architecture decisions and stack equivalents
  - `openspec-artifact-verify` / unified `openspec-verify` for docs-first review
  - `openspec-apply-change`, `openspec-verify-change`, and `openspec-repair-change` for implementation and source-first review
  - `verify-sequence/default` with `verify-reviewer-inline-v3`, authoritative findings/evidence JSON, and caller-maintained current-state `agent-table.json`
