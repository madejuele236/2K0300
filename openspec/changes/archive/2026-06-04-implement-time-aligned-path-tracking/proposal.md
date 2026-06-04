## Why

The current control-side reference time alignment only compensates yaw between
camera capture time and the current control tick. This leaves forward motion,
lateral body-frame motion, and control-effective delay outside the tracking
geometry that the yaw controller consumes.

## What Changes

- Add a time-aligned path tracking capability that preserves
  `PerceptionResult.reference_path` as `Path(t_camera)` while producing a
  control-side aligned copy for `Path(t_control_effective)`.
- Add a `ControlCommandHistory` fact buffer and a `VehiclePoseDelta` contract so
  motion and applied-command facts can be estimated without making reference
  alignment read sensor or controller internals.
- Replace yaw-only reference alignment with an SE(2) alignment API that consumes
  `BEVReferencePath + VehiclePoseDelta` and produces an aligned path plus
  detailed alignment facts.
- Clean up migration compatibility fields after implementation slices are
  complete so final active contracts use explicit current names such as
  `delta_forward_m` instead of retaining legacy aliases.
- Extend `REFERENCE_TIME_ALIGNMENT` parameters, debug snapshot fields, and
  parameter documentation for effective delay, measured/predicted pose deltas,
  encoder forward integration, wheel-yaw fallback, and command-yaw prediction.
- Keep defaults fail-closed and behavior-preserving: alignment, encoder forward
  integration, future prediction, and command-yaw prediction remain disabled by
  default until calibrated.

## Capabilities

### New Capabilities
- `time-aligned-path-tracking`: Defines control-effective-time reference
  alignment, vehicle pose-delta estimation, command history facts, runtime
  orchestration ownership, parameter gates, debug evidence, and fail-closed
  behavior.

### Modified Capabilities
- `reference-tracking-geometry`: No requirement change. It continues to consume
  the selected/aligned `BEVReferencePath`; this change only changes which path
  the runtime provides to it when time alignment is enabled.
- `steering-tuning-media-observability`: No requirement change is needed for
  media grouping, but implementation will extend control debug/report evidence
  for the richer `reference_time_alignment` facts.

## Risk Tier

- `STANDARD`: The change touches `port` contracts, runtime control-loop
  orchestration, runtime parameter parsing/defaults, reference alignment,
  observability fields, and local verification tests. It does not change camera
  acquisition, visual reference generation, safety-gate ownership, yaw
  controller inputs, wheel mixer APIs, actuator adapter APIs, or default runtime
  behavior because all new compensation paths are disabled by default.

## Impact

- `port`: add command-history and vehicle-pose-delta types; extend
  `ReferenceTimeAlignmentFacts` and `ReferenceTimeAlignmentParameters`.
- `estimation`: add `VehiclePoseDeltaEstimator` that consumes motion history,
  command history, and alignment parameters without depending on reference path
  or controller types.
- `reference`: replace yaw-only alignment internals with SE(2) path alignment
  from an already-estimated `VehiclePoseDelta`.
- `runtime`: make `ControlLoop` the sole orchestrator that combines perception,
  motion history, command history, pose-delta estimation, reference alignment,
  tracking geometry, readiness, and downstream control.
- `platform`/`config`: parse, validate, document, and default the expanded
  `REFERENCE_TIME_ALIGNMENT` parameter surface.
- `observability`: expose measured/predicted alignment facts and source flags in
  control debug output.
- `verification`: add focused estimator and SE(2) alignment tests, update
  existing alignment/default/build regressions, and run the required OpenSpec
  docs-first/source-first gates.
