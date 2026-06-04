# time-aligned-path-tracking Specification

## Purpose
Define capture-time BEV reference preservation, control-effective-time reference
alignment, vehicle pose-delta estimation, command-history ownership, default-off
runtime parameters, fail-closed behavior, and debug/media observability for
time-aligned path tracking.

## Requirements

### Requirement: Perception Path Remains Capture-Time Fact
The system SHALL preserve `PerceptionResult.reference_path` published by
perception as the BEV reference path at camera capture time,
`Path(t_camera)`. Control-side time alignment SHALL operate on a control-loop
copy and SHALL NOT mutate the shared perception fact.

#### Scenario: Control copy is aligned without polluting perception
- **WHEN** the control loop receives a fresh perception result and time
  alignment is enabled
- **THEN** the control-loop copy MAY replace its `reference_path` with an
  aligned path
- **AND** the shared perception result SHALL continue to represent
  `Path(t_camera)`

#### Scenario: Perception remains unaware of control delay
- **WHEN** reviewers inspect perception pipeline and frontend code
- **THEN** perception SHALL NOT read effective-delay, command-history, actuator,
  wheel-mixer, or yaw-controller state to modify reference path timing

### Requirement: Vehicle Pose Delta Estimator Is Sensor And Command Fact Owner
The system SHALL provide a `VehiclePoseDeltaEstimator` that consumes
`MotionHistory`, `ControlCommandHistory`, a start time, current time,
control-effective end time, and `REFERENCE_TIME_ALIGNMENT` parameters to produce
a `VehiclePoseDelta`.

The estimator SHALL NOT depend on `BEVReferencePath`,
`ReferenceTrackingGeometry`, visual reference candidates, safety-gate state,
wheel mixer internals, yaw-controller internals, or actuator adapters.

#### Scenario: Measured window produces body-frame delta
- **WHEN** motion history covers the requested capture-time to current-time
  window with acceptable sampling gaps
- **THEN** the estimator SHALL integrate yaw from the configured yaw source
- **AND** it SHALL integrate forward distance only when encoder forward
  integration is enabled and calibrated
- **AND** it SHALL report measured velocity, measured yaw rate, integrated
  segment count, source flags, and `valid=true`

#### Scenario: Wheel yaw fallback is explicit
- **WHEN** IMU yaw is unavailable for a measured segment
- **AND** `USE_WHEEL_YAW_FALLBACK` is enabled
- **AND** encoder samples are valid
- **AND** `ENCODER_TICKS_TO_METER` and `WHEEL_TRACK_M` are calibrated positive
  values
- **THEN** the estimator SHALL estimate yaw from the left/right encoder
  distance difference divided by wheel track
- **AND** it SHALL set `used_wheel_yaw=true`
- **AND** it SHALL fail closed with a yaw-history or calibration reason when
  those prerequisites are not met

#### Scenario: Future effective time is gated
- **WHEN** the requested end time is after the current control time
- **THEN** the estimator SHALL require future prediction to be enabled
- **AND** it SHALL reject horizons above `FUTURE_PREDICTION_MAX_MS`
- **AND** it SHALL report `predicted_ms` and prediction source fields when the
  future segment is accepted

#### Scenario: Command yaw prediction uses only usable applied commands
- **WHEN** command-yaw prediction is enabled
- **THEN** the estimator SHALL use only a latest command-history sample that is
  valid, actuator-applied, not diagnostic-only, not hold-disarmed, and not
  emergency-stop
- **AND** invalid command samples SHALL NOT drive predicted yaw rate

### Requirement: Reference Time Alignment Consumes Vehicle Pose Delta
The reference time-alignment module SHALL transform a leading observed
`BEVReferencePath` prefix using a supplied `VehiclePoseDelta` and SHALL NOT read
motion history, command history, sensor adapters, controller internals, or
actuator state.

#### Scenario: SE2 alignment transforms path to control-effective body frame
- **WHEN** alignment is enabled, timestamps are valid, the reference is within
  age limits, and the pose delta is valid
- **THEN** the aligner SHALL transform the leading observed prefix by inverse
  SE(2) using `delta_forward_m`, `delta_lateral_m`, and `delta_yaw_rad`
- **AND** samples that transform behind the vehicle or to non-finite
  coordinates SHALL be dropped
- **AND** the aligned path SHALL be accepted only if at least
  `MIN_ALIGNED_SAMPLES` remain

#### Scenario: Disabled alignment is pass-through
- **WHEN** `REFERENCE_TIME_ALIGNMENT.ENABLED` is false
- **THEN** alignment SHALL return the input path unchanged
- **AND** facts SHALL report `enabled=false`, `valid=true`, and
  `reason=disabled`

#### Scenario: Alignment failure clears control reference facts
- **WHEN** alignment is enabled but reference time, age, pose delta, delta
  bounds, or aligned sample count fails validation
- **THEN** the control-loop copy SHALL clear reference usability,
  lateral-error comparison, tracking geometry, and reference-control readiness
- **AND** the failure reason SHALL be prefixed as
  `reference_time_alignment_*` for downstream diagnostics

### Requirement: ControlLoop Owns Control-Effective-Time Orchestration
`runtime::ControlLoop` SHALL be the only layer that combines perception,
`MotionHistory`, `ControlCommandHistory`, runtime parameters, pose-delta
estimation, reference time alignment, reference usability, tracking geometry,
reference-control readiness, yaw control, wheel mixing, and actuator output.

#### Scenario: BuildControlTimePerception composes estimator and aligner
- **WHEN** a control tick builds its control-time perception copy
- **THEN** it SHALL compute
  `control_effective_time_ms = now_ms + EFFECTIVE_DELAY_MS`
- **AND** it SHALL call the vehicle pose-delta estimator before calling
  reference time alignment
- **AND** it SHALL recompute usability, lateral-error comparison, tracking
  geometry, and readiness from the aligned path only after alignment succeeds

#### Scenario: Business modules stay unaware
- **WHEN** reviewers inspect module dependencies
- **THEN** the yaw controller SHALL still consume only
  `ReferenceTrackingGeometry`
- **AND** reference tracking geometry SHALL remain unaware of delay,
  `VehiclePoseDelta`, command history, IMU, encoder, actuator, and safety-gate
  state
- **AND** safety gate SHALL remain the owner of low voltage, stale perception,
  invalid perception, IMU, and encoder vetoes

### Requirement: Command History Is Separate From Motion History
The runtime SHALL maintain a `ControlCommandHistory` ring buffer separate from
`MotionHistory`. `MotionHistory` SHALL remain sensor facts only, while
`ControlCommandHistory` SHALL record requested/applied command facts for the
control tick.

#### Scenario: Command sample records applied facts
- **WHEN** a control tick completes actuator application and debug snapshot
  assembly
- **THEN** runtime SHALL push a command-history sample containing time, validity,
  actuator-applied state, diagnostics-only state, hold-disarmed state,
  emergency-stop state, raw and applied turn output, wheel targets, and PWM
  command values

#### Scenario: Reset clears command history
- **WHEN** control state is started, disarmed reset, timer-failure latched, or
  controller state reset
- **THEN** command history SHALL be cleared with other control-loop memory

### Requirement: Reference Time Alignment Parameters Are Explicit And Default Off
The runtime parameter surface SHALL expose explicit
`REFERENCE_TIME_ALIGNMENT` keys for enablement, maximum age, effective delay,
future prediction horizon, integration gap, minimum aligned samples, encoder
forward integration, encoder scale, wheel track, yaw source toggles, command
prediction, actuator yaw response, and maximum SE(2) deltas.

Defaults SHALL preserve current runtime behavior by keeping alignment disabled
and by keeping encoder forward integration, future prediction, and command-yaw
prediction disabled.

#### Scenario: Defaults are behavior preserving
- **WHEN** the default parameter file is loaded
- **THEN** `REFERENCE_TIME_ALIGNMENT.ENABLED` SHALL be false
- **AND** `USE_ENCODER_FORWARD`, `FUTURE_PREDICTION_ENABLED`, and
  `COMMAND_YAW_PREDICTION_ENABLED` SHALL be false
- **AND** unset board calibration values SHALL NOT affect runtime behavior

#### Scenario: Parameter validation rejects unsafe values
- **WHEN** `REFERENCE_TIME_ALIGNMENT` parameters are parsed
- **THEN** age, delay, prediction horizon, integration gap, aligned sample
  count, encoder scale, wheel track, yaw gain, actuator tau, and maximum delta
  values SHALL be range-validated
- **AND** formula code SHALL NOT silently clamp invalid semantic values to make
  an unsafe configuration appear valid

### Requirement: Alignment Facts Are Observable
The system SHALL expose control-side reference time-alignment facts in debug
observability so local and board evidence can explain measured and predicted
alignment behavior.

The observable facts SHALL include:

- `enabled`
- `valid`
- `reason`
- `age_ms`
- `reference_capture_time_ms`
- `control_time_ms`
- `control_effective_time_ms`
- `measured_until_ms`
- `predicted_ms`
- `delta_forward_m`
- `delta_lateral_m`
- `delta_yaw_rad`
- `measured_forward_mps`
- `measured_yaw_rate_radps`
- `predicted_forward_mps`
- `predicted_yaw_rate_radps`
- `used_encoder_forward`
- `used_imu_yaw`
- `used_wheel_yaw`
- `used_command_prediction`
- `input_sample_count`
- `aligned_sample_count`

#### Scenario: Debug snapshot mirrors alignment facts
- **WHEN** a control debug snapshot is built
- **THEN** the snapshot SHALL copy the full reference time-alignment fact set
  from the control-loop perception copy
- **AND** debug reporting SHALL include the expanded fields needed to diagnose
  capture time, effective time, measured motion, prediction, and source choices

### Requirement: Migration Compatibility Fields Are Removed Before Closure
The implementation SHALL remove temporary compatibility fields from active
contracts after the code slices are complete. Final active port types, debug
views, reporters, tests, parameter docs, and default config surfaces SHALL use
current fact names and SHALL NOT retain legacy aliases such as `delta_s_m`.

#### Scenario: Active contracts contain no legacy forward alias
- **WHEN** the change is ready for source-first verification
- **THEN** active implementation and documentation SHALL use
  `delta_forward_m` for forward displacement
- **AND** active code, tests, and docs SHALL NOT expose or assert `delta_s_m` as
  a compatibility field
- **AND** any retained external protocol compatibility would require an explicit
  requirement and documented owner before archive

### Requirement: Fail-Closed Reasons Are Stable
The system SHALL preserve stable reason strings for disabled, invalid,
stale, unavailable, prediction-disabled, horizon-exceeded, excessive-delta, and
insufficient-sample alignment outcomes so tests and telemetry can distinguish
why control-time reference facts are unavailable.

#### Scenario: Pose delta failure is carried into alignment reason
- **WHEN** the estimator reports `valid=false`
- **THEN** reference alignment facts SHALL report a reason beginning with
  `pose_delta_`
- **AND** the control-loop unavailable facts SHALL use a
  `reference_time_alignment_pose_delta_*` reason downstream
