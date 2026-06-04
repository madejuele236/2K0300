# Implementation Evidence: implement-time-aligned-path-tracking

## Docs-First Gate

- `docs-first-findings.json`: verdict `pass`, findings `[]`
- `docs-first-verifier-evidence.json`: `coverage_status=complete`, `exhaustive=true`
- `rtk openspec validate --all`: passed after artifact repair

## Focused Tests

- `rtk bash new/verification/tests/run_runtime_parameter_defaults_test.sh`: passed
- `rtk bash new/verification/tests/run_vehicle_pose_delta_estimator_test.sh`: passed
- `rtk bash new/verification/tests/run_reference_time_alignment_test.sh`: passed
- `rtk bash new/verification/tests/run_assistant_telemetry_selftest.sh`: passed
- `rtk bash new/verification/tests/run_steering_media_selftest.sh`: passed

## Required Regression Tests

- `rtk bash new/verification/tests/run_power_adapter_threshold_test.sh`: passed
- `rtk bash new/verification/tests/run_startup_low_voltage_order_test.sh`: passed
- `rtk bash new/verification/tests/run_bev_simple_perception_test.sh`: passed
- `rtk bash new/verification/tests/run_visual_reference_orchestration_test.sh`: passed
- `rtk bash new/verification/tests/run_reference_usability_lateral_error_test.sh`: passed
- `rtk bash new/verification/tests/run_reference_tracking_geometry_test.sh`: passed
- `rtk bash new/verification/tests/run_bev_simple_residual_check.sh`: passed
- `rtk bash new/verification/tests/run_perf_counter_test.sh`: passed

## Build And Static Checks

- `rtk env SKIP_UPLOAD=1 new/user/build.sh`: passed
- `rtk git diff --check`: passed
- `rtk openspec validate --all`: passed
- `rtk codegraph sync`: passed, already up to date

## Compatibility Cleanup

- Active-surface search command:
  `rtk rg -n "delta_s_m" new/code new/config new/user new/verification/tests new/docs/time_aligned_path_tracking_implementation.md`
- Result: no matches

## Board Smoke

- Board upload/start was not run because this implementation turn has not been
  explicitly authorized to upload or start runtime on the board.
- The required local safety gate was run instead:
  `rtk env SKIP_UPLOAD=1 new/user/build.sh`: passed.

## Source-First Repair Evidence

- Source-first verifier initially returned `block` because steering-media live
  image frames exposed config parameters but not per-frame
  `reference_time_alignment` facts.
- Repair added live `reference_time_alignment` fields to
  `SteeringMediaSnapshotView`, copied them from `SteeringDebugSnapshot`, encoded
  them in image-frame JSON, and added `steering_media_selftest` assertions for
  the live frame fields.
- Post-repair checks:
  - `rtk bash new/verification/tests/run_reference_time_alignment_test.sh`: passed
  - `rtk bash new/verification/tests/run_vehicle_pose_delta_estimator_test.sh`: passed
  - `rtk bash new/verification/tests/run_steering_media_selftest.sh`: passed
  - `rtk git diff --check`: passed
  - `rtk env SKIP_UPLOAD=1 new/user/build.sh`: passed

## Source-First Pass2 Repair Evidence

- Second source-first verifier returned `block` for two control-loop boundary
  issues:
  - enabled-but-unavailable reference time alignment did not clear downstream
    control-copy reference facts
  - `ControlLoop::Start` did not clear `ControlCommandHistory`
- Repair updated `BuildControlTimePerception` so an enabled alignment path with
  unpublished or stale perception clears `reference_usability`,
  `reference_lateral_error`, `reference_tracking_geometry`, and
  `reference_control` with
  `reference_time_alignment_perception_unavailable`.
- Repair updated `ControlLoop::Start` to clear `state_.command_history`
  alongside the existing startup observation/debug/control-memory reset.
- Post-repair checks:
  - `rtk bash new/verification/tests/run_reference_time_alignment_test.sh`: passed
  - `rtk bash new/verification/tests/run_vehicle_pose_delta_estimator_test.sh`: passed
  - `rtk bash new/verification/tests/run_steering_media_selftest.sh`: passed
  - `rtk bash new/verification/tests/run_runtime_parameter_defaults_test.sh`: passed
  - `rtk git diff --check`: passed
  - `rtk rg -n "delta_s_m" new/code new/config new/user new/verification/tests new/docs/time_aligned_path_tracking_implementation.md`:
    no matches
  - `rtk openspec validate --all`: passed
  - `rtk env SKIP_UPLOAD=1 new/user/build.sh`: passed

## Final Source-First And Archive Evidence

- `source-first-pass2-findings.json`: verdict `pass`, findings `[]`
- `source-first-pass3-findings.json`: verdict `pass`, findings `[]`
- `source-first-pass3-verifier-evidence.json`: `coverage_status=complete`,
  `exhaustive=true`
- Specs synced to `openspec/specs/time-aligned-path-tracking/spec.md`.
- Change archived to
  `openspec/changes/archive/2026-06-04-implement-time-aligned-path-tracking/`.
- Post-archive checks:
  - `rtk openspec validate --all`: passed
  - `rtk git diff --check`: passed
  - `rtk rg -n "delta_s_m" new/code new/config new/user new/verification/tests new/docs/time_aligned_path_tracking_implementation.md`:
    no matches
  - `rtk bash new/verification/tests/run_reference_time_alignment_test.sh`: passed
  - `rtk bash new/verification/tests/run_vehicle_pose_delta_estimator_test.sh`:
    passed
  - `rtk bash new/verification/tests/run_runtime_parameter_defaults_test.sh`:
    passed
  - `rtk bash new/verification/tests/run_steering_media_selftest.sh`: passed
  - `rtk env SKIP_UPLOAD=1 new/user/build.sh`: passed
