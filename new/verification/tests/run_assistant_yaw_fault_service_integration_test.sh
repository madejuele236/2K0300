#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
source "${SCRIPT_DIR}/common_steering_test_build.sh"

OUT_DIR="${REPO_ROOT}/new/out/tests"
OPENCV_ROOT="${OPENCV_ROOT:-/opt/ls_2k0300_env/opencv_4_10_build}"
ARTIFACT_DIR="$(mktemp -d "${TMPDIR:-/tmp}/ls2k-assistant-yaw.XXXXXX")"
trap 'rm -rf "${ARTIFACT_DIR}"' EXIT
mkdir -p "${OUT_DIR}"

compile_test_binary \
  "${OUT_DIR}/assistant_yaw_fault_service_integration_test" \
  -DLS2K_PERF_ENABLED=0 \
  -I"${OPENCV_ROOT}/include/opencv4" \
  "${REPO_ROOT}/new/verification/tests/assistant_yaw_fault_service_integration_test.cpp" \
  "${REPO_ROOT}/new/code/runtime/loops/control_loop.cpp" \
  "${REPO_ROOT}/new/code/runtime/services/assistant_service.cpp" \
  "${REPO_ROOT}/new/code/transport/assistant_link.cpp" \
  "${REPO_ROOT}/new/code/transport/assistant_protocol.cpp" \
  "${REPO_ROOT}/new/code/control/steering_yaw_controller.cpp" \
  "${REPO_ROOT}/new/code/control/actuator_command_builder.cpp" \
  "${REPO_ROOT}/new/code/control/wheel_target_mixer.cpp" \
  "${REPO_ROOT}/new/code/control/wheel_pid.cpp" \
  "${REPO_ROOT}/new/code/control/motion_supervisor.cpp" \
  "${REPO_ROOT}/new/code/control/tuning_state.cpp" \
  "${REPO_ROOT}/new/code/safety/control_gate.cpp" \
  "${REPO_ROOT}/new/code/safety/control_apply_observation.cpp" \
  "${REPO_ROOT}/new/code/estimation/vehicle_pose_delta_estimator.cpp" \
  "${REPO_ROOT}/new/code/reference/reference_time_alignment.cpp" \
  "${REPO_ROOT}/new/code/reference/reference_usability.cpp" \
  "${REPO_ROOT}/new/code/reference/reference_lateral_error.cpp" \
  "${REPO_ROOT}/new/code/reference/reference_tracking_geometry.cpp" \
  "${REPO_ROOT}/new/code/reference/reference_control_readiness.cpp" \
  "${REPO_ROOT}/new/code/observability/control_debug_reporter.cpp" \
  -L"${OPENCV_ROOT}/lib" \
  -Wl,-rpath,"${OPENCV_ROOT}/lib" \
  -lopencv_core

LS2K_STRICT_JSON_ARTIFACT_DIR="${ARTIFACT_DIR}" \
  "${OUT_DIR}/assistant_yaw_fault_service_integration_test"
node "${SCRIPT_DIR}/strict_json_protocol_output_check.mjs" assistant-yaw-fault \
  "${ARTIFACT_DIR}/assistant_yaw_fault_first.json" \
  "${ARTIFACT_DIR}/assistant_yaw_fault_later.json"
