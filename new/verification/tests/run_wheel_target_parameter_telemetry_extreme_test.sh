#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
OUT_BIN="${SCRIPT_DIR}/wheel_target_parameter_telemetry_extreme_test"
FIXTURE="${SCRIPT_DIR}/wheel_target_parameter_telemetry_extreme_fixture.json"

cleanup() {
  rm -f "${FIXTURE}"
}
trap cleanup EXIT

if ! pkg-config --exists opencv4; then
  echo "[ERROR] host OpenCV is required for the production loader/protocol test" >&2
  exit 2
fi

# shellcheck disable=SC2046
c++ -std=c++17 -Wall -Wextra -Werror -pthread \
  -I"${REPO_ROOT}/new/code" \
  -I"${REPO_ROOT}/new/code/port" \
  $(pkg-config --cflags opencv4) \
  "${REPO_ROOT}/new/verification/tests/wheel_target_parameter_telemetry_extreme_test.cpp" \
  "${REPO_ROOT}/new/code/platform/param_store.cpp" \
  "${REPO_ROOT}/new/code/control/wheel_target_mixer.cpp" \
  "${REPO_ROOT}/new/code/control/wheel_pid.cpp" \
  "${REPO_ROOT}/new/code/control/motion_supervisor.cpp" \
  "${REPO_ROOT}/new/code/safety/control_apply_observation.cpp" \
  "${REPO_ROOT}/new/code/transport/assistant_protocol.cpp" \
  -o "${OUT_BIN}" \
  $(pkg-config --libs opencv4)

"${OUT_BIN}" "${REPO_ROOT}/new/config/default_params.json" "${FIXTURE}" | \
  node "${SCRIPT_DIR}/wheel_target_parameter_telemetry_json_check.mjs"
