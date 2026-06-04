#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_BIN="${SCRIPT_DIR}/vehicle_pose_delta_estimator_test"

source "${SCRIPT_DIR}/common_steering_test_build.sh"

compile_test_binary \
  "${OUT_BIN}" \
  "${REPO_ROOT}/new/verification/tests/vehicle_pose_delta_estimator_test.cpp" \
  "${REPO_ROOT}/new/code/estimation/vehicle_pose_delta_estimator.cpp"

"${OUT_BIN}"
