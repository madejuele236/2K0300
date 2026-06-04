#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_BIN="${SCRIPT_DIR}/runtime_parameter_validation_test"

source "${SCRIPT_DIR}/common_steering_test_build.sh"

compile_test_binary \
  "${OUT_BIN}" \
  "${REPO_ROOT}/new/verification/tests/runtime_parameter_validation_test.cpp"

"${OUT_BIN}"
