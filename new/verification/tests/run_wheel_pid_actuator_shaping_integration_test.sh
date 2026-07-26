#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
OUT_BIN="${SCRIPT_DIR}/wheel_pid_actuator_shaping_integration_test"

c++ -std=c++17 -Wall -Wextra -Werror -pthread \
  -I"${REPO_ROOT}/new/code" \
  "${REPO_ROOT}/new/verification/tests/wheel_pid_actuator_shaping_integration_test.cpp" \
  "${REPO_ROOT}/new/code/control/wheel_pid.cpp" \
  "${REPO_ROOT}/new/code/control/actuator_command_builder.cpp" \
  -o "${OUT_BIN}"

"${OUT_BIN}"
