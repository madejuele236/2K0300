#!/bin/bash
set -euo pipefail

SCRIPT_PATH="$(rtk realpath "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(rtk dirname "${SCRIPT_PATH}")"
REPO_ROOT="$(rtk realpath "${SCRIPT_DIR}/../../..")"
OUT_BIN="${SCRIPT_DIR}/wheel_pid_test"

rtk c++ -std=c++17 -Wall -Wextra -Werror -pthread \
  -I"${REPO_ROOT}/new/code" \
  "${REPO_ROOT}/new/verification/tests/wheel_pid_test.cpp" \
  "${REPO_ROOT}/new/code/control/wheel_pid.cpp" \
  -o "${OUT_BIN}"

rtk "${OUT_BIN}"
