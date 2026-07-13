#!/bin/bash
set -euo pipefail

SCRIPT_PATH="$(rtk realpath "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(rtk dirname "${SCRIPT_PATH}")"
REPO_ROOT="$(rtk realpath "${SCRIPT_DIR}/../../..")"
OUT_BIN="${SCRIPT_DIR}/imu_device_test.bin"

rtk g++ -std=c++17 -Wall -Wextra -Werror \
  -I"${REPO_ROOT}/new/code" \
  "${REPO_ROOT}/new/verification/tests/imu_device_test.cpp" \
  "${REPO_ROOT}/new/code/platform/true_ls2k0300/imu_bridge.cpp" \
  "${REPO_ROOT}/new/code/platform/linux/linux_io.cpp" \
  -o "${OUT_BIN}"

rtk "${OUT_BIN}"
rtk echo "imu_device_test passed"
