#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(rtk realpath "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(rtk dirname "${SCRIPT_PATH}")"
REPO_ROOT="$(rtk realpath "${SCRIPT_DIR}/../../..")"
CXX="${CXX:-g++}"
OUT="${TMPDIR:-/tmp}/ls2k_motor_device_test"

rtk "${CXX}" -std=c++17 -Wall -Wextra -Werror -I"${REPO_ROOT}/new/code" \
  "${REPO_ROOT}/new/verification/tests/motor_device_test.cpp" \
  "${REPO_ROOT}/new/code/platform/true_ls2k0300/motor_bridge.cpp" \
  "${REPO_ROOT}/new/code/platform/linux/linux_io.cpp" \
  -o "${OUT}"
rtk "${OUT}"
