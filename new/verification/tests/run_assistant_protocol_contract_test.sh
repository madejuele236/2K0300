#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(rtk realpath "$(rtk dirname "${BASH_SOURCE[0]}")")"
REPO_ROOT="$(rtk realpath "${SCRIPT_DIR}/../../..")"
OUT_BIN="${SCRIPT_DIR}/assistant_protocol_contract_test"
OPENCV_ROOT="${OPENCV_ROOT:-/opt/ls_2k0300_env/opencv_4_10_build}"

rtk c++ -std=c++17 -Wall -Wextra -Werror \
  -I"${REPO_ROOT}/new/code" \
  -I"${OPENCV_ROOT}/include/opencv4" \
  "${REPO_ROOT}/new/verification/tests/assistant_protocol_contract_test.cpp" \
  "${REPO_ROOT}/new/code/transport/assistant_protocol.cpp" \
  -L"${OPENCV_ROOT}/lib" \
  -Wl,-rpath,"${OPENCV_ROOT}/lib" \
  -lopencv_core \
  -o "${OUT_BIN}"

rtk timeout 10s rtk "${OUT_BIN}"
