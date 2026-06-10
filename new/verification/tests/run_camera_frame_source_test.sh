#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
OUT_BIN="${SCRIPT_DIR}/camera_frame_source_test"
CXX_BIN="${CXX:-c++}"

"${CXX_BIN}" \
  -std=c++17 \
  -Wall \
  -Wextra \
  -Werror \
  -pthread \
  -I"${REPO_ROOT}/new/code" \
  -I"${REPO_ROOT}/new/code/port" \
  -I"${REPO_ROOT}/new/code/platform" \
  "${REPO_ROOT}/new/verification/tests/camera_frame_source_test.cpp" \
  -o "${OUT_BIN}"

"${OUT_BIN}"
