#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
OUT_BIN="${SCRIPT_DIR}/bev_boundary_row_test"

c++ -std=c++17 -Wall -Wextra -Werror -pthread \
  -I"${REPO_ROOT}/new/code" \
  -I"${REPO_ROOT}/new/code/port" \
  "${REPO_ROOT}/new/verification/tests/bev_boundary_row_test.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_projector.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/single_boundary_offset.cpp" \
  "${REPO_ROOT}/new/code/vision/bev/bev_simple_perception.cpp" \
  "${REPO_ROOT}/new/code/vision/image/luma_sampler.cpp" \
  -o "${OUT_BIN}"

"${OUT_BIN}"
