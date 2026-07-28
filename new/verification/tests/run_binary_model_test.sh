#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
OUT_BIN="${SCRIPT_DIR}/binary_model_test"

g++ -std=c++17 -Wall -Wextra -Werror \
  -I"${REPO_ROOT}/new/code" \
  "${REPO_ROOT}/new/verification/tests/binary_model_test.cpp" \
  "${REPO_ROOT}/new/code/vision/image/illumination_binary_model.cpp" \
  "${REPO_ROOT}/new/code/vision/image/luma_sampler.cpp" \
  -o "${OUT_BIN}"

"${OUT_BIN}"
