#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
OUT_BIN="${SCRIPT_DIR}/binary_model_replay_probe"
REFERENCE="${REPO_ROOT}/new/verification/binary-algorithm-python-review-20260728/compare_binary_algorithms.py"
RAW_DIRECTORY="${1:?usage: run_binary_model_python_parity_test.sh RAW_DIRECTORY}"

c++ -std=c++17 -O3 -Wall -Wextra -Werror \
  -I"${REPO_ROOT}/new/code" \
  "${SCRIPT_DIR}/binary_model_replay_probe.cpp" \
  "${REPO_ROOT}/new/code/vision/image/luma_sampler.cpp" \
  "${REPO_ROOT}/new/code/vision/image/illumination_binary_model.cpp" \
  -o "${OUT_BIN}"

python3 "${SCRIPT_DIR}/binary_model_python_parity_test.py" \
  "${OUT_BIN}" "${REFERENCE}" "${RAW_DIRECTORY}"
