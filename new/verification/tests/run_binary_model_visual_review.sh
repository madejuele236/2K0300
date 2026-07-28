#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
OUT_BIN="${REPO_ROOT}/.tmp/binary_model_visual_review"
RAW_DIRECTORY="${1:?usage: run_binary_model_visual_review.sh RAW_DIRECTORY OUTPUT_DIRECTORY}"
OUTPUT_DIRECTORY="${2:?usage: run_binary_model_visual_review.sh RAW_DIRECTORY OUTPUT_DIRECTORY}"

mkdir -p "${REPO_ROOT}/.tmp"
c++ -std=c++17 -O3 -Wall -Wextra -Werror \
  -I"${REPO_ROOT}/new/code" \
  "${SCRIPT_DIR}/binary_model_visual_review.cpp" \
  "${REPO_ROOT}/new/code/vision/image/luma_sampler.cpp" \
  "${REPO_ROOT}/new/code/vision/image/illumination_binary_model.cpp" \
  -o "${OUT_BIN}"

"${OUT_BIN}" "${RAW_DIRECTORY}" "${OUTPUT_DIRECTORY}"
