#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
OUT_BIN="${REPO_ROOT}/.tmp/binary_model_vs_global_otsu_review"
RAW_DIRECTORY="${1:?usage: review RAW_DIRECTORY KNOWN_HARSH_RAW OUTPUT_DIRECTORY}"
KNOWN_HARSH_RAW="${2:?usage: review RAW_DIRECTORY KNOWN_HARSH_RAW OUTPUT_DIRECTORY}"
OUTPUT_DIRECTORY="${3:?usage: review RAW_DIRECTORY KNOWN_HARSH_RAW OUTPUT_DIRECTORY}"

mkdir -p "${REPO_ROOT}/.tmp"
c++ -std=c++17 -O3 -Wall -Wextra -Werror \
  -I"${REPO_ROOT}/new/code" \
  "${SCRIPT_DIR}/binary_model_vs_global_otsu_review.cpp" \
  "${REPO_ROOT}/new/code/vision/image/luma_sampler.cpp" \
  "${REPO_ROOT}/new/code/vision/image/illumination_binary_model.cpp" \
  -o "${OUT_BIN}"

"${OUT_BIN}" \
  "${RAW_DIRECTORY}" \
  "${KNOWN_HARSH_RAW}" \
  "${OUTPUT_DIRECTORY}" \
  "${@:4}"
