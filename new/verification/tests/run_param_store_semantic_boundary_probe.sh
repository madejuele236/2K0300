#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
OUT_BIN="${SCRIPT_DIR}/param_store_semantic_boundary_probe"

if ! pkg-config --exists opencv4; then
  echo "[ERROR] host OpenCV is required for the production MakeParamStore probe" >&2
  exit 2
fi

# shellcheck disable=SC2046
c++ -std=c++17 -Wall -Wextra -Werror -pthread \
  -I"${REPO_ROOT}/new/code" \
  -I"${REPO_ROOT}/new/code/port" \
  -I"${REPO_ROOT}/new/code/platform" \
  $(pkg-config --cflags opencv4) \
  "${SCRIPT_DIR}/param_store_semantic_boundary_probe.cpp" \
  "${REPO_ROOT}/new/code/platform/param_store.cpp" \
  -o "${OUT_BIN}" \
  $(pkg-config --libs opencv4)

"${OUT_BIN}"
