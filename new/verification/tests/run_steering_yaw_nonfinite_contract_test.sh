#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
OUT_DIR="${REPO_ROOT}/new/out/tests"
OUT_BIN="${OUT_DIR}/steering_yaw_nonfinite_contract_test"
mkdir -p "${OUT_DIR}"

if ! pkg-config --exists opencv4; then
  echo "[ERROR] host OpenCV is unavailable; cannot run yaw production-loader contract test" >&2
  exit 2
fi

c++ -std=c++17 -Wall -Wextra -Werror -pthread \
  -fsanitize=undefined,float-cast-overflow -fno-sanitize-recover=all \
  -I"${REPO_ROOT}/new/code" \
  -I"${REPO_ROOT}/new/code/port" \
  -I"${REPO_ROOT}/new/code/platform" \
  $(pkg-config --cflags opencv4) \
  "${REPO_ROOT}/new/verification/tests/steering_yaw_nonfinite_contract_test.cpp" \
  "${REPO_ROOT}/new/code/platform/param_store.cpp" \
  "${REPO_ROOT}/new/code/control/steering_yaw_controller.cpp" \
  -o "${OUT_BIN}" \
  $(pkg-config --libs opencv4)

"${OUT_BIN}" "${REPO_ROOT}/new/config/default_params.json"
