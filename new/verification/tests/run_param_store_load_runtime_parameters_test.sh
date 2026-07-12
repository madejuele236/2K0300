#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
OUT_BIN="${SCRIPT_DIR}/param_store_load_runtime_parameters_test"

compile_with_flags() {
  local cflags="${1:-}"
  local ldflags="${2:-}"
  # shellcheck disable=SC2086
  c++ -std=c++17 -Wall -Wextra -Werror -pthread \
    -I"${REPO_ROOT}/new/code" \
    -I"${REPO_ROOT}/new/code/port" \
    -I"${REPO_ROOT}/new/code/platform" \
    ${cflags} \
    "${REPO_ROOT}/new/verification/tests/param_store_load_runtime_parameters_test.cpp" \
    "${REPO_ROOT}/new/code/platform/param_store.cpp" \
    -o "${OUT_BIN}" \
    ${ldflags}
}

if pkg-config --exists opencv4; then
  compile_with_flags "$(pkg-config --cflags opencv4)" "$(pkg-config --libs opencv4)"
  "${OUT_BIN}" "${REPO_ROOT}/new/config/default_params.json"
  exit 0
fi

if [[ -n "${OPENCV_ROOT:-}" ]] && [[ -d "${OPENCV_ROOT}/include/opencv4" ]]; then
  compile_with_flags \
    "-I${OPENCV_ROOT}/include/opencv4" \
    "-L${OPENCV_ROOT}/lib -Wl,-rpath,${OPENCV_ROOT}/lib -lopencv_core"
  "${OUT_BIN}" "${REPO_ROOT}/new/config/default_params.json"
  exit 0
fi

cat >&2 <<'EOF'
[ERROR] host OpenCV is unavailable; cannot run local param-store load test.
[INFO] Set OPENCV_ROOT to a host-architecture OpenCV build or run the explicit
[INFO] board wrapper after board access is allowed:
[INFO]   new/verification/tests/run_param_store_load_runtime_parameters_board_test.sh
EOF
exit 2
