#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
source "${SCRIPT_DIR}/common_steering_test_build.sh"

OUT_DIR="${REPO_ROOT}/new/out/tests"
mkdir -p "${OUT_DIR}"

compile_test_binary \
  "${OUT_DIR}/shutdown_disable_result_test" \
  "${REPO_ROOT}/new/verification/tests/shutdown_disable_result_test.cpp" \
  "${REPO_ROOT}/new/code/runtime/lifecycle/shutdown.cpp"

"${OUT_DIR}/shutdown_disable_result_test"
