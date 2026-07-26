#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
source "${SCRIPT_DIR}/common_steering_test_build.sh"

OUT_DIR="${REPO_ROOT}/new/out/tests"
mkdir -p "${OUT_DIR}"

compile_test_binary \
  "${OUT_DIR}/bench_pwm_pulse_result_test" \
  -ffunction-sections \
  -fdata-sections \
  "${REPO_ROOT}/new/verification/tests/bench_pwm_pulse_result_test.cpp" \
  -Wl,--gc-sections

"${OUT_DIR}/bench_pwm_pulse_result_test"
