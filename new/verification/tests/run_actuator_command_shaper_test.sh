#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
OUT_BIN="${SCRIPT_DIR}/actuator_command_shaper_test"

c++ -std=c++17 -Wall -Wextra -Werror -pthread \
  -I"${REPO_ROOT}/new/code" \
  "${REPO_ROOT}/new/verification/tests/actuator_command_shaper_test.cpp" \
  "${REPO_ROOT}/new/code/control/actuator_command_builder.cpp" \
  -o "${OUT_BIN}"

"${OUT_BIN}"
