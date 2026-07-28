#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
OUT_BIN="/tmp/zebra_controlled_stop_test"

g++ -std=c++17 -O2 -Wall -Wextra -Werror \
  -I"${REPO_ROOT}/new/code" \
  "${SCRIPT_DIR}/zebra_controlled_stop_test.cpp" \
  "${REPO_ROOT}/new/code/control/zebra_controlled_stop.cpp" \
  "${REPO_ROOT}/new/code/control/motion_supervisor.cpp" \
  -o "${OUT_BIN}"

"${OUT_BIN}"
