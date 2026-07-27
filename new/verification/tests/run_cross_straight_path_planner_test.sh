#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
OUT_BIN="${SCRIPT_DIR}/cross_straight_path_planner_test"

g++ -std=c++17 -Wall -Wextra -Werror \
  -I"${REPO_ROOT}/new/code" \
  -I"${REPO_ROOT}/new/code/port" \
  "${REPO_ROOT}/new/verification/tests/cross_straight_path_planner_test.cpp" \
  "${REPO_ROOT}/new/code/vision/elements/cross_straight_path_planner.cpp" \
  -o "${OUT_BIN}"

"${OUT_BIN}"
