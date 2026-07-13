#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(rtk realpath "$(rtk dirname "${BASH_SOURCE[0]}")")"
REPO_ROOT="$(rtk realpath "${SCRIPT_DIR}/../../..")"
OUT_BIN="${SCRIPT_DIR}/assistant_bridge_contract_test"

rtk c++ -std=c++17 -Wall -Wextra -Werror -pthread \
  -I"${REPO_ROOT}/new/code" \
  "${REPO_ROOT}/new/verification/tests/assistant_bridge_contract_test.cpp" \
  "${REPO_ROOT}/new/code/platform/true_ls2k0300/assistant_bridge.cpp" \
  -Wl,--wrap=send \
  -o "${OUT_BIN}"

rtk timeout 10s rtk "${OUT_BIN}"
