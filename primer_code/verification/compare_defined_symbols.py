#!/usr/bin/env python3
"""Require the refactored executable to retain every baseline global symbol."""

from __future__ import annotations

import pathlib
import subprocess
import sys

EXPECTED_BASELINE_DEFINITIONS = 2143


def symbols(binary: pathlib.Path) -> set[str]:
    result = subprocess.run(
        ["rtk", "nm", "-g", "-C", "--defined-only", str(binary)],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    found: set[str] = set()
    for line in result.stdout.splitlines():
        fields = line.split(maxsplit=2)
        if len(fields) == 3:
            found.add(fields[2])
    return found


def main() -> int:
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} BASELINE_BINARY REFACTORED_BINARY", file=sys.stderr)
        return 2
    baseline_path = pathlib.Path(sys.argv[1]).resolve()
    refactored_path = pathlib.Path(sys.argv[2]).resolve()
    baseline = symbols(baseline_path)
    refactored = symbols(refactored_path)
    missing = sorted(baseline - refactored)
    print(f"baseline global definitions: {len(baseline)}")
    print(f"refactored global definitions: {len(refactored)}")
    if len(baseline) != EXPECTED_BASELINE_DEFINITIONS:
        print(
            "baseline symbol coverage changed: "
            f"expected {EXPECTED_BASELINE_DEFINITIONS}, found {len(baseline)}"
        )
        return 1
    if missing:
        print("missing baseline definitions:")
        for symbol in missing:
            print(f"  - {symbol}")
        return 1
    print("PASS: every baseline global definition remains available")
    return 0


if __name__ == "__main__":
    sys.exit(main())
