#!/usr/bin/env python3
"""Prove that init.cpp's global construction block stayed in one ordered TU."""

from __future__ import annotations

import hashlib
import pathlib
import subprocess
import sys

from compare_function_bodies import cpp_tokens


BASELINE = "70f3c71ea"
BASELINE_PATH = "primer_code/project/code/init.cpp"
ROOT = pathlib.Path(__file__).resolve().parents[2]
CURRENT = ROOT / "primer_code" / "project" / "code" / "runtime" / "hardware_composition.cpp"
START = "#define KEY_1_PATH"
END = "volatile int16 dl1x_distance_raw = 0;"
IGNORED_DECLARATION = ("extern", "lq_camera_ex", "cam", ";")
EXPECTED_GLOBAL_ORDER_TOKENS = 263


def baseline_text() -> str:
    result = subprocess.run(
        ["rtk", "git", "show", f"{BASELINE}:{BASELINE_PATH}"],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return result.stdout


def block(text: str, source: str) -> tuple[str, ...]:
    start = text.find(START)
    end = text.find(END, start)
    if start < 0 or end < 0:
        raise RuntimeError(f"global construction markers missing in {source}")
    tokens = list(cpp_tokens(text[start : end + len(END)]))
    index = 0
    while index <= len(tokens) - len(IGNORED_DECLARATION):
        if tuple(tokens[index : index + len(IGNORED_DECLARATION)]) == IGNORED_DECLARATION:
            del tokens[index : index + len(IGNORED_DECLARATION)]
        else:
            index += 1
    return tuple(tokens)


def digest(tokens: tuple[str, ...]) -> str:
    return hashlib.sha256("\x1f".join(tokens).encode("utf-8")).hexdigest()


def main() -> int:
    if not CURRENT.exists():
        print(f"missing composition owner: {CURRENT.relative_to(ROOT)}")
        return 1
    expected = block(baseline_text(), f"{BASELINE}:{BASELINE_PATH}")
    actual = block(CURRENT.read_text(encoding="utf-8"), str(CURRENT.relative_to(ROOT)))
    print(f"baseline global-order tokens: {len(expected)} sha256={digest(expected)}")
    print(f"current global-order tokens: {len(actual)} sha256={digest(actual)}")
    if len(expected) != EXPECTED_GLOBAL_ORDER_TOKENS:
        print(
            "baseline parser coverage changed: "
            f"expected {EXPECTED_GLOBAL_ORDER_TOKENS}, found {len(expected)}"
        )
        return 1
    if actual != expected:
        mismatch = next(
            (index for index, pair in enumerate(zip(expected, actual)) if pair[0] != pair[1]),
            min(len(expected), len(actual)),
        )
        print(f"global construction order differs at token {mismatch}")
        print(f"baseline tail: {expected[mismatch:mismatch + 12]}")
        print(f"current tail: {actual[mismatch:mismatch + 12]}")
        return 1
    print("PASS: original init global definitions remain in one TU with identical order and initializers")
    return 0


if __name__ == "__main__":
    sys.exit(main())
