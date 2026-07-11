#!/usr/bin/env python3
"""Compare original and refactored function bodies without compiling.

The refactor is intentionally mechanical at the algorithm boundary.  This
check reads the committed original baseline and requires every discovered
function body to have a token-identical active definition after relocation.
Comments and whitespace are ignored; literals, operators, calls, ordering, and
all other C/C++ tokens remain significant.

This is supporting evidence, not a proof of hardware/runtime equivalence.
"""

from __future__ import annotations

import collections
import dataclasses
import hashlib
import pathlib
import re
import subprocess
import sys
from typing import Iterable


BASELINE = "70f3c71ea"
EXPECTED_BASELINE_FUNCTIONS = 102
ORIGINAL_SOURCES = (
    "primer_code/project/user/main.cpp",
    "primer_code/project/code/init.cpp",
    "primer_code/project/code/control.cpp",
    "primer_code/project/code/filt.cpp",
    "primer_code/project/code/flash.cpp",
    "primer_code/project/code/image.cpp",
    "primer_code/project/code/show.cpp",
    "primer_code/project/code/lq_ncnn.cpp",
    "primer_code/project/code/ww_transmission.cpp",
)
RENAMED_FUNCTIONS = {
    "main": "RunApplication",
    "LPF_1": "ApplyLowPass",
    "Dis_PID_Calculate": "Dis_PID_CalculateCore",
    "gyroOffset_init": "GyroOffset_InitCore",
    "ICM_getEulerianAngles": "ICM_GetEulerianAnglesCore",
    "huandao_yaw_correct": "HuandaoYawCorrectCore",
}
CONTROL_WORDS = {"if", "for", "while", "switch", "catch"}

# Six original entry points now delegate to one owner implementation.  These
# token rewrites compare that implementation with the original body while
# accounting only for explicit input/callback names introduced at the boundary.
BODY_REWRITES: dict[str, dict[str, tuple[tuple[tuple[str, ...], tuple[str, ...]], ...]]] = {
    "Speed_PID_Cal": {
        "original": ((('LPF_1',), ('LOW_PASS',)),),
        "current": ((('primer', '::', 'port', '::', 'ApplyLowPass'), ('LOW_PASS',)),),
    },
    "Dis_PID_Calculate": {
        "original": (
            (("encoder_L", ".", "D_speed"), ("LEFT_D_SPEED",)),
            (("encoder_R", ".", "D_speed"), ("RIGHT_D_SPEED",)),
        ),
        "current": (
            (("encoder_left_d_speed",), ("LEFT_D_SPEED",)),
            (("encoder_right_d_speed",), ("RIGHT_D_SPEED",)),
        ),
    },
    "gyroOffset_init": {
        "original": (
            (("imu660ra_get_acc",), ("READ_ACC",)),
            (("imu660ra_get_gyro",), ("READ_GYRO",)),
            (("system_delay_ms",), ("DELAY_MS",)),
        ),
        "current": (
            (("read_acc",), ("READ_ACC",)),
            (("read_gyro",), ("READ_GYRO",)),
            (("delay_ms",), ("DELAY_MS",)),
        ),
    },
    "ICM_getEulerianAngles": {
        "original": (
            (("imu660ra_get_acc",), ("READ_ACC",)),
            (("imu660ra_get_gyro",), ("READ_GYRO",)),
        ),
        "current": (
            (("read_acc",), ("READ_ACC",)),
            (("read_gyro",), ("READ_GYRO",)),
        ),
    },
    "huandao_yaw_correct": {
        "original": (
            (("icm_data", ".", "yaw"), ("CURRENT_YAW",)),
            (("Yaw_Huandao_err",), ("YAW_ERROR",)),
            (("Yaw_Huandao",), ("YAW_TARGET",)),
            (("yaw_correct",), ("YAW_CORRECT",)),
        ),
        "current": (
            (("*", "yaw_huandao_error"), ("YAW_ERROR",)),
            (("*", "yaw_correct_value"), ("YAW_CORRECT",)),
            (("yaw_huandao",), ("YAW_TARGET",)),
            (("current_yaw",), ("CURRENT_YAW",)),
        ),
    },
}

WRAPPER_BODIES = {
    "LPF_1": "{ primer::port::ApplyLowPass(hz, time, in, out); }",
    "Dis_PID_Calculate": "{ return Dis_PID_CalculateCore(pid, expect, feedback, encoder_L.D_speed, encoder_R.D_speed); }",
    "gyroOffset_init": "{ GyroOffset_InitCore(imu660ra_get_acc, imu660ra_get_gyro, DelayMilliseconds); }",
    "ICM_getEulerianAngles": "{ ICM_GetEulerianAnglesCore(imu660ra_get_gyro, imu660ra_get_acc); }",
    "huandao_yaw_correct": "{ HuandaoYawCorrectCore(Yaw_Huandao, icm_data.yaw, &yaw_correct, &Yaw_Huandao_err); }",
}


@dataclasses.dataclass(frozen=True)
class FunctionBody:
    name: str
    source: str
    line: int
    tokens: tuple[str, ...]

    @property
    def digest(self) -> str:
        payload = "\x1f".join(self.tokens).encode("utf-8")
        return hashlib.sha256(payload).hexdigest()[:16]


def repo_root() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[2]


def baseline_text(path: str) -> str:
    result = subprocess.run(
        ["rtk", "git", "show", f"{BASELINE}:{path}"],
        cwd=repo_root(),
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return result.stdout


def raw_string_end(text: str, start: int) -> int | None:
    """Return the end offset of a C++ raw string literal starting at start."""
    prefix = next(
        (candidate for candidate in ("u8R\"", "uR\"", "UR\"", "LR\"", "R\"")
         if text.startswith(candidate, start)),
        None,
    )
    if prefix is None:
        return None
    delimiter_start = start + len(prefix)
    open_paren = text.find("(", delimiter_start, delimiter_start + 17)
    if open_paren < 0:
        return None
    delimiter = text[delimiter_start:open_paren]
    if any(ch.isspace() or ch in "()\\" for ch in delimiter):
        return None
    marker = ")" + delimiter + '"'
    close = text.find(marker, open_paren + 1)
    return None if close < 0 else close + len(marker)


def mask_non_code(text: str) -> str:
    """Replace comments and literal contents while preserving offsets/braces."""
    chars = list(text)
    i = 0
    state = "code"
    quote = ""
    while i < len(chars):
        ch = chars[i]
        nxt = chars[i + 1] if i + 1 < len(chars) else ""
        if state == "code":
            raw_end = raw_string_end(text, i)
            if raw_end is not None:
                for index in range(i, raw_end):
                    if chars[index] != "\n":
                        chars[index] = " "
                i = raw_end
                continue
            if ch == "/" and nxt == "/":
                chars[i] = chars[i + 1] = " "
                i += 2
                state = "line_comment"
                continue
            if ch == "/" and nxt == "*":
                chars[i] = chars[i + 1] = " "
                i += 2
                state = "block_comment"
                continue
            if ch in {'"', "'"}:
                quote = ch
                chars[i] = " "
                i += 1
                state = "literal"
                continue
            i += 1
            continue
        if state == "line_comment":
            if ch == "\n":
                state = "code"
            else:
                chars[i] = " "
            i += 1
            continue
        if state == "block_comment":
            if ch == "*" and nxt == "/":
                chars[i] = chars[i + 1] = " "
                i += 2
                state = "code"
            else:
                if ch != "\n":
                    chars[i] = " "
                i += 1
            continue
        if state == "literal":
            if ch == "\\":
                chars[i] = " "
                if i + 1 < len(chars):
                    if chars[i + 1] != "\n":
                        chars[i + 1] = " "
                    i += 2
                else:
                    i += 1
                continue
            if ch == quote:
                chars[i] = " "
                i += 1
                state = "code"
            else:
                if ch != "\n":
                    chars[i] = " "
                i += 1
    return "".join(chars)


def cpp_tokens(text: str) -> tuple[str, ...]:
    tokens: list[str] = []
    i = 0
    multi_ops = (
        "<<=", ">>=", "<=>", "->*", "...", "::", "->", "++", "--",
        "<<", ">>", "<=", ">=", "==", "!=", "&&", "||", "+=", "-=",
        "*=", "/=", "%=", "&=", "|=", "^=", ".*", "##",
    )
    while i < len(text):
        ch = text[i]
        nxt = text[i + 1] if i + 1 < len(text) else ""
        if ch.isspace():
            i += 1
            continue
        if ch == "/" and nxt == "/":
            newline = text.find("\n", i + 2)
            i = len(text) if newline < 0 else newline + 1
            continue
        if ch == "/" and nxt == "*":
            end = text.find("*/", i + 2)
            i = len(text) if end < 0 else end + 2
            continue
        raw_end = raw_string_end(text, i)
        if raw_end is not None:
            tokens.append(text[i:raw_end])
            i = raw_end
            continue
        if ch in {'"', "'"}:
            quote = ch
            start = i
            i += 1
            while i < len(text):
                if text[i] == "\\":
                    i += 2
                elif text[i] == quote:
                    i += 1
                    break
                else:
                    i += 1
            tokens.append(text[start:i])
            continue
        if ch.isalpha() or ch == "_":
            start = i
            i += 1
            while i < len(text) and (text[i].isalnum() or text[i] == "_"):
                i += 1
            tokens.append(text[start:i])
            continue
        if ch.isdigit() or (ch == "." and nxt.isdigit()):
            start = i
            i += 1
            while i < len(text):
                if text[i].isalnum() or text[i] in "._":
                    i += 1
                    continue
                if text[i] in "+-" and i > start and text[i - 1] in "eEpP":
                    i += 1
                    continue
                break
            tokens.append(text[start:i])
            continue
        op = next((candidate for candidate in multi_ops if text.startswith(candidate, i)), None)
        if op:
            tokens.append(op)
            i += len(op)
        else:
            tokens.append(ch)
            i += 1
    return tuple(tokens)


SIGNATURE = re.compile(
    r"(?m)^[ \t]*(?:extern[ \t]+\"C\"[ \t]+)?"
    r"(?P<prefix>[^#;{}\n]*?)"
    r"(?P<name>(?:[A-Za-z_]\w*::)*~?[A-Za-z_]\w*)"
    r"[ \t]*\([^;{}]*\)\s*(?:const\s*)?(?:noexcept\s*)?"
    r"(?:\:\s*[^;{}]*)?\{"
)


def function_bodies(text: str, source: str) -> list[FunctionBody]:
    masked = mask_non_code(text)
    found: list[FunctionBody] = []
    for match in SIGNATURE.finditer(masked):
        name = match.group("name")
        if name in CONTROL_WORDS:
            continue
        open_brace = match.end() - 1
        depth = 0
        close_brace = None
        for index in range(open_brace, len(masked)):
            if masked[index] == "{":
                depth += 1
            elif masked[index] == "}":
                depth -= 1
                if depth == 0:
                    close_brace = index
                    break
        if close_brace is None:
            raise RuntimeError(f"unbalanced function body in {source}:{text.count(chr(10), 0, open_brace) + 1}")
        body = text[open_brace : close_brace + 1]
        found.append(
            FunctionBody(
                name=name,
                source=source,
                line=text.count("\n", 0, match.start()) + 1,
                tokens=cpp_tokens(body),
            )
        )
    return found


def active_sources() -> Iterable[pathlib.Path]:
    root = repo_root() / "primer_code" / "project"
    yield root / "user" / "main.cpp"
    for path in sorted((root / "code").rglob("*.cpp")):
        yield path
    yield root / "code" / "port" / "low_pass_filter.hpp"


def rewrite_tokens(
    tokens: tuple[str, ...],
    rewrites: tuple[tuple[tuple[str, ...], tuple[str, ...]], ...],
) -> tuple[str, ...]:
    result = list(tokens)
    for old, new in rewrites:
        rewritten: list[str] = []
        index = 0
        while index < len(result):
            if tuple(result[index : index + len(old)]) == old:
                rewritten.extend(new)
                index += len(old)
            else:
                rewritten.append(result[index])
                index += 1
        result = rewritten
    return tuple(result)


def main() -> int:
    originals: list[FunctionBody] = []
    for path in ORIGINAL_SOURCES:
        originals.extend(function_bodies(baseline_text(path), f"{BASELINE}:{path}"))

    current: list[FunctionBody] = []
    root = repo_root()
    for path in active_sources():
        current.extend(function_bodies(path.read_text(encoding="utf-8"), str(path.relative_to(root))))

    by_name: dict[str, list[FunctionBody]] = collections.defaultdict(list)
    for function in current:
        by_name[function.name].append(function)

    failures: list[str] = []
    if len(originals) != EXPECTED_BASELINE_FUNCTIONS:
        failures.append(
            f"baseline parser coverage changed: expected "
            f"{EXPECTED_BASELINE_FUNCTIONS}, found {len(originals)}"
        )
    matched = 0
    for original in originals:
        target_name = RENAMED_FUNCTIONS.get(original.name, original.name)
        candidates = by_name.get(target_name, [])
        rules = BODY_REWRITES.get(original.name, {})
        expected_tokens = rewrite_tokens(original.tokens, rules.get("original", ()))
        exact = next(
            (
                candidate
                for candidate in candidates
                if rewrite_tokens(candidate.tokens, rules.get("current", ())) == expected_tokens
            ),
            None,
        )
        if exact:
            matched += 1
            continue
        candidate_summary = ", ".join(
            f"{candidate.source}:{candidate.line} sha={candidate.digest}"
            for candidate in candidates
        ) or "no active definition"
        failures.append(
            f"{original.name} from {original.source}:{original.line} sha={original.digest}: {candidate_summary}"
        )

    wrapper_failures: list[str] = []
    for name, expected_body in WRAPPER_BODIES.items():
        expected = cpp_tokens(expected_body)
        wrappers = by_name.get(name, [])
        if not any(wrapper.tokens == expected for wrapper in wrappers):
            actual = ", ".join(
                f"{wrapper.source}:{wrapper.line} sha={wrapper.digest}" for wrapper in wrappers
            ) or "no active definition"
            wrapper_failures.append(f"{name}: {actual}")

    print(f"baseline functions discovered: {len(originals)}")
    print(f"token-identical active bodies: {matched}")
    if failures:
        print("mismatches:")
        for failure in failures:
            print(f"  - {failure}")
    if wrapper_failures:
        print("delegation wrapper mismatches:")
        for failure in wrapper_failures:
            print(f"  - {failure}")
    if failures or wrapper_failures:
        return 1
    print("PASS: every original function is token-identical or mechanically delegated to one token-equivalent owner")
    return 0


if __name__ == "__main__":
    sys.exit(main())
