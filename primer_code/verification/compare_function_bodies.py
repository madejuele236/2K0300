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

# Runtime/application was split into these deliberately short stages.  The
# outer entries and every stage must retain this exact call/control skeleton;
# only these helpers may be expanded into a baseline body.
RUNTIME_WRAPPER_BODIES = {
    "pit_callback": "{ SamplePeriodicInputs(); ApplyActiveDriveCycle(); ApplyStoppedDriveCycle(); CompletePeriodicCycle(); }",
    "RunApplication": "{ InitializeApplication(); while(1) { RunForegroundCycle(); } }",
    "SamplePeriodicInputs": "{ ICM_getEulerianAngles(); if(primer::runtime::CycleCounter()%10==0) primer::platform::MutableDistanceRawSignal() = primer::platform::ReadDistanceSensor(); Encoder_update(); distance_judge(); }",
    "InitializeApplication": "{ init(); pid_init(); image_init(); primer::platform::StartPeriodicTimer(5, pit_callback); Param_Init(); }",
    "RunForegroundCycle": "{ UpdateForegroundBeeper(); UpdateForegroundFrameTiming(); RunForegroundPresentation(); ImageDeal(); }",
}

RUNTIME_HELPERS = frozenset({
    "SamplePeriodicInputs", "ApplyActiveDriveCycle", "ApplyStoppedDriveCycle",
    "CompletePeriodicCycle", "InitializeApplication", "UpdateForegroundBeeper",
    "UpdateForegroundFrameTiming", "RunForegroundPresentation", "RunForegroundCycle",
})


def owner_rule(current: str, baseline: str, count: int):
    return (current, baseline, count)


# Counts are part of the contract: a missing use or an extra owner-API use is
# rejected rather than silently passing through a broad spelling substitution.
RUNTIME_OWNER_RULES = (
    owner_rule("primer::runtime::CycleCounter()", "it_time", 3),
    owner_rule("primer::runtime::RunFlag()", "run_flag", 4),
    owner_rule("primer::platform::MutableDistanceRawSignal()", "dl1x_distance_raw", 2),
    owner_rule("primer::platform::ReadDistanceSensor()", "dl1x_dev.get_distance()", 1),
    owner_rule("primer::platform::SetEscDuty(500)", "esc_pwm.set_duty(500)", 1),
    owner_rule("primer::platform::StartPeriodicTimer(5, pit_callback)", "pit_timer.init_ms(5, pit_callback)", 1),
    owner_rule("primer::platform::SetBeeper(true)", "beep.set_level(1)", 1),
    owner_rule("primer::platform::SetBeeper(false)", "beep.set_level(0)", 1),
    owner_rule("primer::estimation::CurrentImuEstimate().yaw", "icm_data.yaw", 1),
    owner_rule("primer::vision::SetVisionDynamicForward(41-primer::control::AccessRuntimeControlState().master_speed/20)", "forward1=41-Master_Speed/20", 1),
    owner_rule("primer::vision::SetVisionDynamicForward(30-primer::control::AccessRuntimeControlState().master_speed/60)", "forward1=30-Master_Speed/60", 1),
    *(owner_rule(f"primer::control::AccessRuntimeControlState().{field}", legacy, count) for field, legacy, count in (
        ("distance_controller", "Dis_1", 3), ("distance_output", "Dis_Out", 7),
        ("distance_speed", "Dis_Speed", 1), ("image_output", "Image_out", 1),
        ("left_pwm", "PWM_L", 2), ("left_velocity_controller", "Velocity_L", 1),
        ("left_velocity_target", "v_left_target", 3), ("master_speed", "Master_Speed", 1),
        ("right_pwm", "PWM_R", 2), ("right_velocity_controller", "Velocity_R", 1),
        ("right_velocity_target", "v_right_target", 3), ("speed_goal", "speed_goal", 8),
    )),
    *(owner_rule(f"primer::vision::ObserveVisionControlLiveView().{field}", legacy, count) for field, legacy, count in (
        ("direction_error", "Dir_err", 1), ("elements.Huandao_L", "Flag.Huandao_L", 1),
        ("elements.Huandao_R", "Flag.Huandao_R", 1), ("elements.Zebra_cross", "Flag.Zebra_cross", 3),
        ("elements.picture", "Flag.picture", 3), ("elements.ramp", "Flag.ramp", 3),
        ("elements.small_rock", "Flag.small_rock", 1), ("image.top", "imgInfo.top", 1),
        ("jump_point", "jump_point", 1), ("left_high_corner.row", "L_h_guai.row", 1),
        ("right_high_corner.row", "R_h_guai.row", 1), ("row_distance", "real_distance", 1),
        ("steering_difference_error", "D_ERR", 2),
    )),
    *(owner_rule(f"primer::platform::{side}Encoder().{field}", legacy, count) for side, field, legacy, count in (
        ("Left", "D_speed", "encoder_L.D_speed", 2), ("Left", "count_now", "encoder_L.count_now", 1),
        ("Left", "speed", "encoder_L.speed", 1), ("Right", "D_speed", "encoder_R.D_speed", 2),
        ("Right", "count_now", "encoder_R.count_now", 1), ("Right", "speed", "encoder_R.speed", 1),
    )),
)

PRESENTATION_WRAPPER_BODIES = {
    "key_scan": "{ presentation_scan_input(); }",
    "oled_show": "{ presentation_render_oled_pages(); }",
    "presentation_render_oled_pages": "{ primer::presentation::RenderRedDebugPage(); primer::presentation::RenderPage3(); primer::presentation::RenderPage2(); primer::presentation::RenderPage1(); primer::presentation::RenderPage0(); }",
}
PRESENTATION_HELPER_CALLS = {
    "presentation_scan_input": "presentation_scan_input",
    "presentation_render_oled_pages": "presentation_render_oled_pages",
    "primer::presentation::RenderRedDebugPage": "RenderRedDebugPage",
    "primer::presentation::RenderPage3": "RenderPage3",
    "primer::presentation::RenderPage2": "RenderPage2",
    "primer::presentation::RenderPage1": "RenderPage1",
    "primer::presentation::RenderPage0": "RenderPage0",
}

PRESENTATION_OWNER_RULES = (
    *(owner_rule(f"primer::port::presentation::DigitalKeyAt({index})", f"key_{index + 1}", 1) for index in range(6)),
    *(owner_rule(f"primer::port::presentation::AnalogKeyAt({index})", f"key_{index + 7}", 1) for index in range(2)),
    owner_rule("primer::port::presentation::OperatorDisplay()", "ips200", 89),
    owner_rule("primer::port::presentation::SetRunFlag(2)", "run_flag=2", 1),
    owner_rule("primer::port::presentation::SetRunFlag(1)", "run_flag=1", 1),
    owner_rule("primer::port::presentation::SetEscDuty(i)", "esc_pwm.set_duty(i)", 1),
    owner_rule("primer::port::presentation::SaveParameters()", "Param_SaveAll()", 2),
    owner_rule("primer::port::presentation::ObserveDistanceRaw()", "dl1x_distance_raw", 1),
    owner_rule("primer::port::presentation::ObserveImu().gyro_y", "icm_data.gyro_y", 1),
    owner_rule("primer::port::presentation::ObserveImu().gyro_z", "icm_data.gyro_z", 1),
    owner_rule("primer::port::presentation::ObserveImu().yaw", "icm_data.yaw", 1),
    owner_rule("primer::port::presentation::ObserveTelemetry().encoder_distance", "encoder_abs", 1),
    owner_rule("primer::port::presentation::ObserveTelemetry().left_encoder_total", "encode_l_total", 1),
    owner_rule("primer::port::presentation::ObserveTelemetry().right_encoder_total", "encode_r_total", 1),
    *(owner_rule(f"primer::port::presentation::ObserveParameters().{field}", f"Flash.{field}", 4) for field in (
        "debug_rgb_r_min", "debug_rgb_rg_diff", "debug_rgb_rb_diff",
    )),
    owner_rule("primer::port::presentation::DistanceToRow", "real_distance_to_row", 2),
    owner_rule("((primer::port::vision::ObserveRightHighCorner().row) > (primer::port::vision::ObserveLeftHighCorner().row) ? (primer::port::vision::ObserveRightHighCorner().row) : (primer::port::vision::ObserveLeftHighCorner().row))", "MAX(R_h_guai.row,L_h_guai.row)", 1),
    owner_rule("primer::port::vision::kProcessedHeight", "LCDH_1", 21),
    owner_rule("primer::port::vision::kProcessedWidth", "LCDW_1", 12),
    *(owner_rule(f"primer::port::vision::{api}", legacy, count) for api, legacy, count in (
        ("ObserveBinaryImage()", "Image_Use", 3), ("ObserveBlackRatio()", "black_ratio", 1),
        ("ObserveDirectionError()", "Dir_err", 3), ("ObserveDistance()", "distance", 3),
        ("ObserveElementFacts().Huandao_L", "Flag.Huandao_L", 2),
        ("ObserveElementFacts().Huandao_R", "Flag.Huandao_R", 2),
        ("ObserveElementFacts().Redblock", "Flag.Redblock", 1),
        ("ObserveElementFacts().Zebra_cross", "Flag.Zebra_cross", 1),
        ("ObserveElementFacts().Zhangai", "Flag.Zhangai", 1),
        ("ObserveElementFacts().picture", "Flag.picture", 4),
        ("ObserveImageInformation().Both_lose", "imgInfo.Both_lose", 2),
        ("ObserveImageInformation().L_loselineSum", "imgInfo.L_loselineSum", 1),
        ("ObserveImageInformation().R_loselineSum", "imgInfo.R_loselineSum", 1),
        ("ObserveImageInformation().top", "imgInfo.top", 2),
        ("ObserveJumpPoint()", "jump_point", 2), ("ObserveJumpPointSecondary()", "jump_point1", 1),
        ("ObserveLeftHighCorner()", "L_h_guai", 8), ("ObserveRightHighCorner()", "R_h_guai", 8),
        ("ObserveLeftHighCornerSecondary()", "L_h_guai1", 4),
        ("ObserveRightHighCornerSecondary()", "R_h_guai1", 3),
        ("ObserveLeftLowCorner()", "L_l_guai", 2), ("ObserveRightLowCorner()", "R_l_guai", 2),
        ("ObserveLeftSideline()", "Left_Sideline", 5), ("ObserveRightSideline()", "Right_Sideline", 5),
        ("ObserveMidline()", "Mid_Line", 3), ("ObserveLongMax()", "long_max", 1),
        ("ObserveMaxlongColumn()", "maxlong_colume", 1), ("ObservePictureBlack()", "picture_black", 1),
        ("ObservePictureWhite()", "picture_white", 1), ("ObserveRedFindX()", "red_find_x", 1),
        ("ObserveRedFindY()", "red_find_y", 1), ("ObserveResizedFrame()", "resizedFrame", 5),
        ("ObserveRoundaboutYawError()", "Yaw_Huandao_err", 1),
        ("ObserveRowDistance()", "real_distance", 3),
    )),
)


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


def expand_runtime_helpers(
    tokens: tuple[str, ...],
    helpers: dict[str, FunctionBody],
    stack: tuple[str, ...] = (),
) -> tuple[str, ...]:
    """Inline only allowlisted zero-argument helper call statements."""
    result: list[str] = []
    index = 0
    while index < len(tokens):
        name = tokens[index]
        if name in RUNTIME_HELPERS and tokens[index:index + 4] == (name, "(", ")", ";"):
            if name in stack:
                raise ValueError(f"runtime helper recursion: {' -> '.join(stack + (name,))}")
            helper = helpers.get(name)
            if helper is None:
                raise ValueError(f"runtime helper has no unique active definition: {name}")
            if helper.tokens[:1] != ("{",) or helper.tokens[-1:] != ("}",):
                raise ValueError(f"runtime helper body is not braced: {name}")
            result.extend(expand_runtime_helpers(helper.tokens[1:-1], helpers, stack + (name,)))
            index += 4
            continue
        result.append(tokens[index])
        index += 1
    return tuple(result)


def canonicalize_runtime_tokens(
    tokens: tuple[str, ...], validate_counts: bool = True,
) -> tuple[tuple[str, ...], list[str]]:
    result = tokens
    failures: list[str] = []
    for current_text, baseline_text_value, expected_count in RUNTIME_OWNER_RULES:
        current = cpp_tokens(current_text)
        baseline = cpp_tokens(baseline_text_value)
        count = sum(
            result[index:index + len(current)] == current
            for index in range(len(result) - len(current) + 1)
        )
        if validate_counts and count != expected_count:
            failures.append(
                f"owner mapping {current_text!r}: expected {expected_count} uses, found {count}"
            )
        result = rewrite_tokens(result, ((current, baseline),))
    return result, failures


def expand_presentation_helpers(
    tokens: tuple[str, ...],
    helpers: dict[str, FunctionBody],
    stack: tuple[str, ...] = (),
) -> tuple[str, ...]:
    """Inline only the exact, qualified presentation call statements."""
    result = tokens
    for call_text, helper_name in PRESENTATION_HELPER_CALLS.items():
        call = cpp_tokens(call_text + "();")
        while True:
            match = next((i for i in range(len(result) - len(call) + 1) if result[i:i + len(call)] == call), None)
            if match is None:
                break
            if helper_name in stack:
                raise ValueError(f"presentation helper recursion: {' -> '.join(stack + (helper_name,))}")
            helper = helpers.get(helper_name)
            if helper is None:
                raise ValueError(f"presentation helper has no unique active definition: {helper_name}")
            if helper.tokens[:1] != ("{",) or helper.tokens[-1:] != ("}",):
                raise ValueError(f"presentation helper body is not braced: {helper_name}")
            expanded = expand_presentation_helpers(helper.tokens[1:-1], helpers, stack + (helper_name,))
            result = result[:match] + expanded + result[match + len(call):]
    return result


def canonicalize_presentation_tokens(
    tokens: tuple[str, ...], validate_counts: bool = True,
) -> tuple[tuple[str, ...], list[str]]:
    result = tokens
    failures: list[str] = []
    for current_text, baseline_text_value, expected_count in PRESENTATION_OWNER_RULES:
        current = cpp_tokens(current_text)
        baseline = cpp_tokens(baseline_text_value)
        count = sum(result[i:i + len(current)] == current for i in range(len(result) - len(current) + 1))
        if validate_counts and count != expected_count:
            failures.append(f"presentation mapping {current_text!r}: expected {expected_count} uses, found {count}")
        result = rewrite_tokens(result, ((current, baseline),))
    return result, failures


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
    token_identical = 0
    mechanically_composed = 0
    presentation_composed = 0
    runtime_originals = {function.name: function for function in originals if function.name in {"pit_callback", "main"}}
    runtime_entries: dict[str, FunctionBody] = {}
    for original_name, current_name in (("pit_callback", "pit_callback"), ("main", "RunApplication")):
        candidates = by_name.get(current_name, [])
        if len(candidates) == 1:
            runtime_entries[original_name] = candidates[0]
        else:
            failures.append(f"runtime entry {current_name}: expected one active definition, found {len(candidates)}")

    helper_definitions: dict[str, FunctionBody] = {}
    for name in RUNTIME_HELPERS:
        candidates = by_name.get(name, [])
        if len(candidates) == 1:
            helper_definitions[name] = candidates[0]
        else:
            failures.append(f"runtime helper {name}: expected one active definition, found {len(candidates)}")

    expanded_runtime: dict[str, tuple[str, ...]] = {}
    for name, entry in runtime_entries.items():
        try:
            expanded_runtime[name] = expand_runtime_helpers(entry.tokens, helper_definitions)
        except ValueError as error:
            failures.append(str(error))
    if len(expanded_runtime) == 2:
        _, mapping_failures = canonicalize_runtime_tokens(
            expanded_runtime["pit_callback"] + expanded_runtime["main"]
        )
        failures.extend(mapping_failures)
        for name, expanded in expanded_runtime.items():
            canonical, _ = canonicalize_runtime_tokens(expanded, validate_counts=False)
            if canonical == runtime_originals[name].tokens:
                mechanically_composed += 1
            else:
                failures.append(
                    f"{name} runtime composition differs from baseline "
                    f"sha={runtime_originals[name].digest}"
                )

    presentation_originals = {
        function.name: function for function in originals if function.name in {"key_scan", "oled_show"}
    }
    presentation_helpers: dict[str, FunctionBody] = {}
    for name in set(PRESENTATION_HELPER_CALLS.values()):
        candidates = by_name.get(name, [])
        if len(candidates) == 1:
            presentation_helpers[name] = candidates[0]
        else:
            failures.append(f"presentation helper {name}: expected one active definition, found {len(candidates)}")
    expanded_presentation: dict[str, tuple[str, ...]] = {}
    for name in ("key_scan", "oled_show"):
        candidates = by_name.get(name, [])
        if len(candidates) != 1:
            failures.append(f"presentation entry {name}: expected one active definition, found {len(candidates)}")
            continue
        try:
            expanded_presentation[name] = expand_presentation_helpers(candidates[0].tokens, presentation_helpers)
        except ValueError as error:
            failures.append(str(error))
    if len(expanded_presentation) == 2:
        _, mapping_failures = canonicalize_presentation_tokens(
            expanded_presentation["key_scan"] + expanded_presentation["oled_show"]
        )
        failures.extend(mapping_failures)
        for name, expanded in expanded_presentation.items():
            canonical, _ = canonicalize_presentation_tokens(expanded, validate_counts=False)
            if canonical == presentation_originals[name].tokens:
                presentation_composed += 1
            else:
                failures.append(
                    f"{name} presentation composition differs from baseline "
                    f"sha={presentation_originals[name].digest}"
                )

    for original in originals:
        if original.name in {"pit_callback", "main", "key_scan", "oled_show"}:
            continue
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
            token_identical += 1
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
    for name, expected_body in RUNTIME_WRAPPER_BODIES.items():
        expected = cpp_tokens(expected_body)
        wrappers = by_name.get(name, [])
        if len(wrappers) != 1 or wrappers[0].tokens != expected:
            actual = ", ".join(
                f"{wrapper.source}:{wrapper.line} sha={wrapper.digest}" for wrapper in wrappers
            ) or "no active definition"
            wrapper_failures.append(f"runtime {name}: {actual}")
    for name, expected_body in PRESENTATION_WRAPPER_BODIES.items():
        expected = cpp_tokens(expected_body)
        wrappers = by_name.get(name, [])
        if len(wrappers) != 1 or wrappers[0].tokens != expected:
            actual = ", ".join(
                f"{wrapper.source}:{wrapper.line} sha={wrapper.digest}" for wrapper in wrappers
            ) or "no active definition"
            wrapper_failures.append(f"presentation {name}: {actual}")

    print(f"baseline functions discovered: {len(originals)}")
    print(f"token-identical active bodies: {token_identical}")
    print(f"mechanically composed runtime bodies: {mechanically_composed}")
    print(f"mechanically composed presentation bodies: {presentation_composed}")
    print(f"accounted baseline functions: {token_identical + mechanically_composed + presentation_composed}")
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
    print("PASS: all 102 baseline functions are token-identical, mechanically delegated, or mechanically composed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
