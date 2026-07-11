#!/usr/bin/env python3
"""Compare original application static storage and used macro definitions.

Function-body comparison deliberately ignores file-scope declarations.  This
gate closes that gap: every original application global definition must retain
an identical declaration/initializer token sequence after relocation, and every
application macro referenced by those declarations or function bodies must keep
the same value (directly or through a single constexpr source of truth).  New
composition helpers are allowed, but they cannot replace or silently alter an
original state definition.
"""

from __future__ import annotations

import collections
import dataclasses
import pathlib
import re
import subprocess
import sys
from typing import Iterable

from compare_function_bodies import (
    BASELINE,
    ORIGINAL_SOURCES,
    SIGNATURE,
    cpp_tokens,
    mask_non_code,
    repo_root,
)


APP_SUFFIXES = {".cpp", ".h", ".hpp"}
EXPECTED_GLOBAL_STATEMENTS = 151
EXPECTED_REFERENCED_MACROS = 47
SKIP_PREFIXES = {
    "class",
    "enum",
    "namespace",
    "static_assert",
    "template",
    "typedef",
    "using",
}
BUILTIN_FUNCTION_RETURNS = {
    "auto",
    "bool",
    "char",
    "double",
    "float",
    "int",
    "int8_t",
    "int16_t",
    "int32_t",
    "int64_t",
    "long",
    "short",
    "signed",
    "size_t",
    "uint8_t",
    "uint16_t",
    "uint32_t",
    "uint64_t",
    "unsigned",
    "void",
}
IDENTIFIER = re.compile(r"^[A-Za-z_]\w*$")


@dataclasses.dataclass(frozen=True)
class GlobalStatement:
    source: str
    line: int
    tokens: tuple[str, ...]


@dataclasses.dataclass(frozen=True)
class MacroDefinition:
    name: str
    parameters: tuple[str, ...]
    replacement: tuple[str, ...]
    source: str
    line: int

    @property
    def signature(self) -> tuple[str, tuple[str, ...], tuple[str, ...]]:
        return self.name, self.parameters, self.replacement


def git(*args: str) -> str:
    result = subprocess.run(
        ["rtk", "git", *args],
        cwd=repo_root(),
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return result.stdout


def baseline_text(path: str) -> str:
    return git("show", f"{BASELINE}:{path}")


def blank_ranges(text: str, ranges: Iterable[tuple[int, int]]) -> str:
    chars = list(text)
    for start, end in ranges:
        for index in range(start, end):
            if chars[index] != "\n":
                chars[index] = " "
    return "".join(chars)


def function_ranges(text: str, source: str) -> list[tuple[int, int]]:
    masked = mask_non_code(text)
    ranges: list[tuple[int, int]] = []
    for match in SIGNATURE.finditer(masked):
        open_brace = match.end() - 1
        depth = 0
        close_brace = None
        for index in range(open_brace, len(masked)):
            if masked[index] == "{":
                depth += 1
            elif masked[index] == "}":
                depth -= 1
                if depth == 0:
                    close_brace = index + 1
                    break
        if close_brace is None:
            line = text.count("\n", 0, open_brace) + 1
            raise RuntimeError(f"unbalanced function in {source}:{line}")
        ranges.append((match.start(), close_brace))
    return ranges


def blank_preprocessor(text: str) -> str:
    lines = text.splitlines(keepends=True)
    result: list[str] = []
    continuing = False
    for line in lines:
        stripped = line.lstrip()
        if not continuing and stripped.startswith("#/"):
            marker = line.index("#")
            result.append(line[:marker] + " " + line[marker + 1 :])
            continuing = False
            continue
        directive = continuing or line.lstrip().startswith("#")
        continuing = directive and line.rstrip("\r\n").rstrip().endswith("\\")
        if directive:
            result.append("".join("\n" if ch == "\n" else " " for ch in line))
        else:
            result.append(line)
    return "".join(result)


def looks_like_function_declaration(tokens: tuple[str, ...]) -> bool:
    if not tokens or "(" not in tokens or "=" in tokens or "{" in tokens:
        return False
    first = next((token for token in tokens if token not in {"static", "inline", "constexpr"}), "")
    return first in BUILTIN_FUNCTION_RETURNS


def global_statements(text: str, source: str) -> list[GlobalStatement]:
    without_functions = blank_ranges(text, function_ranges(text, source))
    visible = blank_preprocessor(without_functions)
    masked = mask_non_code(visible)
    statements: list[GlobalStatement] = []
    start = 0
    braces = parentheses = brackets = 0
    for index, char in enumerate(masked):
        if char == "{":
            braces += 1
        elif char == "}":
            braces = max(0, braces - 1)
            if braces == 0:
                tail = masked[index + 1 :]
                next_nonspace = next((value for value in tail if not value.isspace()), "")
                if next_nonspace != ";":
                    start = index + 1
        elif char == "(":
            parentheses += 1
        elif char == ")":
            parentheses = max(0, parentheses - 1)
        elif char == "[":
            brackets += 1
        elif char == "]":
            brackets = max(0, brackets - 1)
        elif char == ";" and braces == 0 and parentheses == 0 and brackets == 0:
            chunk = visible[start : index + 1]
            tokens = cpp_tokens(chunk)
            start = index + 1
            if not tokens:
                continue
            prefix = next((token for token in tokens if IDENTIFIER.match(token)), "")
            if prefix in SKIP_PREFIXES or prefix == "extern":
                continue
            if prefix == "struct" and "{" in tokens:
                continue
            if looks_like_function_declaration(tokens):
                continue
            statements.append(
                GlobalStatement(
                    source=source,
                    line=text.count("\n", 0, index - len(chunk)) + 1,
                    tokens=tokens,
                )
            )
    return statements


def logical_lines(text: str) -> Iterable[tuple[int, str]]:
    lines = text.splitlines()
    index = 0
    while index < len(lines):
        start = index + 1
        logical = lines[index]
        while logical.rstrip().endswith("\\") and index + 1 < len(lines):
            logical = logical.rstrip()[:-1] + " " + lines[index + 1].lstrip()
            index += 1
        yield start, logical
        index += 1


MACRO = re.compile(
    r"^\s*#\s*define\s+([A-Za-z_]\w*)(\([^)]*\))?\s*(.*?)\s*$"
)
CONSTEXPR_VALUE = re.compile(
    r"\bconstexpr\s+[^=;]*?\b([A-Za-z_]\w*)\s*=\s*([^;]+);"
)


def macro_definitions(text: str, source: str) -> list[MacroDefinition]:
    definitions: list[MacroDefinition] = []
    for line, logical in logical_lines(text):
        match = MACRO.match(logical)
        if not match:
            continue
        parameter_text = match.group(2)
        parameters = cpp_tokens(parameter_text[1:-1]) if parameter_text else ()
        definitions.append(
            MacroDefinition(
                name=match.group(1),
                parameters=parameters,
                replacement=cpp_tokens(match.group(3)),
                source=source,
                line=line,
            )
        )
    return definitions


def constexpr_values(texts: Iterable[str]) -> dict[str, tuple[str, ...]]:
    values: dict[str, tuple[str, ...]] = {}
    for text in texts:
        for match in CONSTEXPR_VALUE.finditer(text):
            values[match.group(1)] = cpp_tokens(match.group(2))
    return values


def canonical_macro_signature(
    definition: MacroDefinition,
    constants: dict[str, tuple[str, ...]],
) -> tuple[str, tuple[str, ...], tuple[str, ...]]:
    replacement = definition.replacement
    if replacement:
        final_name = replacement[-1]
        qualified_prefix = replacement[:-1]
        if (
            final_name in constants
            and qualified_prefix in {
                ("primer", "::", "port", "::", "vision", "::"),
                ("::", "primer", "::", "port", "::", "vision", "::"),
            }
        ):
            replacement = constants[final_name]
    return definition.name, definition.parameters, replacement


def baseline_app_paths() -> list[str]:
    paths = git(
        "ls-tree", "-r", "--name-only", BASELINE,
        "primer_code/project/code", "primer_code/project/user",
    ).splitlines()
    return [path for path in paths if pathlib.PurePosixPath(path).suffix in APP_SUFFIXES]


def current_app_paths() -> list[pathlib.Path]:
    root = repo_root() / "primer_code" / "project"
    paths = list((root / "code").rglob("*")) + list((root / "user").rglob("*"))
    return sorted(path for path in paths if path.is_file() and path.suffix in APP_SUFFIXES)


def referenced_macro_names(
    source_texts: Iterable[str], definitions: list[MacroDefinition]
) -> set[str]:
    tokens: set[str] = set()
    for text in source_texts:
        tokens.update(cpp_tokens(blank_preprocessor(text)))
    by_name: dict[str, list[MacroDefinition]] = collections.defaultdict(list)
    for definition in definitions:
        by_name[definition.name].append(definition)
    referenced = set(by_name) & tokens
    changed = True
    while changed:
        changed = False
        for name in tuple(referenced):
            for definition in by_name[name]:
                for token in definition.replacement:
                    if token in by_name and token not in referenced:
                        referenced.add(token)
                        changed = True
    return referenced


def format_tokens(tokens: tuple[str, ...], limit: int = 18) -> str:
    shown = " ".join(tokens[:limit])
    return shown + (" ..." if len(tokens) > limit else "")


def main() -> int:
    baseline_source_texts = {
        path: baseline_text(path) for path in ORIGINAL_SOURCES
    }
    current_paths = current_app_paths()
    root = repo_root()

    expected: list[GlobalStatement] = []
    for path, text in baseline_source_texts.items():
        expected.extend(global_statements(text, f"{BASELINE}:{path}"))

    actual: list[GlobalStatement] = []
    for path in current_paths:
        if path.suffix != ".cpp":
            continue
        actual.extend(
            global_statements(
                path.read_text(encoding="utf-8"), str(path.relative_to(root))
            )
        )

    available = collections.Counter(statement.tokens for statement in actual)
    missing_globals: list[GlobalStatement] = []
    for statement in expected:
        if available[statement.tokens]:
            available[statement.tokens] -= 1
        else:
            missing_globals.append(statement)

    baseline_files = {
        path: baseline_text(path) for path in baseline_app_paths()
    }
    current_files = {
        str(path.relative_to(root)): path.read_text(encoding="utf-8")
        for path in current_paths
    }
    baseline_macros = [
        definition
        for path, text in baseline_files.items()
        for definition in macro_definitions(text, f"{BASELINE}:{path}")
    ]
    current_macros = [
        definition
        for path, text in current_files.items()
        for definition in macro_definitions(text, path)
    ]
    current_constants = constexpr_values(current_files.values())
    referenced = referenced_macro_names(baseline_source_texts.values(), baseline_macros)
    expected_macros = [definition for definition in baseline_macros if definition.name in referenced]
    available_macros = collections.Counter(
        canonical_macro_signature(definition, current_constants)
        for definition in current_macros
    )
    missing_macros: list[MacroDefinition] = []
    for definition in expected_macros:
        expected_signature = canonical_macro_signature(definition, {})
        if available_macros[expected_signature]:
            available_macros[expected_signature] -= 1
        else:
            missing_macros.append(definition)

    print(f"baseline global definition statements: {len(expected)}")
    print(f"token-identical current global definitions: {len(expected) - len(missing_globals)}")
    print(f"baseline referenced app macro definitions: {len(expected_macros)}")
    print(
        "equivalent current macro definitions: "
        f"{len(expected_macros) - len(missing_macros)}"
    )
    coverage_failures: list[str] = []
    if len(expected) != EXPECTED_GLOBAL_STATEMENTS:
        coverage_failures.append(
            f"global parser coverage changed: expected {EXPECTED_GLOBAL_STATEMENTS}, "
            f"found {len(expected)}"
        )
    if len(expected_macros) != EXPECTED_REFERENCED_MACROS:
        coverage_failures.append(
            f"macro parser coverage changed: expected {EXPECTED_REFERENCED_MACROS}, "
            f"found {len(expected_macros)}"
        )
    if coverage_failures:
        print("baseline coverage failures:")
        for failure in coverage_failures:
            print(f"  - {failure}")
    if missing_globals:
        print("missing or changed global definitions:")
        for statement in missing_globals:
            print(
                f"  - {statement.source}:{statement.line}: "
                f"{format_tokens(statement.tokens)}"
            )
    if missing_macros:
        print("missing or changed referenced macro definitions:")
        for definition in missing_macros:
            print(
                f"  - {definition.source}:{definition.line}: {definition.name} "
                f"{format_tokens(definition.replacement)}"
            )
    if coverage_failures or missing_globals or missing_macros:
        return 1
    print(
        "PASS: every original global initializer remains token-identical and "
        "every referenced app macro keeps an equivalent value"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
