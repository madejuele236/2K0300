#!/usr/bin/env python3
"""Enforce the refactored primer ownership and include boundaries."""

from __future__ import annotations

import pathlib
import re
import sys

from compare_function_bodies import function_bodies


ROOT = pathlib.Path(__file__).resolve().parents[2]
CODE = ROOT / "primer_code" / "project" / "code"
CMAKE = ROOT / "primer_code" / "project" / "user" / "CMakeLists.txt"
UMBRELLA = ROOT / "primer_code" / "libraries" / "zf_common" / "zf_common_headfile.hpp"

LAYERS = {
    "control",
    "estimation",
    "inference",
    "parameters",
    "platform",
    "port",
    "presentation",
    "runtime",
    "transport",
    "vision",
}

COMPATIBILITY_TUS = {
    "control.cpp",
    "filt.cpp",
    "flash.cpp",
    "image.cpp",
    "init.cpp",
    "lq_ncnn.cpp",
    "show.cpp",
    "ww_transmission.cpp",
}

APPLICATION_HEADERS = {
    "init.h",
    "filt.h",
    "flash.h",
    "control.h",
    "image.h",
    "show.h",
    "lq_ncnn.hpp",
    "lq_camera_ex.hpp",
    "ww_transmission.h",
}

EXPECTED_DEFINITION_OWNERS = {
    r"^LQ_NCNN\s+classifier\s*;": "runtime/service_composition.cpp",
    r"^TransmissionStreamServer\s+camera_server\s*;": "runtime/service_composition.cpp",
    r"^lq_camera_ex\s+cam\s*\(": "runtime/service_composition.cpp",
    r"^unsigned\s+char\s+Image_IFS\s*\[": "presentation/show.cpp",
    r"^char\s+txt\s*\[": "presentation/show.cpp",
}

INCLUDE = re.compile(r'^\s*#\s*include\s+"([^"]+)"', re.MULTILINE)
EXTERN_DECLARATION = re.compile(r'^\s*extern\s+', re.MULTILINE)


def layer_of(path: pathlib.Path) -> str | None:
    try:
        relative = path.resolve().relative_to(CODE.resolve())
    except ValueError:
        return None
    return relative.parts[0] if relative.parts and relative.parts[0] in LAYERS else None


def resolve_include(source: pathlib.Path, include: str) -> pathlib.Path | None:
    local = (source.parent / include).resolve()
    if local.exists():
        return local
    rooted = (CODE / include).resolve()
    if rooted.exists():
        return rooted
    return None


def is_private_header(path: pathlib.Path) -> bool:
    relative = path.resolve().relative_to(CODE.resolve())
    return "internal" in relative.parts or "_internal" in path.name


def is_public_layer_header(path: pathlib.Path) -> bool:
    return path.suffix in {".h", ".hpp"} and not is_private_header(path)


def main() -> int:
    failures: list[str] = []
    cmake = CMAKE.read_text(encoding="utf-8")

    if "aux_source_directory(../code" in cmake.replace(" ", ""):
        failures.append("CMake still discovers application sources through aux_source_directory")

    cmake_sources = {
        match.replace("../code/", "")
        for match in re.findall(r"\.\./code/[A-Za-z0-9_./-]+\.cpp", cmake)
    }
    active_sources = {
        str(path.relative_to(CODE))
        for path in CODE.rglob("*.cpp")
        if layer_of(path) is not None
    }
    missing_from_build = sorted(active_sources - cmake_sources)
    stale_in_build = sorted(cmake_sources - active_sources)
    if missing_from_build:
        failures.append(f"active layered sources missing from CMake: {missing_from_build}")
    if stale_in_build:
        failures.append(f"CMake lists nonexistent layered sources: {stale_in_build}")
    for compatibility in COMPATIBILITY_TUS:
        if f"../code/{compatibility}" in cmake:
            failures.append(f"compatibility TU is incorrectly active in CMake: {compatibility}")

    source_texts = {
        path: path.read_text(encoding="utf-8")
        for path in CODE.rglob("*.cpp")
        if layer_of(path) is not None
    }
    for pattern, expected_owner in EXPECTED_DEFINITION_OWNERS.items():
        owners = [
            str(path.relative_to(CODE))
            for path, text in source_texts.items()
            if re.search(pattern, text, re.MULTILINE)
        ]
        if owners != [expected_owner]:
            failures.append(
                f"definition owner mismatch for /{pattern}/: "
                f"expected {[expected_owner]}, found {owners}"
            )

    service_composition = source_texts.get(CODE / "runtime" / "service_composition.cpp", "")
    service_order = [
        service_composition.find("LQ_NCNN classifier;"),
        service_composition.find("TransmissionStreamServer camera_server;"),
        service_composition.find("lq_camera_ex cam("),
    ]
    if any(index < 0 for index in service_order) or service_order != sorted(service_order):
        failures.append(
            "runtime service composition does not preserve classifier -> server -> camera order"
        )

    umbrella = UMBRELLA.read_text(encoding="utf-8")
    for header in sorted(APPLICATION_HEADERS):
        if re.search(rf'#\s*include\s+"{re.escape(header)}"', umbrella):
            failures.append(f"vendor umbrella includes application facade: {header}")

    layered_files = [
        path
        for path in CODE.rglob("*")
        if path.suffix in {".cpp", ".h", ".hpp"} and layer_of(path) is not None
    ]
    public_layer_headers = [path for path in layered_files if is_public_layer_header(path)]
    for header in public_layer_headers:
        if EXTERN_DECLARATION.search(header.read_text(encoding="utf-8")):
            failures.append(
                f"public layer header exposes legacy extern state: {header.relative_to(ROOT)}"
            )

    for source in layered_files:
        text = source.read_text(encoding="utf-8")
        if '"zf_common_headfile.hpp"' in text:
            failures.append(f"active layer uses application-wide umbrella: {source.relative_to(ROOT)}")
        for include in INCLUDE.findall(text):
            target = resolve_include(source, include)
            if target is None:
                continue
            source_layer = layer_of(source)
            target_layer = layer_of(target)
            target_is_private = is_private_header(target)
            if (
                is_public_layer_header(source)
                and target_is_private
            ):
                failures.append(
                    f"public layer header reaches private implementation: "
                    f"{source.relative_to(ROOT)} -> {target.relative_to(ROOT)}"
                )
            if (
                target_is_private
                and target_layer is not None
                and source_layer != target_layer
                and source_layer != "runtime"
            ):
                failures.append(
                    f"{source.relative_to(ROOT)} reaches private {target.relative_to(ROOT)}"
                )
            if target.name == "presentation_adapter_dependencies.hpp" and source_layer != "presentation":
                failures.append(
                    f"presentation adapter leaks into {source.relative_to(ROOT)}"
                )
            if target.parent == CODE.resolve() and target.name in APPLICATION_HEADERS:
                failures.append(
                    f"active layer includes root compatibility facade: "
                    f"{source.relative_to(ROOT)} -> {target.name}"
                )

    forbidden_vision_umbrella = CODE / "vision" / "internal" / "vision_dependencies.hpp"
    if forbidden_vision_umbrella.exists():
        failures.append("vision_dependencies.hpp recreates the removed application umbrella")

    pure_contracts = (
        CODE / "vision" / "vision_facts.hpp",
        CODE / "vision" / "internal" / "vision_stage_contracts.hpp",
    )
    forbidden_contract_tokens = (
        "control.h",
        "filt.h",
        "flash.h",
        "init.h",
        "lq_ncnn",
        "lq_camera",
        "ww_transmission",
        "zf_driver_",
        "zf_device_",
    )
    for contract in pure_contracts:
        if not contract.exists():
            failures.append(f"missing pure vision contract: {contract.relative_to(ROOT)}")
            continue
        text = contract.read_text(encoding="utf-8")
        for token in forbidden_contract_tokens:
            if token in text:
                failures.append(f"{contract.relative_to(ROOT)} depends on {token}")

    for filename in sorted(COMPATIBILITY_TUS):
        path = CODE / filename
        bodies = function_bodies(path.read_text(encoding="utf-8"), str(path.relative_to(ROOT)))
        if bodies:
            names = [body.name for body in bodies]
            failures.append(f"root compatibility TU owns active definitions: {filename}: {names}")

    low_pass_formula_owners: list[str] = []
    for path in layered_files:
        if "6.28f * time" in path.read_text(encoding="utf-8"):
            low_pass_formula_owners.append(str(path.relative_to(ROOT)))
    if low_pass_formula_owners != ["primer_code/project/code/port/low_pass_filter.hpp"]:
        failures.append(f"low-pass formula owners are not singular: {low_pass_formula_owners}")

    image_deal_definitions = []
    for path in CODE.rglob("*.cpp"):
        for body in function_bodies(path.read_text(encoding="utf-8"), str(path.relative_to(ROOT))):
            if body.name == "ImageDeal":
                image_deal_definitions.append(f"{path.relative_to(ROOT)}:{body.line}")
    if len(image_deal_definitions) != 1:
        failures.append(f"ImageDeal must have one pipeline owner: {image_deal_definitions}")

    print(f"active layered sources: {len(active_sources)}")
    print(f"explicit CMake application sources: {len(cmake_sources)}")
    print(f"layered headers/sources scanned: {len(layered_files)}")
    print(f"public layer headers scanned: {len(public_layer_headers)}")
    if failures:
        print("architecture failures:")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    print(
        "PASS: explicit source ownership, public API, pure vision facts, "
        "private-header, and umbrella boundaries hold"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
