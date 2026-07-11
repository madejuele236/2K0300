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

PRIVATE_WIRING_TUS = {
    "runtime/hardware_composition.cpp",
    "runtime/service_composition.cpp",
}

SERVICE_OBJECT_TYPES = {
    "LQ_NCNN",
    "TransmissionStreamServer",
    "lq_camera_ex",
}

VISION_SOURCE_DEPENDENCIES = {
    "vision/facts.cpp": "vision/internal/dependencies/facts_dependencies.hpp",
    "vision/preprocess.cpp": "vision/internal/dependencies/preprocess_dependencies.hpp",
    "vision/track.cpp": "vision/internal/dependencies/track_dependencies.hpp",
    "vision/line_repair.cpp": "vision/internal/dependencies/line_repair_dependencies.hpp",
    "vision/steering.cpp": "vision/internal/dependencies/steering_dependencies.hpp",
    "vision/pipeline.cpp": "vision/internal/dependencies/pipeline_dependencies.hpp",
    "vision/pipeline_init.cpp": "vision/internal/dependencies/pipeline_init_dependencies.hpp",
    "vision/red_target.cpp": "vision/internal/dependencies/red_target_dependencies.hpp",
    "vision/scenes/roundabout.cpp": "vision/internal/dependencies/roundabout_dependencies.hpp",
    "vision/scenes/picture_scene.cpp": "vision/internal/dependencies/picture_scene_dependencies.hpp",
    "vision/scenes/element_scenes.cpp": "vision/internal/dependencies/element_scenes_dependencies.hpp",
}

INCLUDE = re.compile(r'^\s*#\s*include\s+"([^"]+)"', re.MULTILINE)
EXTERN_DECLARATION = re.compile(r'^\s*extern\s+', re.MULTILINE)
FORBIDDEN_PUBLIC_MACROS = {
    "ABS",
    "LIMIT",
    "MAX",
    "MIN",
    "PARAM_FILE_NAME",
    "PORT",
    "SERVER_IP",
    "SaveFloat",
    "SaveInt",
}


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

    service_object_definition = re.compile(
        rf"^\s*(?:(?:const|inline|static)\s+)*"
        rf"(?:{'|'.join(sorted(SERVICE_OBJECT_TYPES))})\s+"
        rf"[A-Za-z_]\w*\s*(?:[;(={{])",
        re.MULTILINE,
    )
    invalid_service_owners = sorted(
        str(path.relative_to(CODE))
        for path, text in source_texts.items()
        if path != CODE / "runtime" / "service_composition.cpp"
        and service_object_definition.search(text)
    )
    if invalid_service_owners:
        failures.append(
            "cross-owner service objects must be defined by "
            "runtime/service_composition.cpp: "
            f"{invalid_service_owners}"
        )

    service_header_owners = sorted(
        str(path.relative_to(CODE))
        for path in CODE.rglob("*")
        if path.suffix in {".h", ".hpp"}
        and layer_of(path) is not None
        and service_object_definition.search(path.read_text(encoding="utf-8"))
    )
    if service_header_owners:
        failures.append(
            "layer headers define cross-owner service objects: "
            f"{service_header_owners}"
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
        header_text = header.read_text(encoding="utf-8")
        if EXTERN_DECLARATION.search(header_text):
            failures.append(
                f"public layer header exposes legacy extern state: {header.relative_to(ROOT)}"
            )
        macros = set(
            re.findall(r"^\s*#\s*define\s+([A-Za-z_]\w*)", header_text, re.MULTILINE)
        )
        forbidden_macros = sorted(macros & FORBIDDEN_PUBLIC_MACROS)
        if forbidden_macros:
            failures.append(
                f"public layer header publishes generic legacy macros "
                f"{forbidden_macros}: {header.relative_to(ROOT)}"
            )

    for source in layered_files:
        text = source.read_text(encoding="utf-8")
        source_relative = str(source.relative_to(CODE))
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
                source_layer not in {None, "runtime", "port"}
                and target_layer not in {None, source_layer, "port"}
            ):
                failures.append(
                    f"non-runtime owner depends directly on another owner instead of port: "
                    f"{source.relative_to(ROOT)} -> {target.relative_to(ROOT)}"
                )
            if (
                source_layer == "port"
                and target_layer not in {None, "port"}
            ):
                failures.append(
                    f"port contract depends on an owner implementation: "
                    f"{source.relative_to(ROOT)} -> {target.relative_to(ROOT)}"
                )
            if source_layer == "vision" and target_layer == "runtime":
                failures.append(
                    f"vision depends on runtime instead of a port contract: "
                    f"{source.relative_to(ROOT)} -> {target.relative_to(ROOT)}"
                )
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
                and source_relative not in PRIVATE_WIRING_TUS
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

    dependency_root = CODE / "vision" / "internal" / "dependencies"
    expected_dependency_headers = set(VISION_SOURCE_DEPENDENCIES.values())
    actual_dependency_headers = {
        str(path.relative_to(CODE)) for path in dependency_root.glob("*.hpp")
    }
    if actual_dependency_headers != expected_dependency_headers:
        failures.append(
            "vision per-TU dependency inventory differs: "
            f"expected {sorted(expected_dependency_headers)}, "
            f"found {sorted(actual_dependency_headers)}"
        )
    for source_name, dependency_name in VISION_SOURCE_DEPENDENCIES.items():
        source = CODE / source_name
        included_dependencies = []
        for include in INCLUDE.findall(source.read_text(encoding="utf-8")):
            target = resolve_include(source, include)
            if target is not None and target.parent == dependency_root.resolve():
                included_dependencies.append(str(target.relative_to(CODE)))
        if included_dependencies != [dependency_name]:
            failures.append(
                f"{source_name} must include only its own dependency header: "
                f"expected {[dependency_name]}, found {included_dependencies}"
            )

    forbidden_vision_umbrella = CODE / "vision" / "internal" / "vision_dependencies.hpp"
    if forbidden_vision_umbrella.exists():
        failures.append("vision_dependencies.hpp recreates the removed application umbrella")

    for retired_aggregate in (
        CODE / "vision" / "internal" / "legacy_vision_state.hpp",
        CODE / "vision" / "internal" / "legacy_owner_bindings.hpp",
    ):
        if retired_aggregate.exists():
            failures.append(
                f"retired vision aggregate still exists: {retired_aggregate.relative_to(ROOT)}"
            )

    pure_contracts = (
        CODE / "vision" / "vision_facts.hpp",
        CODE / "vision" / "internal" / "vision_stage_contracts.hpp",
    )
    forbidden_vision_contract_includes = {
        "legacy_owner_bindings.hpp",
        "legacy_vision_state.hpp",
    }
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
        if contract.name == "vision_stage_contracts.hpp":
            for include in INCLUDE.findall(text):
                if pathlib.PurePosixPath(include).name in forbidden_vision_contract_includes:
                    failures.append(
                        f"{contract.relative_to(ROOT)} includes legacy aggregate {include}"
                    )
        for token in forbidden_contract_tokens:
            if token in text:
                failures.append(f"{contract.relative_to(ROOT)} depends on {token}")

    vision_facts_text = (CODE / "vision" / "vision_facts.hpp").read_text(encoding="utf-8")
    if re.search(r"^\s*#\s*define\b", vision_facts_text, re.MULTILINE):
        failures.append("public vision_facts.hpp publishes preprocessor macros")

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
    print(f"one-to-one vision dependency headers: {len(actual_dependency_headers)}")
    if failures:
        print("architecture failures:")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    print(
        "PASS: explicit source ownership, mutually unaware non-runtime owners, "
        "public API, pure vision facts, private-header, and umbrella boundaries hold"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
