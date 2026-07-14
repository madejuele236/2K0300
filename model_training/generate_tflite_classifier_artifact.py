#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ast
import hashlib
import struct
import zipfile
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class NpyArray:
    descr: str
    shape: tuple[int, ...]
    payload: bytes


def format_bytes(payload: bytes) -> str:
    rows = []
    for offset in range(0, len(payload), 16):
        rows.append("    " + ", ".join(f"0x{value:02x}" for value in payload[offset : offset + 16]))
    return ",\n".join(rows)


def format_prototypes(payload: bytes) -> str:
    values = struct.unpack(f"{len(payload)}b", payload)
    rows = []
    for offset in range(0, len(values), 4):
        rows.append("    {" + ", ".join(str(value) for value in values[offset : offset + 4]) + "}")
    return ",\n".join(rows)


def format_u8(values: tuple[int, ...]) -> str:
    rows = []
    for offset in range(0, len(values), 24):
        rows.append("    " + ", ".join(str(value) for value in values[offset : offset + 24]))
    return ",\n".join(rows)


def load_npy(archive: zipfile.ZipFile, key: str) -> NpyArray:
    name = f"{key}.npy"
    try:
        payload = archive.read(name)
    except KeyError as exc:
        raise ValueError(f"identity NPZ is missing required key {key!r}") from exc
    if payload[:6] != b"\x93NUMPY":
        raise ValueError(f"identity NPZ key {key!r} is not an NPY payload")
    major = payload[6]
    if major == 1:
        header_size = struct.unpack_from("<H", payload, 8)[0]
        data_offset = 10 + header_size
        header_offset = 10
    elif major in (2, 3):
        header_size = struct.unpack_from("<I", payload, 8)[0]
        data_offset = 12 + header_size
        header_offset = 12
    else:
        raise ValueError(f"identity NPZ key {key!r} uses unsupported NPY version {major}")
    header = ast.literal_eval(payload[header_offset:data_offset].decode("latin1"))
    if header.get("fortran_order") is not False:
        raise ValueError(f"identity NPZ key {key!r} must use C order")
    shape = tuple(int(value) for value in header["shape"])
    return NpyArray(str(header["descr"]), shape, payload[data_offset:])


def load_unicode_scalar(array: NpyArray, key: str) -> str:
    if array.shape != () or not array.descr.startswith("<U"):
        raise ValueError(f"identity NPZ key {key!r} must be a scalar little-endian Unicode string")
    characters = int(array.descr[2:])
    if len(array.payload) != characters * 4:
        raise ValueError(f"identity NPZ key {key!r} has an invalid payload size")
    return array.payload.decode("utf-32-le").rstrip("\0")


def load_float32_scalar(array: NpyArray, key: str) -> float:
    if array.shape != () or array.descr != "<f4" or len(array.payload) != 4:
        raise ValueError(f"identity NPZ key {key!r} must be a scalar float32")
    return struct.unpack("<f", array.payload)[0]


def main() -> int:
    parser = argparse.ArgumentParser(description="Embed the frozen d4 TFLite backbone and identity prototype classifier.")
    parser.add_argument("model", type=Path)
    parser.add_argument("identity_npz", type=Path)
    parser.add_argument("header", type=Path)
    parser.add_argument("source", type=Path)
    parser.add_argument("expected_model_sha256")
    parser.add_argument("expected_identity_sha256")
    args = parser.parse_args()

    model_payload = args.model.read_bytes()
    model_digest = hashlib.sha256(model_payload).hexdigest()
    if model_digest != args.expected_model_sha256:
        raise ValueError(
            f"TFLite model SHA-256 mismatch: expected {args.expected_model_sha256}, got {model_digest}"
        )
    if model_payload[4:8] != b"TFL3":
        raise ValueError("input does not contain a TFL3 FlatBuffer identifier")

    identity_payload = args.identity_npz.read_bytes()
    identity_digest = hashlib.sha256(identity_payload).hexdigest()
    if identity_digest != args.expected_identity_sha256:
        raise ValueError(
            f"identity NPZ SHA-256 mismatch: expected {args.expected_identity_sha256}, got {identity_digest}"
        )
    artifact_digest = hashlib.sha256(model_payload + identity_payload).hexdigest()
    with zipfile.ZipFile(args.identity_npz) as archive:
        prototypes = load_npy(archive, "prototypes_int8")
        parents = load_npy(archive, "prototype_parent")
        feature_source = load_unicode_scalar(load_npy(archive, "feature_source"), "feature_source")
        tie_break_policy = load_unicode_scalar(load_npy(archive, "tie_break_policy"), "tie_break_policy")
        int8_scale = load_float32_scalar(load_npy(archive, "int8_scale"), "int8_scale")

    if prototypes.descr != "|i1" or prototypes.shape != (844, 4) or len(prototypes.payload) != 844 * 4:
        raise ValueError("prototypes_int8 must have dtype int8 and shape (844, 4)")
    if parents.descr != "<i8" or parents.shape != (844,) or len(parents.payload) != 844 * 8:
        raise ValueError("prototype_parent must have dtype int64 and shape (844,)")
    parent_values = struct.unpack("<844q", parents.payload)
    if set(parent_values) != {0, 1, 2}:
        raise ValueError("prototype_parent must contain exactly parent classes 0, 1, and 2")
    if feature_source != "canonicalized_identity":
        raise ValueError(f"unexpected feature_source: {feature_source!r}")
    if tie_break_policy != "argmin_parent_order":
        raise ValueError(f"unexpected tie_break_policy: {tie_break_policy!r}")
    if int8_scale != 1.0:
        raise ValueError(f"unexpected int8_scale: {int8_scale!r}")

    args.header.parent.mkdir(parents=True, exist_ok=True)
    args.source.parent.mkdir(parents=True, exist_ok=True)
    args.header.write_text(
        """#ifndef LS2K_GENERATED_TFLITE_CLASSIFIER_ARTIFACT_HPP
#define LS2K_GENERATED_TFLITE_CLASSIFIER_ARTIFACT_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace ls2k::vision::ml::generated {
constexpr std::size_t kIdentityFeatureSize = 4;
constexpr std::size_t kIdentityPrototypeCount = 844;

struct IdentityPrototypeScore {
    std::array<int, 3> class_distances{};
    int class_id = -1;
    int margin = 0;
};

extern const std::uint8_t kTfliteClassifierModel[];
extern const std::size_t kTfliteClassifierModelSize;
extern const std::int8_t kIdentityPrototypes[kIdentityPrototypeCount][kIdentityFeatureSize];
extern const std::uint8_t kIdentityPrototypeParents[kIdentityPrototypeCount];
extern const char kTfliteClassifierModelSha256[];
extern const char kIdentityClassifierParamsSha256[];
extern const char kTfliteClassifierArtifactSha256[];
extern const char kTfliteClassifierArtifactId[];

IdentityPrototypeScore ScoreIdentityFeature(const std::int8_t* feature);
}  // namespace ls2k::vision::ml::generated

#endif
"""
    )
    args.source.write_text(
        f"""#include \"generated_tflite_classifier_artifact.hpp\"

#include <limits>

namespace ls2k::vision::ml::generated {{
alignas(16) const std::uint8_t kTfliteClassifierModel[] = {{
{format_bytes(model_payload)}
}};
const std::size_t kTfliteClassifierModelSize = sizeof(kTfliteClassifierModel);

const std::int8_t kIdentityPrototypes[kIdentityPrototypeCount][kIdentityFeatureSize] = {{
{format_prototypes(prototypes.payload)}
}};
const std::uint8_t kIdentityPrototypeParents[kIdentityPrototypeCount] = {{
{format_u8(parent_values)}
}};

const char kTfliteClassifierModelSha256[] = \"{model_digest}\";
const char kIdentityClassifierParamsSha256[] = \"{identity_digest}\";
const char kTfliteClassifierArtifactSha256[] = \"{artifact_digest}\";
const char kTfliteClassifierArtifactId[] = \"v8_d4best_recompile_identity\";

IdentityPrototypeScore ScoreIdentityFeature(const std::int8_t* feature) {{
    IdentityPrototypeScore out{{}};
    out.class_distances.fill(std::numeric_limits<int>::max());
    for (std::size_t prototype_index = 0; prototype_index < kIdentityPrototypeCount;
         ++prototype_index) {{
        int distance = 0;
        for (std::size_t coordinate = 0; coordinate < kIdentityFeatureSize; ++coordinate) {{
            const int delta = static_cast<int>(feature[coordinate]) -
                              static_cast<int>(kIdentityPrototypes[prototype_index][coordinate]);
            distance += delta * delta;
        }}
        const std::size_t parent = kIdentityPrototypeParents[prototype_index];
        if (distance < out.class_distances[parent]) {{
            out.class_distances[parent] = distance;
        }}
    }}
    out.class_id = 0;
    for (int parent = 1; parent < 3; ++parent) {{
        if (out.class_distances[static_cast<std::size_t>(parent)] <
            out.class_distances[static_cast<std::size_t>(out.class_id)]) {{
            out.class_id = parent;
        }}
    }}
    int second_distance = std::numeric_limits<int>::max();
    for (int parent = 0; parent < 3; ++parent) {{
        if (parent != out.class_id &&
            out.class_distances[static_cast<std::size_t>(parent)] < second_distance) {{
            second_distance = out.class_distances[static_cast<std::size_t>(parent)];
        }}
    }}
    out.margin = second_distance - out.class_distances[static_cast<std::size_t>(out.class_id)];
    return out;
}}
}}  // namespace ls2k::vision::ml::generated
"""
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
