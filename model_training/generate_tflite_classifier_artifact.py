#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path


def format_bytes(payload: bytes) -> str:
    rows = []
    for offset in range(0, len(payload), 16):
        rows.append("    " + ", ".join(f"0x{value:02x}" for value in payload[offset : offset + 16]))
    return ",\n".join(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="Embed one frozen TFLite classifier model.")
    parser.add_argument("model", type=Path)
    parser.add_argument("header", type=Path)
    parser.add_argument("source", type=Path)
    parser.add_argument("expected_sha256")
    args = parser.parse_args()

    payload = args.model.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != args.expected_sha256:
        raise ValueError(f"TFLite model SHA-256 mismatch: expected {args.expected_sha256}, got {digest}")
    if payload[4:8] != b"TFL3":
        raise ValueError("input does not contain a TFL3 FlatBuffer identifier")

    args.header.parent.mkdir(parents=True, exist_ok=True)
    args.source.parent.mkdir(parents=True, exist_ok=True)
    args.header.write_text(
        """#ifndef LS2K_GENERATED_TFLITE_CLASSIFIER_ARTIFACT_HPP
#define LS2K_GENERATED_TFLITE_CLASSIFIER_ARTIFACT_HPP

#include <cstddef>
#include <cstdint>

namespace ls2k::vision::ml::generated {
extern const std::uint8_t kTfliteClassifierModel[];
extern const std::size_t kTfliteClassifierModelSize;
extern const char kTfliteClassifierModelSha256[];
extern const char kTfliteClassifierArtifactId[];
}  // namespace ls2k::vision::ml::generated

#endif
"""
    )
    args.source.write_text(
        f"""#include \"generated_tflite_classifier_artifact.hpp\"

namespace ls2k::vision::ml::generated {{
alignas(16) const std::uint8_t kTfliteClassifierModel[] = {{
{format_bytes(payload)}
}};
const std::size_t kTfliteClassifierModelSize = sizeof(kTfliteClassifierModel);
const char kTfliteClassifierModelSha256[] = \"{digest}\";
const char kTfliteClassifierArtifactId[] = \"v8_parent_c2612_d6_d24pca2_qanchor\";
}}  // namespace ls2k::vision::ml::generated
"""
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
