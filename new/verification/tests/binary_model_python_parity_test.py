#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import subprocess
import sys

import numpy as np
from PIL import Image, ImageFilter


WIDTH = 320
HEIGHT = 240
SCALE = 16


def fnv1a(data: bytes) -> int:
    value = 1469598103934665603
    for item in data:
        value ^= item
        value = (value * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return value


def load_reference(path: Path):
    spec = importlib.util.spec_from_file_location("binary_reference", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load reference module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.optimized_centered_additive_ssr_otsu


def main() -> int:
    if len(sys.argv) != 4:
        print(
            "usage: parity_test PROBE_BINARY REFERENCE_PY RAW_DIRECTORY",
            file=sys.stderr,
        )
        return 2
    probe = Path(sys.argv[1])
    reference = load_reference(Path(sys.argv[2]))
    raw_directory = Path(sys.argv[3])
    probe_result = subprocess.run(
        [str(probe), str(raw_directory)],
        check=True,
        capture_output=True,
        text=True,
    )
    output = probe_result.stdout
    if probe_result.stderr:
        print(probe_result.stderr.strip())

    rows = {}
    for line in output.splitlines():
        name, valid, threshold, illumination_hash, binary_hash = line.split("\t")
        rows[name] = (
            valid == "1",
            int(threshold),
            int(illumination_hash),
            int(binary_hash),
        )

    paths = sorted(raw_directory.glob("*.raw"))
    if not paths:
        raise RuntimeError(f"no raw frames in {raw_directory}")
    if set(rows) != {path.name for path in paths}:
        raise RuntimeError("probe output does not cover the raw replay set exactly")

    mismatches = []
    for path in paths:
        gray = np.fromfile(path, dtype=np.uint8).reshape(HEIGHT, WIDTH)
        result = reference(gray)
        valid = bool(result.detail.get("valid", False))
        threshold = int(result.detail.get("threshold", 0))
        reduced = gray[SCALE // 2 : HEIGHT : SCALE,
                       SCALE // 2 : WIDTH : SCALE]
        illumination = np.asarray(
            Image.fromarray(reduced, mode="L").filter(
                ImageFilter.GaussianBlur(radius=35.0 / SCALE)
            ),
            dtype=np.uint8,
        )
        expected = (
            valid,
            threshold,
            fnv1a(illumination.tobytes()),
            fnv1a(result.image.astype(np.uint8).tobytes()),
        )
        if rows[path.name] != expected:
            mismatches.append((path.name, rows[path.name], expected))
            if len(mismatches) == 10:
                break

    if mismatches:
        for mismatch in mismatches:
            print(f"mismatch: {mismatch}", file=sys.stderr)
        return 1
    print(f"PASS: Python/C++ binary model parity frames={len(paths)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
