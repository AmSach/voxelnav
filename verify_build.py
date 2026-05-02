#!/usr/bin/env python3
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BUILD_DIR = ROOT / "build"
SMOKE_BIN = BUILD_DIR / "voxelnav_core_smoke"


def run(cmd: list[str]) -> None:
    print("$ " + " ".join(cmd))
    subprocess.run(cmd, cwd=ROOT, check=True)


def main() -> int:
    BUILD_DIR.mkdir(exist_ok=True)

    opencv_cflags = subprocess.check_output(["pkg-config", "--cflags", "opencv4"], text=True).strip().split()
    opencv_libs = subprocess.check_output(["pkg-config", "--libs", "opencv4"], text=True).strip().split()

    cmd = [
        "g++",
        "-std=c++17",
        "-O2",
        "-Iinclude",
        "-I/usr/include/eigen3",
        *opencv_cflags,
        "test/core_smoke.cpp",
        "src/voxelizer.cpp",
        "src/segmenter.cpp",
        "-o",
        str(SMOKE_BIN),
        *opencv_libs,
    ]
    run(cmd)
    run([str(SMOKE_BIN)])

    print()
    print(f"Build verification succeeded: {SMOKE_BIN}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
