#!/usr/bin/env python3
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BUILD_DIR = ROOT / "build"
SMOKE_BIN = BUILD_DIR / "voxelnav_core_smoke"


def run(cmd: list[str], env: dict[str, str] | None = None) -> None:
    print("$ " + " ".join(cmd))
    subprocess.run(cmd, cwd=ROOT, check=True, env=env)


def main() -> int:
    BUILD_DIR.mkdir(exist_ok=True)

    opencv_cflags = subprocess.check_output(["pkg-config", "--cflags", "opencv4"], text=True).strip().split()
    opencv_libs = subprocess.check_output(["pkg-config", "--libs", "opencv4"], text=True).strip().split()
    ort_root = ROOT / "third_party/onnxruntime/pkg/onnxruntime-linux-x64-1.25.1"
    ort_include = str(ort_root / "include")
    ort_lib = str(ort_root / "lib")

    cmd = [
        "g++",
        "-std=c++17",
        "-O2",
        "-Iinclude",
        "-I/usr/include/eigen3",
        "-I" + ort_include,
        *opencv_cflags,
        "test/core_smoke.cpp",
        "src/voxelizer.cpp",
        "src/segmenter.cpp",
        "-o",
        str(SMOKE_BIN),
        *opencv_libs,
        "-L" + ort_lib,
        "-lonnxruntime",
        "-Wl,-rpath," + ort_lib,
        "-Wl,--disable-new-dtags",
    ]
    run(cmd)

    smoke_env = os.environ.copy()
    ld_library_path = smoke_env.get("LD_LIBRARY_PATH")
    smoke_env["LD_LIBRARY_PATH"] = ort_lib if not ld_library_path else ort_lib + ":" + ld_library_path
    run([str(SMOKE_BIN)], env=smoke_env)

    print()
    print(f"Build verification succeeded: {SMOKE_BIN}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
