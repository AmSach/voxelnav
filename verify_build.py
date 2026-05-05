#!/usr/bin/env python3
from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BUILD_DIR = ROOT / "build"
SMOKE_BIN = BUILD_DIR / "voxelnav_core_smoke"


def run(cmd: list[str], env: dict[str, str] | None = None) -> None:
    print("$ " + " ".join(cmd))
    subprocess.run(cmd, cwd=ROOT, check=True, env=env)


def capture(cmd: list[str], env: dict[str, str] | None = None) -> str:
    print("$ " + " ".join(cmd))
    completed = subprocess.run(
        cmd,
        cwd=ROOT,
        check=True,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if completed.stdout:
        print(completed.stdout, end="")
    if completed.stderr:
        print(completed.stderr, end="", file=sys.stderr)
    return completed.stdout


def sanitize_ld_library_path(existing: str | None, preferred: str) -> str:
    entries: list[str] = []
    seen: set[str] = set()

    def add(path: str) -> None:
        normalized = str(Path(path))
        if normalized in seen:
            return
        seen.add(normalized)
        entries.append(path)

    add(preferred)
    for raw in (existing or "").split(os.pathsep):
        if not raw:
            continue
        if "onnxruntime_vendor" in raw:
            continue
        add(raw)
    return os.pathsep.join(entries)


def assert_linker_tag(binary: Path, ort_lib: str) -> None:
    output = capture(["readelf", "-d", str(binary)])
    if "RUNPATH" in output:
        raise RuntimeError(f"{binary} still has RUNPATH; expected RPATH for deterministic loading")
    if ort_lib not in output:
        raise RuntimeError(f"{binary} does not reference the bundled ONNX Runtime directory in its dynamic tags")


def assert_resolves_bundled_onnxruntime(binary: Path, ort_lib: str, env: dict[str, str]) -> None:
    ldd_output = capture(["ldd", str(binary)], env=env)
    match = re.search(r"libonnxruntime\.so\.1\s+=>\s+(\S+)", ldd_output)
    if not match:
        raise RuntimeError("ldd did not report libonnxruntime.so.1 for the smoke binary")
    resolved = Path(match.group(1)).resolve()
    expected = (Path(ort_lib) / "libonnxruntime.so.1").resolve()
    if resolved != expected:
        raise RuntimeError(f"smoke binary resolves libonnxruntime.so.1 to {resolved}, expected {expected}")


def assert_source_wiring() -> None:
    cmake_text = (ROOT / "CMakeLists.txt").read_text()
    for fragment in [
        'INSTALL_RPATH "$ORIGIN;$ORIGIN/.."',
        'INSTALL_RPATH_USE_LINK_PATH FALSE',
        'BUILD_RPATH "${ONNXRUNTIME_LIB_DIR};$ORIGIN"',
        'install(FILES',
        'libonnxruntime_providers_shared.so',
    ]:
        if fragment not in cmake_text:
            raise RuntimeError(f"CMakeLists.txt is missing required fragment: {fragment}")

    launch_text = (ROOT / "launch/voxelnav.launch.py").read_text()
    for fragment in [
        'SetEnvironmentVariable("LD_LIBRARY_PATH"',
        'SetEnvironmentVariable("LD_PRELOAD"',
        '/camera/camera/depth/color/points',
        '/camera/camera/color/image_raw',
        '/camera/camera/depth/image_rect_raw',
        '/camera/camera/color/camera_info',
    ]:
        if fragment not in launch_text:
            raise RuntimeError(f"launch/voxelnav.launch.py is missing required fragment: {fragment}")

    node_text = (ROOT / "src/voxelnav_node.cpp").read_text()
    for fragment in [
        'cloud_topic_',
        'rgb_topic_',
        'depth_topic_',
        'camera_info_topic_',
    ]:
        if fragment not in node_text:
            raise RuntimeError(f"src/voxelnav_node.cpp is missing required fragment: {fragment}")


def main() -> int:
    BUILD_DIR.mkdir(exist_ok=True)
    assert_source_wiring()

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
    smoke_env["LD_LIBRARY_PATH"] = sanitize_ld_library_path(smoke_env.get("LD_LIBRARY_PATH"), ort_lib)
    smoke_env.pop("LD_PRELOAD", None)
    smoke_env.pop("LD_AUDIT", None)

    with tempfile.TemporaryDirectory(prefix="onnxruntime-conflict-", dir=str(BUILD_DIR)) as temp_dir:
        conflict_dir = Path(temp_dir)
        shutil.copy2(Path(ort_lib) / "libonnxruntime.so.1", conflict_dir / "libonnxruntime.so.1")
        conflict_env = smoke_env.copy()
        conflict_env["LD_LIBRARY_PATH"] = str(conflict_dir)

        assert_linker_tag(SMOKE_BIN, ort_lib)
        assert_resolves_bundled_onnxruntime(SMOKE_BIN, ort_lib, conflict_env)
        run([str(SMOKE_BIN)], env=conflict_env)

    print()
    print(f"Build verification succeeded: {SMOKE_BIN}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
