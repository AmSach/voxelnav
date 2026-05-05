from __future__ import annotations

import os
from pathlib import Path


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


def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    pkg_dir = Path(get_package_share_directory("voxelnav"))
    install_prefix = pkg_dir.parents[1]
    bundled_lib_dir = str(install_prefix / "lib" / "voxelnav")
    bundled_onnxruntime = str(Path(bundled_lib_dir) / "libonnxruntime.so.1")
    ld_library_path = sanitize_ld_library_path(os.environ.get("LD_LIBRARY_PATH"), bundled_lib_dir)

    declare_voxel_size = DeclareLaunchArgument(
        "voxel_size",
        default_value="0.05",
        description="Voxel size in meters",
    )
    declare_max_range = DeclareLaunchArgument(
        "max_range",
        default_value="10.0",
        description="Maximum sensor range in meters",
    )
    declare_enable_semantic = DeclareLaunchArgument(
        "enable_semantic",
        default_value="true",
        description="Enable semantic segmentation",
    )
    declare_config_file = DeclareLaunchArgument(
        "config_file",
        default_value=str(pkg_dir / "config" / "default_params.yaml"),
        description="Path to config file",
    )
    declare_cloud_topic = DeclareLaunchArgument(
        "cloud_topic",
        default_value="/camera/camera/depth/color/points",
        description="Point cloud topic for geometry mapping",
    )
    declare_rgb_topic = DeclareLaunchArgument(
        "rgb_topic",
        default_value="/camera/camera/color/image_raw",
        description="RGB image topic",
    )
    declare_depth_topic = DeclareLaunchArgument(
        "depth_topic",
        default_value="/camera/camera/depth/image_rect_raw",
        description="Depth image topic",
    )
    declare_camera_info_topic = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/camera/color/camera_info",
        description="Camera info topic",
    )

    voxelnav_node = Node(
        package="voxelnav",
        executable="voxelnav_node",
        name="voxelnav_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"voxel_size": LaunchConfiguration("voxel_size")},
            {"max_range": LaunchConfiguration("max_range")},
            {"enable_semantic": LaunchConfiguration("enable_semantic")},
            {"cloud_topic": LaunchConfiguration("cloud_topic")},
            {"rgb_topic": LaunchConfiguration("rgb_topic")},
            {"depth_topic": LaunchConfiguration("depth_topic")},
            {"camera_info_topic": LaunchConfiguration("camera_info_topic")},
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("LD_LIBRARY_PATH", ld_library_path),
            SetEnvironmentVariable("LD_PRELOAD", bundled_onnxruntime),
            declare_voxel_size,
            declare_max_range,
            declare_enable_semantic,
            declare_config_file,
            declare_cloud_topic,
            declare_rgb_topic,
            declare_depth_topic,
            declare_camera_info_topic,
            voxelnav_node,
        ]
    )
