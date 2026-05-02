# VoxelNav architecture

This document explains what the repository is trying to do, what each file actually does, and where the current implementation is strong versus still incomplete.

## Big picture
VoxelNav is a ROS2 package that accepts 3D points from sensors, converts them into a voxel grid, optionally attaches semantic labels, and publishes a point-cloud-style map that Nav2 can consume.

## Main pipeline
1. **Point ingestion**
   - `src/voxelnav_node.cpp` subscribes to point clouds and RGB-D topics.
   - It converts incoming data into Eigen vectors and OpenCV matrices.

2. **Voxelization**
   - `src/voxelizer.cpp` turns 3D points into voxels.
   - Voxels are stored in a hash map for fast lookup.
   - Coordinates are signed, so the map can represent space around the robot instead of only the first quadrant.

3. **Semantic labeling**
   - `src/segmenter.cpp` provides a deterministic fallback segmenter.
   - If a real ONNX model is unavailable, the code still produces predictable labels so the rest of the pipeline can be tested.

4. **Costmap integration**
   - `src/costmap_plugin.cpp` is the Nav2 plugin scaffold.
   - It converts voxels into cost values for navigation.

## File-by-file guide
### `include/voxelnav/voxelizer.hpp`
Defines the voxel data model and the public voxelizer API.

### `src/voxelizer.cpp`
Owns voxel creation, updates, pruning, histogramming, ranking, and dynamic-change detection.

### `include/voxelnav/segmenter.hpp`
Defines the segmentation interface, labels, and class-color mapping.

### `src/segmenter.cpp`
Implements a safe fallback segmenter path and post-processing.

### `src/voxelnav_node.cpp`
Wires ROS2 subscriptions, segmentation, voxelization, and publishing together.

### `src/costmap_plugin.cpp`
Nav2 layer scaffold for consuming voxel data.

### `test/core_smoke.cpp`
A no-ROS smoke test that exercises the core logic directly.

### `verify_build.py`
Convenience wrapper that compiles and runs the smoke test.

## What is actually advanced
The repo now has a more serious core than a typical demo because it includes:
- signed voxel space
- deterministic semantic fallback
- confidence-aware ranking
- stale-voxel pruning
- label histograms
- dynamic change detection
- a real smoke test that checks these behaviors

## What still needs work before anyone should call it production-grade
- a real ONNX segmentation pipeline
- more realistic timestamping and occupancy updates
- full ROS2 integration testing
- validated Nav2 plugin behavior on actual robot hardware
- performance benchmarking on target hardware

## Marketing-safe summary
You can honestly describe this as:
> a ROS2 semantic voxel mapping stack with signed-space voxel hashing, deterministic fallback semantics, and a tested core pipeline designed for Nav2 integration.

That is strong. It is also true.
