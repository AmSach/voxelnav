# VoxelNav marketing sheet

## One-liner
VoxelNav is a ROS2 semantic voxel mapping stack with signed-space hashing, deterministic semantic fallback, and a tested core designed for Nav2 integration.

## Short pitch
It takes raw 3D sensor data, turns it into structured voxels, assigns semantics, and exposes the result in a form navigation software can actually use.

## What makes it feel advanced
- Signed 3D voxel space, not just a toy first-quadrant grid
- Semantic labels plus confidence scores
- Deterministic fallback when the real model is missing
- Helper functions for ranking, pruning, and summarizing voxels
- A smoke test that proves the core paths work
- Nav2 plugin scaffold for navigation use

## Honest marketing language
Use these phrases:
- "tested semantic voxel mapping core"
- "signed-space voxel hashing"
- "deterministic segmentation fallback"
- "Nav2-oriented voxel intelligence"
- "navigation-ready voxel pipeline"

Avoid these phrases unless a real benchmark or robot demo proves them:
- "production ready"
- "fully autonomous"
- "sub-100ms" unless measured on target hardware
- "fully trained model pipeline" unless the ONNX model is actually shipped and validated

## Demo story
1. Show live sensor input.
2. Show the voxel map filling in.
3. Show labels and confidence values.
4. Show the smoke test passing.
5. Show that the map can be summarized, ranked, and pruned.

## Best description for the README
"VoxelNav turns 3D sensor data into a signed semantic voxel grid with deterministic fallback behavior and Nav2-facing outputs."
