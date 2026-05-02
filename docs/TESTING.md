# VoxelNav testing

## What the smoke test covers
The smoke test in `test/core_smoke.cpp` verifies:
- voxel creation from 3D points
- signed coordinates in negative and positive space
- semantic labels and confidence values
- label histograms
- confidence ranking
- bounded voxel queries
- dynamic change detection
- pruning stale or weak voxels
- segmentation fallback with both float and 16-bit depth images

## How to run it
```bash
python3 verify_build.py
```

## What the result means
If it passes, the core math and data-flow are working.
If it fails, the repository should not be marketed as working yet.

## What is not verified here
- ROS2 topic wiring
- live camera integration
- Nav2 plugin execution inside a full robot stack
- real ONNX model inference
- hardware performance claims

## How to extend testing next
If you want this to feel truly advanced, the next upgrades should be:
1. add ROS2 launch tests
2. add a recorded sensor bag replay test
3. add a Nav2 integration test
4. add benchmark reporting
5. compare fallback semantics against a real model
