# VoxelNav deployment readiness

## Short answer
No: this repository is **not yet field deployable** in the strict sense.

It is a strong research/demo prototype with real core data structures and a test harness, but field deployment requires robot-side validation, a real segmentation model, and end-to-end ROS2 integration on the target hardware.

## What is already useful
- 3D voxel hashing from point clouds
- Signed voxel coordinates in all axes
- Semantic label storage and confidence tracking
- Deterministic fallback segmentation mode
- Nav2 costmap plugin scaffold
- Smoke-testable core logic
- Voxel pruning, ranking, and histogram helpers

## What is still missing for real field deployment
- A validated ONNX model that matches the claimed semantic classes
- Real robot integration tests in ROS2
- TF / frame correctness checks on the target robot
- Playback tests against recorded sensor bags
- Runtime monitoring, watchdogs, and lifecycle management
- Performance validation on the actual target CPU/GPU
- Long-run stability testing under sensor noise, dropouts, and memory pressure

## Recommended deployment path
1. **Simulation / bag replay**
   - Verify that the package runs on recorded sensor data.
2. **Hardware-in-the-loop**
   - Run the stack on the actual compute target without driving motors.
3. **Safety-limited robot tests**
   - Low-speed indoor trials with emergency stop support.
4. **Field validation**
   - Only after the map quality, latency, and stability are measured repeatedly.

## Honest marketing position
You can describe VoxelNav as:
- a **ROS2 semantic voxel mapping prototype**
- a **navigation-oriented voxel mapping stack**
- a **field-test candidate**

Do **not** describe it as production-deployed or fully field proven unless those checks are actually completed.
