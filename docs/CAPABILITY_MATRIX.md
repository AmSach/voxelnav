# VoxelNav capability matrix

| Capability | Status | Notes |
|------------|--------|------|
| Point-cloud voxelization | Working | Core `Voxelizer` processes 3D points into hashed voxels. |
| Signed 3D coordinates | Working | Negative and positive coordinates are supported. |
| Semantic labels | Working in fallback mode | Labels and confidence are stored, but the default segmenter is deterministic mock behavior. |
| Real ONNX inference | Not complete | The repo does not yet ship a validated trained model pipeline. |
| Nav2 costmap plugin | Scaffold | The plugin exists, but end-to-end robot validation is still needed. |
| Smoke testing | Working | `python3 verify_build.py` compiles and runs the core smoke test. |
| Voxel ranking | Working | Top-confidence voxels can be selected. |
| Label histogram | Working | The voxelizer can count labels. |
| Stale voxel pruning | Working | Weak or old voxels can be removed. |
| Real robot field validation | Not done | No target-hardware robot test campaign has been completed here. |

## Practical summary
VoxelNav is currently best described as a **highly capable prototype** rather than a field-deployed system.
