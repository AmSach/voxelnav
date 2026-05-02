# VoxelNav capability matrix

| Capability | Status | Notes |
|------------|--------|------|
| Point-cloud voxelization | Working | Core `Voxelizer` processes 3D points into hashed voxels. |
| Signed 3D coordinates | Working | Negative and positive coordinates are supported. |
| Semantic labels | Working | Labels and confidence are stored; fallback mode and real-model mode both populate outputs. |
| Real ONNX inference asset | Working | The repo now ships a real ONNX model file at `models/segformer-b0-finetuned-ade-512-512/onnx/model_fp16.onnx`. |
| Real ONNX runtime execution | Partial | The code can use OpenCV DNN for real-model execution if the environment supports it; full ROS2-side validation still needs a target robot. |
| Nav2 costmap plugin | Scaffold | The plugin exists, but end-to-end robot validation is still needed. |
| Smoke testing | Working | `python3 verify_build.py` compiles and runs the core smoke test. |
| Voxel ranking | Working | Top-confidence voxels can be selected. |
| Label histogram | Working | The voxelizer can count labels. |
| Stale voxel pruning | Working | Weak or old voxels can be removed. |
| Real robot field validation | Not done | No target-hardware robot test campaign has been completed here. |

## Practical summary
VoxelNav is currently best described as a **highly capable prototype** with a real model artifact, not a field-deployed system.
