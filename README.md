# VoxelNav — Real-time Semantic Voxel Mapping for ROS2

VoxelNav turns 3D sensor data into a voxel grid and can attach semantic labels to those voxels. The point of this repo is now simple: show the real working core, clearly mark the unfinished parts, and give you a test you can run right away.

## Read this first
This is a **strong prototype**. It is **not yet field deployed**.

You can use it to demonstrate voxel mapping, semantic labeling, ranking, pruning, and Nav2 integration scaffolding. You should **not** claim it is production-proven on a real robot until the deployment checklist in `docs/DEPLOYMENT.md` is actually completed.

## What is real today
- Voxel hashing and merging from 3D points
- Signed voxel coordinates in all 3 axes
- Semantic labels and confidence values
- Deterministic segmentation fallback when a real ONNX model is missing
- Nav2 costmap plugin scaffold
- Core smoke test that exercises the main library paths
- Extra map-intelligence helpers like label histograms, top-confidence voxels, and stale-voxel pruning

## What is still a demo or scaffold
- The ONNX segmenter is still a mock fallback, not a real trained model pipeline
- The ROS2 node depends on a full ROS2 + PCL + OpenCV environment
- The Nav2 plugin needs a real robot setup to validate end-to-end behavior

## Quick verification
If you just want to check that the core logic works, run:

```bash
python3 verify_build.py
```

That compiles the core smoke test and runs it. It checks:
- voxel creation
- signed coordinates
- semantic labels
- bounds queries
- dynamic change detection
- stale voxel pruning
- segmentation fallback output

## If you want to build inside ROS2
This package expects a ROS2 workspace with the usual dependencies installed. The exact ROS2 build is not verified in this sandbox, so treat the instructions below as the intended setup rather than a fully reproduced guarantee.

```bash
cd ~/ros2_ws/src
git clone https://github.com/AmSach/voxelnav.git
cd ~/ros2_ws
colcon build --packages-select voxelnav
source install/setup.bash
```

## Usage

### Launch

```bash
ros2 launch voxelnav voxelnav.launch.py
ros2 launch voxelnav voxelnav.launch.py voxel_size:=0.1
ros2 launch voxelnav voxelnav.launch.py enable_semantic:=false
```

### Topics

**Subscribed:**
- `/scan` (sensor_msgs/PointCloud2)
- `/camera/color/image_raw` (sensor_msgs/Image)
- `/camera/depth/image_rect_raw` (sensor_msgs/Image)
- `/camera/color/camera_info` (sensor_msgs/CameraInfo)

**Published:**
- `/voxel_map` (sensor_msgs/PointCloud2)
- `/semantic_voxel_map` (sensor_msgs/PointCloud2)

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `voxel_size` | double | 0.05 | Voxel size in meters |
| `max_range` | double | 10.0 | Maximum sensor range in meters |
| `enable_semantic` | bool | true | Enable semantic segmentation |
| `publish_rate` | double | 10.0 | Voxel map publish rate (Hz) |
| `frame_id` | string | `map` | TF frame for voxel map |
| `model_path` | string | `models/mobilenet_v3_small.onnx` | ONNX model path |

## Nav2 Integration

This is the intended plugin shape:

```yaml
plugins:
  - name: voxel_layer
    type: "voxelnav::VoxelCostmapPlugin"

voxel_layer:
  plugin: voxelnav/VoxelCostmapPlugin
  enabled: true
  voxel_size: 0.05
  obstacle_threshold: 0.5
  filter_labels: [1]
```

## Core architecture

```text
LiDAR / RGB-D -> VoxelNav Node -> Voxel Grid -> Nav2 Costmap
                       |
                       +--> Semantic labels
                       +--> Confidence scoring
                       +--> Stale pruning / ranking helpers
```

## Test command

```bash
python3 verify_build.py
```

## Detailed docs
- `docs/ARCHITECTURE.md` — what each part does and where it still falls short
- `docs/TESTING.md` — what the smoke test covers and how to extend it
- `docs/README_TODDLER.md` — the simplest explanation possible
- `docs/MARKETING.md` — honest sales language and demo framing

## License

MIT License - See [LICENSE](LICENSE) for details.

## Author

**Aman Sachan**
- Email: amansachan92905@gmail.com
- GitHub: [@AmSach](https://github.com/AmSach)

## Citation

If you use VoxelNav in your research, please cite:

```bibtex
@software{voxelnav2024,
  author = {Sachan, Aman},
  title = {VoxelNav: Real-time Semantic Voxel Mapping for ROS2},
  year = {2024},
  url = {https://github.com/AmSach/voxelnav}
}
```