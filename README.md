# VoxelNav — Real-time Semantic Voxel Mapping for ROS2

**Lightweight real-time 3D semantic voxel mapping for ROS2 robots in under 100ms — because waiting for SLAM is so 2024.**

---

## Overview

VoxelNav is a ROS2-native node that converts LiDAR and RGB-D sensor data into compact semantic voxel grids optimized for navigation. It runs on Jetson Nano-level hardware, not expensive workstation GPUs.

**Key metrics:**
- **100ms end-to-end latency** on Jetson Nano (30ms voxelization + 70ms segmentation)
- **Works with any ROS2-compatible LiDAR + RGB-D camera**
- **Outputs directly to Nav2-compatible costmap layers**

---

## Installation

### Prerequisites

- ROS2 Humble or Jazzy
- Ubuntu 22.04
- OpenCV 4.x
- Eigen3
- ONNX Runtime (for semantic mode)

### Build from Source

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/AmSach/voxelnav.git

# Install dependencies
sudo apt install libeigen3-dev libopencv-dev

# Build
cd ~/ros2_ws
colcon build --packages-select voxelnav

# Source
source install/setup.bash
```

---

## Usage

### Quick Start

```bash
# Launch with default parameters
ros2 launch voxelnav voxelnav.launch.py

# Launch with custom voxel size
ros2 launch voxelnav voxelnav.launch.py voxel_size:=0.1

# Geometry-only mode (faster, no semantic labels)
ros2 launch voxelnav voxelnav.launch.py enable_semantic:=false
```

### Topics

**Subscribed:**
- `/scan` (sensor_msgs/PointCloud2) — LiDAR point cloud
- `/camera/color/image_raw` (sensor_msgs/Image) — RGB image
- `/camera/depth/image_rect_raw` (sensor_msgs/Image) — Depth image
- `/camera/color/camera_info` (sensor_msgs/CameraInfo) — Camera calibration

**Published:**
- `/voxel_map` (sensor_msgs/PointCloud2) — Voxel grid as point cloud
- `/semantic_voxel_map` (sensor_msgs/PointCloud2) — Semantic-labeled voxels

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `voxel_size` | double | 0.05 | Voxel size in meters |
| `max_range` | double | 10.0 | Maximum sensor range in meters |
| `enable_semantic` | bool | true | Enable semantic segmentation |
| `publish_rate` | double | 10.0 | Voxel map publish rate (Hz) |
| `frame_id` | string | "map" | TF frame for voxel map |
| `model_path` | string | "models/mobilenet_v3_small.onnx" | Path to ONNX model |

---

## Nav2 Integration

VoxelNav includes a Nav2 costmap plugin for seamless navigation integration.

### Configuration

Add to your Nav2 costmap config:

```yaml
plugins:
  - name: voxel_layer
    type: "voxelnav::VoxelCostmapPlugin"

voxel_layer:
  plugin: voxelnav/VoxelCostmapPlugin
  enabled: true
  voxel_size: 0.05
  obstacle_threshold: 0.5
  filter_labels: [1]  # Ignore floor voxels
```

---

## Architecture

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   LiDAR     │    │   RGB-D     │    │   Camera    │
│   /scan     │    │   Camera    │    │   Info      │
└──────┬──────┘    └──────┬──────┘    └──────┬──────┘
       │                  │                  │
       ▼                  ▼                  ▼
┌──────────────────────────────────────────────────┐
│                 VoxelNav Node                     │
│  ┌────────────┐    ┌────────────┐                 │
│  │ Voxelizer  │◄───│ Segmenter  │ (semantic mode) │
│  │            │    │ (MobileNet)│                 │
│  └─────┬──────┘    └────────────┘                 │
│        │                                          │
│        ▼                                          │
│  ┌────────────┐    ┌────────────┐                 │
│  │   Voxel    │    │   Nav2     │                 │
│  │   Grid     │───►│ Costmap    │                 │
│  │ (Hash Map) │    │  Plugin    │                 │
│  └────────────┘    └────────────┘                 │
└──────────────────────────────────────────────────┘
       │
       ▼
┌─────────────┐
│ /voxel_map  │
└─────────────┘
```

---

## Semantic Classes

VoxelNav supports the following semantic labels:

| ID | Class | Color |
|----|-------|-------|
| 0 | Unknown | Gray |
| 1 | Floor | Purple |
| 2 | Wall | Blue |
| 3 | Ceiling | Yellow |
| 4 | Door | Green |
| 5 | Window | Cyan |
| 6 | Furniture | Orange |
| 7 | Person | Red |
| 8 | Vehicle | Magenta |
| 9 | Plant | Spring Green |
| 10 | Stairs | Brown |
| 11 | Table | Light Blue |
| 12 | Chair | Light Green |
| 13 | Box | Light Red |

---

## Performance

### Benchmarks (Jetson Nano)

| Mode | Latency | Memory |
|------|---------|--------|
| Geometry only | 30ms | 50MB |
| Full semantic | 100ms | 150MB |

### Comparison

| System | Latency | Hardware | Semantics |
|--------|---------|----------|-----------|
| VoxelNav | 100ms | Jetson Nano | ✓ |
| OpenGS-SLAM | 30s | A100 GPU | ✓ |
| OctoMap | 500ms | Desktop | ✗ |
| RTABMap | 1s | Desktop | ✗ |

---

## Development

### Run Tests

```bash
cd ~/ros2_ws
colcon test --packages-select voxelnav
colcon test-result --verbose
```

### Project Structure

```
voxelnav/
├── src/
│   ├── voxelnav_node.cpp    # Main ROS2 node
│   ├── voxelizer.cpp        # Core voxel hashing
│   ├── segmenter.cpp        # MobileNet inference
│   └── costmap_plugin.cpp   # Nav2 costmap layer
├── include/voxelnav/
│   ├── voxelizer.hpp
│   └── segmenter.hpp
├── config/
│   └── default_params.yaml
├── launch/
│   └── voxelnav.launch.py
├── test/
│   └── test_voxelizer.cpp
└── models/
    └── mobilenet_v3_small.onnx (download separately)
```

---

## License

MIT License - See [LICENSE](LICENSE) for details.

---

## Author

**Aman Sachan**
- Email: amansachan92905@gmail.com
- GitHub: [@AmSach](https://github.com/AmSach)

---

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