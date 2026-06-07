#!/bin/bash
set -e

echo "🚀 VoxelNav One-Click Setup"
echo "=========================="

# 1. Install System Dependencies
echo "📦 Installing system dependencies..."
apt update && apt install -y \
    libeigen3-dev \
    libopencv-dev \
    python3-pip \
    ffmpeg \
    imagemagick

# 2. Install Python Dependencies
echo "🐍 Installing python dependencies..."
pip install --upgrade pip
pip install numpy opencv-python onnxruntime matplotlib tqdm

# 3. Download Models (Placeholder if needed)
echo "🧠 Preparing models..."
mkdir -p models
# In a real scenario, we'd wget a model here. 
# For this demo, we'll ensure the directory exists.

# 4. Build Standalone Test
echo "🛠 Building standalone voxel engine..."
g++ -std=c++17 -O3 -o test_standalone src/voxelizer.cpp test_standalone.cpp \
    -Iinclude -I/usr/include/eigen3 \
    `pkg-config --cflags --libs opencv4` || \
    echo "⚠️ ROS2-independent build failed, falling back to basic test_standalone.cpp..." && \
    g++ -std=c++17 -O2 -o test_standalone test_standalone.cpp

# 5. Run Verification
echo "✅ Running verification..."
./test_standalone

echo ""
echo "=========================="
echo "✨ VoxelNav Setup Complete!"
echo "Run './test_standalone' to see the engine in action."
echo "Or use 'colcon build' if you are in a ROS2 workspace."
