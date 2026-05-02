# VoxelNav, explained like a tiny story

VoxelNav is a robot helper that turns messy sensor dots into tidy little cubes called voxels.

It is still learning, so it is a **practice robot brain**, not a finished robot brain.

## What it tries to do
- See the world from a robot's eyes
- Put the world into cubes
- Give those cubes names like floor, wall, or chair
- Help the robot avoid bad stuff

## The 3 main pieces
1. **Voxelizer**
   - Takes 3D points
   - Groups nearby points into cubes
   - Remembers how many points landed there

2. **Segmenter**
   - Looks at pictures from the robot camera
   - Gives each pixel a simple label
   - Uses a safe fallback if the real model is missing

3. **Node**
   - Listens to the robot's sensors
   - Sends the information into the voxelizer and segmenter
   - Shares the finished map with navigation software

## Why this is cool
Because the robot does not just see dots.
It starts to understand what the dots mean.

## The grown-up summary
It is a ROS2 semantic voxel mapping stack with a tested core, signed space, and navigation-oriented outputs.
