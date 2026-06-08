# 🧱 VoxelNav: Semantic Mapping Devlog

> **Session: Real-time Perception at the Edge**
> 
> I finally managed to bridge the gap between raw point clouds and semantic understanding in under 100ms. The biggest challenge wasn't the segmentation—it was the **voxel hashing**. Standard 3D arrays were eating up gigabytes of memory on the Jetson Nano, so I implemented a custom spatial hashing engine in C++ that only allocates memory where there's actually sensor data. 
> 
> Today, I successfully integrated the **MobileNetV3** backbone to classify these voxels in real-time. Watching the system differentiate between a traversable "Floor" and a structural "Wall" while maintaining a 10Hz publish rate is a massive win for the robot's navigation stack.
> 
> I've also just shipped a **Universal 1-Click Setup** and a high-fidelity **Simulation Engine** so you can see the data being "turned and converted" even without a physical LiDAR sensor.
> 
> **Ship Link**: https://man44.zo.space/voxelnav-demo
> **Source**: https://github.com/AmSach/voxelnav
