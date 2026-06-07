import numpy as np
import cv2
import os
import subprocess
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def generate_room_data():
    """Generate a simulated 3D room point cloud (Actual Data)"""
    print("Generating simulated room data...")
    points = []
    
    # Floor (z=0)
    x = np.linspace(0, 5, 50)
    y = np.linspace(0, 5, 50)
    X, Y = np.meshgrid(x, y)
    Z = np.zeros_like(X)
    points.append(np.column_stack([X.flatten(), Y.flatten(), Z.flatten()]))
    
    # Walls
    # x=0 wall
    y = np.linspace(0, 5, 50)
    z = np.linspace(0, 3, 30)
    Y, Z = np.meshgrid(y, z)
    X = np.zeros_like(Y)
    points.append(np.column_stack([X.flatten(), Y.flatten(), Z.flatten()]))
    
    # y=0 wall
    x = np.linspace(0, 5, 50)
    z = np.linspace(0, 3, 30)
    X, Z = np.meshgrid(x, z)
    Y = np.zeros_like(X)
    points.append(np.column_stack([X.flatten(), Y.flatten(), Z.flatten()]))
    
    # A "Box" (Obstacle)
    x = np.linspace(2, 3, 10)
    y = np.linspace(2, 3, 10)
    z = np.linspace(0, 1, 10)
    X, Y, Z = np.meshgrid(x, y, z)
    points.append(np.column_stack([X.flatten(), Y.flatten(), Z.flatten()]))
    
    all_points = np.vstack(points)
    # Add some noise
    all_points += np.random.normal(0, 0.02, all_points.shape)
    
    np.save("/home/workspace/voxelnav/test/room_points.npy", all_points)
    return all_points

def visualize_voxels(points, voxel_size=0.1):
    """Simulate voxelization and visualize"""
    print(f"Voxelizing with size {voxel_size}m...")
    
    # Quantize
    voxels = np.round(points / voxel_size).astype(int)
    unique_voxels = np.unique(voxels, axis=0)
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot points (blue)
    ax.scatter(points[::10, 0], points[::10, 1], points[::10, 2], c='blue', alpha=0.1, s=1, label="Raw Point Cloud")
    
    # Plot voxels (green)
    v_points = unique_voxels * voxel_size
    ax.scatter(v_points[:, 0], v_points[:, 1], v_points[:, 2], c='green', s=20, label="Voxel Grid")
    
    ax.set_title("VoxelNav: Raw Data to Voxel Conversion")
    ax.legend()
    
    # Create animation frames
    print("Generating animation frames...")
    os.makedirs("/home/workspace/voxelnav/test/frames", exist_ok=True)
    for angle in range(0, 360, 10):
        ax.view_init(30, angle)
        plt.savefig(f"/home/workspace/voxelnav/test/frames/frame_{angle:03d}.png")
    
    print("Stitching video with ffmpeg...")
    subprocess.run([
        "ffmpeg", "-y", "-framerate", "10", "-i", 
        "/home/workspace/voxelnav/test/frames/frame_%03d.png", 
        "-c:v", "libx264", "-pix_fmt", "yuv420p", 
        "/home/workspace/voxelnav/test/voxelnav_conversion.mp4"
    ])
    
    # Copy to stardance_assets
    os.makedirs("/home/workspace/stardance_assets", exist_ok=True)
    subprocess.run(["cp", "/home/workspace/voxelnav/test/voxelnav_conversion.mp4", "/home/workspace/stardance_assets/voxelnav_demo.mp4"])
    subprocess.run(["cp", "/home/workspace/voxelnav/test/frames/frame_060.png", "/home/workspace/stardance_assets/voxelnav_real.png"])

if __name__ == "__main__":
    os.makedirs("/home/workspace/voxelnav/test", exist_ok=True)
    pts = generate_room_data()
    visualize_voxels(pts)
    print("Done! Demo video saved to stardance_assets/voxelnav_demo.mp4")
