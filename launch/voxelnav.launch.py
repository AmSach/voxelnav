# VoxelNav Launch File

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('voxelnav')
    
    # Declare arguments
    declare_voxel_size = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.05',
        description='Voxel size in meters'
    )
    
    declare_max_range = DeclareLaunchArgument(
        'max_range',
        default_value='10.0',
        description='Maximum sensor range in meters'
    )
    
    declare_enable_semantic = DeclareLaunchArgument(
        'enable_semantic',
        default_value='true',
        description='Enable semantic segmentation'
    )
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'default_params.yaml'),
        description='Path to config file'
    )
    
    # VoxelNav node
    voxelnav_node = Node(
        package='voxelnav',
        executable='voxelnav_node',
        name='voxelnav_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'voxel_size': LaunchConfiguration('voxel_size')},
            {'max_range': LaunchConfiguration('max_range')},
            {'enable_semantic': LaunchConfiguration('enable_semantic')}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/camera/color/image_raw', '/camera/color/image_raw'),
            ('/camera/depth/image_rect_raw', '/camera/depth/image_rect_raw'),
        ]
    )
    
    return LaunchDescription([
        declare_voxel_size,
        declare_max_range,
        declare_enable_semantic,
        declare_config_file,
        voxelnav_node
    ])