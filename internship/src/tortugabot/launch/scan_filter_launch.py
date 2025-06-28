#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    min_range_arg = DeclareLaunchArgument(
        'min_range',
        default_value='0.1',
        description='Minimum range to keep (meters)'
    )
    
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='10.0',
        description='Maximum range to keep (meters)'
    )
    
    angular_resolution_arg = DeclareLaunchArgument(
        'angular_resolution',
        default_value='0.02',
        description='Target angular resolution in radians (0.02 rad ≈ 1.15°)'
    )
    
    median_filter_size_arg = DeclareLaunchArgument(
        'median_filter_size',
        default_value='3',
        description='Size of median filter window'
    )
    
    pole_reflection_threshold_arg = DeclareLaunchArgument(
        'pole_reflection_threshold',
        default_value='0.05',
        description='Distance threshold to remove pole reflections (meters)'
    )
    
    # Scan filter node
    scan_filter_node = Node(
        package='tortugabot',  # Adjust package name as needed
        executable='scan_filter.py',
        name='scan_filter',
        output='screen',
        parameters=[{
            'min_range': LaunchConfiguration('min_range'),
            'max_range': LaunchConfiguration('max_range'),
            'angular_resolution': LaunchConfiguration('angular_resolution'),
            'median_filter_size': LaunchConfiguration('median_filter_size'),
            'pole_reflection_threshold': LaunchConfiguration('pole_reflection_threshold'),
        }],
        remappings=[
            # Uncomment and adjust if you need to remap topics
            # ('/scan', '/your_scan_topic'),
            # ('/scan_filtered', '/your_filtered_topic'),
        ]
    )
    
    return LaunchDescription([
        min_range_arg,
        max_range_arg,
        angular_resolution_arg,
        median_filter_size_arg,
        pole_reflection_threshold_arg,
        scan_filter_node,
    ])