#!/usr/bin/env python3
"""Start one Gazebo instance plus the entire map navigation stack and automatic goal."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('retreival_robot'), 'launch')
    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value='2.5'),
        DeclareLaunchArgument('goal_y', default_value='0.5'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'gazebo.retreival.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'navigation_stack.launch.py')),
            launch_arguments={
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'goal_yaw': LaunchConfiguration('goal_yaw'),
            }.items()),
    ])
