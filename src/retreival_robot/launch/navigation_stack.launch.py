#!/usr/bin/env python3
"""Start perception and map/navigation; Gazebo+odom are started by existing robot launch."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    root=get_package_share_directory('retreival_robot')
    p=lambda *parts:os.path.join(root,*parts)
    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value='2.5'),
        DeclareLaunchArgument('goal_y', default_value='0.5'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='camera_pointcloud_bridge',
             arguments=['/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],output='screen'),
        Node(package='tf2_ros',executable='static_transform_publisher',name='camera_sensor_static_tf',
             arguments=['--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0',
                        '--frame-id','camera_link','--child-frame-id','retreival_robot/base_link/rgbd_camera'],
             output='screen'),
        Node(package='retreival_robot',executable='cylinder_detector.py',name='cylinder_detector',
             parameters=[{'use_sim_time':True,'target_frame':'base_link','voxel_size':0.025,
                          'cluster_radius':0.10,'min_range':0.15,'max_range':100.0,
                          'min_height':-0.25,'max_height':1.5}],output='screen'),
        Node(package='retreival_robot',executable='map_publisher.py',name='static_map_publisher',
             parameters=[{'use_sim_time':True,'map_yaml':p('maps','retrieval_landmarks.yaml')}],output='screen'),
        Node(package='retreival_robot',executable='landmark_localizer.py',name='landmark_localizer',
             parameters=[{'use_sim_time':True,'landmarks_json':p('maps','landmarks.json'),
                          'initial_x':0.,'initial_y':0.,'initial_yaw':0.,
                          'association_gate_m':0.75,'measurement_std_m':0.12}],output='screen'),
        Node(package='retreival_robot',executable='motion_planner.py',name='motion_planner',
             parameters=[{'use_sim_time':True,'robot_clearance_m':1.5}],output='screen'),
        Node(package='retreival_robot',executable='navigator.py',name='navigator',
             parameters=[{'use_sim_time':True,'wheel_radius':0.1132,'wheel_separation':0.76}],output='screen'),
        Node(package='retreival_robot',executable='startup_goal.py',name='startup_goal',
             parameters=[{'use_sim_time':True,
                          'goal_x':ParameterValue(LaunchConfiguration('goal_x'),value_type=float),
                          'goal_y':ParameterValue(LaunchConfiguration('goal_y'),value_type=float),
                          'goal_yaw':ParameterValue(LaunchConfiguration('goal_yaw'),value_type=float)}],output='screen'),
    ])
