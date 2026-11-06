
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # ==================================================
    # 1. Camera PointCloud2 bridge
    # ==================================================

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_pointcloud_bridge",
        arguments=[
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"
        ],
        output="screen",
    )

    # ==================================================
    # 2. Static TF: camera_link -> Gazebo sensor frame
    # ==================================================

    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_sensor_static_tf",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "camera_link",
            "--child-frame-id", "retreival_robot/base_link/rgbd_camera",
        ],
        output="screen",
    )

    # ==================================================
    # 3. Cylinder perception detector
    # ==================================================

    cylinder_detector = Node(
        package="retreival_robot",
        executable="cylinder_detector.py",
        name="cylinder_detector",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "target_frame": "base_link",
                "voxel_size": 0.025,
                "cluster_radius": 0.10,
                "min_range": 0.15,
                "max_range": 100.0,
                "min_height": -0.25,
                "max_height": 1.5,
            }
        ],
    )

    # ==================================================
    # 4. Start perception pipeline
    # No test cylinder spawning
    # ==================================================

    return LaunchDescription([
        camera_bridge,
        camera_tf,
        cylinder_detector,
    ])
