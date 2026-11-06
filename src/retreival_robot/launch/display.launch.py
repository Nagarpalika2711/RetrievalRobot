
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = "retreival_robot"

    package_share = get_package_share_directory(package_name)

    # Path to the robot Xacro file
    xacro_file = os.path.join(
        package_share,
        "urdf",
        "retreival_robot.urdf.xacro"
    )

    # Convert Xacro to URDF
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description}
        ]
    )

    # Joint State Publisher GUI
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )

    # RViz2
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": "1"
        }
    )

    return LaunchDescription([
        robot_state_publisher,
        
        rviz
    ])
