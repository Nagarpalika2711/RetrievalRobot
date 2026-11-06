
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = "retreival_robot"

    package_share = get_package_share_directory(package_name)

    # ======================================================
    # GRAPHICS / FLICKER FIX
    # ======================================================

    software_rendering = SetEnvironmentVariable(
        name="LIBGL_ALWAYS_SOFTWARE",
        value="1"
    )

    mesa_gl_version = SetEnvironmentVariable(
        name="MESA_GL_VERSION_OVERRIDE",
        value="3.3"
    )

    mesa_glsl_version = SetEnvironmentVariable(
        name="MESA_GLSL_VERSION_OVERRIDE",
        value="330"
    )

    # ======================================================
    # GAZEBO RESOURCE PATH
    # ======================================================

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join(
            filter(
                None,
                [
                    os.path.dirname(package_share),
                    os.environ.get("GZ_SIM_RESOURCE_PATH", "")
                ]
            )
        )
    )

    # ======================================================
    # ROBOT DESCRIPTION
    # ======================================================

    xacro_file = os.path.join(
        package_share,
        "urdf",
        "retreival_robot.urdf.xacro"
    )

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str
    )

    # ======================================================
    # GAZEBO WORLD
    # ======================================================

    world_file = os.path.join(
        package_share,
        "worlds",
        "retrieval_landmarks.sdf"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": f"-r {world_file}"
        }.items()
    )

    # ======================================================
    # GAZEBO CLOCK BRIDGE
    # ======================================================

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_clock_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ]
    )

    # ======================================================
    # ROBOT STATE PUBLISHER
    # ======================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True
            }
        ]
    )

    # ======================================================
    # SPAWN ROBOT IN GAZEBO
    # ======================================================

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_retreival_robot",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "retreival_robot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.5"
        ]
    )

    # ======================================================
    # JOINT STATE BROADCASTER
    # ======================================================

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # ======================================================
    # WHEEL VELOCITY CONTROLLER
    # ======================================================

    wheel_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # ======================================================
    # GRIPPER POSITION CONTROLLER
    # ======================================================

    gripper_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_position_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # ======================================================
    # SIMPLE ODOMETRY NODE
    # ======================================================

    simple_odometry = Node(
        package="retreival_robot",
        executable="simple_odometry.py",
        name="simple_odometry",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,

                # Wheel geometry in meters
                "wheel_radius": 0.1132,
                "wheel_separation": 0.76,

                # Wheel joint names from URDF
                "left_joint": "left_wheel_joint",
                "right_joint": "right_wheel_joint",
            }
        ]
    )

    # ======================================================
    # START CONTROLLERS AFTER ROBOT SPAWNING
    # ======================================================

    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster,
                wheel_velocity_controller,
                gripper_position_controller
            ]
        )
    )

    # ======================================================
    # START ODOMETRY AFTER JOINT STATE BROADCASTER
    # ======================================================

    start_odometry = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                simple_odometry
            ]
        )
    )

    # ======================================================
    # LAUNCH DESCRIPTION
    # ======================================================

    return LaunchDescription([
        software_rendering,
        mesa_gl_version,
        mesa_glsl_version,
        gazebo_resource_path,

        gazebo,
        clock_bridge,

        robot_state_publisher,
        spawn_robot,

        start_controllers,
        start_odometry
    ])
