import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_dog_description")

    xacro_file = os.path.join(pkg_share, "xacro", "robot.xacro")
    world_file = os.path.join(pkg_share, "worlds", "single_leg_empty.world")

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(pkg_share, "..")
    )

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str
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
            "gz_args": world_file
        }.items()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }],
        output="screen"
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "robot_dog_single_leg",
            "-x", "0",
            "-y", "0",
            "-z", "1.0"
        ],
        output="screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "front_right_leg_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        robot_state_publisher,

        TimerAction(
            period=2.0,
            actions=[spawn_robot]
        ),

        TimerAction(
            period=6.0,
            actions=[
                joint_state_broadcaster_spawner,
                leg_controller_spawner
            ]
        ),
    ])
