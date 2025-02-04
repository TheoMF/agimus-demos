from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_franka_launch,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    package_name = "agimus_demo_03_mpc_dummy_traj"
    franka_robot_launch = generate_include_franka_launch()

    agimus_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "mpc_controller_params.yaml",
        ]
    )

    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        name="agimus_controller_node",
        output="screen",
        parameters=[agimus_controller_yaml],
    )

    simple_trajectory_publisher_node = Node(
        package="agimus_controller_ros",
        executable="simple_trajectory_publisher",
        name="simple_trajectory_publisher",
        output="screen",
    )

    xacro_path = (
        Path(get_package_share_directory(package_name)) / "urdf" / "obstacles.xacro"
    )
    environment = Node(
        package="agimus_controller_ros",
        executable="environment_publisher",
        name="environment_publisher",
        output="screen",
        parameters=[{"environment_path": str(xacro_path)}],
    )

    return [
        franka_robot_launch,
        agimus_controller_node,
        simple_trajectory_publisher_node,
        environment,
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_franka_args() + [OpaqueFunction(function=launch_setup)]
    )
