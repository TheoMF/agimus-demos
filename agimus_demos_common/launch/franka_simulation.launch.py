from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    gz_verbose = LaunchConfiguration("gz_verbose")
    gz_headless = LaunchConfiguration("gz_headless")

    gz_verbose_bool = context.perform_substitution(gz_verbose).lower() == "true"
    gz_headless_bool = context.perform_substitution(gz_headless).lower() == "true"
    gz_gui_config_path_str = context.perform_substitution(
        PathJoinSubstitution(
            [
                FindPackageShare("agimus_demos_common"),
                "config",
                "gz_gui.config",
            ]
        )
    )
    package_name = "agimus_demo_03_mpc_dummy_traj"
    sdf_path = Path(get_package_share_directory(package_name)) / "urdf" / "test.sdf"
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": "empty.sdf -r"
            + f" {'-s' if gz_headless_bool else ''}"
            + f" {'-v 3' if gz_verbose_bool else ''}"
            + f" --gui-config {gz_gui_config_path_str}"
        }.items(),
    )

    robot_spawner_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description"],
        output="screen",
    )

    spawn_default_controllers = generate_controllers_spawner_launch_description(
        [
            "joint_state_broadcaster",
            "gripper_action_controller",
        ]
    )

    return [
        gazebo_empty_world,
        robot_spawner_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_spawner_node,
                on_exit=[spawn_default_controllers],
            )
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gz_verbose",
            default_value="false",
            description="Wether to set verbosity level of Gazebo to 3.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "gz_headless",
            default_value="false",
            description="Wether to launch Gazebo in headless mode (no GUI is launched, only physics server).",
            choices=["true", "false"],
        ),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
