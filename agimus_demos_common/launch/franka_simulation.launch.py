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

from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    arm_id = LaunchConfiguration("arm_id")
    franka_controllers_params = LaunchConfiguration("franka_controllers_params")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_path = LaunchConfiguration("rviz_config_path")
    verbose = LaunchConfiguration("verbose")
    headless = LaunchConfiguration("headless")

    franka_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demos_common"),
                        "launch",
                        "franka_common.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "arm_id": arm_id,
            "robot_ip": "",
            "use_gazebo": "true",
            "load_gripper": "true",
            "use_fake_hardware": "false",
            "fake_sensor_commands": "false",
            "franka_controllers_params": franka_controllers_params,
            "use_rviz": use_rviz,
            "rviz_config_path": rviz_config_path,
        }.items(),
    )

    verbose_bool = context.perform_substitution(verbose).lower() == "true"
    headless_bool = context.perform_substitution(headless).lower() == "true"
    gz_gui_config_path_str = context.perform_substitution(
        PathJoinSubstitution(
            [
                FindPackageShare("agimus_demos_common"),
                "config",
                "gz_gui.config",
            ]
        )
    )

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
            + f" {'-s' if headless_bool else ''}"
            + f" {'-v 3' if verbose_bool else ''}"
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
        franka_common_launch,
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
            "arm_id",
            default_value="fer",
            description="ID of the type of arm used. Supported values: fer, fr3, fp3",
        ),
        DeclareLaunchArgument(
            "franka_controllers_params",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "config",
                    "franka_controllers.yaml",
                ]
            ),
            description="Path to the yaml file used to define controller parameters.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Visualize the robot in RViz",
        ),
        DeclareLaunchArgument(
            "rviz_config_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "rviz",
                    "franka_preview.rviz",
                ]
            ),
            description="Path to RViz configuration file",
        ),
        DeclareLaunchArgument(
            "verbose",
            default_value="false",
            description="Wether to set verbosity level of Gazebo to 3.",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Wether to launch Gazebo in headless mode (no GUI is launched, only physics server).",
        ),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
