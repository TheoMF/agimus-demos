from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
    RegisterEventHandler,
)
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    arm_id = LaunchConfiguration("arm_id")
    robot_ip = LaunchConfiguration("robot_ip")
    franka_controllers_params = LaunchConfiguration("franka_controllers_params")

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            franka_controllers_params,
            {"arm_id": arm_id},
            {"load_gripper": "true"},
        ],
        remappings=[("joint_states", "franka/joint_states")],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        on_exit=Shutdown(),
    )

    set_robot_description_node = Node(
        package="agimus_demos_common",
        executable="set_robot_description_node",
        name="set_robot_description_node",
        parameters=[
            {
                "nodes_to_set": [
                    "/gz_ros2_control",
                ],
            }
        ],
    )

    spawn_default_controllers = generate_controllers_spawner_launch_description(
        [
            "joint_state_broadcaster",
            "franka_robot_state_broadcaster",
        ]
    )

    franka_gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("franka_gripper"),
                        "launch",
                        "gripper.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": "false",
        }.items(),
    )

    return [
        controller_manager_node,
        franka_gripper_launch,
        set_robot_description_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=set_robot_description_node,
                on_exit=[spawn_default_controllers],
            )
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            description="Hostname or IP address of the robot.",
        ),
        DeclareLaunchArgument(
            "arm_id",
            default_value="fer",
            description="ID of the type of arm used. Supported values: fer, fr3, fp3",
            choices=["fer", "fr3", "fp3"],
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
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
