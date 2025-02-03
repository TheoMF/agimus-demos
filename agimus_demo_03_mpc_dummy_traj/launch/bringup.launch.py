from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,
)

from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_franka_launch,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    franka_robot_launch = generate_include_franka_launch()

    lfc_params = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_03_mpc_dummy_traj"),
            "config",
            "lfc_params.yaml",
        ]
    )

    load_joint_state_estimator = generate_controllers_spawner_launch_description(
        controller_names=["joint_state_estimator", "linear_feedback_controller"],
        controller_params_file=lfc_params,
    )

    agimus_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_03_mpc_dummy_traj"),
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

    simple_trajectory_publisher_node = (
        Node(
            package="agimus_controller_ros",
            executable="simple_trajectory_publisher",
            name="simple_trajectory_publisher",
            output="screen",
        ),
    )

    return [
        franka_robot_launch,
        load_joint_state_estimator,
        agimus_controller_node,
        simple_trajectory_publisher_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_franka_args() + [OpaqueFunction(function=launch_setup)]
    )
