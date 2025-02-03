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
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    arm_id = LaunchConfiguration("arm_id")
    robot_ip = LaunchConfiguration("robot_ip")
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_rviz = LaunchConfiguration("use_rviz")
    verbose = LaunchConfiguration("verbose")
    headless = LaunchConfiguration("headless")

    franka_hardware_launch = IncludeLaunchDescription(
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
            "robot_ip": robot_ip,
            "use_gazebo": use_gazebo,
            "use_rviz": use_rviz,
            "verbose": verbose,
            "headless": headless,
        }.items(),
    )

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
        franka_hardware_launch,
        load_joint_state_estimator,
        agimus_controller_node,
        simple_trajectory_publisher_node,
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
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware",
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description='Fake sensor commands. Only valid when "use_fake_hardware" is true',
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Visualize the robot in RViz",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
