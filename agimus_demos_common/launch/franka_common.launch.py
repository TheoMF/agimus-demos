from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.event_handlers import OnProcessExit

from pathlib import Path
from controller_manager.launch_utils import (
    generate_load_controller_launch_description,
)


from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    arm_id = LaunchConfiguration("arm_id")
    robot_ip = LaunchConfiguration("robot_ip")
    use_gazebo = LaunchConfiguration("use_gazebo")
    franka_controllers_params = LaunchConfiguration("franka_controllers_params")
    linear_feedback_controller_params = LaunchConfiguration(
        "linear_feedback_controller_params"
    )
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_path = LaunchConfiguration("rviz_config_path")
    gz_verbose = LaunchConfiguration("gz_verbose")
    gz_headless = LaunchConfiguration("gz_headless")

    robot_ip_empty = context.perform_substitution(robot_ip) == ""
    use_gazebo_bool = context.perform_substitution(use_gazebo).lower() == "true"
    if robot_ip_empty and not use_gazebo_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Set `robot_ip` to configure hardware or "
            "`gazebo:=true` to configure simulation."
        )

    if not robot_ip_empty and use_gazebo_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`gazebo:=true` and non-empty `robot_ip`."
        )

    franka_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demos_common"),
                        "launch",
                        "franka_hardware.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "arm_id": arm_id,
            "franka_controllers_params": franka_controllers_params,
        }.items(),
        condition=UnlessCondition(use_gazebo),
    )

    franka_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demos_common"),
                        "launch",
                        "franka_simulation.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "gz_verbose": gz_verbose,
            "gz_headless": gz_headless,
        }.items(),
        condition=IfCondition(use_gazebo),
    )

    load_linear_feedback_controller = generate_load_controller_launch_description(
        controller_name="linear_feedback_controller",
        controller_params_file=linear_feedback_controller_params,
        extra_spawner_args=["--controller-manager-timeout", "1000"],
    )

    load_joint_state_estimator = generate_load_controller_launch_description(
        controller_name="joint_state_estimator",
        controller_params_file=linear_feedback_controller_params,
        extra_spawner_args=["--controller-manager-timeout", "1000"],
    )

    arm_id_str = context.perform_substitution(arm_id)
    xacro_args = {
        "robot_ip": robot_ip,
        "arm_id": arm_id,
        "ros2_control": "true",
        "hand": "true",
        "use_fake_hardware": "false",
        "fake_sensor_commands": "false",
        "gazebo": use_gazebo,
        "ee_id": "franka_hand",
        "gazebo_effort": "true",
        "with_sc": "false",
        "franka_controllers_params": franka_controllers_params,
    }
    package_name = "agimus_demo_03_mpc_dummy_traj"
    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("franka_description"),
                        "robots",
                        arm_id_str,
                        f"{arm_id_str}.urdf.xacro",
                        # FindPackageShare(package_name),
                        # "urdf",
                        # "demo2.urdf.xacro",
                    ]
                ),
                # Convert dict to list of parameters
                *[arg for key, val in xacro_args.items() for arg in (f" {key}:=", val)],
            ]
        ),
        value_type=str,
    )

    xacro_collision_args = {
        "robot_ip": robot_ip,
        "arm_id": arm_id,
        "ros2_control": "true",
        "hand": "true",
        "use_fake_hardware": "false",
        "fake_sensor_commands": "false",
        "gazebo": "false",
        "ee_id": "franka_hand",
        "gazebo_effort": "true",
        "with_sc": "true",
        "franka_controllers_params": franka_controllers_params,
    }
    robot_collision_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("franka_description"),
                        "robots",
                        arm_id_str,
                        f"{arm_id_str}.urdf.xacro",
                    ]
                ),
                # Convert dict to list of parameters
                *[
                    arg
                    for key, val in xacro_collision_args.items()
                    for arg in (f" {key}:=", val)
                ],
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    robot_collision_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_collision_publisher",
        output="screen",
        remappings=[("robot_description", "robot_description_with_collision")],
        parameters=[{"robot_description": robot_collision_description}],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {
                "source_list": [
                    "franka/joint_states",
                    f"{arm_id_str}_gripper/joint_states",
                ],
                "rate": 30,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["--display-config", rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    folder_idx = 0
    folder_path = Path(f"/tmp/rosbag_mpc_data_{folder_idx}")
    while folder_path.exists():
        folder_idx += 1
        folder_path = Path(f"/tmp/rosbag_mpc_data_{folder_idx}")
    rosbag_recorder = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            str(folder_path),
            "mpc_debug",
            "mpc_input",
            "ocp_solve_time",
            "ocp_x0",
        ],
        output="screen",
    )

    return [
        franka_hardware_launch,
        franka_simulation_launch,
        load_linear_feedback_controller,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_linear_feedback_controller.entities[-1],
                on_exit=[load_joint_state_estimator],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_linear_feedback_controller.entities[-1],
                on_exit=[rosbag_recorder],
            )
        ),
        robot_state_publisher_node,
        robot_collision_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "franka_controllers_params",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "config",
                    "franka_controllers.yaml",
                ]
            ),
            description="Path to the yaml file use to define controller parameters.",
        ),
        DeclareLaunchArgument(
            "linear_feedback_controller_params",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "config",
                    "linear_feedback_controller_params.yaml",
                ]
            ),
            description="Path to the yaml file use to define "
            + "Linear Feedback Controller's and Joint State Estimator's params.",
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
    ]

    return LaunchDescription(
        declared_arguments
        + generate_default_franka_args()
        + [OpaqueFunction(function=launch_setup)]
    )
