from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_default_franka_args() -> list[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="",
            description="Hostname or IP address of the robot. "
            + "If not empty launch file is configured for real robot. "
            + "If empty `use_gazebo` is expected to be set to `true`.",
        ),
        DeclareLaunchArgument(
            "arm_id",
            default_value="fer",
            description="ID of the type of arm used. Supported values: fer, fr3, fp3",
            choices=["fer", "fr3", "fp3"],
        ),
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="false",
            description="Configures launch file for Gazebo simulation. "
            + "If set to `true` launch file is configured for simulated robot. "
            + "If set to `false` argument `robot_ip` is expected not to be empty.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Visualize the robot in RViz",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "gz_verbose",
            default_value="false",
            description="Wether to set verbosity level of Gazebo to 3.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "gz_headless",
            default_value="false",
            description="Wether to launch Gazebo in headless mode "
            + "(no GUI is launched, only physics server).",
            choices=["true", "false"],
        ),
    ]


def generate_include_franka_launch() -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
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
            "arm_id": LaunchConfiguration("arm_id"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_gazebo": LaunchConfiguration("use_gazebo"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "gz_verbose": LaunchConfiguration("gz_verbose"),
            "gz_headless": LaunchConfiguration("gz_headless"),
        }.items(),
    )
