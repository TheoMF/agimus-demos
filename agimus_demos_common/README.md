# Agimus Demos Common

Package containing common launch and config files used across different demos.

This package intends to be used as a common source of basic launch files allowing to bring up both simulation and real robot in a unified manner.

## Launch files

**franka_common.launch.py** is meant to be included in demos where Franka Emika robots are used. Depending on configuration it launches Gazebo simulation or connects to the real robot. Aside from unifying include interface of Franka Emika robot between simulation and hardware it also provides options to launch RViz2 and start Lineaf Feedback Controller. This launch file in a sense is equivalent to *Demo 01 LFC Alone*.

Launch arguments:
- `franka_controllers_params`: Path to the yaml file use to define controller parameters.

    **default**: *agimus_demos_common/config/franka_controllers.yaml*

- `linear_feedback_controller_params`: Path to the yaml file use to define Linear Feedback Controller\`s and Joint State Estimator\`s params.

    **default**: *agimus_demos_common/config/linear_feedback_controller_params.yaml*

- `rviz_config_path`: Path to RViz configuration file

    **default**: *agimus_demos_common/rviz/franka_preview.rviz*

- `robot_ip`: Hostname or IP address of the robot. If not empty launch file is configured for real robot. If empty **use_gazebo** is expected to be set to *true*.

    **default**: *""*

- `arm_id`: ID of the type of arm used. Supported values: fer, fr3, fp3.

    Valid choices: [*fer*, *fr3*, *fp3*]

    **default**: *fer*

- `use_gazebo`: Configures launch file for Gazebo simulation. If set to *true* launch file is configured for simulated robot. If set to *false* argument `robot_ip` is expected not to be empty.

    **Valid choices**: [*true*, *false*]

    **default**: *false*

- `use_rviz`: Visualize the robot in RViz.

    **Valid choices**: [*true*, *false*]

    **default**: *false*

- `gz_verbose`: Wether to set verbosity level of Gazebo to 3.

    **Valid choices**: [*true*, *false*]

    **default**: *false*

- `gz_headless`: Wether to launch Gazebo in headless mode (no GUI is launched, only physics server).

    **Valid choices**: [*true*, *false*]

    **default**: *false*

## Utils

This package provides utility functions to ease up creation of new launch files. Simple example launch file can look as follows:

```python
from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_franka_launch,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    # Helper function that includes `franka_common.launch.py`
    franka_robot_launch = generate_include_franka_launch()

    my_awesome_node = Node(
        package="my_awesome_package",
        executable="my_awesome_node",
    )

    return [
        franka_robot_launch,
        my_awesome_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        # Helper function creates launch arguments required by `franka_common.launch.py`
        generate_default_franka_args()
        + [OpaqueFunction(function=launch_setup)]
    )
```

Function `generate_default_franka_args()` ensures all launch arguments used by `franka_common.launch.py` are exposed by launch file, while `generate_include_franka_launch()` includes that file and uses those declared parameters.

Function `generate_default_franka_args()` is directly used by `franka_common.launch.py`, so all arguments exposed by it are described in the documentation above.
