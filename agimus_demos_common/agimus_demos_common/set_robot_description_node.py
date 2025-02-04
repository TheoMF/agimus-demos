import sys

import rclpy
from rclpy.timer import Timer
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String

from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically


class SetRobotDescriptionNode(Node):
    """Main class implementing ROS node redirecting /robot_description topic
    to parameter of a node."""

    def __init__(self):
        super().__init__("set_robot_description_node")

        # List of nodes to set robot_description topic as a parameter
        self.declare_parameter("nodes_to_set", [""])
        # Time to wait for ROS nodes to appear
        self.declare_parameter("nodes_wait_timeout", 5.0)

        nodes_to_set = [
            node_name[1:] if node_name.startswith("/") else node_name
            for node_name in self.get_parameter("nodes_to_set")
            .get_parameter_value()
            .string_array_value
        ]

        self._exit_code = 1
        if len(nodes_to_set) == 0:
            self.get_logger().error("Parameter 'nodes_to_set' is empty!")
            raise SystemExit()

        self._retires = 5
        self._retires_counter = 0
        self._nodes_wait_timeout = (
            self.get_parameter("nodes_wait_timeout").get_parameter_value().double_value
        )

        if self._nodes_wait_timeout < 0:
            self.get_logger().error(
                "Parameter 'nodes_wait_timeout' can not be negative!"
            )
            raise SystemExit()

        self._empty_sub = self.create_subscription(
            String,
            "/robot_description",
            self._robot_description_cb,
            qos_profile=QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self._set_param_clients = {
            node_name: self.create_client(
                SetParametersAtomically, f"{node_name}/set_parameters_atomically"
            )
            for node_name in nodes_to_set
        }

        self.get_logger().info(
            "Node initialized, waiting for '/robot_description' to be published."
        )

        self._set_params_timer: Timer | None = None

    @property
    def exit_code(self) -> int:
        """Return value of exit code set by the node.

        Returns:
            int: Last value of exit code.
        """
        return self._exit_code

    def _robot_description_cb(self, msg: String) -> None:
        self.get_logger().info("Robot description received successfully.")
        self._robot_description = msg.data
        self._retires_counter = 0

        self._set_params_timer = self.create_timer(
            1.0 / (self._nodes_wait_timeout / self._retires), self._set_params_timer_cb
        )

    def _exit_the_node(self, exit_code: int) -> None:
        if self._set_params_timer is not None:
            self._set_params_timer.cancel()
        self._exit_code = exit_code

        raise SystemExit()

    def _log_failed_params_set(self, msg: str) -> None:
        if self._retires_counter < self._retires:
            self.get_logger().info(msg)
        else:
            self.get_logger().error(msg)

    def _set_params_timer_cb(self) -> None:
        available_nodes = self.get_node_names()
        self.get_logger().warn(f"{available_nodes}")
        successfully_removed = []
        for node_name, client in self._set_param_clients.items():
            if node_name not in available_nodes:
                self._log_failed_params_set(
                    f"Node with a name '{node_name}' was not found!"
                )
                continue

            if not client.service_is_ready():
                self._log_failed_params_set(
                    f"Service with a name '{client.srv_name}' is not ready!"
                )
                continue

            req = SetParametersAtomically.Request(
                parameters=[
                    Parameter(
                        "robot_description",
                        Parameter.Type.STRING,
                        self._robot_description,
                    ).to_parameter_msg()
                ]
            )
            result = client.call(req)
            if not result.result.successful:
                self._log_failed_params_set(
                    f"Filed to set parameter with a service '{client.srv_name}'!"
                )
                continue

            successfully_removed.append(node_name)
        self._set_param_clients = {
            k: v
            for k, v in self._set_param_clients.items()
            if k not in successfully_removed
        }

        if len(self._set_param_clients.keys()) == 0:
            self.get_logger().info("All nodes had params set correctly!")
            self._exit_the_node(0)

        self._retires_counter += 1
        if self._retires_counter == self._retires:
            self.get_logger().error("Failed to set params for all nodes!")
            self._exit_the_node(1)


def main(args=None):
    rclpy.init(args=args)
    ret = 1
    set_robot_description_node = SetRobotDescriptionNode()
    try:
        rclpy.spin(set_robot_description_node)
    except SystemExit:
        set_robot_description_node.destroy_node()
        ret = set_robot_description_node.exit_code
        rclpy.try_shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
