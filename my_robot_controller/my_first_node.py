#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyNode(Node):
    """
    Summary : Node for the testing ROS Node creation
    """

    counter = 0

    def __init__(self):
        """
        Class Constructor
        Args:
            self : current instance
        Returns:
            None
        """
        super().__init__("first_node")
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """
        Logs an output when the project is executed.
        Args:
            self : current instance
        Returns:
            None
        """
        self.get_logger().info("Hello ROS2 " + str(self.counter))
        self.counter += 1


def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    # Shutdown ROS
    rclpy.shutdown()


if __name__ == '__main__':
    main()

