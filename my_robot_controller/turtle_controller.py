#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleControllerNode(Node):
    """
    The class responsible for controlling the turtle
    Args:

    Return:
        None
    """
    def __init__(self):
        """
        The constructor for the class, that would initialize the
        execution

        Args:
            self : the current instance of this class
        Return:
            None
        """
        super().__init__("turtle_controller")
        # creating publisher
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            "/turtle1/cmd_vel",
            10
        )
        # creating the subscriber
        self.pose_subscriber = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.pose_callback,
            10
        )

        self.get_logger().info("turtle controller has started")

    def pose_callback(self, pose: Pose):
        cmd = Twist()
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)


def main(args=None):
    """
    The programs Entry point
    Args:
        args: the node passed or None if not passed
    Return:
        None
    """
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
