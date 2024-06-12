#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DrawCircle(Node):
    """
    Would have the various methods that would aid the drawing of a
    circle with turtlesim

    Args:
        Node : rclpy node
    Return:
    """
    def __init__(self):
        """
        DrawCircle class constructor
        Args:
            self : current instance
        Return:
            None
        """
        # Initializing the node with the name "draw_circle"
        super().__init__("draw_circle")
        # 10 is the queue size
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Draw circle node has been started")

    def send_velocity_command(self):
        """
        This method is responsible for sending turtlesim direction commands

        Args:
            self : current instance/class
        Return:
            None
        """
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_pub_.publish(msg)


def main(args=None):
    """
    The programs entry point

    Args:
        args : argument passed from the DrawCircle class constructure
    Return:
        None
    """
    rclpy.init(args=args)
    node = DrawCircle()
    rclpy.spin(node)
    rclpy.shutdown()

