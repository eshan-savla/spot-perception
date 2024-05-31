#!/usr/bin/env python3
# Debug
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class nav2_to_spot(Node):

    def __init__(self):
        super().__init__('nav2_to_spot_node')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/hkaspot/cmd_vel', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    twist_relay = nav2_to_spot()
    rclpy.spin(twist_relay)
    twist_relay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

