#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.get_logger().info("🚀 CmdVelBridge started. Bridging /cmd_vel → /ap/cmd_vel")

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cb,
            10)

        self.pub = self.create_publisher(
            TwistStamped,
            '/ap/cmd_vel',
            10)

    def cb(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist = msg
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
