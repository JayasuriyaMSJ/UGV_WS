#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math


class OdomDebugOnChange(Node):
    def __init__(self):
        super().__init__('odom_debug_on_change')

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.prev_yaw = None
        self.yaw_threshold = math.radians(1.0)  # 1 degree threshold

        self.get_logger().info('Odom debug (publish on change) started')

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation

        # Planar yaw extraction (debug-friendly)
        yaw_rad = 2.0 * math.atan2(q.z, q.w)
        yaw_deg = math.degrees(yaw_rad)

        # First message → just store
        if self.prev_yaw is None:
            self.prev_yaw = yaw_rad
            return

        # Normalize difference to [-pi, pi]
        delta = yaw_rad - self.prev_yaw
        delta = math.atan2(math.sin(delta), math.cos(delta))

        if abs(delta) >= self.yaw_threshold:
            self.get_logger().info(
                f"Yaw changed → {yaw_rad:.4f} rad | {yaw_deg:.2f} deg"
            )
            self.prev_yaw = yaw_rad


def main():
    rclpy.init()
    node = OdomDebugOnChange()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
