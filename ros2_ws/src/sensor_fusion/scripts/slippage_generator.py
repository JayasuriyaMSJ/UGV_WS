#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class DriftTestDriver(Node):
    def __init__(self):
        super().__init__('drift_test_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dt = 0.05  # 20 Hz

    def run_tests(self):
        tests = [
            ("Forward 2m", lambda: self.move(0.2, 0.0, 10.0)),
            ("Spin 360°", lambda: self.move(0.0, 1.0, 6.28)),
            ("Figure-8", self.figure_eight),
            ("Stop-Start", self.stop_start),
        ]

        for name, test in tests:
            self.get_logger().info(f"Running: {name}")
            test()
            time.sleep(2.0)

    def move(self, linear, angular, duration):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular

        start = time.time()
        while time.time() - start < duration:
            self.pub.publish(cmd)
            time.sleep(self.dt)

        self.stop()

    def stop(self):
        self.pub.publish(Twist())
        time.sleep(0.5)

    def figure_eight(self):
        for _ in range(2):
            self.move(0.25, 0.6, 5.5)
            self.move(0.25, -0.6, 5.5)

    def stop_start(self):
        """Aggressive accel → wheel slip"""
        for _ in range(6):
            self.move(0.6, 0.0, 0.4)
            self.stop()
            time.sleep(0.2)


def main():
    rclpy.init()
    node = DriftTestDriver()

    print("Starting drift test in 3 seconds...")
    time.sleep(3)

    node.run_tests()

    print("Test complete! Check odom vs EKF.")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
