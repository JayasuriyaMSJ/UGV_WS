#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from controller import Robot


class RosbotDriver(Node):
    def __init__(self):
        super().__init__('rosbot_driver')

        # Webots robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Motors
        self.motors = {}
        for name in ['fl_wheel_joint', 'fr_wheel_joint',
                     'rl_wheel_joint', 'rr_wheel_joint']:
            motor = self.robot.getDevice(name)
            if motor is not None:
                motor.setPosition(float('inf'))
                motor.setVelocity(0.0)
                self.motors[name] = motor
            else:
                self.get_logger().error(f'Motor {name} not found')

        # Lidar
        self.lidar = self.robot.getDevice('laser')
        if self.lidar is not None:
            self.lidar.enable(self.timestep)
            self.lidar.enablePointCloud()
            self.get_logger().info('Lidar enabled')
        else:
            self.get_logger().warn('Lidar not found')

        # RGB camera
        self.camera_rgb = self.robot.getDevice('camera rgb')
        if self.camera_rgb is not None:
            self.camera_rgb.enable(self.timestep)
            self.get_logger().info('RGB camera enabled')
        else:
            self.get_logger().warn('RGB camera not found')

        # Depth camera
        self.camera_depth = self.robot.getDevice('camera depth')
        if self.camera_depth is not None:
            self.camera_depth.enable(self.timestep)
            self.get_logger().info('Depth camera enabled')
        else:
            self.get_logger().warn('Depth camera not found')

        # ============ NEW: Subscribe to cmd_vel ============
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('Subscribed to /cmd_vel')

        # Robot parameters (from Rosbot specs)
        self.wheel_radius = 0.043  # meters
        self.wheel_separation = 0.220  # meters (distance between left and right wheels)
        # ===================================================

        # Webots step timer
        self.timer = self.create_timer(
            self.timestep / 1000.0,
            self.step
        )

        self.get_logger().info('Rosbot driver initialized')

    def cmd_vel_callback(self, msg):
        """Convert Twist message to wheel velocities"""
        # Extract linear and angular velocities from the message
        linear_vel = msg.linear.x      # Forward/backward velocity (m/s)
        angular_vel = msg.angular.z    # Rotation velocity (rad/s)

        # Differential drive kinematics:
        # left_wheel_vel = (v - ω*L/2) / r
        # right_wheel_vel = (v + ω*L/2) / r
        # where v = linear velocity, ω = angular velocity, L = wheel separation, r = wheel radius
        
        left_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        right_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius

        # Set motor velocities
        self.motors['fl_wheel_joint'].setVelocity(left_vel)
        self.motors['rl_wheel_joint'].setVelocity(left_vel)
        self.motors['fr_wheel_joint'].setVelocity(right_vel)
        self.motors['rr_wheel_joint'].setVelocity(right_vel)

        self.get_logger().debug(
            f'cmd_vel: linear={linear_vel:.2f}, angular={angular_vel:.2f} '
            f'-> left={left_vel:.2f}, right={right_vel:.2f}'
        )

    def step(self):
        if self.robot.step(self.timestep) == -1:
            self.get_logger().info('Simulation ended')
            rclpy.shutdown()

    def print_available_devices(self):
        num_devices = self.robot.getNumberOfDevices()
        self.get_logger().info(f'Total devices: {num_devices}')
        for i in range(num_devices):
            device = self.robot.getDeviceByIndex(i)
            self.get_logger().info(
                f'  Device {i}: {device.getName()} '
                f'(type: {device.getNodeType()})'
            )


def main(args=None):
    rclpy.init(args=args)

    controller = RosbotDriver()
    controller.print_available_devices()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()