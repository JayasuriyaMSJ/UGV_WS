#!/usr/bin/env python3
"""
Standalone UGV Webots Controller Node

This is a complete, production-ready ROS 2 node that directly controls
the UGV robot in Webots simulation. It provides:
- Differential drive control via cmd_vel
- Odometry computation and publishing
- TF broadcasting
- Sensor data publishing (Lidar, Camera)
- Robust error handling

This node runs as a Webots robot controller (not extern).
"""

import rclpy
from rclpy.node import Node  
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image, JointState, PointCloud2, PointField, Imu
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import math
import struct
from controller import Robot, Motor, Lidar, Camera, Accelerometer, Gyro, InertialUnit

class UGVWebotsController(Node):
    """Production-grade UGV controller for Webots."""
    
    def __init__(self):
        super().__init__('ugv_webots_controller')
        
        # Initialize Webots Robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.print_available_devices()
        
        # Robot parameters
        self.wheel_radius = 0.1  # meters
        self.wheel_base = 0.4    # meters between left and right wheels
        self.max_speed = 5000.0  # rad/s
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.robot.getTime()
        
        # Initialize hardware
        self._init_motors()
        self._init_sensors()
        
        # ROS 2 Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.lidar_3d_pub = self.create_publisher(PointCloud2, 'velodyne_points', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/ugv_robot/imu', 10)
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ROS 2 Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Target velocities
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # Control loop timer
        self.timer = self.create_timer(self.timestep / 1000.0, self.control_loop)
        
        self.get_logger().info('UGV Webots Controller initialized successfully')
        self.get_logger().info(f'Timestep: {self.timestep}ms')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m, Wheel base: {self.wheel_base}m')
    
    def _init_motors(self):
        """Initialize motor devices."""
        try:
            # Get motors
            self.left_motors = [
                self.robot.getDevice('wheel_fl_joint'),
                self.robot.getDevice('wheel_rl_joint')
            ]
            self.right_motors = [
                self.robot.getDevice('wheel_fr_joint'),
                self.robot.getDevice('wheel_rr_joint')
            ]
            
            # Configure for velocity control
            all_motors = self.left_motors + self.right_motors
            for i, motor in enumerate(all_motors):
                if motor is None:
                    raise RuntimeError(f"Motor {i} not found!")
                motor.setPosition(float('inf'))
                motor.setVelocity(0.0)
            
            self.get_logger().info('Motors initialized: FL, FR, RL, RR')
            
        except Exception as e:
            self.get_logger().error(f'Motor initialization failed: {e}')
            raise
    
    def _init_sensors(self):
        """Initialize sensor devices."""

        # IMU
        # Orientation
        self.inertial = self.robot.getDevice('imu')
        if self.inertial:
            self.inertial.enable(self.timestep)
            self.get_logger().info('InertialUnit enabled')

        # Angular velocity
        self.gyro = self.robot.getDevice('gyro')
        if self.gyro:
            self.gyro.enable(self.timestep)
            self.get_logger().info('Gyro enabled')

        # Linear acceleration
        self.accel = self.robot.getDevice('accelerometer')
        if self.accel:
            self.accel.enable(self.timestep)
            self.get_logger().info('Accelerometer enabled')

        # Lidar
        self.lidar = self.robot.getDevice('sick_2d_lidar')
        if self.lidar:
            self.lidar.enable(self.timestep)
            self.get_logger().info('Lidar enabled')
        else:
            self.get_logger().warn('Lidar not found')

        # 3D Lidar (Velodyne VLP-16)
        self.lidar_3d = self.robot.getDevice('velodyne')
        if self.lidar_3d is not None:
            self.lidar_3d.enable(self.timestep)
            self.lidar_3d.enablePointCloud()
            self.get_logger().info('3D Lidar (velodyne) enabled')
        else:
            self.get_logger().warn('3D Lidar (velodyne) not found')
        
        # Camera
        self.camera = self.robot.getDevice('realsense_camera')
        if self.camera:
            self.camera.enable(self.timestep)
            self.get_logger().info(f'Camera enabled: {self.camera.getWidth()}x{self.camera.getHeight()}')
        else:
            self.get_logger().warn('Camera not found')
    
    def cmd_vel_callback(self, msg):
        """Handle cmd_vel messages."""
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z
    
    def compute_wheel_velocities(self):
        """Compute wheel velocities from cmd_vel using inverse kinematics."""
        # Differential drive inverse kinematics
        v_left = self.target_linear - (self.target_angular * self.wheel_base / 2.0)
        v_right = self.target_linear + (self.target_angular * self.wheel_base / 2.0)
        
        # Convert to angular velocities
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius
        
        # Clamp to max speed
        omega_left = max(min(omega_left, self.max_speed), -self.max_speed)
        omega_right = max(min(omega_right, self.max_speed), -self.max_speed)
        
        return omega_left, omega_right
    
    def update_odometry(self, omega_left, omega_right, dt):
        """Update odometry from wheel velocities."""
        if dt <= 0:
            return 0.0, 0.0
        
        # Forward kinematics
        v_left = omega_left * self.wheel_radius
        v_right = omega_right * self.wheel_radius
        
        # Robot velocities
        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.wheel_base
        
        # Update pose
        if abs(omega) < 1e-6:
            # Straight motion
            delta_x = v * dt * math.cos(self.theta)
            delta_y = v * dt * math.sin(self.theta)
            delta_theta = 0.0
        else:
            # Curved motion
            radius = v / omega
            delta_theta = omega * dt
            delta_x = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            delta_y = -radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize angle
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        return v, omega
    
    def publish_odometry(self, v, omega):
        """Publish odometry and TF."""
        current_time = self.get_clock().now()
        
        # Odometry message
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega
        
        self.odom_pub.publish(odom)
        
        # TF
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_imu(self):
        if not (self.inertial and self.gyro and self.accel):
            return

        # Orientation
        roll, pitch, yaw = self.inertial.getRollPitchYaw()

        # Angular velocity
        wx, wy, wz = self.gyro.getValues()

        # Linear acceleration
        ax, ay, az = self.accel.getValues()

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Orientation (RPY → quaternion)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw

        # Angular velocity
        imu_msg.angular_velocity.x = wx
        imu_msg.angular_velocity.y = wy
        imu_msg.angular_velocity.z = wz

        # Linear acceleration
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        self.imu_pub.publish(imu_msg)

    
    def publish_lidar(self):
        """Publish lidar scan."""
        if not self.lidar:
            return
        
        ranges = self.lidar.getRangeImage()
        if not ranges:
            return
        
        ranges = list(ranges)[::-1]
        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'sick_2d_lidar'
        
        fov = self.lidar.getFov()          # radians
        resolution = self.lidar.getHorizontalResolution()

        scan.angle_min = -fov / 2.0
        scan.angle_max =  fov / 2.0
        scan.angle_increment = fov / (resolution - 1)

        # scan.angle_increment = 2 * math.pi / len(ranges)
        scan.time_increment = 0.0
        scan.scan_time = self.timestep / 1000.0
        scan.range_min = 0.1
        scan.range_max = 30.0
        scan.ranges = list(ranges)
        
        self.scan_pub.publish(scan)
    
    def publish_lidar_3d(self):
        if self.lidar_3d is None:
            return

        try:
            point_cloud = self.lidar_3d.getPointCloud()
            if not point_cloud:
                return

            cloud_msg = PointCloud2()
            cloud_msg.header.stamp = self.get_clock().now().to_msg()
            cloud_msg.header.frame_id = 'velodyne'

            cloud_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
            ]

            cloud_msg.is_bigendian = False
            cloud_msg.point_step = 16
            cloud_msg.height = 1
            cloud_msg.width = len(point_cloud)
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            cloud_msg.is_dense = False

            buffer = []
            for point in point_cloud:
                x = point.x
                y = point.y
                z = point.z

                # Webots does not provide intensity or ring
                intensity = 0.0

                buffer.append(struct.pack('ffff', x, y, z, intensity))

            cloud_msg.data = b''.join(buffer)
            self.lidar_3d_pub.publish(cloud_msg)

        except Exception as e:
            self.get_logger().error(f'3D LiDAR publish error: {e}')

    def publish_camera(self):
        """Publish camera image."""
        if not self.camera:
            return
        
        image_data = self.camera.getImage()
        if not image_data:
            return
        
        img = Image()
        img.header = Header()
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = 'realsense_link'
        img.height = self.camera.getHeight()
        img.width = self.camera.getWidth()
        img.encoding = 'bgra8'
        img.is_bigendian = False
        img.step = img.width * 4
        img.data = list(image_data)
        
        self.image_pub.publish(img)
    
    def publish_joint_states(self):
        """Publish joint states."""
        js = JointState()
        js.header = Header()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['wheel_fl_joint', 'wheel_fr_joint', 'wheel_rl_joint', 'wheel_rr_joint']
        js.position = [0.0, 0.0, 0.0, 0.0]  # Would need position sensors for actual values
        js.velocity = []
        js.effort = []
        
        self.joint_pub.publish(js)
    
    def control_loop(self):
        """Main control loop."""
        # Step simulation
        if self.robot.step(self.timestep) == -1:
            return
        
        # Calculate dt
        current_time = self.robot.getTime()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Compute and apply motor velocities
        omega_left, omega_right = self.compute_wheel_velocities()
        
        for motor in self.left_motors:
            if motor:
                motor.setVelocity(omega_left)
        
        for motor in self.right_motors:
            if motor:
                motor.setVelocity(omega_right)
        
        # Update and publish odometry
        v, omega = self.update_odometry(omega_left, omega_right, dt)
        self.publish_odometry(v, omega)
        
        # Publish sensors
        self.publish_imu()
        self.publish_lidar_3d()
        self.publish_lidar()
        self.publish_camera()
        self.publish_joint_states()

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
    
    try:
        controller = UGVWebotsController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
