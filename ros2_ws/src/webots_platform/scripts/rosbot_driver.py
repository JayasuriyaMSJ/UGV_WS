#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from controller import Robot
import math
import struct

class RosbotDriverCalibrated(Node):
    def __init__(self):
        super().__init__('rosbot_driver')

        # Webots robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # ========== OFFICIAL PROTO PARAMETERS ==========
        # These come directly from the Husarion Rosbot PROTO file
        self.declare_parameter('wheel_radius', 0.043)     # Official PROTO value
        self.declare_parameter('wheel_separation', 0.186)  # Official Physical Track
        
        # Skid-steer robots need a high separation multiplier (usually 1.2x to 1.5x)
        # Based on your previous error (3.14 ROS / 2.90 Webots):
        self.declare_parameter('wheel_radius_calibration', 1.0)     # 1.106
        self.declare_parameter('wheel_separation_calibration', 1.45)        # 1.49  # Adjusted for the 0.192 base
        
        # Debug mode
        self.declare_parameter('debug_encoders', False)
        
        # Get parameters
        base_wheel_radius = self.get_parameter('wheel_radius').value
        base_wheel_separation = self.get_parameter('wheel_separation').value
        radius_cal = self.get_parameter('wheel_radius_calibration').value
        separation_cal = self.get_parameter('wheel_separation_calibration').value
        self.debug_encoders = self.get_parameter('debug_encoders').value
        
        # Apply calibration
        self.wheel_radius = base_wheel_radius * radius_cal
        self.wheel_separation = base_wheel_separation * separation_cal
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('ROSBOT DRIVER - CALIBRATED VERSION')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Base wheel radius: {base_wheel_radius}m')
        self.get_logger().info(f'Radius calibration: {radius_cal}')
        self.get_logger().info(f'Final wheel radius: {self.wheel_radius}m')
        self.get_logger().info(f'Base wheel separation: {base_wheel_separation}m')
        self.get_logger().info(f'Separation calibration: {separation_cal}')
        self.get_logger().info(f'Final wheel separation: {self.wheel_separation}m')
        self.get_logger().info(f'Debug mode: {self.debug_encoders}')
        self.get_logger().info('=' * 60)
        # ===========================================
        
        # Odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.robot.getTime()

        # Motors
        self.motors = {}
        motor_names = ['fl_wheel_joint', 'fr_wheel_joint',
                      'rl_wheel_joint', 'rr_wheel_joint']
        
        for name in motor_names:
            motor = self.robot.getDevice(name)
            if motor is not None:
                motor.setPosition(float('inf'))
                motor.setVelocity(0.0)
                self.motors[name] = motor
                self.get_logger().info(f'Motor {name} initialized')
        
        # Position sensors for odometry
        sensor_names = {
            'fl': 'front left wheel motor sensor',
            'fr': 'front right wheel motor sensor',
            'rl': 'rear left wheel motor sensor',
            'rr': 'rear right wheel motor sensor'
        }
        
        self.motor_sensors = {}
        for key, name in sensor_names.items():
            sensor = self.robot.getDevice(name)
            if sensor is not None:
                sensor.enable(self.timestep)
                self.motor_sensors[key] = sensor
                self.get_logger().info(f'Position sensor {name} enabled')
        
        # Wait one step to get initial encoder values
        self.robot.step(self.timestep)
        
        # Initialize previous positions AFTER first step
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0
        self.first_odometry_update = True
        
        # Debug counter
        self.debug_counter = 0

        # Lidar
        self.lidar = self.robot.getDevice('laser')
        if self.lidar is not None:
            self.lidar.enable(self.timestep)
            self.lidar.enablePointCloud()
            self.get_logger().info('Lidar enabled')

        # 3D Lidar (Velodyne VLP-16)
        self.lidar_3d = self.robot.getDevice('velodyne')
        if self.lidar_3d is not None:
            self.lidar_3d.enable(self.timestep)
            self.lidar_3d.enablePointCloud()
            self.get_logger().info('3D Lidar (velodyne) enabled')
        else:
            self.get_logger().warn('3D Lidar (velodyne) not found')

        # RGB camera
        self.camera_rgb = self.robot.getDevice('camera rgb')
        if self.camera_rgb is not None:
            self.camera_rgb.enable(self.timestep)
            self.get_logger().info('RGB camera enabled')

        # IMU
        self.imu = self.robot.getDevice('imu')
        if self.imu is not None:
            self.imu.enable(self.timestep)
            self.get_logger().info('IMU enabled')
        
        self.gyro = self.robot.getDevice('imu gyro')
        if self.gyro is not None:
            self.gyro.enable(self.timestep)
            self.get_logger().info('Gyro enabled')
        
        self.accelerometer = self.robot.getDevice('imu accelerometer')
        if self.accelerometer is not None:
            self.accelerometer.enable(self.timestep)
            self.get_logger().info('Accelerometer enabled')

        # ========== ROS Publishers ==========
        self.laser_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.lidar_3d_pub = self.create_publisher(PointCloud2, 'velodyne_points', 10)
        self.camera_rgb_pub = self.create_publisher(Image, 'camera/rgb/image_raw', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Example: raw value to meters
        # raw_val = self.front_sensor.getValue()
        # Most Webots IR sensors: distance = lookup_table_inverse(raw_val)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Webots step timer
        self.timer = self.create_timer(
            self.timestep / 1000.0,
            self.step
        )

        self.get_logger().info('Rosbot driver initialized and ready!')

    def cmd_vel_callback(self, msg):
        """Convert Twist message to wheel velocities"""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z    # Positive is CCW

        # angular_vel = -angular_vel  

        # Kinematics: No need to invert angular_vel if wheels are assigned correctly
        # left = v - (w * L / 2), right = v + (w * L / 2)
        left_wheel_vel = (linear_vel - (angular_vel * self.wheel_separation / 2.0)) / self.wheel_radius
        right_wheel_vel = (linear_vel + (angular_vel * self.wheel_separation / 2.0)) / self.wheel_radius

        # Apply to all motors
        if 'fl_wheel_joint' in self.motors:
            self.motors['fl_wheel_joint'].setVelocity(left_wheel_vel)
        if 'rl_wheel_joint' in self.motors:
            self.motors['rl_wheel_joint'].setVelocity(left_wheel_vel)
        if 'fr_wheel_joint' in self.motors:
            self.motors['fr_wheel_joint'].setVelocity(right_wheel_vel)
        if 'rr_wheel_joint' in self.motors:
            self.motors['rr_wheel_joint'].setVelocity(right_wheel_vel)

    def update_odometry(self):
        """Calculate odometry from wheel encoders"""
        if 'fl' not in self.motor_sensors or 'fr' not in self.motor_sensors:
            return
        
        # Get current wheel positions (in radians)
        # Average front and rear wheels for each side
        left_pos = (self.motor_sensors['fl'].getValue() + 
                self.motor_sensors['rl'].getValue()) / 2.0
        right_pos = (self.motor_sensors['fr'].getValue() + 
                    self.motor_sensors['rr'].getValue()) / 2.0
        
        # Initialize on first update
        if self.first_odometry_update:
            self.prev_left_pos = left_pos
            self.prev_right_pos = right_pos
            self.first_odometry_update = False
            self.get_logger().info(f'Odometry initialized: left={left_pos:.4f}, right={right_pos:.4f}')
            return
        
        # Calculate change in wheel positions (radians)
        delta_left = left_pos - self.prev_left_pos
        delta_right = right_pos - self.prev_right_pos
        
        # Debug output
        if self.debug_encoders and self.debug_counter % 20 == 0:
            self.get_logger().info(f'Encoders - Left: {left_pos:.4f} (Δ{delta_left:.4f}), '
                                f'Right: {right_pos:.4f} (Δ{delta_right:.4f})')
        self.debug_counter += 1
        
        # Convert to linear distances (meters)
        left_dist = delta_left * self.wheel_radius
        right_dist = delta_right * self.wheel_radius
        
        # Calculate robot displacement
        center_dist = (left_dist + right_dist) / 2.0
        
        # ===== FIX: Swap the order to fix angular direction =====
        delta_theta = (right_dist - left_dist) / self.wheel_separation
        # ========================================================
        
        # Update pose using exact integration
        if abs(delta_theta) < 1e-6:
            # Straight line motion
            delta_x = center_dist * math.cos(self.theta)
            delta_y = center_dist * math.sin(self.theta)
        else:
            # Arc motion
            radius = center_dist / delta_theta
            delta_x = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            delta_y = -radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Save positions for next iteration
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos
        
        # Calculate velocities
        current_time = self.robot.getTime()
        dt = current_time - self.last_time
        
        if dt > 0:
            vx = center_dist / dt
            vth = delta_theta / dt
        else:
            vx = 0.0
            vth = 0.0
        
        self.last_time = current_time
            
        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = vth
        
        self.odom_pub.publish(odom)
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

    def publish_lidar(self):
        """Publish laser scan data"""
        if self.lidar is None:
            return
        
        ranges = self.lidar.getRangeImage()
        if ranges is None:
            return
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        
        scan.angle_min = -math.pi + 3.14159
        scan.angle_max = math.pi + 3.14159
        scan.angle_increment = 2.0 * math.pi / len(ranges)
        scan.time_increment = 0.0
        scan.scan_time = self.timestep / 1000.0
        scan.range_min = self.lidar.getMinRange()
        scan.range_max = self.lidar.getMaxRange()
        scan.ranges = ranges
        
        self.laser_pub.publish(scan)

    def publish_camera_rgb(self):
        """Publish RGB camera image"""
        if self.camera_rgb is None:
            return
        
        img_data = self.camera_rgb.getImage()
        if img_data is None:
            return
        
        img = Image()
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = 'camera_rgb'
        img.height = self.camera_rgb.getHeight()
        img.width = self.camera_rgb.getWidth()
        img.encoding = 'bgra8'
        img.step = img.width * 4
        img.data = img_data
        
        self.camera_rgb_pub.publish(img)

    def publish_imu(self):
        """Publish IMU data"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        if self.imu is not None:
            try:
                rpy = self.imu.getRollPitchYaw()
                cy = math.cos(rpy[2] * 0.5)
                sy = math.sin(rpy[2] * 0.5)
                cp = math.cos(rpy[1] * 0.5)
                sp = math.sin(rpy[1] * 0.5)
                cr = math.cos(rpy[0] * 0.5)
                sr = math.sin(rpy[0] * 0.5)
                
                imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
                imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
                imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
                imu_msg.orientation.z = cr * cp * sy - sr * sp * cy
            except:
                pass
        
        if self.gyro is not None:
            try:
                gyro_values = self.gyro.getValues()
                imu_msg.angular_velocity.x = gyro_values[0]
                imu_msg.angular_velocity.y = gyro_values[1]
                imu_msg.angular_velocity.z = gyro_values[2]
            except:
                pass
        
        if self.accelerometer is not None:
            try:
                accel_values = self.accelerometer.getValues()
                imu_msg.linear_acceleration.x = accel_values[0]
                imu_msg.linear_acceleration.y = accel_values[1]
                imu_msg.linear_acceleration.z = accel_values[2]
            except:
                pass
        
        self.imu_pub.publish(imu_msg)

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
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]

            cloud_msg.is_bigendian = False
            cloud_msg.point_step = 16
            cloud_msg.height = 1
            cloud_msg.width = len(point_cloud)
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            cloud_msg.is_dense = True

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


    def step(self):
        """Main simulation step"""
        if self.robot.step(self.timestep) == -1:
            self.get_logger().info('Simulation ended')
            rclpy.shutdown()
            return
        
        self.update_odometry()
        self.publish_lidar()
        self.publish_lidar_3d()
        self.publish_camera_rgb()
        self.publish_imu()


def main(args=None):
    rclpy.init(args=args)
    controller = RosbotDriverCalibrated()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()