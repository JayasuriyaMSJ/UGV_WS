#!/usr/bin/env python3
"""
UGV Webots Controller - TIMESTAMP SYNCHRONIZED VERSION

Key fix: All sensor messages in one loop iteration use THE SAME timestamp
This ensures rgbd_sync can properly synchronize RGB, Depth, and Camera Info
"""

import rclpy
from rclpy.node import Node  
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image, JointState, PointCloud2, PointField, Imu, CameraInfo
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import math
from controller import Robot
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
import struct


class UGVWebotsController(Node):
    """Production-grade UGV controller for Webots with synchronized timestamps."""
    
    def __init__(self):
        super().__init__('ugv_webots_controller')
        
        # Initialize Webots Robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Robot parameters
        self.wheel_radius = 0.1
        self.wheel_base = 0.4
        self.max_speed = 5000.0
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.robot.getTime()
        
        # Initialize hardware
        self._init_motors()
        self._init_sensors()
        
        # ROS 2 Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.lidar_3d_pub = self.create_publisher(PointCloud2, '/velodyne_points', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu_data', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.clock_pub = self.create_publisher(Clock, 'clock', 10)

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
        
        # Wheel positions for visualization
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]
        
        # Position Sensors
        self.left_sensors = [
            self.robot.getDevice('wheel_fl_sensor'),
            self.robot.getDevice('wheel_rl_sensor')
        ]
        self.right_sensors = [
            self.robot.getDevice('wheel_fr_sensor'),
            self.robot.getDevice('wheel_rr_sensor')
        ]
        for sensor in self.left_sensors + self.right_sensors:
            if sensor:
                sensor.enable(self.timestep)
        
        self.last_left_pos = 0.0
        self.last_right_pos = 0.0
        self.initialized = False
        
        self.get_logger().info('UGV Webots Controller initialized successfully')
        self.get_logger().info(f'Timestep: {self.timestep}ms')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m, Wheel base: {self.wheel_base}m')
    
    def _init_motors(self):
        """Initialize motor devices."""
        try:
            self.left_motors = [
                self.robot.getDevice('wheel_fl_joint'),
                self.robot.getDevice('wheel_rl_joint')
            ]
            self.right_motors = [
                self.robot.getDevice('wheel_fr_joint'),
                self.robot.getDevice('wheel_rr_joint')
            ]
            
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
        # IMU - InertialUnit
        self.inertial = self.robot.getDevice('imu')
        if self.inertial:
            self.inertial.enable(self.timestep)
            self.get_logger().info('InertialUnit (IMU) enabled')
        
        # Gyro
        self.gyro = self.robot.getDevice('gyro')
        if self.gyro:
            self.gyro.enable(self.timestep)
            self.get_logger().info('Gyro enabled')
        
        # Accelerometer
        self.accel = self.robot.getDevice('accelerometer')
        if self.accel:
            self.accel.enable(self.timestep)
            self.get_logger().info('Accelerometer enabled')

        # Lidar
        self.lidar = self.robot.getDevice('sick_2d_lidar')
        if self.lidar:
            self.lidar.enable(self.timestep)
            self.get_logger().info('Lidar enabled')

        # 3D Lidar (Velodyne VLP-16)
        self.lidar_3d = self.robot.getDevice('velodyne')
        if self.lidar_3d is not None:
            self.lidar_3d.enable(self.timestep)
            self.lidar_3d.enablePointCloud()
            self.get_logger().info('3D Lidar (velodyne) enabled')
        
        # Camera
        self.camera = self.robot.getDevice('realsense_camera')
        if self.camera:
            self.camera.enable(self.timestep)
            self.get_logger().info(f'Camera enabled: {self.camera.getWidth()}x{self.camera.getHeight()}')

        # Depth camera
        self.depth_camera = self.robot.getDevice('realsense_depth')
        if self.depth_camera:
            self.depth_camera.enable(self.timestep)
            self.get_logger().info(f'Depth camera enabled: {self.depth_camera.getWidth()}x{self.depth_camera.getHeight()}')
    
    def cmd_vel_callback(self, msg):
        """Handle cmd_vel messages."""
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z
    
    def compute_wheel_velocities(self):
        """Compute wheel velocities from cmd_vel using inverse kinematics."""
        v_left = self.target_linear - (self.target_angular * self.wheel_base / 2.0)
        v_right = self.target_linear + (self.target_angular * self.wheel_base / 2.0)
        
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius
        
        omega_left = max(min(omega_left, self.max_speed), -self.max_speed)
        omega_right = max(min(omega_right, self.max_speed), -self.max_speed)
        
        return omega_left, omega_right
    
    def step_once(self):
        """Single simulation step with synchronized timestamps."""
        # Step simulation
        if self.robot.step(self.timestep) == -1:
            return False
            
        # 1. Get current Webots time and create ROS timestamp
        current_time = self.robot.getTime()
        seconds = int(current_time)
        nanoseconds = int((current_time - seconds) * 1e9)
        
        ros_time_msg = Time()
        ros_time_msg.sec = seconds
        ros_time_msg.nanosec = nanoseconds
        
        # 2. Publish Clock FIRST
        clock_msg = Clock()
        clock_msg.clock = ros_time_msg
        self.clock_pub.publish(clock_msg)
        
        # 3. Update Dynamics
        dt = current_time - self.last_time
        if dt <= 0:
            return True
        self.last_time = current_time
        
        # Compute and apply motor velocities
        omega_left_cmd, omega_right_cmd = self.compute_wheel_velocities()
        
        for motor in self.left_motors:
            if motor:
                motor.setVelocity(omega_left_cmd)
        
        for motor in self.right_motors:
            if motor:
                motor.setVelocity(omega_right_cmd)
        
        # Update wheel positions
        self.wheel_positions[0] += omega_left_cmd * dt
        self.wheel_positions[1] += omega_right_cmd * dt
        self.wheel_positions[2] += omega_left_cmd * dt
        self.wheel_positions[3] += omega_right_cmd * dt

        # 4. Publish ALL sensors with THE SAME timestamp
        try:
            # *** CRITICAL: Use ros_time_msg for ALL sensor publications ***
            self.publish_odometry(dt, ros_time_msg)
            self.publish_imu(ros_time_msg)
            self.publish_lidar(ros_time_msg)
            self.publish_lidar_3d(ros_time_msg)
            
            # Camera topics MUST have identical timestamps for rgbd_sync
            self.publish_camera_synchronized(ros_time_msg)
            
            self.publish_joint_states(ros_time_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing sensors: {e}")
        
        return True

    def publish_camera_synchronized(self, timestamp):
        """Publish ALL camera topics with THE SAME timestamp - critical for rgbd_sync!"""
        if not self.camera or not self.depth_camera:
            return
        
        # Get data from both cameras
        rgb_data = self.camera.getImage()
        depth_data = self.depth_camera.getRangeImage()
        
        if not rgb_data or not depth_data:
            return
        
        # Camera parameters
        rgb_width = self.camera.getWidth()
        rgb_height = self.camera.getHeight()
        rgb_fov = self.camera.getFov()
        
        depth_width = self.depth_camera.getWidth()
        depth_height = self.depth_camera.getHeight()
        depth_fov = self.depth_camera.getFov()
        max_range = self.depth_camera.getMaxRange()
        
        # Calculate intrinsics
        fx_rgb = rgb_width / (2.0 * math.tan(rgb_fov / 2.0))
        fy_rgb = fx_rgb
        cx_rgb = rgb_width / 2.0
        cy_rgb = rgb_height / 2.0
        
        fx_depth = depth_width / (2.0 * math.tan(depth_fov / 2.0))
        fy_depth = fx_depth
        cx_depth = depth_width / 2.0
        cy_depth = depth_height / 2.0
        
        # ========================================
        # PUBLISH ALL WITH SAME TIMESTAMP
        # ========================================
        
        # 1. RGB Camera Info
        rgb_info = CameraInfo()
        rgb_info.header.stamp = timestamp  # ← SAME TIMESTAMP
        rgb_info.header.frame_id = 'realsense_link'
        rgb_info.width = rgb_width
        rgb_info.height = rgb_height
        rgb_info.k = [fx_rgb, 0.0, cx_rgb, 0.0, fy_rgb, cy_rgb, 0.0, 0.0, 1.0]
        rgb_info.p = [fx_rgb, 0.0, cx_rgb, 0.0, 0.0, fy_rgb, cy_rgb, 0.0, 0.0, 0.0, 1.0, 0.0]
        rgb_info.distortion_model = "plumb_bob"
        rgb_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_pub.publish(rgb_info)
        
        # 2. RGB Image
        rgb_img = Image()
        rgb_img.header.stamp = timestamp  # ← SAME TIMESTAMP
        rgb_img.header.frame_id = 'realsense_link'
        rgb_img.height = rgb_height
        rgb_img.width = rgb_width
        rgb_img.encoding = 'bgra8'
        rgb_img.is_bigendian = False
        rgb_img.step = rgb_width * 4
        rgb_img.data = list(rgb_data)
        self.image_pub.publish(rgb_img)
        
        # 3. Depth Camera Info
        depth_info = CameraInfo()
        depth_info.header.stamp = timestamp  # ← SAME TIMESTAMP
        depth_info.header.frame_id = 'realsense_link'
        depth_info.width = depth_width
        depth_info.height = depth_height
        depth_info.k = [fx_depth, 0.0, cx_depth, 0.0, fy_depth, cy_depth, 0.0, 0.0, 1.0]
        depth_info.p = [fx_depth, 0.0, cx_depth, 0.0, 0.0, fy_depth, cy_depth, 0.0, 0.0, 0.0, 1.0, 0.0]
        depth_info.distortion_model = "plumb_bob"
        depth_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.depth_info_pub.publish(depth_info)
        
        # 4. Depth Image
        import array
        processed_depth = []
        for val in depth_data:
            if val >= max_range or math.isinf(val) or val <= 0.0:
                processed_depth.append(0)
            else:
                mm_val = int(val * 1000.0)
                processed_depth.append(min(mm_val, 65535))
        
        depth_img = Image()
        depth_img.header.stamp = timestamp  # ← SAME TIMESTAMP
        depth_img.header.frame_id = 'realsense_link'
        depth_img.height = depth_height
        depth_img.width = depth_width
        depth_img.encoding = '16UC1'
        depth_img.is_bigendian = False
        depth_img.step = depth_width * 2
        depth_array = array.array('H', processed_depth)
        depth_img.data = depth_array.tobytes()
        self.depth_pub.publish(depth_img)

    def publish_odometry(self, dt, timestamp):
        """Update and publish odometry and TF."""
        l_sens = [s for s in self.left_sensors if s]
        r_sens = [s for s in self.right_sensors if s]
        
        if not l_sens or not r_sens:
            return

        left_pos_val = sum([s.getValue() for s in l_sens]) / len(l_sens)
        right_pos_val = sum([s.getValue() for s in r_sens]) / len(r_sens)

        if math.isnan(left_pos_val) or math.isnan(right_pos_val):
            return

        left_pos = left_pos_val
        right_pos = right_pos_val

        if not self.initialized:
            self.last_left_pos = left_pos
            self.last_right_pos = right_pos
            self.initialized = True
            return

        d_left = (left_pos - self.last_left_pos) * self.wheel_radius
        d_right = (right_pos - self.last_right_pos) * self.wheel_radius
        
        self.last_left_pos = left_pos
        self.last_right_pos = right_pos

        dist = (d_left + d_right) / 2.0
        theta_old = self.theta
        
        if self.inertial:
            roll, pitch, yaw = self.inertial.getRollPitchYaw()
            if not math.isnan(yaw):
                self.theta = yaw
            else:
                self.theta += (d_right - d_left) / self.wheel_base
        else:
            self.theta += (d_right - d_left) / self.wheel_base

        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        theta_avg = (theta_old + self.theta) / 2.0
        if abs(self.theta - theta_old) > math.pi:
             theta_avg = self.theta

        self.x += dist * math.cos(theta_avg)
        self.y += dist * math.sin(theta_avg)
        
        v = dist / dt
        omega = (self.theta - theta_old) / dt
        if omega > math.pi / dt: omega -= 2*math.pi/dt
        if omega < -math.pi / dt: omega += 2*math.pi/dt
        
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = timestamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega
        
        self.odom_pub.publish(odom)
        
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

    def publish_imu(self, timestamp):
        """Publish IMU data."""
        if not (self.inertial and self.gyro and self.accel):
            return

        try:
            roll, pitch, yaw = self.inertial.getRollPitchYaw()
            wx, wy, wz = self.gyro.getValues()
            ax, ay, az = self.accel.getValues()

            imu_msg = Imu()
            imu_msg.header.stamp = timestamp
            imu_msg.header.frame_id = 'imu_link'

            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)

            imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
            imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
            imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
            imu_msg.orientation.z = cr * cp * sy - sr * sp * cy

            imu_msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

            imu_msg.angular_velocity.x = wx
            imu_msg.angular_velocity.y = wy
            imu_msg.angular_velocity.z = wz
            imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

            self.imu_pub.publish(imu_msg)
        except Exception as e:
            self.get_logger().error(f'IMU publish error: {e}')

    def publish_lidar_3d(self, timestamp):
        """Publish 3D LiDAR point cloud."""
        if self.lidar_3d is None:
            return

        try:
            point_cloud = self.lidar_3d.getPointCloud()
            if not point_cloud:
                return

            cloud_msg = PointCloud2()
            cloud_msg.header.stamp = timestamp
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
                buffer.append(struct.pack('ffff', point.x, point.y, point.z, 0.0))

            cloud_msg.data = b''.join(buffer)
            self.lidar_3d_pub.publish(cloud_msg)
        except Exception as e:
            self.get_logger().error(f'3D LiDAR publish error: {e}')

    def publish_lidar(self, timestamp):
        """Publish 2D lidar scan."""
        if not self.lidar:
            return
        
        ranges = self.lidar.getRangeImage()
        if not ranges:
            return
        
        ranges = list(ranges)[::-1]
        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = timestamp
        scan.header.frame_id = 'sick_2d_lidar'
        
        fov = self.lidar.getFov()
        resolution = self.lidar.getHorizontalResolution()

        scan.angle_min = -fov / 2.0
        scan.angle_max =  fov / 2.0
        scan.angle_increment = fov / (resolution - 1)
        scan.time_increment = 0.0
        scan.scan_time = self.timestep / 1000.0
        scan.range_min = 0.1
        scan.range_max = 30.0
        scan.ranges = list(ranges)
        
        self.scan_pub.publish(scan)

    def publish_joint_states(self, timestamp):
        """Publish joint states."""
        js = JointState()
        js.header = Header()
        js.header.stamp = timestamp
        js.name = ['wheel_fl_joint', 'wheel_fr_joint', 'wheel_rl_joint', 'wheel_rr_joint']
        js.position = self.wheel_positions
        js.velocity = []
        js.effort = []
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = UGVWebotsController()
        
        # Main Loop
        while rclpy.ok():
            rclpy.spin_once(controller, timeout_sec=0)
            
            if not controller.step_once():
                controller.get_logger().error("Webots step returned -1. Exiting.")
                break
                 
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()