#!/usr/bin/python3
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
from sensor_msgs.msg import LaserScan, Image, JointState, PointCloud2, PointField, Imu, CameraInfo
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import math
from controller import Robot, Motor, Lidar, Camera
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
import struct


class UGVWebotsController(Node):
    """Production-grade UGV controller for Webots."""
    
    def __init__(self):
        super().__init__('ugv_webots_controller')
        
        # Initialize Webots Robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Robot parameters
        self.wheel_radius = 0.1  # meters
        self.wheel_base = 0.4    # meters between left and right wheels
        self.max_speed = 5000.0    # rad/s (Increased for faster mapping/nav)
        
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
        self.depth_info_pub = self.create_publisher(
            CameraInfo,
            '/camera/depth/camera_info',
            10
        )
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)

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
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR
        
        # Clock publisher
        self.clock_pub = self.create_publisher(Clock, 'clock', 10)
        
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
            self.get_logger().info('InertialUnit (IMU) enabled')
        else:
            self.get_logger().warn('InertialUnit (IMU) not found')
            self.inertial = None

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

        # Depth camera
        self.depth_camera = self.robot.getDevice('realsense_depth')
        if self.depth_camera:
            self.depth_camera.enable(self.timestep)
            self.get_logger().info(f'Depth camera enabled: {self.depth_camera.getWidth()}x{self.depth_camera.getHeight()}')
        else:
            self.get_logger().warn('Depth camera not found')

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

        
        # Last sensor values
        self.last_left_pos = 0.0
        self.last_right_pos = 0.0
        self.initialized = False
    
    def cmd_vel_callback(self, msg):
        """Handle cmd_vel messages."""
        # self.get_logger().info(f"Received cmd_vel: Lin={msg.linear.x}, Ang={msg.angular.z}")
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
    
    def step_once(self):
        """Single simulation step."""
        # Step simulation
        if self.robot.step(self.timestep) == -1:
            return False
            
        # 1. Publish Clock
        current_time = self.robot.getTime()
        
        # Webots time is float seconds. Convert to ROS Time.
        # This is CRITICAL for Nav2 and ROS Timers to work.
        seconds = int(current_time)
        nanoseconds = int((current_time - seconds) * 1e9)
        
        ros_time_msg = Time()
        ros_time_msg.sec = seconds
        ros_time_msg.nanosec = nanoseconds
        
        clock_msg = Clock()
        clock_msg.clock = ros_time_msg
        self.clock_pub.publish(clock_msg)
        
        # 2. Update Dynamics
        dt = current_time - self.last_time
        if dt <= 0: return True # Wait for time to progress
        self.last_time = current_time
        
        # Compute and apply motor velocities
        omega_left_cmd, omega_right_cmd = self.compute_wheel_velocities()
        
        for motor in self.left_motors:
            if motor:
                motor.setVelocity(omega_left_cmd)
        
        for motor in self.right_motors:
            if motor:
                motor.setVelocity(omega_right_cmd)
        
        # Update wheel positions for visualization
        self.wheel_positions[0] += omega_left_cmd * dt
        self.wheel_positions[1] += omega_right_cmd * dt
        self.wheel_positions[2] += omega_left_cmd * dt
        self.wheel_positions[3] += omega_right_cmd * dt

        # Update and publish odometry
        try:
            self.publish_odometry(dt, ros_time_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing odometry: {e}")
        
        # Publish sensors
        try:
            self.publish_imu(ros_time_msg)
            self.publish_lidar(ros_time_msg)
            self.publish_lidar_3d(ros_time_msg)
            self.publish_camera(ros_time_msg)
            self.publish_depth(ros_time_msg)
            self.publish_depth_camera_info(ros_time_msg)
            self.publish_joint_states(ros_time_msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing sensors: {e}")
        
        return True

    def publish_odometry(self, dt, current_time):
        """Update and publish odometry and TF."""
        # Get actual wheel positions
        l_sens = [s for s in self.left_sensors if s]
        r_sens = [s for s in self.right_sensors if s]
        
        if not l_sens or not r_sens:
            self.get_logger().warn_once("Wheel sensors not found, skipping odometry.")
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

        # Calculate deltas
        d_left = (left_pos - self.last_left_pos) * self.wheel_radius
        d_right = (right_pos - self.last_right_pos) * self.wheel_radius
        
        self.last_left_pos = left_pos
        self.last_right_pos = right_pos

        # Distance and rotation deltas
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
        
        # Use average theta for more accurate integration
        theta_avg = (theta_old + self.theta) / 2.0
        if abs(self.theta - theta_old) > math.pi: # Wrap around
             theta_avg = self.theta # Fallback for wrap

        self.x += dist * math.cos(theta_avg)
        self.y += dist * math.sin(theta_avg)
        
        # Linear and angular velocities for msg
        v = dist / dt
        omega = (self.theta - theta_old) / dt
        if omega > math.pi / dt: omega -= 2*math.pi/dt
        if omega < -math.pi / dt: omega += 2*math.pi/dt
        
        # Odometry message
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = current_time
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

    def publish_imu(self, current_time):
        if not (self.inertial and self.gyro and self.accel):
            return

        # Orientation
        roll, pitch, yaw = self.inertial.getRollPitchYaw()

        # Angular velocity
        wx, wy, wz = self.gyro.getValues()

        # Linear acceleration
        ax, ay, az = self.accel.getValues()

        imu_msg = Imu()
        imu_msg.header.stamp = current_time
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

    def publish_lidar_3d(self, current_time):
        if self.lidar_3d is None:
            return

        try:
            point_cloud = self.lidar_3d.getPointCloud()
            if not point_cloud:
                return

            cloud_msg = PointCloud2()
            cloud_msg.header.stamp = current_time
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

    def publish_lidar(self, current_time):
        """Publish lidar scan."""
        if not self.lidar:
            return
        
        ranges = self.lidar.getRangeImage()
        if not ranges:
            return
        
        ranges = list(ranges)[::-1]
        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = current_time
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

    def publish_camera(self, current_time):
        """Publish camera image."""
        if not self.camera:
            return
        
        image_data = self.camera.getImage()


        if not image_data:
            return
        
        info = CameraInfo()
        info.header.stamp = current_time
        info.header.frame_id = 'realsense_link'
        info.width = self.camera.getWidth()
        info.height = self.camera.getHeight()
        
        # Simple pinhole camera model (adjust fx, fy, cx, cy as needed)
        fx = fy = info.width / (2 * math.tan(self.camera.getFov() / 2))
        cx = info.width / 2.0
        cy = info.height / 2.0
        
        info.k = [fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0]
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.camera_info_pub.publish(info)
        
        img = Image()
        img.header = Header()
        img.header.stamp = current_time
        img.header.frame_id = 'realsense_link'
        img.height = self.camera.getHeight()
        img.width = self.camera.getWidth()
        img.encoding = 'bgra8'
        img.is_bigendian = False
        img.step = img.width * 4
        img.data = list(image_data)
        
        self.image_pub.publish(img)

    def publish_joint_states(self, current_time):
        js = JointState()
        js.header = Header()
        js.header.stamp = current_time
        js.name = ['wheel_fl_joint', 'wheel_fr_joint', 'wheel_rl_joint', 'wheel_rr_joint']
        js.position = self.wheel_positions
        js.velocity = []
        js.effort = []
        self.joint_pub.publish(js)

    def publish_depth_camera_info(self, current_time):
        if not self.depth_camera:
            return

        info = CameraInfo()
        info.header.stamp = current_time
        info.header.frame_id = 'realsense_link'

        width = self.depth_camera.getWidth()
        height = self.depth_camera.getHeight()
        fov = self.depth_camera.getFov()  # radians (horizontal)

        info.width = width
        info.height = height

        # Pinhole model from Webots RangeFinder
        fx = width / (2.0 * math.tan(fov / 2.0))
        fy = fx  # assume square pixels
        cx = width / 2.0
        cy = height / 2.0

        info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]

        info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.depth_info_pub.publish(info)


    def publish_depth(self, current_time):
        """Publish depth image in 16UC1 format (millimeters)."""
        if not self.depth_camera:
            return
        
        depth_data = self.depth_camera.getRangeImage()
        if not depth_data:
            return
        
        depth_img = Image()
        depth_img.header = Header()
        depth_img.header.stamp = current_time
        depth_img.header.frame_id = 'realsense_link'
        depth_img.height = self.depth_camera.getHeight()
        depth_img.width = self.depth_camera.getWidth()
        depth_img.encoding = '16UC1'  # 16-bit unsigned int (millimeters)
        depth_img.is_bigendian = False
        depth_img.step = depth_img.width * 2  # 2 bytes per pixel
        
        # Convert meters to millimeters, handle invalid values
        import array
        max_range = self.depth_camera.getMaxRange()
        
        processed_depth = []
        for val in depth_data:
            if val >= max_range or math.isinf(val) or val <= 0.0:
                processed_depth.append(0)  # 0 = invalid in 16UC1
            else:
                # Convert meters to millimeters, clamp to uint16 range
                mm_val = int(val * 1000.0)
                processed_depth.append(min(mm_val, 65535))
        
        depth_array = array.array('H', processed_depth)  # 'H' = unsigned short
        depth_img.data = depth_array.tobytes()
        
        self.depth_pub.publish(depth_img)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = UGVWebotsController()
        
        # Main Loop behaving as Clock Source + Driver
        iter_count = 0
        while rclpy.ok():
            # 1. Spin ROS to handle callbacks (processing cmd_vel)
            rclpy.spin_once(controller, timeout_sec=0)
            
            # 2. Step Physics & Publish Clock & Dynamics
            if not controller.step_once():
                controller.get_logger().error("Webots step returned -1 (Connection Lost). Exiting loop.")
                break
                
            iter_count += 1
            if iter_count % 500 == 0:
                 pass # Be quiet to avoid spam, but loop is working
                 
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
