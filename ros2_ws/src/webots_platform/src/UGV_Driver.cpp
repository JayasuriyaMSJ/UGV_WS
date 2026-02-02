#include "webots_platform/UGV_Driver.hpp"

#include <functional>
#include <cmath>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/inertial_unit.h>
#include <webots/position_sensor.h>
#include <webots/gyro.h>           // NEW
#include <webots/accelerometer.h>  // NEW
#include <webots/lidar.h>
#include <webots/camera.h>
#include <webots/range_finder.h>

namespace ugv_driver
{

void UGVDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> & /* parameters */)
{
    std::cout << "=== UGVDriver::init() starting ===" << std::endl;

    node_ = node;
    timestep_ms_ = wb_robot_get_basic_time_step();
    int timestep = timestep_ms_;

    // Handle namespacing for frames
    std::string ns = node_->get_namespace();
    if (ns == "/") {
        frame_prefix_ = "";
    } else {
        // Remove leading slash and ensure trailing slash
        if (ns.front() == '/') ns.erase(0, 1);
        frame_prefix_ = ns + "/";
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    std::cout << "Simulation timestep: " << timestep_ms_ << " ms" << std::endl;

    // Get motors
    left_front_motor_ = wb_robot_get_device("left_front_motor");
    right_front_motor_ = wb_robot_get_device("right_front_motor");
    left_rear_motor_ = wb_robot_get_device("left_rear_motor");
    right_rear_motor_ = wb_robot_get_device("right_rear_motor");

    // Get position sensors
    left_front_sensor_ = wb_robot_get_device("left_front_sensor");
    right_front_sensor_ = wb_robot_get_device("right_front_sensor");
    left_rear_sensor_ = wb_robot_get_device("left_rear_sensor");
    right_rear_sensor_ = wb_robot_get_device("right_rear_sensor");

    // Enable position sensors
    if (left_front_sensor_) wb_position_sensor_enable(left_front_sensor_, timestep);
    if (right_front_sensor_) wb_position_sensor_enable(right_front_sensor_, timestep);
    if (left_rear_sensor_) wb_position_sensor_enable(left_rear_sensor_, timestep);
    if (right_rear_sensor_) wb_position_sensor_enable(right_rear_sensor_, timestep);

    // Set motors to velocity control
    wb_motor_set_position(left_front_motor_, INFINITY);
    wb_motor_set_position(right_front_motor_, INFINITY);
    wb_motor_set_position(left_rear_motor_, INFINITY);
    wb_motor_set_position(right_rear_motor_, INFINITY);

    wb_motor_set_velocity(left_front_motor_, 0.0);
    wb_motor_set_velocity(right_front_motor_, 0.0);
    wb_motor_set_velocity(left_rear_motor_, 0.0);
    wb_motor_set_velocity(right_rear_motor_, 0.0);

    // Get IMU device and enable it
    imu_ = wb_robot_get_device("imu");
    if (imu_)
    {
        wb_inertial_unit_enable(imu_, timestep);
        std::cout << "IMU enabled" << std::endl;
    }
    else
    {
        std::cout << "WARNING: IMU not found!" << std::endl;
    }

    // Get Gyroscope - CRITICAL FOR EKF
    gyro_ = wb_robot_get_device("gyro");
    if (gyro_)
    {
        wb_gyro_enable(gyro_, timestep);
        std::cout << "Gyroscope enabled" << std::endl;
    }
    else
    {
        std::cout << "WARNING: Gyroscope not found!" << std::endl;
    }

    // Get Accelerometer - CRITICAL FOR EKF
    accelerometer_ = wb_robot_get_device("accelerometer");
    if (accelerometer_)
    {
        wb_accelerometer_enable(accelerometer_, timestep);
        std::cout << "Accelerometer enabled" << std::endl;
    }
    else
    {
        std::cout << "WARNING: Accelerometer not found!" << std::endl;
    }

    // 3D LiDAR (Velodyne VLP-16)
    lidar_3d_ = wb_robot_get_device("velodyne");
    if (lidar_3d_)
    {
        wb_lidar_enable(lidar_3d_, timestep);
        wb_lidar_enable_point_cloud(lidar_3d_);
        std::cout << "3D LiDAR (Velodyne) enabled" << std::endl;
    }
    else
    {
        std::cout << "WARNING: 3D LiDAR not found!" << std::endl;
    }

    // 2D LiDAR (SICK)
    lidar_2d_ = wb_robot_get_device("sick_2d_lidar");
    if (lidar_2d_)
    {
        wb_lidar_enable(lidar_2d_, timestep);
        std::cout << "2D LiDAR (SICK) enabled" << std::endl;
    }
    else
    {
        std::cout << "WARNING: 2D LiDAR not found!" << std::endl;
    }

    // ========== CAMERAS ==========
    // RGB Camera
    camera_rgb_ = wb_robot_get_device("camera_rgb");
    if (camera_rgb_)
    {
        wb_camera_enable(camera_rgb_, timestep);
        std::cout << "RGB Camera enabled" << std::endl;
    }
    else
    {
        std::cout << "WARNING: RGB Camera not found!" << std::endl;
    }

    // Depth Camera (RangeFinder)
    camera_depth_ = wb_robot_get_device("camera_depth");
    if (camera_depth_)
    {
        wb_range_finder_enable(camera_depth_, timestep);
        std::cout << "Depth Camera enabled" << std::endl;
    }
    else
    {
        std::cout << "WARNING: Depth Camera not found!" << std::endl;
    }

    std::cout << "Simulation timestep: " << timestep_ms_ << " ms" << std::endl;

    // ========== SUBSCRIBERS ==========

    // Subscribe to cmd_vel
    cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        rclcpp::QoS(10),
        std::bind(&UGVDriver::cmdVelCallback, this, std::placeholders::_1));

    // ========== PUBLISHERS ==========

    // Create publishers
    odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>(
        "odom",
        rclcpp::QoS(10));

    imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>(
        "imu/data",  // Changed from /UGV/imu to /imu/data for EKF
        rclcpp::QoS(10));

    joint_state_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states",
        rclcpp::QoS(10));

    // LiDAR publishers
    lidar_3d_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "velodyne/points",
        rclcpp::QoS(10));

    lidar_2d_pub_ = node->create_publisher<sensor_msgs::msg::LaserScan>(
        "scan",
        rclcpp::QoS(10));

    // Camera publishers
    camera_rgb_pub_ = node->create_publisher<sensor_msgs::msg::Image>(
        "camera/rgb/image_raw",
        rclcpp::QoS(10));

    camera_rgb_info_pub_ = node->create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/rgb/camera_info",
        rclcpp::QoS(10));

    camera_depth_pub_ = node->create_publisher<sensor_msgs::msg::Image>(
        "camera/depth/image_raw",
        rclcpp::QoS(10));

    camera_depth_info_pub_ = node->create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/depth/camera_info",
        rclcpp::QoS(10));

    std::cout << "=== UGVDriver::init() complete ===" << std::endl;
}

void UGVDriver::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    cmd_vel_ = *msg;
}

void UGVDriver::updateOdometry(const rclcpp::Time &current_time)
{
    double current_sim_time = wb_robot_get_time();
    if (first_update_)
    {
        first_update_ = false;
        last_sim_time_ = current_sim_time;
        if (left_front_sensor_ && right_front_sensor_)
        {
            last_left_position_ = (wb_position_sensor_get_value(left_front_sensor_) + wb_position_sensor_get_value(left_rear_sensor_)) / 2.0;
            last_right_position_ = (wb_position_sensor_get_value(right_front_sensor_) + wb_position_sensor_get_value(right_rear_sensor_)) / 2.0;
        }
        return;
    }

    double dt = current_sim_time - last_sim_time_;
    last_sim_time_ = current_sim_time;

    if (dt <= 0.0 || !left_front_sensor_ || !right_front_sensor_)
    {
        return;
    }

    // Get average encoder values for robustness
    double left_pos = (wb_position_sensor_get_value(left_front_sensor_) + wb_position_sensor_get_value(left_rear_sensor_)) / 2.0;
    double right_pos = (wb_position_sensor_get_value(right_front_sensor_) + wb_position_sensor_get_value(right_rear_sensor_)) / 2.0;

    // Sanity check for NaNs (can happen on first few frames)
    if (std::isnan(left_pos) || std::isnan(right_pos))
    {
        return;
    }

    // Calculate wheel displacement
    double delta_left = (left_pos - last_left_position_) * wheel_radius_;
    double delta_right = (right_pos - last_right_position_) * wheel_radius_;

    // Calculate robot displacement
    double delta_s = (delta_right + delta_left) / 2.0;
    double delta_theta = (delta_right - delta_left) / track_width_;

    // Update pose
    double theta_old = theta_;
    theta_ += delta_theta;

    // Use IMU if available for more reliable yaw
    if (imu_)
    {
        const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu_);
        if (rpy && !std::isnan(rpy[2]))
        {
            theta_ = rpy[2];
        }
    }

    // Normalize theta (radiance formula / normalization)
    theta_ = atan2(sin(theta_), cos(theta_));

    x_ += delta_s * cos((theta_old + theta_) / 2.0);
    y_ += delta_s * sin((theta_old + theta_) / 2.0);

    // Calculate velocities
    double v = delta_s / dt;
    double w = delta_theta / dt;

    // Update last values
    last_left_position_ = left_pos;
    last_right_position_ = right_pos;

    // Create and publish odometry message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;  // Use synchronized timestamp
    odom_msg.header.frame_id = frame_prefix_ + "odom";
    odom_msg.child_frame_id = frame_prefix_ + "base_link";

    // Position
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    // Orientation (quaternion from yaw)
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(theta_ / 2.0);
    odom_msg.pose.pose.orientation.w = cos(theta_ / 2.0);

    // Velocity
    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = w;

    // Covariance (tuned for differential drive on smooth surface)
    // Position covariance
    odom_msg.pose.covariance[0] = 0.001;   // x
    odom_msg.pose.covariance[7] = 0.001;   // y
    odom_msg.pose.covariance[14] = 1e6;    // z (not used)
    odom_msg.pose.covariance[21] = 1e6;    // roll (not used)
    odom_msg.pose.covariance[28] = 1e6;    // pitch (not used)
    odom_msg.pose.covariance[35] = 0.01;   // yaw

    // Velocity covariance
    odom_msg.twist.covariance[0] = 0.001;  // vx
    odom_msg.twist.covariance[7] = 1e6;    // vy (not used)
    odom_msg.twist.covariance[14] = 1e6;   // vz (not used)
    odom_msg.twist.covariance[21] = 1e6;   // roll rate (not used)
    odom_msg.twist.covariance[28] = 1e6;   // pitch rate (not used)
    odom_msg.twist.covariance[35] = 0.01;  // yaw rate

    odom_pub_->publish(odom_msg);

    // Create and publish transform
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = current_time;
    t.header.frame_id = frame_prefix_ + "odom";
    t.child_frame_id = frame_prefix_ + "base_link";

    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);

    // Publish Joint States for RViz wheel visualization
    sensor_msgs::msg::JointState js;
    js.header.stamp = current_time;
    js.name = {"left_front_motor", "right_front_motor", "left_rear_motor", "right_rear_motor"};
    js.position = {
        wb_position_sensor_get_value(left_front_sensor_),
        wb_position_sensor_get_value(right_front_sensor_),
        wb_position_sensor_get_value(left_rear_sensor_),
        wb_position_sensor_get_value(right_rear_sensor_)
    };
    joint_state_pub_->publish(js);
}

void UGVDriver::publishIMU(const rclcpp::Time &current_time)
{
    if (!imu_)
    {
        return;
    }

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = current_time;  // Use synchronized timestamp
    imu_msg.header.frame_id = frame_prefix_ + "imu"; // Fixed: name in URDF is 'imu'

    // ===== ORIENTATION (from InertialUnit) =====
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu_);
    if (rpy)
    {
        double roll = rpy[0];
        double pitch = rpy[1];
        double yaw = rpy[2];

        // Convert RPY to quaternion
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;

        // Orientation covariance (relatively accurate in simulation)
        imu_msg.orientation_covariance[0] = 0.01;  // roll
        imu_msg.orientation_covariance[4] = 0.01;  // pitch
        imu_msg.orientation_covariance[8] = 0.01;  // yaw
    }
    else
    {
        // If orientation unavailable, mark as unknown
        imu_msg.orientation_covariance[0] = -1;
    }

    // ===== ANGULAR VELOCITY (from Gyroscope) - CRITICAL FOR EKF =====
    if (gyro_)
    {
        const double *gyro_values = wb_gyro_get_values(gyro_);
        if (gyro_values)
        {
            imu_msg.angular_velocity.x = gyro_values[0];
            imu_msg.angular_velocity.y = gyro_values[1];
            imu_msg.angular_velocity.z = gyro_values[2];

            // Angular velocity covariance
            imu_msg.angular_velocity_covariance[0] = 0.001;  // x
            imu_msg.angular_velocity_covariance[4] = 0.001;  // y
            imu_msg.angular_velocity_covariance[8] = 0.001;  // z
        }
        else
        {
            imu_msg.angular_velocity_covariance[0] = -1;
        }
    }
    else
    {
        // Mark as unavailable
        imu_msg.angular_velocity_covariance[0] = -1;
    }

    // ===== LINEAR ACCELERATION (from Accelerometer) - CRITICAL FOR EKF =====
    if (accelerometer_)
    {
        const double *accel_values = wb_accelerometer_get_values(accelerometer_);
        if (accel_values)
        {
            imu_msg.linear_acceleration.x = accel_values[0];
            imu_msg.linear_acceleration.y = accel_values[1];
            imu_msg.linear_acceleration.z = accel_values[2];

            // Linear acceleration covariance
            imu_msg.linear_acceleration_covariance[0] = 0.01;  // x
            imu_msg.linear_acceleration_covariance[4] = 0.01;  // y
            imu_msg.linear_acceleration_covariance[8] = 0.01;  // z
        }
        else
        {
            imu_msg.linear_acceleration_covariance[0] = -1;
        }
    }
    else
    {
        // Mark as unavailable
        imu_msg.linear_acceleration_covariance[0] = -1;
    }

    imu_pub_->publish(imu_msg);
}

void UGVDriver::publishLidar3D(const rclcpp::Time &current_time)
{
    if (!lidar_3d_)
    {
        return;
    }

    const WbLidarPoint *point_cloud = wb_lidar_get_point_cloud(lidar_3d_);
    int num_points = wb_lidar_get_number_of_points(lidar_3d_);

    if (!point_cloud || num_points <= 0)
    {
        return;
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = current_time;
    cloud_msg.header.frame_id = frame_prefix_ + "velodyne";

    // PointCloud2 field setup
    cloud_msg.height = 1;
    cloud_msg.width = num_points;
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = false;

    // Define fields: x, y, z, intensity
    sensor_msgs::msg::PointField field_x, field_y, field_z, field_intensity;
    
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count = 1;

    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;

    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;

    field_intensity.name = "intensity";
    field_intensity.offset = 12;
    field_intensity.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_intensity.count = 1;

    cloud_msg.fields = {field_x, field_y, field_z, field_intensity};
    cloud_msg.point_step = 16; // 4 floats * 4 bytes
    cloud_msg.row_step = cloud_msg.point_step * num_points;

    // Allocate data buffer
    cloud_msg.data.resize(cloud_msg.row_step);

    // Fill point cloud data
    for (int i = 0; i < num_points; ++i)
    {
        const WbLidarPoint &pt = point_cloud[i];
        
        // Convert to float*
        float *data_ptr = reinterpret_cast<float*>(&cloud_msg.data[i * cloud_msg.point_step]);
        
        data_ptr[0] = static_cast<float>(pt.x);
        data_ptr[1] = static_cast<float>(pt.y);
        data_ptr[2] = static_cast<float>(pt.z);
        data_ptr[3] = 100.0f; // Dummy intensity value
    }

    lidar_3d_pub_->publish(cloud_msg);
}

void UGVDriver::publishLidar2D(const rclcpp::Time &current_time)
{
    if (!lidar_2d_)
    {
        return;
    }

    const float *range_image = wb_lidar_get_range_image(lidar_2d_);
    int horizontal_resolution = wb_lidar_get_horizontal_resolution(lidar_2d_);
    double fov = wb_lidar_get_fov(lidar_2d_);
    double max_range = wb_lidar_get_max_range(lidar_2d_);
    double min_range = wb_lidar_get_min_range(lidar_2d_);

    if (!range_image || horizontal_resolution <= 0)
    {
        return;
    }

    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.stamp = current_time;
    scan_msg.header.frame_id = frame_prefix_ + "sick_2d_lidar";

    scan_msg.angle_min = -fov / 2.0;
    scan_msg.angle_max = fov / 2.0;
    scan_msg.angle_increment = fov / (horizontal_resolution - 1);
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = timestep_ms_ / 1000.0;
    scan_msg.range_min = min_range;
    scan_msg.range_max = max_range;

    scan_msg.ranges.resize(horizontal_resolution);
    scan_msg.intensities.resize(horizontal_resolution);

    for (int i = 0; i < horizontal_resolution; ++i)
    {
        float range = range_image[i];
        
        // Handle infinite/invalid readings
        if (std::isinf(range) || range >= max_range)
        {
            scan_msg.ranges[i] = max_range + 1.0; // Mark as out of range
        }
        else if (range < min_range)
        {
            scan_msg.ranges[i] = min_range - 0.01; // Mark as too close
        }
        else
        {
            scan_msg.ranges[i] = range;
        }
        
        scan_msg.intensities[i] = 100.0; // Dummy intensity
    }

    lidar_2d_pub_->publish(scan_msg);
}

void UGVDriver::publishCamera(const rclcpp::Time &current_time)
{
    if (!camera_rgb_)
    {
        return;
    }

    const unsigned char *image_data = wb_camera_get_image(camera_rgb_);
    int width = wb_camera_get_width(camera_rgb_);
    int height = wb_camera_get_height(camera_rgb_);
    double fov = wb_camera_get_fov(camera_rgb_);

    if (!image_data || width <= 0 || height <= 0)
    {
        return;
    }

    // Publish Image
    sensor_msgs::msg::Image img_msg;
    img_msg.header.stamp = current_time;
    img_msg.header.frame_id = frame_prefix_ + "camera_rgb";
    img_msg.height = height;
    img_msg.width = width;
    img_msg.encoding = "bgra8"; // Webots uses BGRA format
    img_msg.is_bigendian = false;
    img_msg.step = width * 4; // 4 bytes per pixel (BGRA)

    size_t data_size = img_msg.step * height;
    img_msg.data.resize(data_size);
    std::memcpy(img_msg.data.data(), image_data, data_size);

    camera_rgb_pub_->publish(img_msg);

    // Publish CameraInfo
    sensor_msgs::msg::CameraInfo info_msg;
    info_msg.header = img_msg.header;
    info_msg.height = height;
    info_msg.width = width;
    info_msg.distortion_model = "plumb_bob";

    // Calculate focal length from FOV
    double focal_length = (width / 2.0) / tan(fov / 2.0);

    // Camera matrix [fx 0 cx; 0 fy cy; 0 0 1]
    info_msg.k[0] = focal_length;  // fx
    info_msg.k[2] = width / 2.0;   // cx
    info_msg.k[4] = focal_length;  // fy
    info_msg.k[5] = height / 2.0;  // cy
    info_msg.k[8] = 1.0;

    // Rectification matrix (identity for monocular camera)
    info_msg.r[0] = 1.0;
    info_msg.r[4] = 1.0;
    info_msg.r[8] = 1.0;

    // Projection matrix [fx' 0 cx' 0; 0 fy' cy' 0; 0 0 1 0]
    info_msg.p[0] = focal_length;
    info_msg.p[2] = width / 2.0;
    info_msg.p[5] = focal_length;
    info_msg.p[6] = height / 2.0;
    info_msg.p[10] = 1.0;

    // No distortion coefficients
    info_msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    camera_rgb_info_pub_->publish(info_msg);
}

void UGVDriver::publishDepthCamera(const rclcpp::Time &current_time)
{
    if (!camera_depth_)
    {
        return;
    }

    const float *range_image = wb_range_finder_get_range_image(camera_depth_);
    int width = wb_range_finder_get_width(camera_depth_);
    int height = wb_range_finder_get_height(camera_depth_);
    double fov = wb_range_finder_get_fov(camera_depth_);
    double max_range = wb_range_finder_get_max_range(camera_depth_);

    if (!range_image || width <= 0 || height <= 0)
    {
        return;
    }

    // Publish Depth Image (as 32FC1)
    sensor_msgs::msg::Image depth_msg;
    depth_msg.header.stamp = current_time;
    depth_msg.header.frame_id = frame_prefix_ + "camera_depth";
    depth_msg.height = height;
    depth_msg.width = width;
    depth_msg.encoding = "32FC1"; // Single channel float32
    depth_msg.is_bigendian = false;
    depth_msg.step = width * sizeof(float);

    size_t data_size = depth_msg.step * height;
    depth_msg.data.resize(data_size);

    // Copy and convert range data
    float *depth_data = reinterpret_cast<float*>(depth_msg.data.data());
    for (int i = 0; i < width * height; ++i)
    {
        float range = range_image[i];
        
        // Handle infinite readings
        if (std::isinf(range) || range >= max_range)
        {
            depth_data[i] = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
            depth_data[i] = range;
        }
    }

    camera_depth_pub_->publish(depth_msg);

    // Publish CameraInfo
    sensor_msgs::msg::CameraInfo info_msg;
    info_msg.header = depth_msg.header;
    info_msg.height = height;
    info_msg.width = width;
    info_msg.distortion_model = "plumb_bob";

    // Calculate focal length from FOV
    double focal_length = (width / 2.0) / tan(fov / 2.0);

    // Camera matrix
    info_msg.k[0] = focal_length;
    info_msg.k[2] = width / 2.0;
    info_msg.k[4] = focal_length;
    info_msg.k[5] = height / 2.0;
    info_msg.k[8] = 1.0;

    // Rectification matrix
    info_msg.r[0] = 1.0;
    info_msg.r[4] = 1.0;
    info_msg.r[8] = 1.0;

    // Projection matrix
    info_msg.p[0] = focal_length;
    info_msg.p[2] = width / 2.0;
    info_msg.p[5] = focal_length;
    info_msg.p[6] = height / 2.0;
    info_msg.p[10] = 1.0;

    info_msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    camera_depth_info_pub_->publish(info_msg);
}

void UGVDriver::step()
{
    // Get synchronized timestamp ONCE per step
    rclcpp::Time current_time = node_->now();

    // Apply motor commands
    const double v = cmd_vel_.linear.x;
    const double w = cmd_vel_.angular.z;

    const double v_left = v - (w * track_width_ / 2.0);
    const double v_right = v + (w * track_width_ / 2.0);

    const double left_speed = v_left / wheel_radius_;
    const double right_speed = v_right / wheel_radius_;

    wb_motor_set_velocity(left_front_motor_, left_speed);
    wb_motor_set_velocity(left_rear_motor_, left_speed);
    wb_motor_set_velocity(right_front_motor_, right_speed);
    wb_motor_set_velocity(right_rear_motor_, right_speed);

    // Publish with synchronized timestamp
    updateOdometry(current_time);
    publishIMU(current_time);
}

UGVDriver::~UGVDriver()
{
    // Cleanup
}

} // namespace ugv_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    ugv_driver::UGVDriver,
    webots_ros2_driver::PluginInterface)
    