#include "webots_platform/UGV_Driver.hpp"

#include <functional>
#include <cmath>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/inertial_unit.h>
#include <webots/position_sensor.h>
#include <webots/gyro.h>           // NEW
#include <webots/accelerometer.h>  // NEW

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

    std::cout << "Simulation timestep: " << timestep_ms_ << " ms" << std::endl;

    // Subscribe to cmd_vel
    cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        rclcpp::QoS(10),
        std::bind(&UGVDriver::cmdVelCallback, this, std::placeholders::_1));

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
    