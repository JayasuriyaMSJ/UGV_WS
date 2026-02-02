#ifndef WEBOTS_ROS2_UGV_DRIVE_HPP
#define WEBOTS_ROS2_UGV_DRIVE_HPP

#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "rosgraph_msgs/msg/clock.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

namespace ugv_driver
{
    class UGVDriver : public webots_ros2_driver::PluginInterface
    {
    public:
        UGVDriver() = default;
        ~UGVDriver();

        void init(
            webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;

        void step() override;

    private:
        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void updateOdometry(const rclcpp::Time &current_time);
        void publishIMU(const rclcpp::Time &current_time);

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        geometry_msgs::msg::Twist cmd_vel_;

        // Webots devices
        WbDeviceTag left_front_motor_;
        WbDeviceTag right_front_motor_;
        WbDeviceTag left_rear_motor_;
        WbDeviceTag right_rear_motor_;
        WbDeviceTag left_front_sensor_;
        WbDeviceTag right_front_sensor_;
        WbDeviceTag left_rear_sensor_;
        WbDeviceTag right_rear_sensor_;
        WbDeviceTag imu_;
        WbDeviceTag gyro_;          // NEW: Gyroscope
        WbDeviceTag accelerometer_; // NEW: Accelerometer

        // Robot parameters
        double wheel_radius_ = 0.1;
        double track_width_ = 0.6;

        // Odometry state
        double x_ = 0.0;
        double y_ = 0.0;
        double theta_ = 0.0;
        double last_left_position_ = 0.0;
        double last_right_position_ = 0.0;
        std::string frame_prefix_;

        // IMU state - NEW
        double last_angular_z_ = 0.0;
        double last_linear_x_ = 0.0;
        double last_linear_y_ = 0.0;

        webots_ros2_driver::WebotsNode *node_;
        bool first_update_ = true;
        int timestep_ms_;
        double last_sim_time_ = 0.0;
    };

} // namespace ugv_driver

#endif