// FROM CLAUDE:

#include "webots_platform/UGV_Driver.hpp"

#include <functional>
#include <cmath>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/inertial_unit.h>
#include <webots/position_sensor.h>
#include <webots/gyro.h>
#include <webots/accelerometer.h>
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

        // ========== NAMESPACE HANDLING ==========
        // WebotsController passes robot_name as a parameter, not as the node namespace.
        // node_->get_namespace() returns "/" inside the plugin.
        // We get the robot name from the node name instead:
        //   WebotsController names the node: "webots_controller_<robot_name>"
        // OR we just read the robot_name param that WebotsController sets.
        // Safest: extract from node name.

        std::string node_name = node_->get_name();
        std::cout << "Plugin node name: " << node_name << std::endl;
        std::cout << "Plugin node namespace: " << node_->get_namespace() << std::endl;

        // node name is "webots_controller_ugv_01" → extract "ugv_01"
        std::string prefix = "webots_controller_";
        if (node_name.substr(0, prefix.size()) == prefix)
        {
            robot_name_ = node_name.substr(prefix.size());
        }
        else
        {
            // fallback: use namespace if it ever works
            std::string ns = node_->get_namespace();
            if (ns != "/")
            {
                if (ns.front() == '/') ns.erase(0, 1);
                robot_name_ = ns;
            }
            else
            {
                robot_name_ = "";
            }
        }

        // frame_prefix_ = "ugv_01/" used in TF frame_ids
        // topic_prefix_ = "ugv_01/" used in topic names
        if (robot_name_.empty())
        {
            frame_prefix_ = "";
            topic_prefix_ = "";
        }
        else
        {
            frame_prefix_ = robot_name_ + "/";
            topic_prefix_ = robot_name_ + "/";
        }

        std::cout << "Robot name:     '" << robot_name_ << "'" << std::endl;
        std::cout << "Frame prefix:   '" << frame_prefix_ << "'" << std::endl;
        std::cout << "Topic prefix:   '" << topic_prefix_ << "'" << std::endl;

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

        std::cout << "Simulation timestep: " << timestep_ms_ << " ms" << std::endl;

        // ========== MOTORS ==========
        left_front_motor_ = wb_robot_get_device("left_front_motor");
        right_front_motor_ = wb_robot_get_device("right_front_motor");
        left_rear_motor_ = wb_robot_get_device("left_rear_motor");
        right_rear_motor_ = wb_robot_get_device("right_rear_motor");

        // ========== POSITION SENSORS ==========
        left_front_sensor_ = wb_robot_get_device("left_front_sensor");
        right_front_sensor_ = wb_robot_get_device("right_front_sensor");
        left_rear_sensor_ = wb_robot_get_device("left_rear_sensor");
        right_rear_sensor_ = wb_robot_get_device("right_rear_sensor");

        if (left_front_sensor_)  wb_position_sensor_enable(left_front_sensor_, timestep);
        if (right_front_sensor_) wb_position_sensor_enable(right_front_sensor_, timestep);
        if (left_rear_sensor_)   wb_position_sensor_enable(left_rear_sensor_, timestep);
        if (right_rear_sensor_)  wb_position_sensor_enable(right_rear_sensor_, timestep);

        // Velocity control mode
        wb_motor_set_position(left_front_motor_, INFINITY);
        wb_motor_set_position(right_front_motor_, INFINITY);
        wb_motor_set_position(left_rear_motor_, INFINITY);
        wb_motor_set_position(right_rear_motor_, INFINITY);

        wb_motor_set_velocity(left_front_motor_, 0.0);
        wb_motor_set_velocity(right_front_motor_, 0.0);
        wb_motor_set_velocity(left_rear_motor_, 0.0);
        wb_motor_set_velocity(right_rear_motor_, 0.0);

        // ========== IMU ==========
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

        // ========== GYROSCOPE ==========
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

        // ========== ACCELEROMETER ==========
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

        // ========== 3D LIDAR (Velodyne) ==========
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

        // ========== 2D LIDAR (SICK) ==========
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

        // ========== RGB CAMERA ==========
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

        // ========== DEPTH CAMERA ==========
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

        // ========== SUBSCRIBERS ==========
        // topic_prefix_ is prepended manually so the topic lands under /ugv_01/
        cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
            topic_prefix_ + "cmd_vel",
            rclcpp::QoS(10),
            std::bind(&UGVDriver::cmdVelCallback, this, std::placeholders::_1));

        // ========== PUBLISHERS ==========
        odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>(
            topic_prefix_ + "odom",
            rclcpp::QoS(10));

        imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>(
            topic_prefix_ + "imu/data",
            rclcpp::QoS(10));

        joint_state_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(
            topic_prefix_ + "joint_states",
            rclcpp::QoS(10));

        lidar_3d_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
            topic_prefix_ + "velodyne_points",
            rclcpp::QoS(10));

        lidar_2d_pub_ = node->create_publisher<sensor_msgs::msg::LaserScan>(
            topic_prefix_ + "scan",
            rclcpp::QoS(10));

        camera_rgb_pub_ = node->create_publisher<sensor_msgs::msg::Image>(
            topic_prefix_ + "camera/rgb/image_raw",
            rclcpp::QoS(10));

        camera_rgb_info_pub_ = node->create_publisher<sensor_msgs::msg::CameraInfo>(
            topic_prefix_ + "camera/rgb/camera_info",
            rclcpp::QoS(10));

        camera_depth_pub_ = node->create_publisher<sensor_msgs::msg::Image>(
            topic_prefix_ + "camera/depth/image_raw",
            rclcpp::QoS(10));

        camera_depth_info_pub_ = node->create_publisher<sensor_msgs::msg::CameraInfo>(
            topic_prefix_ + "camera/depth/camera_info",
            rclcpp::QoS(10));

        clock_pub_ = node->create_publisher<rosgraph_msgs::msg::Clock>(
            "/clock",
            rclcpp::QoS(10));

        std::cout << "=== UGVDriver::init() complete ===" << std::endl;

        std::cout << "=== UGVDriver::init() complete ===" << std::endl;
        std::cout << "Topics will publish under: /" << topic_prefix_ << std::endl;
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
                last_left_position_ =
                    (wb_position_sensor_get_value(left_front_sensor_) +
                     wb_position_sensor_get_value(left_rear_sensor_)) / 2.0;
                last_right_position_ =
                    (wb_position_sensor_get_value(right_front_sensor_) +
                     wb_position_sensor_get_value(right_rear_sensor_)) / 2.0;
            }
            return;
        }

        double dt = current_sim_time - last_sim_time_;
        last_sim_time_ = current_sim_time;

        if (dt <= 0.0 || !left_front_sensor_ || !right_front_sensor_)
        {
            return;
        }

        double left_pos =
            (wb_position_sensor_get_value(left_front_sensor_) +
             wb_position_sensor_get_value(left_rear_sensor_)) / 2.0;
        double right_pos =
            (wb_position_sensor_get_value(right_front_sensor_) +
             wb_position_sensor_get_value(right_rear_sensor_)) / 2.0;

        if (std::isnan(left_pos) || std::isnan(right_pos))
        {
            return;
        }

        double delta_left  = (left_pos  - last_left_position_)  * wheel_radius_;
        double delta_right = (right_pos - last_right_position_) * wheel_radius_;

        double delta_s     = (delta_right + delta_left) / 2.0;
        double delta_theta = (delta_right - delta_left) / track_width_;

        double theta_old = theta_;
        theta_ += delta_theta;

        // Use IMU yaw directly (ground truth in sim)
        if (imu_)
        {
            const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu_);
            if (rpy && !std::isnan(rpy[2]))
            {
                theta_ = rpy[2];
            }
        }

        theta_ = atan2(sin(theta_), cos(theta_));

        x_ += delta_s * cos((theta_old + theta_) / 2.0);
        y_ += delta_s * sin((theta_old + theta_) / 2.0);

        double v = delta_s / dt;
        double w = delta_theta / dt;

        last_left_position_  = left_pos;
        last_right_position_ = right_pos;

        // ── Odometry message ──
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp    = current_time;
        odom_msg.header.frame_id = frame_prefix_ + "odom";
        odom_msg.child_frame_id  = frame_prefix_ + "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = sin(theta_ / 2.0);
        odom_msg.pose.pose.orientation.w = cos(theta_ / 2.0);

        odom_msg.twist.twist.linear.x  = v;
        odom_msg.twist.twist.linear.y  = 0.0;
        odom_msg.twist.twist.angular.z = w;

        // Covariance
        odom_msg.pose.covariance[0]  = 0.001;  // x
        odom_msg.pose.covariance[7]  = 0.001;  // y
        odom_msg.pose.covariance[14] = 1e6;    // z
        odom_msg.pose.covariance[21] = 1e6;    // roll
        odom_msg.pose.covariance[28] = 1e6;    // pitch
        odom_msg.pose.covariance[35] = 0.01;   // yaw

        odom_msg.twist.covariance[0]  = 0.001;
        odom_msg.twist.covariance[7]  = 1e6;
        odom_msg.twist.covariance[14] = 1e6;
        odom_msg.twist.covariance[21] = 1e6;
        odom_msg.twist.covariance[28] = 1e6;
        odom_msg.twist.covariance[35] = 0.01;

        odom_pub_->publish(odom_msg);

        // ── TF: odom → base_link ──
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp    = current_time;
        t.header.frame_id = frame_prefix_ + "odom";
        t.child_frame_id  = frame_prefix_ + "base_link";

        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;
        t.transform.rotation      = odom_msg.pose.pose.orientation;

        tf_broadcaster_->sendTransform(t);

        // ── Joint states ──
        sensor_msgs::msg::JointState js;
        js.header.stamp = current_time;
        js.name = {"left_front_motor", "right_front_motor",
                   "left_rear_motor",  "right_rear_motor"};
        js.position = {
            wb_position_sensor_get_value(left_front_sensor_),
            wb_position_sensor_get_value(right_front_sensor_),
            wb_position_sensor_get_value(left_rear_sensor_),
            wb_position_sensor_get_value(right_rear_sensor_)};
        joint_state_pub_->publish(js);
    }

    void UGVDriver::publishIMU(const rclcpp::Time &current_time)
    {
        if (!imu_) return;

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp    = current_time;
        imu_msg.header.frame_id = frame_prefix_ + "imu";

        // Orientation from InertialUnit
        const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu_);
        if (rpy)
        {
            double roll  = rpy[0];
            double pitch = rpy[1];
            double yaw   = rpy[2];

            double cy = cos(yaw * 0.5),  sy = sin(yaw * 0.5);
            double cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
            double cr = cos(roll * 0.5),  sr = sin(roll * 0.5);

            imu_msg.orientation.w = cr*cp*cy + sr*sp*sy;
            imu_msg.orientation.x = sr*cp*cy - cr*sp*sy;
            imu_msg.orientation.y = cr*sp*cy + sr*cp*sy;
            imu_msg.orientation.z = cr*cp*sy - sr*sp*cy;

            imu_msg.orientation_covariance[0] = 0.01;
            imu_msg.orientation_covariance[4] = 0.01;
            imu_msg.orientation_covariance[8] = 0.01;
        }
        else
        {
            imu_msg.orientation_covariance[0] = -1;
        }

        // Angular velocity from Gyroscope
        if (gyro_)
        {
            const double *gv = wb_gyro_get_values(gyro_);
            if (gv)
            {
                imu_msg.angular_velocity.x = gv[0];
                imu_msg.angular_velocity.y = gv[1];
                imu_msg.angular_velocity.z = gv[2];
                imu_msg.angular_velocity_covariance[0] = 0.001;
                imu_msg.angular_velocity_covariance[4] = 0.001;
                imu_msg.angular_velocity_covariance[8] = 0.001;
            }
            else { imu_msg.angular_velocity_covariance[0] = -1; }
        }
        else { imu_msg.angular_velocity_covariance[0] = -1; }

        // Linear acceleration from Accelerometer
        if (accelerometer_)
        {
            const double *av = wb_accelerometer_get_values(accelerometer_);
            if (av)
            {
                imu_msg.linear_acceleration.x = av[0];
                imu_msg.linear_acceleration.y = av[1];
                imu_msg.linear_acceleration.z = av[2];
                imu_msg.linear_acceleration_covariance[0] = 0.01;
                imu_msg.linear_acceleration_covariance[4] = 0.01;
                imu_msg.linear_acceleration_covariance[8] = 0.01;
            }
            else { imu_msg.linear_acceleration_covariance[0] = -1; }
        }
        else { imu_msg.linear_acceleration_covariance[0] = -1; }

        imu_pub_->publish(imu_msg);
    }

    void UGVDriver::publishLidar3D(const rclcpp::Time &current_time)
    {
        if (!lidar_3d_) return;

        const WbLidarPoint *point_cloud = wb_lidar_get_point_cloud(lidar_3d_);
        int num_points = wb_lidar_get_number_of_points(lidar_3d_);

        if (!point_cloud || num_points <= 0) return;

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp    = current_time;
        cloud_msg.header.frame_id = frame_prefix_ + "lidar_3d";   // match URDF link name

        cloud_msg.height       = 1;
        cloud_msg.width        = num_points;
        cloud_msg.is_bigendian = false;
        cloud_msg.is_dense     = false;

        sensor_msgs::msg::PointField fx, fy, fz, fi;
        fx.name = "x"; fx.offset =  0; fx.datatype = sensor_msgs::msg::PointField::FLOAT32; fx.count = 1;
        fy.name = "y"; fy.offset =  4; fy.datatype = sensor_msgs::msg::PointField::FLOAT32; fy.count = 1;
        fz.name = "z"; fz.offset =  8; fz.datatype = sensor_msgs::msg::PointField::FLOAT32; fz.count = 1;
        fi.name = "intensity"; fi.offset = 12; fi.datatype = sensor_msgs::msg::PointField::FLOAT32; fi.count = 1;

        cloud_msg.fields    = {fx, fy, fz, fi};
        cloud_msg.point_step = 16;
        cloud_msg.row_step   = cloud_msg.point_step * num_points;
        cloud_msg.data.resize(cloud_msg.row_step);

        for (int i = 0; i < num_points; ++i)
        {
            float *p = reinterpret_cast<float*>(&cloud_msg.data[i * cloud_msg.point_step]);
            p[0] = static_cast<float>(point_cloud[i].x);
            p[1] = static_cast<float>(point_cloud[i].y);
            p[2] = static_cast<float>(point_cloud[i].z);
            p[3] = 100.0f;
        }

        lidar_3d_pub_->publish(cloud_msg);
    }

    void UGVDriver::publishLidar2D(const rclcpp::Time &current_time)
    {
        if (!lidar_2d_) return;

        const float *range_image        = wb_lidar_get_range_image(lidar_2d_);
        int          horizontal_res     = wb_lidar_get_horizontal_resolution(lidar_2d_);
        double       fov                = wb_lidar_get_fov(lidar_2d_);
        double       max_range          = wb_lidar_get_max_range(lidar_2d_);
        double       min_range          = wb_lidar_get_min_range(lidar_2d_);

        if (!range_image || horizontal_res <= 0) return;

        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp    = current_time;
        scan_msg.header.frame_id = frame_prefix_ + "lidar_2d";   // match URDF link name

        scan_msg.angle_min      = -fov / 2.0;
        scan_msg.angle_max      =  fov / 2.0;
        scan_msg.angle_increment = fov / (horizontal_res - 1);
        scan_msg.time_increment = 0.0;
        scan_msg.scan_time      = timestep_ms_ / 1000.0;
        scan_msg.range_min      = min_range;
        scan_msg.range_max      = max_range;

        scan_msg.ranges.resize(horizontal_res);
        scan_msg.intensities.resize(horizontal_res);

        for (int i = 0; i < horizontal_res; ++i)
        {
            float range = range_image[i];
            if (std::isinf(range) || range >= max_range)
                scan_msg.ranges[i] = max_range + 1.0;
            else if (range < min_range)
                scan_msg.ranges[i] = min_range - 0.01;
            else
                scan_msg.ranges[i] = range;

            scan_msg.intensities[i] = 100.0;
        }

        lidar_2d_pub_->publish(scan_msg);
    }

    void UGVDriver::publishCamera(const rclcpp::Time &current_time)
    {
        if (!camera_rgb_) return;

        const unsigned char *image_data = wb_camera_get_image(camera_rgb_);
        int    width  = wb_camera_get_width(camera_rgb_);
        int    height = wb_camera_get_height(camera_rgb_);
        double fov    = wb_camera_get_fov(camera_rgb_);

        if (!image_data || width <= 0 || height <= 0) return;

        // Image
        sensor_msgs::msg::Image img_msg;
        img_msg.header.stamp    = current_time;
        img_msg.header.frame_id = frame_prefix_ + "camera_rgb";
        img_msg.height          = height;
        img_msg.width           = width;
        img_msg.encoding        = "bgra8";
        img_msg.is_bigendian    = false;
        img_msg.step            = width * 4;

        img_msg.data.resize(img_msg.step * height);
        std::memcpy(img_msg.data.data(), image_data, img_msg.step * height);
        camera_rgb_pub_->publish(img_msg);

        // CameraInfo
        double fx = (width / 2.0) / tan(fov / 2.0);

        sensor_msgs::msg::CameraInfo info;
        info.header = img_msg.header;
        info.height = height;
        info.width  = width;
        info.distortion_model = "plumb_bob";
        info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

        info.k = {fx, 0, width/2.0, 0, fx, height/2.0, 0, 0, 1.0};
        info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        info.p = {fx, 0, width/2.0, 0,  0, fx, height/2.0, 0,  0, 0, 1.0, 0};

        camera_rgb_info_pub_->publish(info);
    }

    void UGVDriver::publishDepthCamera(const rclcpp::Time &current_time)
    {
        if (!camera_depth_) return;

        const float *range_image = wb_range_finder_get_range_image(camera_depth_);
        int    width    = wb_range_finder_get_width(camera_depth_);
        int    height   = wb_range_finder_get_height(camera_depth_);
        double fov      = wb_range_finder_get_fov(camera_depth_);
        double max_range = wb_range_finder_get_max_range(camera_depth_);

        if (!range_image || width <= 0 || height <= 0) return;

        // Depth image (32FC1)
        sensor_msgs::msg::Image depth_msg;
        depth_msg.header.stamp    = current_time;
        depth_msg.header.frame_id = frame_prefix_ + "camera_depth";
        depth_msg.height          = height;
        depth_msg.width           = width;
        depth_msg.encoding        = "32FC1";
        depth_msg.is_bigendian    = false;
        depth_msg.step            = width * sizeof(float);

        depth_msg.data.resize(depth_msg.step * height);
        float *depth_data = reinterpret_cast<float*>(depth_msg.data.data());

        for (int i = 0; i < width * height; ++i)
        {
            float r = range_image[i];
            depth_data[i] = (std::isinf(r) || r >= max_range)
                            ? std::numeric_limits<float>::quiet_NaN()
                            : r;
        }

        camera_depth_pub_->publish(depth_msg);

        // CameraInfo
        double fx = (width / 2.0) / tan(fov / 2.0);

        sensor_msgs::msg::CameraInfo info;
        info.header = depth_msg.header;
        info.height = height;
        info.width  = width;
        info.distortion_model = "plumb_bob";
        info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

        info.k = {fx, 0, width/2.0, 0, fx, height/2.0, 0, 0, 1.0};
        info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        info.p = {fx, 0, width/2.0, 0,  0, fx, height/2.0, 0,  0, 0, 1.0, 0};

        camera_depth_info_pub_->publish(info);
    }

    void UGVDriver::step()
    {
        rclcpp::Time current_time = node_->now();

        // Motor commands
        const double v = cmd_vel_.linear.x;
        const double w = cmd_vel_.angular.z;

        const double v_left  = v - (w * track_width_ / 2.0);
        const double v_right = v + (w * track_width_ / 2.0);

        wb_motor_set_velocity(left_front_motor_,  v_left  / wheel_radius_);
        wb_motor_set_velocity(left_rear_motor_,   v_left  / wheel_radius_);
        wb_motor_set_velocity(right_front_motor_, v_right / wheel_radius_);
        wb_motor_set_velocity(right_rear_motor_,  v_right / wheel_radius_);

        // Publish all sensors
        updateOdometry(current_time);
        publishIMU(current_time);
        publishLidar3D(current_time);
        publishLidar2D(current_time);
        publishCamera(current_time);
        publishDepthCamera(current_time);
    }

    UGVDriver::~UGVDriver()
    {
    }

} // namespace ugv_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    ugv_driver::UGVDriver,
    webots_ros2_driver::PluginInterface)