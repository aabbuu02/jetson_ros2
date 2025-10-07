/**
 * @file RosClass.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "lower_level_controller/RosClass.h"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

RosClass::RosClass(const rclcpp::NodeOptions & options) : Node("lower_level_controller_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Lower Level Controller...");

    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<double>("wheel_radius", 0.085);
    this->declare_parameter<double>("wheel_separation", 0.42);
    this->declare_parameter<int>("encoder_cpr", 4096);
    this->declare_parameter<std::string>("modbus_ip", "192.168.1.10");
    this->declare_parameter<int>("modbus_port", 502);

    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    double wheel_radius = this->get_parameter("wheel_radius").as_double();
    double wheel_separation = this->get_parameter("wheel_separation").as_double();
    int encoder_cpr = this->get_parameter("encoder_cpr").as_int();
    std::string modbus_ip = this->get_parameter("modbus_ip").as_string();
    int modbus_port = this->get_parameter("modbus_port").as_int();

    try {
        modbusController_ = std::make_unique<ModbusController>(modbus_ip, modbus_port, this->get_logger());
    } catch (const std::runtime_error &e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize. Shutting down. Error: %s", e.what());
        rclcpp::shutdown();
        return;
    }
    
    lowerLevelComputations_ = std::make_unique<LowerLevelComputations>(wheel_radius, wheel_separation, encoder_cpr);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&RosClass::cmdVelCallback, this, std::placeholders::_1));
    
    if (publish_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    timer_ = this->create_wall_timer(20ms, std::bind(&RosClass::update_loop, this));
    RCLCPP_INFO(this->get_logger(), "Lower Level Controller initialized successfully.");
}

void RosClass::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    auto [left_rpm, right_rpm] = lowerLevelComputations_->twistToRPM(msg->linear.x, msg->angular.z);
    modbusController_->writeData(10, static_cast<int>(left_rpm)); // Assuming register 10 is left RPM
    modbusController_->writeData(11, static_cast<int>(right_rpm)); // Assuming register 11 is right RPM
}

void RosClass::update_loop()
{
    long left_encoder = modbusController_->readData(20); // Assuming register 20 is left encoder counts
    long right_encoder = modbusController_->readData(21); // Assuming register 21 is right encoder counts

    auto [x, y, theta, vx, vth] = lowerLevelComputations_->updateOdometry(left_encoder, right_encoder, this->get_clock()->now());

    auto now = this->get_clock()->now();
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.angular.z = vth;

    odom_pub_->publish(odom_msg);

    if (publish_tf_) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = odom_frame_;
        tf_msg.child_frame_id = base_frame_;
        tf_msg.transform.translation.x = x;
        tf_msg.transform.translation.y = y;
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);
    }
}
