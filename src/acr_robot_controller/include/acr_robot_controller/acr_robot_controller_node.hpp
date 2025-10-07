/**
 * @file acr_robot_controller_node.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef ACR_ROBOT_CONTROLLER_NODE_HPP
#define ACR_ROBOT_CONTROLLER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "anscer_msgs/msg/motor_diagnostics_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "acr_robot_controller/RoboteqDevice.h"

// Main ROS 2 Node Class that consolidates all logic
class AcrRobotControllerNode : public rclcpp::Node
{
public:
    AcrRobotControllerNode();
    ~AcrRobotControllerNode();

private:
    void control_loop();
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void estop_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // Helper classes
    std::unique_ptr<RoboteqDevice> roboteq_device_;

    // ROS 2 Publishers and Subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Publisher<anscer_msgs::msg::MotorDiagnosticsArray>::SharedPtr motor_diag_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ROS 2 Timer for the main control loop
    rclcpp::TimerBase::SharedPtr timer_;

    // Member variables
    geometry_msgs::msg::Twist last_cmd_vel_;
    rclcpp::Time last_cmd_vel_time_;
    bool estop_active_ = false;

    // Parameters
    std::string roboteq_port_;
    double wheel_base_;
    double wheel_radius_;
    int encoder_cpr_;
    double cmd_vel_timeout_;

    // Odometry state
    double x_pos_ = 0.0;
    double y_pos_ = 0.0;
    double theta_ = 0.0;
    rclcpp::Time last_odom_time_;
    long last_left_encoder_ = 0;
    long last_right_encoder_ = 0;
};

#endif // ACR_ROBOT_CONTROLLER_NODE_HPP


