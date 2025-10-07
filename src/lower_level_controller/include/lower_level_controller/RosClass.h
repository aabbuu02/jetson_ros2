/**
 * @file RosClass.h
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef ROS_CLASS_H
#define ROS_CLASS_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "lower_level_controller/LowerLevelComputations.h"
#include "lower_level_controller/ModbusController.h"
#include <memory>

class RosClass : public rclcpp::Node
{
public:
    explicit RosClass(const rclcpp::NodeOptions & options);

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void update_loop();

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<ModbusController> modbusController_;
    std::unique_ptr<LowerLevelComputations> lowerLevelComputations_;

    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_;
};
#endif // ROS_CLASS_H
