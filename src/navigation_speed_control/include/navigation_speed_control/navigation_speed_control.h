/**
 * @file navigation_speed_control.h
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef NAVIGATION_SPEED_CONTROL_H
#define NAVIGATION_SPEED_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

class NavigationSpeedControl : public rclcpp::Node
{
public:
    explicit NavigationSpeedControl(const rclcpp::NodeOptions & options);

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_vel_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_vel_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_path_sub;

    double m_max_linear_vel;
    double m_min_linear_vel;
    double m_max_angular_vel;
    double m_min_angular_vel;
    double m_final_dist;
};

#endif // NAVIGATION_SPEED_CONTROL_H
