/**
 * @file navigation_speed_control.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "navigation_speed_control/navigation_speed_control.h"
#include <cmath>

NavigationSpeedControl::NavigationSpeedControl(const rclcpp::NodeOptions & options)
    : Node("navigation_speed_control_node", options)
{
    this->declare_parameter<double>("max_linear_vel", 0.5);
    this->declare_parameter<double>("min_linear_vel", 0.05);
    this->declare_parameter<double>("max_angular_vel", 0.8);
    this->declare_parameter<double>("min_angular_vel", 0.1);
    this->declare_parameter<double>("final_dist", 0.3);

    this->get_parameter("max_linear_vel", m_max_linear_vel);
    this->get_parameter("min_linear_vel", m_min_linear_vel);
    this->get_parameter("max_angular_vel", m_max_angular_vel);
    this->get_parameter("min_angular_vel", m_min_angular_vel);
    this->get_parameter("final_dist", m_final_dist);

    m_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);
    m_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_in", 10, std::bind(&NavigationSpeedControl::cmdVelCallback, this, std::placeholders::_1));
    m_path_sub = this->create_subscription<nav_msgs::msg::Path>(
        "plan", 10, std::bind(&NavigationSpeedControl::pathCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Navigation Speed Control node has started.");
}

void NavigationSpeedControl::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.size() > 1) {
        auto& last_pose = msg->poses.back().pose;
        auto& second_last_pose = msg->poses[msg->poses.size() - 2].pose;
        m_final_dist = std::hypot(last_pose.position.x - second_last_pose.position.x, 
                                  last_pose.position.y - second_last_pose.position.y);
    }
}

void NavigationSpeedControl::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    geometry_msgs::msg::Twist final_twist = *msg;

    if (final_twist.linear.x > 0 && final_twist.linear.x < m_min_linear_vel) {
        final_twist.linear.x = m_min_linear_vel;
    } else if (final_twist.linear.x < 0 && final_twist.linear.x > -m_min_linear_vel) {
        final_twist.linear.x = -m_min_linear_vel;
    }

    if (final_twist.angular.z > 0 && final_twist.angular.z < m_min_angular_vel) {
        final_twist.angular.z = m_min_angular_vel;
    } else if (final_twist.angular.z < 0 && final_twist.angular.z > -m_min_angular_vel) {
        final_twist.angular.z = -m_min_angular_vel;
    }

    m_vel_pub->publish(final_twist);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<NavigationSpeedControl>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
