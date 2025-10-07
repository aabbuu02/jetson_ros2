/**
 * @file Ui_Interface.h
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "lift_action/ModbusCommunicator.h"
#include <memory>

class UIInterface {
public:
    UIInterface(rclcpp::Node* parent_node, std::shared_ptr<ModbusCommunicator> mcPtr);
    ~UIInterface();

private:
    void cycleResetCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void homingButtonCallback(const std_msgs::msg::Bool::SharedPtr msg);
    std::shared_ptr<ModbusCommunicator> p_modbusCommunicatorPtr;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cycleResetDataSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr homingButtonSubscriber;
};

#endif // UI_INTERFACE_H
