/**
 * @file UI_Interface.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "lift_action/Ui_Interface.h"
#include <unistd.h>

UIInterface::UIInterface(rclcpp::Node* parent_node, std::shared_ptr<ModbusCommunicator> mcPtr)
    : p_modbusCommunicatorPtr(mcPtr) {
    cycleResetDataSubscriber = parent_node->create_subscription<std_msgs::msg::Bool>(
        "/lifter/cycle_reset", 1, std::bind(&UIInterface::cycleResetCallback, this, std::placeholders::_1));
    homingButtonSubscriber = parent_node->create_subscription<std_msgs::msg::Bool>(
        "/lifter/homing_button", 1, std::bind(&UIInterface::homingButtonCallback, this, std::placeholders::_1));
}

UIInterface::~UIInterface() {}

void UIInterface::homingButtonCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    p_modbusCommunicatorPtr->writeDataToRegister(19, msg->data ? 1 : 0);
}

void UIInterface::cycleResetCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        p_modbusCommunicatorPtr->writeDataToRegister(47, 1);
        rclcpp::sleep_for(std::chrono::seconds(1));
        p_modbusCommunicatorPtr->writeDataToRegister(47, 0);
    }
}
