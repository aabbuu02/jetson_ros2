/**
 * @file HigherLevelInterface.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "brake_action/HigherLevelInterface.h"

// This class now handles all ROS 2 logic
HigherLevelInterface::HigherLevelInterface() : Node("brake_action_node")
{
    // Declare ROS 2 parameters with default values
    this->declare_parameter<std::string>("ip_address", "127.0.0.1");
    this->declare_parameter<int>("port", 502);

    // Get parameters
    std::string ip = this->get_parameter("ip_address").as_string();
    int port = this->get_parameter("port").as_int();

    RCLCPP_INFO(this->get_logger(), "Attempting to connect to Modbus server at %s:%d", ip.c_str(), port);

    // Initialize the Modbus communicator with the parameters
    if (modbus_communicator_.initiateConnection(ip, port) == 0) {
        RCLCPP_INFO(this->get_logger(), "Modbus connection successful.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to Modbus server.");
    }

    // Create the ROS 2 service server
    service_ = this->create_service<std_srvs::srv::SetBool>(
        "brake_service",
        std::bind(&HigherLevelInterface::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Brake Action service is ready.");
}

void HigherLevelInterface::service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    int register_value = request->data ? 1 : 0;
    int write_status = modbus_communicator_.writeData(40001, register_value);

    if (write_status == 0) {
        response->success = true;
        response->message = "Brake state set successfully.";
        RCLCPP_INFO(this->get_logger(), "Brake state changed to: %d", register_value);
    } else {
        response->success = false;
        response->message = "Failed to write to brake register.";
        RCLCPP_ERROR(this->get_logger(), "Failed to set brake state.");
    }
}
