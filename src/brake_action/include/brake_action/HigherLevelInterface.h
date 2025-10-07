/**
 * @file HigherLevelInterface.h
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef HIGHER_LEVEL_INTERFACE_H
#define HIGHER_LEVEL_INTERFACE_H

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "brake_action/ModbusCommunicator.h"
#include <memory>
#include <string>

// This is the main ROS 2 Node class
class HigherLevelInterface : public rclcpp::Node
{
public:
    HigherLevelInterface();

private:
    void service_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    ModbusCommunicator modbus_communicator_;
};

#endif // HIGHER_LEVEL_INTERFACE_H


