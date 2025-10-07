/**
 * @file power_control.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef POWER_CONTROL_HPP
#define POWER_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <vector>
#include <string>

// The node is now a class inheriting from rclcpp::Node
class PowerControl : public rclcpp::Node
{
public:
    PowerControl();

private:
    // ROS 2 Service Callbacks
    void poweroff_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void reboot_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // ROS 2 Parameter Callback (replaces dynamic_reconfigure)
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters);

    // Helper functions
    bool call_slave_services(const std::string& service_type);
    void execute_system_command(const std::string& command_type);

    // Member Variables
    bool is_master_system_;
    bool enable_system_pwr_ctrl_;
    bool enable_slave_pwr_ctrl_;
    std::string system_name_;
    std::vector<std::string> slave_system_names_;

    // ROS 2 Services and Clients
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr poweroff_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_srv_;
    std::vector<rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> slave_poweroff_clients_;
    std::vector<rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> slave_reboot_clients_;

    // ROS 2 Parameter callback handle
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

#endif // POWER_CONTROL_HPP

