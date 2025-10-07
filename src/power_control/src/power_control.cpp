/**
 * @file power_control.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "power_control.hpp"
#include <unistd.h>
#include <sys/reboot.h>
#include <chrono>

using namespace std::chrono_literals;

PowerControl::PowerControl() : Node("power_control_node")
{
    // --- Declare and get parameters ---
    this->declare_parameter<std::string>("system_name", "master_system");
    this->declare_parameter<bool>("is_system_master", false);
    this->declare_parameter<std::vector<std::string>>("slave_system_names", std::vector<std::string>());
    this->declare_parameter<bool>("enable_system_power_control", false);
    this->declare_parameter<bool>("enable_slave_power_control", false);

    system_name_ = this->get_parameter("system_name").as_string();
    is_master_system_ = this->get_parameter("is_system_master").as_bool();
    slave_system_names_ = this->get_parameter("slave_system_names").as_string_array();
    enable_system_pwr_ctrl_ = this->get_parameter("enable_system_power_control").as_bool();
    enable_slave_pwr_ctrl_ = this->get_parameter("enable_slave_power_control").as_bool();

    RCLCPP_INFO(this->get_logger(), "System name: %s", system_name_.c_str());
    if (is_master_system_) {
        RCLCPP_WARN(this->get_logger(), "Current system is MASTER for power control.");
        for (const auto& name : slave_system_names_) {
            RCLCPP_INFO(this->get_logger(), " - Configured slave: %s", name.c_str());
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Current system is SLAVE for power control.");
    }

    // --- Create Services ---
    poweroff_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/poweroff", std::bind(&PowerControl::poweroff_callback, this, std::placeholders::_1, std::placeholders::_2));
    reboot_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/reboot", std::bind(&PowerControl::reboot_callback, this, std::placeholders::_1, std::placeholders::_2));

    // --- Create Clients if Master ---
    if (is_master_system_) {
        for (const auto& slave_name : slave_system_names_) {
            slave_poweroff_clients_.push_back(
                this->create_client<std_srvs::srv::Trigger>("/" + slave_name + "/power_control/poweroff"));
            slave_reboot_clients_.push_back(
                this->create_client<std_srvs::srv::Trigger>("/" + slave_name + "/power_control/reboot"));
        }
    }

    // --- Set up Parameter Callback (replaces dynamic_reconfigure) ---
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&PowerControl::parameters_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Power Control Node has been started.");
}

// Service callback for poweroff
void PowerControl::poweroff_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_WARN(this->get_logger(), "Poweroff service called for %s", system_name_.c_str());
    execute_system_command("poweroff");
    response->success = true;
    response->message = "Powering down " + system_name_;
}

// Service callback for reboot
void PowerControl::reboot_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_WARN(this->get_logger(), "Reboot service called for %s", system_name_.c_str());
    execute_system_command("reboot");
    response->success = true;
    response->message = "Rebooting " + system_name_;
}

// Main logic for executing commands
void PowerControl::execute_system_command(const std::string& command_type)
{
    if (is_master_system_) {
        if (!call_slave_services(command_type)) {
            RCLCPP_ERROR(this->get_logger(), "%s of slave systems failed. Aborting master %s.", command_type.c_str(), command_type.c_str());
            return;
        }
    }

    if (enable_system_pwr_ctrl_) {
        RCLCPP_WARN(this->get_logger(), "Executing %s on system %s NOW.", command_type.c_str(), system_name_.c_str());
        sync(); // Commit changes to disk
        if (command_type == "poweroff") {
            system("sudo poweroff");
        } else if (command_type == "reboot") {
            system("sudo reboot");
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "System power control is disabled. Cannot %s %s.", command_type.c_str(), system_name_.c_str());
    }
}

// Helper to call all slave services
bool PowerControl::call_slave_services(const std::string& service_type)
{
    if (!enable_slave_pwr_ctrl_) {
        RCLCPP_WARN(this->get_logger(), "Slave power control is disabled. Skipping slave commands.");
        return true;
    }

    auto& clients = (service_type == "poweroff") ? slave_poweroff_clients_ : slave_reboot_clients_;
    RCLCPP_INFO(this->get_logger(), "Calling %s service on %zu slaves...", service_type.c_str(), clients.size());

    for (auto& client : clients) {
        if (!client->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Service %s not available.", client->get_service_name());
            return false;
        }
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);
        
        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service %s", client->get_service_name());
            return false;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "All slave services called successfully.");
    return true;
}

// Parameter callback (replaces dynamic_reconfigure)
rcl_interfaces::msg::SetParametersResult PowerControl::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : parameters) {
        if (param.get_name() == "enable_system_power_control") {
            enable_system_pwr_ctrl_ = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "Parameter 'enable_system_power_control' updated to: %s", enable_system_pwr_ctrl_ ? "true" : "false");
        }
        if (param.get_name() == "enable_slave_power_control") {
            enable_slave_pwr_ctrl_ = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "Parameter 'enable_slave_power_control' updated to: %s", enable_slave_pwr_ctrl_ ? "true" : "false");
        }
    }
    return result;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PowerControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

