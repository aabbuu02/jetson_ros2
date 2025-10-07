/**
 * @file graph_control_plugin_loader.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "graph_control/graph_control_plugin_loader.hpp"

using namespace std::chrono_literals;

GraphControlPluginLoader::GraphControlPluginLoader(const rclcpp::NodeOptions & options)
  : Node("graph_control_loader_node", options), 
    plugin_loader_(std::make_unique<pluginlib::ClassLoader<graph_control::BaseGraphControl>>("graph_control", "graph_control::BaseGraphControl"))
{
    this->declare_parameter<std::string>("plugin_name", "pure_pursuit_controller/PurePursuitController");
    std::string plugin_name = this->get_parameter("plugin_name").as_string();

    try {
        controller_ = plugin_loader_->createSharedInstance(plugin_name);
        RCLCPP_INFO(this->get_logger(), "Successfully loaded plugin: %s", plugin_name.c_str());
    } catch (pluginlib::PluginlibException& ex) {
        RCLCPP_FATAL(this->get_logger(), "Failed to load plugin: %s. Error: %s", plugin_name.c_str(), ex.what());
        rclcpp::shutdown();
        return;
    }

    graph_sub_ = this->create_subscription<graph_msgs::msg::Graph>(
        "graph", 10, std::bind(&GraphControlPluginLoader::graphCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(50ms, std::bind(&GraphControlPluginLoader::controlLoop, this));
}

void GraphControlPluginLoader::graphCallback(const graph_msgs::msg::Graph::ConstSharedPtr& msg)
{
    RCLCPP_INFO(this->get_logger(), "Received new graph. Initializing controller plugin.");
    controller_->initialize(this, msg);
}

void GraphControlPluginLoader::controlLoop()
{
    if (controller_) {
        auto cmd = controller_->getCommand();
        cmd_vel_pub_->publish(cmd);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<GraphControlPluginLoader>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
