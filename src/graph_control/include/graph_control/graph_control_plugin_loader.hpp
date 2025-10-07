/**
 * @file graph_control_plugin_loader.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef GRAPH_CONTROL_PLUGIN_LOADER_HPP
#define GRAPH_CONTROL_PLUGIN_LOADER_HPP

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "graph_control/base_graph_control.hpp"
#include "graph_msgs/msg/graph.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>

class GraphControlPluginLoader : public rclcpp::Node
{
public:
  explicit GraphControlPluginLoader(const rclcpp::NodeOptions & options);

private:
  void graphCallback(const graph_msgs::msg::Graph::ConstSharedPtr& msg);
  void controlLoop();

  std::unique_ptr<pluginlib::ClassLoader<graph_control::BaseGraphControl>> plugin_loader_;
  std::shared_ptr<graph_control::BaseGraphControl> controller_;

  rclcpp::Subscription<graph_msgs::msg::Graph>::SharedPtr graph_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // GRAPH_CONTROL_PLUGIN_LOADER_HPP
