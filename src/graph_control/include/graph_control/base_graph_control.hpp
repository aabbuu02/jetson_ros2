/**
 * @file base_graph_control.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef BASE_GRAPH_CONTROL_HPP
#define BASE_GRAPH_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "graph_msgs/msg/graph.hpp"

namespace graph_control
{
  class BaseGraphControl
  {
  public:
    virtual ~BaseGraphControl() {}
    virtual void initialize(rclcpp::Node* node, const graph_msgs::msg::Graph::ConstSharedPtr& graph) = 0;
    virtual geometry_msgs::msg::Twist getCommand() = 0;
  };
}
#endif // BASE_GRAPH_CONTROL_HPP
