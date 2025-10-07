/**
 * @file graph_viz_ctrl.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef GRAPH_VIZ_CTRL_HPP
#define GRAPH_VIZ_CTRL_HPP

#include "rclcpp/rclcpp.hpp"
#include "graph_msgs/msg/graph.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <string>
#include <vector>

class GraphVizCtrl : public rclcpp::Node
{
public:
    explicit GraphVizCtrl(const rclcpp::NodeOptions & options);

private:
    void graphCallback(const graph_msgs::msg::Graph::ConstSharedPtr msg);
    visualization_msgs::msg::MarkerArray getGraphMarkers(const graph_msgs::msg::Graph& graph);

    rclcpp::Subscription<graph_msgs::msg::Graph>::SharedPtr graph_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
};

#endif // GRAPH_VIZ_CTRL_HPP
