/**
 * @file graph_viz_ctrl.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "graph_visual_control/graph_viz_ctrl.hpp"
#include "tf2/LinearMath/Quaternion.h"

GraphVizCtrl::GraphVizCtrl(const rclcpp::NodeOptions & options)
    : Node("graph_viz_ctrl_node", options)
{
    graph_sub_ = this->create_subscription<graph_msgs::msg::Graph>(
        "graph", 10, std::bind(&GraphVizCtrl::graphCallback, this, std::placeholders::_1));
    
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("graph_markers", 10);

    RCLCPP_INFO(this->get_logger(), "Graph Visual Control node has started.");
}

void GraphVizCtrl::graphCallback(const graph_msgs::msg::Graph::ConstSharedPtr msg)
{
    if (msg->vertices.empty()) {
        return;
    }
    auto markers = getGraphMarkers(*msg);
    markers_pub_->publish(markers);
}

visualization_msgs::msg::MarkerArray GraphVizCtrl::getGraphMarkers(const graph_msgs::msg::Graph& graph)
{
    visualization_msgs::msg::MarkerArray marker_array;
    
    for (const auto& node : graph.vertices) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "graph_nodes";
        marker.id = node.id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // CORRECTED: Access position through node.pose.position
        marker.pose.position.x = node.pose.position.x;
        marker.pose.position.y = node.pose.position.y;
        marker.pose.position.z = 0.1; // Keep a small z offset for visibility
        
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker_array.markers.push_back(marker);
    }
    return marker_array;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<GraphVizCtrl>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
