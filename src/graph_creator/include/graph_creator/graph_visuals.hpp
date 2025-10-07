/**
 * @file graph_visuals.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#pragma once

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "graph_data_types.hpp"

namespace graph_creator
{
    class GraphVisuals
    {
    public:
        static visualization_msgs::msg::MarkerArray createButtonMarkers(const Graph::VertexList &vertices);
        static visualization_msgs::msg::MarkerArray createVertexMarkers(const Graph::VertexList &vertices);
        static visualization_msgs::msg::MarkerArray createEdgeMarkers(const Graph::EdgeList &edges);
        static visualization_msgs::msg::MarkerArray createControlPoseMarkers(const Graph::EdgeList &edges);
        static visualization_msgs::msg::MarkerArray createOrientationMarkers(const Graph::VertexList &vertices);
        static visualization_msgs::msg::MarkerArray createDirectionMarkers(const Graph::EdgeList &edges);
    };
}