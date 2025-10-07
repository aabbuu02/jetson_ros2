/**
 * @file graph_visuals.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "graph_creator/graph_visuals.hpp"
#include <rclcpp/rclcpp.hpp>

namespace graph_creator
{
    visualization_msgs::msg::MarkerArray GraphVisuals::createButtonMarkers(const Graph::VertexList &vertices)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (size_t i = 0; i < vertices.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = rclcpp::Clock().now();
            marker.ns = "graph_buttons";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose = vertices[i].vertex_pose;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            
            marker_array.markers.push_back(marker);
        }
        
        return marker_array;
    }

    visualization_msgs::msg::MarkerArray GraphVisuals::createVertexMarkers(const Graph::VertexList &vertices)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (size_t i = 0; i < vertices.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = rclcpp::Clock().now();
            marker.ns = "graph_vertices";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose = vertices[i].vertex_pose;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.1;
            
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            
            marker_array.markers.push_back(marker);
        }
        
        return marker_array;
    }

    visualization_msgs::msg::MarkerArray GraphVisuals::createEdgeMarkers(const Graph::EdgeList &edges)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (size_t i = 0; i < edges.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = rclcpp::Clock().now();
            marker.ns = "graph_edges";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            
            marker_array.markers.push_back(marker);
        }
        
        return marker_array;
    }

    visualization_msgs::msg::MarkerArray GraphVisuals::createControlPoseMarkers(const Graph::EdgeList &edges)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        int marker_id = 0;
        for (const auto& edge : edges) {
            for (const auto& pose : edge.control_poses) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = rclcpp::Clock().now();
                marker.ns = "control_poses";
                marker.id = marker_id++;
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                
                marker.pose = pose;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                
                marker_array.markers.push_back(marker);
            }
        }
        
        return marker_array;
    }

    visualization_msgs::msg::MarkerArray GraphVisuals::createOrientationMarkers(const Graph::VertexList &vertices)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (size_t i = 0; i < vertices.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = rclcpp::Clock().now();
            marker.ns = "orientations";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose = vertices[i].vertex_pose;
            marker.scale.x = 0.5;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            
            marker_array.markers.push_back(marker);
        }
        
        return marker_array;
    }

    visualization_msgs::msg::MarkerArray GraphVisuals::createDirectionMarkers(const Graph::EdgeList &edges)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (size_t i = 0; i < edges.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = rclcpp::Clock().now();
            marker.ns = "directions";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.scale.x = 0.3;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
            marker.color.a = 1.0;
            
            marker_array.markers.push_back(marker);
        }
        
        return marker_array;
    }
}