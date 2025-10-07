/**
 * @file global_path_graph.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "global_planner/global_path_graph.hpp"
#include <sstream>

GlobalPathGraph::GlobalPathGraph() {}

void GlobalPathGraph::getGraphFromMsg(const graph_msgs::msg::Graph &graph_msg, rclcpp::Logger logger) {
    clearGraph();
    
    // Add all nodes (vertices) to the graph
    for (const auto& node : graph_msg.vertices) {
        auto pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
        pose_stamped->header.frame_id = "map";
        pose_stamped->pose = node.pose;
        
        m_point_data[node.id] = pose_stamped;
        m_valid_points.insert(node.id);
        m_neighbors[node.id] = std::vector<std::pair<uint32_t, double>>();
        
        RCLCPP_INFO(logger, "Added node with ID %u", node.id);
    }
    
    // Add all edges to create neighbor relationships
    for (const auto& edge : graph_msg.edges) {
        if (m_valid_points.count(edge.from_id) && m_valid_points.count(edge.to_id)) {
            m_neighbors[edge.from_id].push_back({edge.to_id, edge.weight});
            RCLCPP_INFO(logger, "Added edge from %u to %u with weight %f", edge.from_id, edge.to_id, edge.weight);
        }
    }
    
    RCLCPP_INFO(logger, "Graph loaded with %zu nodes and %zu edges", m_point_data.size(), graph_msg.edges.size());
}

std::shared_ptr<geometry_msgs::msg::PoseStamped> GlobalPathGraph::getPointData(const uint32_t &point_id) {
    auto it = m_point_data.find(point_id);
    return (it != m_point_data.end()) ? it->second : nullptr;
}

std::vector<std::pair<uint32_t, double>> GlobalPathGraph::getNeighbors(const uint32_t &point_id) {
    auto it = m_neighbors.find(point_id);
    return (it != m_neighbors.end()) ? it->second : std::vector<std::pair<uint32_t, double>>();
}

bool GlobalPathGraph::isPointValid(const uint32_t &point_id) {
    return m_valid_points.count(point_id) > 0;
}

void GlobalPathGraph::clearGraph() {
    m_point_data.clear();
    m_neighbors.clear();
    m_valid_points.clear();
}
