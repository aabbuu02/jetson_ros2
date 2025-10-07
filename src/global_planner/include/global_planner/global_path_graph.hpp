/**
 * @file global_path_graph.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "graph_msgs/msg/node.hpp"
#include "graph_msgs/msg/edge.hpp"
#include "graph_msgs/msg/graph.hpp"
#include <memory>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <utility>

class GlobalPathGraph {
public:
    GlobalPathGraph();
    void getGraphFromMsg(const graph_msgs::msg::Graph &graph_msg, rclcpp::Logger logger);
    std::shared_ptr<geometry_msgs::msg::PoseStamped> getPointData(const uint32_t &point_id);
    std::vector<std::pair<uint32_t, double>> getNeighbors(const uint32_t &point_id);
    bool isPointValid(const uint32_t &point_id);
    void clearGraph();

private:
    std::unordered_map<uint32_t, std::shared_ptr<geometry_msgs::msg::PoseStamped>> m_point_data;
    std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, double>>> m_neighbors;
    std::unordered_set<uint32_t> m_valid_points;
};
