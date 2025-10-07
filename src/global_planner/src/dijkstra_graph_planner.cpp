/**
 * @file dijkstra_graph_planner.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "global_planner/dijkstra_graph_planner.hpp"
#include <algorithm>
#include <limits>
#include <sstream>

DijkstraGraphPlanner::DijkstraGraphPlanner() {}

bool DijkstraGraphPlanner::setGraph(GlobalPathGraph &gpg) {
    m_gpg = gpg;
    return true;
}

bool DijkstraGraphPlanner::getPlanFromGraph(const uint32_t &src, const uint32_t &dst, std::vector<geometry_msgs::msg::PoseStamped> &plan, rclcpp::Clock::SharedPtr clock, bool ignore_goal, bool ignore_path) {
    plan.clear();
    m_points_in_plan_v.clear();
    if (!dijkstraPlanner(src, dst, m_points_in_plan_v)) return false;
    if (!getPlanFromPoints(m_points_in_plan_v, plan, clock, ignore_goal, ignore_path)) return false;
    return true;
}

bool DijkstraGraphPlanner::getPlanFromPoints(const std::vector<uint32_t> &point_plan, std::vector<geometry_msgs::msg::PoseStamped> &plan, rclcpp::Clock::SharedPtr clock, bool ignore_goal_orientation, bool ignore_path_orientation) {
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    m_plan_distance = 0.0;
    
    for (size_t i = 0; i < point_plan.size(); ++i) {
        auto point_data = m_gpg.getPointData(point_plan[i]);
        if (!point_data) {
            return false;
        }
        
        pose_stamped_msg = *point_data;
        pose_stamped_msg.header.stamp = clock->now();
        plan.push_back(pose_stamped_msg);
        
        // Calculate distance to next point
        if (i < point_plan.size() - 1) {
            auto next_point_data = m_gpg.getPointData(point_plan[i + 1]);
            if (next_point_data) {
                double dx = next_point_data->pose.position.x - point_data->pose.position.x;
                double dy = next_point_data->pose.position.y - point_data->pose.position.y;
                m_plan_distance += sqrt(dx*dx + dy*dy);
            }
        }
    }
    
    return true;
}

bool DijkstraGraphPlanner::dijkstraPlanner(const uint32_t &src, const uint32_t &dst, std::vector<uint32_t> &path) {
    if (!m_gpg.isPointValid(src) || !m_gpg.isPointValid(dst)) {
        return false;
    }
    
    std::unordered_map<uint32_t, double> distances;
    std::unordered_map<uint32_t, uint32_t> previous;
    std::priority_queue<PointCost, std::vector<PointCost>, std::greater<PointCost>> pq;
    
    // Initialize distances
    distances[src] = 0.0;
    pq.push(PointCost(src, 0.0));
    
    while (!pq.empty()) {
        PointCost current = pq.top();
        pq.pop();
        
        if (current.point_id == dst) {
            // Reconstruct path
            uint32_t node = dst;
            while (node != src) {
                path.push_back(node);
                node = previous[node];
            }
            path.push_back(src);
            std::reverse(path.begin(), path.end());
            return true;
        }
        
        auto neighbors = m_gpg.getNeighbors(current.point_id);
        for (const auto& neighbor : neighbors) {
            uint32_t neighbor_id = neighbor.first;
            double edge_weight = neighbor.second;
            double new_distance = current.cost + edge_weight;
            
            if (distances.find(neighbor_id) == distances.end() || new_distance < distances[neighbor_id]) {
                distances[neighbor_id] = new_distance;
                previous[neighbor_id] = current.point_id;
                pq.push(PointCost(neighbor_id, new_distance));
            }
        }
    }
    
    return false; // No path found
}

void DijkstraGraphPlanner::getPointsInPlan(std::vector<uint32_t> &points_in_plan) {
    points_in_plan = m_points_in_plan_v;
}

double DijkstraGraphPlanner::getPlanDistance() {
    return m_plan_distance;
}