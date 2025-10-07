/**
 * @file dijkstra_graph_planner.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "global_planner/global_path_graph.hpp"
#include <vector>
#include <queue>
#include <unordered_map>

class DijkstraGraphPlanner {
public:
    DijkstraGraphPlanner();
    bool setGraph(GlobalPathGraph &gpg);
    bool getPlanFromGraph(const uint32_t &src_point_id, const uint32_t &dst_point_id, std::vector<geometry_msgs::msg::PoseStamped> &plan, rclcpp::Clock::SharedPtr clock, bool ignore_goal = false, bool ignore_path = false);
    void getPointsInPlan(std::vector<uint32_t> &points_in_plan);
    double getPlanDistance();

private:
    struct PointCost {
        uint32_t point_id; 
        double cost;
        PointCost(uint32_t id, double c) : point_id(id), cost(c) {}
        bool operator>(const PointCost& other) const { return cost > other.cost; }
    };
    
    bool dijkstraPlanner(const uint32_t &src, const uint32_t &dst, std::vector<uint32_t> &path);
    bool getPlanFromPoints(const std::vector<uint32_t> &point_plan, std::vector<geometry_msgs::msg::PoseStamped> &plan, rclcpp::Clock::SharedPtr clock, bool ignore_goal_orientation, bool ignore_path_orientation);
    
    GlobalPathGraph m_gpg;
    std::vector<uint32_t> m_points_in_plan_v;
    double m_plan_distance = 0.0;
};
