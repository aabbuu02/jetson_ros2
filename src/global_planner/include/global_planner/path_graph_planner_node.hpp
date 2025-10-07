/**
 * @file path_graph_planner_node.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "graph_msgs/msg/graph.hpp"
#include "graph_msgs/srv/get_graph_plan.hpp"
#include "global_planner/dijkstra_graph_planner.hpp"
#include "global_planner/global_path_graph.hpp"

class PathGraphPlannerNode : public rclcpp::Node
{
public:
    PathGraphPlannerNode();
private:
    void graph_callback(const graph_msgs::msg::Graph::SharedPtr msg);
    void get_graph_plan_service_callback(
        const std::shared_ptr<graph_msgs::srv::GetGraphPlan::Request> req,
        std::shared_ptr<graph_msgs::srv::GetGraphPlan::Response> res);
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_graph_plan_pub;
    rclcpp::Subscription<graph_msgs::msg::Graph>::SharedPtr m_graph_sub;
    rclcpp::Service<graph_msgs::srv::GetGraphPlan>::SharedPtr m_get_graph_plan_srv;
    
    GlobalPathGraph m_gpg;
    DijkstraGraphPlanner m_planner;
    bool m_is_graph_initialized = false;
};
