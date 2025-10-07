/**
 * @file path_graph_planner_node.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "global_planner/path_graph_planner_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

PathGraphPlannerNode::PathGraphPlannerNode() : Node("global_planner_node")
{
    m_graph_plan_pub = this->create_publisher<nav_msgs::msg::Path>("graph_plan", 10);
    m_graph_sub = this->create_subscription<graph_msgs::msg::Graph>( "graph", 10, std::bind(&PathGraphPlannerNode::graph_callback, this, _1));
    m_get_graph_plan_srv = this->create_service<graph_msgs::srv::GetGraphPlan>( "make_graph_plan", std::bind(&PathGraphPlannerNode::get_graph_plan_service_callback, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Global Planner Node has been started.");
}

void PathGraphPlannerNode::graph_callback(const graph_msgs::msg::Graph::SharedPtr msg)
{
    m_gpg.getGraphFromMsg(*msg, this->get_logger());
    if (!m_planner.setGraph(m_gpg)) {
        RCLCPP_WARN(this->get_logger(), "Setting graph in planner failed");
        m_is_graph_initialized = false;
        return;
    }
    m_is_graph_initialized = true;
    RCLCPP_INFO(this->get_logger(), "Path graph planner received and set a valid graph.");
}

void PathGraphPlannerNode::get_graph_plan_service_callback(
    const std::shared_ptr<graph_msgs::srv::GetGraphPlan::Request> req,
    std::shared_ptr<graph_msgs::srv::GetGraphPlan::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "Get plan service called for %u to %u", req->start_node_id, req->goal_node_id);
    if (!m_is_graph_initialized) {
        return;
    }

    std::vector<geometry_msgs::msg::PoseStamped> retrieved_plan;
    if (!m_planner.getPlanFromGraph(req->start_node_id, req->goal_node_id, retrieved_plan, this->get_clock(), false, false)) {
        return;
    }
    
    std::vector<uint32_t> retrieved_vertices_in_plan;
    m_planner.getPointsInPlan(retrieved_vertices_in_plan);
    for(const auto& v_id : retrieved_vertices_in_plan) {
        auto point_data = m_gpg.getPointData(v_id);
        if (point_data) {
            graph_msgs::msg::Node node;
            node.id = v_id;
            node.pose = point_data->pose;
            res->path.push_back(node);
        }
    }
    
    // Publish the plan
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.poses = retrieved_plan;
    m_graph_plan_pub->publish(path_msg);
    
    RCLCPP_INFO(this->get_logger(), "Plan published with %zu poses", retrieved_plan.size());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathGraphPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
