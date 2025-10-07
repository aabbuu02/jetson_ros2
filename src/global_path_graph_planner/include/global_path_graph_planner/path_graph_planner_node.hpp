#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <graph_msgs/Vertex.h>
#include <graph_msgs/Edge.h>
#include <graph_msgs/Graph.h>
#include <graph_msgs/GetGraphPlan.h>

#include <global_path_graph/global_path_graph.hpp>
#include <global_path_graph_planner/dijkstra_graph_planner.hpp>

class PathGraphPlanner
{
    public:

    PathGraphPlanner();

    private:

    ros::NodeHandle m_nh;

    //Publishers
    ros::Publisher m_graph_plan_pub;

    //Subscribers
    ros::Subscriber m_graph_sub;

    void graphCallback(const graph_msgs::Graph &graph_msg);

    //Services
    ros::ServiceServer m_get_graph_plan_srv;

    bool getGraphPlanServiceCallback(graph_msgs::GetGraphPlan::Request &req, 
                                     graph_msgs::GetGraphPlan::Response &res);

    //Graph
    bool m_is_graph_initialized = false;

    GlobalPathGraph m_gpg;
    DijkstraGraphPlanner m_planner;
    GlobalPathGraph::AdjacencySetMapPtr m_graph;
};