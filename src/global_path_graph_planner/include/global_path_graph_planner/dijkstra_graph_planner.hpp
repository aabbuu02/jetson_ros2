#pragma once

#include <vector>
#include <set>
#include <unordered_map>
#include <queue>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <graph_msgs/Vertex.h>
#include <graph_msgs/Edge.h>
#include <graph_msgs/Graph.h>

#include <graph_creator/graph_curves.hpp>
#include <global_path_graph/global_path_graph.hpp>

class DijkstraGraphPlanner
{
    public:

    DijkstraGraphPlanner();

    bool setGraph(GlobalPathGraph &gpg);

    bool dijkstraPlanner(const uint32_t &src_point_id, const uint32_t &dst_point_id, 
                         std::vector<uint32_t> &points_in_path);

    bool getPointPlanFromGraph(const uint32_t &src_point_id, const uint32_t &dst_point_id, 
                               std::vector<uint32_t> &points_in_plan);

    bool getPlanFromGraph(const uint32_t &src_point_id, const uint32_t &dst_point_id, 
                          std::vector<geometry_msgs::PoseStamped> &plan, bool ignore_goal_orientation=false, 
                          bool ignore_path_orientation=true);

    //Given a vector of points get a plan recreated from edge information
    bool getPlanFromPoints(const std::vector<uint32_t> &point_plan, std::vector<geometry_msgs::PoseStamped> &plan,
                           bool ignore_goal_orientation=false, bool ignore_path_orientation=true); 

    void getPointsInPlan(std::set<uint32_t> &points_in_plan);
    void getPointsInPlan(std::vector<uint32_t> &points_in_plan);

    double getPlanDistance();

    void printPointsInPlan();

    //Planner and plan settings
    void enableCostFactor(bool enable);
    bool isCostFactorEnabled();

    private:

    struct PointCost
    {
        uint32_t point_id;
        double cost;

        PointCost(uint32_t pnt_id, double pnt_cost):
            point_id(pnt_id), cost(pnt_cost)
        {}
    };

    struct PointCostCompare
    {
        bool operator()(const PointCost &lhs, const PointCost &rhs)
        {
            return lhs.cost > rhs.cost; //To get the minimum distance queue
        }
    };

    //Line and curve properties variable
    double m_pose_spacing = 0.1;
    double m_resolution = 1.0;

    //Planner and plan settings
    bool m_enable_cost_factor = true;

    //For Dijkstra shortest path search
    std::unordered_map<uint32_t, double> m_point_distance;
    std::unordered_map<uint32_t, uint32_t> m_previous_point; //This is used for backtracking
    double m_plan_distance = 0.0f;

    //Global path graph
    GlobalPathGraph m_gpg;
    GlobalPathGraph::AdjacencySetMapPtr m_graph;
    
    std::vector<uint32_t> m_points_in_plan_v;
};