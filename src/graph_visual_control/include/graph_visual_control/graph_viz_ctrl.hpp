#pragma once

#include <string>
#include <vector>
#include <memory>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <graph_msgs/Vertex.h>
#include <graph_msgs/Edge.h>
#include <graph_msgs/Graph.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <graph_creator/graph_curves.hpp>
#include <graph_creator/graph_visuals.hpp>
#include <graph_creator/graph_utils.hpp>
#include <graph_creator/graph_data_types.hpp>
#include <graph_control/graph_control_plugin_loader.hpp>

class GraphVizCtrl
{
    public:

    GraphVizCtrl();

    private:
    
    //Vertex and edge maps
    Graph::VerticesMap m_vertex_map;
    Graph::EdgesMap m_edge_map;

    //Curve properties
    double m_pose_spacing = 0.1;
    double m_resolution = 1.0;

    //Nodehandles
    ros::NodeHandle m_nh;

    //Subscribers
    ros::Subscriber m_graph_sub;

    //Subscriber callbacks
    void graphCallback(const graph_msgs::Graph &graph_msg);

    //Action client
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac_;

    //Graph control plugin loader
    GraphControlPluginLoader m_gcpl;
    boost::shared_ptr<graph_control::BaseGraphControl> m_graph_control_instance;

    void initialize();

    //Menu handlers

    //Vertex menu handlers
    interactive_markers::MenuHandler m_vert_menu_handler;

    void initVertexMenu();

    //Edge menu handlers
    interactive_markers::MenuHandler m_edge_menu_handler;

    //Menu callbacks
    void sendGraphNavigationGoalMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void cancelGraphNavigationGoalMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    
    void sendNavigationGoalMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void cancelNavigationGoalsMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    
    void cancelAllGoalsMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void initEdgeMenu();

    //Interactive markers for visualization and control

    interactive_markers::InteractiveMarkerServer m_graph_marker_server;

    //For vertices
    bool addInteractiveVertexMarker(const Graph::Vertex &v);

    //For edges
    bool addInteractiveEdgeMarker(Graph::Edge &e);
    void addInteractiveDirectionMarker(const std::string &direction_name, const geometry_msgs::Pose &pose, int16_t marker_type);
    void addInteractiveOrientationMarker(const std::string &orientation_name, const geometry_msgs::Pose &pose);

    //For graph based goals
    void sendGraphNavigationGoal(const std::string &vertex_name);
    void cancelGraphNavigationGoal();

    //For navigation goals
    void sendNavigationGoal(const geometry_msgs::Pose &goal_pose);
    void cancelNavigationGoal();
};