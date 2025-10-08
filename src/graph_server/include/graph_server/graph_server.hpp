#pragma once

#include <string>

#include <ros/ros.h>
#include <graph_msgs/Vertex.h>
#include <graph_msgs/Edge.h>
#include <graph_msgs/Graph.h>
#include <graph_msgs/GetGraph.h>
#include <graph_msgs/LoadGraph.h>

#include <graph_creator/graph_utils.hpp>
#include <graph_creator/graph_file_utils.hpp>
#include <graph_creator/graph_loader.hpp>
#include <graph_creator/graph_data_types.hpp>

class GraphServer
{
    public:

    GraphServer();
    bool loadGraphFromFileAndPublish(const std::string &filename);

    private:

    ros::NodeHandle m_nh;

    bool m_is_initialized = false;

    //Message
    graph_msgs::Graph m_graph_msg;

    //Publisher
    ros::Publisher m_graph_pub;

    void publishGraph(const Graph::VertexList &vl, const Graph::EdgeList &el);

    //Services and service callbacks
    ros::ServiceServer m_load_graph_srv;
    ros::ServiceServer m_get_graph_srv;

    bool loadGraphServiceCallback(graph_msgs::LoadGraph::Request &req, graph_msgs::LoadGraph::Response &res);
    bool getGraphServiceCallback(graph_msgs::GetGraph::Request &req, graph_msgs::GetGraph::Response &res);
};


