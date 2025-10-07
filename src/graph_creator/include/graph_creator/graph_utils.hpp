#pragma once

#include <vector>
#include <string>

#include <ros/ros.h>
#include <graph_msgs/Vertex.h>
#include <graph_msgs/Edge.h>
#include <graph_msgs/Property.h>
#include <graph_msgs/GraphMetaData.h>
#include <graph_msgs/Graph.h>

#include "directed_graph.hpp"
#include "graph_data_types.hpp"

namespace Graph::Utilities {

    void convertVertexToMsg(Graph::VertexPtr v_ptr, graph_msgs::Vertex &vertex_msg);
    void convertEdgeToMsg(Graph::EdgePtr e_ptr, graph_msgs::Edge &edge_msg);
    void convertFromMsgToVertex(const graph_msgs::Vertex &vertex_msg, Graph::VertexPtr v_ptr);
    void convertFromMsgToEdge(const graph_msgs::Edge &edge_msg, Graph::EdgePtr e_ptr);

    void convertFromGraphToMsg(Graph::DiGraph &di_graph, graph_msgs::Graph &graph_msg);

    void convertFromEdgeAndVertexListToMsg(const std::vector<Graph::Vertex> &vl, const std::vector<Graph::Edge> &el, graph_msgs::Graph &graph_msg);
    void convertFromMsgToEdgeAndVertexList(const graph_msgs::Graph &graph_msg, std::vector<Graph::Vertex> &vl, std::vector<Graph::Edge> &el);

    void createGraphMessage(Graph::DiGraph &di_graph, const std::string &graph_name, graph_msgs::Graph &graph_msg);
}