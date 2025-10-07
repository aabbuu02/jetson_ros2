/**
 * @file graph_utils.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#pragma once

#include <string>
#include <vector>
#include <graph_msgs/msg/graph.hpp>
#include <graph_msgs/msg/node.hpp>
#include <graph_msgs/msg/edge.hpp>
#include "graph_data_types.hpp"

namespace graph_creator
{
    class GraphUtils
    {
    public:
        static void convertVertexToMsg(Graph::VertexPtr v_ptr, graph_msgs::msg::Node &vertex_msg);
        static void convertEdgeToMsg(Graph::EdgePtr e_ptr, graph_msgs::msg::Edge &edge_msg);
        static void convertFromMsgToVertex(const graph_msgs::msg::Node &vertex_msg, Graph::VertexPtr v_ptr);
        static void convertFromMsgToEdge(const graph_msgs::msg::Edge &edge_msg, Graph::EdgePtr e_ptr);
        static void convertFromGraphToMsg(Graph::DirectedGraph &di_graph, graph_msgs::msg::Graph &graph_msg);
        static void convertFromEdgeAndVertexListToMsg(const std::vector<Graph::Vertex> &vl, const std::vector<Graph::Edge> &el, graph_msgs::msg::Graph &graph_msg);
        static void convertFromMsgToEdgeAndVertexList(const graph_msgs::msg::Graph &graph_msg, std::vector<Graph::Vertex> &vl, std::vector<Graph::Edge> &el);
        static void createGraphMessage(Graph::DirectedGraph &di_graph, const std::string &graph_name, graph_msgs::msg::Graph &graph_msg);
    };
}