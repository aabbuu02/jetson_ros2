/**
 * @file graph_utils.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "graph_creator/graph_utils.hpp"

namespace graph_creator
{
    void GraphUtils::convertVertexToMsg(Graph::VertexPtr v_ptr, graph_msgs::msg::Node &vertex_msg)
    {
        if (v_ptr) {
            vertex_msg.id = v_ptr->vertex_id;
            vertex_msg.pose = v_ptr->vertex_pose;
        }
    }

    void GraphUtils::convertEdgeToMsg(Graph::EdgePtr e_ptr, graph_msgs::msg::Edge &edge_msg)
    {
        if (e_ptr) {
            edge_msg.from_id = e_ptr->edge_vertex_id.first;
            edge_msg.to_id = e_ptr->edge_vertex_id.second;
            edge_msg.weight = e_ptr->edge_length;
        }
    }

    void GraphUtils::convertFromMsgToVertex(const graph_msgs::msg::Node &vertex_msg, Graph::VertexPtr v_ptr)
    {
        if (v_ptr) {
            v_ptr->vertex_id = vertex_msg.id;
            v_ptr->vertex_pose = vertex_msg.pose;
        }
    }

    void GraphUtils::convertFromMsgToEdge(const graph_msgs::msg::Edge &edge_msg, Graph::EdgePtr e_ptr)
    {
        if (e_ptr) {
            e_ptr->edge_vertex_id.first = edge_msg.from_id;
            e_ptr->edge_vertex_id.second = edge_msg.to_id;
            e_ptr->edge_length = edge_msg.weight;
        }
    }

    void GraphUtils::convertFromGraphToMsg(Graph::DirectedGraph &di_graph, graph_msgs::msg::Graph &graph_msg)
    {
        Graph::VertexList vertices = di_graph.getVertices();
        Graph::EdgeList edges = di_graph.getEdges();
        
        convertFromEdgeAndVertexListToMsg(vertices, edges, graph_msg);
    }

    void GraphUtils::convertFromEdgeAndVertexListToMsg(const std::vector<Graph::Vertex> &vl, const std::vector<Graph::Edge> &el, graph_msgs::msg::Graph &graph_msg)
    {
        // Clear existing data - use available fields
        graph_msg.vertices.clear();
        graph_msg.edges.clear();
        
        // Convert vertices
        for (const auto& vertex : vl) {
            graph_msgs::msg::Node node_msg;
            node_msg.id = vertex.vertex_id;
            node_msg.pose = vertex.vertex_pose;
            graph_msg.vertices.push_back(node_msg);
        }
        
        // Convert edges
        for (const auto& edge : el) {
            graph_msgs::msg::Edge edge_msg;
            edge_msg.from_id = edge.edge_vertex_id.first;
            edge_msg.to_id = edge.edge_vertex_id.second;
            edge_msg.weight = edge.edge_length;
            graph_msg.edges.push_back(edge_msg);
        }
    }

    void GraphUtils::convertFromMsgToEdgeAndVertexList(const graph_msgs::msg::Graph &graph_msg, std::vector<Graph::Vertex> &vl, std::vector<Graph::Edge> &el)
    {
        vl.clear();
        el.clear();
        
        // Convert vertices - use available fields
        for (const auto& node_msg : graph_msg.vertices) {
            Graph::Vertex vertex;
            vertex.vertex_id = node_msg.id;
            vertex.vertex_pose = node_msg.pose;
            vl.push_back(vertex);
        }
        
        // Convert edges
        for (const auto& edge_msg : graph_msg.edges) {
            Graph::Edge edge;
            edge.edge_vertex_id.first = edge_msg.from_id;
            edge.edge_vertex_id.second = edge_msg.to_id;
            edge.edge_length = edge_msg.weight;
            el.push_back(edge);
        }
    }

    void GraphUtils::createGraphMessage(Graph::DirectedGraph &di_graph, const std::string &graph_name, graph_msgs::msg::Graph &graph_msg)
    {
        convertFromGraphToMsg(di_graph, graph_msg);
    }
}