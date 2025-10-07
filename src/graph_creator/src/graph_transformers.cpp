/**
 * @file graph_transformers.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "graph_creator/graph_transformers.hpp"
#include "graph_creator/graph_utils.hpp"

namespace graph_creator
{
    graph_msgs::msg::Graph GraphTransformers::toROSGraph(const Graph::DirectedGraph& graph)
    {
        graph_msgs::msg::Graph graph_msg;
        Graph::VertexList vertices = graph.getVertices();
        Graph::EdgeList edges = graph.getEdges();
        
        GraphUtils::convertFromEdgeAndVertexListToMsg(vertices, edges, graph_msg);
        
        return graph_msg;
    }

    Graph::DirectedGraph GraphTransformers::fromROSGraph(const graph_msgs::msg::Graph& graph_msg)
    {
        Graph::DirectedGraph graph;
        Graph::VertexList vertices;
        Graph::EdgeList edges;
        
        GraphUtils::convertFromMsgToEdgeAndVertexList(graph_msg, vertices, edges);
        
        for (const auto& vertex : vertices) {
            graph.addVertex(vertex);
        }
        
        for (const auto& edge : edges) {
            graph.addEdge(edge);
        }
        
        return graph;
    }
}