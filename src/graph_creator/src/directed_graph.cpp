/**
 * @file directed_graph.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "graph_creator/graph_data_types.hpp"

namespace Graph
{
    void DirectedGraph::addVertex(const Vertex& vertex)
    {
        VertexPtr vertex_ptr = std::make_shared<Vertex>(vertex);
        vertices_[vertex.vertex_id] = vertex_ptr;
    }

    void DirectedGraph::addEdge(const Edge& edge)
    {
        EdgePtr edge_ptr = std::make_shared<Edge>(edge);
        edges_.push_back(edge_ptr);
    }

    VertexPtr DirectedGraph::getVertex(int id)
    {
        auto it = vertices_.find(id);
        if (it != vertices_.end()) {
            return it->second;
        }
        return nullptr;
    }

    EdgePtr DirectedGraph::getEdge(int from_id, int to_id)
    {
        for (auto& edge : edges_) {
            if (edge->edge_vertex_id.first == from_id && edge->edge_vertex_id.second == to_id) {
                return edge;
            }
        }
        return nullptr;
    }

    VertexList DirectedGraph::getVertices() const
    {
        VertexList vertices;
        for (const auto& pair : vertices_) {
            vertices.push_back(*pair.second);
        }
        return vertices;
    }

    EdgeList DirectedGraph::getEdges() const
    {
        EdgeList edges;
        for (const auto& edge_ptr : edges_) {
            edges.push_back(*edge_ptr);
        }
        return edges;
    }

    void DirectedGraph::clear()
    {
        vertices_.clear();
        edges_.clear();
    }
}