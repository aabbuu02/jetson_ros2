/**
 * @file graph_data_types.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <functional>
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

namespace Graph
{
    struct Vertex
    {
        int vertex_id;
        geometry_msgs::msg::Pose vertex_pose;
        std::unordered_map<std::string, std::string> properties;
        
        Vertex() : vertex_id(0) {}
        Vertex(int id, const geometry_msgs::msg::Pose& pose) : vertex_id(id), vertex_pose(pose) {}
    };

    struct Edge
    {
        std::pair<int, int> edge_vertex_id; // from_id, to_id
        double edge_length;
        std::vector<geometry_msgs::msg::Pose> control_poses;
        geometry_msgs::msg::Pose control_orientation_pose;
        bool use_independent_orientation;
        std::unordered_map<std::string, std::string> properties;
        
        Edge() : edge_length(0.0), use_independent_orientation(false) {}
        Edge(int from, int to, double length) : edge_vertex_id(from, to), edge_length(length), use_independent_orientation(false) {}
    };

    struct Properties
    {
        std::unordered_map<std::string, std::string> data;
    };

    typedef std::shared_ptr<Vertex> VertexPtr;
    typedef std::shared_ptr<Edge> EdgePtr;
    typedef std::vector<Vertex> VertexList;
    typedef std::vector<Edge> EdgeList;

    class DirectedGraph
    {
    public:
        DirectedGraph() {}
        ~DirectedGraph() {}

        void addVertex(const Vertex& vertex);
        void addEdge(const Edge& edge);
        VertexPtr getVertex(int id);
        EdgePtr getEdge(int from_id, int to_id);
        VertexList getVertices() const;
        EdgeList getEdges() const;
        void clear();

    private:
        std::unordered_map<int, VertexPtr> vertices_;
        std::vector<EdgePtr> edges_;
    };
}