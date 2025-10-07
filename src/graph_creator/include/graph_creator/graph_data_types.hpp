#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <functional>
#include <memory>
#include <unordered_set>
#include <unordered_map>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>

namespace Graph
{
    //Properties for vertices, edges and graph
    typedef std::unordered_map<std::string, std::string> Properties;
    typedef std::shared_ptr<Properties> PropertiesPtr;
    
    struct  Edge
    {
        std::pair<uint32_t, uint32_t> edge_vertex_id;
        std::pair<std::string, std::string> edge_vertices_name; //Vertices that form this edge
        std::string edge_name;
        std::string edge_alias;
        double edge_length = 0.0f;
        double edge_cost_factor = 1.0f;

        uint8_t edge_type = 0;
        uint8_t edge_direction_type = 0;
        std::vector<std::string> control_poses_name; //Control pose marker name
        std::vector<geometry_msgs::Pose> control_poses;
        
        bool is_bidirectional = false;
        std::string forward_direction_name, reverse_direction_name; //The edge direction marker name (Poses not used) 

        bool use_independent_orientation = false;
        std::string control_orientation_name; //Control orientation marker name
        geometry_msgs::Pose control_orientation_pose;

        geometry_msgs::PoseArray forward_edge_poses, backward_edge_poses; //Backward edge poses not used
        Properties edge_properties;

        Edge() {}

        Edge(const std::string &name):
            edge_name(name)
        {}

        bool operator==(const Edge &other) const
        {
            return (this->edge_vertex_id.first == other.edge_vertex_id.first && this->edge_vertex_id.second == other.edge_vertex_id.second);
        }
    };

    struct EdgeHashFunction
    {
        size_t operator()(const Edge &e) const
        {
            return std::hash<std::string>()(e.edge_name);
        }
    };

    struct Vertex
    {
        uint32_t vertex_id = 0;
        std::string vertex_name;
        std::string vertex_alias; //Will be empty on creation
        
        uint32_t vertex_type = 0;

        geometry_msgs::Pose vertex_pose;
        bool ignore_orientation = false;
        
        Properties vertex_properties; //Will be empty on creation

        Vertex(){}

        Vertex(const std::string &name):
            vertex_name(name)
        {}

        bool operator==(const Vertex &other) const
        {
            return this->vertex_id == other.vertex_id;
        }
    };

    struct VertexHashFunction
    {
        size_t operator()(const Vertex &v) const
        {
            return std::hash<std::string>()(v.vertex_name);
        }
    };

    //Vertex and edge list
    typedef std::vector<Vertex> VertexList;
    typedef std::vector<Edge> EdgeList;

    //Vertex and edge pointers
    typedef std::shared_ptr<Vertex> VertexPtr;
    typedef std::shared_ptr<Edge> EdgePtr;

    //Graph vertices
    typedef std::unordered_map<std::string, VertexPtr> VerticesMap;
    typedef std::unordered_map<std::string, EdgePtr> EdgesMap;

    //Edge and vertex map pointers
    typedef std::shared_ptr<VerticesMap> VerticesMapPtr;
    typedef std::shared_ptr<EdgesMap> EdgesMapPtr;
    
    //Graph vertex to edge dependencies
    typedef std::unordered_set<std::string> EdgeDepSet;
    typedef std::unordered_map<std::string, EdgeDepSet> VertexToEdgesDepMap;

    //Graph edge dependencies
    typedef std::pair<std::string, std::string> VertexDepPair;
    typedef std::unordered_map<std::string, VertexDepPair> EdgeToVerticesDepMap;

    //Graph miscellaneous dependencies
    typedef std::unordered_set<std::string> MiscDepSet;
    typedef std::unordered_map<std::string, MiscDepSet> MiscDepMap;

    //Graph adjacency list
    typedef std::unordered_set<std::string> AdjacencyVertexSet;
    typedef std::unordered_map<std::string, AdjacencyVertexSet> AdjacencyListMap;
}

