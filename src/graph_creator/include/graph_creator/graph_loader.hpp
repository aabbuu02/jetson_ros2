#pragma once

#include <vector>
#include <string>
#include <unordered_map>

#include <tinyxml.h>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include "graph_data_types.hpp"

class GraphLoader
{
    public:

    GraphLoader();
    
    bool loadGraph(const std::string& file_path);
    bool loadGraph(const std::string& path, const std::string &filename);
    bool getVerticesAndEdges(Graph::VertexList &vl, Graph::EdgeList &el);
    
    void getGraphName(std::string &graph_name);
    void getGraphType(std::string &graph_type);
    int getVerticesCount();
    int getEdgesCount();
    void getGraphGenerationOrder(std::string &order);

    void printVertices(const Graph::VertexList &vl) const;
    void printEdges(const Graph::EdgeList &el) const;

    private:

    typedef std::unordered_map<std::string, std::string> ParseDataMap;

    ParseDataMap m_vert_parse_map, m_edge_parse_map;

    TiXmlDocument m_document;

    std::string m_graph_name = "";
    std::string m_graph_type = "";
    int m_no_of_nodes = 0;
    int m_no_of_edges = 0;
    std::string m_graph_gen_order = "";

    bool parseVertexInformation(TiXmlElement* e_node, Graph::Vertex &v);
    bool parseEdgeInformation(TiXmlElement* e_edge, Graph::Edge &e);
    
    void stringToPose(const std::string &pose_str, geometry_msgs::Pose &pose);
    void stringToPoses(const std::string &poses_str, std::vector<geometry_msgs::Pose> &poses);
    void stringToProperties(const std::string &properties_str, Graph::Properties &properties);

    bool checkDataKey(const ParseDataMap &parse_data_map, const std::string &check_key) const;
};