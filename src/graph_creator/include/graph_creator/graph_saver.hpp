#pragma once

#include <vector>
#include <string>
#include <unordered_map>

#include <tinyxml.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include "directed_graph.hpp"


class GraphSaver
{
    public:

    GraphSaver(const std::string &graph_name);
    void convertGraph(Graph::DiGraph &di_graph);
    bool saveGraph(const std::string &file_path);
    bool saveGraph(const std::string &path, const std::string &filename);

    private:

    std::string m_graph_name;

    TiXmlDocument m_document;

    TiXmlDeclaration* m_declaration;
    TiXmlElement* m_graphml;
    TiXmlElement* m_graph;

    TiXmlElement* createNode(const Graph::VertexPtr vertex);
    TiXmlElement* createEdge(const Graph::EdgePtr edge);
    TiXmlElement* addData(const std::string& key, const std::string &value);
    //TODO: Add key information that support default values
    //FIXME: This needs to support Graph Modelling Language (GML) as well as GraphML

    void quaternionToEulerYPR(const geometry_msgs::Quaternion &quat_msg, double &y, double &p, double &r) const;
    void poseToString(const geometry_msgs::Pose &pose, std::string &pose_str);
    void propertiesToString(const Graph::Properties &properties, std::string &properties_str);
};