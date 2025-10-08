#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include <ros/ros.h>
#include <graph_msgs/Vertex.h>
#include <graph_msgs/Edge.h>
#include <graph_creator_msgs/GraphGeneratorConfig.h>

#include <graph_creator/graph_data_types.hpp>
#include <graph_creator/graph_visuals.hpp>

#include <graph_creator/dxf_graph_generator.hpp>
#include <graph_creator/graph_transformers.hpp>

class GraphGenerator
{
    public:

    GraphGenerator();
    GraphGenerator(const graph_creator_msgs::GraphGeneratorConfig &graph_gen_cfg_msg);
    
    bool generateVerticesAndEdgesList(Graph::VertexList &vl, Graph::EdgeList &el);
    bool generateVerticesAndEdgesList(const graph_creator_msgs::GraphGeneratorConfig &graph_gen_cfg_msg, 
                       Graph::VertexList &vl, Graph::EdgeList &el);

    bool generateGridVerticesAndEdgesList(const graph_creator_msgs::GraphGeneratorConfig &graph_gen_cfg_msg,
                                          Graph::VertexList &vl, Graph::EdgeList &el);

    bool checkGridGraphGeneratorConfig(const graph_creator_msgs::GraphGeneratorConfig &config) const;

    private:

    Graph::Transform::VertexOrientationTransform m_vot;

    graph_creator_msgs::GraphGeneratorConfig m_graph_gen_cfg_msg;
    
    DxfGraphGenerator m_dgg;
};