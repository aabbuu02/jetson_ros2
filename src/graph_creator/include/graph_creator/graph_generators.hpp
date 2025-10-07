/**
 * @file graph_generators.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#pragma once

#include <graph_creator_msgs/msg/graph_generator_config.hpp>
#include "graph_data_types.hpp"

namespace graph_creator
{
    class GraphGenerator
    {
    public:
        GraphGenerator();
        GraphGenerator(const graph_creator_msgs::msg::GraphGeneratorConfig &graph_gen_cfg_msg);
        
        bool generateVerticesAndEdgesList(Graph::VertexList &vl, Graph::EdgeList &el);
        bool generateVerticesAndEdgesList(const graph_creator_msgs::msg::GraphGeneratorConfig &graph_gen_cfg_msg, Graph::VertexList &vl, Graph::EdgeList &el);
        bool generateGridVerticesAndEdgesList(const graph_creator_msgs::msg::GraphGeneratorConfig &graph_gen_cfg_msg, Graph::VertexList &vl, Graph::EdgeList &el);

    private:
        graph_creator_msgs::msg::GraphGeneratorConfig m_graph_gen_cfg_msg;
        bool checkGridGraphGeneratorConfig(const graph_creator_msgs::msg::GraphGeneratorConfig &config) const;
    };
}