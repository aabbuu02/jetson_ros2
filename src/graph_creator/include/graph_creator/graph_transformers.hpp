/**
 * @file graph_transformers.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#pragma once

#include <graph_msgs/msg/graph.hpp>
#include "graph_data_types.hpp"

namespace graph_creator
{
    class GraphTransformers
    {
    public:
        static graph_msgs::msg::Graph toROSGraph(const Graph::DirectedGraph &graph);
        static Graph::DirectedGraph fromROSGraph(const graph_msgs::msg::Graph &graph_msg);
    };
}