/**
 * @file graph_curves.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#pragma once

#include "graph_data_types.hpp"

namespace graph_creator
{
    class GraphCurves
    {
    public:
        static void generateCurvedEdges(Graph::EdgeList& edges);
        static void smoothPath(Graph::VertexList& vertices);
    };
}