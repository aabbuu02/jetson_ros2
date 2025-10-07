/**
 * @file graph_saver.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#pragma once

#include <string>
#include "graph_data_types.hpp"

namespace graph_creator
{
    class GraphSaver
    {
    public:
        static bool saveGraphToFile(const std::string& file_path, const Graph::DirectedGraph& graph);
        static bool saveGraphToXML(const std::string& file_path, const Graph::DirectedGraph& graph);
    };
}