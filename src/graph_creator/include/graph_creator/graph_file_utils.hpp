/**
 * @file graph_file_utils.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#pragma once

#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "graph_data_types.hpp"

namespace graph_creator
{
    class GraphFileUtils
    {
    public:
        static std::string getGraphFilePath(const std::string& graph_name);
        static bool saveGraphToFile(const std::string& file_path, const Graph::DirectedGraph& graph);
        static bool loadGraphFromFile(const std::string& file_path, Graph::DirectedGraph& graph);
    };
}