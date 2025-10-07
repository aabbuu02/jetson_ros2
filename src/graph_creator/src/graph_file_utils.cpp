/**
 * @file graph_file_utils.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "graph_creator/graph_file_utils.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace graph_creator
{
    std::string GraphFileUtils::getGraphFilePath(const std::string& graph_name)
    {
        try {
            std::string package_path = ament_index_cpp::get_package_share_directory("graph_creator");
            return package_path + "/maps/" + graph_name + ".graphml";
        } catch (...) {
            return "";
        }
    }

    bool GraphFileUtils::saveGraphToFile(const std::string& file_path, const Graph::DirectedGraph& graph)
    {
        // Implementation for saving graph to file
        return true;
    }

    bool GraphFileUtils::loadGraphFromFile(const std::string& file_path, Graph::DirectedGraph& graph)
    {
        // Implementation for loading graph from file
        return true;
    }
}