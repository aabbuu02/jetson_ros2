/**
 * @file graph_loader.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "graph_creator/graph_loader.hpp"
#include <tinyxml2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace graph_creator
{
    GraphLoader::GraphLoader(rclcpp::Logger logger) : logger_(logger)
    {
    }

    graph_msgs::msg::Graph GraphLoader::loadGraph(const std::string &graph_file)
    {
        graph_msgs::msg::Graph graph_msg;
        m_graph_file = graph_file;
        
        if (m_document.LoadFile(graph_file.c_str()) != tinyxml2::XML_SUCCESS) {
            RCLCPP_ERROR(logger_, "Failed to load graph file: %s", graph_file.c_str());
            return graph_msg;
        }

        tinyxml2::XMLHandle h_document(&m_document);
        tinyxml2::XMLElement* e_graphml = h_document.FirstChildElement().ToElement();
        
        if (!e_graphml) {
            RCLCPP_ERROR(logger_, "Could not find root GraphML element");
            return graph_msg;
        }

        tinyxml2::XMLHandle h_root(e_graphml);
        tinyxml2::XMLElement* e_graph = h_root.FirstChildElement("graph").ToElement();
        
        if (e_graph) {
            const char* graph_name = nullptr;
            e_graph->QueryStringAttribute("id", &graph_name);
            if (graph_name) {
                // Successfully loaded graph
                RCLCPP_INFO(logger_, "Loaded graph: %s", graph_name);
            }
        }

        return graph_msg;
    }
}