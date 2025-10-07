/**
 * @file graph_loader.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#pragma once

#include <string>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <graph_msgs/msg/graph.hpp>
#include <graph_msgs/msg/node.hpp>
#include <graph_msgs/msg/edge.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tinyxml2.h>
#include "graph_data_types.hpp"

namespace graph_creator
{
    class GraphLoader
    {
    public:
        GraphLoader(rclcpp::Logger logger);
        graph_msgs::msg::Graph loadGraph(const std::string &graph_file);

    private:
        rclcpp::Logger logger_;
        std::string m_graph_file;
        tinyxml2::XMLDocument m_document;
    };
}