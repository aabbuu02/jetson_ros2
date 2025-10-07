/**
 * @file graph_server_node.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <filesystem>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "graph_msgs/msg/graph.hpp"
#include "graph_msgs/msg/node.hpp"
#include "graph_msgs/msg/edge.hpp"
#include "graph_msgs/srv/get_graph.hpp"
#include "graph_msgs/srv/load_graph.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tinyxml2.h"

namespace fs = std::filesystem;

class GraphServerNode : public rclcpp::Node
{
public:
    GraphServerNode() : Node("graph_server")
    {
        // Declare parameter for graph file
        this->declare_parameter<std::string>("graph_file", "");

        // Create publisher with transient_local QoS (latched)
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        m_graph_pub = this->create_publisher<graph_msgs::msg::Graph>("graph", qos);

        // Create services
        m_get_graph_srv = this->create_service<graph_msgs::srv::GetGraph>(
            "get_graph",
            std::bind(&GraphServerNode::getGraphCallback, this, std::placeholders::_1, std::placeholders::_2));

        m_load_graph_srv = this->create_service<graph_msgs::srv::LoadGraph>(
            "load_graph",
            std::bind(&GraphServerNode::loadGraphCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Load graph from parameter if provided
        std::string graph_file = this->get_parameter("graph_file").as_string();
        if (!graph_file.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Loading graph from: %s", graph_file.c_str());
            loadGraphFromFileAndPublish(graph_file);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No graph file provided. Use 'load_graph' service.");
        }
    }

private:
    void getGraphCallback(
        const std::shared_ptr<graph_msgs::srv::GetGraph::Request> request,
        std::shared_ptr<graph_msgs::srv::GetGraph::Response> response)
    {
        (void)request; // Unused
        response->graph = m_graph_msg;
        response->result = m_is_initialized ? response->RESULT_GRAPH_VALID : response->RESULT_GRAPH_INVALID;
        RCLCPP_INFO(this->get_logger(), "Get graph service called. Result: %d", response->result);
    }

    void loadGraphCallback(
        const std::shared_ptr<graph_msgs::srv::LoadGraph::Request> request,
        std::shared_ptr<graph_msgs::srv::LoadGraph::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Load graph service called for: %s", request->graph_url.c_str());
        if (loadGraphFromFileAndPublish(request->graph_url))
        {
            response->result = response->RESULT_SUCCESS;
            response->graph = m_graph_msg;
        }
        else
        {
            response->result = response->RESULT_GRAPH_DOES_NOT_EXIST;
        }
    }

    bool loadGraphFromFileAndPublish(const std::string& file_uri)
    {
        std::string file_path = resolveUri(file_uri);
        if (file_path.empty() || !fs::exists(file_path))
        {
            RCLCPP_ERROR(this->get_logger(), "Graph file does not exist: %s", file_path.c_str());
            return false;
        }

        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(file_path.c_str()) != tinyxml2::XML_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse graphml file: %s", file_path.c_str());
            return false;
        }

        tinyxml2::XMLElement* graphml = doc.FirstChildElement("graphml");
        if (!graphml)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid graphml: <graphml> tag missing.");
            return false;
        }

        tinyxml2::XMLElement* graph = graphml->FirstChildElement("graph");
        if (!graph)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid graphml: <graph> tag missing.");
            return false;
        }

        // Set graph type
        std::string graph_type = graph->Attribute("edgedefault", "undirected");
        m_graph_msg.meta_data.graph_type = (graph_type == "directed") ? m_graph_msg.meta_data.DIRECTED : m_graph_msg.meta_data.UNDIRECTED;

        m_graph_msg.vertices.clear();
        m_graph_msg.edges.clear();

        // Parse nodes
        for (tinyxml2::XMLElement* node = graph->FirstChildElement("node"); node; node = node->NextSiblingElement("node"))
        {
            graph_msgs::msg::Node n;
            n.id = std::stoul(node->Attribute("id"));
            // Parse position (yEd format, key="d6")
            tinyxml2::XMLElement* data = node->FirstChildElement("data");
            while (data)
            {
                if (data->Attribute("key", "d6"))
                {
                    tinyxml2::XMLElement* geometry = data->FirstChildElement("y:Geometry");
                    if (geometry)
                    {
                        n.pose.position.x = geometry->FloatAttribute("x");
                        n.pose.position.y = geometry->FloatAttribute("y");
                    }
                    break;
                }
                data = data->NextSiblingElement("data");
            }
            m_graph_msg.vertices.push_back(n);
        }

        // Parse edges
        for (tinyxml2::XMLElement* edge = graph->FirstChildElement("edge"); edge; edge = edge->NextSiblingElement("edge"))
        {
            graph_msgs::msg::Edge e;
            e.from_id = std::stoul(edge->Attribute("source"));
            e.to_id = std::stoul(edge->Attribute("target"));
            e.weight = 1.0; // Default weight
            m_graph_msg.edges.push_back(e);
        }

        // Update metadata
        m_graph_msg.header.stamp = this->get_clock()->now();
        m_graph_msg.header.frame_id = "map";
        m_graph_msg.meta_data.graph_name = fs::path(file_path).stem().string();
        m_graph_msg.meta_data.number_of_vertices = m_graph_msg.vertices.size();
        m_graph_msg.meta_data.number_of_edges = m_graph_msg.edges.size();

        RCLCPP_INFO(this->get_logger(), "Loaded graph '%s': %u vertices, %u edges, type: %s",
                    m_graph_msg.meta_data.graph_name.c_str(),
                    m_graph_msg.meta_data.number_of_vertices,
                    m_graph_msg.meta_data.number_of_edges,
                    (m_graph_msg.meta_data.graph_type == m_graph_msg.meta_data.DIRECTED ? "directed" : "undirected"));

        m_graph_pub->publish(m_graph_msg);
        m_is_initialized = true;
        return true;
    }

    std::string resolveUri(const std::string& uri)
    {
        const std::string package_prefix = "package://";
        if (uri.rfind(package_prefix, 0) == 0)
        {
            std::string path_part = uri.substr(package_prefix.length());
            size_t first_slash = path_part.find('/');
            if (first_slash == std::string::npos)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid URI: %s", uri.c_str());
                return "";
            }
            std::string package_name = path_part.substr(0, first_slash);
            std::string file_path = path_part.substr(first_slash);
            try
            {
                std::string share_dir = ament_index_cpp::get_package_share_directory(package_name);
                return share_dir + file_path;
            }
            catch (const std::runtime_error& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Package not found: %s (%s)", package_name.c_str(), e.what());
                return "";
            }
        }
        return uri; // Assume absolute or relative path
    }

    rclcpp::Publisher<graph_msgs::msg::Graph>::SharedPtr m_graph_pub;
    rclcpp::Service<graph_msgs::srv::GetGraph>::SharedPtr m_get_graph_srv;
    rclcpp::Service<graph_msgs::srv::LoadGraph>::SharedPtr m_load_graph_srv;
    graph_msgs::msg::Graph m_graph_msg;
    bool m_is_initialized = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraphServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
