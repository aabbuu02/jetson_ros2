/**
 * @file graph_creator.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include <rclcpp/rclcpp.hpp>
#include <graph_creator_msgs/msg/graph_generator_config.hpp>
#include <graph_msgs/msg/graph.hpp>
#include "graph_creator/graph_generators.hpp"
#include "graph_creator/graph_visuals.hpp"
#include "graph_creator/graph_data_types.hpp"

class GraphCreatorNode : public rclcpp::Node
{
public:
    GraphCreatorNode() : Node("graph_creator_node")
    {
        RCLCPP_INFO(this->get_logger(), "Graph Creator Node started");
        
        // Initialize graph generator with default config
        graph_creator_msgs::msg::GraphGeneratorConfig config;
        
        graph_creator::GraphGenerator generator(config);
        
        Graph::VertexList vertices;
        Graph::EdgeList edges;
        
        if (generator.generateVerticesAndEdgesList(vertices, edges)) {
            RCLCPP_INFO(this->get_logger(), "Generated %zu vertices and %zu edges", vertices.size(), edges.size());
            
            // Create visualization markers
            auto vertex_markers = graph_creator::GraphVisuals::createVertexMarkers(vertices);
            auto edge_markers = graph_creator::GraphVisuals::createEdgeMarkers(edges);
            
            RCLCPP_INFO(this->get_logger(), "Created visualization markers");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to generate graph");
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraphCreatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}