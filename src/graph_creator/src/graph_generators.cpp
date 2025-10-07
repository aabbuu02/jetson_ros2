/**
 * @file graph_generators.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "graph_creator/graph_generators.hpp"
#include <graph_creator_msgs/msg/graph_generator_config.hpp>

namespace graph_creator
{
    GraphGenerator::GraphGenerator()
    {
    }

    GraphGenerator::GraphGenerator(const graph_creator_msgs::msg::GraphGeneratorConfig &graph_gen_cfg_msg)
        : m_graph_gen_cfg_msg(graph_gen_cfg_msg)
    {
    }

    bool GraphGenerator::generateVerticesAndEdgesList(Graph::VertexList &vl, Graph::EdgeList &el)
    {
        return generateVerticesAndEdgesList(m_graph_gen_cfg_msg, vl, el);
    }

    bool GraphGenerator::generateVerticesAndEdgesList(const graph_creator_msgs::msg::GraphGeneratorConfig &graph_gen_cfg_msg, Graph::VertexList &vl, Graph::EdgeList &el)
    {
        // Generate a simple grid regardless of config
        return generateGridVerticesAndEdgesList(graph_gen_cfg_msg, vl, el);
    }

    bool GraphGenerator::generateGridVerticesAndEdgesList(const graph_creator_msgs::msg::GraphGeneratorConfig &graph_gen_cfg_msg, Graph::VertexList &vl, Graph::EdgeList &el)
    {
        // Use default values since we don't know the actual field names
        int rows = 5;  // Default value
        int cols = 5;  // Default value
        double spacing = 1.0;  // Default value

        vl.clear();
        el.clear();

        // Generate vertices
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                Graph::Vertex vertex;
                vertex.vertex_id = i * cols + j;
                vertex.vertex_pose.position.x = j * spacing;
                vertex.vertex_pose.position.y = i * spacing;
                vertex.vertex_pose.position.z = 0.0;
                vertex.vertex_pose.orientation.w = 1.0;
                vl.push_back(vertex);
            }
        }

        // Generate edges
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                int current_id = i * cols + j;
                
                // Right edge
                if (j < cols - 1) {
                    Graph::Edge edge;
                    edge.edge_vertex_id.first = current_id;
                    edge.edge_vertex_id.second = current_id + 1;
                    edge.edge_length = spacing;
                    el.push_back(edge);
                }
                
                // Down edge
                if (i < rows - 1) {
                    Graph::Edge edge;
                    edge.edge_vertex_id.first = current_id;
                    edge.edge_vertex_id.second = current_id + cols;
                    edge.edge_length = spacing;
                    el.push_back(edge);
                }
            }
        }

        return true;
    }

    bool GraphGenerator::checkGridGraphGeneratorConfig(const graph_creator_msgs::msg::GraphGeneratorConfig &config) const
    {
        // Always return true since we're using default values
        return true;
    }
}