/**
 * @file dxf_graph_generator.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "graph_creator/dxf_graph_generator.hpp"
#include "graph_creator/graph_data_types.hpp"

namespace graph_creator
{
    void DXFGraphGenerator::addLine(const DL_LineData& d)
    {
        Graph::Vertex v1, v2;
        v1.vertex_id = ++last_vertex_id_;
        v1.vertex_pose.position.x = d.x1;
        v1.vertex_pose.position.y = d.y1;
        v1.vertex_pose.position.z = 0.0;
        v1.vertex_pose.orientation.w = 1.0;
        
        v2.vertex_id = ++last_vertex_id_;
        v2.vertex_pose.position.x = d.x2;
        v2.vertex_pose.position.y = d.y2;
        v2.vertex_pose.position.z = 0.0;
        v2.vertex_pose.orientation.w = 1.0;

        Graph::Edge e;
        e.edge_vertex_id.first = v1.vertex_id;
        e.edge_vertex_id.second = v2.vertex_id;
        e.edge_length = sqrt((d.x2 - d.x1) * (d.x2 - d.x1) + (d.y2 - d.y1) * (d.y2 - d.y1));

        vertices_.push_back(v1);
        vertices_.push_back(v2);
        edges_.push_back(e);
    }
}