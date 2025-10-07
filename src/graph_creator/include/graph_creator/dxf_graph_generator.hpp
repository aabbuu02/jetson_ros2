/**
 * @file dxf_graph_generator.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#pragma once

#include <dl_dxf.h>
#include <dl_creationadapter.h>
#include "graph_data_types.hpp"

namespace graph_creator
{
    class DXFGraphGenerator : public DL_CreationAdapter
    {
    public:
        DXFGraphGenerator();
        virtual ~DXFGraphGenerator();
        
        virtual void addLine(const DL_LineData& d);
        
        Graph::VertexList getVertices() const { return vertices_; }
        Graph::EdgeList getEdges() const { return edges_; }

    private:
        Graph::VertexList vertices_;
        Graph::EdgeList edges_;
        int last_vertex_id_;
    };
}