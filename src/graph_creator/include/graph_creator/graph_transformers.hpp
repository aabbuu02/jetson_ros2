#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include "graph_data_types.hpp"


namespace Graph::Transform
{

class VertexOrientationTransform
{
    public:

    struct VertexEdgeCount
    {
        uint32_t vertex_id = 0;
        uint32_t incoming_count = 0;
        uint32_t outging_count = 0;

        VertexEdgeCount(uint32_t id):
            vertex_id(id)
        {}

        uint32_t getTotalCount()
        {
            return incoming_count + outging_count;
        }
    };

    void updateOutgoingEdgeCount(uint32_t vertex_id);
    void updateIncomingEdgeCount(uint32_t vertex_id);    
    void updateOutAndInEdgeCount(uint32_t out_vertex_id, uint32_t in_vertex_id);

    uint32_t getOutgoingEdgeCount(uint32_t vertex_id);
    uint32_t getIncomingEdgeCount(uint32_t vertex_id);
    VertexEdgeCount getOutAndInEdgeCount(uint32_t vertex_id);
    uint32_t getTotalEdgeCount(uint32_t vertex_id);

    bool updateVertexOrientationUsingInCount(Graph::Vertex &v, uint32_t in_count_threshold);
    bool updateVertexOrientationUsingOutCount(Graph::Vertex &v, uint32_t out_count_threshold);
    bool updateVertexOrientationUsingTotalCount(Graph::Vertex &v, uint32_t total_count_threshold);

    void reset();

    private:

    std::unordered_map<uint32_t, VertexEdgeCount> m_vertex_edge_count_map;
};

}