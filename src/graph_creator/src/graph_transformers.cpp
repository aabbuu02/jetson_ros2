#include <graph_creator/graph_transformers.hpp>


namespace Graph::Transform
{

void VertexOrientationTransform::updateIncomingEdgeCount(uint32_t vertex_id)
{
    if(m_vertex_edge_count_map.find(vertex_id) != m_vertex_edge_count_map.end())
    {
        m_vertex_edge_count_map.at(vertex_id).incoming_count++;
    }
    else
    {
        VertexEdgeCount v_ec(vertex_id);
        v_ec.incoming_count = 1;
        m_vertex_edge_count_map.emplace(vertex_id, v_ec);
    }
}

void VertexOrientationTransform::updateOutgoingEdgeCount(uint32_t vertex_id)
{
    if(m_vertex_edge_count_map.find(vertex_id) != m_vertex_edge_count_map.end())
    {
        m_vertex_edge_count_map.at(vertex_id).outging_count++;
    }
    else
    {
        VertexEdgeCount v_ec(vertex_id);
        v_ec.outging_count = 1;
        m_vertex_edge_count_map.emplace(vertex_id, v_ec);
    }
}

void VertexOrientationTransform::updateOutAndInEdgeCount(uint32_t out_vertex_id, uint32_t in_vertex_id)
{
    updateOutgoingEdgeCount(out_vertex_id);
    updateIncomingEdgeCount(in_vertex_id);
}


uint32_t VertexOrientationTransform::getOutgoingEdgeCount(uint32_t vertex_id)
{
    if(m_vertex_edge_count_map.find(vertex_id) != m_vertex_edge_count_map.end())
        return m_vertex_edge_count_map.at(vertex_id).outging_count;

    return 0;
}

uint32_t VertexOrientationTransform::getIncomingEdgeCount(uint32_t vertex_id)
{
    if(m_vertex_edge_count_map.find(vertex_id) != m_vertex_edge_count_map.end())
        return m_vertex_edge_count_map.at(vertex_id).incoming_count;

    return 0;
}

VertexOrientationTransform::VertexEdgeCount VertexOrientationTransform::getOutAndInEdgeCount(uint32_t vertex_id)
{
    if(m_vertex_edge_count_map.find(vertex_id) != m_vertex_edge_count_map.end())
        return m_vertex_edge_count_map.at(vertex_id);

    return VertexEdgeCount(0);
}

uint32_t VertexOrientationTransform::getTotalEdgeCount(uint32_t vertex_id)
{
    if(m_vertex_edge_count_map.find(vertex_id) != m_vertex_edge_count_map.end())
        return m_vertex_edge_count_map.at(vertex_id).getTotalCount();

    return 0;
}

bool VertexOrientationTransform::updateVertexOrientationUsingInCount(Graph::Vertex &v, uint32_t in_count_threshold)
{
    if(getIncomingEdgeCount(v.vertex_id) >= in_count_threshold)
    {
        v.ignore_orientation = true;
        return true;
    }

    return false;
}

bool VertexOrientationTransform::updateVertexOrientationUsingOutCount(Graph::Vertex &v, uint32_t out_count_threshold)
{
    if(getOutgoingEdgeCount(v.vertex_id) >= out_count_threshold)
    {
        v.ignore_orientation = true;
        return true;
    }

    return false;
}   

bool VertexOrientationTransform::updateVertexOrientationUsingTotalCount(Graph::Vertex &v, uint32_t total_count_threshold)
{
    if(getTotalEdgeCount(v.vertex_id) >= total_count_threshold) 
    {
        v.ignore_orientation = true;
        return true;
    }

    return false;
}

void VertexOrientationTransform::reset()
{
    m_vertex_edge_count_map.clear();
}

}