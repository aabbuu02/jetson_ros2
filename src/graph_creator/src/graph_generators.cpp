#include <graph_creator/graph_generators.hpp>


GraphGenerator::GraphGenerator()
{
    
}

GraphGenerator::GraphGenerator(const graph_creator_msgs::GraphGeneratorConfig &graph_gen_cfg_msg):
    m_graph_gen_cfg_msg(graph_gen_cfg_msg)
{

}
    
bool GraphGenerator::generateVerticesAndEdgesList(Graph::VertexList &vl, Graph::EdgeList &el)
{
    return generateVerticesAndEdgesList(m_graph_gen_cfg_msg, vl, el);
}

bool GraphGenerator::generateVerticesAndEdgesList(const graph_creator_msgs::GraphGeneratorConfig &graph_gen_cfg_msg, 
                                   Graph::VertexList &vl, Graph::EdgeList &el)
{
    if(graph_gen_cfg_msg.generation_type == graph_creator_msgs::GraphGeneratorConfig::GRAPH_GENERATION_TYPE_GRID)
    {
        ROS_INFO_STREAM("Grid based graph generation requested");
        return generateGridVerticesAndEdgesList(graph_gen_cfg_msg, vl, el);
    }
    else if(graph_gen_cfg_msg.generation_type == graph_creator_msgs::GraphGeneratorConfig::GRAPH_GENERATION_TYPE_DXF)
    {
        ROS_INFO_STREAM("DXF based graph generation requested");
        return m_dgg.generateVerticesAndEdgesListFromDxf(graph_gen_cfg_msg, vl, el);
    }

    ROS_ERROR_STREAM("Unknown graph generation type was specified");
    return false;
}

bool GraphGenerator::generateGridVerticesAndEdgesList(const graph_creator_msgs::GraphGeneratorConfig &graph_gen_cfg_msg,
                                                      Graph::VertexList &vl, Graph::EdgeList &el)
{
    m_graph_gen_cfg_msg = graph_gen_cfg_msg;

    vl.clear();
    el.clear();
    m_vot.reset();

    if(!checkGridGraphGeneratorConfig(graph_gen_cfg_msg)) return false;

    //TODO: Reserve for vertex and edge list
    Graph::Vertex v;
    v.vertex_type = graph_msgs::Vertex::NORMAL;
    if(m_graph_gen_cfg_msg.ignore_vertex_orientation) 
        v.ignore_orientation = true;

    Graph::Edge e;
    e.edge_type = graph_gen_cfg_msg.edge_creation_type.type;
    e.edge_direction_type = graph_gen_cfg_msg.edge_direction_type.direction_type;

    //TODO: This will need to be changed on the edge direction type field is used for setting edge directionality
    if(e.edge_direction_type == graph_msgs::Edge::BIDIRECTIONAL)  e.is_bidirectional = true;
    else e.is_bidirectional = false;

    for(uint32_t i = 0; i < graph_gen_cfg_msg.columns; ++i)
    {
        for(uint32_t j = 0; j < graph_gen_cfg_msg.rows; ++j)
        {
            v.vertex_id++;
            v.vertex_name = Graph::Visuals::generateVertexMarkerName(v.vertex_id);
            v.vertex_pose.position.x = graph_gen_cfg_msg.start_pose.position.x + (graph_gen_cfg_msg.column_spacing * i);
            v.vertex_pose.position.y = graph_gen_cfg_msg.start_pose.position.y + (graph_gen_cfg_msg.row_spacing * j);
            vl.push_back(v);

            if(j < graph_gen_cfg_msg.rows-1) //Do not generate edges beyond the maximum row count
            {
                e.edge_vertex_id.first = v.vertex_id;
                e.edge_vertex_id.second = v.vertex_id+1;
                e.edge_vertices_name.first = v.vertex_name;
                e.edge_vertices_name.second = Graph::Visuals::generateVertexMarkerName(e.edge_vertex_id.second);
                e.edge_name = Graph::Visuals::generateEdgeMarkerName(e.edge_vertex_id.first, e.edge_vertex_id.second);

                m_vot.updateOutAndInEdgeCount(e.edge_vertex_id.first, e.edge_vertex_id.second);

                el.push_back(e);
            }

            if(i < graph_gen_cfg_msg.columns-1) //Do not generate edges beyond the maximum column count
            {
                e.edge_vertex_id.first = v.vertex_id;
                e.edge_vertex_id.second = v.vertex_id + graph_gen_cfg_msg.rows;
                e.edge_vertices_name.first = Graph::Visuals::generateVertexMarkerName(v.vertex_id);
                e.edge_vertices_name.second = Graph::Visuals::generateVertexMarkerName(e.edge_vertex_id.second);
                e.edge_name = Graph::Visuals::generateEdgeMarkerName(e.edge_vertex_id.first, e.edge_vertex_id.second);

                m_vot.updateOutAndInEdgeCount(e.edge_vertex_id.first, e.edge_vertex_id.second);

                el.push_back(e);
            }
        }
    }

    if(!m_graph_gen_cfg_msg.ignore_vertex_orientation && m_graph_gen_cfg_msg.use_edge_count_threshold)
    {
        for(auto &v: vl)
            m_vot.updateVertexOrientationUsingTotalCount(v, m_graph_gen_cfg_msg.edge_count_threshold);
    }

    return true;
}

bool GraphGenerator::checkGridGraphGeneratorConfig(const graph_creator_msgs::GraphGeneratorConfig &config) const
{
    if(config.rows == 0 && config.columns == 0)
    {
        ROS_ERROR_STREAM("Number of rows and columns were set to 0. Needs to be at least 1 x 1");
        return false;
    }

    if(config.rows == 0)
    {
        ROS_ERROR_STREAM("Number of rows was set to 0. Needs to be at least 1");
        return false;
    }

    if(config.columns == 0)
    {
        ROS_ERROR_STREAM("Number of columns was set to 0. Needs to be at least 1");
        return false;
    }

    if(config.row_spacing <= 0.0 && config.column_spacing <= 0.0)
    {
        ROS_ERROR_STREAM("Row and column spacing were set to 0. Needs to greater than 0 for both rows and columns");
        return false;
    }

    if(config.row_spacing <= 0.0) 
    {
        ROS_ERROR_STREAM("Row spacing was set to 0. Needs to be greater than 0");
        return false;
    }

    if(config.column_spacing <= 0.0)
    {
        ROS_ERROR_STREAM("Column spacing was set to 0. Needs to be greater than 0");
        return false;
    }

    return true;
}