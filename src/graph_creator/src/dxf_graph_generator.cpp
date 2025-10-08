#include <graph_creator/dxf_graph_generator.hpp>


DxfGraphGenerator::DxfGraphGenerator():
    m_vertex_rtree(bgi::dynamic_quadratic(16))
{

}

DxfGraphGenerator::DxfGraphGenerator(const graph_creator_msgs::GraphGeneratorConfig &config):
    m_config(config), m_vertex_rtree(bgi::dynamic_quadratic(16))
{

}

bool DxfGraphGenerator::getDxfFilePath(const std::string &in_file_path, std::string &out_file_path)
{
    //TODO: An extension check needs to be done
    out_file_path = "";

    if(in_file_path.size() == 0)
    {
        ROS_ERROR_STREAM("The DXF file path provided is empty");
        return false;
    }

    boost::filesystem::path graph_file_path(in_file_path);
    if(!graph_file_path.is_absolute())
    {
        std::string package_name = graph_file_path.begin()->string();
        ROS_INFO_STREAM("Retrieving path relative to package: "<<package_name);
        std::string package_path_str = ros::package::getPath(package_name);
        if(package_path_str.size() == 0)
        {
            ROS_ERROR_STREAM("Could not locate DXF file relative to package");
            return false;
        }

        boost::filesystem::path package_path(package_path_str);
        out_file_path = (package_path.parent_path()/graph_file_path).string();
        return true;
    }
    
    out_file_path = graph_file_path.string();
    return true;
}

bool DxfGraphGenerator::loadDxfFile()
{
    return loadDxfFile(m_config.dxf_file_path);
}
    
bool DxfGraphGenerator::loadDxfFile(const std::string &file_path)
{
    std::string final_path = "";
    if(!getDxfFilePath(file_path, final_path))
    {
        ROS_ERROR_STREAM("An invalid DXF file path was provided");
        return false;
    }

    //This will call the add line and other overidden methods, so before that the list is cleared for the newly loaded file
    m_vl.clear();
    m_el.clear();
    m_vot.reset();

    if(!m_dxf.in(final_path, this))
    {
        ROS_ERROR_STREAM("Loading DXF file from path "<<final_path<<" failed");
        return false;
    }

    return true;
}

bool DxfGraphGenerator::generateVerticesAndEdgesListFromDxf(Graph::VertexList &vl, Graph::EdgeList &el)
{
    return generateVerticesAndEdgesListFromDxf(m_config, vl, el);
}

bool DxfGraphGenerator::generateVerticesAndEdgesListFromDxf(const graph_creator_msgs::GraphGeneratorConfig &config, 
                                                            Graph::VertexList &vl, Graph::EdgeList &el)
{
    m_config = config; //TODO: This copying is uneccessary

    if(!checkDxfGraphGeneratorConfig(m_config)) return false;

    if(!loadDxfFile(m_config.dxf_file_path)) return false;

    if(!m_config.ignore_vertex_orientation && m_config.use_edge_count_threshold)
    {
        for(auto &v: m_vl)
            m_vot.updateVertexOrientationUsingTotalCount(v, m_config.edge_count_threshold);
    }

    vl = m_vl;
    el = m_el;

    return true;
}

bool DxfGraphGenerator::checkDxfGraphGeneratorConfig(const graph_creator_msgs::GraphGeneratorConfig &config)
{
    if(config.scale_x <= 0.0f && config.scale_y <= 0.0f)
    {
        ROS_ERROR_STREAM("Both scale x and y are set to 0. Needs to be greater than 0");
        return false;
    }

    if(config.scale_x <= 0.0f)
    {
        ROS_ERROR_STREAM("Scale X is set to 0. Needs to be greater than 0");
        return false;
    }

    if(config.scale_y <= 0.0f)
    {
        ROS_ERROR_STREAM("Scale y is set to 0. Needs to be greater than 0");
        return false;
    }

    return true;
}

void DxfGraphGenerator::createVertexTreeQueryBox(const geometry_msgs::Pose &pose, double box_size_x, double box_size_y, VertexBox &query_box)
{
    //Querying the tree for the closest point
    double x1, y1, x2, y2;
    
    double box_size_half_x = box_size_x/2.0f;
    double box_size_half_y = box_size_y/2.0f;

    x1 = pose.position.x - box_size_half_x;
    y1 = pose.position.y - box_size_half_y;
    x2 = pose.position.x + box_size_half_x;
    y2 = pose.position.y + box_size_half_y;

    query_box = VertexBox(VertexPoint(x1, y1), VertexPoint(x2, y2));
}

bool DxfGraphGenerator::addEdgeForLine(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2)
{
    Graph::Vertex start_v, end_v;

    Graph::Edge e;
    e.edge_type = m_config.edge_creation_type.type;
    e.edge_direction_type = m_config.edge_direction_type.direction_type;

    //TODO: This will need to be changed on the edge direction type field is used for setting edge directionality
    if(e.edge_direction_type == graph_msgs::Edge::BIDIRECTIONAL)  e.is_bidirectional = true;
    else e.is_bidirectional = false;

    if(!addVertexFromPose(p1, p1, p2, start_v))
    {
        ROS_ERROR_STREAM("Could not retreive start vertex");
        return false;
    }

    if(!addVertexFromPose(p2, p1, p2, end_v))
    {
        ROS_ERROR_STREAM("Could not retreived end vertex");
        return false;
    }

    e.edge_name = Graph::Visuals::generateEdgeMarkerName(start_v.vertex_id, end_v.vertex_id);

    if(m_edge_set.find(e.edge_name) != m_edge_set.end())
    {
        ROS_WARN_STREAM("Multiple edge between vertices "<<start_v.vertex_id<<" and "<<end_v.vertex_id);
        return true;
    }

    e.edge_vertex_id.first = start_v.vertex_id;
    e.edge_vertices_name.first = start_v.vertex_name;
    e.edge_vertex_id.second = end_v.vertex_id;
    e.edge_vertices_name.second = end_v.vertex_name;

    m_vot.updateOutAndInEdgeCount(e.edge_vertex_id.first, e.edge_vertex_id.second);

    m_el.push_back(e);
    m_edge_set.emplace(e.edge_name);

    return true;
}

bool DxfGraphGenerator::addVertexFromPose(const geometry_msgs::Pose &p, const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2, Graph::Vertex &v)
{
    m_nearest_vertex_point.clear();

    v.vertex_type = graph_msgs::Vertex::NORMAL;
    if(m_config.ignore_vertex_orientation) v.ignore_orientation = true;

    VertexBox query_box;
    createVertexTreeQueryBox(p, m_box_size_x, m_box_size_y, query_box);
    
    //Check if a line starting point is already near a previously inserted point
    m_vertex_rtree.query(bgi::intersects(query_box), std::back_inserter(m_nearest_vertex_point));

    if(m_nearest_vertex_point.size() == 0)
    {
        m_vertex_counter++;

        m_vertex_rtree.insert(std::make_pair(VertexPoint(p.position.x, p.position.y), m_vertex_counter));
        
        v.vertex_id = m_vertex_counter;
        v.vertex_name = Graph::Visuals::generateVertexMarkerName(v.vertex_id);
        v.vertex_pose = p;
        computeVertexOrientationForLine(p1, p2, v.vertex_pose.orientation);
        
        m_vl.push_back(v);
        m_vertex_map.emplace(v.vertex_id, v); //Key needs to be unique

        ROS_INFO_STREAM("A new vertex with ID "<<v.vertex_id<<" was inserted for ("<<p.position.x<<", "<<p.position.y<<")");
    }
    else if(m_nearest_vertex_point.size() == 1)
    {
        uint32_t v_id = m_nearest_vertex_point.back().second;

        if(m_vertex_map.find(v_id) != m_vertex_map.end())
        {
            v = m_vertex_map.at(v_id);
            ROS_INFO_STREAM("Previously inserted vertex with ID "<<v.vertex_id<<" already available for ("<<p.position.x<<", "<<p.position.y<<")");
        }
        else
        {
            ROS_ERROR_STREAM("Previously discovered vertex with ID "<<v_id<<" was not found. This should not happen");
            return false;
        }
    }
    else //Assuming the query box is small, only 1 vertex should be near the query otherwise something went wrong
    {
        ROS_WARN_STREAM("Multiple vertices near ("<<p.position.x<<", "<<p.position.y<<")");
        return false;
    }

    return true;
}

void DxfGraphGenerator::computeVertexOrientationForLine(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2, geometry_msgs::Quaternion &q) const
{
    tf2::Vector3 v1, v2;

    tf2::fromMsg(p1.position, v1);
    tf2::fromMsg(p2.position, v2);

    tf2::Vector3 unit_vec = (v2 - v1).normalize();
    double yaw = std::atan2(unit_vec.getY(), unit_vec.getX());
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    tf2::convert(quat.normalize(), q);
}

void DxfGraphGenerator::addPoint(const DL_PointData &data)
{
    ROS_INFO_STREAM("Point (x, y, z): ("<<data.x<<", "<<data.y<<", "<<data.z<<")");
    ROS_WARN_STREAM("Point is not supported");
}

void DxfGraphGenerator::addLine(const DL_LineData &data) 
{
    ROS_INFO_STREAM("Line (x1, y1) to (x2, y2) : ("<< data.x1<<", "<<data.y1<<") to ("<<data.x2 <<", "<<data.y2<<")");
    
    geometry_msgs::Pose p1, p2;
    p1.orientation.w = p2.orientation.w = 1.0;
    
    p1.position.x = data.x1 * m_config.scale_x;
    p1.position.y = data.y1 * m_config.scale_y;
    
    p2.position.x = data.x2 * m_config.scale_x;
    p2.position.y = data.y2 * m_config.scale_y;

    addEdgeForLine(p1, p2);
}   

void DxfGraphGenerator::addPolyline(const DL_PolylineData &data)
{
    ROS_WARN_STREAM("Poly line is not supported");
}

void DxfGraphGenerator::addSpline(const DL_SplineData &data)
{
    ROS_WARN_STREAM("Spline is not supported");
}

void DxfGraphGenerator::addControlPoint(const DL_ControlPointData &data)
{
    ROS_WARN_STREAM("Spline control points is not supported");
}

void DxfGraphGenerator::addKnot(const DL_KnotData &data)
{
    ROS_WARN_STREAM("Spline knots are not supported");
}

void DxfGraphGenerator::addDimAlign(const DL_DimensionData &dd, const DL_DimAlignedData &dad)
{
    ROS_WARN_STREAM("Ignoring dimension aligned");
}

void DxfGraphGenerator::addDimLinear(const DL_DimensionData &dd, const DL_DimLinearData &dld)
{
    ROS_WARN_STREAM("Ignoring dimension linear");
}

void DxfGraphGenerator::addDimRadial(const DL_DimensionData &dd, const DL_DimRadialData &drd)
{
    ROS_WARN_STREAM("Ignoring dimension radial");
}

void DxfGraphGenerator::addDimDiametric(const DL_DimensionData &dd, const DL_DimDiametricData &ddd)
{
    ROS_WARN_STREAM("Ignoring dimenstion diametric");
}

void DxfGraphGenerator::addDimAngular(const DL_DimensionData &dd, const DL_DimAngularData &dad)
{
    ROS_WARN_STREAM("Ignoring dimension angular");
}

void DxfGraphGenerator::addDimAngular3P(const DL_DimensionData &dd, const DL_DimAngular3PData &dad)
{
    ROS_WARN_STREAM("Ignoring dimension angular 3 point");
}