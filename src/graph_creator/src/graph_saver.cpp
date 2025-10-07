#include <graph_creator/graph_saver.hpp>

GraphSaver::GraphSaver(const std::string &graph_name):
    m_graph_name(graph_name), m_declaration(new TiXmlDeclaration("1.0", "UTF-8", "no")),
    m_graphml(new TiXmlElement("graphml")), m_graph(new TiXmlElement("graph"))
{
    m_document.LinkEndChild(m_declaration);

    m_graphml->SetAttribute("xmlns", "http://graphml.graphdrawing.org/xmlns");
    m_graphml->SetAttribute("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance");
    
    std::stringstream ss;
    ss<<"http://graphml.graphdrawing.org/xmlns ";
    ss<<"http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd";

    m_graphml->SetAttribute("xsi:schemaLocation", ss.str());
    m_document.LinkEndChild(m_graphml);

    m_graph->SetAttribute("id", m_graph_name);
    m_graph->SetAttribute("edgedefault", "directed");
}

void GraphSaver::convertGraph(Graph::DiGraph &di_graph)
{
    Graph::VerticesMapPtr vm_ptr = di_graph.getVertices();
    Graph::EdgesMapPtr em_ptr = di_graph.getEdges();

    //TODO: More parse information to be added
    m_graph->SetAttribute("parse.nodes", vm_ptr->size());
    m_graph->SetAttribute("parse.edges", em_ptr->size());
    m_graph->SetAttribute("parse.order", "nodesfirst");

    //For vertices
    for(Graph::VerticesMap::iterator v_it = vm_ptr->begin(); v_it != vm_ptr->end(); ++v_it)
    {
        m_graph->LinkEndChild(createNode(v_it->second));
    }

    //For edges
    for(Graph::EdgesMap::iterator e_it = em_ptr->begin(); e_it != em_ptr->end(); ++e_it)
    {
        m_graph->LinkEndChild(createEdge(e_it->second));
    }

    m_graphml->LinkEndChild(m_graph);
}

bool GraphSaver::saveGraph(const std::string &file_path)
{
    return m_document.SaveFile(file_path);
}

bool GraphSaver::saveGraph(const std::string &path, const std::string &filename)
{
    return saveGraph(path+filename);
}

TiXmlElement* GraphSaver::createNode(const Graph::VertexPtr vertex)
{
    TiXmlElement* node_element = new TiXmlElement("node");

    node_element->SetAttribute("id", vertex->vertex_id);
    node_element->LinkEndChild(addData("name", vertex->vertex_name));
    node_element->LinkEndChild(addData("alias", vertex->vertex_alias));
    node_element->LinkEndChild(addData("type", std::to_string(vertex->vertex_type)));

    std::ostringstream ss_bool;
    ss_bool<<std::boolalpha<<vertex->ignore_orientation;
    node_element->LinkEndChild(addData("ignore.orientation", ss_bool.str()));

    std::string pose_str;
    poseToString(vertex->vertex_pose, pose_str);
    node_element->LinkEndChild(addData("pose", pose_str));

    node_element->LinkEndChild(addData("properties.size", std::to_string(vertex->vertex_properties.size())));

    std::string properties_str;
    propertiesToString(vertex->vertex_properties, properties_str);
    node_element->LinkEndChild(addData("properties", properties_str));

    return node_element;
}

TiXmlElement* GraphSaver::createEdge(const Graph::EdgePtr edge)
{
    TiXmlElement* edge_element = new TiXmlElement("edge");

    std::stringstream edge_id_ss;
    edge_id_ss<<edge->edge_vertex_id.first<<"---"<<edge->edge_vertex_id.second;
    edge_element->SetAttribute("id", edge_id_ss.str());
    edge_element->SetAttribute("source", edge->edge_vertex_id.first);
    edge_element->SetAttribute("target", edge->edge_vertex_id.second);
    
    edge_element->LinkEndChild(addData("name", edge->edge_name));
    edge_element->LinkEndChild(addData("alias", edge->edge_alias));
    edge_element->LinkEndChild(addData("source.vertex.name", edge->edge_vertices_name.first));
    edge_element->LinkEndChild(addData("target.vertex.name", edge->edge_vertices_name.second));
    edge_element->LinkEndChild(addData("type", std::to_string(edge->edge_type)));
    edge_element->LinkEndChild(addData("direction.type", std::to_string(edge->edge_direction_type)));
    edge_element->LinkEndChild(addData("length", std::to_string(edge->edge_length)));
    edge_element->LinkEndChild(addData("cost.factor", std::to_string(edge->edge_cost_factor)));

    //TODO: Pose spacing and any other curve related property needs to be saved in the GraphML file
    
    std::ostringstream ss_bool;
    ss_bool<<std::boolalpha<<edge->is_bidirectional;
    edge_element->LinkEndChild(addData("bidirectional",ss_bool.str()));

    edge_element->LinkEndChild(addData("control.points.size", std::to_string(edge->control_poses.size())));
    
    std::stringstream cp_ss;

    //TODO: This needs to be updated to extended version of GraphML for list of Poses
    for(int i = 0; i < edge->control_poses.size(); ++i)
    {
        std::string pose_str;
        poseToString(edge->control_poses.at(i), pose_str);

        cp_ss<<pose_str;

        if(i < edge->control_poses.size()-1) cp_ss<<" ";
    }

    edge_element->LinkEndChild(addData("control.points.pose", cp_ss.str()));

    ss_bool.str(std::string());
    ss_bool<<std::boolalpha<<edge->use_independent_orientation;
    edge_element->LinkEndChild(addData("use.independent.orientation", ss_bool.str()));

    std::string pose_str;
    poseToString(edge->control_orientation_pose, pose_str);

    edge_element->LinkEndChild(addData("control.edge.orientation.pose", pose_str));

    edge_element->LinkEndChild(addData("properties.size", std::to_string(edge->edge_properties.size())));

    std::string properties_str;
    propertiesToString(edge->edge_properties, properties_str);
    edge_element->LinkEndChild(addData("properties", properties_str));

    return edge_element;
}

TiXmlElement* GraphSaver::addData(const std::string& key, const std::string &data)
{
    TiXmlElement* data_element = new TiXmlElement("data");
    data_element->SetAttribute("key", key);
    
    TiXmlText* data_text = new TiXmlText(data);
    data_element->LinkEndChild(data_text);

    return data_element;
}

void GraphSaver::quaternionToEulerYPR(const geometry_msgs::Quaternion &quat_msg, double &y, double &p, double &r) const
{
    tf2::Quaternion q;
    tf2::fromMsg(quat_msg, q);
    
    if(q.getZ() == 0.0f && q.getW() == 0.0f) 
        q.setW(1.0f);

    tf2::getEulerYPR(q, y, p, r);
}

void GraphSaver::poseToString(const geometry_msgs::Pose &pose, std::string &pose_str)
{
    std::stringstream ss;

    //In XYZ format
    ss<<pose.position.x<<" "<<pose.position.y<<" "<<pose.position.z<<" ";

    double yaw, pitch, roll;
    quaternionToEulerYPR(pose.orientation, yaw, pitch, roll);

    //In RPY format
    ss<<roll<<" "<<pitch<<" "<<yaw;

    pose_str = ss.str();
}

void GraphSaver::propertiesToString(const Graph::Properties &properties, std::string &properties_str)
{
    std::stringstream ss;

    //TODO: This need to be robust to spaces in key and value string. Right now spaces cannot be used in key and value string
    for(auto it = properties.begin(); it != properties.end(); ++it)
    {
        ss<<it->first<<" "<<it->second;
        if(std::next(it) != properties.end()) ss<<" ";
    }

    properties_str = ss.str();
}