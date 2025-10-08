#include <graph_creator/graph_loader.hpp>


GraphLoader::GraphLoader()
{

}

bool GraphLoader::loadGraph(const std::string& file_path)
{
    TiXmlDocument document(file_path);

    if(!document.LoadFile()) return false;

    m_document = document;

    //TODO: Parse basic graph information here itself
    
    return true;
}

bool GraphLoader::loadGraph(const std::string& path, const std::string &filename)
{
    return loadGraph(path+filename);
}

bool GraphLoader::getVerticesAndEdges(Graph::VertexList &vl, Graph::EdgeList &el)
{
    //TODO: Move to GML (Graph Modelling Language) which uses YAML like format
    //Clear the lists just in case
    vl.clear();
    el.clear();

    TiXmlHandle h_document(&m_document);

    TiXmlElement* e_graphml;

    TiXmlHandle h_root(0);

    //GraphML

    e_graphml = h_document.FirstChildElement().Element();

    if(!e_graphml) 
    {
        ROS_ERROR_STREAM("Could not find root GraphML element");
        return false;
    }

    h_root = TiXmlHandle(e_graphml);

    //Graph
    TiXmlElement* e_graph = h_root.FirstChild("graph").Element();
    e_graph->QueryStringAttribute("id", &m_graph_name);
    e_graph->QueryStringAttribute("edgedefault", &m_graph_type);
    e_graph->QueryIntAttribute("parse.nodes", &m_no_of_nodes);
    e_graph->QueryIntAttribute("parse.edges", &m_no_of_edges);
    e_graph->QueryStringAttribute("parse.order", &m_graph_gen_order);

    //TODO: Graph name and type should be accessible before attempting to parse the GraphML file
    ROS_INFO_STREAM("Graph name: "<<m_graph_name);
    ROS_INFO_STREAM("Graph type: "<<m_graph_type);
    ROS_INFO_STREAM("Number of nodes: "<<m_no_of_nodes);
    ROS_INFO_STREAM("Number of edges: "<<m_no_of_edges);
    ROS_INFO_STREAM("Graph generation order: "<<m_graph_gen_order);

    vl.reserve(m_no_of_nodes);
    el.reserve(m_no_of_edges);

    //Nodes and edges
    TiXmlElement* e_ne = e_graph->FirstChildElement();

    for(e_ne; e_ne; e_ne = e_ne->NextSiblingElement())
    {
        if(e_ne->ValueStr() == "node")
        {
            Graph::Vertex v;
            if(!parseVertexInformation(e_ne, v)) 
            {
                ROS_ERROR_STREAM("Invalid node provided");
                return false;
            }
            vl.push_back(v);
        }
        else if(e_ne->ValueStr() == "edge")
        {
            Graph::Edge e;
            if(!parseEdgeInformation(e_ne, e)) 
            {
                ROS_ERROR_STREAM("Invalid edge provided");
                return false;
            }
            el.push_back(e);
        }
        else ROS_ERROR_STREAM("Unknown or unsupported tag "<<e_ne->ValueStr());
    }

    return true;
}

bool GraphLoader::parseVertexInformation(TiXmlElement* e_node, Graph::Vertex &v)
{
    e_node->QueryUnsignedAttribute("id", &v.vertex_id);

    TiXmlElement* e_data = e_node->FirstChildElement();

    m_vert_parse_map.clear();
    for(e_data; e_data; e_data = e_data->NextSiblingElement())
    {   
        std::string key;
        e_data->QueryStringAttribute("key", &key);
        
        if(m_vert_parse_map.find(key) != m_vert_parse_map.end())
        {
            ROS_INFO_STREAM("Multiple keys of "<<key<<" present for node with ID "<<v.vertex_id);
            return false;
        }


        if(e_data->GetText()) m_vert_parse_map.emplace(key, e_data->GetText());
        else m_vert_parse_map.emplace(key, "");
    }

    if(!checkDataKey(m_vert_parse_map, "name"))
    {
        ROS_ERROR_STREAM("Field name was not found for node with ID "<<v.vertex_id);
        return false;
    }
    v.vertex_name = m_vert_parse_map.at("name");

    if(checkDataKey(m_vert_parse_map, "alias")) v.vertex_alias = m_vert_parse_map.at("alias");

    if(!checkDataKey(m_vert_parse_map, "type"))
    {
        ROS_ERROR_STREAM("Field type was not found for node with ID "<<v.vertex_id);
        return false;
    }
    v.vertex_type = std::atoi(m_vert_parse_map.at("type").c_str());

    //TODO: This will strictly checked in later versions
    if(checkDataKey(m_vert_parse_map, "ignore.orientation"))
    {
        std::stringstream ss;
        if(m_vert_parse_map.at("ignore.orientation").size() > 0)
        {
            ss.str(m_vert_parse_map.at("ignore.orientation"));
            ss>>std::boolalpha>>v.ignore_orientation;
        }
        else ROS_WARN_STREAM("Vertex ignore orientation data could not be retreived");
    }
    else
    {
        ROS_WARN_STREAM("Field ignore orientation for vertex with ID "<<v.vertex_id);
        ROS_WARN_STREAM("Setting ignore orientation to false");
        v.ignore_orientation = false;
    }

    if(!checkDataKey(m_vert_parse_map, "pose"))
    {
        ROS_ERROR_STREAM("Field pose was not found for node with ID "<<v.vertex_id);
        return false;
    }
    stringToPose(m_vert_parse_map.at("pose"), v.vertex_pose);

    int no_of_properties = 0;
    if(checkDataKey(m_vert_parse_map, "properties.size")) 
        no_of_properties = std::atoi(m_vert_parse_map.at("properties.size").c_str());

    if(no_of_properties != 0)
    {
        if(!checkDataKey(m_vert_parse_map, "properties"))
        {
            ROS_ERROR_STREAM("Field properties was not found for node with ID "<<v.vertex_id);
            return false;
        }

        stringToProperties(m_vert_parse_map.at("properties"), v.vertex_properties);

        if(no_of_properties != v.vertex_properties.size())
        {
            ROS_ERROR_STREAM("Vertex properties count does not match for node with ID "<<v.vertex_id);
            return false;
        }
    }

    return true;
}

bool GraphLoader::parseEdgeInformation(TiXmlElement* e_edge, Graph::Edge &e)
{
    e_edge->QueryUnsignedAttribute("source", &e.edge_vertex_id.first);
    e_edge->QueryUnsignedAttribute("target", &e.edge_vertex_id.second);

    TiXmlElement* e_data = e_edge->FirstChildElement();

    m_edge_parse_map.clear();
    for(e_data; e_data; e_data = e_data->NextSiblingElement())
    {   
        std::string key;
        e_data->QueryStringAttribute("key", &key);
        
        if(m_edge_parse_map.find(key) != m_edge_parse_map.end())
        {
            ROS_INFO_STREAM("Multiple keys of "<<key<<" present for edge from "<<e.edge_vertex_id.first<<" to "<<e.edge_vertex_id.second);
            return false;
        }

        if(e_data->GetText()) m_edge_parse_map.emplace(key, e_data->GetText());
        else m_edge_parse_map.emplace(key, "");
    }

    if(!checkDataKey(m_edge_parse_map, "name"))
    {
        ROS_ERROR_STREAM("Field name was not found for edge from "<<e.edge_vertex_id.first<<" to "<<e.edge_vertex_id.second);
        return false;
    }
    e.edge_name = m_edge_parse_map.at("name");

    if(checkDataKey(m_edge_parse_map, "alias")) e.edge_alias = m_edge_parse_map.at("alias");

    if(!checkDataKey(m_edge_parse_map, "source.vertex.name"))
    {
        ROS_ERROR_STREAM("Field source.vertex.name was not found for edge from "<<e.edge_vertex_id.first<<" to "<<e.edge_vertex_id.second);
        return false;
    }
    e.edge_vertices_name.first = m_edge_parse_map.at("source.vertex.name");

    if(!checkDataKey(m_edge_parse_map, "target.vertex.name"))
    {
        ROS_ERROR_STREAM("Field target.vertex.name was not found for edge from "<<e.edge_vertex_id.first<<" to "<<e.edge_vertex_id.second);
        return false;
    }
    e.edge_vertices_name.second = m_edge_parse_map.at("target.vertex.name");

    if(!checkDataKey(m_edge_parse_map, "type"))
    {
        ROS_ERROR_STREAM("Field type was not found for edge from "<<e.edge_vertex_id.first<<" to "<<e.edge_vertex_id.second);
        return false;
    }
    e.edge_type = std::atoi(m_edge_parse_map.at("type").c_str());

    if(!checkDataKey(m_edge_parse_map, "direction.type"))
    {
        ROS_ERROR_STREAM("Field direction.type was not for edge from "<<e.edge_vertex_id.first<<" to "<<e.edge_vertex_id.second);
        return false;
    }
    e.edge_direction_type = std::atoi(m_edge_parse_map.at("direction.type").c_str());

    if(!checkDataKey(m_edge_parse_map, "length"))
    {
        ROS_ERROR_STREAM("Field length was not for edge from "<<e.edge_vertex_id.first<<" to "<<e.edge_vertex_id.second);
        return false;
    }
    e.edge_length = std::atoi(m_edge_parse_map.at("length").c_str());

    if(checkDataKey(m_edge_parse_map, "cost.factor")) e.edge_cost_factor = std::atof(m_edge_parse_map.at("cost.factor").c_str());
    else
    {
        ROS_WARN_STREAM("Field cost.factor was not found for edge from "<<e.edge_vertex_id.first<<" to "<<e.edge_vertex_id.second);
        ROS_WARN_STREAM("Setting cost factor to default value of 1.0");
        e.edge_cost_factor = 1.0f;
    }

    if(!checkDataKey(m_edge_parse_map, "bidirectional"))
    {
        ROS_ERROR_STREAM("Field bidirectional was not found for edge from "<<e.edge_vertex_id.first<<" to "<<e.edge_vertex_id.second);
        return false;
    }

    std::stringstream ss;
    if(m_edge_parse_map.at("bidirectional").size() > 0)
    {
        ss.str(m_edge_parse_map.at("bidirectional"));
        ss>>std::boolalpha>>e.is_bidirectional;
    }
    else
    {
        ROS_ERROR_STREAM("Edge bidirectionality data could not be retreived");
        return false;
    }

    int no_of_control_points = 0;
    if(!checkDataKey(m_edge_parse_map, "control.points.size"))
    {
        ROS_ERROR_STREAM("Field control.points.size was not found for edge from "<<e.edge_vertex_id.first<<" to "<<e.edge_vertex_id.second);
        return false;
    }
    no_of_control_points = std::atoi(m_edge_parse_map.at("control.points.size").c_str());

    if(!checkDataKey(m_edge_parse_map, "control.points.pose"))
    {
        ROS_ERROR_STREAM("Field control.points.pose was not found for edge from "<<e.edge_vertex_id.first<<" to "<<e.edge_vertex_id.second);
        return false;
    }
    stringToPoses(m_edge_parse_map.at("control.points.pose"), e.control_poses);

    if(no_of_control_points != e.control_poses.size()) 
    {
        ROS_ERROR_STREAM("Control point count does not match for edge from "<<e.edge_vertex_id.first<<" to "<<e.edge_vertex_id.second);
        return false;
    }

    if(checkDataKey(m_edge_parse_map, "use.independent.orientation"))
    {
        ss.str("");
        if(m_edge_parse_map.at("use.independent.orientation").size() > 0)
        {
            ss.str(m_edge_parse_map.at("use.independent.orientation"));
            ss>>std::boolalpha>>e.use_independent_orientation;
        }
        else
        {
            ROS_ERROR_STREAM("Edge independent orientation data could not be retrieved");
            return false;
        }
        
        if(!checkDataKey(m_edge_parse_map, "control.edge.orientation.pose"))
        {
            ROS_ERROR_STREAM("Field control.edge.orientation.pose was not found for edge from "<<e.edge_vertex_id.first<<" to "<<e.edge_vertex_id.second);
            return false;
        }
        stringToPose(m_edge_parse_map.at("control.edge.orientation.pose"), e.control_orientation_pose);
    }
    else
    {
        ROS_ERROR_STREAM("Field use.independent.orientation was not found for edge from "<<e.edge_vertex_id.first<<" to "<<e.edge_vertex_id.second);
        return false;
    }

    int no_of_properties = 0;
    if(checkDataKey(m_edge_parse_map, "properties.size")) 
        no_of_properties = std::atoi(m_edge_parse_map.at("properties.size").c_str());

    if(no_of_properties != 0)
    {
        if(!checkDataKey(m_edge_parse_map, "properties"))
        {
            ROS_ERROR_STREAM("Field properties was not found for edge from "<<e.edge_vertex_id.first<<" to "<<e.edge_vertex_id.second);
            return false;
        }

        stringToProperties(m_edge_parse_map.at("properties"), e.edge_properties);

        if(no_of_properties != e.edge_properties.size())
        {
            ROS_ERROR_STREAM("Properties count does not match for edge from "<<e.edge_vertex_id.first<<" to "<<e.edge_vertex_id.second);
            return false;
        }
    }

    return true;
}

void GraphLoader::getGraphName(std::string &graph_name)
{
    graph_name = m_graph_name;
}

void GraphLoader::getGraphType(std::string &graph_type)
{
    graph_type = m_graph_type;
}

int GraphLoader::getVerticesCount()
{
    return m_no_of_nodes;
}

int GraphLoader::getEdgesCount()
{
    return m_no_of_edges;
}

void GraphLoader::getGraphGenerationOrder(std::string &order)
{
    order = m_graph_gen_order;
}

void GraphLoader::stringToPose(const std::string &pose_str, geometry_msgs::Pose &pose)
{
    std::stringstream ss(pose_str);
    
    double x,y,z,roll,pitch,yaw;
    ss>>x>>y>>z>>roll>>pitch>>yaw;

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf2::convert(q.normalize(), pose.orientation);
}

void GraphLoader::stringToPoses(const std::string &poses_str, std::vector<geometry_msgs::Pose> &poses)
{
    std::stringstream ss(poses_str);
    poses.clear();

    double x, y, z, roll, pitch, yaw;
    while(ss>>x>>y>>z>>roll>>pitch>>yaw)
    {
        geometry_msgs::Pose pose;

        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        tf2::convert(q.normalize(), pose.orientation);

        poses.push_back(pose);
    }
}

void GraphLoader::stringToProperties(const std::string &properties_str, Graph::Properties &properties)
{
    std::stringstream ss(properties_str);
    properties.clear();

    std::string key, value;
    while(ss>>key>>value)
    {
        properties.emplace(key, value);
    }
}

bool GraphLoader::checkDataKey(const ParseDataMap &parse_data_map, const std::string &check_key) const
{
    if(parse_data_map.find(check_key) == parse_data_map.end()) return false;
    return true;
}

void GraphLoader::printVertices(const Graph::VertexList &vl) const
{
    std::cout<<"\nVertices\n\n";
    
    int i = 1;
    
    for(const Graph::Vertex &v: vl)
    {
        std::cout<<"---Vertex ("<<i<<")---\n";
        std::cout<<"ID: "<<v.vertex_id<<"\n";
        std::cout<<"Name: "<<v.vertex_name<<"\n";
        std::cout<<"Alias: "<<v.vertex_alias<<"\n";
        std::cout<<"Type: "<<(int)v.vertex_type<<"\n";
        std::cout<<"Pose:\n"<<v.vertex_pose<<"\n";
        std::cout<<"------\n";
        
        i++;
    }
}

void GraphLoader::printEdges(const Graph::EdgeList &el) const
{
    std::cout<<"\nEdges\n\n";

    int i = 1;

    for(const Graph::Edge &e: el)
    {
        std::cout<<"---Edge ("<<i<<")---\n";
        std::cout<<"Edge vertices: "<<e.edge_vertex_id.first<<" -> "<<e.edge_vertex_id.second<<"\n";
        std::cout<<"Edge vertices name: "<<e.edge_vertices_name.first<<" -> "<<e.edge_vertices_name.second<<"\n";
        std::cout<<"Name: "<<e.edge_name<<"\n";
        std::cout<<"Alias: "<<e.edge_alias<<"\n";
        std::cout<<"Type: "<<(int)e.edge_type<<"\n";
        std::cout<<"Length: "<<e.edge_length<<"\n";
        std::cout<<"Bidirectional: "<<std::boolalpha<<e.is_bidirectional<<"\n";

        std::cout<<"Control point poses:\n";
        for(const geometry_msgs::Pose &p: e.control_poses)
        {
            std::cout<<p<<"\n";
        }

        std::cout<<"Use independent orientation: "<<std::boolalpha<<e.use_independent_orientation<<"\n";
        std::cout<<"Edge orientation control pose: \n"<<e.control_orientation_pose<<"\n";
        std::cout<<"------\n";

        i++;
    }
}

