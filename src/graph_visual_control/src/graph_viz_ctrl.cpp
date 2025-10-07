#include <graph_visual_control/graph_viz_ctrl.hpp>

GraphVizCtrl::GraphVizCtrl():
    m_graph_marker_server("graph_visual_control_marker_server", "gvcms", true),
    move_base_ac_("move_base", true)
{
    //Subscribers
    m_graph_sub = m_nh.subscribe("graph", 1, &GraphVizCtrl::graphCallback, this);

    //Initialize menu
    initVertexMenu();
    initEdgeMenu();

    initialize();
}

void GraphVizCtrl::initialize()
{
    //Graph control plugin loader
    //TODO: A default graph control plugin is required
    if(!m_nh.hasParam("graph_control"))
    {
        ROS_WARN_STREAM("No graph control plugin provided for graph visual control");
        return;
    }

    std::string graph_control="";
    m_nh.getParam("graph_control", graph_control);

    if(m_gcpl.loadGraphControlPlugin(graph_control))
    {
        m_graph_control_instance = m_gcpl.getGraphControlInstance();
        m_graph_control_instance->initialize();
    }
}

void GraphVizCtrl::graphCallback(const graph_msgs::Graph &graph_msg)
{
    //NOTE: This should have no dependency on the global path graph
    ROS_INFO_STREAM("New graph received for visual control");

    m_vertex_map.clear();
    m_edge_map.clear();

    m_graph_marker_server.clear();
    m_graph_marker_server.applyChanges();

    std::vector<Graph::Vertex> vl;
    std::vector<Graph::Edge> el;

    Graph::Utilities::convertFromMsgToEdgeAndVertexList(graph_msg, vl, el);

    ROS_INFO_STREAM("Received graph message information");
    ROS_INFO_STREAM("Number of vertices: "<<graph_msg.vertices.size());
    ROS_INFO_STREAM("Number of edges: "<<graph_msg.edges.size());
    ROS_INFO_STREAM("Final retrived graph information");
    ROS_INFO_STREAM("Number of vertices: "<<vl.size());
    ROS_INFO_STREAM("Number of edges: "<<el.size());

    for(const Graph::Vertex &v: vl)
    {
        if(!addInteractiveVertexMarker(v)) 
            return;
    }

    for(Graph::Edge e: el) //Some of the parameters of the edges are modified
    {
        if(!addInteractiveEdgeMarker(e))
            return;
    }

    m_graph_marker_server.applyChanges();
}

void GraphVizCtrl::initVertexMenu()
{
    interactive_markers::MenuHandler::EntryHandle graph_goal_sub_menu, nav_goal_sub_menu;

    graph_goal_sub_menu = m_vert_menu_handler.insert("Graph navigation goal");

    m_vert_menu_handler.insert(graph_goal_sub_menu, "Send graph navigation goal to this vertex",
                               boost::bind(&GraphVizCtrl::sendGraphNavigationGoalMenuCallback, this, _1));

    m_vert_menu_handler.insert(graph_goal_sub_menu, "Cancel all navigation goals",
                               boost::bind(&GraphVizCtrl::cancelGraphNavigationGoalMenuCallback, this, _1));

    nav_goal_sub_menu = m_vert_menu_handler.insert("Navigation goal");

    m_vert_menu_handler.insert(nav_goal_sub_menu, "Send navigation goal to this vertex", 
                               boost::bind(&GraphVizCtrl::sendNavigationGoalMenuCallback, this, _1));

    m_vert_menu_handler.insert(nav_goal_sub_menu, "Cancel all navigation goals",
                               boost::bind(&GraphVizCtrl::cancelNavigationGoalsMenuCallback, this, _1));

    m_vert_menu_handler.insert("Cancel all goals", boost::bind(&GraphVizCtrl::cancelAllGoalsMenuCallback, this, _1));
}

void GraphVizCtrl::initEdgeMenu()
{
    m_edge_menu_handler.insert("Cancel all goals", boost::bind(&GraphVizCtrl::cancelAllGoalsMenuCallback, this, _1));
}

void GraphVizCtrl::sendGraphNavigationGoalMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM("Sending graph navigation goal to vertex with name: "<<feedback->marker_name);
    sendGraphNavigationGoal(feedback->marker_name);
}

void GraphVizCtrl::cancelGraphNavigationGoalMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM("Cancelling all graph navigation goals");
    cancelGraphNavigationGoal();
}

void GraphVizCtrl::sendNavigationGoalMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM("Sending a navigation goal to vertex with name: "<<feedback->marker_name);
    sendNavigationGoal(feedback->pose);
}

void GraphVizCtrl::cancelNavigationGoalsMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM("Cancelling all navigation goals");
    cancelNavigationGoal();
}

void GraphVizCtrl::cancelAllGoalsMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM("Cancelling all goals");
    cancelGraphNavigationGoal();
    cancelNavigationGoal();
}

bool GraphVizCtrl::addInteractiveVertexMarker(const Graph::Vertex &v)
{
    if(m_vertex_map.find(v.vertex_name) != m_vertex_map.end())
    {
        ROS_ERROR_STREAM("Multiple copies of vertex with name "<<v.vertex_name);
        return false;
    }

    m_vertex_map.emplace(v.vertex_name, std::make_shared<Graph::Vertex>(v));

    auto vertex_int_marker = Graph::Visuals::makeInteractiveVertexMarker(v.vertex_id, v.vertex_pose);
    
    visualization_msgs::Marker vertex_marker; 
    
    if(!v.ignore_orientation) vertex_marker = Graph::Visuals::makeVertexOrientationArrowMarker();
    else vertex_marker = Graph::Visuals::makeVertexSphereMarker();
    
    Graph::Visuals::makeButtonControl(vertex_int_marker, vertex_marker);

    m_graph_marker_server.insert(vertex_int_marker);
    m_vert_menu_handler.apply(m_graph_marker_server, vertex_int_marker.name);

    return true;
}

bool GraphVizCtrl::addInteractiveEdgeMarker(Graph::Edge &e)
{
    if(m_edge_map.find(e.edge_name) != m_edge_map.end())
    {
        ROS_ERROR_STREAM("Multiple copies of edge with name "<<e.edge_name);
        return false;
    }

    m_edge_map.emplace(e.edge_name, std::make_shared<Graph::Edge>(e));

    std::string start_vertex_name = Graph::Visuals::generateVertexMarkerName(e.edge_vertex_id.first);
    std::string end_vertex_name = Graph::Visuals::generateVertexMarkerName(e.edge_vertex_id.second);

    //TODO: This is similar to the checks used in directed graph class
    if(m_vertex_map.find(start_vertex_name) == m_vertex_map.end() && m_vertex_map.find(end_vertex_name) == m_vertex_map.end())
    {
        ROS_ERROR_STREAM("Could not find both the start vertex "<<start_vertex_name<<" and end vertex "<<end_vertex_name);
        return false;
    }

    if(m_vertex_map.find(start_vertex_name) == m_vertex_map.end())
    {
        ROS_ERROR_STREAM("Could not find the start vertex "<<start_vertex_name);
        return false;
    }

    if(m_vertex_map.find(end_vertex_name) == m_vertex_map.end())
    {
        ROS_ERROR_STREAM("Could not find the end vertex "<<end_vertex_name);
        return false;
    }

    e.edge_vertices_name.first = start_vertex_name;
    e.edge_vertices_name.second = end_vertex_name;

    Graph::VertexPtr v1_ptr = m_vertex_map.at(e.edge_vertices_name.first);
    Graph::VertexPtr v2_ptr = m_vertex_map.at(e.edge_vertices_name.second);

    if(!v1_ptr)
    {
        ROS_ERROR_STREAM("Could not retrived start vertex data");
        return false;
    }

    if(!v2_ptr)
    {
        ROS_ERROR_STREAM("Could not retrived end vertex data");
        return false;
    }

    //TODO: Instead of computing the edge each time, it would be better to use the poses parameter in graph_msgs/Edge. All of the above steps can be ignored

    double length = 0.0f;
    std::vector<geometry_msgs::Pose> out_poses;

    switch(e.edge_type)
    {
        case graph_msgs::Edge::STRAIGHT_LINE:
        {
            Graph::Curves::calculateEquiDistantPosesLine({v1_ptr->vertex_pose, v2_ptr->vertex_pose}, out_poses, length, 0.1);
            break;
        }

        case graph_msgs::Edge::CUBIC_BEZIER:
        {
            if(e.control_poses.size() != 2)
            {
                ROS_ERROR_STREAM("The control point count for cubic Bezier curve is less that 2");
                return false;
            }

            Graph::Curves::calculateEquiDistantPosesBezier({v1_ptr->vertex_pose, e.control_poses.front(), e.control_poses.back(), v2_ptr->vertex_pose},
                                                           out_poses, length, m_pose_spacing, m_resolution);
            break;
        }

        default:
        {
            ROS_ERROR_STREAM("Could not create edge "<<e.edge_name<<" as the edge type is unknown");
            return false;
        }
    }

    //TODO: Depending how edge directions are represented internally the following code will need to change
    geometry_msgs::Pose fwd_dir_mrkr_pose, rev_dir_mrkr_pose;
    Graph::Visuals::calculateEdgeDirectionMarkerPose(out_poses, m_pose_spacing, fwd_dir_mrkr_pose, rev_dir_mrkr_pose);

    //Forward direction marker is always added for now
    e.forward_direction_name = Graph::Visuals::generateEdgeDirectionMarkerName(e.edge_name, Graph::Visuals::FORWARD_DIRECTION_ARROW);
    addInteractiveDirectionMarker(e.forward_direction_name, fwd_dir_mrkr_pose, Graph::Visuals::FORWARD_DIRECTION_ARROW);

    if(e.is_bidirectional) //Reverse direction marker is also added
    {
        e.reverse_direction_name = Graph::Visuals::generateEdgeDirectionMarkerName(e.edge_name, Graph::Visuals::REVERSE_DIRECTION_ARROW);
        addInteractiveDirectionMarker(e.reverse_direction_name, rev_dir_mrkr_pose, Graph::Visuals::REVERSE_DIRECTION_ARROW);
    }

    if(e.use_independent_orientation)
    {
        e.control_orientation_name = Graph::Visuals::generateControlOrientationMarkerName(e.edge_name);
        addInteractiveOrientationMarker(e.control_orientation_name, e.control_orientation_pose);
    }

    //Edge interactive marker
    auto edge_int_marker = Graph::Visuals::makeInteractiveEdgeMarker(e.edge_name);
    auto edge_marker = Graph::Visuals::makeEdgePointMarker(out_poses);
    Graph::Visuals::makeButtonControl(edge_int_marker, edge_marker);

    m_graph_marker_server.insert(edge_int_marker);
    m_edge_menu_handler.apply(m_graph_marker_server, edge_int_marker.name);

    return true;
}

void GraphVizCtrl::addInteractiveDirectionMarker(const std::string &direction_name, const geometry_msgs::Pose &pose, int16_t marker_type)
{
    auto direction_int_marker = Graph::Visuals::makeInteractiveEdgeDirectionMarkers(direction_name, pose, marker_type);
    auto direction_marker = Graph::Visuals::makeEdgeDirectionArrowMarker(marker_type);
    Graph::Visuals::makeButtonControl(direction_int_marker, direction_marker);

    m_graph_marker_server.insert(direction_int_marker);
}

void GraphVizCtrl::addInteractiveOrientationMarker(const std::string &orientation_name, const geometry_msgs::Pose &pose)
{
    auto orientation_int_marker = Graph::Visuals::makeInteractiveControlOrientMarker(orientation_name, pose);
    auto orientation_marker = Graph::Visuals::makeControlOrientArrowMarker();
    Graph::Visuals::makeButtonControl(orientation_int_marker, orientation_marker);

    m_graph_marker_server.insert(orientation_int_marker);
}

void GraphVizCtrl::sendGraphNavigationGoal(const std::string &vertex_name)
{
    if(!m_gcpl.isGraphControlPluginLoaded())
    {
        ROS_WARN_STREAM("Cannot send graph goal as no graph control plugin was loaded");
        return;   
    }

    if(!m_graph_control_instance)
    {
        ROS_ERROR_STREAM("The graph control instance is not valid");
        return;
    }

    if(m_vertex_map.find(vertex_name) == m_vertex_map.end())
    {
        ROS_ERROR_STREAM("Graph goal cannot be send because vertex "<<vertex_name<<" does not exist");
        return;
    }

    Graph::VertexPtr v_ptr = m_vertex_map.at(vertex_name);

    m_graph_control_instance->sendGoal(vertex_name, v_ptr->vertex_id, v_ptr->vertex_pose);
}
    
void GraphVizCtrl::cancelGraphNavigationGoal()
{
    if(!m_gcpl.isGraphControlPluginLoaded())
    {
        ROS_WARN_STREAM("Cannot cancel graph goal as no graph control plugin was loaded");
        return;
    }

    if(!m_graph_control_instance)
    {
        ROS_ERROR_STREAM("The graph control instance is not valid");
        return;
    }

    m_graph_control_instance->cancelGoal();
}

void GraphVizCtrl::sendNavigationGoal(const geometry_msgs::Pose &goal_pose)
{
    if(!move_base_ac_.isServerConnected())
    {
        ROS_WARN_STREAM("Cannot send navigation goal as the server is not connected");
        return;
    }

    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose.header.frame_id = "map";
    mb_goal.target_pose.header.seq++;
    mb_goal.target_pose.header.stamp = ros::Time::now();
    mb_goal.target_pose.pose = goal_pose;

    move_base_ac_.sendGoal(mb_goal); //No callbacks required
}
    
void GraphVizCtrl::cancelNavigationGoal()
{
    if(!move_base_ac_.isServerConnected())
    {
        ROS_WARN_STREAM("Cannot cancel goals as the server is not connected");
        return;
    }

    move_base_ac_.cancelAllGoals();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "graph_visual_control");

    GraphVizCtrl gvc;
    ros::spin();

    return EXIT_SUCCESS;
}