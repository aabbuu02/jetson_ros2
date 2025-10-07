#include <math.h>
#include <graph_creator/graph_creator.hpp>


GraphCreator::GraphCreator():
    m_graph_marker_server("graph_marker_server","gims", true)
{
    //Publishers
    m_graph_pub = m_nh.advertise<graph_msgs::Graph>("graph", 1, true);
    m_cur_vertex_pub = m_nh.advertise<graph_msgs::Vertex>("graph_creator/current/vertex", 10, true);
    m_cur_edge_pub = m_nh.advertise<graph_msgs::Edge>("graph_creator/current/edge", 10, true);
    m_cur_gc_state_pub = m_nh.advertise<graph_creator_msgs::GraphCreatorState>("graph_creator/current_state", 10, true);
    m_fwd_pose_arr_pub = m_nh.advertise<geometry_msgs::PoseArray>("graph_creator/forward_poses", 10, true);
    m_bck_pose_arr_pub = m_nh.advertise<geometry_msgs::PoseArray>("graph_creator/backward_poses", 10, true);

    //Subscribers
    m_vertex_create_sub = m_nh.subscribe("add_vertex_pose", 10, &GraphCreator::vertexCreateCallback, this);
    m_robot_pose_sub = m_nh.subscribe("robot_pose", 10, &GraphCreator::robotPoseCallback, this);
    m_vertex_updates_sub = m_nh.subscribe("graph_creator/updates/vertex", 10, &GraphCreator::vertexUpdatesCallback, this);
    m_edge_updates_sub = m_nh.subscribe("graph_creator/updates/edge", 10, &GraphCreator::edgeUpdatesCallback, this);
    m_selected_graph_elem_sub = m_nh.subscribe("graph_creator/selected_graph_elements", 10, &GraphCreator::selectedGraphElementCallback, this);

    //Service servers
    m_load_graph_srv = m_nh.advertiseService("graph_creator/load_graph", &GraphCreator::loadGraphFileServiceCallback, this);
    m_save_graph_srv = m_nh.advertiseService("graph_creator/save_graph", &GraphCreator::saveGraphFileServiceCallback, this);
    m_set_gc_mode_srv = m_nh.advertiseService("graph_creator/set_mode", &GraphCreator::setGraphCreatorModeCallback, this);
    m_set_ec_type_srv = m_nh.advertiseService("graph_creator/set_edge_creation_type", &GraphCreator::setEdgeCreationTypeCallback, this);
    m_set_ed_type_srv = m_nh.advertiseService("graph_creator/set_edge_direction_type", &GraphCreator::setEdgeDirectionCreationTypeCallback, this);
    m_set_verts_locked_srv = m_nh.advertiseService("graph_creator/set_vertices_locked", &GraphCreator::setVerticesLockedCallback, this);
    m_set_edges_locked_srv = m_nh.advertiseService("graph_creator/set_edges_locked", &GraphCreator::setEdgesLockedCallback, this);
    m_get_gc_state_srv = m_nh.advertiseService("graph_creator/get_state", &GraphCreator::getGraphCreatorStateCallback, this);
    m_gen_graph_srv = m_nh.advertiseService("graph_creator/generate_graph", &GraphCreator::generateGraphCallback, this);
    m_set_snap_pose_srv = m_nh.advertiseService("graph_creator/snap_pose_config", &GraphCreator::setSnapPoseConfigCallback, this);

    //Menu handlers
    initVertexMenu();
    initEdgeMenu();

    //Robit pose for graph creator
    initializeRobotPose();

    //Graph creator state
    publishCurrentGraphCreatorState();
}

GraphCreator::~GraphCreator()
{
    
}

void GraphCreator::initializeRobotPose()
{
    //Initial pose of the robot w.r.t the map
    geometry_msgs::Pose initial_pose;
    initial_pose.orientation.w = 1.0f;
    mapBaseLinkTfBroadcast(initial_pose);
}

void GraphCreator::clearGraph()
{
    //Trash the internal representation of the graph
    m_graph.clear();

    //Clear the interactive marker server
    m_graph_marker_server.clear();
    m_graph_marker_server.applyChanges();

    //Reinitialize menu handlers
    m_vert_menu_handler = interactive_markers::MenuHandler();
    m_edge_menu_handler = interactive_markers::MenuHandler();
    initVertexMenu();
    initEdgeMenu();

    //Reset the graph creator state
    reset();

    //Reset the pose of the robot footprint to origin
    initializeRobotPose();
}

void GraphCreator::reset()
{
    //Clear the poses that where published before
    m_fwd_pose_arr_msg.poses.clear();
    m_bck_pose_arr_msg.poses.clear();
    publishAllPosesInGraph();

    //Reset graph and graph element creation modes
    m_current_graph_mode = CREATE_VERTICES;
    
    //Vertex variables
    m_current_vertex_type = graph_msgs::Vertex::NORMAL;
    m_vertex_ignore_orientation = false;
    m_is_vertices_locked = false;
    m_vertex_id_counter = 0;
    m_prev_selected_vertex = "";

    //Edge variables
    m_current_curve_type = STRAIGHT_LINE;
    m_edge_is_bidirectional = false;
    m_is_edges_locked = false;
    m_control_id_counter = 0;
    m_prev_selected_edge = "";
    m_selected_edge_vertices.clear();

    //Snap variables
    m_enable_vertex_snap = false;
    m_enable_edge_snap = false;
    m_enable_position_snap = false;
    m_enable_orientation_snap = false;
    m_position_snap_grid_size = 1.0f;
    m_orientation_snap_angle = M_PI/2;

    //Reset graph selections
    resetEdgesAndVerticesSelections();
}

void GraphCreator::vertexCreateCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg)
{
    if(m_current_graph_mode != CREATE_VERTICES) 
    {
        ROS_WARN_STREAM("Not in create vertices mode");
        return;
    }

    m_vertex_id_counter++;

    geometry_msgs::Pose req_pose = pose_msg->pose;
    applyVertexPositionSnap(pose_msg->pose.position, req_pose.position);
    applyVertexOrientationSnap(pose_msg->pose.orientation, req_pose.orientation);

    addVertexWithMarker(m_vertex_id_counter, req_pose);
    m_graph_marker_server.applyChanges();
    
    //Move the base link to the current vertex pose in the map frame
    mapBaseLinkTfBroadcast(pose_msg->pose);
}

void GraphCreator::robotPoseCallback(const geometry_msgs::PoseConstPtr &pose_msg)
{
    ROS_INFO_STREAM_ONCE("Graph creator received robot pose");

    m_robot_pose_msg = *pose_msg;
    m_prev_robot_pose_time = ros::Time::now();
    m_is_robot_pose_received = true;
}

void GraphCreator::mapBaseLinkTfBroadcast(const geometry_msgs::Pose &pose_msg)
{
    if(!m_enable_base_link_tf_br) return;
    
    m_map_base_link_tf_msg.header.stamp = ros::Time::now();
    m_map_base_link_tf_msg.header.frame_id = "map";
    m_map_base_link_tf_msg.header.seq++;

    m_map_base_link_tf_msg.child_frame_id = "base_link";

    m_map_base_link_tf_msg.transform.translation.x = pose_msg.position.x;
    m_map_base_link_tf_msg.transform.translation.y = pose_msg.position.y;
    m_map_base_link_tf_msg.transform.translation.z = pose_msg.position.z;

    m_map_base_link_tf_msg.transform.rotation = pose_msg.orientation;

    m_map_base_link_br.sendTransform(m_map_base_link_tf_msg);
}

void GraphCreator::addEdgeWithMarkers(const std::vector<std::string> &selected_vertices, uint8_t edge_type)
{
    Graph::VertexPtr v0_ptr, v1_ptr; //Start and end vertex pointers
    v0_ptr = m_graph.getVertexData(selected_vertices.at(0));
    v1_ptr = m_graph.getVertexData(selected_vertices.at(1));

    std::string edge_name = Graph::Visuals::generateEdgeMarkerName(v0_ptr->vertex_id, v1_ptr->vertex_id);

    if(m_graph.isEdge(edge_name))
    {
        ROS_WARN_STREAM("Edge "<<edge_name<<" already exists");
        return;
    }

    Graph::Edge e;
    e.edge_vertex_id.first = v0_ptr->vertex_id;
    e.edge_vertex_id.second = v1_ptr->vertex_id;
    e.edge_name = edge_name;
    e.edge_alias = ""; //During creation edge alias will always be empty
    e.edge_cost_factor = 1.0f;
    e.edge_type = edge_type;
    e.edge_direction_type = FORWARD;
    e.is_bidirectional = m_edge_is_bidirectional;
    e.use_independent_orientation = false;
    e.edge_vertices_name.first = m_selected_edge_vertices.at(0);
    e.edge_vertices_name.second = m_selected_edge_vertices.at(1);
    
    //Forward poses
    e.forward_edge_poses.header.frame_id = "map";
    e.forward_edge_poses.header.stamp = ros::Time::now();
    e.forward_edge_poses.header.seq++;

    //Backward poses
    e.backward_edge_poses.header.frame_id = "map";
    e.backward_edge_poses.header.stamp = ros::Time::now();
    e.backward_edge_poses.header.seq++;

    createEdgeAndMarkers({v0_ptr, v1_ptr}, e);
    
    publishAllPosesInGraph();
}

void GraphCreator::deleteEdgeWithMarkers(const std::string &edge_name)
{
    Graph::EdgePtr e_ptr = m_graph.getEdgeData(edge_name);

    if(!e_ptr)
    {
        ROS_ERROR_STREAM("Edge data could not be retreived for deletion");
        return;
    }

    if(m_graph.isEdge(edge_name))
    {
        Graph::MiscDepSet mds;
        m_graph.getMiscDependency(edge_name, mds);
        for(Graph::MiscDepSet::iterator it_m = mds.begin(); it_m != mds.end(); ++it_m)
        {
            m_graph.deleteMiscDependencyKey(*it_m);
            m_graph_marker_server.erase(*it_m);

            Graph::VertexDepPair vdp;
            m_graph.getEdgeToVertexDependency(edge_name, vdp);

            m_graph.deleteMiscDependencyValue(vdp.first, *it_m);
            m_graph.deleteMiscDependencyValue(vdp.second, *it_m);
        }
    }
    
    if(e_ptr->use_independent_orientation) m_graph_marker_server.erase(e_ptr->control_orientation_name);

    m_graph_marker_server.erase(e_ptr->forward_direction_name);
    if(e_ptr->is_bidirectional) m_graph_marker_server.erase(e_ptr->reverse_direction_name);

    m_graph_marker_server.erase(edge_name);
    m_graph_marker_server.applyChanges();

    m_graph.deleteEdge(edge_name);

    m_graph.printGraph();

    publishAllPosesInGraph();
}

void GraphCreator::deleteEdgesWithMarkers(const std::unordered_set<std::string> &edge_names)
{
    for(const auto &edge_name: edge_names)
        deleteEdgeWithMarkers(edge_name);
}

void GraphCreator::changeEdgeTypeWithMarkers(const std::string &edge_name, uint8_t edge_type)
{
    Graph::EdgePtr e_ptr = m_graph.getEdgeData(edge_name);

    if(!e_ptr)
    {
        ROS_ERROR_STREAM("Edge "<<edge_name<<" was not found");
        return;
    }

    if(e_ptr->edge_type == edge_type)
    {
        ROS_WARN_STREAM("Selected edge is already of the same curve type");
        return;
    }

    Graph::Edge e = *e_ptr; //Copy of the original edge 
    e.edge_type = edge_type; //Updating the edge type

    e.control_poses.clear();
    e.control_poses_name.clear();
    
    e.forward_edge_poses.header.stamp = ros::Time::now();
    e.forward_edge_poses.header.seq++;
    e.forward_edge_poses.poses.clear();

    e.backward_edge_poses.header.stamp = ros::Time::now();
    e.backward_edge_poses.header.seq++;
    e.backward_edge_poses.poses.clear();

    Graph::VertexPtr v0_ptr, v1_ptr; //Start and end vertex pointers
    v0_ptr = m_graph.getVertexData(e.edge_vertices_name.first);
    v1_ptr = m_graph.getVertexData(e.edge_vertices_name.second);
    
    deleteEdgeWithMarkers(edge_name); //Delete the edge from graph and erase the markers

    createEdgeAndMarkers({v0_ptr, v1_ptr}, e);

    publishAllPosesInGraph();
}

void GraphCreator::changeEdgeTypesWithMarkers(const std::unordered_set<std::string> &edge_names, uint8_t edge_type)
{
    for(const auto &edge_name: edge_names)
    {
        changeEdgeTypeWithMarkers(edge_name, edge_type);
        clearControlPoseMarkersForEdge(edge_name);
    }
}

void GraphCreator::changeEdgeDirectionWithMarkers(const std::string &edge_name, bool is_bidirectional)
{
    Graph::EdgePtr e_ptr = m_graph.getEdgeData(edge_name);

    if(!e_ptr)
    {
        ROS_ERROR_STREAM("The edge data could not be retreived for changing edge directionality");
        return;
    }

    if(e_ptr->is_bidirectional == is_bidirectional)
    {
        if(is_bidirectional) ROS_WARN_STREAM("The edge is already set to bidirectional");
        else ROS_WARN_STREAM("The edge is already set to unidirectional");
        return;
    }

    if(!is_bidirectional && e_ptr->is_bidirectional)
    {
        m_graph_marker_server.erase(e_ptr->reverse_direction_name);
        e_ptr->reverse_direction_name="";
    }
    else if(is_bidirectional && !e_ptr->is_bidirectional)
    {
        geometry_msgs::Pose fwd_dir_mrkr_pose, rev_dir_mrkr_pose;
        Graph::Visuals::calculateEdgeDirectionMarkerPose(e_ptr->forward_edge_poses.poses, m_pose_spacing, fwd_dir_mrkr_pose, rev_dir_mrkr_pose);

        //Only the reverse marker name needs to be update (Forward direction marker name already exists during edge creation)
        //TODO: The edge should be able to set for only reverse direction as well. This would mean this current logic would not be applicable
        e_ptr->reverse_direction_name = Graph::Visuals::generateEdgeDirectionMarkerName(edge_name, Graph::Visuals::REVERSE_DIRECTION_ARROW);
        makeInteractiveEdgeDirectionMarkers(e_ptr->reverse_direction_name, rev_dir_mrkr_pose, Graph::Visuals::REVERSE_DIRECTION_ARROW);
    }

    e_ptr->is_bidirectional = is_bidirectional;
}

void GraphCreator::changeEdgeDirectionsWithMarkers(const std::unordered_set<std::string> &edge_names, bool is_bidirectional)
{
    for(const auto &edge_name: edge_names)
        changeEdgeDirectionWithMarkers(edge_name, is_bidirectional);
}

bool GraphCreator::createEdgeWithMarker(Graph::Edge &edge)
{
    if(m_graph.isEdge(edge.edge_name))
    {
        ROS_FATAL_STREAM("Multiple copies of edge "<<edge.edge_name);
        return false;
    }

    Graph::VertexPtr v1_ptr = m_graph.getVertexData(edge.edge_vertices_name.first);
    Graph::VertexPtr v2_ptr = m_graph.getVertexData(edge.edge_vertices_name.second);

    std::vector<geometry_msgs::Pose> out_poses;

    switch(edge.edge_type)
    {
        case STRAIGHT_LINE:
        {
            Graph::Curves::calculateEquiDistantPosesLine({v1_ptr->vertex_pose, v2_ptr->vertex_pose}, out_poses, 
                                                         edge.edge_length, m_pose_spacing);
            
            edge.forward_edge_poses.poses = out_poses;

            break;   
        }

        case CUBIC_BEZIER:
        {
            std::vector<geometry_msgs::Pose> in_poses = {v1_ptr->vertex_pose, 
                                                         edge.control_poses.front(), 
                                                         edge.control_poses.back(), 
                                                         v2_ptr->vertex_pose};

            Graph::Curves::calculateEquiDistantPosesBezier(in_poses, out_poses, edge.edge_length, m_pose_spacing, m_resolution);
            edge.forward_edge_poses.poses = out_poses;

            for(int i = 0; i < edge.control_poses.size(); ++i)
            {
                std::string ctrl_marker_name = Graph::Visuals::generateControlPoseMarkerName(m_control_id_counter, edge.edge_name);
                edge.control_poses_name.push_back(ctrl_marker_name);

                //Interactive marker for the control poses is not created at the is point
                m_graph.addMiscDependency(edge.edge_name, ctrl_marker_name, true);
                if(i == 0) m_graph.addMiscDependency(edge.edge_vertices_name.first, ctrl_marker_name);
                else m_graph.addMiscDependency(edge.edge_vertices_name.second, ctrl_marker_name);

                m_control_id_counter++;
            }

            break;
        }

        default:
        {
            ROS_FATAL_STREAM("An invalid edge type is being used");
            return false;
        }
    }

    geometry_msgs::Pose fwd_dir_mrkr_pose, rev_dir_mrkr_pose;

    Graph::Visuals::calculateEdgeDirectionMarkerPose(out_poses, m_pose_spacing, fwd_dir_mrkr_pose, rev_dir_mrkr_pose);

    //Forward direction marker is always added for now
    edge.forward_direction_name = Graph::Visuals::generateEdgeDirectionMarkerName(edge.edge_name, Graph::Visuals::FORWARD_DIRECTION_ARROW);
    makeInteractiveEdgeDirectionMarkers(edge.forward_direction_name, fwd_dir_mrkr_pose, Graph::Visuals::FORWARD_DIRECTION_ARROW);

    if(edge.is_bidirectional) //Reverse direction marker is also added
    {
        edge.reverse_direction_name = Graph::Visuals::generateEdgeDirectionMarkerName(edge.edge_name, Graph::Visuals::REVERSE_DIRECTION_ARROW);
        makeInteractiveEdgeDirectionMarkers(edge.reverse_direction_name, rev_dir_mrkr_pose, Graph::Visuals::REVERSE_DIRECTION_ARROW);
    }

    if(edge.use_independent_orientation)
    {
        edge.control_orientation_name = Graph::Visuals::generateControlOrientationMarkerName(edge.edge_name); //For each edge only one orientation control marker
        m_graph.addMiscDependency(edge.control_orientation_name, edge.edge_name); //Not mutual as the orientation control name is in edge data

        for(int i = 0; i < edge.forward_edge_poses.poses.size(); ++i)
        {
            edge.forward_edge_poses.poses.at(i).orientation = edge.control_orientation_pose.orientation;
        }
    }

    m_graph.addEdge(edge.edge_name, edge.edge_vertices_name.first, edge.edge_vertices_name.second, edge);
    makeInteractiveEdgeMarker(edge.edge_name, out_poses);
    m_prev_selected_edge = edge.edge_name;
    m_graph.printGraph();

    return true;
}

inline void GraphCreator::createEdgeAndMarkers(std::vector<Graph::VertexPtr> vertices, Graph::Edge &edge)
{
    std::vector<geometry_msgs::Pose> out_poses;

    switch (edge.edge_type)
    {
        case STRAIGHT_LINE:
        {
            Graph::Curves::calculateEquiDistantPosesLine({vertices.front()->vertex_pose, vertices.back()->vertex_pose}, out_poses, 
                                                         edge.edge_length, m_pose_spacing);
            
            edge.forward_edge_poses.poses = out_poses;

            break;
        }

        case CUBIC_BEZIER:
        {
            std::vector<geometry_msgs::Pose> in_poses;
            Graph::Curves::calculateControlPosesForBezier({vertices.front()->vertex_pose, vertices.back()->vertex_pose}, in_poses, edge.control_poses);
            Graph::Curves::calculateEquiDistantPosesBezier(in_poses, out_poses, edge.edge_length, m_pose_spacing, m_resolution);
            edge.forward_edge_poses.poses = out_poses;

            for(size_t i = 0; i < edge.control_poses.size(); ++i)
            {
                std::string ctrl_marker_name = Graph::Visuals::generateControlPoseMarkerName(m_control_id_counter, edge.edge_name);
                edge.control_poses_name.push_back(ctrl_marker_name);

                makeInteractiveControlPoseMarker(ctrl_marker_name, edge.control_poses.at(i));

                //Add miscellaneous dependencies
                m_graph.addMiscDependency(edge.edge_name, ctrl_marker_name, true);
                if(i == 0) m_graph.addMiscDependency(edge.edge_vertices_name.first, ctrl_marker_name);
                else m_graph.addMiscDependency(edge.edge_vertices_name.second, ctrl_marker_name); 

                m_control_id_counter++;
            }

            break;
        }
        
        default:
        {
            ROS_ERROR_STREAM("Edge type is not valid");
            break;
        }
    }

    //TODO: This currently only represents whether the edge is bidirectional but should be able to show only reverse direction paths (Not implemented in graph)
    geometry_msgs::Pose fwd_dir_mrkr_pose, rev_dir_mrkr_pose;
    
    Graph::Visuals::calculateEdgeDirectionMarkerPose(out_poses, m_pose_spacing, fwd_dir_mrkr_pose, rev_dir_mrkr_pose);

    //Forward direction marker is always added for now
    edge.forward_direction_name = Graph::Visuals::generateEdgeDirectionMarkerName(edge.edge_name, Graph::Visuals::FORWARD_DIRECTION_ARROW);
    makeInteractiveEdgeDirectionMarkers(edge.forward_direction_name, fwd_dir_mrkr_pose, Graph::Visuals::FORWARD_DIRECTION_ARROW);

    if(edge.is_bidirectional) //Reverse direction marker is also added
    {
        edge.reverse_direction_name = Graph::Visuals::generateEdgeDirectionMarkerName(edge.edge_name, Graph::Visuals::REVERSE_DIRECTION_ARROW);
        makeInteractiveEdgeDirectionMarkers(edge.reverse_direction_name, rev_dir_mrkr_pose, Graph::Visuals::REVERSE_DIRECTION_ARROW);
    }

    makeInteractiveEdgeMarker(edge.edge_name, out_poses);
    m_graph.addEdge(edge.edge_name, vertices.front()->vertex_name, vertices.back()->vertex_name, edge);
    m_prev_selected_edge = edge.edge_name;
    m_graph.printGraph();

    publishCurrentGraphEdge(edge.edge_name);
}

void GraphCreator::makeInteractiveEdgeMarker(const std::string &edge_name, const std::vector<geometry_msgs::Pose> &poses)
{
    auto edge_int_marker = Graph::Visuals::makeInteractiveEdgeMarker(edge_name);
    Graph::Visuals::makeEdgePointControls(edge_int_marker, poses);

    m_graph_marker_server.insert(edge_int_marker);
    m_edge_menu_handler.apply(m_graph_marker_server, edge_int_marker.name);
    m_graph_marker_server.setCallback(edge_int_marker.name, boost::bind(&GraphCreator::processInteractiveMarkerFeedback, this, _1));
    m_graph_marker_server.applyChanges();
}

void GraphCreator::makeInteractiveControlPoseMarker(const std::string &control_pose_name, const geometry_msgs::Pose &pose)
{
    auto control_pose_int_marker = Graph::Visuals::makeInteractiveControlPoseMarker(control_pose_name, pose);
    Graph::Visuals::makeControlPoseControls(control_pose_int_marker);

    m_graph_marker_server.insert(control_pose_int_marker);
    m_graph_marker_server.setCallback(control_pose_int_marker.name, boost::bind(&GraphCreator::processInteractiveMarkerFeedback, this, _1));
    m_graph_marker_server.setCallback(control_pose_int_marker.name, 
                                      boost::bind(&GraphCreator::updateInteractiveMarkerPoses, this, _1), 
                                      visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
}

void GraphCreator::clearControlPoseMarkersForEdge(const std::string &edge_name)
{
    Graph::MiscDepSet mds;
    m_graph.getMiscDependency(edge_name, mds);
    
    for(Graph::MiscDepSet::iterator it_m = mds.begin(); it_m != mds.end(); ++it_m)
    {
        m_graph_marker_server.erase(*it_m);
    }
}

void GraphCreator::readdControlPoseMarkersForEdge(const std::string &edge_name)
{
    Graph::EdgePtr e_ptr = m_graph.getEdgeData(edge_name);

    if(!e_ptr)
    {
        ROS_ERROR_STREAM("Unknown graph element "<<edge_name);
        return;
    }
    
    if(e_ptr->control_poses.size() != e_ptr->control_poses_name.size()) 
    {
        ROS_WARN_STREAM("Control poses are not valid");
        return;
    }

    for(size_t i = 0; i < e_ptr->control_poses.size(); ++i)
    {
        makeInteractiveControlPoseMarker(e_ptr->control_poses_name.at(i), e_ptr->control_poses.at(i));
    }
}

void GraphCreator::makeInteractiveControlOrientMarker(const std::string &control_orient_name, const geometry_msgs::Pose &pose)
{
    auto orientation_control_int_marker = Graph::Visuals::makeInteractiveControlOrientMarker(control_orient_name, pose);
    Graph::Visuals::makeControlOrientControls(orientation_control_int_marker);

    m_graph_marker_server.insert(orientation_control_int_marker);
    m_graph_marker_server.setCallback(orientation_control_int_marker.name, boost::bind(&GraphCreator::processInteractiveMarkerFeedback, this, _1));
    m_graph_marker_server.setCallback(orientation_control_int_marker.name, 
                                      boost::bind(&GraphCreator::updateInteractiveMarkerPoses, this, _1), 
                                      visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
}

void GraphCreator::addControlOrientMarkerToEdge(const std::string &edge_name)
{
    Graph::EdgePtr e_ptr = m_graph.getEdgeData(edge_name);

    if(!e_ptr)
    {
        ROS_WARN_STREAM("Cannot retreive data for edge "<<edge_name);
        return;
    }

    //Calculate the pose at which the edge 
    geometry_msgs::Pose ctrl_orient_pose;
    Graph::Curves::calculateControlOrientForCurves(e_ptr->forward_edge_poses.poses, ctrl_orient_pose);

    std::string ctrl_orient_name = Graph::Visuals::generateControlOrientationMarkerName(e_ptr->edge_name); //For each edge only one orientation control marker
    
    //Update edge information
    e_ptr->use_independent_orientation = true;
    e_ptr->control_orientation_name = ctrl_orient_name;
    e_ptr->control_orientation_pose = ctrl_orient_pose;

    m_graph.addMiscDependency(ctrl_orient_name, edge_name); //Not mutual as the orientation control name is in edge data

    makeInteractiveControlOrientMarker(ctrl_orient_name, ctrl_orient_pose);

    for(size_t i = 0; i < e_ptr->forward_edge_poses.poses.size(); ++i)
    {
        e_ptr->forward_edge_poses.poses.at(i).orientation = ctrl_orient_pose.orientation;
    }

    publishAllPosesInGraph();
}

void GraphCreator::addControlOrientMarkerToEdges(const std::unordered_set<std::string> edge_names)
{
    for(const auto &edge_name: edge_names)
    {
        addControlOrientMarkerToEdge(edge_name);
        clearControlOrientMarkersForEdge(edge_name);
    }
}

void GraphCreator::deleteControlOrientMarkerFromEdge(const std::string &edge_name)
{
    Graph::EdgePtr e_ptr = m_graph.getEdgeData(edge_name);

    if(!e_ptr)
    {
        ROS_WARN_STREAM("Cannot retreive data for edge "<<edge_name);
        return;
    }

    if(e_ptr->use_independent_orientation)
    {
        m_graph.deleteMiscDependencyKey(e_ptr->control_orientation_name);
        m_graph_marker_server.erase(e_ptr->control_orientation_name);
    }

    e_ptr->use_independent_orientation = false;
    e_ptr->control_orientation_name = "";
    e_ptr->control_orientation_pose = geometry_msgs::Pose();

    Graph::VertexPtr v0_ptr = m_graph.getVertexData(e_ptr->edge_vertices_name.first);
    Graph::VertexPtr v1_ptr = m_graph.getVertexData(e_ptr->edge_vertices_name.second);

    switch (e_ptr->edge_type)
    {
        case STRAIGHT_LINE:
        {
            Graph::Curves::calculateEquiDistantPosesLine({v0_ptr->vertex_pose, v1_ptr->vertex_pose}, e_ptr->forward_edge_poses.poses, 
                                                         e_ptr->edge_length, m_pose_spacing);
            
            break;
        }

        case CUBIC_BEZIER:
        {
            Graph::Curves::calculateEquiDistantPosesBezier({v0_ptr->vertex_pose, e_ptr->control_poses.front(), e_ptr->control_poses.back(), v1_ptr->vertex_pose}, 
                                                           e_ptr->forward_edge_poses.poses, e_ptr->edge_length, m_pose_spacing, m_resolution);
            break;
        }
        
        default:
            break;
    }

    m_graph_marker_server.applyChanges();
    publishAllPosesInGraph();
}

void GraphCreator::deleteControlOrientMarkerFromEdges(const std::unordered_set<std::string> edge_names)
{
    for(const auto &edge_name: edge_names)
        deleteControlOrientMarkerFromEdge(edge_name);
}

void GraphCreator::clearControlOrientMarkersForEdge(const std::string &edge_name)
{
    Graph::EdgePtr e_ptr = m_graph.getEdgeData(edge_name);

    if(!e_ptr)
    {
        ROS_WARN_STREAM("Cannot retreive data for edge "<<edge_name);
        return;
    }

    if(!e_ptr->use_independent_orientation) return;

    m_graph_marker_server.erase(e_ptr->control_orientation_name);
    m_graph_marker_server.applyChanges();
}

void GraphCreator::readdControlOrientMarkerForEdge(const std::string &edge_name)
{
   Graph::EdgePtr e_ptr = m_graph.getEdgeData(edge_name);

   if(!e_ptr)
   {
       ROS_WARN_STREAM("Cannot retreive data for edge "<<edge_name);
       return;
   }

   if(!e_ptr->use_independent_orientation) return;

   makeInteractiveControlOrientMarker(e_ptr->control_orientation_name, e_ptr->control_orientation_pose);
}

void GraphCreator::makeInteractiveEdgeDirectionMarkers(const std::string &dir_name, const geometry_msgs::Pose &pose, int16_t marker_type)
{
    auto direction_int_marker = Graph::Visuals::makeInteractiveEdgeDirectionMarkers(dir_name, pose, marker_type);
    Graph::Visuals::makeEdgeDirectionMarkerControls(direction_int_marker, marker_type);

    m_graph_marker_server.insert(direction_int_marker);
    m_graph_marker_server.setCallback(direction_int_marker.name, boost::bind(&GraphCreator::processInteractiveMarkerFeedback, this, _1));
}

void GraphCreator::initEdgeMenu()
{
    interactive_markers::MenuHandler::EntryHandle edge_crt_type_sub_menu, edge_dir_crt_type_sub_menu, edge_type_cvt_sub_menu; 
    interactive_markers::MenuHandler::EntryHandle edge_orient_cvt_sub_menu, edge_dir_cvt_sub_menu;

    //Default menu items
    edge_crt_type_sub_menu = m_edge_menu_handler.insert("Edge creation type");
    m_edge_type_handle = m_edge_menu_handler.insert(edge_crt_type_sub_menu, "Cubic Bezier", boost::bind(&GraphCreator::edgeTypeMenuCallback, this, _1));
    m_edge_type_cb_handle = m_edge_type_handle;
    m_edge_menu_handler.setCheckState(m_edge_type_handle, interactive_markers::MenuHandler::UNCHECKED);
    
    m_edge_type_handle = m_edge_menu_handler.insert(edge_crt_type_sub_menu, "Straight Line", boost::bind(&GraphCreator::edgeTypeMenuCallback, this, _1));
    m_edge_type_sl_handle = m_edge_type_handle;
    m_edge_menu_handler.setCheckState(m_edge_type_handle, interactive_markers::MenuHandler::CHECKED);
    m_default_edge_entry_handles.push_back(edge_crt_type_sub_menu);

    edge_dir_crt_type_sub_menu = m_edge_menu_handler.insert("Edge direction type");
    m_edge_dir_type_handle = m_edge_menu_handler.insert(edge_dir_crt_type_sub_menu, "Bidirectional", boost::bind(&GraphCreator::edgeDirectionTypeMenuCallback, this, _1));
    m_edge_dir_bi_handle = m_edge_dir_type_handle;
    m_edge_menu_handler.setCheckState(m_edge_dir_type_handle, interactive_markers::MenuHandler::UNCHECKED);

    m_edge_dir_type_handle = m_edge_menu_handler.insert(edge_dir_crt_type_sub_menu, "Unidirectional", boost::bind(&GraphCreator::edgeDirectionTypeMenuCallback, this, _1));
    m_edge_dir_fwd_handle = m_edge_dir_type_handle;
    m_edge_menu_handler.setCheckState(m_edge_dir_type_handle, interactive_markers::MenuHandler::CHECKED);
    m_default_edge_entry_handles.push_back(edge_dir_crt_type_sub_menu);

    edge_dir_cvt_sub_menu = m_edge_menu_handler.insert("Change this edge direction");
    m_edge_menu_handler.insert(edge_dir_cvt_sub_menu, "To bidirectional", boost::bind(&GraphCreator::edgeDirectionChangeMenuCallback, this, _1));
    m_edge_menu_handler.insert(edge_dir_cvt_sub_menu, "To unidirectional", boost::bind(&GraphCreator::edgeDirectionChangeMenuCallback, this, _1));
    m_default_edge_entry_handles.push_back(edge_dir_cvt_sub_menu);

    edge_type_cvt_sub_menu = m_edge_menu_handler.insert("Change this edge type");
    m_edge_menu_handler.insert(edge_type_cvt_sub_menu, "To straight line", boost::bind(&GraphCreator::edgeTypeChangeMenuCallback, this, _1));
    m_edge_menu_handler.insert(edge_type_cvt_sub_menu, "To cubic Bezier", boost::bind(&GraphCreator::edgeTypeChangeMenuCallback, this, _1));
    m_default_edge_entry_handles.push_back(edge_type_cvt_sub_menu);

    edge_orient_cvt_sub_menu = m_edge_menu_handler.insert("Change edge orientation");
    m_edge_menu_handler.insert(edge_orient_cvt_sub_menu, "To independent", boost::bind(&GraphCreator::edgeMenuCallback, this, _1));
    m_edge_menu_handler.insert(edge_orient_cvt_sub_menu, "To automatic", boost::bind(&GraphCreator::edgeMenuCallback, this, _1));
    m_default_edge_entry_handles.push_back(edge_orient_cvt_sub_menu);

    auto handle = m_edge_menu_handler.insert("Delete", boost::bind(&GraphCreator::edgeMenuCallback, this, _1));
    m_default_edge_entry_handles.push_back(handle);

    //Selection menu items
    auto edges_sel_sub_menu = m_edge_menu_handler.insert("Selection");
    m_edge_menu_handler.insert(edges_sel_sub_menu, "Select only edges", boost::bind(&GraphCreator::edgeSelectionMenuCallback, this, _1));
    m_edge_menu_handler.insert(edges_sel_sub_menu, "Deselect only edges", boost::bind(&GraphCreator::edgeSelectionMenuCallback, this, _1));
    m_edge_menu_handler.insert(edges_sel_sub_menu, "Reselect all", boost::bind(&GraphCreator::edgeSelectionMenuCallback, this, _1));
    m_edge_menu_handler.insert(edges_sel_sub_menu, "Deselect all", boost::bind(&GraphCreator::edgeSelectionMenuCallback, this, _1));
    m_select_edge_entry_handles.push_back(edges_sel_sub_menu);

    auto edges_dir_cvt_sub_menu = m_edge_menu_handler.insert("Change direction for edges");
    m_edge_menu_handler.insert(edges_dir_cvt_sub_menu, "To bidirectional", boost::bind(&GraphCreator::edgeDirectionsChangeMenuCallback, this, _1));
    m_edge_menu_handler.insert(edges_dir_cvt_sub_menu, "To unidirectional", boost::bind(&GraphCreator::edgeDirectionsChangeMenuCallback, this, _1));
    m_select_edge_entry_handles.push_back(edges_dir_cvt_sub_menu);

    auto edges_type_cvt_sub_menu = m_edge_menu_handler.insert("Change edge type for edges");
    m_edge_menu_handler.insert(edges_type_cvt_sub_menu, "To straight lines", boost::bind(&GraphCreator::edgeTypesChangeMenuCallback, this, _1));
    m_edge_menu_handler.insert(edges_type_cvt_sub_menu, "To cubic Bezier curves", boost::bind(&GraphCreator::edgeTypesChangeMenuCallback, this, _1));
    m_select_edge_entry_handles.push_back(edges_type_cvt_sub_menu);


    auto edges_orient_cvt_sub_menu = m_edge_menu_handler.insert("Change orientation for edges");
    m_edge_menu_handler.insert(edges_orient_cvt_sub_menu, "To independent", boost::bind(&GraphCreator::edgeOrientationsChangeMenuCallback, this, _1));
    m_edge_menu_handler.insert(edges_orient_cvt_sub_menu, "To automatic", boost::bind(&GraphCreator::edgeOrientationsChangeMenuCallback, this, _1));
    m_select_edge_entry_handles.push_back(edges_orient_cvt_sub_menu);

    auto del_edges_sub_menu = m_edge_menu_handler.insert("Delete");
    m_edge_menu_handler.insert(del_edges_sub_menu, "Delete only edges", boost::bind(&GraphCreator::deleteEdgesMenuCallback, this, _1));
    m_edge_menu_handler.insert(del_edges_sub_menu, "Delete all", boost::bind(&GraphCreator::deleteEdgesMenuCallback, this, _1));
    m_select_edge_entry_handles.push_back(del_edges_sub_menu);
    
    hideDefaultEdgeMenu(false);
    hideSelectionEdgeMenu(true);
}

void GraphCreator::edgeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std::string active_menu_entry;
    m_edge_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);

    ROS_INFO_STREAM("Edge marker: "<<feedback->marker_name);
    ROS_INFO_STREAM("Edge menu: "<<active_menu_entry);

    if(active_menu_entry == "Delete")  deleteEdgeWithMarkers(feedback->marker_name);
    else if(active_menu_entry == "To independent") addControlOrientMarkerToEdge(feedback->marker_name);
    else if(active_menu_entry == "To automatic") deleteControlOrientMarkerFromEdge(feedback->marker_name);

    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();
}

void GraphCreator::edgeSelectionMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std::string active_menu_entry;
    m_edge_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);

    ROS_INFO_STREAM("Edge marker: "<<feedback->marker_name);
    ROS_INFO_STREAM("Edge active menu selection: "<<active_menu_entry);

    if(active_menu_entry == "Select only edges")
    {
        deselectVerticesOnly();
        selectEdgesOnly();
    }
    else if(active_menu_entry == "Deselect only edges") deselectEdgesOnly();
    else if(active_menu_entry == "Reselect all") selectAllVerticesAndEdges();
    else if(active_menu_entry == "Deselect all") resetEdgesAndVerticesSelections();

    m_vert_menu_handler.reApply(m_graph_marker_server);
    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();
}

void GraphCreator::deleteEdgesMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std::string active_menu_entry;
    m_edge_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);

    ROS_INFO_STREAM("Edge marker: "<<feedback->marker_name);
    ROS_INFO_STREAM("Selected edges delete option: "<<active_menu_entry);

    if(active_menu_entry == "Delete only edges") deleteEdgesWithMarkers(m_sel_grp_edges_set);
    else if(active_menu_entry == "Delete all")
    {
        deleteEdgesWithMarkers(m_sel_grp_edges_set);
        deleteVerticesWithMarkers(m_sel_grp_vertices_set);
    }
    
    //After an operation has been completed all the selections are reset
    resetEdgesAndVerticesSelections();

    m_vert_menu_handler.reApply(m_graph_marker_server);
    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();    
}

void GraphCreator::edgeTypeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    m_edge_menu_handler.setCheckState(m_edge_type_handle, interactive_markers::MenuHandler::UNCHECKED);
    m_edge_type_handle = feedback->menu_entry_id;
    m_edge_menu_handler.setCheckState(m_edge_type_handle, interactive_markers::MenuHandler::CHECKED);

    std::string active_menu_entry;
    m_edge_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);

    ROS_INFO_STREAM("Edge marker: "<<feedback->marker_name);
    ROS_INFO_STREAM("Edge creation type changed to: "<<active_menu_entry);

    if(active_menu_entry == "Straight Line") m_current_curve_type = STRAIGHT_LINE;
    else if(active_menu_entry == "Cubic Bezier") m_current_curve_type = CUBIC_BEZIER;

    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();

    publishCurrentGraphCreatorState();
}

void GraphCreator::edgeDirectionTypeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    m_edge_menu_handler.setCheckState(m_edge_dir_type_handle, interactive_markers::MenuHandler::UNCHECKED);
    m_edge_dir_type_handle = feedback->menu_entry_id;
    m_edge_menu_handler.setCheckState(m_edge_dir_type_handle, interactive_markers::MenuHandler::CHECKED);

    std::string active_menu_entry;
    m_edge_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);

    ROS_INFO_STREAM("Edge marker: "<<feedback->marker_name);
    ROS_INFO_STREAM("Edge creation direction type changed to: "<<active_menu_entry);

    if(active_menu_entry == "Bidirectional") m_edge_is_bidirectional = true;
    else if(active_menu_entry == "Unidirectional") m_edge_is_bidirectional = false;

    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();

    publishCurrentGraphCreatorState();
}

void GraphCreator::edgeTypeChangeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std::string active_menu_entry;
    m_edge_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);

    ROS_INFO_STREAM("Edge marker: "<<feedback->marker_name);
    ROS_INFO_STREAM("Edge type changed to: "<<active_menu_entry);

    //New curves will be of this type
    uint8_t newly_selected_curve_type = m_current_curve_type;
    if(active_menu_entry == "To straight line") newly_selected_curve_type = STRAIGHT_LINE;
    else if(active_menu_entry == "To cubic Bezier") newly_selected_curve_type = CUBIC_BEZIER;

    changeEdgeTypeWithMarkers(feedback->marker_name, newly_selected_curve_type);

    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();
}

void GraphCreator::edgeTypesChangeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std::string active_menu_entry;
    m_edge_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);

    ROS_INFO_STREAM("Edge marker: "<<feedback->marker_name);
    ROS_INFO_STREAM("Type of edges changed to: "<<active_menu_entry);

    uint8_t newly_selected_curve_type = m_current_curve_type;
    if(active_menu_entry == "To straight lines") newly_selected_curve_type = STRAIGHT_LINE;
    else if(active_menu_entry == "To cubic Bezier curves") newly_selected_curve_type = CUBIC_BEZIER;
    
    changeEdgeTypesWithMarkers(m_sel_grp_edges_set, newly_selected_curve_type);

    //After an operation has been completed all the selections are reset
    resetEdgesAndVerticesSelections();

    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();
}

void GraphCreator::edgeDirectionChangeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std::string active_menu_entry;
    m_edge_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);

    ROS_INFO_STREAM("Edge marker: "<<feedback->marker_name);
    ROS_INFO_STREAM("Edge directionality changed to: "<<active_menu_entry);

    if(active_menu_entry == "To bidirectional") changeEdgeDirectionWithMarkers(feedback->marker_name, true);
    else if(active_menu_entry == "To unidirectional") changeEdgeDirectionWithMarkers(feedback->marker_name, false);

    //After an operation has been completed all the selections are reset
    resetEdgesAndVerticesSelections();

    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();
}

void GraphCreator::edgeDirectionsChangeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std::string active_menu_entry;
    m_edge_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);

    ROS_INFO_STREAM("Edge marker: "<<feedback->marker_name);
    ROS_INFO_STREAM("Edge directions changed to: "<<active_menu_entry);

    if(active_menu_entry == "To bidirectional") changeEdgeDirectionsWithMarkers(m_sel_grp_edges_set, true);
    else if(active_menu_entry == "To unidirectional") changeEdgeDirectionsWithMarkers(m_sel_grp_edges_set, false);

    //After an operation has been completed all the selections are reset
    resetEdgesAndVerticesSelections();

    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();
}

void GraphCreator::edgeOrientationsChangeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std::string active_menu_entry;
    m_edge_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);

    ROS_INFO_STREAM("Edge marker: "<<feedback->marker_name);
    ROS_INFO_STREAM("Edge directions changed to: "<<active_menu_entry);

    if(active_menu_entry == "To independent") addControlOrientMarkerToEdges(m_sel_grp_edges_set);
    else if(active_menu_entry == "To automatic") deleteControlOrientMarkerFromEdges(m_sel_grp_edges_set);

    //After an operation has been completed all the selections are reset
    resetEdgesAndVerticesSelections();

    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();
}

void GraphCreator::hideDefaultEdgeMenu(bool hide)
{
    for(const auto &handle: m_default_edge_entry_handles)
        m_edge_menu_handler.setVisible(handle, !hide);
}

void GraphCreator::hideSelectionEdgeMenu(bool hide)
{
    for(const auto &handle: m_select_edge_entry_handles)
        m_edge_menu_handler.setVisible(handle, !hide);
}

bool GraphCreator::createVertexAndMarker(const Graph::Vertex &vertex)
{
    if(m_graph.isVertex(vertex.vertex_name))
    {
        ROS_FATAL_STREAM("Multiple copies of vertex "<<vertex.vertex_name);
        return false;
    }

    auto vertex_int_marker = Graph::Visuals::makeInteractiveVertexMarker(vertex.vertex_id, vertex.vertex_pose);
    if(!vertex.ignore_orientation) Graph::Visuals::makeVertexArrowControls(vertex_int_marker);
    else Graph::Visuals::makeVertexSphereControls(vertex_int_marker);

    m_graph_marker_server.insert(vertex_int_marker);
    m_vert_menu_handler.apply(m_graph_marker_server, vertex_int_marker.name);
    m_graph_marker_server.setCallback(vertex_int_marker.name, boost::bind(&GraphCreator::processInteractiveMarkerFeedback, this, _1));
    m_graph_marker_server.setCallback(vertex_int_marker.name, 
                                      boost::bind(&GraphCreator::updateInteractiveMarkerPoses, this, _1), 
                                      visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);

    m_graph.addVertex(vertex.vertex_name, vertex);
    m_graph.printGraph();
    
    //Update the vertex ID counter using the vertex ID itself
    if(vertex.vertex_id > m_vertex_id_counter) m_vertex_id_counter = vertex.vertex_id;

    m_prev_selected_vertex = vertex_int_marker.name;

    return true;
}

void GraphCreator::addVertexWithMarker(int vertex_id, const geometry_msgs::Pose &pose)
{
    std::string vertex_name = Graph::Visuals::generateVertexMarkerName(vertex_id);
    
    if(m_graph.isVertex(vertex_name))
    {
        ROS_WARN_STREAM( "Vertex "<<vertex_name<<" already exists");
        return;
    }

    auto vertex_int_marker = Graph::Visuals::makeInteractiveVertexMarker(vertex_id, pose);

    Graph::Vertex v;
    v.vertex_id = vertex_id;
    v.vertex_name = vertex_int_marker.name;
    v.vertex_alias = ""; //During creation alias is always empty
    v.vertex_type = m_current_vertex_type;
    v.vertex_pose = pose;
    v.ignore_orientation = m_vertex_ignore_orientation;
    m_graph.addVertex(vertex_int_marker.name, v);
    m_graph.printGraph();

    if(!v.ignore_orientation) Graph::Visuals::makeVertexArrowControls(vertex_int_marker);
    else Graph::Visuals::makeVertexSphereControls(vertex_int_marker);

    m_graph_marker_server.insert(vertex_int_marker);
    m_vert_menu_handler.apply(m_graph_marker_server, vertex_int_marker.name);
    m_graph_marker_server.setCallback(vertex_int_marker.name, boost::bind(&GraphCreator::processInteractiveMarkerFeedback, this, _1));
    m_graph_marker_server.setCallback(vertex_int_marker.name, 
                                      boost::bind(&GraphCreator::updateInteractiveMarkerPoses, this, _1), 
                                      visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);

    publishCurrentGraphVertex(v.vertex_name);

    m_prev_selected_vertex = vertex_int_marker.name;
}

void GraphCreator::deleteVertexWithMarker(const std::string &vertex_name)
{
    if(!m_graph.isVertex(vertex_name))
    {
        ROS_WARN_STREAM("The vertex "<<vertex_name<<" cannot be deleted as it does not exist");
        return;
    }

    Graph::EdgeDepSet eds;
    m_graph.getVertexToEdgeDependency(vertex_name, eds);
    for(Graph::EdgeDepSet::iterator it_e = eds.begin(); it_e != eds.end(); ++it_e) //For each edge delete miscellaneous dependency
    {
        //NOTE: If the control pose markers are clear some warning might be seen
        ROS_INFO_STREAM("Deleting edge "<<*it_e<<" that is connected to vertex "<<vertex_name);
        deleteEdgeWithMarkers(*it_e);
    }
    
    m_graph.deleteVertex(vertex_name);
    m_graph_marker_server.erase(vertex_name);
    m_graph.printGraph();

    publishAllPosesInGraph();
}

void GraphCreator::deleteVerticesWithMarkers(const std::unordered_set<std::string> &vertex_names)
{
    for(const auto &vertex_name: vertex_names)
        deleteVertexWithMarker(vertex_name);
}

void GraphCreator::changeVertexOrientationWithMarker(const std::string &vertex_name, bool ignore_orientation)
{
    Graph::VertexPtr v_ptr = m_graph.getVertexData(vertex_name);

    if(!v_ptr)
    {
        ROS_ERROR_STREAM("Vertex "<<vertex_name<<" was not found");
        return;
    }

    if(v_ptr->ignore_orientation == ignore_orientation)
    {
        ROS_WARN_STREAM("Selected was already to "<<(ignore_orientation ? "" : "not")<<"ignore orientation");
        return;
    }

    v_ptr->ignore_orientation = ignore_orientation; //Update vertex 

    //Update interactive marker
    visualization_msgs::InteractiveMarker vertex_int_marker;
    m_graph_marker_server.get(vertex_name, vertex_int_marker);

    if(!v_ptr->ignore_orientation) Graph::Visuals::makeVertexArrowControls(vertex_int_marker);
    else Graph::Visuals::makeVertexSphereControls(vertex_int_marker);

    m_graph_marker_server.erase(vertex_name); //Delete the vertex marker from graph marker server
    
    m_graph_marker_server.insert(vertex_int_marker); //Insert updated interactive marker for the edge
}

void GraphCreator::changeVerticesOrientationWithMarkers(const std::unordered_set<std::string> &vertex_names, bool ignore_orientations)
{
    for(const auto &vertex_name: vertex_names)
        changeVertexOrientationWithMarker(vertex_name, ignore_orientations);
}

void GraphCreator::lockAllVertices(bool lock)
{
    visualization_msgs::InteractiveMarker int_marker;
    Graph::VerticesMapPtr vm_ptr = m_graph.getVertices();

    for(Graph::VerticesMap::iterator it_vm = vm_ptr->begin(); it_vm != vm_ptr->end(); ++it_vm)
    {
        if(!m_graph_marker_server.get(it_vm->first, int_marker)) continue;

        if(lock) 
        {
            int_marker.controls.at(0).interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
            int_marker.controls.at(1).interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
        }
        else 
        {
            if(!it_vm->second->ignore_orientation) 
            {
                int_marker.controls.at(0).interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
                int_marker.controls.at(1).interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
            }
            else 
            {
                int_marker.controls.at(0).interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
                int_marker.controls.at(1).interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
            }
        }
        m_graph_marker_server.insert(int_marker);
    }

    m_is_vertices_locked = lock;
}

void GraphCreator::lockAllEdges(bool lock)
{
    visualization_msgs::InteractiveMarker int_marker;
    Graph::EdgesMapPtr em_ptr = m_graph.getEdges();

    for(Graph::EdgesMap::iterator it_em = em_ptr->begin(); it_em != em_ptr->end(); ++it_em)
    {
        clearControlPoseMarkersForEdge(it_em->first);
        clearControlOrientMarkersForEdge(it_em->first);
    }

    m_is_edges_locked = lock;
}

void GraphCreator::publishAllPosesInGraph()
{
    Graph::EdgesMapPtr em_ptr = m_graph.getEdges();

    if(!em_ptr)
    {
        ROS_ERROR_STREAM("Unable to get data for edges");
        return;
    }

    m_fwd_pose_arr_msg.header.frame_id = "map";
    m_fwd_pose_arr_msg.header.seq++;
    m_fwd_pose_arr_msg.header.stamp = ros::Time::now();

    m_bck_pose_arr_msg.header = m_fwd_pose_arr_msg.header;
    
    m_fwd_pose_arr_msg.poses.clear();
    m_bck_pose_arr_msg.poses.clear();

    for(Graph::EdgesMap::const_iterator it_em = em_ptr->begin(); it_em != em_ptr->end(); ++it_em)
    {
        m_fwd_pose_arr_msg.poses.reserve(m_fwd_pose_arr_msg.poses.size() + std::distance(it_em->second->forward_edge_poses.poses.begin(), 
                                                                                         it_em->second->forward_edge_poses.poses.end()));
        
        m_fwd_pose_arr_msg.poses.insert(m_fwd_pose_arr_msg.poses.end(), it_em->second->forward_edge_poses.poses.begin(), 
                                        it_em->second->forward_edge_poses.poses.end());

        m_bck_pose_arr_msg.poses.reserve(m_bck_pose_arr_msg.poses.size() + std::distance(it_em->second->backward_edge_poses.poses.begin(), 
                                                                                         it_em->second->backward_edge_poses.poses.end()));

        m_bck_pose_arr_msg.poses.insert(m_bck_pose_arr_msg.poses.end(), it_em->second->backward_edge_poses.poses.begin(), 
                                        it_em->second->backward_edge_poses.poses.end());
    }

    m_fwd_pose_arr_pub.publish(m_fwd_pose_arr_msg);
    m_bck_pose_arr_pub.publish(m_bck_pose_arr_msg);
}

void GraphCreator::initVertexMenu()
{
    //Default menu items
    interactive_markers::MenuHandler::EntryHandle vert_mode_sub_menu = m_vert_menu_handler.insert("Vertex mode");
    m_vert_mode_handle = m_vert_menu_handler.insert(vert_mode_sub_menu, "Create edges", boost::bind(&GraphCreator::vertexModeCallback, this, _1));
    m_create_edges_handle = m_vert_mode_handle;
    m_vert_menu_handler.setCheckState(m_vert_mode_handle, interactive_markers::MenuHandler::UNCHECKED);

    m_vert_mode_handle = m_vert_menu_handler.insert(vert_mode_sub_menu, "Create vertices", boost::bind(&GraphCreator::vertexModeCallback, this, _1));
    m_create_vertices_handle = m_vert_mode_handle;
    m_vert_menu_handler.setCheckState(m_vert_mode_handle, interactive_markers::MenuHandler::CHECKED);
    m_default_vert_entry_handles.push_back(vert_mode_sub_menu);

    auto handle = m_vert_menu_handler.insert("Set vertex pose to robot pose", boost::bind(&GraphCreator::vertexPoseToRobotPoseMenuCallback, this, _1));
    m_default_vert_entry_handles.push_back(handle);
    
    interactive_markers::MenuHandler::EntryHandle vert_ornt_ign_sub_menu = m_vert_menu_handler.insert("Vertex orientation");
    m_vert_ornt_ign_handle = m_vert_menu_handler.insert(vert_ornt_ign_sub_menu, "Ignore orientation", 
                                                        boost::bind(&GraphCreator::vertexOrientationMenuCallback, this, _1));

    m_vert_menu_handler.setCheckState(m_vert_ornt_ign_handle, interactive_markers::MenuHandler::UNCHECKED);

    m_vert_ornt_ign_handle = m_vert_menu_handler.insert(vert_ornt_ign_sub_menu, "Acknowledge orientation", 
                                                        boost::bind(&GraphCreator::vertexOrientationMenuCallback, this, _1));

    m_vert_menu_handler.setCheckState(m_vert_ornt_ign_handle, interactive_markers::MenuHandler::CHECKED);
    m_default_vert_entry_handles.push_back(vert_ornt_ign_sub_menu);

    interactive_markers::MenuHandler::EntryHandle chg_vert_ornt_sub_menu = m_vert_menu_handler.insert("Change this vertex orientation");
    m_vert_menu_handler.insert(chg_vert_ornt_sub_menu, "To ignored", boost::bind(&GraphCreator::vertexOrienationChangeMenuCallback, this, _1));
    m_vert_menu_handler.insert(chg_vert_ornt_sub_menu, "To acknowledged", boost::bind(&GraphCreator::vertexOrienationChangeMenuCallback, this, _1));
    m_default_vert_entry_handles.push_back(chg_vert_ornt_sub_menu);

    handle = m_vert_menu_handler.insert("Delete", boost::bind(&GraphCreator::vertexMenuCallback, this, _1));
    m_default_vert_entry_handles.push_back(handle);

    interactive_markers::MenuHandler::EntryHandle set_graph_sub_menu = m_vert_menu_handler.insert("Set graph");
    m_vert_menu_handler.insert(set_graph_sub_menu, "Publish current graph", boost::bind(&GraphCreator::setGraphMenuCallback, this, _1));
    m_vert_menu_handler.insert(set_graph_sub_menu, "Load and publish graph", boost::bind(&GraphCreator::setGraphMenuCallback, this, _1));
    m_default_vert_entry_handles.push_back(set_graph_sub_menu);

    handle = m_vert_menu_handler.insert("Save graph", boost::bind(&GraphCreator::saveGraphMenuCallback, this, _1));
    m_default_vert_entry_handles.push_back(handle);
    handle = m_vert_menu_handler.insert("Load graph", boost::bind(&GraphCreator::loadGraphMenuCallback, this, _1));
    m_default_vert_entry_handles.push_back(handle);

    //Selection menu items
    auto verts_sel_sub_menu = m_vert_menu_handler.insert("Selection");
    m_vert_menu_handler.insert(verts_sel_sub_menu, "Select only vertices", boost::bind(&GraphCreator::vertexSelectionMenuCallback, this, _1));
    m_vert_menu_handler.insert(verts_sel_sub_menu, "Deselect only vertices", boost::bind(&GraphCreator::vertexSelectionMenuCallback, this, _1));
    m_vert_menu_handler.insert(verts_sel_sub_menu, "Reselect all", boost::bind(&GraphCreator::vertexSelectionMenuCallback, this, _1));
    m_vert_menu_handler.insert(verts_sel_sub_menu, "Deselect all", boost::bind(&GraphCreator::vertexSelectionMenuCallback, this, _1));
    m_select_vert_entry_handles.push_back(verts_sel_sub_menu);

    auto chg_verts_ornt_sub_menu = m_vert_menu_handler.insert("Change orientation for vertices");
    m_vert_menu_handler.insert(chg_verts_ornt_sub_menu, "To ignored", boost::bind(&GraphCreator::verticesOrientationChangeMenuCallback, this, _1));
    m_vert_menu_handler.insert(chg_verts_ornt_sub_menu, "To acknowledged", boost::bind(&GraphCreator::verticesOrientationChangeMenuCallback, this, _1));
    m_select_vert_entry_handles.push_back(chg_verts_ornt_sub_menu);

    auto del_verts_sub_menu = m_vert_menu_handler.insert("Delete");
    m_vert_menu_handler.insert(del_verts_sub_menu, "Delete only vertices", boost::bind(&GraphCreator::deleteVerticesMenuCallback, this, _1));
    m_vert_menu_handler.insert(del_verts_sub_menu, "Delete all", boost::bind(&GraphCreator::deleteVerticesMenuCallback, this, _1));
    m_select_vert_entry_handles.push_back(del_verts_sub_menu);

    hideDefaultVertexMenu(false);
    hideSelectionVertexMenu(true);
}

void GraphCreator::vertexMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std::string active_menu_entry;
    m_vert_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);

    ROS_INFO_STREAM("Vertex marker: "<<feedback->marker_name);
    ROS_INFO_STREAM("Vertex menu: "<<active_menu_entry);

    if(active_menu_entry == "Delete") deleteVertexWithMarker(feedback->marker_name);
    else ROS_ERROR_STREAM("Unknown vertex menu activated");

    m_vert_menu_handler.reApply(m_graph_marker_server);
    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();
}

void GraphCreator::vertexModeCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    m_vert_menu_handler.setCheckState(m_vert_mode_handle, interactive_markers::MenuHandler::UNCHECKED);
    m_vert_mode_handle = feedback->menu_entry_id;
    m_vert_menu_handler.setCheckState(m_vert_mode_handle, interactive_markers::MenuHandler::CHECKED);

    std::string active_menu_entry;
    m_vert_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);
    
    ROS_INFO_STREAM("Vertex marker: "<<feedback->marker_name);
    ROS_INFO_STREAM("Vertex mode changed to: "<<active_menu_entry);
    
    if(active_menu_entry == "Create edges")
    {
        m_current_graph_mode = CREATE_EDGES;
        lockAllVertices(true);
        ROS_INFO_STREAM("Graph is currently in create connections mode");

        visualization_msgs::InteractiveMarker int_marker;
        applyStashedMarkerColors();
    }
    else if(active_menu_entry == "Create vertices")
    {
        m_current_graph_mode = CREATE_VERTICES;
        lockAllVertices(false);
        ROS_INFO_STREAM("Graph is currently in create vertices mode");

        m_selected_edge_vertices.clear(); //Reset any previously selected vertex

        visualization_msgs::InteractiveMarker int_marker;
        applyStashedMarkerColors();
    }
    else
    {
        ROS_ERROR_STREAM("Unknown vertex mode menu activated");
    }

    m_vert_menu_handler.reApply(m_graph_marker_server);
    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();

    publishCurrentGraphCreatorState();
}

void GraphCreator::vertexOrientationMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    m_vert_menu_handler.setCheckState(m_vert_ornt_ign_handle, interactive_markers::MenuHandler::UNCHECKED);
    m_vert_ornt_ign_handle = feedback->menu_entry_id;
    m_vert_menu_handler.setCheckState(m_vert_ornt_ign_handle, interactive_markers::MenuHandler::CHECKED);

    std::string active_menu_entry;
    m_vert_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);

    ROS_INFO_STREAM("Vertex marker: "<<feedback->marker_name);
    ROS_INFO_STREAM("Vertex orientation on creation changed to: "<<active_menu_entry);
    
    if(active_menu_entry == "Ignore orientation") m_vertex_ignore_orientation = true;
    else if(active_menu_entry == "Acknowledge orientation") m_vertex_ignore_orientation = false;
    
    ROS_INFO_STREAM("Vertex will be created with "<<(m_vertex_ignore_orientation ? "ignored orientation" : "orientation acknowledged"));

    m_vert_menu_handler.reApply(m_graph_marker_server);
    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();
}

void GraphCreator::vertexOrienationChangeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std::string active_menu_entry;
    m_vert_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);

    ROS_INFO_STREAM("Vertex marker: "<<feedback->marker_name);
    ROS_INFO_STREAM("Vertex orientation changed to: "<<active_menu_entry);

    if(active_menu_entry == "To ignored") changeVertexOrientationWithMarker(feedback->marker_name, true);
    else if(active_menu_entry == "To acknowledged") changeVertexOrientationWithMarker(feedback->marker_name, false);

    resetEdgesAndVerticesSelections();

    m_vert_menu_handler.reApply(m_graph_marker_server);
    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();
}

void GraphCreator::vertexSelectionMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std::string active_menu_entry;
    m_vert_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);

    ROS_INFO_STREAM("Vertex marker: "<<feedback->marker_name);
    ROS_INFO_STREAM("Vertex active menu selection: "<<active_menu_entry);

    if(active_menu_entry == "Select only vertices")
    {
        deselectEdgesOnly();
        selectVerticesOnly();
    }
    else if(active_menu_entry == "Deselect only vertices") deselectVerticesOnly();
    else if(active_menu_entry == "Reselect all") selectAllVerticesAndEdges();
    else if(active_menu_entry == "Deselect all") resetEdgesAndVerticesSelections();

    m_vert_menu_handler.reApply(m_graph_marker_server);
    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();
}

void GraphCreator::verticesOrientationChangeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{

    std::string active_menu_entry;
    m_vert_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);

    ROS_INFO_STREAM("Vertex marker: "<<feedback->marker_name);
    ROS_INFO_STREAM("Orientation for vertices changed to: "<<active_menu_entry);

    bool is_ignored = false;
    if(active_menu_entry == "To ignored") is_ignored = true;
    else if(active_menu_entry == "To acknowledged") is_ignored = false;
    else
    {
        ROS_ERROR_STREAM("Unknown vertex orientation type was selected");
        return;
    }

    changeVerticesOrientationWithMarkers(m_sel_grp_vertices_set, is_ignored);

    //After an operation has been completed all the selections are reset
    resetEdgesAndVerticesSelections();

    m_vert_menu_handler.reApply(m_graph_marker_server);
    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();
}

void GraphCreator::deleteVerticesMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std::string active_menu_entry;
    m_vert_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);

    ROS_INFO_STREAM("Vertex marker: "<<feedback->marker_name);
    ROS_INFO_STREAM("Selected vertices delete option: "<<active_menu_entry);

    if(active_menu_entry == "Delete only vertices") deleteVerticesWithMarkers(m_sel_grp_vertices_set);
    else if(active_menu_entry == "Delete all")
    {
        deleteEdgesWithMarkers(m_sel_grp_edges_set);
        deleteVerticesWithMarkers(m_sel_grp_vertices_set);
    }
    
    //After an operation has been completed all the selections are reset
    resetEdgesAndVerticesSelections();

    m_vert_menu_handler.reApply(m_graph_marker_server);
    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();
}

void GraphCreator::hideDefaultVertexMenu(bool hide)
{
    for(const auto &handle: m_default_vert_entry_handles)
        m_vert_menu_handler.setVisible(handle, !hide);
}

void GraphCreator::hideSelectionVertexMenu(bool hide)
{
    for(const auto &handle: m_select_vert_entry_handles)
        m_vert_menu_handler.setVisible(handle, !hide);
}

void GraphCreator::vertexPoseToRobotPoseMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    if(!m_is_robot_pose_received)
    {
        ROS_WARN_STREAM("Cannot set vertex pose to robot pose as it was never received");
        return;
    }

    ros::Duration time_out_duration = ros::Time::now() - m_prev_robot_pose_time;
    if(time_out_duration.toSec() <= 2.0f)
    {
        ROS_INFO_STREAM("Setting the vertex pose to the robot pose");
        
        updateVertexAndMarker(feedback->marker_name, m_robot_pose_msg);

        m_graph_marker_server.setPose(feedback->marker_name, m_robot_pose_msg);
        m_graph_marker_server.applyChanges();

        return;
    }

    ROS_WARN_STREAM("The robot pose was not received with in the specified time period");
}

void GraphCreator::processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    switch(feedback->event_type)
    {
        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        {
            ROS_INFO_STREAM("Mouse down event received");
            onMouseDown(feedback);
            break;
        }

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        {
            ROS_INFO_STREAM("Mouse up event received");
            onMouseUp(feedback);
            break;
        }

        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        {
            ROS_INFO_STREAM("Button click event received");
            onButtonClick(feedback);
            break;
        }

        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
        {
            ROS_INFO_STREAM("Menu select event received");
            break;
        }

        case visualization_msgs::InteractiveMarkerFeedback::KEEP_ALIVE:
        {
            ROS_INFO_STREAM("Keep alive event received");
            break;
        }
    }

    m_graph_marker_server.applyChanges();
}

void GraphCreator::onMouseUp(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    visualization_msgs::InteractiveMarker int_marker;

    switch(m_current_graph_mode)
    {
        case CREATE_VERTICES:
        {

            if(m_graph.isVertex(feedback->marker_name))
            {
                m_prev_selected_vertex = feedback->marker_name;
                mapBaseLinkTfBroadcast(feedback->pose); //Update base link pose to the vertex pose
                publishCurrentGraphVertex(feedback->marker_name);
            }
            else if(m_graph.isEdge(feedback->marker_name))
                m_prev_selected_edge = feedback->marker_name;

            applyStashedMarkerColors();
            break;
        }

        case CREATE_EDGES:
        {
            bool clear_selected_marker = false;

            if(m_graph.isVertex(feedback->marker_name))
            {
                if(m_selected_edge_vertices.size() == 0)
                {
                    ROS_INFO_STREAM("Selected edge source vertex: "<<feedback->marker_name);
                    m_selected_edge_vertices.push_back(feedback->marker_name);
                    mapBaseLinkTfBroadcast(feedback->pose);
                }
                else if(m_selected_edge_vertices.at(0) != feedback->marker_name && m_selected_edge_vertices.size() == 1)
                {
                    ROS_INFO_STREAM("Selected edge destination vertex: "<<feedback->marker_name);
                    m_selected_edge_vertices.push_back(feedback->marker_name);
                    addEdgeWithMarkers(m_selected_edge_vertices ,m_current_curve_type);
                    mapBaseLinkTfBroadcast(feedback->pose); //TODO this need to be updated if the the plan is successful
                    clear_selected_marker = true;
                }
                else
                {
                    ROS_WARN_STREAM("The same vertex was again selected for edge creation");
                    clear_selected_marker = true;
                }
            }
            else
            {
                ROS_WARN_STREAM("Selection was not a vertex");
                clear_selected_marker = true;
            }

            //Clear properties of the selected marker(s)
            if(clear_selected_marker)
            {
                for(size_t i = 0; i < m_selected_edge_vertices.size(); ++i)
                {
                    if(!m_graph_marker_server.get(m_selected_edge_vertices.at(i), int_marker)) continue;

                    int_marker.controls.back().markers.front().color = Graph::Colors::getColorRGBAMsg(Graph::Colors::RED);

                    m_graph_marker_server.insert(int_marker);
                }

                m_selected_edge_vertices.clear();
                clearStashedMarkersColors();
            }

            break;
        }
    }
}

void GraphCreator::onMouseDown(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    visualization_msgs::InteractiveMarker int_marker;
    m_graph_marker_server.get(feedback->marker_name, int_marker);

    //TODO: Color is being reset when clicking on graph elements while in selection. Color needs to be kept constant to magenta
    switch(m_current_graph_mode)
    {
        case CREATE_VERTICES:
        {
            stashMarkerColor(int_marker);
            int_marker.controls.back().markers.front().color = Graph::Colors::getColorRGBAMsg(Graph::Colors::BLUE);
            m_graph_marker_server.insert(int_marker);

            break;
        }

        case CREATE_EDGES:
        {
            if(!m_graph.isVertex(feedback->marker_name)) break;

            stashMarkerColor(int_marker);
            int_marker.controls.back().markers.front().color = Graph::Colors::getColorRGBAMsg(Graph::Colors::MAGENTA);
            m_graph_marker_server.insert(int_marker);

            m_prev_selected_vertex = feedback->marker_name;

            break;
        }
    }
}

void GraphCreator::onButtonClick(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    if(m_graph.isVertex(feedback->marker_name))
    {
        clearControlPoseMarkersForEdge(m_prev_selected_edge);
        clearControlOrientMarkersForEdge(m_prev_selected_edge);
        
        m_prev_selected_vertex = feedback->marker_name;

        publishCurrentGraphVertex(feedback->marker_name);
    }
    else if(m_graph.isEdge(feedback->marker_name))
    {
        Graph::EdgePtr e_ptr = m_graph.getEdgeData(feedback->marker_name);
        
        if(!m_is_edges_locked)
        {
            visualization_msgs::InteractiveMarker int_marker;
            for(const auto cpn: e_ptr->control_poses_name)
            {
                if(!m_graph_marker_server.get(cpn, int_marker)) //Check whether the marker exists in the server
                {
                    readdControlPoseMarkersForEdge(feedback->marker_name);
                    break;
                }
                else
                {
                    clearControlPoseMarkersForEdge(feedback->marker_name);
                    break;
                }
            }
        
            if(!m_graph_marker_server.get(e_ptr->control_orientation_name, int_marker)) 
                readdControlOrientMarkerForEdge(feedback->marker_name);
            else
                clearControlOrientMarkersForEdge(feedback->marker_name);

            if(m_prev_selected_edge != feedback->marker_name)
            {
                clearControlPoseMarkersForEdge(m_prev_selected_edge);
                clearControlOrientMarkersForEdge(m_prev_selected_edge);
            }
        }

        m_prev_selected_edge = feedback->marker_name;

        publishCurrentGraphEdge(feedback->marker_name);
    }
}

geometry_msgs::Point GraphCreator::snapPosition(const geometry_msgs::Point &position, double snap_grid) const
{
    geometry_msgs::Point snap_position;

    snap_position.x = ((position.x - std::fmod(position.x, snap_grid)) / snap_grid) * snap_grid;
    snap_position.y = ((position.y - std::fmod(position.y, snap_grid)) / snap_grid) * snap_grid;

    return snap_position;
}

geometry_msgs::Quaternion GraphCreator::snapOrientation(const geometry_msgs::Quaternion &orientation, double snap_angle) const
{
    geometry_msgs::Quaternion snap_orientation;
    snap_orientation.w = 1;

    tf2::Quaternion input_quat, snap_quat;
    tf2::convert(orientation, input_quat);
    
    double input_roll, input_pitch ,input_yaw;
    tf2::Matrix3x3(input_quat).getRPY(input_roll, input_pitch, input_yaw);
    double snap_yaw = std::round(input_yaw / snap_angle) * snap_angle;
    snap_quat.setRPY(0.0, 0.0, snap_yaw);

    tf2::convert(snap_quat.normalize(), snap_orientation);

    return snap_orientation;
}

bool GraphCreator::applyVertexPositionSnap(const geometry_msgs::Point &in_position, geometry_msgs::Point &out_position)
{
    if(!m_enable_vertex_snap) return false;

    if(!m_enable_position_snap)
    {
        out_position = in_position;
        return false;
    }
    
    out_position = snapPosition(in_position, m_position_snap_grid_size);
    return true;
}

bool GraphCreator::applyVertexOrientationSnap(const geometry_msgs::Quaternion &in_orientation, geometry_msgs::Quaternion &out_orientation)
{
    if(!m_enable_vertex_snap) return false;

    if(!m_enable_orientation_snap)
    {
        out_orientation = in_orientation;
        return false;
    }

    out_orientation = snapOrientation(in_orientation, m_orientation_snap_angle);
    return true;
}

bool GraphCreator::applyEdgeElementPositionSnap(const geometry_msgs::Point &in_position, geometry_msgs::Point &out_position)
{
    if(!m_enable_edge_snap) return false;

    if(!m_enable_position_snap)
    {
        out_position = in_position;
        return false;
    }

    out_position = snapPosition(in_position, m_position_snap_grid_size);
    return true;
}

bool GraphCreator::applyEdgeElementOrientationSnap(const geometry_msgs::Quaternion &in_orientation, geometry_msgs::Quaternion &out_orientation)
{
    if(!m_enable_edge_snap) return false;

    if(!m_enable_orientation_snap)
    {
        out_orientation = in_orientation;
        return false;
    }

    out_orientation = snapOrientation(in_orientation, m_orientation_snap_angle);
    return true;
}

bool GraphCreator::updateVertexAndMarker(const std::string &vertex_name, const geometry_msgs::Pose &vertex_pose)
{
    if(!m_graph.isVertex(vertex_name))
    {
        ROS_WARN_STREAM("Cannot update vertex pose as the given graph element "<<vertex_name<<" is not a vertex");
        return false;
    }

    //Update the vertex data
    Graph::VertexPtr v_ptr = m_graph.getVertexData(vertex_name);
    
    if(!v_ptr)
    {
        ROS_ERROR_STREAM("Update for vertex "<<vertex_name<<" could not be done as the data could not be retreived");
        return false;
    }

    v_ptr->vertex_pose = vertex_pose;
    applyVertexPositionSnap(vertex_pose.position, v_ptr->vertex_pose.position);
    applyVertexOrientationSnap(vertex_pose.orientation, v_ptr->vertex_pose.orientation);
    
    m_graph_marker_server.setPose(vertex_name, v_ptr->vertex_pose); //Set the updated pose for the vertex

    Graph::EdgeDepSet eds;
    m_graph.getVertexToEdgeDependency(v_ptr->vertex_name, eds);

    std::vector<geometry_msgs::Pose> out_poses;

    for(Graph::EdgeDepSet::iterator it_e = eds.begin(); it_e != eds.end(); ++it_e) //Update poses for all edges
    {
        Graph::EdgePtr e_ptr = m_graph.getEdgeData(*it_e);

        if(!e_ptr)
        {
            ROS_ERROR_STREAM("Could not retreive data for edge with name "<<*it_e);
            continue;
        }

        Graph::VertexPtr vs_ptr, ve_ptr;
        vs_ptr = m_graph.getVertexData(e_ptr->edge_vertices_name.first);
        ve_ptr = m_graph.getVertexData(e_ptr->edge_vertices_name.second);
        
        switch(e_ptr->edge_type)
        {
            case STRAIGHT_LINE: 
            {
                Graph::Curves::calculateEquiDistantPosesLine({vs_ptr->vertex_pose, ve_ptr->vertex_pose}, out_poses, 
                                                                e_ptr->edge_length, m_pose_spacing);
                
                e_ptr->forward_edge_poses.poses = out_poses;

                break;
            }

            case CUBIC_BEZIER:
            {
                std::vector<geometry_msgs::Pose> in_poses;
                in_poses.push_back(vs_ptr->vertex_pose);
                in_poses.push_back(e_ptr->control_poses.front());
                in_poses.push_back(e_ptr->control_poses.back());
                in_poses.push_back(ve_ptr->vertex_pose);

                Graph::Curves::calculateEquiDistantPosesBezier(in_poses, out_poses, e_ptr->edge_length, m_pose_spacing, m_resolution);
                e_ptr->forward_edge_poses.poses = out_poses;
                break;
            }
        }

        geometry_msgs::Pose fwd_dir_mrkr_pose, rev_dir_mrkr_pose;
        Graph::Visuals::calculateEdgeDirectionMarkerPose(out_poses, m_pose_spacing, fwd_dir_mrkr_pose, rev_dir_mrkr_pose);
        m_graph_marker_server.setPose(e_ptr->forward_direction_name, fwd_dir_mrkr_pose);
        if(e_ptr->is_bidirectional) m_graph_marker_server.setPose(e_ptr->reverse_direction_name, rev_dir_mrkr_pose);

        if(e_ptr->use_independent_orientation)
        {
            geometry_msgs::Pose ctrl_pose;
            Graph::Curves::calculateControlOrientForCurves(e_ptr->forward_edge_poses.poses, ctrl_pose);
            e_ptr->control_orientation_pose.position = ctrl_pose.position; //Update only the position
            m_graph_marker_server.setPose(e_ptr->control_orientation_name, e_ptr->control_orientation_pose);
            for(size_t i = 0; i < e_ptr->forward_edge_poses.poses.size(); ++i)
                e_ptr->forward_edge_poses.poses.at(i).orientation = e_ptr->control_orientation_pose.orientation;
        }

        makeInteractiveEdgeMarker(e_ptr->edge_name, e_ptr->forward_edge_poses.poses);
        publishAllPosesInGraph();
    }

    return true;
}

bool GraphCreator::updateEdgeAndMarkers(const std::string &edge_element_name, const geometry_msgs::Pose &edge_element_pose)
{
    //NOTE: Edge elements can be either control points or control orientation markers

    if(!m_graph.isMiscellaneous(edge_element_name))
    {
        ROS_WARN_STREAM("Cannot update edge as the given graph element "<<edge_element_name<<" is not relatated to any edge");
        return false;
    }

    Graph::MiscDepSet mds;
    m_graph.getMiscDependency(edge_element_name, mds);
    for(Graph::MiscDepSet::iterator it_m = mds.begin(); it_m != mds.end(); ++it_m)
    {
        if(m_graph.isEdge(*it_m))
        {
            Graph::EdgePtr e_ptr = m_graph.getEdgeData(*it_m);
            
            if(!e_ptr)
            {
                ROS_ERROR_STREAM("Update for edge "<<*it_m<<" could not be done as the data could not be retreived");
                continue;
            }

            std::vector<geometry_msgs::Pose> out_poses;

            switch(e_ptr->edge_type)
            {
                case STRAIGHT_LINE:
                {
                    //TODO: This can be converted into a common function for straight line updates
                    Graph::VertexPtr vs_ptr, ve_ptr;
                    vs_ptr = m_graph.getVertexData(e_ptr->edge_vertices_name.first);
                    ve_ptr = m_graph.getVertexData(e_ptr->edge_vertices_name.second);

                    Graph::Curves::calculateEquiDistantPosesLine({vs_ptr->vertex_pose, ve_ptr->vertex_pose}, out_poses, 
                                                                e_ptr->edge_length, m_pose_spacing);
                
                    e_ptr->forward_edge_poses.poses = out_poses;

                    break;
                }

                case CUBIC_BEZIER:
                {
                    Graph::MiscDepSet vs_mds, ve_mds;
                    m_graph.getMiscDependency(e_ptr->edge_vertices_name.first, vs_mds);
                    m_graph.getMiscDependency(e_ptr->edge_vertices_name.second, ve_mds);

                    geometry_msgs::Pose control_pose = edge_element_pose;
                    applyEdgeElementPositionSnap(edge_element_pose.position, control_pose.position);

                    if(vs_mds.find(edge_element_name) != vs_mds.end()) e_ptr->control_poses.front() = control_pose;
                    else if(ve_mds.find(edge_element_name) != ve_mds.end()) e_ptr->control_poses.back() = control_pose;
                    m_graph_marker_server.setPose(edge_element_name, control_pose);
                    
                    //TODO: Can be converted into a common function for lines and beziers
                    Graph::VertexPtr vs_ptr, ve_ptr;
                    vs_ptr = m_graph.getVertexData(e_ptr->edge_vertices_name.first);
                    ve_ptr = m_graph.getVertexData(e_ptr->edge_vertices_name.second);

                    std::vector<geometry_msgs::Pose> in_poses;
                    in_poses.push_back(vs_ptr->vertex_pose);
                    in_poses.push_back(e_ptr->control_poses.front());
                    in_poses.push_back(e_ptr->control_poses.back());
                    in_poses.push_back(ve_ptr->vertex_pose);

                    Graph::Curves::calculateEquiDistantPosesBezier(in_poses, out_poses, e_ptr->edge_length, m_pose_spacing, m_resolution);
                    e_ptr->forward_edge_poses.poses = out_poses;
                    break;
                }
            }
            
            //TODO: This will change if edge direction type field is used in the future
            geometry_msgs::Pose fwd_dir_mrkr_pose, rev_dir_mrkr_pose;
            Graph::Visuals::calculateEdgeDirectionMarkerPose(out_poses, m_pose_spacing, fwd_dir_mrkr_pose, rev_dir_mrkr_pose);
            m_graph_marker_server.setPose(e_ptr->forward_direction_name, fwd_dir_mrkr_pose);
            if(e_ptr->is_bidirectional) m_graph_marker_server.setPose(e_ptr->reverse_direction_name, rev_dir_mrkr_pose);

            if(e_ptr->use_independent_orientation)
            {
                geometry_msgs::Pose ctrl_pose;
                visualization_msgs::InteractiveMarker int_marker;

                //Update the position to place the control orientation marker
                Graph::Curves::calculateControlOrientForCurves(e_ptr->forward_edge_poses.poses, ctrl_pose);
                e_ptr->control_orientation_pose.position = ctrl_pose.position; //Update only the position
                
                m_graph_marker_server.get(e_ptr->control_orientation_name, int_marker);
                
                geometry_msgs::Quaternion final_ctrl_ornt = int_marker.pose.orientation;
                applyEdgeElementOrientationSnap(int_marker.pose.orientation, final_ctrl_ornt);
                
                e_ptr->control_orientation_pose.orientation = final_ctrl_ornt;

                m_graph_marker_server.setPose(e_ptr->control_orientation_name, e_ptr->control_orientation_pose);
                for(size_t i = 0; i < e_ptr->forward_edge_poses.poses.size(); ++i)
                    e_ptr->forward_edge_poses.poses.at(i).orientation = e_ptr->control_orientation_pose.orientation;
            }

            makeInteractiveEdgeMarker(e_ptr->edge_name, e_ptr->forward_edge_poses.poses);
            publishAllPosesInGraph();

            break;
        }
    }

    return true;
}

void GraphCreator::updateInteractiveMarkerPoses(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    if(m_graph.isVertex(feedback->marker_name))
    {
        updateVertexAndMarker(feedback->marker_name, feedback->pose);
    }
    else if(m_graph.isMiscellaneous(feedback->marker_name)) //Update graph elements relatied to miscellaneous markers (Mainly edges)
    {
        updateEdgeAndMarkers(feedback->marker_name, feedback->pose);
    }

    m_graph_marker_server.applyChanges();
}

bool GraphCreator::saveGraphToFile(const std::string &graph_name, const std::string &file_path)
{
    ROS_INFO_STREAM("Saving graph to "<<file_path<<" with graph name "<<graph_name);

    if(m_graph.isEmpty())
    {
        ROS_WARN_STREAM("Graph is currenly empty. Create at least one vertex to save the graph");
        return false;
    }

    GraphSaver gc(graph_name); //Filename is used as the graph name as well
    gc.convertGraph(m_graph);
    if(!gc.saveGraph(file_path))
    {
        ROS_ERROR_STREAM("Graph could not be saved to file");
        return false;
    }

    ROS_INFO_STREAM("Graph saved to "<<file_path);

    return true;
}

void GraphCreator::saveGraphMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM("Graph save from vertex menu");
    saveGraphToFile("point_path_graph", m_cur_graph_filename+".graphml"); //This is saved to the current working directory
}

bool GraphCreator::loadGraphFromFile(const std::string &file_path)
{
    ROS_INFO_STREAM("Loading graph from "<<file_path);

    GraphLoader gl;

    if(!gl.loadGraph(file_path))
    {
        ROS_ERROR_STREAM("Graph could not be loaded from file");
        return false;
    }

    Graph::VertexList vl;
    Graph::EdgeList el;

    if(!gl.getVerticesAndEdges(vl, el))
    {
        ROS_ERROR_STREAM("Could not retreive vertices and edges from loaded graph");
        return false;
    }

    std::string graph_name;
    gl.getGraphName(graph_name);
    
    ROS_INFO_STREAM("Loaded graph from file with name "<<graph_name);
    ROS_INFO_STREAM("Number of vertices: "<<vl.size());
    ROS_INFO_STREAM("Number of edges: "<<el.size());

    if(graph_name != m_cur_graph_name)
    {
        ROS_INFO_STREAM("Loaded a new graph from file");
        m_cur_graph_name = graph_name;
    }
    else ROS_INFO_STREAM("Loaded previous graph from file again");

    ROS_INFO_STREAM("Graph name: "<<m_cur_graph_name);

    std::string graph_type;
    gl.getGraphType(graph_type);
    if(graph_type == "directed")
    {
        m_cur_graph_type = graph_type;
        ROS_INFO_STREAM("Graph type: "<<m_cur_graph_type);
    }
    else
    {
        ROS_ERROR_STREAM("Only directed graphs are supported. Current graph is "<<graph_type);
        return false;
    }

    if(!createGraphWithInteractiveMarkers(vl, el))
    {
        ROS_ERROR_STREAM("Graph creation from loaded file failed");
        return false;
    }

    return true;
}

bool GraphCreator::createGraphWithInteractiveMarkers(Graph::VertexList &vl, Graph::EdgeList &el)
{
    //Trash any graph we currently have
    clearGraph();

    for(Graph::Vertex &v: vl)
    {
        if(!createVertexAndMarker(v))
        {
            ROS_FATAL_STREAM("Vertex creation from loaded graph failed");
            return false;
        }
    }

    for(Graph::Edge &e: el)
    {
        if(!createEdgeWithMarker(e))
        {
            ROS_FATAL_STREAM("Edge creation from loaded graph failed");
            return false;
        }
    }

    m_graph_marker_server.applyChanges();
    publishAllPosesInGraph();

    return true;
}

void GraphCreator::loadGraphMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM("Graph load from vertex menu");
    loadGraphFromFile(m_cur_graph_filename+".graphml"); //This is loaded from the current working directory
}

bool GraphCreator::loadGraphFileServiceCallback(graph_msgs::LoadGraph::Request &req, graph_msgs::LoadGraph::Response &res)
{
    std::string load_file_path = "";

    if(req.graph_url.size() == 0)
    {
        ROS_ERROR_STREAM("The load graph file path is empty");
        res.result = graph_msgs::LoadGraph::Response::RESULT_INVALID_GRAPH_FILE_PATH;
        return true;
    }

    if(!Graph::File::getGraphFilePath(req.graph_url, load_file_path))
    {
        ROS_ERROR_STREAM("The load graph file path is not valid");
        res.result = graph_msgs::LoadGraph::Response::RESULT_INVALID_GRAPH_FILE_PATH;
        return true;
    }

    if(!loadGraphFromFile(load_file_path))
    {
        res.result = graph_msgs::LoadGraph::Response::RESULT_UNDEFINED_FAILURE;
        return true;
    }

    graph_msgs::Graph graph_msg;
    Graph::Utilities::createGraphMessage(m_graph, m_cur_graph_name, graph_msg);

    res.graph = graph_msg;
    res.result = graph_msgs::LoadGraph::Response::RESULT_SUCCESS;
    return true;
}

bool GraphCreator::saveGraphFileServiceCallback(graph_msgs::SaveGraph::Request &req, graph_msgs::SaveGraph::Response &res)
{
    std::string save_file_path = "";

    if(req.save_graph_url.size() == 0)
    {
        ROS_ERROR_STREAM("The save graph file path is empty");
        res.result = graph_msgs::SaveGraph::Response::RESULT_INVALID_GRAPH_FILE_PATH;
        return true;
    }

    if(req.graph_file_name.size() == 0)
    {
        ROS_ERROR_STREAM("The save graph file name is empty");
        res.result = graph_msgs::SaveGraph::Response::RESULT_INVALID_GRAPH_FILE_NAME;
        return true;
    }

    if(!Graph::File::getGraphFilePath(req.save_graph_url, save_file_path))
    {
        ROS_ERROR_STREAM("The save graph file path is not valid");
        res.result = graph_msgs::SaveGraph::Response::RESULT_INVALID_GRAPH_FILE_PATH;
        return true;
    }
    
    save_file_path = (boost::filesystem::path(save_file_path)/req.graph_file_name).string()+".graphml"; //Update save path to include file name
    if(!saveGraphToFile(req.graph_file_name, save_file_path))
    {
        res.result = graph_msgs::SaveGraph::Response::RESULT_UNDEFINED_FAILURE;
        return true;
    }

    graph_msgs::Graph graph_msg;
    Graph::Utilities::createGraphMessage(m_graph, req.graph_file_name, graph_msg);

    res.graph = graph_msg;
    res.result = graph_msgs::SaveGraph::Response::RESULT_SUCCESS;
    return true;
}

void GraphCreator::publishGraph()
{
    ROS_INFO_STREAM("Publishing current graph from graph creator");

    if(m_graph.isEmpty())
    {
        ROS_WARN_STREAM("Cannot publish graph as the graph is empty");
        return;
    }

    graph_msgs::Graph graph_msg;
    Graph::Utilities::createGraphMessage(m_graph, m_cur_graph_name, graph_msg);

    ROS_INFO_STREAM("Number of vertices: "<<graph_msg.meta_data.number_of_vertices);
    ROS_INFO_STREAM("Number of edges: "<<graph_msg.meta_data.number_of_edges);

    m_graph_pub.publish(graph_msg);
}

void GraphCreator::setGraphMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM("Publish graph from vertex menu");

    std::string active_menu_entry;
    m_vert_menu_handler.getTitle(feedback->menu_entry_id, active_menu_entry);

    ROS_INFO_STREAM("Set graph selection: "<<active_menu_entry);

    if(active_menu_entry == "Publish current graph") publishGraph();
    else if(active_menu_entry == "Load and publish graph")
    {
        loadGraphFromFile(m_cur_graph_filename+".graphml");
        publishGraph();
    }
    else ROS_ERROR_STREAM("Unknown set graph selection selection was made");   
}

void GraphCreator::publishCurrentGraphVertex(const std::string &vertex_name)
{
    Graph::VertexPtr v_ptr = m_graph.getVertexData(vertex_name);

    if(!v_ptr)
    {
        ROS_ERROR_STREAM("Could not retrieve vertex data for publishing current vertex "<<vertex_name);
        return;
    }

    graph_msgs::Vertex vertex_msg;
    Graph::Utilities::convertVertexToMsg(v_ptr, vertex_msg);
    m_cur_vertex_pub.publish(vertex_msg);
}

void GraphCreator::publishCurrentGraphEdge(const std::string &edge_name)
{
    Graph::EdgePtr e_ptr = m_graph.getEdgeData(edge_name);

    if(!e_ptr)
    {
        ROS_ERROR_STREAM("Could not retrieve edge data for publishing current edge "<<edge_name);
        return;
    }

    graph_msgs::Edge edge_msg;
    Graph::Utilities::convertEdgeToMsg(e_ptr, edge_msg);
    m_cur_edge_pub.publish(edge_msg);
}

void GraphCreator::vertexUpdatesCallback(const graph_msgs::Vertex &vertex_msg)
{
    ROS_INFO_STREAM("Received vertex update for vertex "<<vertex_msg.name);

    Graph::VertexPtr v_ptr = m_graph.getVertexData(vertex_msg.name);

    if(!v_ptr)
    {
        ROS_WARN_STREAM("Cannot update vertex "<<vertex_msg.name<<" as it could not be found the graph");
        return;
    }

    Graph::Utilities::convertFromMsgToVertex(vertex_msg, v_ptr); //The position of the vertex does not change during update
    //TODO: A visual update will be required in the future were the vertex marker position and associated edges needs to be updated

    m_graph_marker_server.applyChanges();
}

void GraphCreator::edgeUpdatesCallback(const graph_msgs::Edge &edge_msg)
{
    ROS_INFO_STREAM("Received edge update for edge "<<edge_msg.name);

    Graph::EdgePtr e_ptr = m_graph.getEdgeData(edge_msg.name);

    if(!e_ptr)
    {
        ROS_WARN_STREAM("Cannot update edge "<<edge_msg.name<<" as it could not be found in the graph");
        return;
    }

    //FIXME: The bidirectional field is currently used but will be changed later to edge direction type
    bool bidirectional = false;
    if(edge_msg.edge_direction_type == graph_msgs::Edge::BIDIRECTIONAL)
        bidirectional = true;
    else if(edge_msg.edge_direction_type == graph_msgs::Edge::FORWARD)
        bidirectional = false;
    
    //Update if a change is actually required
    if(e_ptr->edge_type != edge_msg.type)
    {
        ROS_INFO_STREAM("The edge type was updated for edge "<<edge_msg.name);
        changeEdgeTypeWithMarkers(edge_msg.name, edge_msg.type);
    }

    if(e_ptr->is_bidirectional != bidirectional)
    {
        //FIXME: The bidirectional field is currently used but will be changed later to edge direction type
        ROS_INFO_STREAM("The edge direction type was updated for edge "<<edge_msg.name);
        changeEdgeDirectionWithMarkers(edge_msg.name, bidirectional);
    }

    uint8_t edge_dir_type = e_ptr->edge_direction_type; //FIXME: This reset is not required when moving to edge direction type field
    Graph::Utilities::convertFromMsgToEdge(edge_msg, e_ptr);
    e_ptr->edge_direction_type = edge_dir_type; //FIXME: This reset is not required when moving to edge direction type field

    m_graph_marker_server.applyChanges();
    
    //TODO: Independent orientation for edge
}

void GraphCreator::selectedGraphElementCallback(const graph_creator_msgs::SelectedGraphElements &sel_msg)
{
    deselectAllVerticesAndEdges();
    clearAllVertexAndEdgeSelections();

    lockAllVertices(false);
    lockAllEdges(false);

    for(const std::string& gph_elem_name: sel_msg.selected_elements)
    {
        if(m_graph.isVertex(gph_elem_name)) m_sel_grp_vertices_set.emplace(gph_elem_name);
        else if(m_graph.isEdge(gph_elem_name)) m_sel_grp_edges_set.emplace(gph_elem_name);
    }

    selectAllVerticesAndEdges();
    hideDefaultVertexMenu(true);
    hideSelectionVertexMenu(false);
    hideDefaultEdgeMenu(true);
    hideSelectionEdgeMenu(false);

    lockAllVertices(true);
    lockAllEdges(true);

    m_vert_menu_handler.reApply(m_graph_marker_server);
    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();

    publishCurrentGraphCreatorState();
}

void GraphCreator::stashMarkerColor(const visualization_msgs::InteractiveMarker &int_marker)
{
    m_sel_mkr_clr_queue.push_back({int_marker.name, int_marker.controls.back().markers.front().color});
}

void GraphCreator::applyStashedMarkerColors()
{
    visualization_msgs::InteractiveMarker int_marker;

    while(!m_sel_mkr_clr_queue.empty())
    {
        const auto &mkr_clr = m_sel_mkr_clr_queue.front();
        m_sel_mkr_clr_queue.pop_front();

        if(!m_graph_marker_server.get(mkr_clr.marker_name, int_marker)) continue;

        int_marker.controls.back().markers.front().color = mkr_clr.color;

        m_graph_marker_server.insert(int_marker);
    }
}

void GraphCreator::clearStashedMarkersColors()
{
    m_sel_mkr_clr_queue.clear();
}

void GraphCreator::selectVerticesOnly()
{
    visualization_msgs::InteractiveMarker int_marker;

    for(const std::string& vert_mrkr_name: m_sel_grp_vertices_set)
    {
        if(!m_graph_marker_server.get(vert_mrkr_name, int_marker)) continue;

        int_marker.controls.back().markers.front().color = Graph::Colors::getColorRGBAMsg(Graph::Colors::MAGENTA);

        m_graph_marker_server.insert(int_marker);
    }
}

void GraphCreator::deselectVerticesOnly()
{
    visualization_msgs::InteractiveMarker int_marker;

    for(const std::string& vert_mrkr_name: m_sel_grp_vertices_set)
    {
        if(!m_graph_marker_server.get(vert_mrkr_name, int_marker)) continue;

        int_marker.controls.back().markers.front().color = Graph::Colors::getColorRGBAMsg(Graph::Colors::RED);

        m_graph_marker_server.insert(int_marker);
    }
}

void GraphCreator::clearVertexSelections()
{
    m_sel_grp_vertices_set.clear();
}

void GraphCreator::selectEdgesOnly()
{
    visualization_msgs::InteractiveMarker int_marker;

    for(const std::string& edge_mrkr_name: m_sel_grp_edges_set)
    {
        if(!m_graph_marker_server.get(edge_mrkr_name, int_marker)) continue;

        int_marker.controls.back().markers.front().color = Graph::Colors::getColorRGBAMsg(Graph::Colors::MAGENTA);

        m_graph_marker_server.insert(int_marker);
    }
}

void GraphCreator::deselectEdgesOnly()
{
    visualization_msgs::InteractiveMarker int_marker;

    for(const std::string& edge_mrkr_name: m_sel_grp_edges_set)
    {
        if(!m_graph_marker_server.get(edge_mrkr_name, int_marker)) continue;

        int_marker.controls.back().markers.front().color = Graph::Colors::getColorRGBAMsg(Graph::Colors::GREEN);

        m_graph_marker_server.insert(int_marker);
    }
}

void GraphCreator::clearEdgeSelections()
{
    m_sel_grp_edges_set.clear();
}

void GraphCreator::selectAllVerticesAndEdges()
{
    selectVerticesOnly();
    selectEdgesOnly();
}

void GraphCreator::deselectAllVerticesAndEdges()
{
    deselectVerticesOnly();
    deselectEdgesOnly();
}

void GraphCreator::clearAllVertexAndEdgeSelections()
{
    m_sel_grp_vertices_set.clear();
    m_sel_grp_edges_set.clear();
}

void GraphCreator::resetEdgesAndVerticesSelections()
{
    deselectAllVerticesAndEdges();
    clearAllVertexAndEdgeSelections();
    
    hideDefaultVertexMenu(false);
    hideSelectionVertexMenu(true);
    
    hideDefaultEdgeMenu(false);
    hideSelectionEdgeMenu(true);

    lockAllVertices(false);
    lockAllEdges(false);
}

bool GraphCreator::setGraphCreatorModeCallback(graph_creator_msgs::SetGraphCreatorMode::Request &req,
                                               graph_creator_msgs::SetGraphCreatorMode::Response &res)
{
    m_current_graph_mode = req.mode.mode;

    if(m_current_graph_mode == CREATE_VERTICES)
    {
        m_vert_menu_handler.setCheckState(m_create_vertices_handle, interactive_markers::MenuHandler::CHECKED);
        m_vert_menu_handler.setCheckState(m_create_edges_handle, interactive_markers::MenuHandler::UNCHECKED);
        m_vert_mode_handle = m_create_vertices_handle;
        applyStashedMarkerColors();
        lockAllVertices(false);
    }
    else if(m_current_graph_mode == CREATE_EDGES)
    {
        m_vert_menu_handler.setCheckState(m_create_edges_handle, interactive_markers::MenuHandler::CHECKED);
        m_vert_menu_handler.setCheckState(m_create_vertices_handle, interactive_markers::MenuHandler::UNCHECKED);
        m_vert_mode_handle = m_create_edges_handle;
        applyStashedMarkerColors();
        lockAllVertices(true);
    }

    m_vert_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();

    publishCurrentGraphCreatorState();

    res.success = true;
    res.message = "Graph creator mode set successfully";

    return true;
}

bool GraphCreator::setEdgeCreationTypeCallback(graph_creator_msgs::SetEdgeCreationType::Request &req,
                                               graph_creator_msgs::SetEdgeCreationType::Response &res)
{
    m_current_curve_type = req.type.type;

    if(m_current_curve_type == STRAIGHT_LINE)
    {
        m_edge_menu_handler.setCheckState(m_edge_type_sl_handle, interactive_markers::MenuHandler::CHECKED);
        m_edge_menu_handler.setCheckState(m_edge_type_cb_handle, interactive_markers::MenuHandler::UNCHECKED);
        m_edge_type_handle = m_edge_type_sl_handle;
    }
    else if(m_current_curve_type == CUBIC_BEZIER)
    {
        m_edge_menu_handler.setCheckState(m_edge_type_cb_handle, interactive_markers::MenuHandler::CHECKED);
        m_edge_menu_handler.setCheckState(m_edge_type_sl_handle, interactive_markers::MenuHandler::UNCHECKED);
        m_edge_type_handle = m_edge_type_cb_handle;
    }

    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();

    publishCurrentGraphCreatorState();

    res.success = true;
    res.message = "Edge creation type set successfully";

    return true;
}


bool GraphCreator::setEdgeDirectionCreationTypeCallback(graph_creator_msgs::SetEdgeDirectionCreationType::Request &req,
                                                        graph_creator_msgs::SetEdgeDirectionCreationType::Response &res)
{
    //FIXME: The bidirectional field is currently used but will be changed later to edge direction type
    if(req.type.direction_type == graph_creator_msgs::EdgeDirectionCreationType::EDGE_DIRECTION_CREATION_TYPE_BIDIRECTIONAL)
        m_edge_is_bidirectional = true;
    else if(req.type.direction_type == graph_creator_msgs::EdgeDirectionCreationType::EDGE_DIRECTION_CREATION_TYPE_FORWARD)
        m_edge_is_bidirectional = false;

    if(m_edge_is_bidirectional)
    {
        m_edge_menu_handler.setCheckState(m_edge_dir_bi_handle, interactive_markers::MenuHandler::CHECKED);
        m_edge_menu_handler.setCheckState(m_edge_dir_fwd_handle, interactive_markers::MenuHandler::UNCHECKED);
        m_edge_dir_type_handle = m_edge_dir_bi_handle;
    }
    else
    {
        m_edge_menu_handler.setCheckState(m_edge_dir_fwd_handle, interactive_markers::MenuHandler::CHECKED);
        m_edge_menu_handler.setCheckState(m_edge_dir_bi_handle, interactive_markers::MenuHandler::UNCHECKED);
        m_edge_dir_type_handle = m_edge_dir_fwd_handle;
    }

    m_edge_menu_handler.reApply(m_graph_marker_server);
    m_graph_marker_server.applyChanges();

    publishCurrentGraphCreatorState();

    res.success = true;
    res.message = "Edge creation direction type set successfully";

    return true;
}

bool GraphCreator::setVerticesLockedCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    lockAllVertices(req.data);
    m_graph_marker_server.applyChanges();

    publishCurrentGraphCreatorState();

    res.success = true;
    res.message = "Locked all vertices";    

    return true;
}

bool GraphCreator::setEdgesLockedCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    lockAllEdges(req.data);
    m_graph_marker_server.applyChanges();

    publishCurrentGraphCreatorState();

    res.success = true;
    res.message = "Locked all edges";

    return true;
}

bool GraphCreator::getGraphCreatorStateCallback(graph_creator_msgs::GetGraphCreatorState::Request &req,
                                                graph_creator_msgs::GetGraphCreatorState::Response &res)
{
    //TODO: Better to get the individual state again instead of relying on the state set in the message?
    res.state = m_gc_state_msg;

    res.success = true;
    res.message = "Getting graph creator state successful";

    return true;
}

bool GraphCreator::generateGraphCallback(graph_creator_msgs::GenerateGraph::Request &req, 
                                         graph_creator_msgs::GenerateGraph::Response &res)
{
    ROS_INFO_STREAM("Generation graph service called");

    GraphGenerator gg(req.graph_generator_config);

    Graph::VertexList vl;
    Graph::EdgeList el;

    if(!gg.generateVerticesAndEdgesList(vl, el))
    {
        ROS_ERROR_STREAM("Graph generation failed");

        res.success = false;
        res.message = "Graph generation failed";

        return true;
    }

    if(!createGraphWithInteractiveMarkers(vl, el))
    {
        ROS_ERROR_STREAM("Graph generation from generated vertices and edges failed");

        res.success = false;
        res.message = "Graph generation from generated vertices and edges failed";

        return true;
    }

    res.success = true;
    res.message = "Graph generation successful";    

    return true;
}

bool GraphCreator::setSnapPoseConfigCallback(graph_creator_msgs::SetSnapPose::Request &req,
                                             graph_creator_msgs::SetSnapPose::Response &res)
{
    if(req.snap_pose_config.position_snap_grid_size <= 0 && 
       (req.snap_pose_config.orientation_snap_angle <= 0 || 
       req.snap_pose_config.orientation_snap_angle >= (2.0 * M_PI)))
    {
        std::stringstream err_ss;
        err_ss<<"Failed to set both snap position grid size and orientation.";
        err_ss<<" Snap position grid size needs to greated than 0";
        err_ss<<" and snap orientation angle needs to be between 0 and 2Pi";

        res.success = false;
        res.message = err_ss.str();
        ROS_ERROR_STREAM(res.message);

        return true;
    }

    if(req.snap_pose_config.position_snap_grid_size <= 0)
    {
        res.success = false;
        res.message = "Failed set snap position grid size. Snap position grid size needs to greater than 0";
        ROS_ERROR_STREAM(res.message);

        return true;
    }

    if(req.snap_pose_config.orientation_snap_angle <= 0 || req.snap_pose_config.orientation_snap_angle >= (2.0 * M_PI))
    {
        res.success = false;
        res.message = "Failed set snap orientation angle. Snap orientation angle needs to between 0 and 2Pi";
        ROS_ERROR_STREAM(res.message);

        return true;
    }

    m_enable_vertex_snap = req.snap_pose_config.enable_for_vertices;
    m_enable_edge_snap = req.snap_pose_config.enable_for_edges;

    m_enable_position_snap = req.snap_pose_config.enable_position_snap;
    m_enable_orientation_snap = req.snap_pose_config.enable_orientation_snap;

    m_position_snap_grid_size = req.snap_pose_config.position_snap_grid_size;
    m_orientation_snap_angle = req.snap_pose_config.orientation_snap_angle;

    res.success = true;
    res.message = "Successfully set snap configuration";

    return true;
}

void GraphCreator::publishCurrentGraphCreatorState()
{
    m_gc_state_msg.graph_creator_mode.mode = m_current_graph_mode;
    m_gc_state_msg.edge_creation_type.type = m_current_curve_type;

    //FIXME: Needs to use forward, reverse and bidirectional option
    if(m_edge_is_bidirectional) 
        m_gc_state_msg.edge_direction_creation_type.direction_type = graph_creator_msgs::EdgeDirectionCreationType::EDGE_DIRECTION_CREATION_TYPE_BIDIRECTIONAL;
    else 
        m_gc_state_msg.edge_direction_creation_type.direction_type = graph_creator_msgs::EdgeDirectionCreationType::EDGE_DIRECTION_CREATION_TYPE_FORWARD;

    m_gc_state_msg.snap_pose_config.enable_for_vertices = m_enable_vertex_snap;
    m_gc_state_msg.snap_pose_config.enable_for_edges = m_enable_edge_snap;
    m_gc_state_msg.snap_pose_config.enable_position_snap = m_enable_position_snap;
    m_gc_state_msg.snap_pose_config.enable_orientation_snap = m_enable_orientation_snap;
    m_gc_state_msg.snap_pose_config.position_snap_grid_size = m_position_snap_grid_size;
    m_gc_state_msg.snap_pose_config.orientation_snap_angle = m_orientation_snap_angle;

    m_gc_state_msg.all_vertices_locked = m_is_vertices_locked;
    m_gc_state_msg.all_edges_locked = m_is_edges_locked;

    //Publish graph creator state
    m_cur_gc_state_pub.publish(m_gc_state_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "graph_creator");

    GraphCreator gc;

    ros::spin();

    return EXIT_SUCCESS;
}
