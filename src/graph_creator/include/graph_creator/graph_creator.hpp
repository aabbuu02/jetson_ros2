#pragma once

#include <string>
#include <sstream>
#include <deque>
#include <unordered_map>
#include <unordered_set>

#include <ros/ros.h>
#include <graph_msgs/Graph.h>
#include <graph_msgs/GraphMetaData.h>
#include <graph_msgs/LoadGraph.h>
#include <graph_msgs/SaveGraph.h>
#include <std_srvs/SetBool.h>
#include <graph_creator_msgs/GraphCreatorState.h>
#include <graph_creator_msgs/SetGraphCreatorMode.h>
#include <graph_creator_msgs/SetEdgeCreationType.h>
#include <graph_creator_msgs/SetEdgeDirectionCreationType.h>
#include <graph_creator_msgs/GetGraphCreatorState.h>
#include <graph_creator_msgs/GenerateGraph.h>
#include <graph_creator_msgs/SelectedGraphElements.h>
#include <graph_creator_msgs/SetSnapPose.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include "directed_graph.hpp"
#include "graph_curves.hpp"
#include "graph_colors.hpp"
#include "graph_visuals.hpp"
#include "graph_saver.hpp"
#include "graph_loader.hpp"
#include "graph_utils.hpp"
#include "graph_file_utils.hpp"
#include "graph_generators.hpp"

class GraphCreator
{
    //TODO: A FSM needs to be implemented if more complex modes are introduced
    public:

    GraphCreator();
    ~GraphCreator();

    void initializeRobotPose();

    private:

    //Node
    ros::NodeHandle m_nh;

    //Messages
    geometry_msgs::PoseArray m_fwd_pose_arr_msg, m_bck_pose_arr_msg;
    geometry_msgs::TransformStamped m_map_base_link_tf_msg;
    geometry_msgs::Pose m_robot_pose_msg;
    graph_creator_msgs::GraphCreatorState m_gc_state_msg;

    //Publishers
    ros::Publisher m_graph_pub;
    ros::Publisher m_cur_vertex_pub;
    ros::Publisher m_cur_edge_pub;
    ros::Publisher m_cur_gc_state_pub;
    ros::Publisher m_fwd_pose_arr_pub;
    ros::Publisher m_bck_pose_arr_pub;

    //Subscribers
    ros::Subscriber m_vertex_create_sub;
    ros::Subscriber m_robot_pose_sub;
    ros::Subscriber m_vertex_updates_sub;
    ros::Subscriber m_edge_updates_sub;
    ros::Subscriber m_selected_graph_elem_sub;

    //Subscriber callbacks
    void vertexCreateCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg);
    void robotPoseCallback(const geometry_msgs::PoseConstPtr &pose_msg);
    void vertexUpdatesCallback(const graph_msgs::Vertex &vertex_msg);
    void edgeUpdatesCallback(const graph_msgs::Edge &edge_msg);
    void selectedGraphElementCallback(const graph_creator_msgs::SelectedGraphElements &sel_msg);

    //Service servers
    ros::ServiceServer m_load_graph_srv, m_save_graph_srv;
    ros::ServiceServer m_set_gc_mode_srv, m_set_ec_type_srv, m_set_ed_type_srv;
    ros::ServiceServer m_set_verts_locked_srv, m_set_edges_locked_srv;
    ros::ServiceServer m_get_gc_state_srv;
    ros::ServiceServer m_gen_graph_srv;
    ros::ServiceServer m_set_snap_pose_srv;

    //Service server callbacks
    bool loadGraphFileServiceCallback(graph_msgs::LoadGraph::Request &req, graph_msgs::LoadGraph::Response &res);
    bool saveGraphFileServiceCallback(graph_msgs::SaveGraph::Request &req, graph_msgs::SaveGraph::Response &res);

    bool setGraphCreatorModeCallback(graph_creator_msgs::SetGraphCreatorMode::Request &req,
                                     graph_creator_msgs::SetGraphCreatorMode::Response &res);

    bool setEdgeCreationTypeCallback(graph_creator_msgs::SetEdgeCreationType::Request &req,
                                     graph_creator_msgs::SetEdgeCreationType::Response &res);

    bool setEdgeDirectionCreationTypeCallback(graph_creator_msgs::SetEdgeDirectionCreationType::Request &req,
                                              graph_creator_msgs::SetEdgeDirectionCreationType::Response &res);

    bool setVerticesLockedCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool setEdgesLockedCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    bool getGraphCreatorStateCallback(graph_creator_msgs::GetGraphCreatorState::Request &req,
                                      graph_creator_msgs::GetGraphCreatorState::Response &res);

    bool generateGraphCallback(graph_creator_msgs::GenerateGraph::Request &req, 
                               graph_creator_msgs::GenerateGraph::Response &res);

    bool setSnapPoseConfigCallback(graph_creator_msgs::SetSnapPose::Request &req,
                                   graph_creator_msgs::SetSnapPose::Response &res);
    
    //Time
    ros::Time m_prev_robot_pose_time;

    //TF broadcasters
    bool m_enable_base_link_tf_br = false;
    tf2_ros::StaticTransformBroadcaster m_map_base_link_br;

    void mapBaseLinkTfBroadcast(const geometry_msgs::Pose &pose_msg);

    //Graph saving and loading
    std::string m_cur_graph_filename = "point_path_graph";
    std::string m_cur_graph_path = "point_path_graph";
    std::string m_cur_graph_name = ""; //This can be same as the filename
    std::string m_cur_graph_type = "";

    //Graph
    Graph::DiGraph m_graph;

    //TODO: These enums should be removed and uses the graph message constants
    enum GraphCreateMode
    {
        CREATE_VERTICES = 0,
        CREATE_EDGES = 1
    };

    int m_current_graph_mode = CREATE_VERTICES;
    
    //Graph edge curve types
    enum CurveTypes
    {
        STRAIGHT_LINE = 0,
        CUBIC_BEZIER = 1
    };

    //Graph edge direction types
    enum DirectionTypes
    {
        FORWARD = 0,
        REVERESE = 1,
        BIDIRECTIONAL = 2
    };

    std::unordered_map<int8_t, std::string> curve_name_map = 
    {
        {STRAIGHT_LINE, "straight line"},
        {CUBIC_BEZIER, "cubic bezier"}
    };

    //Graph vertex properties
    uint8_t m_current_vertex_type = graph_msgs::Vertex::NORMAL;
    bool m_vertex_ignore_orientation = false;
    
    //Graph edge curve properties
    uint8_t m_current_curve_type = STRAIGHT_LINE;
    bool m_edge_is_bidirectional = false; //TODO: This needs to be changed to depend only on edge direction type
    double m_pose_spacing = 0.1;
    double m_resolution = 1.0;

    //Vertex variables
    bool m_is_vertices_locked = false;
    uint32_t m_vertex_id_counter = 0;
    std::string m_prev_selected_vertex = "";
    bool m_is_robot_pose_received = false;

    //Edge variables
    bool m_is_edges_locked = false;
    uint32_t m_control_id_counter = 0;
    std::string m_prev_selected_edge = "";

    //Snap pose variables
    bool m_enable_vertex_snap = false;
    bool m_enable_edge_snap = false;     //Elements of edges will also be snapped to grid
    bool m_enable_position_snap = false;
    bool m_enable_orientation_snap = false;
    double m_position_snap_grid_size = 1.0f;  //Grid size of 1m
    double m_orientation_snap_angle = M_PI/2; //Angle of 90 degrees

    //Selection variables for edge creation
    std::vector<std::string> m_selected_edge_vertices; //For selecting vertices for edge creation

    //Graph element selection for color change
    struct MarkerColor
    {
        std::string marker_name="";
        std_msgs::ColorRGBA color;

        MarkerColor(const std::string &mn, const std_msgs::ColorRGBA c):
            marker_name(mn), color(c)
        {}
    };

    std::deque<MarkerColor> m_sel_mkr_clr_queue;

    void stashMarkerColor(const visualization_msgs::InteractiveMarker &int_marker);
    void applyStashedMarkerColors();
    void clearStashedMarkersColors();

    //Selection for graph manipulation
    //Selected vertices
    std::unordered_set<std::string> m_sel_grp_vertices_set; //Group of vertices

    void selectVerticesOnly();
    void deselectVerticesOnly();
    void clearVertexSelections();

    //Selected edges
    std::unordered_set<std::string> m_sel_grp_edges_set; //Group of edges

    void selectEdgesOnly();
    void deselectEdgesOnly();
    void clearEdgeSelections();

    //Edges and vertices selection common function
    void selectAllVerticesAndEdges();
    void deselectAllVerticesAndEdges();
    void clearAllVertexAndEdgeSelections();

    void resetEdgesAndVerticesSelections();

    //Vertex menu handlers
    interactive_markers::MenuHandler m_vert_menu_handler;
    interactive_markers::MenuHandler::EntryHandle m_vert_mode_handle, m_vert_ornt_ign_handle;
    interactive_markers::MenuHandler::EntryHandle m_create_vertices_handle, m_create_edges_handle;

    std::vector<interactive_markers::MenuHandler::EntryHandle> m_default_vert_entry_handles;
    std::vector<interactive_markers::MenuHandler::EntryHandle> m_select_vert_entry_handles;

    void initVertexMenu();
    void vertexMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void vertexModeCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void vertexOrientationMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void vertexOrienationChangeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    
    void vertexSelectionMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    
    void deleteVerticesMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void verticesOrientationChangeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void hideDefaultVertexMenu(bool hide);
    void hideSelectionVertexMenu(bool hide);

    //Set vertex pose to robot pose
    void vertexPoseToRobotPoseMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    //Edge menu handlers
    interactive_markers::MenuHandler m_edge_menu_handler;
    interactive_markers::MenuHandler::EntryHandle m_edge_type_handle, m_edge_dir_type_handle;
    interactive_markers::MenuHandler::EntryHandle m_edge_type_sl_handle, m_edge_type_cb_handle;
    interactive_markers::MenuHandler::EntryHandle m_edge_dir_fwd_handle, m_edge_dir_bi_handle;

    std::vector<interactive_markers::MenuHandler::EntryHandle> m_default_edge_entry_handles;
    std::vector<interactive_markers::MenuHandler::EntryHandle> m_select_edge_entry_handles;

    void initEdgeMenu();
    void edgeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void edgeTypeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void edgeDirectionTypeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void edgeTypeChangeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void edgeDirectionChangeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void edgeSelectionMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void deleteEdgesMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void edgeTypesChangeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void edgeDirectionsChangeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void edgeOrientationsChangeMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void hideDefaultEdgeMenu(bool hide);
    void hideSelectionEdgeMenu(bool hide);

    //Interactive marker server for the graph vertices and edges
    interactive_markers::InteractiveMarkerServer m_graph_marker_server;

    //Function for vertex markers
    void addVertexWithMarker(int vertex_id, const geometry_msgs::Pose &pose);
    void deleteVertexWithMarker(const std::string &vertex_name);
    void deleteVerticesWithMarkers(const std::unordered_set<std::string> &vertex_names);
    void changeVertexOrientationWithMarker(const std::string &vertex_name, bool ignore_orientation);
    void changeVerticesOrientationWithMarkers(const std::unordered_set<std::string> &vertex_names, bool ignore_orientations);
    bool createVertexAndMarker(const Graph::Vertex &vertex);

    // Function for edge markers
    bool createEdgeWithMarker(Graph::Edge &edge);
    void makeInteractiveEdgeMarker(const std::string &edge_name, const std::vector<geometry_msgs::Pose> &poses);

    void addEdgeWithMarkers(const std::vector<std::string> &selected_vertices, uint8_t edge_type);
    void deleteEdgeWithMarkers(const std::string &edge_name);
    void deleteEdgesWithMarkers(const std::unordered_set<std::string> &edge_names);
    void changeEdgeTypeWithMarkers(const std::string &edge_name, uint8_t edge_type);
    void changeEdgeTypesWithMarkers(const std::unordered_set<std::string> &edge_names, uint8_t edge_type);
    void changeEdgeDirectionWithMarkers(const std::string &edge_name, bool is_bidirectional);
    void changeEdgeDirectionsWithMarkers(const std::unordered_set<std::string> &edge_names, bool is_bidirectional);
    inline void createEdgeAndMarkers(std::vector<Graph::VertexPtr> vertices, Graph::Edge &edge); //Existing vertices with new edge data

    //Function for control points or poses markers
    void makeInteractiveControlPoseMarker(const std::string &control_pose_name, const geometry_msgs::Pose &pose);

    void clearControlPoseMarkersForEdge(const std::string &edge_name);
    void readdControlPoseMarkersForEdge(const std::string &edge_name);

    //Function for control orientation markers
    void makeInteractiveControlOrientMarker(const std::string &control_orient_name, const geometry_msgs::Pose &pose);

    void addControlOrientMarkerToEdge(const std::string &edge_name);
    void addControlOrientMarkerToEdges(const std::unordered_set<std::string> edge_names);
    void deleteControlOrientMarkerFromEdge(const std::string &edge_name);
    void deleteControlOrientMarkerFromEdges(const std::unordered_set<std::string> edge_names);

    void clearControlOrientMarkersForEdge(const std::string &edge_name);
    void readdControlOrientMarkerForEdge(const std::string &edge_name);

    //Functions for edge directionality markers
    void makeInteractiveEdgeDirectionMarkers(const std::string &edge_name, const geometry_msgs::Pose &pose, int16_t marker_type);

    //Clearing the constructed graph and required variables
    void clearGraph();
    void reset();
    
    //Function to update markers and other elements based on interaction
    void processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    geometry_msgs::Point snapPosition(const geometry_msgs::Point &position, double snap_grid=1.0f) const;
    geometry_msgs::Quaternion snapOrientation(const geometry_msgs::Quaternion &orientation, double snap_angle=M_PI/2.0) const;
    
    bool applyVertexPositionSnap(const geometry_msgs::Point &in_position, geometry_msgs::Point &out_position);
    bool applyVertexOrientationSnap(const geometry_msgs::Quaternion &in_orientation, geometry_msgs::Quaternion &out_orientation);
    bool applyEdgeElementPositionSnap(const geometry_msgs::Point &in_position, geometry_msgs::Point &out_position);
    bool applyEdgeElementOrientationSnap(const geometry_msgs::Quaternion &in_orientation, geometry_msgs::Quaternion &out_orientation);
    
    bool updateVertexAndMarker(const std::string &vertex_name, const geometry_msgs::Pose &vertex_pose);
    bool updateEdgeAndMarkers(const std::string &edge_element_name, const geometry_msgs::Pose &edge_element_pose);
    void updateInteractiveMarkerPoses(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void lockAllVertices(bool lock);
    void lockAllEdges(bool lock);
    
    void publishAllPosesInGraph();
    
    //User interaction handlers
    void onMouseUp(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void onMouseDown(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void onButtonClick(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    //Save graph to file in GraphML format
    bool saveGraphToFile(const std::string &graph_name, const std::string &file_path);

    void saveGraphMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    //Loading graph from file in GraphML format
    bool loadGraphFromFile(const std::string &file_path); 
    bool createGraphWithInteractiveMarkers(Graph::VertexList &vl, Graph::EdgeList &el);

    void loadGraphMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    //Publish graph that is being created
    void publishGraph();
    
    void setGraphMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    //Currently active vertex or edge publishers
    void publishCurrentGraphVertex(const std::string &vertex_name);
    void publishCurrentGraphEdge(const std::string &edge_name);

    //Graph creator state
    void publishCurrentGraphCreatorState();
};
