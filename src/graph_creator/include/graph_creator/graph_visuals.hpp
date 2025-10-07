#pragma once

#include <vector>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include "graph_colors.hpp"


namespace Graph::Visuals
{
    //Graph marker types
    enum GraphMarkerTypes
    {
        VERTEX_NON_ORIENTED = 0,
        VERTEX_ORIENTED = 1,
        EDGE = 2,
        CONTROL_POSE = 3,
        CONTROL_ORIENTATION = 4,
        FORWARD_DIRECTION_ARROW = 5,
        REVERSE_DIRECTION_ARROW = 6
    };

    //Common marker and controls
    visualization_msgs::InteractiveMarkerControl makeButtonControl(visualization_msgs::InteractiveMarker& int_marker, visualization_msgs::Marker& marker);

    //For vertices

    std::string generateVertexMarkerName(int vertex_id);
    visualization_msgs::InteractiveMarker makeInteractiveVertexMarker(int vertex_id, const geometry_msgs::Pose &pose);
    visualization_msgs::Marker makeVertexSphereMarker();
    visualization_msgs::Marker makeVertexOrientationArrowMarker();
    visualization_msgs::InteractiveMarkerControl makeVertexSphereControls(visualization_msgs::InteractiveMarker &int_marker);
    visualization_msgs::InteractiveMarkerControl makeVertexArrowControls(visualization_msgs::InteractiveMarker &int_marker);

    //For edges

    std::string generateEdgeMarkerName(int start_vertex_id, int end_vertex_id);
    visualization_msgs::InteractiveMarker makeInteractiveEdgeMarker(const std::string &edge_name);
    visualization_msgs::Marker makeEdgePointMarker(const std::vector<geometry_msgs::Pose>& poses);
    visualization_msgs::InteractiveMarkerControl makeEdgePointControls(visualization_msgs::InteractiveMarker& int_marker,
                                                                        const std::vector<geometry_msgs::Pose>& poses);

    //Function for control points or poses markers
    std::string generateControlPoseMarkerName(int control_pose_id,const std::string &edge_name);
    visualization_msgs::InteractiveMarker makeInteractiveControlPoseMarker(const std::string &control_pose_name, const geometry_msgs::Pose &pose);
    visualization_msgs::Marker makeControlPoseSphereMarker();
    visualization_msgs::InteractiveMarkerControl makeControlPoseControls(visualization_msgs::InteractiveMarker& int_marker);

    //Function for control orientation markers
    std::string generateControlOrientationMarkerName(const std::string &edge_name);
    visualization_msgs::InteractiveMarker makeInteractiveControlOrientMarker(const std::string &control_orient_name, const geometry_msgs::Pose &pose);
    visualization_msgs::Marker makeControlOrientArrowMarker();
    visualization_msgs::InteractiveMarkerControl makeControlOrientControls(visualization_msgs::InteractiveMarker& int_marker);

    void calculateControlOrientationMarkerPose(const std::vector<geometry_msgs::Pose> &poses, geometry_msgs::Pose &ctrl_orient_pose);

    //Functions for edge directionality markers
    std::string generateEdgeDirectionMarkerName(const std::string &edge_name, int16_t marker_type);
    visualization_msgs::InteractiveMarker makeInteractiveEdgeDirectionMarkers(const std::string &edge_name, const geometry_msgs::Pose &pose, int16_t marker_type);
    visualization_msgs::Marker makeEdgeDirectionArrowMarker(int16_t marker_type); //Based on directionality arrow color changes
    visualization_msgs::InteractiveMarkerControl makeEdgeDirectionMarkerControls(visualization_msgs::InteractiveMarker& int_marker,
                                                                                 int16_t marker_type);

    void calculateEdgeDirectionMarkerPose(const std::vector<geometry_msgs::Pose> &poses, double pose_spacing, 
                                          geometry_msgs::Pose &fwd_dir_pose, geometry_msgs::Pose &rev_dir_pose);
}