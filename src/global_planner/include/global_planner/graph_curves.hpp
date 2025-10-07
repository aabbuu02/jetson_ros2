/**
 * @file graph_curves.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#pragma once

#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace Graph::Curves {
    void calculateEquiDistantPosesLine(const std::vector<geometry_msgs::msg::Pose> &in_poses, std::vector<geometry_msgs::msg::Pose> &eqd_poses, double &length, double pose_spacing);
    void calculateEquiDistantPosesArc(const std::vector<geometry_msgs::msg::Pose> &in_poses, std::vector<geometry_msgs::msg::Pose> &eqd_poses, double &length, double pose_spacing);
    void calculateEquiDistantPosesBezier(const std::vector<geometry_msgs::msg::Pose> &in_poses, std::vector<geometry_msgs::msg::Pose> &eqd_poses, double &length, double pose_spacing);
}
