#pragma once

#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

namespace Graph::Curves {

    //Function for straight lines
    void calculateEquiDistantPosesLine(const std::vector<geometry_msgs::Pose> &in_poses, std::vector<geometry_msgs::Pose> &eqd_poses,
                                       double &lenght, double pose_spacing);

    //Functions for cubic bezier curves
    inline void quadraticCurve(const tf2::Vector3 &p_1, const tf2::Vector3 &p_2, const tf2::Vector3 &p_3, double t, tf2::Vector3 &p_out);
    inline void cubicCurve(const tf2::Vector3 &p_1, const tf2::Vector3 &p_2, const tf2::Vector3 p_3, const tf2::Vector3 p_4, double t, tf2::Vector3 &p_out);
    
    void calculateControlPosesForBezier(const std::vector<geometry_msgs::Pose> &anchor_poses, std::vector<geometry_msgs::Pose> &in_poses,
                                        std::vector<geometry_msgs::Pose> &control_poses);

    void calculateControlOrientForCurves(const std::vector<geometry_msgs::Pose> &in_poses, geometry_msgs::Pose &ctrl_orient_pose); 

    void calculateEquiDistantPosesBezier(const std::vector<geometry_msgs::Pose> &in_poses, std::vector<geometry_msgs::Pose> &eqd_poses, 
                                         double &length, double pose_spacing, double resolution);

}