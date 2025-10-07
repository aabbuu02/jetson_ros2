/**
 * @file graph_curves.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "global_planner/graph_curves.hpp"
#include <cmath>

namespace Graph::Curves {
    void calculateEquiDistantPosesLine(const std::vector<geometry_msgs::msg::Pose> &in_poses, std::vector<geometry_msgs::msg::Pose> &eqd_poses, double &length, double pose_spacing)
    {
        eqd_poses.clear();
        if (in_poses.size() < 2) return;
        tf2::Vector3 p1, p2;
        tf2::fromMsg(in_poses.front().position, p1);
        tf2::fromMsg(in_poses.back().position, p2);
        length = p1.distance(p2);
        if (length < 1e-6) {
            eqd_poses.push_back(in_poses.front());
            return;
        }
        int divisions = std::ceil(length / pose_spacing);
        tf2::Vector3 v = (p2 - p1).normalize();
        tf2::Quaternion q;
        q.setRPY(0, 0, std::atan2(v.getY(), v.getX()));
        for (int i = 0; i < divisions; ++i) {
            geometry_msgs::msg::Pose pose;
            tf2::toMsg(p1 + (v * i * pose_spacing), pose.position);
            pose.orientation = tf2::toMsg(q);
            eqd_poses.push_back(pose);
        }
        eqd_poses.push_back(in_poses.back());
        eqd_poses.back().orientation = tf2::toMsg(q);
    }
    
    inline void quadraticCurve(const tf2::Vector3 &p_1, const tf2::Vector3 &p_2, const tf2::Vector3 &p_3, double t, tf2::Vector3 &p_out) {
        p_out = (1.0-t)*(1.0-t)*p_1 + 2.0*(1.0-t)*t*p_2 + t*t*p_3;
    }

    inline void cubicCurve(const tf2::Vector3 &p_1, const tf2::Vector3 &p_2, const tf2::Vector3 p_3, const tf2::Vector3 p_4, double t, tf2::Vector3 &p_out) {
        tf2::Vector3 p_1_2_3, p_2_3_4;
        quadraticCurve(p_1, p_2, p_3, t, p_1_2_3);
        quadraticCurve(p_2, p_3, p_4, t, p_2_3_4);
        p_out = tf2::lerp(p_1_2_3, p_2_3_4, t);
    }

    void calculateEquiDistantPosesBezier(const std::vector<geometry_msgs::msg::Pose> &in_poses, std::vector<geometry_msgs::msg::Pose> &eqd_poses, double &length, double pose_spacing, double resolution)
    {
        eqd_poses.clear();
        if (in_poses.size() != 4) return;
        std::vector<tf2::Vector3> in_points;
        for (const auto& p : in_poses) {
            tf2::Vector3 point;
            tf2::fromMsg(p.position, point);
            in_points.push_back(point);
        }

        std::vector<tf2::Vector3> eqd_points;
        eqd_points.push_back(in_points.front());
        tf2::Vector3 prev_point = in_points.front();
        double dst_since_last = 0.0;
        int divisions = std::ceil(100.0 * resolution);
        
        for (int i = 1; i <= divisions; ++i) {
            double t = static_cast<double>(i) / divisions;
            tf2::Vector3 point_on_curve;
            cubicCurve(in_points[0], in_points[1], in_points[2], in_points[3], t, point_on_curve);
            dst_since_last += prev_point.distance(point_on_curve);
            
            while (dst_since_last >= pose_spacing) {
                double overshoot = dst_since_last - pose_spacing;
                tf2::Vector3 new_eqd_point = point_on_curve + (prev_point - point_on_curve).normalize() * overshoot;
                eqd_points.push_back(new_eqd_point);
                dst_since_last = overshoot;
                prev_point = new_eqd_point;
            }
            prev_point = point_on_curve;
        }
        eqd_points.push_back(in_points.back());
        
        length = 0.0;
        for (size_t i = 0; i < eqd_points.size(); ++i) {
            geometry_msgs::msg::Pose pose;
            tf2::toMsg(eqd_points[i], pose.position);
            if (i < eqd_points.size() - 1) {
                tf2::Vector3 tangent = (eqd_points[i+1] - eqd_points[i]).normalize();
                tf2::Quaternion q;
                q.setRPY(0, 0, std::atan2(tangent.y(), tangent.x()));
                pose.orientation = tf2::toMsg(q);
                length += eqd_points[i].distance(eqd_points[i+1]);
            } else if (!eqd_poses.empty()){
                pose.orientation = eqd_poses.back().orientation; // Use previous orientation
            }
            eqd_poses.push_back(pose);
        }
    }
}


