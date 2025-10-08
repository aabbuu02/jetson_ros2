#include <graph_creator/graph_curves.hpp>

namespace Graph::Curves {

    void calculateEquiDistantPosesLine(const std::vector<geometry_msgs::Pose> &in_poses, std::vector<geometry_msgs::Pose> &eqd_poses,
                                       double &length, double pose_spacing)
    {
        eqd_poses.clear();

        std::vector<tf2::Vector3> in_points;

        for(const auto ip: in_poses)
        {
            tf2::Vector3 in_point;
            tf2::fromMsg(ip.position, in_point);
            in_points.push_back(in_point);
        }

        double distance = length = in_points.front().distance(in_points.back());
        int divisions = std::ceil(distance/pose_spacing);
        
        tf2::Vector3 unit_vec = (in_points.back() - in_points.front()).normalize();
        double yaw = std::atan2(unit_vec.getY(), unit_vec.getX());
        tf2::Quaternion quat, quat_rot;
        quat.setRPY(0.0, 0.0, yaw);
        // quat_rot.setRPY(0.0, 0.0, M_PI);
        // quat = quat_rot *quat;

        for(int i = 0; i < divisions; ++i)
        {
            geometry_msgs::Pose pose;
            tf2::Vector3 point_on_line = in_points.front() + (unit_vec * i * pose_spacing);
            tf2::toMsg(point_on_line, pose.position);
            tf2::convert(quat.normalize(), pose.orientation);
            eqd_poses.push_back(pose);
        }

        eqd_poses.push_back(in_poses.back());
        tf2::convert(quat.normalize(), eqd_poses.back().orientation); //Set the end pose orientation in the direction of the vector as well
    }

    inline void quadraticCurve(const tf2::Vector3 &p_1, const tf2::Vector3 &p_2, const tf2::Vector3 &p_3, double t, tf2::Vector3 &p_out)
    {
        tf2::Vector3 p_1_2 = tf2::lerp(p_1, p_2, t);
        tf2::Vector3 p_2_3 = tf2::lerp(p_2, p_3, t);
        p_out = tf2::lerp(p_1_2, p_2_3, t);
    }

    inline void cubicCurve(const tf2::Vector3 &p_1, const tf2::Vector3 &p_2, const tf2::Vector3 p_3, const tf2::Vector3 p_4, double t, tf2::Vector3 &p_out)
    {
        tf2::Vector3 p_1_2_3, p_2_3_4;
        quadraticCurve(p_1, p_2, p_3, t, p_1_2_3);
        quadraticCurve(p_2, p_3, p_4, t, p_2_3_4);
        p_out = tf2::lerp(p_1_2_3, p_2_3_4, t);
    }

    void calculateControlPosesForBezier(const std::vector<geometry_msgs::Pose> &anchor_poses, std::vector<geometry_msgs::Pose> &in_poses,
                                        std::vector<geometry_msgs::Pose> &control_poses)
    {
        in_poses.clear();
        control_poses.clear();

        std::vector<tf2::Vector3> anchor_points;
        
        for(const auto ap: anchor_poses)
        {
            tf2::Vector3 anchor_point;
            tf2::fromMsg(ap.position, anchor_point);
            anchor_points.push_back(anchor_point);
        }

        double distance_ap = anchor_points.front().distance(anchor_points.back());
        tf2::Vector3 unit_vec_ap = (anchor_points.back() - anchor_points.front()).normalize();

        tf2::Vector3 cp1, cp2;
        geometry_msgs::Pose control_pose;

        cp1 = anchor_points.front() + (unit_vec_ap * distance_ap * 0.25);
        cp2 = anchor_points.back() - (unit_vec_ap * distance_ap * 0.25);
        
        in_poses.push_back(anchor_poses.front());

        tf2::toMsg(cp1, control_pose.position);
        control_poses.push_back(control_pose);
        in_poses.push_back(control_pose);

        tf2::toMsg(cp2, control_pose.position);
        control_poses.push_back(control_pose);
        in_poses.push_back(control_pose);

        in_poses.push_back(anchor_poses.back());
    }

    void calculateControlOrientForCurves(const std::vector<geometry_msgs::Pose> &in_poses, geometry_msgs::Pose &ctrl_orient_pose)
    {
        //Calculates the control orientation pose for the Bezier curve
        int pose_list_size = in_poses.size();

        if(pose_list_size % 2 == 0) //For even number of poses
        {
            int first_idx, second_idx;
            first_idx =  std::ceil(pose_list_size/2.0);
            second_idx = first_idx + 1;

            tf2::Vector3 unit_vec, vec_1, vec_2;
            tf2::fromMsg(in_poses.at(first_idx).position, vec_1);
            tf2::fromMsg(in_poses.at(second_idx).position, vec_2);

            //Place in the middle between the two poses
            tf2::toMsg(tf2::lerp(vec_1, vec_2, 0.5), ctrl_orient_pose.position);
            ctrl_orient_pose.orientation = in_poses.at(first_idx).orientation;
        }
        else //For odd number of poses
        {
            int req_idx = std::ceil(pose_list_size/2.0);
            ctrl_orient_pose = in_poses.at(req_idx);
        }
    }

    void calculateEquiDistantPosesBezier(const std::vector<geometry_msgs::Pose> &in_poses, std::vector<geometry_msgs::Pose> &eqd_poses, 
                                         double &length, double pose_spacing, double resolution)
    {
        eqd_poses.clear();

        std::vector<tf2::Vector3> in_points;
        for(const auto cp: in_poses)
        {
            tf2::Vector3 control_point;
            tf2::fromMsg(cp.position, control_point);
            in_points.push_back(control_point);
        }

        std::vector<tf2::Vector3> eqd_points;
        eqd_points.push_back(in_points.front());
        tf2::Vector3 prev_point = in_points.front();
        double dst_since_last_eqd_point = 0.0;

        double control_net_length = 0.0;
        for(size_t i = 0; i < in_points.size()-1; ++i)
            control_net_length += in_points.at(i).distance(in_points.at(i+1));

        double est_curve_length = in_points.front().distance(in_points.back()) + control_net_length/2.0;
        int divisions = std::ceil(est_curve_length * resolution * 10.0);

        for(double t = 0.0; t < 1.0; t+=1.0/divisions)
        {
            tf2::Vector3 point_on_curve;
            cubicCurve(in_points.front(), in_points.at(1), in_points.at(2), in_points.back(), t, point_on_curve);
            dst_since_last_eqd_point += prev_point.distance(point_on_curve);

            while(dst_since_last_eqd_point >= pose_spacing)
            {
                double point_overshoot_dst = dst_since_last_eqd_point - pose_spacing;
                tf2::Vector3 new_eqd_point = point_on_curve + (prev_point - point_on_curve).normalize() * point_overshoot_dst;
                eqd_points.push_back(new_eqd_point);

                dst_since_last_eqd_point = point_overshoot_dst;
                prev_point = new_eqd_point;
            }

            prev_point = point_on_curve;
        }

        eqd_points.push_back(in_points.back());

        length = 0.0; //Length of the curve

        for(size_t i = 0; i < eqd_points.size(); ++i)
        {
            geometry_msgs::Pose pose;
            tf2::Vector3 vec_sum(0.0, 0.0, 0.0);
            
            tf2::toMsg(eqd_points.at(i), pose.position);

            if(i < eqd_points.size() - 1) 
            {
                vec_sum += eqd_points.at(i+1) - eqd_points.at(i);
                length += eqd_points.at(i).distance(eqd_points.at(i+1));
            }  
            
            if(i > 0)
            {
                vec_sum += eqd_points.at(i) - eqd_points.at(i-1);
            }

            vec_sum = vec_sum.normalize();

            double yaw = std::atan2(vec_sum.getY(), vec_sum.getX());
            tf2::Quaternion quat, quat_rot;
            quat.setRPY(0.0, 0.0, yaw);
            // quat_rot.setRPY(0.0, 0.0, M_PI); //Rotate the quaternion about the Z-axis 180 degrees
            // quat = quat_rot * quat;
            tf2::convert(quat.normalize(), pose.orientation);
            
            eqd_poses.push_back(pose);
        }
    }
}