/**
 * @file tag_monitor.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef TAG_MONITOR_HPP
#define TAG_MONITOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "anscer_msgs/msg/pgv_pose.hpp"

#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TagMonitor : public rclcpp::Node
{
public:
    TagMonitor();

private:
    // Callbacks
    void odometry_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void detected_tag_cb(const anscer_msgs::msg::PGVPose::SharedPtr msg);
    void initialize_robot_cb(const std_msgs::msg::Bool::SharedPtr msg);

    // Member variables
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_eStopPub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_initRobotPub;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odomSub;
    rclcpp::Subscription<anscer_msgs::msg::PGVPose>::SharedPtr m_detectedTagSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_initRobotSub;

    bool m_isRobotInitialized = false;
    bool m_enableTagDetAsInit;
    double m_maxDetectionDistance;
    double m_curDistTravelled = 0.0;
    
    int m_curTagId = -1;
    int m_prevTagId = -1;

    tf2::Vector3 m_curPoseVec;
    tf2::Vector3 m_prevPoseVec;
};

#endif // TAG_MONITOR_HPP


