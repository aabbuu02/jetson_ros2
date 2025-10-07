/**
 * @file tag_monitor.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "tag_monitor/tag_monitor.hpp"

TagMonitor::TagMonitor() : Node("tag_monitor_node")
{
    // Declare and get parameters
    this->declare_parameter<std::string>("odom_topic", "odom");
    this->declare_parameter<std::string>("initialize_robot_topic", "initialize_robot");
    this->declare_parameter<std::string>("deinitialize_robot_topic", "initialize_robot");
    this->declare_parameter<std::string>("tag_detection_topic", "detected_tag");
    this->declare_parameter<std::string>("emergency_stop_topic", "e_stop/tag_detection");
    this->declare_parameter<bool>("initialize_on_tag_detection", false);
    this->declare_parameter<double>("max_detection_distance", 1.0);

    std::string m_odomTopic = this->get_parameter("odom_topic").as_string();
    std::string m_initRobotTopic = this->get_parameter("initialize_robot_topic").as_string();
    std::string m_deinitRobotTopic = this->get_parameter("deinitialize_robot_topic").as_string();
    std::string m_detTagTopic = this->get_parameter("tag_detection_topic").as_string();
    std::string m_eStopTopic = this->get_parameter("emergency_stop_topic").as_string();
    m_enableTagDetAsInit = this->get_parameter("initialize_on_tag_detection").as_bool();
    m_maxDetectionDistance = this->get_parameter("max_detection_distance").as_double();

    // Publishers
    m_eStopPub = this->create_publisher<std_msgs::msg::Bool>(m_eStopTopic, 1);
    m_initRobotPub = this->create_publisher<std_msgs::msg::Bool>(m_deinitRobotTopic, 1);

    // Subscribers
    m_odomSub = this->create_subscription<nav_msgs::msg::Odometry>(
        m_odomTopic, 1, std::bind(&TagMonitor::odometry_cb, this, std::placeholders::_1));
    m_detectedTagSub = this->create_subscription<anscer_msgs::msg::PGVPose>(
        m_detTagTopic, 1, std::bind(&TagMonitor::detected_tag_cb, this, std::placeholders::_1));
    m_initRobotSub = this->create_subscription<std_msgs::msg::Bool>(
        m_initRobotTopic, 1, std::bind(&TagMonitor::initialize_robot_cb, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Tag Monitor node has been started.");
}

void TagMonitor::odometry_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // In ROS 2, tf2_geometry_msgs provides this helper function
    tf2::fromMsg(msg->pose.pose.position, m_curPoseVec);

    auto eStopMsg = std_msgs::msg::Bool();
    eStopMsg.data = false; // Default to not stopping

    if(!m_isRobotInitialized)
    {
        m_prevPoseVec = m_curPoseVec;
        m_curDistTravelled = 0.0;
        m_eStopPub->publish(eStopMsg); // Publish false e-stop
        return;
    }

    m_curDistTravelled = m_curPoseVec.distance(m_prevPoseVec);
    RCLCPP_DEBUG(this->get_logger(), "Distance travelled from last detected tag with ID %d is %f", m_prevTagId, m_curDistTravelled);

    auto initRobotMsg = std_msgs::msg::Bool();
    initRobotMsg.data = m_isRobotInitialized;

    if(m_curDistTravelled >= m_maxDetectionDistance)
    {
        eStopMsg.data = true;
        RCLCPP_FATAL(this->get_logger(), "Robot has exceeded the maximum allowed travel distance of %f without detecting a new tag", m_maxDetectionDistance);

        initRobotMsg.data = false;
        if(m_isRobotInitialized) m_initRobotPub->publish(initRobotMsg);
        m_isRobotInitialized = false;
    }

    m_eStopPub->publish(eStopMsg);
}

void TagMonitor::detected_tag_cb(const anscer_msgs::msg::PGVPose::SharedPtr msg)
{
    m_curTagId = msg->id;

    if(m_enableTagDetAsInit && !m_isRobotInitialized)
    {
        RCLCPP_DEBUG(this->get_logger(), "Robot initialized using tag detection");
        m_isRobotInitialized = true;
    }

    if(m_curTagId != m_prevTagId)
    {
        RCLCPP_INFO(this->get_logger(), "Detected tag with ID: %d", m_curTagId);
        m_prevTagId = m_curTagId;
        m_prevPoseVec = m_curPoseVec;
    }
}

void TagMonitor::initialize_robot_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Robot initialized: %s", msg->data ? "true" : "false");
    m_isRobotInitialized = msg->data;
}

// Main function to run the node
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagMonitor>());
    rclcpp::shutdown();
    return 0;
}


