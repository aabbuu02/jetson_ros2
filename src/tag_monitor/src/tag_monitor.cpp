#include <tag_monitor/tag_monitor.hpp>


TagMonitor::TagMonitor():
    m_nhp("~")
{
    getParameters();    

    //Publishers
    m_eStopPub = m_nh.advertise<std_msgs::Bool>(m_eStopTopic, 1);
    m_initRobotPub = m_nh.advertise<std_msgs::Bool>(m_deinitRobotTopic, 1);

    //Subscirbers
    m_odomSub = m_nh.subscribe(m_odomTopic, 1, &TagMonitor::odometryCb, this);
    m_detectedTagSub = m_nh.subscribe(m_detTagTopic, 1, &TagMonitor::detectedTagCb, this);
    m_initRobotSub = m_nh.subscribe(m_initRobotTopic, 1, &TagMonitor::initializeRobotCb, this);
}

void TagMonitor::odometryCb(const nav_msgs::Odometry &odomMsg)
{
   tf2::fromMsg(odomMsg.pose.pose.position, m_curPoseVec);
 
   m_eStopPub.publish(m_eStopMsg);
   
   if(!m_isRobotInitialized)
   {
        m_prevPoseVec = m_curPoseVec;
        m_curDistTravelled = 0.0f;
        return;
   }

   m_curDistTravelled = m_curPoseVec.distance(m_prevPoseVec);
   ROS_DEBUG_STREAM_NAMED("distance_travelled", "Distance travelled from last detected tag with ID "<<m_prevTagId<<" is "<<m_curDistTravelled);
   
   m_eStopMsg.data = false;
   m_initRobotMsg.data = m_isRobotInitialized;

   if(m_curDistTravelled >= m_maxDetectionDistance)
   {
     m_eStopMsg.data = true;
     ROS_FATAL_STREAM("Robot has exceeded the maximum allowed travel distance of "<<m_maxDetectionDistance<<" without detecting a new tag");
    
     m_initRobotMsg.data = false;
     if(m_isRobotInitialized) m_initRobotPub.publish(m_initRobotMsg);
     m_isRobotInitialized = false;
   }
}

void TagMonitor::detectedTagCb(const anscer_msgs::PGVPose &tagMsg)
{
    m_curTagId = tagMsg.id;
    
    if(m_enableTagDetAsInit && !m_isRobotInitialized)
    {
        ROS_DEBUG_STREAM_NAMED("initialization", "Robot initialized using tag detection");
        m_isRobotInitialized = true;
    }
    
    if(m_curTagId != m_prevTagId)
    {
        ROS_INFO_STREAM_NAMED("detected_tag", "Detected tag with ID: "<<m_curTagId);
        m_prevTagId = m_curTagId;
        m_prevPoseVec = m_curPoseVec;
    }
}

void TagMonitor::initializeRobotCb(const std_msgs::Bool &initStateMsg)
{
   ROS_INFO_STREAM_NAMED("initialization", "Robot initialized: "<<std::boolalpha<<initStateMsg.data);
   m_isRobotInitialized = initStateMsg.data; 
}

void TagMonitor::getParameters()
{
    m_nhp.param<std::string>("odom_topic", m_odomTopic, "odom");
    m_nhp.param<std::string>("initialize_robot_topic", m_initRobotTopic, "initialize_robot");
    m_nhp.param<std::string>("deinitialize_robot_topic", m_deinitRobotTopic, "initialize_robot");
    m_nhp.param<std::string>("tag_detection_topic", m_detTagTopic, "detected_tag");
    m_nhp.param<std::string>("emergency_stop_topic", m_eStopTopic, "e_stop/tag_detection");
    
    m_nhp.param("initialize_on_tag_detection", m_enableTagDetAsInit, false);
    m_nhp.param("max_detection_distance", m_maxDetectionDistance, 1.0);
}
