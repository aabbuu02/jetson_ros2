#pragma once

#include <ros/ros.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <anscer_msgs/PGVPose.h>


class TagMonitor
{

public:

    TagMonitor(); 

private:
    
    ros::NodeHandle m_nh, m_nhp;

    //Messages
    std_msgs::Bool m_eStopMsg, m_initRobotMsg;

    //Publishers
    ros::Publisher m_eStopPub, m_initRobotPub;

    //Subscribers
    ros::Subscriber m_odomSub, m_detectedTagSub, m_initRobotSub;

    //Subscriber callbacks
    void odometryCb(const nav_msgs::Odometry &odomMsg);
    void detectedTagCb(const anscer_msgs::PGVPose &tagMsg);
    void initializeRobotCb(const std_msgs::Bool &initStateMsg);

    //Topic names
    std::string m_odomTopic, m_initRobotTopic, m_deinitRobotTopic, m_detTagTopic, m_eStopTopic;

    //Tag monitoring variables
    bool m_enableTagDetAsInit = false;
    uint32_t m_curTagId = 0;
    uint32_t m_prevTagId = 0;
    double m_maxDetectionDistance = 1.0f; //Maximum distance that can be travelled without detecting a tag in meters
    
    //Distance travelled by the robot
    bool m_isRobotInitialized = false;  
    tf2::Vector3 m_prevPoseVec, m_curPoseVec;
    double m_curDistTravelled = 0.0f;

    void getParameters();
};
