#ifndef EMERGENCY_HANDLER_H
#define EMERGENCY_HANDLER_H

#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <anscer_msgs/SafetyField.h>
#include <anscer_msgs/SafetyFields.h>



using namespace std;

/**
* @class EmergengencyHandler
* @brief A node that controls the e_stops from different nodes and combines together
*/

class EmergengencyHandler
{
public:
    
    /**
    * @brief  Constructor for the EmergengencyHandler
    */
    EmergengencyHandler();
    
    /**
    * @brief  Destructor for the EmergengencyHandler
    */
    ~EmergengencyHandler();

    void publishMessages();

private:

    ros::NodeHandle nh;

    ros::Subscriber detectionSub;
    ros::Subscriber buttonSub;
    ros::Subscriber webAppSub;
    ros::Subscriber cmdSub;
    ros::Subscriber keycmdSub;
    ros::Subscriber tagDetFailSub;

    ros::Publisher eStopPub;
    ros::Publisher eStopLidarPub;
    ros::Publisher eStopButtonPub;
    /*Member functions*/

    void detectionCallback(const anscer_msgs::SafetyFields &msg);
    void eStopButtonCallback(const std_msgs::Bool &msg);
    void eStopWebAppCallback(const std_msgs::Bool &msg);
    void cmdVelCallback(const geometry_msgs::Twist &msg);
    void keycmdVelCallback(const geometry_msgs::Twist &msg); 
    void tagDetFailCallback(const std_msgs::Bool &msg);

    /*Member variables*/

    bool m_lidarForward = false;
    bool m_lidarBackward = false;
    bool m_eStopButton = false;
    bool m_eStopWebApp = false;
    bool m_eStopTagDetection = false;

    double m_keyLinearVel = 0.0;
    double m_keyAngularVel = 0.0;
    double m_linearVel = 0.0;
};
#endif
