#ifndef NAVIGATION_SPEED_CONTROL_H
#define NAVIGATION_SPEED_CONTROL_H

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include "anscer_msgs/FieldStatus.h"
#include "anscer_msgs/ZoneInformation.h"

using namespace std;


constexpr int AUTO_MODE = 1;

/**
* @class NavigationSpeedControl
* @brief Controls the speed from teb/dwa based on the feedback from the LiDAR
*/

class NavigationSpeedControl
{
public:
    NavigationSpeedControl();

    ~NavigationSpeedControl();

private:
    ros::NodeHandle nh;
    ros::NodeHandle nhp;
    ros::Subscriber fieldSub;
    ros::Subscriber cmdVelSub;
    ros::Subscriber zoneInfoSub;

    ros::Subscriber selectorSub;
    ros::Publisher cmdVelPub;


    /*Member functions*/
    void fieldCallback(const anscer_msgs::FieldStatus &msg);
    void reconfigureParameters(bool warningStatus);
    void selectorSwitchCallback(const std_msgs::Int32 &msg);
    void preNavigateCallback(const geometry_msgs::Twist &msg);
    void zoneInfoCallback(const anscer_msgs::ZoneInformation &msg);

    void initializeParameters();
    /*Member variables*/

    bool m_warningPreviousStatus = false;
    bool m_warningCurrentStatus = false;
    double m_linearVel__m_s,m_reverseVel__m_s;
    double m_speedFactor;
    double m_zoneSpeedFactor = 1.0;
    int m_selectorSwitchStatus;
    std::string m_controllerMaxVelocityParam,m_controllerReversedVelocityParam;
    std::string m_fieldTopic,m_preNavigationTopic,m_navigationTopic,m_selectorSwitchTopic,m_zoneInfoTopic;
    std::string m_controller;

    /*This is an strucure which implements the  operator overloading */

};
#endif
