#ifndef ROS_CLASS_H
#define ROS_CLASS_H

#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/BatteryState.h>
#include "anscer_msgs/BatteryState.h"

#include "Utilities.h"

class RosClass
{
public:
    /**
    * @brief  Constructor for the Ros class
    */

    RosClass();

    /**
    * @brief  Destructor for the Ros class
    */
    ~RosClass();

    /*Getter Functions*/
    geometry_msgs::Twist getCommandVelocity();
    bool getEStopStatus();
    bool setRelayState();
    bool getBrakeActivationState();

    /*Setter function*/
    void sendOdometry(const nav_msgs::Odometry &odom);
    void sendReadParameters(const readParameters &parameterValue);
    void updateTF(const nav_msgs::Odometry &odom);


private:
    /**
    * @brief NodeHandle ROS
    */
    ros::NodeHandle nh;
    ros::NodeHandle nhp;

    /**
     * @brief ROS Subscriber
    */
    ros::Subscriber eStopSub;
    ros::Subscriber cmdVelSub;

    /**
     * @brief ROS Publishers
    */
    ros::Publisher odomPub;
    ros::Publisher motorDiagPub;
    ros::Publisher emergencyPub;
    ros::Publisher batteryPub;

    /**
     * @brief ROS services
    */

    /**
     * @brief ROS timers
    */
    ros::Time m_cmdVelPrevUpdateTime;


    /*Member Functions*/

    void eStopCallback(const std_msgs::Bool &msg);
    void initializeParameters();
    void cmdVelCallback(const geometry_msgs::Twist &msg);



    /*Member variables*/
    int m_controllerType;
    bool m_eStop = false;
    bool m_isBrakeActivated = false;
    bool m_relayState = false;  
    std::string m_odomTopic,m_cmdVelTopic,m_emergencyStopTopic,m_emergencyButtonTopic, m_motorDiagnosticsTopic,m_batteryTopic;
    std::string m_modeSelTopic;
    std::string m_relayStateService;
    geometry_msgs::Twist m_cmdVel;
    double m_cmdVelResetTimeout = 0.25f;


    tf::TransformBroadcaster m_broadcaster;
    tf::Transform m_transform;
};

#endif
