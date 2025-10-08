#ifndef ROS_CLASS_H
#define ROS_CLASS_H


#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/BatteryState.h>
#include <actionlib/server/simple_action_server.h>
#include <anscer_msgs/LiftAction.h>

#include "anscer_msgs/RelayState.h"
#include "anscer_msgs/Feedback.h"
#include "anscer_msgs/ControllerMode.h"
#include <anscer_msgs/UserFeedback.h>


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


    /*Setter function*/
    void sendOdometry(const nav_msgs::Odometry &odom);

    void sendReadParameters(const readParameters &parameterValue);
    void updateTF(const nav_msgs::Odometry &odom);
    void getWriteParams(writeParameters &value);
    void sendLifterState(const readParameters &rp);

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
    ros::Subscriber lifterSub;

    /**
     * @brief ROS Publishers
    */
    ros::Publisher odomPub;
    ros::Publisher emergencyPub;
    ros::Publisher confirmationPub;
    ros::Publisher modePub;
    ros::Publisher botShelfPub;

    /**
     * @brief ROS services
    */
    ros::ServiceServer relayService;
    ros::ServiceClient confirmationClient;

    /*Timers*/
    ros::Timer heartBeatTimer;

    /*Timer callback*/
    void heartBeatTimerCallback(const ros::TimerEvent &event);

    /*Action server*/

    int m_currentLiftCommand = 0;


    /*Action server callbacks*/


    /*Member Functions*/

    void eStopCallback(const std_msgs::Bool &msg);
    void feedbackCallback(const anscer_msgs::Feedback &msg);
    void cmdVelCallback(const geometry_msgs::Twist &msg);
    void lifterCallback(const lift_action::LiftAction &msg);


    bool relayStatus(anscer_msgs::RelayState::Request &req, anscer_msgs::RelayState::Response &res);
    // void ledIndicationCallback(const anscer_msgs::Indication &msg);
    void fieldSetCallback(const std_msgs::Int8MultiArray &fieldMsg);
    void taskRequestStatusCallback(const std_msgs::Int8 &statusMsg);

    /*Member variables*/
    int m_controllerType;
    bool m_eStop = false;
    bool m_relayState = false;  
    int m_ledCode=0;
    int m_buzzerCode=0;
    int m_prevLocationId = -1;
    int m_frontFieldId = 0;
    int m_rearFieldId = 0;
    int m_heartBeatState = 0;

    int8_t m_taskStatus = 0;
    int m_controllerMode = 0;

    bool m_currentConfirmState = false;

    //TODO: Wouldn't using <remap> be better?
    lift_action::LiftAction m_lifterData;

    std::string m_odomTopic,m_cmdVelTopic,m_emergencyStopTopic,m_emergencyButtonTopic;


    geometry_msgs::Twist m_cmdVel;


    tf::TransformBroadcaster m_broadcaster;
    tf::Transform m_transform;
};

#endif
