#include <ros/console.h> 
#include "RosClass.h"


/**
* @brief  Constructor for the Computations
*/
RosClass::RosClass():nhp("~")
{
    ROS_INFO("RosClass Constructor called ");
    
    initializeParameters();


    /*Subscribers*/
    cmdVelSub       = nh.subscribe(m_cmdVelTopic.c_str(), 1, & RosClass::cmdVelCallback, this);
    eStopSub        = nh.subscribe(m_emergencyStopTopic.c_str(), 1, & RosClass::eStopCallback, this);
    
    /*Publishers*/
    odomPub         = nh.advertise <nav_msgs::Odometry> (m_odomTopic.c_str(), 1);
    batteryPub      = nh.advertise <sensor_msgs::BatteryState> (m_batteryTopic.c_str(), 1);

    if(m_controllerType == ROBOTEQ_DEVICE)
    {
        motorDiagPub   = nh.advertise <anscer_msgs::MotorDiagnosticsArray> (m_motorDiagnosticsTopic.c_str(), 1);   
    }


}

/**
* @brief  Destructor for the Computations
*/

RosClass::~RosClass()
{
    nhp.param("controller_type",m_controllerType,1);

    nhp.param<std::string>("cmd_topic", m_cmdVelTopic,"cmd_vel");
    nhp.param<std::string>("odom_topic", m_odomTopic,"odom");
    nhp.param<std::string>("emergency_stop_topic",m_emergencyStopTopic,"e_stop");
    nhp.param<std::string>("motor_diagnostics_topic",m_motorDiagnosticsTopic,"motor_diagnostics");
    nhp.param<std::string>("battery_topic",m_batteryTopic,"bms/battery_state");

    nhp.param<double>("cmd_vel_timeout", m_cmdVelResetTimeout, 0.25);
}

/**
* @brief  Initializes the parameters
*/
void RosClass::initializeParameters()
{

	   nhp.param("controller_type",m_controllerType,1);

    nhp.param<std::string>("cmd_topic", m_cmdVelTopic,"cmd_vel");
    nhp.param<std::string>("odom_topic", m_odomTopic,"odom");
    nhp.param<std::string>("emergency_stop_topic",m_emergencyStopTopic,"e_stop");
    nhp.param<std::string>("motor_diagnostics_topic",m_motorDiagnosticsTopic,"motor_diagnostics");
    nhp.param<std::string>("battery_topic",m_batteryTopic,"bms/battery_state");

    nhp.param<double>("cmd_vel_timeout", m_cmdVelResetTimeout, 0.25);

}

/**
* @brief  Call back for command velocities for robot
*/
void RosClass::cmdVelCallback(const geometry_msgs::Twist &msg)
{
    m_cmdVel  = msg;
    m_cmdVelPrevUpdateTime = ros::Time::now();
}


/**
* @brief  Call back for emergency stops(UI/Hardware)
*/
void RosClass::eStopCallback(const std_msgs::Bool &msg)
{
    m_eStop = msg.data;
} 





/*Getter Functions*/

/**
* @brief  Access the robot velocity command
* @return velocity command to the interface class
*/
geometry_msgs::Twist RosClass::getCommandVelocity()
{

   bool resetVel = false;
   ros::Duration elapsedTime = ros::Time::now() - m_cmdVelPrevUpdateTime;
   if(elapsedTime.toSec() >= m_cmdVelResetTimeout) resetVel = true;

   if(m_eStop || resetVel)
   {
     m_cmdVel.linear.x  = 0.0;
     m_cmdVel.angular.z = 0.0;
   } 
   return m_cmdVel;
}


/**
* @brief  Gets the estop status
* @return boolean value of estop status
*/
bool RosClass::getEStopStatus()
{
    return m_eStop;
}




/**
* @brief  Publishing the odometry value for the robot
* @param odom value computed for the robot
*/
void RosClass::sendOdometry(const nav_msgs::Odometry &odom)
{
    odomPub.publish(odom);
}


/**
* @brief  Publishing the transformation about odom frame
* @param odom value computed for the robot
*/
void RosClass::updateTF(const nav_msgs::Odometry &odom)
{
    geometry_msgs::Pose inputPose;
    std::string targetFrame           = "/base_footprint";
    std::string parentFrame           = "/odom_frame";
    inputPose.position                = odom.pose.pose.position;
    inputPose.orientation             = odom.pose.pose.orientation; 
    m_transform.setOrigin(tf::Vector3(inputPose.position.x,inputPose.position.y, 0.0) );
    m_transform.setRotation(tf::Quaternion(0.0, 0.0, inputPose.orientation.z, inputPose.orientation.w));          
    m_broadcaster.sendTransform(tf::StampedTransform(m_transform, ros::Time::now(),parentFrame, targetFrame)); 
}



/**
* @brief  Publishing the read parmaters from the controller
* @param parameterValue the read parameters in the readParameters structure
*/
void RosClass::sendReadParameters(const readParameters &parameterValue)
{
    std_msgs::Bool emergencyState;
    emergencyState.data = parameterValue.m_emergencyFeedback;
   // emergencyPub.publish(emergencyState);

    // std_msgs::UInt8 modeSelMsg;
    // modeSelMsg.data = parameterValue.m_modeSelectionFeedback;
    // modeSelPub.publish(modeSelMsg);

    if (m_controllerType == ROBOTEQ_DEVICE)
    {
        anscer_msgs::MotorDiagnosticsArray motorDiagnosticsArray;
        motorDiagnosticsArray.motor_diagnostics = parameterValue.m_roboteqDiagnosticsFeedback.motor_diagnostics;
        sensor_msgs::BatteryState batteryMsg;

	batteryMsg.header.stamp = ros::Time::now();
        batteryMsg.charge   = 100.0f; //Dummy value as charge cannot be calculated
        batteryMsg.capacity = 120.0f; //Dummy value as capacity is unknown
        batteryMsg.design_capacity = 120.0f; //Dummy design capcity
        batteryMsg.percentage = 1.0f; //Dummy percentage
        batteryMsg.power_supply_status = 1;
        batteryMsg.power_supply_health = 1;
        batteryMsg.power_supply_technology = 2;
        batteryMsg.present = 1;
        batteryMsg.voltage  = parameterValue.m_batteryVoltage;
        batteryMsg.current  = parameterValue.m_batteryCurrent;
        
        batteryPub.publish(batteryMsg);
        motorDiagPub.publish(motorDiagnosticsArray);
    }
}
