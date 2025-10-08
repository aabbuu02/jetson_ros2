#include <ros/console.h> 
#include "RosClass.h"

/**
* @brief  Constructor for the Computations
*/
RosClass::RosClass():nhp("~")
{
    ROS_INFO("RosClass Constructor called ");
    
    nhp.param("controller_type",m_controllerType,1);

    nhp.param<std::string>("cmd_topic", m_cmdVelTopic,"cmd_vel");
    nhp.param<std::string>("odom_topic", m_odomTopic,"odom");
    nhp.param<std::string>("emergency_stop_topic",m_emergencyStopTopic,"e_stop");
    nhp.param<std::string>("emergency_button_topic",m_emergencyButtonTopic,"e_stop/button");


    // nhp.param<std::string>("tower_light_topic", m_towerLightTopic, "tower_light/indication");

    /*Subscribers*/
    cmdVelSub       = nh.subscribe(m_cmdVelTopic.c_str(), 1, & RosClass::cmdVelCallback, this);
    eStopSub        = nh.subscribe(m_emergencyStopTopic.c_str(), 1, & RosClass::eStopCallback, this);
    lifterSub       = nh.subscribe("lift_action", 1, & RosClass::lifterCallback, this);
//    distanceToGoal  =nh.subscribe("disatance_to_goal",1, & RosClass::distancetoGoalCallback,this); 




    /*Publishers*/
    odomPub         = nh.advertise <nav_msgs::Odometry> (m_odomTopic.c_str(), 1);
    botShelfPub     = nh.advertise<std_msgs::Int8>("bot_shelf_number", 1);
    // batteryPub        = nh.advertise <sensor_msgs::BatteryState> ("/bms/battery_state", 1);
    emergencyPub      = nh.advertise <std_msgs::Bool> (m_emergencyButtonTopic.c_str(), 1);

    /*Timer callback*/
    heartBeatTimer = nh.createTimer(ros::Duration(1/4), &RosClass::heartBeatTimerCallback, this);

    /*Action server*/


    // if(m_controllerType == MODBUS_ROBOTEQ_DEVICE)
    // {
    //     motorDiagPub   = nh.advertise <anscer_msgs::MotorDiagnosticsArray> (m_motorDiagnosticsTopic.c_str(), 1);   
    // }

    /*Services*/


   
}

/**
* @brief  Destructor for the Computations
*/

RosClass::~RosClass()
{

}

void RosClass::lifterCallback(const lift_action::LiftAction &msg)
{
  m_lifterData = msg;
}

void RosClass::heartBeatTimerCallback(const ros::TimerEvent &event)
{
  m_heartBeatState = m_heartBeatState == 0 ? 1 : 0;
}


/**
* @brief  Call back for command velocities for robot
*/
void RosClass::cmdVelCallback(const geometry_msgs::Twist &msg)
{
    m_cmdVel  = msg;
} 

/**
 * @brief callback for distance to goal 
 *
 */

//void RosClass::distancetoGoalCallback(const std_msgs::Float32 &msg)
//{
//	m_distanceToGoal = msg.data;
//}

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
   if(m_eStop)
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

void RosClass::getWriteParams(writeParameters &value)
{
    // value.m_ledMode = m_ledCode;
    // value.m_buzzerMode = m_buzzerCode;
    // value.m_frontLidarField = m_frontFieldId;
    // value.m_rearLidarField = m_rearFieldId;
    // value.m_liftCommand = m_currentLiftCommand;
    value.m_heartBeat = m_heartBeatState;
    // value.m_controllerMode = m_controllerMode;
    // value.m_relayState = m_relayState;
    value.m_lifterData = m_lifterData; 
}

/**
* @brief  Publishing the read parmaters from the controller
* @param parameterValue the read parameters in the readParameters structure
*/
void RosClass::sendReadParameters(const readParameters &parameterValue)
{
  std_msgs::Int8 botShelfMsg;
  botShelfMsg.data = parameterValue.m_botShelfNo;
  botShelfPub.publish(botShelfMsg);  
  //   sensor_msgs::BatteryState battery;
  //   battery.header.stamp = ros::Time::now();
  //   battery.voltage = (parameterValue.m_batteryVoltage -20)/10;
  //   battery.percentage = parameterValue.m_batteryPercentage/100;
  //   battery.capacity = 72.0;
  //   battery.design_capacity = 72.0;
  //   if(battery.percentage > 0.001)
  //       batteryPub.publish(battery);


  //   std_msgs::Bool emergencyState, confirmationState, dockState, frontFieldState, rearFieldState;
  //   emergencyState.data = parameterValue.m_emergencyFeedback;
  //   emergencyPub.publish(emergencyState);

  //   std::bitset<16> confirmationRegBits(parameterValue.m_confirmationFeedback);

  //   confirmationState.data = !confirmationRegBits[0];  
  //   confirmationPub.publish(confirmationState);

  //   if(confirmationState.data != m_currentConfirmState)
  //   {
	// ROS_INFO_STREAM("Confirmation button"<<(confirmationState.data ? "pressed" : "released"));
	// anscer_msgs::UserFeedback userFeedbackSrv;
	// userFeedbackSrv.request.user_feedback = confirmationState.data;
	// if(!confirmationClient.call(userFeedbackSrv))
	//   ROS_ERROR_STREAM("Failed to call user feedback service");
  //   	m_currentConfirmState = confirmationState.data;	    
  //   }

  //   std::bitset<16> modeSelectionRegBits(parameterValue.m_modeSelectionFeedback);

  //   std_msgs::Int32 modeSelectionMsg;
    
  //   if(modeSelectionRegBits[4]) modeSelectionMsg.data = 1; //Auto mode
  //   else if (modeSelectionRegBits[5]) modeSelectionMsg.data = 2; //Manual mode
  //   else modeSelectionMsg.data = 3; //Brake release mode

  //   modePub.publish(modeSelectionMsg);
    
  //   dockState.data = parameterValue.m_dockStateFeedback;
  //   dockStatePub.publish(dockState);

  //   frontFieldState.data = parameterValue.m_frontLidarFeedback;
  //   frontFieldActPub.publish(frontFieldState);

  //   rearFieldState.data = parameterValue.m_rearLidarFeedback;
  //   rearFieldActPub.publish(rearFieldState);


  //   std_msgs::Int8 valueId = parameterValue.m_locationId;
  //   if(m_prevLocationId != valueId.data)
  //   {   
	// m_prevLocationId = valueId.data;
  //   	locationIdPub.publish(valueId);
  //   }
  //   if (m_controllerType == MODBUS_ROBOTEQ_DEVICE)
  //   {
  //       anscer_msgs::MotorDiagnosticsArray motorDiagnosticsArray;
  //       motorDiagnosticsArray.motor_diagnostics = parameterValue.m_roboteqDiagnosticsFeedback.motor_diagnostics;
  //       motorDiagPub.publish(motorDiagnosticsArray);
  //   }
}
