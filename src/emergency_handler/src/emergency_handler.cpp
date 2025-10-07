#include "emergency_handler.h"


/**
* @brief  Constructor for the EmergengencyHandler
*/
EmergengencyHandler::EmergengencyHandler()
{
    ROS_INFO("EmergengencyHandler Constructor called");

    /*Subscribers*/
    detectionSub      = nh.subscribe("/obstacle_detection", 1, &EmergengencyHandler::detectionCallback,this);
    buttonSub         = nh.subscribe("/e_stop/button", 1, &EmergengencyHandler::eStopButtonCallback,this);    
    webAppSub         = nh.subscribe("/e_stop/webApp", 1, &EmergengencyHandler::eStopWebAppCallback,this);
    
    cmdSub            = nh.subscribe("cmd_vel", 1, &EmergengencyHandler::cmdVelCallback,this);
    keycmdSub         = nh.subscribe("joy/cmd_vel", 1, &EmergengencyHandler::keycmdVelCallback,this);
    
    tagDetFailSub = nh.subscribe("e_stop/tag_detection", 1, &EmergengencyHandler::tagDetFailCallback, this);

    /*Publishers*/

    eStopPub          = nh.advertise<std_msgs::Bool>("e_stop", 1);
    eStopLidarPub     = nh.advertise<std_msgs::Bool>("e_stop/lidar", 1);
    eStopButtonPub    = nh.advertise<std_msgs::Bool>("e_stop/buttons", 1);

   // cmdPub	      = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
}

/**
* @brief  Destructor for the EmergengencyHandler
*/

EmergengencyHandler::~EmergengencyHandler()
{
}





/**
* @brief  Call back for checking if object detected on front side of robot
*/
void EmergengencyHandler::detectionCallback(const anscer_msgs::SafetyFields &msg)
{
    m_lidarForward  = msg.safety[0].is_front;
    m_lidarBackward =  msg.safety[0].is_rear;
}


/**
* @brief  Call back for emergency stop button
*/
void EmergengencyHandler::eStopButtonCallback(const std_msgs::Bool &msg)
{
     m_eStopButton  = msg.data;
}

/**
* @brief  Call back for emergency stop from WebApp
*/
void EmergengencyHandler::eStopWebAppCallback(const std_msgs::Bool &msg)
{
     m_eStopWebApp  = msg.data;
}

/**
* @brief  Call back for emergency stop from Lidar
*/
void EmergengencyHandler::cmdVelCallback(const geometry_msgs::Twist &msg)
{
    m_linearVel  = msg.linear.x;
}


/**
* @brief  Call back for emergency stop from Lidar
*/
void EmergengencyHandler::keycmdVelCallback(const geometry_msgs::Twist &msg)
{
    m_keyLinearVel  = msg.linear.x;
    m_keyAngularVel = msg.angular.z;
}

/**
* @brief  Callback for emergency stop from tag detection failure
*/
void EmergengencyHandler::tagDetFailCallback(const std_msgs::Bool &msg)
{
    m_eStopTagDetection = msg.data;
}

/**
* @brief  Publishes e_stop based on input from various sources
*/
void EmergengencyHandler::publishMessages()
{
    std_msgs::Bool eStopMsg,eStopLidarMsg,eStopButtonMsg;
    bool estopLidar = false;
    bool cmdVelLidar    = ((m_lidarForward && (m_linearVel > 0.0)) || (m_lidarBackward && (m_linearVel < 0.0)));
    bool stopedcmdVelLidar    = ((m_lidarForward && (m_linearVel == 0.0)) || (m_lidarBackward && (m_linearVel == 0.0)));
    bool keyCmdVelLidar = ((m_lidarForward && (m_keyLinearVel < 0.0)) || (m_lidarBackward && (m_keyLinearVel > 0.0)));
    
    bool keyCmdTagDetection = ((m_keyLinearVel != 0.0) || (m_keyAngularVel != 0.0));    

    if(cmdVelLidar||stopedcmdVelLidar)
    {
        estopLidar = true;
    }

    if(keyCmdVelLidar)
    {
        estopLidar = false;
    }

    if(keyCmdTagDetection)
    {
        m_eStopTagDetection = false;
    }

    if(m_eStopButton || m_eStopWebApp)
    {
        eStopButtonMsg.data = true;
    }
    else
    {
        eStopButtonMsg.data = false;
    }

    if(estopLidar)
    {
        eStopLidarMsg.data=true;
    }
    else
    {
        eStopLidarMsg.data = false;
    }

    if(estopLidar || m_eStopButton || m_eStopWebApp || m_eStopTagDetection)
    {
        eStopMsg.data = true;
    }
    else
    {
        eStopMsg.data = false;
    }
    eStopPub.publish(eStopMsg);
    eStopButtonPub.publish(eStopButtonMsg);
    eStopLidarPub.publish(eStopLidarMsg);
   
    /*

    if(eStopMsg.data == true)
    {
    	geometry_msgs::Twist cmdMsg;
	cmdMsg.linear.x  = 0.0;
	cmdMsg.angular.z = 0.0;
	cmdPub.publish(cmdMsg);
	    
    }	    
   */	
}



/**
* @brief Main function
*/
int main(int argc, char** argv)
{

    ros::init(argc, argv, "EmergengencyHandler");
    EmergengencyHandler emergencyHandler;
    ros::Rate rate(20);

    while(ros::ok())
    {
        emergencyHandler.publishMessages();
        rate.sleep();
        ros::spinOnce();
    }
    ros::shutdown();
    return 0;
}







