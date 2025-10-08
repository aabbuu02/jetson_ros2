#include "LowerLevelInterface.h"
#include <ros/console.h> 

/**
* @brief  Constructor for the LowerLevelInterface
*/
LowerLevelInterface::LowerLevelInterface():nhp("~")
{

    initializeParameters();
    p_getVariables          = new RosClass();
    p_lowerLevelComputation = new LowerLevelComputations();

    
	p_client 		   		 = new Client(nh,m_controllerType);
	p_highLevelController    = p_client->getHighLevelController();

    //authenticationProcess();

    m_lastTime = m_currentTime = Utilities::millis();

    
    ROS_INFO_NAMED("LowerLevelInterface","LowerLevelInterface Constructor called ");


}

/**
* @brief  Destructor for the LowerLevelInterface
*/
LowerLevelInterface::~LowerLevelInterface()
{
}

/**
* @brief  Initialize parameters from the parameter server
*/
void LowerLevelInterface::initializeParameters()
{
    nhp.param("controller_type",m_controllerType,1);
    nhp.param("publish_tf", m_publishTF,true);
    m_odom.pose.pose.orientation.z = 0.0;
    m_odom.pose.pose.orientation.w = 1.0;
}

void LowerLevelInterface::authenticationProcess()
{
    getMACAddress();
    if(m_macAddress == MACADDRESS)
    {
        ROS_INFO_NAMED("LowerLevelInterface","LowerLevelInterface::Authentication succeeful");
    }
    else
    {
        ROS_FATAL_NAMED("LowerLevelInterface","Authentication failed.Contact Vendor");
        //exit(0);
    }
}


/**
* @brief Access the MAC address
*/
void LowerLevelInterface::getMACAddress()
{
  int max_buffer = 256;
  char buffer[max_buffer]; 
  std::string str = "cat /sys/class/net/eth0/address";

  /*The system command is often run first, before any output commands and the function 
  returns an integer indicating success or failure, but not the output of the string*/

  /*Opens up a read-only stream, runs the command and captures the output,
   stuffs it into the buffer and returns it as a string.*/
  FILE *stream = popen(str.c_str(), "r");
  if (stream) 
  {
    while (!feof(stream))
    {
        if (fgets(buffer, max_buffer, stream) != NULL) 
        {
            
        	m_macAddress.append(buffer);
            //std::cout << m_address << std::endl;
        }  
        
    }
    
    /*To avoid the new line character by the end of data*/
    m_macAddress = m_macAddress.substr(0, m_macAddress.length() - 1);
    pclose(stream);

   }
}



/**
* @brief  Entire control flow for the LowerLevelInterface
*/
void LowerLevelInterface::controlFlow()
{
    //ROS_INFO("FLOW");
    m_currentTime = Utilities::millis();
    m_deltaTime = m_currentTime - m_lastTime;

    geometry_msgs::Twist cmdVel     = p_getVariables->getCommandVelocity();
    bool eStopStatus                = p_getVariables->getEStopStatus();
    //ROS_INFO("FLOW1");


    if (eStopStatus)
    {
        ROS_ERROR_THROTTLE(2.0, "Lower level Robot in emergency stop condition");
        resetCmdVelocity(cmdVel);
    }
      //ROS_INFO("FLOW2");
diagnoseMotorControllerHealth(m_readParams,cmdVel);
    ROS_WARN("Linear : %f   Angular : %f",cmdVel.linear.x,cmdVel.angular.z);
   //ROS_INFO("FLOW3");

    p_lowerLevelComputation->initializeComputation(cmdVel, m_writeParams);
    //ROS_INFO("FLOW4");

    /*Passes the left and right motor command to motor controller*/
    p_highLevelController->writeCommands(m_writeParams);
    //ROS_INFO("FLOW5");

    /*Reads the left and right encoder readings from the motor controller*/
    p_highLevelController->readCommands(m_readParams);
    //ROS_INFO("FLOW6");

    p_lowerLevelComputation->updateOdometry(m_deltaTime, m_readParams, m_odom);
   //ROS_INFO("FLOW7");

    /*Publishes the odometry using the setter function*/
    p_getVariables->sendOdometry(m_odom);
    //ROS_INFO("flow flow 7");
    p_getVariables->sendReadParameters(m_readParams);
    //ROS_INFO("FLOW8");

    /*Comment the following line if the transformation is not required to be published*/
    if (m_publishTF)
    {
        p_getVariables->updateTF(m_odom);
    }
    //ROS_INFO("FLOW8");

    

    m_lastTime = m_currentTime;
      //ROS_INFO("END XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");

}



/**
* @brief  Diagnose motor controller health and stops the robot if any error observed
*/
void LowerLevelInterface::diagnoseMotorControllerHealth(const readParameters &readParams,geometry_msgs::Twist &cmdVel)
{

    anscer_msgs::MotorDiagnosticsArray motorDiagnosticsArray = readParams.m_roboteqDiagnosticsFeedback;

    return;

    bool motorErrorCondition = (motorDiagnosticsArray.motor_diagnostics[0].fault_flags.overheat ||
                                motorDiagnosticsArray.motor_diagnostics[0].fault_flags.overvoltage ||
                                motorDiagnosticsArray.motor_diagnostics[0].fault_flags.undervoltage ||
                                motorDiagnosticsArray.motor_diagnostics[0].fault_flags.short_circuit ||
                                motorDiagnosticsArray.motor_diagnostics[0].fault_flags.emergency_stop ||
                                motorDiagnosticsArray.motor_diagnostics[0].fault_flags.motor_setup_fault ||
                                motorDiagnosticsArray.motor_diagnostics[0].fault_flags.mosfet_failure);

    if(motorErrorCondition)
    {
        ROS_ERROR_NAMED("LowerLevelInterface","Robot stopping due to motor errors,please check /motor_diagnostics topic for details");
        resetCmdVelocity(cmdVel);
    }
}

/**
* @brief  Resets the command velocity for robot
*/
void LowerLevelInterface::resetCmdVelocity(geometry_msgs::Twist &cmdVel)
{
    cmdVel.linear.x  = 0.0;
    cmdVel.angular.z = 0.0;
}

/**
* @brief Main function
*/
int main(int argc, char** argv)
{

    ros::init(argc, argv, "acr_robot_controller");
    LowerLevelInterface lowerlevelcontroller;
   
    ros::Rate rate(40);

    while(ros::ok())
    {
        lowerlevelcontroller.controlFlow();
        rate.sleep();
        ros::spinOnce();
    }

    ros::shutdown();
    return 0;
}





