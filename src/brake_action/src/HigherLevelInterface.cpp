/**
 * @file HigherLevelInterface.cpp
 * @author Jishnu (jishnu@anscer.com)
 * @brief communication node used for communicating between two  PLC s using modbus protocol
 *
 * @version 0.1
 * @date 2022-07-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */
/**
 * @brief include headerfiles
 */

#include "brake_action/HigherLevelInterface.h"

HigherLevelInterface::HigherLevelInterface()
{
    ROS_INFO("Higher_interface_constructor_called");
    p_modbusCommunicator=new ModbusCommunicator();
    m_updateRate={0};
    m_writeRegistornumber={0};
    m_writeValue={0};

    m_parameterExistanceFlag=checkParameters();

    if(m_parameterExistanceFlag==0)
    {
        ROS_ERROR("failed to load HigherLevelInterface parameters");
        
        exit(0);
    }
    else
    {
        int feedBack=readParameters();
        if(feedBack !=0)
          {
            ROS_ERROR("parameter reading failed");
            
            
            exit(0);
            
          }
        else
          {
            ROS_INFO("All HigherLevelInterface parameter read successfully");
            
            
            
            
          }
    }
    
    /**
     * @brief publisher definition
     * 
     */

    brakeActivator = p_modbusCommunicator->nh.subscribe(m_subscriberTopic, 1, &HigherLevelInterface::brakeActivatorCallback, this);

}
/**
 * @brief Destroy the Higher Level Interface:: Hiigher Level Interface object
 * 
 */
HigherLevelInterface::~HigherLevelInterface()
{
 
}




/**
 * @brief callback function for brakeActivatorCallback
 * 
 */
void HigherLevelInterface::brakeActivatorCallback(const std_msgs::Bool &msg)
{


 
  ROS_INFO("brake Activator subscriber  called");
  m_writeValue=int(msg.data);

  writeData(m_writeRegistornumber,m_writeValue);



}




/**
 * @brief for checking the availability of parameters
 * @brief returns true if all the parameters exists or return false
 */

bool HigherLevelInterface::checkParameters()
{
  ROS_INFO("checking existance of parameters");
  if(p_modbusCommunicator->nh.hasParam("/HigherLevelInterface/update_rate")!=1)
  {
    ROS_WARN("updateRate param misssing");
    return 0;
  }
 else if(p_modbusCommunicator->nh.hasParam("/HigherLevelInterface/subscriberTopic")!=1)
  {
    ROS_WARN("subscriberTopic param misssing");
    return 0;
  }
else  if(p_modbusCommunicator->nh.hasParam("/HigherLevelInterface/registerNumber")!=1)
  {
    ROS_WARN("regsiter number param misssing");
    return 0;
  }


else
  {
    ROS_INFO("all parameters available");
    return 1;
    
  }
}

/**
 * @brief readParameters definition
 * 
 */
int HigherLevelInterface::readParameters()
{
 ROS_INFO("reading higher level interface parameters");
  
  
 p_modbusCommunicator->nh.getParam("/HigherLevelInterface/update_rate",m_updateRate);
 
 p_modbusCommunicator->nh.getParam("/HigherLevelInterface/subscriberTopic", m_subscriberTopic);


 p_modbusCommunicator->nh.getParam("/HigherLevelInterface/registerNumber",m_writeRegistornumber);

 
 return 0;
}


int HigherLevelInterface::writeData(int m_writeRegistornumber,int m_writeValue)
{
   ROS_INFO("linside writeData");

    p_modbusCommunicator->writeDataToRegister(m_writeRegistornumber,m_writeValue);
}

/**
 *@brief main function
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "modbus_communicator");
  HigherLevelInterface HigherLevelInterface;
  ros::Rate loop_rate(HigherLevelInterface.m_updateRate); 
  
  while (ros::ok())
  {

   ros::spinOnce();
   loop_rate.sleep();
  }

  

  return 0;
}
