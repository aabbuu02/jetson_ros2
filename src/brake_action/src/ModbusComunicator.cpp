/**
 * @file PlcCommunicator
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
#include "brake_action/ModbusCommunicator.h"

/**
 * @brief constructor definition
 * 
 */

ModbusCommunicator::ModbusCommunicator()
{
    ROS_INFO("ModbusCommunicator constructor called");
    m_parameterExistanceFlag=checkParameters();
    if(m_parameterExistanceFlag==0)
    {
        ROS_ERROR("failed to load parameters");
        
        exit(0);
    }
    else
    {
        int feedback=readParameters();
        if(feedback==0)
        {
            ROS_ERROR("failed to load Modbus Communicator parameters");
            
            exit(0);
        }
        else
        {
          ROS_INFO("Modbus Communicator parameters read successfully");

          initiateConnection();
        }
    }
}
/**
 * @brief Destroy the Modbus Commuinicator:: Modbus Communicator object
 * 
 */
ModbusCommunicator::~ModbusCommunicator()
{

}

/**
 * @brief for checking the availability of parameters
 * @brief returns true if all the parameters exists or return false
 */

bool ModbusCommunicator::checkParameters()
{
  ROS_INFO("checking existance of parameters");
  if(nh.hasParam("/ModbusCommunicator/ip")!=1)
  {
    ROS_WARN("ip param misssing");
    return 0;
  }
 
else  if(nh.hasParam("/ModbusCommunicator/port")!=1)
  {
    ROS_WARN("port param misssing");
    return 0;
  }


else
  {
    ROS_INFO("Parameter availability checking completed");
    return 1;
    
  }
}

/**
 * @brief read parameter definition
 * @brief for reading parameters from config file
 */
int ModbusCommunicator::readParameters()
{

  ROS_INFO("reading_parameters");
  
  nh.getParam("/ModbusCommunicator/ip", m_ipAddress);
  nh.getParam("/ModbusCommunicator/port",m_port);
 

}
/**
 * @brief initiateConnection definition
 * 
 */
int ModbusCommunicator::initiateConnection()
{
  m_ip=m_ipAddress.c_str();
  ROS_INFO(m_ip);
  ROS_INFO("%d",m_port);
    p_ptx=modbus_new_tcp(m_ip,m_port);

    if(modbus_connect(p_ptx)==-1)
    {
        ROS_INFO("connecting modbus device");
        ROS_ERROR("Modbus Connection failed with am error code %s",modbus_strerror(errno));
        modbus_free(p_ptx);
        return -1;
        exit(0);
    }
    else
    {
        ROS_INFO("Modbus device connected Successfully");
    }
}



/**
 * @brief writeData definition
 * @arguements register number
 */
int ModbusCommunicator::writeDataToRegister(int registorNumber,uint32_t data)
{
    
    if(modbus_write_register(p_ptx,registorNumber,data)==-1)
    {
        ROS_ERROR("failed to write data in to register number %d with an error number %s",registorNumber,modbus_strerror(errno));
        modbus_free(p_ptx);
        return -1;
    }
    else
    {
      ROS_INFO("written to %d value %d",registorNumber,data);
    }
    return 0;
}

