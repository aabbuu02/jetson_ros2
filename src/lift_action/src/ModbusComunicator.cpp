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
#include "lift_action/ModbusCommunicator.h"

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
  
  return 1;
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
  
  ROS_INFO("connecting modbus device");
  sleep(60);

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
 * @brief readRegister definition
 * @brief readRegister will readData from single register and return integer value
 * @arguements registerNumber to be read
 */
int ModbusCommunicator::readRegister(int registerNumber)
{
  m_registerData[0]=0;

  // ROS_INFO("reading data from register  %d",registerNumber);
  if(modbus_read_registers(p_ptx,registerNumber,1,m_registerData)==-1)  //reading from modbus registers
    {
        ROS_ERROR("failed to read data from register %s",modbus_strerror(errno));
        //modbus_free(p_ptx);
        
    }
  else
    {
      return m_registerData[0];
    }

}

/**
 * @brief readData definition
 * @brief read data will be returned std::vector<uint16_t>
 * @arguements number of registers to read data
 * 
 */
uint16_t * ModbusCommunicator::readData(int numberOfRegisters)
{
  if(modbus_read_registers(p_ptx,60,numberOfRegisters,p_readData)==-1)  //reading from modbus registers
    {
        ROS_ERROR("failed to read data from registers %s",modbus_strerror(errno));
        //modbus_free(p_ptx);
        
    }
  else
    {
      return p_readData;
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
 //       modbus_free(p_ptx);
        return -1;
    }
    else
    {
      //ROS_INFO("written to %d value %d",registorNumber,data);
    }
    return 0;
}

/**
 * @brief writeData to registers
 * @arguements registers number number of bytes and data 
 */
int ModbusCommunicator::writeDataToRegisters(int registorNumber,char data[],int bytes)
{
  int k=0;
  
    std::string arr[5]={"","","",""};
    
    for(int i=0;i<20;i+=1)
    {
        
     if(i==0 || i==4 || i==8|| i==12 ||i==16)
      {
        
        std::string temp="";
        temp+=data[i];
        temp+=data[i+1];
        temp+=data[i+2];
        temp+=data[i+3];

        arr[k]= temp;
      std::cout<<arr[k]<<std::endl;
      k++;
      }
      
    }
    uint16_t passingData[5]={};
    for(int i=0;i<5;i++)
    {
      passingData[i]=stoi(arr[i]);
    }

    if(modbus_write_registers(p_ptx,registorNumber,4,passingData)==-1)
    {
        ROS_ERROR("failed to write data in to register number %d with an error number %s",registorNumber,modbus_strerror(errno));
   //     modbus_free(p_ptx);
        return -1;
    }
    return 0;
}
