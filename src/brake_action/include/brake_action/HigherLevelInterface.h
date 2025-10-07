/**
 * @file HigherLevelInterface.h
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

#ifndef HIGHER_LEVEL_INTERFACE
#define HIGHER_LEVEL_INTERFACE

#include "ModbusCommunicator.h"
#include "std_msgs/Bool.h"

#include <fstream>
#include<iostream>
#include <sstream>  // for string streams
#include <string>

class HigherLevelInterface
{
    public:
    HigherLevelInterface();
    ~HigherLevelInterface();


    int writeData(int m_writeRegistornumber,int m_writeValue);

    ModbusCommunicator *p_modbusCommunicator;
    int m_updateRate;


    
    void brakeActivatorCallback(const std_msgs::Bool &msg);

     
    private:
    bool  checkParameters();
    int readParameters();
    bool m_parameterExistanceFlag;
    

    int m_writeRegistornumber;
    int m_writeValue;

    std::string m_subscriberTopic;

    


 
    
    ros::Subscriber brakeActivator;

    

    

};

#endif
