/**
 * @file PlcCommunicator
 * @author Jishnu (jishnu@anscer.com)
 * @brief communication node used for communicating between two PLC s
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
 
#ifndef MODBUS_COMMUNICATOR
#define MODBUS_COMMUNICATOR

#include "ros/ros.h"
#include <modbus/modbus.h>
#include "iostream"
#include <stdlib.h>
#include <errno.h>

class ModbusCommunicator
{
    public:

     ModbusCommunicator();
     ~ModbusCommunicator();

     ros::NodeHandle nh;
     uint16_t * readData(int numberOfRegisters);
     int readRegister(int registerNumber);
     int writeDataToRegister(int registorNumber,uint32_t data);
     int writeDataToRegisters(int registorNumber,char data[],int bytes);
    private:
     bool checkParameters();
     int readParameters();
     int initiateConnection();

     const char* m_ip;
     std::string m_ipAddress;
     int m_port;
     uint16_t p_readData[9];
     uint16_t m_registerData[1];
     uint16_t m_writeData;
     modbus_t *p_ptx;  //object for modbus
     bool m_parameterExistanceFlag;

     
};
#endif