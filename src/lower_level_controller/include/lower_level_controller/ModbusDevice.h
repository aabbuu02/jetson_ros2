#ifndef MODBUS_DEVICE_H
#define MODBUS_DEVICE_H

#include <ros/ros.h>
#include <ros/time.h>


#include <iostream>
#include <modbus/modbus.h>



#define READREGISTERNUMBER 100
#define STARTINGINDEX 100           /*Offset register index from which writing starts*/
#define WRITEREGISTERNUMBER 100

class ModbusDevice
{

public:
    /*Member Functions*/
    void connectToDevice(const std::string &ipAddress, int port);
    void readRegister();
    void writeRegister(int reg_add, int value);
    void writeRegisters(int addr, int nb, uint16_t data[]);
    uint32_t readEncoder(int encoder_num);
    uint16_t m_registerData[READREGISTERNUMBER];

private:
    /*Member Functions*/

    /*Member variables*/

    int m_port;
    std::string m_ipAddress;    
    modbus_t *plc;
};

#endif
