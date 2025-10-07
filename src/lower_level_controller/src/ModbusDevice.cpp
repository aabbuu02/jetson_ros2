#include <ros/console.h>
#include "ModbusDevice.h"


/**
* @brief  Establishes the connection to the MODBUS device
* @param  ipAddress  IP address of the MODBUS device 
  @param  port the port number of the device
*/
void ModbusDevice::connectToDevice(const std::string &ipAddress,int port)
{

    ROS_INFO_NAMED("ModbusController","Connecting to modbus device on %s/%d", ipAddress.c_str(), port);
    plc = modbus_new_tcp(ipAddress.c_str(), port);
    if (plc == NULL)
    {
        ROS_FATAL_NAMED("ModbusController","Unable to allocate libmodbus context\n");
        return;
    }
    if (modbus_connect(plc) == -1)
    {
        ROS_FATAL_NAMED("ModbusController","Failed to connect to modbus device -  %s",modbus_strerror(errno));
        modbus_free(plc);
        return;
    }
    else
    {
        ROS_INFO_NAMED("ModbusController","Connection to modbus device establisher");
    }

}





/**
* @brief  Reads the data from PLC registers 
*/
void ModbusDevice::readRegister()
{
    //modbus_read_register does not throw any exeception. No need for try catch block
    
	try
    {
        int reg_data_test = modbus_read_registers(plc, 0, READREGISTERNUMBER, m_registerData);

        if (reg_data_test == -1) 
        {
            ROS_ERROR_STREAM("Unable to read reg addr: "<<modbus_strerror(errno));
            throw -1; //The return from the modbus_read_register function indicates an error anyway
        }
    }
    catch (int e) //Redundant
    {
        ROS_ERROR("modbus_error");
    }

}


/**
* @brief  Writing the value to PLC register
* @param  regAddress  register address to which the value is to be written
* @param  value value to be written to the register
*/
void ModbusDevice::writeRegister(int registerAddress, int value)
{
	ROS_INFO("writeRegister()");

    try
    {
        int reg_data_test = modbus_write_register(plc, registerAddress, value);
        if (reg_data_test == -1)
        {
            ROS_ERROR("Unable to write  reg addr %d:", registerAddress);
            throw value;
        }
    }
    catch (int e)
    {
        ROS_ERROR("modbus error value %d: out of range", e);
    }
}

/*Reference : https://libmodbus.org/docs/v3.0.8/modbus_write_registers.html*/
/**
* @brief  Writing the value to PLC registers
* @param  write the content of the nb holding registers from the array src at address addr of the remote device.
*/
void ModbusDevice::writeRegisters(int addr, int nb, uint16_t data[])
{
    try
    {
        int reg_data_test = modbus_write_registers(plc, addr, nb, data);
        if (reg_data_test == -1)
        {
            ROS_ERROR("Unable to write register address");
            throw -1;
        }
    }
    catch (int e)
    {
        ROS_ERROR("Unable to write registers address");
    }
}

/**
* @brief  Reads motor feedback ,which is a 32 bit value 
* @param  encoderNuber the index of the encoder to be read
* @returns distance travelled in cms
*/
uint32_t ModbusDevice::readEncoder(int encoderNumber)
{
    uint32_t distanceInCm = (m_registerData[encoderNumber] * 65536) + m_registerData[encoderNumber + 1];
    return distanceInCm;
}




