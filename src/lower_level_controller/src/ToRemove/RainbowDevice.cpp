#include "RainbowDevice.h"


/**
* @brief  Establishes the connection to the MODBUS device
* @param  left_motor  Device name of left motor as per udev rules
* @param  right_motor  Device name of right motor as per udev rules
  @param  baud_rate of the device
*/
void RainbowDevice::connectToDevice(const std::string &left_motor,const std::string &right_motor, int baud_rate)
{
    ROS_INFO_NAMED("RainbowDevice","Connecting to modbus device %s and %s  on  baudrate%d", left_motor.c_str(),right_motor.c_str(), baud_rate);

    p_modbusLeft  = modbus_new_rtu(left_motor.c_str(), baud_rate, m_parity, m_dataBit, m_stopBit);
    p_modbusRight = modbus_new_rtu(right_motor.c_str(), baud_rate, m_parity, m_dataBit, m_stopBit);
    if (!isDriverConnected(p_modbusLeft))
    {
        ROS_INFO_NAMED("RainbowDevice","Failed to connect to left motor");
    }
    else
    {
        ROS_INFO_NAMED("RainbowDevice","Successfully connected to left motor");
    }

    if (!isDriverConnected(p_modbusRight))
    {
        ROS_INFO_NAMED("RainbowDevice","Failed to connect to right motor");
    }
    else
    {
        ROS_INFO_NAMED("RainbowDevice","Successfully connected to right motor");
    }
}



/**
* @brief  Checks if the modbus RTU device is connected
* @returns true if device is connected else return false
*/
bool RainbowDevice::isDriverConnected(modbus_t *value)
{
    if (!value)
    {
        ROS_ERROR_STREAM("Failed to create context in ModBus RTU for driver");
        return false;
    }

    if (modbus_connect(value) == -1)
    {
        ROS_ERROR_STREAM("Failed to connect to modbus device: " << modbus_strerror(errno));
        modbus_free(value);
        return false;
    }

    if (modbus_set_slave(value, m_slaveId) == -1)
    {
        ROS_ERROR_STREAM("Failed to set slave ID: " << modbus_strerror(errno));
        modbus_free(value);
        return false;
    }

    ROS_INFO_STREAM("Connected to driver");

    return true;
}


/**
* @brief  Writes the motor command to the driver
* @param  left_command is the left motor command in RPM
* @param  right_command is the right motor command in RPM
*/
void RainbowDevice::writeMotorCommands(int16_t left_command,int16_t right_command)
{
	int register_address = 20000;
    if (modbus_write_register(p_modbusLeft, register_address, left_command) == -1)
    {
        ROS_ERROR_NAMED("RainbowDevice","Left motor write failed:  %d",modbus_strerror(errno));
    }
    if (modbus_write_register(p_modbusRight, register_address, right_command) == -1)
    {
        ROS_ERROR_NAMED("RainbowDevice","Right motor write failed:  %d",modbus_strerror(errno));
    }
}

/**
* @brief  Reads the data from Modbus RTU registers 
*/
void RainbowDevice::readMotorFeedback(uint32_t &left_feedback,uint32_t &right_feedback, BatteryData &batteryData)
{
    uint16_t leftRegisterData[100];
    uint16_t rightRegisterData[100];
	uint16_t batteryReading[2];

    if (modbus_read_registers(p_modbusLeft, 0, 30, leftRegisterData) == -1)
    {
        ROS_ERROR_NAMED("RainbowDevice","Left motor read failed:  %d",modbus_strerror(errno));
    }

    if (modbus_read_registers(p_modbusRight, 0, 30, rightRegisterData) == -1)
    {
        ROS_ERROR_NAMED("RainbowDevice","Right motor read failed:  %d",modbus_strerror(errno));
    }

    if(modbus_read_registers(p_modbusLeft, 0x80, 1, batteryReading) == -1)
    {
        ROS_ERROR_NAMED("RainbowDevice","Battery read failed:  %d",modbus_strerror(errno));
    }
	
	
	left_feedback  = readEncoder(leftRegisterData[25],leftRegisterData[26]);
	right_feedback = readEncoder(rightRegisterData[25],rightRegisterData[26]);

	ROS_ERROR_NAMED("RainbowDevice","Verify the following power computations ..!!");
    batteryData.m_voltage    = batteryReading[0]/10;
    batteryData.m_percentage = (batteryReading[0] - 180)/100;

}


/**
* @brief  Reads motor feedback ,which is a 32 bit value 
* @param  lower_bit the lower bit value
* @param  higher_bit the lower bit value
* @returns encoder value
*/
int32_t RainbowDevice::readEncoder(uint16_t lower_bit, uint16_t higher_bit)

{
    return (static_cast<uint32_t>(higher_bit) << 16) + static_cast<uint32_t>(lower_bit);
}


