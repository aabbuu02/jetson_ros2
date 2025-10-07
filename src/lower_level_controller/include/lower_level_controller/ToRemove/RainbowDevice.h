#ifndef RAINBOW_DEVICE_H
#define RAINBOW_DEVICE_H

#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>

#include <modbus/modbus.h>

struct BatteryData
{
	double m_voltage = 0;
	double m_percentage = 0;
};


class RainbowDevice
{

public:
    /*Member Functions*/

    void connectToDevice(const std::string &left_motor,const std::string &right_motor, int baud_rate);
	void writeMotorCommands(int16_t left_command,int16_t right_command);
	void readMotorFeedback(uint32_t &left_feedback,uint32_t &right_feedback, BatteryData &batteryData);

private:

    /*Member Functions*/
	bool isDriverConnected(modbus_t *value);
	int32_t readEncoder(uint16_t lower_bit, uint16_t higher_bit);
    
	
	/*Member variables*/

	/*Communication specific parameters*/
    char m_parity = 'N';
    int m_dataBit = 8;
    int m_stopBit = 1;
    int m_slaveId = 1;


    modbus_t *p_modbusLeft;
    modbus_t *p_modbusRight;
};

#endif