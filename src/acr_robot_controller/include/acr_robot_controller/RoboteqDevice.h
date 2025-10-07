/**
 * @file RoboteqDevice.h
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef ROBOTEQDEVICE_H
#define ROBOTEQDEVICE_H

#include <iostream>
#include <string>

// This includes the helper files we copied
#include "acr_robot_controller/ErrorCodes.h"
#include "acr_robot_controller/Constants.h"

class RoboteqDevice
{
public:
	RoboteqDevice();
	~RoboteqDevice();

	int Connect(std::string port);
	void Disconnect();

	int SetCommand(int command, int value);
	int SetCommand(int command, int index, int value);
	
	int GetValue(int query, int index, int &value);
	int GetValue(int query, int &value);

private:
	int fd;
	int write_port(const std::string &str);
	int read_port(std::string &response);
};
#endif


