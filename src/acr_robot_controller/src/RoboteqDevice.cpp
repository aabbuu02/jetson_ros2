/**
 * @file RoboteqDevice.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "acr_robot_controller/RoboteqDevice.h"
// These are the new, critical lines to include the helper files
#include "acr_robot_controller/Constants.h"
#include "acr_robot_controller/ErrorCodes.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

// This is the clean, non-ROS C++ implementation of the Roboteq driver.
RoboteqDevice::RoboteqDevice() { fd = -1; }
RoboteqDevice::~RoboteqDevice() { Disconnect(); }

int RoboteqDevice::Connect(std::string port)
{
	if(fd != -1)
	{
		std::cerr << "Error: Device is already connected." << std::endl;
		return RQ_ERR_OPEN_PORT;
	}

	if((fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) == -1)
	{
		std::cerr << "Error: Cannot open serial port. " << strerror(errno) << std::endl;
		return RQ_ERR_OPEN_PORT;
	}
	
	struct termios options;
	tcgetattr(fd, &options);
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options.c_oflag &= ~OPOST;
	cfsetospeed(&options, B115200);
	cfsetispeed(&options, B115200);
	tcsetattr(fd, TCSANOW, &options);
	return RQ_SUCCESS;
}

void RoboteqDevice::Disconnect()
{
	if(fd != -1)
		close(fd);
	fd = -1;
}

int RoboteqDevice::write_port(const std::string &str)
{
    if (fd == -1) return RQ_ERR_NOT_CONNECTED;
    int count = write(fd, str.c_str(), str.length());
    if (count < 0) return RQ_ERR_TRANSMIT_FAILED;
    return RQ_SUCCESS;
}

int RoboteqDevice::read_port(std::string &response)
{
    if (fd == -1) return RQ_ERR_NOT_CONNECTED;
    char buf[2048] = "";
    int count = read(fd, buf, 2048);
    if(count < 0) return RQ_ERR_SERIAL_IO;
    response = buf;
    return RQ_SUCCESS;
}

int RoboteqDevice::SetCommand(int command, int value)
{
    return SetCommand(command, -1, value);
}

int RoboteqDevice::SetCommand(int command, int index, int value)
{
	std::string commandStr = "!";
	std::string indexStr = "";

	if(command < 0 || command > 255)
		return RQ_INVALID_COMMAND;

	commandStr += Script_GetCommandString(command);

	if(index != -1)
	{
		if(index < 1 || index > 2)
			return RQ_INVALID_INDEX;
		
		indexStr = std::to_string(index);
		commandStr += " " + indexStr;
	}
	commandStr += " " + std::to_string(value) + "\r";
	return write_port(commandStr);
}

int RoboteqDevice::GetValue(int query, int &value)
{
    return GetValue(query, -1, value);
}

int RoboteqDevice::GetValue(int query, int index, int &value)
{
	std::string commandStr = "?";
	std::string indexStr = "";

	if(query < 0 || query > 255)
		return RQ_INVALID_QUERY;
	
	commandStr += Script_GetQueryString(query);

	if(index != -1)
	{
		if(index < 1 || index > 2)
			return RQ_INVALID_INDEX;
		
		indexStr = std::to_string(index);
		commandStr += " " + indexStr;
	}
	commandStr += "\r";
	
	if(write_port(commandStr) != RQ_SUCCESS)
		return RQ_ERR_TRANSMIT_FAILED;
	
	std::string response = "";
	usleep(10000); // Wait 10ms for a response
	if(read_port(response) != RQ_SUCCESS)
		return RQ_ERR_SERIAL_IO;

	// Parse response
	size_t pos = response.find("=");
	if(pos == std::string::npos)
		return RQ_UNEXPECTED_RESPONSE;

	try {
		value = std::stoi(response.substr(pos + 1));
	} catch (...) {
		return RQ_UNEXPECTED_RESPONSE;
	}

	return RQ_SUCCESS;
}


