#include <ros/console.h>
#include "ODriveDevice.h"


ODriveDevice::ODriveDevice(CommunicationSerial &serial, int motor_number) : serial(serial), motorNumber(motor_number)
{
  ROS_INFO("ODriveSerialInterface constructor  : %d",motor_number);
}

void ODriveDevice::writeCommand(std::string property, int motor_id)
{
    property += std::to_string(motor_id);
    property += ' ';
    property += '\n';
    serial.serialWrite(property);
    //	std::cout<<"sending watch_dog_timer command "<< property<<std::endl;
}


void ODriveDevice::writeProperty(std::string property)
{
    sendCommand(property, PR_WRITE, AXIS_COMMAND);
}


void ODriveDevice::writeProperty(std::string property, double value)
{
    property += ' ';
    property += std::to_string(value);
    sendCommand(property, PR_WRITE, AXIS_COMMAND);
}


void ODriveDevice::writeProperty(std::string property, long value)
{
    property += ' ';
    property += std::to_string(value);
    sendCommand(property, PR_WRITE, AXIS_COMMAND);
}


void ODriveDevice::readProperty(std::string property, double &value)
{
    std::string response;
    sendCommand(property, PR_READ, AXIS_COMMAND);
    response = serial.serialRead();
    double x = std::stod(response);
    value = x;
}

void ODriveDevice::readProperty(std::string property, long &value, std::string useCase)
{
    std::string response;
    sendCommand(property, PR_READ, AXIS_COMMAND);
    response = serial.serialRead();
    try
    {

        long x = std::stol(response);
        value = x;
    }
    catch (const std::invalid_argument)
    {
        std::cout << "RA-1 Invalid argument" << useCase << "\n";
    }
}

long ODriveDevice::getError()
{
    long response = 0;
    std::string getError = "getError";
    readProperty(ODRV_AXIS_ERROR_R, response,getError);
    return response;
}



void ODriveDevice::sendCommand(std::string property, bool read_write, bool axis)
{
    std::string command;
    if (read_write == PR_WRITE)
        command.append("w ");
    else if (read_write == PR_READ)
        command.append("r ");
    if (axis == AXIS_COMMAND)
    {
        command.append("axis");
        command += std::to_string(motorNumber);
        command += '.';
    }

    command.append(property);
    command += '\n';
    serial.serialWrite(command);
    usleep(WAIT_TO_NEXT_COMMAND);
}


bool ODriveDevice::runState(long requested_state, bool wait_for_idle)
{
    writeProperty(ODRV_REQUEST_STATE, requested_state);

    // Check whether system has entered the requested_state
    std::string state = "state";
    long current_state = AXIS_STATE_UNDEFINED;
    readProperty(ODRV_CURRENT_STATE, current_state,state);
    if (current_state != requested_state)
        return false;

    // If we need to wait for AXIS_STATE_IDLE
    if (wait_for_idle == STATE_RQST_WAIT)
        return waitToIdle();

    return true;
}


bool ODriveDevice::waitToIdle()
{
    long t = 0;
    long state = 0;
    std::string waitIdle = "wait_idle";
    for (t = 0; t < STATE_RQST_COUNTS; t++)
    {
        readProperty(ODRV_CURRENT_STATE, state,waitIdle);
        if (state == AXIS_STATE_IDLE)
        {
            // If State becomes IDLE, return true i.e. success
            return true;
        }
    }
    return false;
}