/**
 * @file ModbusCommunicator.h
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef MODBUS_COMMUNICATOR_H
#define MODBUS_COMMUNICATOR_H

#include <string>
#include <modbus/modbus.h>

// This is now a pure C++ class with no ROS dependencies.
class ModbusCommunicator
{
public:
    ModbusCommunicator();
    ~ModbusCommunicator();
    
    // Initialize connection with parameters passed from the ROS 2 node.
    int initiateConnection(const std::string& ip, int port);
    int writeData(int address, int value);

private:
    modbus_t* m_modbus_context;
};

#endif // MODBUS_COMMUNICATOR_H


