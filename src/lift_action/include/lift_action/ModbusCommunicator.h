/**
 * @file ModbusCommunicator.h
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef MODBUS_COMMUNICATOR_H
#define MODBUS_COMMUNICATOR_H

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <modbus/modbus.h>

class ModbusCommunicator : public rclcpp::Node {
public:
    // Constructor matching the .cpp implementation
    ModbusCommunicator(const rclcpp::NodeOptions & options);
    
    // Destructor
    ~ModbusCommunicator();

    // Member functions
    void readParameters();
    void initiateConnection();
    int readRegister(int registerNumber);
    uint16_t* readData(int numberOfRegisters);
    int writeDataToRegister(int registorNumber, uint32_t data);
    int writeDataToRegisters(int registorNumber, char data[], int bytes);

private:
    // Member variables
    modbus_t* p_ptx;
    std::string m_ipAddress;
    int m_port;
    const char* m_ip;
    uint16_t m_registerData[256];
    uint16_t p_readData[256];
};

#endif // MODBUS_COMMUNICATOR_H
