/**
 * @file ModbusDevice.h
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef MODBUSDEVICE_H
#define MODBUSDEVICE_H

#include <modbus.h>
#include <string>
#include "rclcpp/rclcpp.hpp"

class ModbusDevice
{
public:
    ModbusDevice(const std::string& ip_address, int port, rclcpp::Logger logger);
    ~ModbusDevice();
    int readRegister(int addr);
    bool writeRegister(int addr, int value);
private:
    modbus_t *ctx_;
    rclcpp::Logger logger_;
};
#endif //MODBUSDEVICE_H
