/**
 * @file ModbusController.h
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef MODBUSCONTROLLER_H
#define MODBUSCONTROLLER_H

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "lower_level_controller/ModbusDevice.h"

class ModbusController
{
public:
    ModbusController(const std::string& ip_address, int port, rclcpp::Logger logger);
    int readData(int register_addr);
    bool writeData(int register_addr, int value);
private:
    std::unique_ptr<ModbusDevice> modbusDevice_;
    rclcpp::Logger logger_;
};
#endif //MODBUSCONTROLLER_H
