/**
 * @file ModbusController.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "lower_level_controller/ModbusController.h"

ModbusController::ModbusController(const std::string& ip_address, int port, rclcpp::Logger logger)
: logger_(logger)
{
    modbusDevice_ = std::make_unique<ModbusDevice>(ip_address, port, logger_);
    RCLCPP_INFO(logger_, "ModbusController initialized.");
}

int ModbusController::readData(int register_addr)
{
    return modbusDevice_->readRegister(register_addr);
}

bool ModbusController::writeData(int register_addr, int value)
{
    return modbusDevice_->writeRegister(register_addr, value);
}
