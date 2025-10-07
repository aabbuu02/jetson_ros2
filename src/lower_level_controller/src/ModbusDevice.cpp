/**
 * @file ModbusDevice.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "lower_level_controller/ModbusDevice.h"
#include <stdexcept>

ModbusDevice::ModbusDevice(const std::string& ip_address, int port, rclcpp::Logger logger) : logger_(logger)
{
    ctx_ = modbus_new_tcp(ip_address.c_str(), port);
    if (ctx_ == nullptr) {
        throw std::runtime_error("modbus_new_tcp failed");
    }
    if (modbus_connect(ctx_) == -1) {
        modbus_free(ctx_);
        throw std::runtime_error("modbus_connect failed");
    }
    RCLCPP_INFO(logger_, "Modbus TCP connection successful.");
}

ModbusDevice::~ModbusDevice()
{
    if (ctx_ != nullptr) {
        modbus_close(ctx_);
        modbus_free(ctx_);
    }
}

int ModbusDevice::readRegister(int addr)
{
    uint16_t val;
    if (modbus_read_registers(ctx_, addr, 1, &val) == -1) {
        RCLCPP_ERROR(logger_, "Failed to read register %d: %s", addr, modbus_strerror(errno));
        return -1;
    }
    return val;
}

bool ModbusDevice::writeRegister(int addr, int value)
{
    if (modbus_write_register(ctx_, addr, value) == -1) {
        RCLCPP_ERROR(logger_, "Failed to write to register %d: %s", addr, modbus_strerror(errno));
        return false;
    }
    return true;
}
