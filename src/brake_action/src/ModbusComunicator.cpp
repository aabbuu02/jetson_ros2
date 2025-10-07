/**
 * @file ModbusComunicator.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "brake_action/ModbusCommunicator.h"
#include <cerrno>
#include <cstring>
#include <iostream>

ModbusCommunicator::ModbusCommunicator() : m_modbus_context(nullptr) {}

ModbusCommunicator::~ModbusCommunicator()
{
    if (m_modbus_context) {
        modbus_close(m_modbus_context);
        modbus_free(m_modbus_context);
    }
}

int ModbusCommunicator::initiateConnection(const std::string& ip, int port)
{
    m_modbus_context = modbus_new_tcp(ip.c_str(), port);
    if (m_modbus_context == NULL) {
        std::cerr << "Unable to create libmodbus context" << std::endl;
        return -1;
    }

    if (modbus_connect(m_modbus_context) == -1) {
        std::cerr << "Modbus connection failed: " << modbus_strerror(errno) << std::endl;
        modbus_free(m_modbus_context);
        m_modbus_context = nullptr;
        return -1;
    }

    return 0; // Success
}

int ModbusCommunicator::writeData(int address, int value)
{
    if (!m_modbus_context) {
        return -1; // Not connected
    }

    uint16_t value_to_write = static_cast<uint16_t>(value);
    if (modbus_write_register(m_modbus_context, address, value_to_write) == -1) {
        std::cerr << "Modbus write failed: " << modbus_strerror(errno) << std::endl;
        return -1; // Write failed
    }
    
    return 0; // Success
}
