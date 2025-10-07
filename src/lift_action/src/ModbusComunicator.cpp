/**
 * @file ModbusComunicator.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "lift_action/ModbusCommunicator.h"
#include <unistd.h> // For sleep()

/**
 * @brief Constructor: Initializes the ROS 2 node and starts the connection process.
 */
ModbusCommunicator::ModbusCommunicator(const rclcpp::NodeOptions & options)
    : Node("modbus_communicator_node", options)
{
    RCLCPP_INFO(this->get_logger(), "ModbusCommunicator constructor called");
    this->readParameters();
    this->initiateConnection();
}

/**
 * @brief Destructor
 */
ModbusCommunicator::~ModbusCommunicator()
{
    if (p_ptx) {
        modbus_close(p_ptx);
        modbus_free(p_ptx);
    }
}

/**
 * @brief Declares and reads parameters from the ROS 2 parameter server.
 */
void ModbusCommunicator::readParameters()
{
    RCLCPP_INFO(this->get_logger(), "Reading parameters");
    // Declare parameters with default values
    this->declare_parameter<std::string>("ip", "192.168.1.10");
    this->declare_parameter<int>("port", 502);

    // Get parameters
    this->get_parameter("ip", m_ipAddress);
    this->get_parameter("port", m_port);

    RCLCPP_INFO(this->get_logger(), "Parameters read successfully: IP=%s, Port=%d", m_ipAddress.c_str(), m_port);
}

/**
 * @brief Initiates the connection to the Modbus device.
 */
void ModbusCommunicator::initiateConnection()
{
    m_ip = m_ipAddress.c_str();
    p_ptx = modbus_new_tcp(m_ip, m_port);

    RCLCPP_INFO(this->get_logger(), "Attempting to connect to Modbus device at %s:%d...", m_ip, m_port);
    
    sleep(5); 

    if (modbus_connect(p_ptx) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Modbus Connection failed: %s", modbus_strerror(errno));
        modbus_free(p_ptx);
        throw std::runtime_error("Failed to connect to Modbus device.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Modbus device connected successfully");
    }
}

/**
 * @brief Reads a single register.
 */
int ModbusCommunicator::readRegister(int registerNumber)
{
    m_registerData[0] = 0;
    if (modbus_read_registers(p_ptx, registerNumber, 1, m_registerData) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to read data from register: %s", modbus_strerror(errno));
        return -1;
    }
    return m_registerData[0];
}

/**
 * @brief Reads multiple registers.
 */
uint16_t* ModbusCommunicator::readData(int numberOfRegisters)
{
    if (modbus_read_registers(p_ptx, 60, numberOfRegisters, p_readData) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to read data from registers: %s", modbus_strerror(errno));
        return nullptr;
    }
    return p_readData;
}

/**
 * @brief Writes data to a single register.
 */
int ModbusCommunicator::writeDataToRegister(int registorNumber, uint32_t data)
{
    if (modbus_write_register(p_ptx, registorNumber, data) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to write data to register %d: %s", registorNumber, modbus_strerror(errno));
        return -1;
    }
    return 0;
}

/**
 * @brief Writes data to multiple registers.
 */
int ModbusCommunicator::writeDataToRegisters(int registorNumber, char data[], int bytes)
{
    int k = 0;
    std::string arr[5] = {"", "", "", "", ""};
    for (int i = 0; i < 20; i += 1)
    {
        if (i == 0 || i == 4 || i == 8 || i == 12 || i == 16)
        {
            std::string temp = "";
            temp += data[i];
            temp += data[i + 1];
            temp += data[i + 2];
            temp += data[i + 3];
            arr[k] = temp;
            RCLCPP_DEBUG(this->get_logger(), "Segment %d: %s", k, arr[k].c_str());
            k++;
        }
    }
    uint16_t passingData[5] = {};
    for (int i = 0; i < 5; i++)
    {
        passingData[i] = std::stoi(arr[i]);
    }

    if (modbus_write_registers(p_ptx, registorNumber, 4, passingData) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to write data to registers %d: %s", registorNumber, modbus_strerror(errno));
        return -1;
    }
    return 0;
}

/**
 * @brief Main function to create and run the node.
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    try {
        auto node = std::make_shared<ModbusCommunicator>(options);
        rclcpp::spin(node);
    } catch (const std::runtime_error &e) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Modbus communicator failed to initialize: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
