/**
 * @file shelf_reader.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "reader_modules/shelf_reader.h"

/**
 * @brief Constructor for the ShelfReader
 */
ShelfReader::ShelfReader() : Node("shelf_reader")
{
    // Declare parameters
    this->declare_parameter<std::string>("ip_address", "192.168.1.166");
    this->declare_parameter<int>("port_number", 23);
    
    initializeParameters();
    // initializeConnection(); // Uncomment when ready to connect to hardware

    shelfSrvc = this->create_service<anscer_msgs::srv::ShelfReader>(
        "shelf_reader",
        std::bind(&ShelfReader::shelfLocation, this, std::placeholders::_1, std::placeholders::_2));
        
    RCLCPP_INFO(this->get_logger(), "ShelfReader node initialized");
}

/**
 * @brief Destructor for the ShelfReader
 */
ShelfReader::~ShelfReader()
{
    if (m_socket > 0) {
        close(m_socket);
    }
}

/**
 * @brief Initializes the parameters from the parameter server
 */
void ShelfReader::initializeParameters()
{
    this->get_parameter("ip_address", m_ipAddress);
    this->get_parameter("port_number", m_portNumber);
    
    RCLCPP_INFO(this->get_logger(), "IP: %s, Port: %d", m_ipAddress.c_str(), m_portNumber);
}

/**
 * @brief Initializes the connection with the shelf reading module
 */
void ShelfReader::initializeConnection()
{
    int connectionStatus = 0;
    if ((m_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Socket creation error");
        connectionStatus = -1;
    }

    m_socketAddress.sin_family = AF_INET;
    m_socketAddress.sin_port = htons(m_portNumber);

    if (inet_pton(AF_INET, m_ipAddress.c_str(), &m_socketAddress.sin_addr) <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid address. Address not supported");
        connectionStatus = -1;
    }

    if ((m_clientFD = connect(m_socket, (struct sockaddr *)&m_socketAddress, sizeof(m_socketAddress))) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Connection Failed");
        connectionStatus = -1;
    }
    
    if (connectionStatus < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed connecting with the COGNEX device");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Successfully connected with COGNEX device");
    }
}

/**
 * @brief Reads the barcode data from the shelf
 */
std::string ShelfReader::startReadData()
{
    int buffer[1024] = {0};
    int valRead;

    RCLCPP_INFO(this->get_logger(), "Starting reading data");
    int writeData = write(m_socket, m_triggerOn, strlen(m_triggerOn));
    if (writeData < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Couldn't trigger the device, initializing reattempt");
        close(m_clientFD);
        RCLCPP_ERROR(this->get_logger(), "Attempting re-connection with COGNEX device");
        initializeConnection();
    }

    fd.fd = m_socket;
    fd.events = POLLIN;
    int ret = poll(&fd, 1, 1000); // 1 second timeout
    
    switch (ret)
    {
    case -1:
        RCLCPP_ERROR(this->get_logger(), "Poll error");
        break;
    case 0:
        RCLCPP_WARN(this->get_logger(), "Poll timeout");
        break;
    default:
        valRead = recv(m_socket, buffer, sizeof(buffer), 0);
        break;
    }
    
    std::string data;
    for (int i = 0; i < valRead; i++)
    {
        data += std::to_string(buffer[i]);
    }

    return data;
}

/**
 * @brief Stops reading the device data from the sensor
 */
void ShelfReader::stopReadData()
{
    RCLCPP_INFO(this->get_logger(), "Stops reading COGNEX data");
    send(m_socket, m_triggerOff, strlen(m_triggerOff), 0);
}

/**
 * @brief Service callback that returns shelf barcode data
 */
void ShelfReader::shelfLocation(
    const std::shared_ptr<anscer_msgs::srv::ShelfReader::Request> req,
    std::shared_ptr<anscer_msgs::srv::ShelfReader::Response> res)
{
    // For testing in simulation use the following
    int64_t dummyReq = req->dummy_req;
    res->shelf_response = dummyReq;

    // Post testing in real robot use the following
    // res->shelf_response = startReadData();
    // stopReadData();

    RCLCPP_WARN(this->get_logger(), "ShelfReader requested dummy: %ld", dummyReq);
}

/**
 * @brief Main function
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ShelfReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
