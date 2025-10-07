/**
 * @file shelf_reader.h
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef SHELF_READER_H
#define SHELF_READER_H

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <cstring>
#include <stdio.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <poll.h>
#include <unistd.h>
#include "anscer_msgs/srv/shelf_reader.hpp"

class ShelfReader : public rclcpp::Node
{
public:
    ShelfReader();
    ~ShelfReader();
    
    struct sockaddr_in m_socketAddress;
    struct pollfd fd;

private:
    // ROS2 service
    rclcpp::Service<anscer_msgs::srv::ShelfReader>::SharedPtr shelfSrvc;

    // Member functions
    void initializeParameters();
    void initializeConnection();
    void stopReadData();
    std::string startReadData();
    void shelfLocation(
        const std::shared_ptr<anscer_msgs::srv::ShelfReader::Request> req,
        std::shared_ptr<anscer_msgs::srv::ShelfReader::Response> res);

    // Member variables
    int m_socket = 0;
    int m_clientFD;
    int m_portNumber;
    std::string m_ipAddress;
    
    const char* m_triggerOn  = "||>TRIGGER ON\r\n";
    const char* m_triggerOff = "||>TRIGGER OFF\r\n";
};

#endif
