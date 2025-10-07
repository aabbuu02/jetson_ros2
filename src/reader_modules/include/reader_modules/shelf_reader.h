#ifndef SHELF_READER_H
#define SHELF_READER_H

#include <iostream>
#include <ros/ros.h>
#include <cstring>
#include <stdio.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <poll.h>




#include "anscer_msgs/ShelfReader.h"

class ShelfReader
{
public:
    ShelfReader();
    ~ShelfReader();

    struct sockaddr_in m_socketAddress;
    struct pollfd fd;


private:
    ros::NodeHandle nh;
    ros::NodeHandle nhp;
 
    ros::ServiceServer shelfSrvc;

    /*Member functions*/
    void initializeParameters();
    void initializeConnection();
    void stopReadData();
    std::string startReadData();

    bool shelfLocation(anscer_msgs::ShelfReader::Request &req, anscer_msgs::ShelfReader::Response &res);

    /*Member variables*/
    int m_socket = 0;
    int m_clientFD;
    int m_portNumber;
    std::string m_ipAddress;
    // char m_buffer[1024];

    
    
    char* m_triggerOn  = "||>TRIGGER ON\r\n";
	char* m_triggerOff = "||>TRIGGER OFF\r\n";
 
};
#endif
