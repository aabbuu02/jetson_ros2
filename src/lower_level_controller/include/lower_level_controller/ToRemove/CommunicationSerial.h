
#ifndef COMMUNICATION_SERIAL_H
#define COMMUNICATION_SERIAL_H

#include <termios.h>
#include <fcntl.h>
#include <string>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <libudev.h>
#include <sys/stat.h>
#include <iostream>
#include <ros/ros.h>
#define SERIAL_BUFFER_SIZE 128

class CommunicationSerial
{
public:
    CommunicationSerial(std::string serial_number, int speed);
    ~CommunicationSerial();

    bool serialWrite(std::string command);
    std::string serialRead(void);

private:
    int setInterfaceAttributes(int speed);
    void setBlocking(int should_block);
    void readUsbIdentifiers(std::string dev_path, std::string &serial_number);



    int fd;
    char buffer[SERIAL_BUFFER_SIZE];
};

#endif