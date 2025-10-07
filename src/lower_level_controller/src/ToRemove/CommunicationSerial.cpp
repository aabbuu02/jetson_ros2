#include "CommunicationSerial.h"

CommunicationSerial::CommunicationSerial(std::string serial_number, int speed)
{
    ROS_INFO("CommunicationSerial Constructor called");
    std::string portname;
    std::string rec_serial_number;
    std::cout << "Searching for " << serial_number << std::endl;
    for (int i = 0; i < 4; i++)
    {
        portname = "/dev/ttyACM" + std::to_string(i);
        readUsbIdentifiers(portname, rec_serial_number);
        if (rec_serial_number == serial_number)
        {
            std::cout << "SUCCESS: Found serial number " << serial_number << " on device " << portname << std::endl;
            break;
        }
    }
    fd = open(portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf("error %d opening %s", errno, portname.c_str());
    }
    setInterfaceAttributes(speed);
    setBlocking(1); // set blocking
}

/**
* @brief  Destructor for the ODriveMotorControl
*/

CommunicationSerial::~CommunicationSerial()
{
}

int CommunicationSerial::setInterfaceAttributes(int speed)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
    {
        printf(" error from tcgetattr: %d", errno);
        return -1;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break as \000 chars
    tty.c_iflag &= ~IGNBRK; // ignore break signal
    tty.c_lflag = 0;        // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                        // no remapping, no delays
    tty.c_cc[VMIN] = 0;                     // read doesn't block
    tty.c_cc[VTIME] = 5;                    // 0.5 seconds read timeout
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);        // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= 0;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        printf("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void CommunicationSerial::setBlocking(int should_block)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
    {
        printf("error %d from tggetattr", errno);
        return;
    }
    tty.c_cc[VMIN] = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout
    if (tcsetattr(fd, TCSANOW, &tty) != 0)
        printf("error %d setting term attributes", errno);
}

/* Source : https://stackoverflow.com/questions/49207803/how-to-get-the-usb-vid-pid-and-serial-number-of-a-device-in-ubuntu-using-c-fr?rq=1
*/
void CommunicationSerial::readUsbIdentifiers(std::string dev_path, std::string &serial_number)
{
    auto udev = udev_new();
    if (!udev)
        return;

    struct stat statbuf;
    if (stat(dev_path.c_str(), &statbuf) < 0)
        return;
    auto type = S_ISBLK(statbuf.st_mode) ? 'b' : S_ISCHR(statbuf.st_mode) ? 'c' : 0;

    auto opened_dev = udev_device_new_from_devnum(udev, type, statbuf.st_rdev);
    auto dev = opened_dev;

    while (dev != nullptr)
    {
        auto serial = udev_device_get_sysattr_value(dev, "serial");
        if (nullptr == serial)
        {
            dev = udev_device_get_parent(dev);
        }
        else
        {
            std::cout << "VID: " << udev_device_get_sysattr_value(dev, "idVendor") << std::endl;
            std::cout << "PID: " << udev_device_get_sysattr_value(dev, "idProduct") << std::endl;
            std::cout << "Serial Number: " << serial << std::endl;
            serial_number = serial;
            break;
        }
    }
    if (opened_dev)
        udev_device_unref(opened_dev);
    udev_unref(udev);
}

bool CommunicationSerial::serialWrite(std::string command)
{
    tcflush(fd, TCIFLUSH); // To flush the terminal I/O buffers
    int n = write(fd, command.c_str(), command.length());
    if (n != command.length())
        return false;
    return true;
}

/* Read Response from Serial Port */
std::string CommunicationSerial::serialRead(void)
{
    std::string response;
    memset(buffer, 0, SERIAL_BUFFER_SIZE);
    int n = read(fd, buffer, SERIAL_BUFFER_SIZE);
    if (n > 0)
    {
        response = buffer;
    }
    return response;
}
