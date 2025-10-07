#include "shelf_reader.h"
#include <ros/console.h>

/**
 * @brief  Constructor for the ShelfReader
 */
ShelfReader::ShelfReader() : nhp("~")
{
    //initializeParameters();
    //initializeConnection();

    shelfSrvc = nh.advertiseService("shelf_reader", &ShelfReader::shelfLocation, this);
}

/**
 * @brief  Destructor for the ShelfReader
 */
ShelfReader::~ShelfReader()
{
}

/**
 * @brief  Initializes the parameters from the parameter server
 */
void ShelfReader::initializeParameters()
{
    nhp.param<std::string>("ip_address", m_ipAddress, "192.168.1.166");\
    nhp.param("port_number", m_portNumber, 23);
}


/**
 * @brief  Initializes the connection with the shelf reading module
 */
void ShelfReader::initializeConnection()
{
    int connectionStatus = 0;
    if ((m_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        ROS_ERROR_NAMED("ShelfReader", "Socket creation error");
        connectionStatus = -1;
    }

    m_socketAddress.sin_family = AF_INET;
    m_socketAddress.sin_port = htons(m_portNumber);

    // Convert IPv4 and IPv6 addresses from text to binary
    // form
    if (inet_pton(AF_INET, m_ipAddress.c_str(), &m_socketAddress.sin_addr) <= 0) // Incase if it fails replace m_ipAddress by "192.168.1.166"
    {
        ROS_ERROR_NAMED("ShelfReader", "Invalid address Address not supported ");
        connectionStatus = -1;
    }

    if ((m_clientFD = connect(m_socket, (struct sockaddr *)&m_socketAddress, sizeof(m_socketAddress))) < 0)
    {
        ROS_ERROR_NAMED("ShelfReader", "Connection Failed");
        connectionStatus = -1;
    }
    if (connectionStatus < 0)
    {
        ROS_ERROR_NAMED("ShelfReader", "Failed connecting with the COGNEX device");
    }
    else
    {
        ROS_ERROR_NAMED("ShelfReader", "Successfully connected with COGNEX device");
    }
}

/**
 * @brief  Reads the barcode data from the shelf
 * @returns the read barcode value in string format
 */
std::string ShelfReader::startReadData()
{

    int buffer[1024] = {0};
    int valRead;

    std::cout << "strating reading data" << std::endl;
    int writeData = write(m_socket, m_triggerOn, strlen(m_triggerOn));
    if (writeData < 0)
    {
        ROS_ERROR_NAMED("ShelfReader", "Couldn't trigger the device,initializing reattempt");
        close(m_clientFD);
        ROS_ERROR_NAMED("ShelfReader", "Attempting re-connection with COGNEX device");
        initializeConnection();
    }

    fd.fd = m_socket; // your socket handler
    fd.events = POLLIN;
    int ret = poll(&fd, 1, 1000); // 1 second for timeout
    switch (ret)
    {
    case -1:
        // Error
        break;
    case 0:
        // Timeout
        break;
    default:
        valRead = recv(m_socket, buffer, sizeof(buffer), 1024); // get your data
        break;
    }
    // std::cout << "valread :" << valRead << "- data: " << buffer << std::endl;
    
    std::string data;
    for (int i = 0; i < valRead; i++)
    {
        // std::cout << buffer[i];
        std::string data_read;
        data_read = buffer[i];
        data.append(data_read);
    }

    return data;
}

/**
* @brief  Stops reading the device data from the sensor
*/
void ShelfReader::stopReadData()
{
    ROS_ERROR_NAMED("ShelfReader", "Stops reading COGNIX data");
	send(m_socket, m_triggerOff, strlen(m_triggerOff), 0);
}


/**
* @brief  Service callback that returns nearest charger location to the robot
*/
bool ShelfReader::shelfLocation(anscer_msgs::ShelfReader::Request &req, anscer_msgs::ShelfReader::Response &res)
{
    /*For testing in simulation use the following and comment the lines below that*/

    int64_t dummyReq       = req.dummy_req;
    res.shelf_response     = dummyReq ;

    /*Post testing in real robot use the following and comment the above lines*/

    // res.shelf_response     = startReadData();
    // stopReadData();


    ROS_WARN("ShelfReader  requested dummy  : %ld",dummyReq);

    return true;
}


/*
 * @brief Main function
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "shelf_reader");
    ShelfReader shelfReader;

    ros::Rate r(1);
    while (ros::ok())
    {

        r.sleep();
        ros::spinOnce();
    }

    ros::shutdown();
    return 0;
}
