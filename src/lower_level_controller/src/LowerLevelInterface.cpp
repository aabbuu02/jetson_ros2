#include "LowerLevelInterface.h"
#include <ros/console.h>

  /**
 * @brief  Constructor for the LowerLevelInterface
 */
LowerLevelInterface::LowerLevelInterface(): nhp("~")
{
    initializeParameters();

    p_getVariables          = new RosClass();
    p_lowerLevelComputation = new LowerLevelComputations();
    p_client                = new Client(nh, m_controllerType);
    p_highLevelController   = p_client->getHighLevelController();

      // authenticationProcess();
    m_lastTime = m_currentTime = Utilities::millis();

    ROS_INFO_NAMED("LowerLevelInterface", "LowerLevelInterface Constructor called ");
}

  /**
 * @brief  Destructor for the LowerLevelInterface
 */
LowerLevelInterface::~LowerLevelInterface()
{
}

  /**
 * @brief  Initialize parameters from the parameter server
 */
void LowerLevelInterface::initializeParameters()
{
    nhp.param("controller_type", m_controllerType, 1);
    nhp.param("publish_tf", m_publishTF, true);
    m_odom.pose.pose.orientation.w = 1.0;
}

  /**
 * @brief  Executes the authentication procedure and procedes if succeded
 */
void LowerLevelInterface::authenticationProcess()
{
    getMACAddress();
    if (m_macAddress == MACADDRESS)
    {
        ROS_INFO_NAMED("LowerLevelInterface", "LowerLevelInterface::Authentication succeeful");
    }
    else
    {
        ROS_FATAL_NAMED("LowerLevelInterface", "Authentication failed.Contact Vendor");
        exit(0);
    }
}

  /**
 * @brief Access the MAC address
 */
void LowerLevelInterface::getMACAddress()
{
    int max_buffer = 256;
    char buffer[max_buffer];
    std::string str = "cat /sys/class/net/eth0/address";

      /*The system command is often run first, before any output commands and the function
    returns an integer indicating success or failure, but not the output of the string*/

      /*Opens up a read-only stream, runs the command and captures the output,
     stuffs it into the buffer and returns it as a string.*/
    FILE *stream = popen(str.c_str(), "r");
    if (stream)
    {
        while (!feof(stream))
        {
            if (fgets(buffer, max_buffer, stream) != NULL)
            {

                m_macAddress.append(buffer);
                  // std::cout << m_address << std::endl;
            }
        }

          /*To avoid the new line character by the end of data*/
        m_macAddress = m_macAddress.substr(0, m_macAddress.length() - 1);
        pclose(stream);
    }
}

  /**
 * @brief  Entire control flow for the LowerLevelInterface
 */
void LowerLevelInterface::controlFlow()
{
      // ROS_INFO("Check 1");
    m_currentTime = Utilities::millis();
    m_deltaTime   = m_currentTime - m_lastTime;

    geometry_msgs::Twist cmdVel      = p_getVariables->getCommandVelocity();
    bool eStopStatus                 = p_getVariables->getEStopStatus();

    if (eStopStatus)
    {

        resetCmdVelocity(cmdVel);
    }

    p_lowerLevelComputation->initializeComputation(cmdVel, m_writeParams);

    p_getVariables->getWriteParams(m_writeParams);

      /*Passes the left and right motor command to motor controller*/
    p_highLevelController->writeCommands(m_writeParams);

      /*Reads the left and right encoder readings from the motor controller*/
    p_highLevelController->readCommands(m_readParams);
      // ROS_WARN_STREAM("Read L command "<< m_readParams.m_leftMotorSpeed<< "  Read R command "<<m_readParams.m_rightMotorSpeed);
      // Update the time stamp in the PLC
    m_writeParams.m_updatedTimeStamp = m_readParams.m_currentTimeStamp;

      /*Using the delta time difference and feedback from controller, the odometry is being updated*/
    p_lowerLevelComputation->updateOdometry(m_deltaTime, m_readParams, m_odom);

      /*Publishes the odometry using the setter function*/
    p_getVariables->sendOdometry(m_odom);

    p_getVariables->sendReadParameters(m_readParams);

      /*Comment the following line if the transformation is not required to be published*/
    if (m_publishTF)
    {
        p_getVariables->updateTF(m_odom);
    }
    m_lastTime = m_currentTime;
}

  /**
 * @brief  Resets the command velocity for robot
 */
void LowerLevelInterface::resetCmdVelocity(geometry_msgs::Twist &cmdVel)
{
    cmdVel.linear.x  = 0.0;
    cmdVel.angular.z = 0.0;
}

  /**
 * @brief Main function
 */
int main(int argc, char **argv)
{

    ros::init(argc, argv, "lower_level_controller");
    LowerLevelInterface lowerlevelcontroller;

    ros::Rate rate(30);

    while (ros::ok())
    {
        lowerlevelcontroller.controlFlow();
        rate.sleep();
        ros::spinOnce();
    }

    ros::shutdown();
    return 0;
}
