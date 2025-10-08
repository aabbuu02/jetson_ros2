#include <ros/console.h>
#include "RainbowController.h"
/**
* @brief  Constructor for the RainbowController
*/

RainbowController::RainbowController(ros::NodeHandle &nh):nhp("~")
{

    ROS_INFO("RainbowController Constructor called ");
    initialize();

}

/**
 * @brief  Initializes the ODrive Motor Controller
 */
void RainbowController::initialize()
{
    initializeParameters();
    m_rainbowDevice.connectToDevice(m_leftMotor, m_rightMotor, m_baudRate);
}

/**
 * @brief  Initializes parameters from the parameter server
 */
void RainbowController::initializeParameters()
{
    nhp.param("rainbow/baud_rate", m_baudRate, 115200);
    nhp.param<std::string>("rainbow/left_motor", m_leftMotor, "/dev/lft_motor");
    nhp.param<std::string>("rainbow/right_motor", m_rightMotor, "/dev/rgt_motor");
}
/**
* @brief  Controlling the motor controller through motor commands
* @param  left and right RPS/CPS in double datatype
*/
void RainbowController::writeCommands(writeParameters &writeParams)
{
    ROS_WARN_NAMED("RainbowController","Make sure the writeCommands configured as per requiremtents.");
    int16_t leftRPM  = -1 * writeParams.m_leftMotorCommand;
    int16_t rightRPM = writeParams.m_rightMotorCommand;

    m_rainbowDevice.writeMotorCommands(leftRPM,rightRPM);
    ROS_INFO_NAMED("RainbowController","Write commands left RPM  : %d  right RPM : %d",leftRPM,rightRPM);
}

/**
* @brief  Reading the encoder readings from controller 
* @param  left and right ticks from the controller in long data type
*/
void RainbowController::readCommands(readParameters &readParam)
{
    uint32_t left_feedback  = 0;
    uint32_t right_feedback = 0;
    BatteryData batteryData;

    m_rainbowDevice.readMotorFeedback(left_feedback,right_feedback,batteryData);

    ROS_WARN_NAMED("RainbowController","Confirm whether swapping of the following feedbacks are by intention or accidental.FIX this...!!!");
    readParam.m_leftMotorEncoder  = right_feedback;
    readParam.m_rightMotorEncoder = -1 * left_feedback;
}

