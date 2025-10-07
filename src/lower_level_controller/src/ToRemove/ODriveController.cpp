#include <ros/console.h>
#include "ODriveController.h"
/**
* @brief  Constructor for the HighLevelController
*/

ODriveController::ODriveController(ros::NodeHandle &nh):nhp("~")
{
    ROS_INFO("ODriveController Constructor called ");
    initialize(nh);
}

// /**
// * @brief  Destructor for the HighLevelController
// */

// virtual ModbusController::~ModbusController()
// {
// }

/**
* @brief  Initializes the ODrive Motor Controller
*/
void ODriveController::initialize(ros::NodeHandle &nh)
{
    initializeParameters();

    p_communicationSerial = new CommunicationSerial(m_odriveSerialNumber, m_baudRate);
    connectToDevice();
}


/**
* @brief  Initializes parameters from the parameter server
*/
void ODriveController::initializeParameters()
{
    nhp.param("baud_rate", m_baudRate, 115200);
    m_odriveSerialNumber = ODRIVESERIALNUMBER;
}

/**
* @brief  Configure the ODrive motor driver
*/
void ODriveController::connectToDevice()
{

    ROS_INFO("ODriveMotorControl::configure Driver initiate");
    int numberOfJoints = 2;

    for (std::size_t jointId = 0; jointId < numberOfJoints; ++jointId)
    {
        /* Initialise the O-Drive Serial Interface*/

        std::shared_ptr<ODriveDevice> individualMotorAxis(new ODriveDevice(*p_communicationSerial, jointId));
        p_ODriveVector.push_back(individualMotorAxis);

        long response = p_ODriveVector[jointId]->getError();
        if (response != 0)
        {
            ROS_ERROR_NAMED("ODriveController","%lu: Axis Error Status: %lu", jointId, response);
        }


        ROS_INFO_NAMED("ODriveController","Initialised O-Drive Serial Interface for axis: %lu", jointId);

        p_ODriveVector[jointId]->writeCommand(ODRV_WATCH_DOG_TIMER_W, jointId);
        ROS_INFO_NAMED("ODriveController"," Setting watch dog timer for axis: %lu", jointId);


        p_ODriveVector[jointId]->writeProperty(ODRV_AXIS_ERROR_W);
        ROS_INFO_NAMED("ODriveController","Clearing  error for axis: %lu", jointId);

        p_ODriveVector[jointId]->writeProperty(ODRV_AXIS_ERROR_W);
        ROS_INFO_NAMED("ODriveController","Clearing  error for axis: %lu", jointId);

        response = p_ODriveVector[jointId]->getError();

        if (response != 0)
        {
            ROS_ERROR_NAMED("ODriveController","%lu: Axis Error Status: %lu", jointId, response);            
        }

        ROS_INFO(" AXIS_STATE_CLOSED_LOOP_CONTROL %lu", jointId);
        if (p_ODriveVector[jointId]->runState(AXIS_STATE_CLOSED_LOOP_CONTROL, STATE_RQST_NO_WAIT) != true)
        {
            ROS_ERROR("%s, Unable to enter in Closed Loop Control", jointId);
        }


        p_ODriveVector[jointId]->writeCommand(ODRV_WATCH_DOG_TIMER_W, jointId);
        ROS_INFO_NAMED("ODriveController","Sending watch dog timer for axis: %lu", jointId);

        double startingVelocity = 0;
        p_ODriveVector[jointId]->writeProperty(ODRV_VELOCITY, startingVelocity);
    }

    ROS_WARN_NAMED("ODriveController","ODrive Motor Driver Configured");
}

/**
* @brief  Controlling the motor controller through motor commands
* @param  left and right RPS/CPS in double datatype
*/
void ODriveController::writeCommands(writeParameters &writeParams)
{
    /*Pass the Left CPS and Right CPS*/

    p_ODriveVector[1]->writeProperty(ODRV_VELOCITY, writeParams.m_leftMotorCommand);
    usleep(10000);
    p_ODriveVector[0]->writeProperty(ODRV_VELOCITY, writeParams.m_rightMotorCommand);
    usleep(10000);

    ROS_INFO_NAMED("ODriveController","Write commands left CPS  : %f  right CPS : %f",writeParams.m_leftMotorCommand,writeParams.m_rightMotorCommand);
}

/**
* @brief  Reading the encoder readings from controller 
* @param  left and right ticks from the controller in long data type
*/
void ODriveController::readCommands(readParameters &readParam)
{
    // /*Reads the entire read register data*/
    std::string leftMotor = "leftMotor";
    std::string rightMotor= "rightMotor";
    ROS_INFO_NAMED("ODriveController","Read commands left CPS  : %f  right CPS : %f",readParam.m_leftEncoder,readParam.m_rightEncoder);

}



/**
* @brief Sets the watch dog timer for each axis
*/
void ODriveController::watchDogTimer()
{
   // ROS_WARN(" watchDogTimer()");
    p_ODriveVector[0]->writeCommand(ODRV_WATCH_DOG_TIMER_W, 0);
    p_ODriveVector[1]->writeCommand(ODRV_WATCH_DOG_TIMER_W, 1);
}