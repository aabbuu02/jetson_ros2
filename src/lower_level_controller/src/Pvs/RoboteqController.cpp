#include <ros/console.h>
#include "RoboteqController.h"

/**
* @brief  Constructor for the RoboteqController
*/
RoboteqController::RoboteqController(ros::NodeHandle &nh)
{

    ROS_INFO("RoboteqController Constructor called ");
    initialize(nh);


    robotTimer = nh.createTimer(ros::Duration(5), &RoboteqController::robotTimerCallback, this);


    /*Configure the following functions as per reqmnt*/
}

/**
* @brief  Destructor for the RoboteqController
*/

RoboteqController::~RoboteqController()
{
    ROS_INFO("Roboteq destructor called,disconnecting the device");
    device.Disconnect();
}

/**
* @brief  Configure the user specfic motor driver.
* This function shall be used to do the initial setups for the motor driver used
*/





void RoboteqController::robotTimerCallback(const ros::TimerEvent &time)
{
if(m_connected)
	return;

initialize(nh);

}


void RoboteqController::initialize(ros::NodeHandle &nh)
{
        /**
    *@brief for connecting to roboteq device
    */

    int statusConnection = device.Connect("/dev/roboteq_driver");

    if (statusConnection != RQ_SUCCESS)
    {
        ROS_ERROR_STREAM( "Error connecting to device: " << statusConnection << "." );
        //exit(0);
        //return -1;
    }
    else if (statusConnection == RQ_SUCCESS)
    {   m_connected = true;
        ROS_INFO_STREAM("Roboteq device connected successfully ");
    }
}


/**
* @brief  Controlling the motor controller through motor commands
* @param  writeParameters struct
*/
void RoboteqController::writeMotorControl(writeParameters &writeParams)
{
    int leftMotorCommand    = device.SetCommand(_MOTVEL, 1, writeParams.m_leftMotorCommand);
    int rightMotorCommand   = device.SetCommand(_MOTVEL, 2, writeParams.m_rightMotorCommand);
    
    ROS_DEBUG_NAMED("motor_command", "Motor Commands | Left RPM: %d Right RPM: %d",writeParams.m_leftMotorCommand,writeParams.m_rightMotorCommand);
}

/**
* @brief  Reading the encoder readings from controller 
* @param  readParameters struct
*/
void RoboteqController::readEncoderValues(readParameters &readParam)
{
    int leftMotorRead   = device.GetValue(_ABCNTR, 2,m_leftEncoderReading);
    int rightMotorRead  = device.GetValue(_ABCNTR, 1,m_rightEncoderReading);

    int leftMotorSpeed  = device.GetValue(_ABSPEED, 2,m_leftSpeed);
    int rightMotorSpeed = device.GetValue(_ABSPEED, 1,m_rightSpeed);

    readMotorControllerFlags();

    readParam.m_rightEncoder = m_rightEncoderReading;
    readParam.m_leftEncoder  = m_leftEncoderReading;

    readParam.m_leftSpeed    = m_leftSpeed;
    readParam.m_rightSpeed   = m_rightSpeed;

    // ROS_INFO("Left Encoder :  %d   Right Encoder  : %d ",m_leftEncoderReading,m_rightEncoderReading);
    // ROS_WARN("Encoder | Left Speed :  %d   Right Speed  : %d ",m_leftSpeed,m_rightSpeed);

}

anscer_msgs::MotorDiagnosticsArray RoboteqController::getMotorDiagnosticsArray()
{
    return m_motorDiagArr;
}

void RoboteqController::readMotorControllerFlags()
{
    int leftFfRetVal = device.GetValue(_FF, 2, m_leftFaultFlags);
    int rightFfRetVal = device.GetValue(_FF, 1, m_rightFaultFlags);

    int leftFsRetVal = device.GetValue(_FS, 2, m_leftStatusFlags);
    int rightFsRetVal = device.GetValue(_FS, 1, m_rightStatusFlags);

    m_motorDiagArr.motor_diagnostics.clear();
    
    m_leftDiag.motor_name = "left_motor";
    setMotorFaultFlags(m_leftFaultFlags, m_leftDiag.fault_flags);
    setMotorStatusFlags(m_leftStatusFlags, m_leftDiag.status_flags);
    m_motorDiagArr.motor_diagnostics.push_back(m_leftDiag);

    m_rightDiag.motor_name = "right_motor";
    setMotorFaultFlags(m_rightFaultFlags, m_rightDiag.fault_flags);
    setMotorStatusFlags(m_rightStatusFlags, m_rightDiag.status_flags);
    m_motorDiagArr.motor_diagnostics.push_back(m_rightDiag);
}

void RoboteqController::setMotorFaultFlags(int setfaultFlags, anscer_msgs::MotorFaultFlags &motorFaultFlags)
{
    std::bitset<16> setFaultBits(setfaultFlags);

    motorFaultFlags.overheat = setFaultBits[0];
    motorFaultFlags.overvoltage = setFaultBits[1];
    motorFaultFlags.undervoltage = setFaultBits[2];
    motorFaultFlags.short_circuit = setFaultBits[3];
    motorFaultFlags.emergency_stop = setFaultBits[4];
    motorFaultFlags.motor_setup_fault = setFaultBits[5];
    motorFaultFlags.mosfet_failure = setFaultBits[6];
    motorFaultFlags.default_configuration = setFaultBits[7];
}

void RoboteqController::setMotorStatusFlags(int setStatusFlags, anscer_msgs::MotorStatusFlags &motorStatusFlags)
{
    std::bitset<16> setStatusBits(setStatusFlags);

    motorStatusFlags.serial_mode = setStatusBits[0];
    motorStatusFlags.pulse_mode = setStatusBits[1];
    motorStatusFlags.analog_mode = setStatusBits[2];
    motorStatusFlags.power_stage_off = setStatusBits[3];
    motorStatusFlags.stall_detected = setStatusBits[4];
    motorStatusFlags.at_limit = setStatusBits[5];
    motorStatusFlags.unused = setStatusBits[6];
    motorStatusFlags.micro_basic_script_running = setStatusBits[7];
    motorStatusFlags.motor_tuning_mode = setStatusBits[8];
}
