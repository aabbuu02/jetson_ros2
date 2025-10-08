#include <ros/console.h>
#include "RoboteqController.h"
/**
* @brief  Constructor for the HighLevelController
*/

RoboteqController::RoboteqController(ros::NodeHandle &nh):nhp("~")
{
    ROS_INFO("RoboteqController Constructor called ");
    initialize();
}



/**
* @brief  Initializes the PLC controller
*/
void RoboteqController::initialize()
{
    initializeParameters();

    /*Connects to the  Roboteq Driver*/    
    int statusConnection = m_roboteqDevice.Connect(m_roboteqPort);
    if (statusConnection != RQ_SUCCESS)
    {
        std::cout << "Error connecting to device: " << statusConnection << "." << std::endl;
        exit(0);
    }
    else if (statusConnection == RQ_SUCCESS)
    {
        std::cout << "Roboteq device connected successfully " << std::endl;
    }
}


/**
* @brief  Initializes parameters from the parameter server
*/
void RoboteqController::initializeParameters()
{
    /*Configurations for initializing communication*/
    nhp.param<std::string>("roboteq_port", m_roboteqPort, "/dev/roboteq_driver");
}


/**
* @brief  Controlling the motor controller through motor commands
* @param  left and right RPS/CPS in double datatype
*/
void RoboteqController::writeCommands(writeParameters &writeParams)
{
    int leftMotorCommand    = m_roboteqDevice.SetCommand(_MOTVEL, 1, writeParams.m_leftMotorCommand);
    int rightMotorCommand   = m_roboteqDevice.SetCommand(_MOTVEL, 2, writeParams.m_rightMotorCommand);
}

/**
* @brief  Reading the encoder readings from controller 
* @param  left and right ticks from the controller in long data type
*/
void RoboteqController::readCommands(readParameters &readParam)
{
     //	   ROS_INFO("ENTER");

    anscer_msgs::MotorDiagnosticsArray motorDiagnosticsArray;
    int batteryVoltage = 0;
    int batteryCurrent = 0;
    // ROS_INFO("ENTER 1");
    
    /*Reads the entire read register data*/
   
    /*Reads value from Roboteq*/
    //ROS_INFO("ENTER 2");

    int leftMotorRead   = m_roboteqDevice.GetValue(_ABCNTR, 2,m_leftEncoderReading);
    int rightMotorRead  = m_roboteqDevice.GetValue(_ABCNTR, 1,m_rightEncoderReading);
    // ROS_INFO("ENTER 3");

    ROS_INFO_STREAM("LEFT " <<leftMotorRead << " Right : " << rightMotorRead);  
    // int leftMotorSpeed  = m_roboteqDevice.GetValue(_ABSPEED, 2,m_leftSpeed);
    // int rightMotorSpeed = m_roboteqDevice.GetValue(_ABSPEED, 1,m_rightSpeed);

    //ROS_INFO_STREAM("LEFT " <<leftMotorSpeed << " Right : " << rightMotorSpeed);

    
    readParam.m_leftEncoder             = m_leftEncoderReading;
    readParam.m_rightEncoder            = m_rightEncoderReading;
    readParam.m_leftSpeed               = m_leftSpeed;
    readParam.m_rightSpeed              = m_rightSpeed;
    // ROS_INFO("ENTER 5");

    readMotorControllerFlags(motorDiagnosticsArray);

    readBatteryParameters(batteryVoltage,batteryCurrent);
    //
    // ROS_INFO("ENTER 7");

    readParam.m_roboteqDiagnosticsFeedback = motorDiagnosticsArray;
    //readParam.m_roboteqDiagnosticsFeedback = motorDiagnosticsArray;
    readParam.m_batteryVoltage             = batteryVoltage;
    readParam.m_batteryCurrent             = batteryCurrent;
    // ROS_INFO("Exit");

}


/**
* @brief  Reading the motor fault flags from controller 
* @param  setfaultFlags is the fault flag value and motorFaultFlags is the converted data in anscer_msgs::MotorFaultFlags datatype
*/
void RoboteqController::setMotorFaultFlags(int setfaultFlags, anscer_msgs::MotorFaultFlags &motorFaultFlags)
{
    std::bitset<16> setFaultBits(setfaultFlags);

    motorFaultFlags.overheat                = setFaultBits[0];
    motorFaultFlags.overvoltage             = setFaultBits[1];
    motorFaultFlags.undervoltage            = setFaultBits[2];
    motorFaultFlags.short_circuit           = setFaultBits[3];
    motorFaultFlags.emergency_stop          = setFaultBits[4];
    motorFaultFlags.motor_setup_fault       = setFaultBits[5];
    motorFaultFlags.mosfet_failure          = setFaultBits[6];
    motorFaultFlags.default_configuration   = setFaultBits[7];
}

/**
* @brief   Reading the motor status flags from controller 
* @param  setfaultFlags is the status flag value and motorStatusFlags is the converted data in anscer_msgs::MotorStatusFlags datatype
*/
void RoboteqController::setMotorStatusFlags(int setStatusFlags, anscer_msgs::MotorStatusFlags &motorStatusFlags)
{
    std::bitset<16> setStatusBits(setStatusFlags);

    motorStatusFlags.serial_mode                = setStatusBits[0];
    motorStatusFlags.pulse_mode                 = setStatusBits[1];
    motorStatusFlags.analog_mode                = setStatusBits[2];
    motorStatusFlags.power_stage_off            = setStatusBits[3];
    motorStatusFlags.stall_detected             = setStatusBits[4];
    motorStatusFlags.at_limit                   = setStatusBits[5];
    motorStatusFlags.unused                     = setStatusBits[6];
    motorStatusFlags.micro_basic_script_running = setStatusBits[7];
    motorStatusFlags.motor_tuning_mode          = setStatusBits[8];
}


/**
* @brief   Reading the motor fault status flags from controller 
* @param  motorDiagnosticsArray returns the motor diagnostics array
*/
void RoboteqController::readMotorControllerFlags(anscer_msgs::MotorDiagnosticsArray &motorDiagnosticsArray)
{
    int rightFfRetVal = m_roboteqDevice.GetValue(_FF, 1, m_motorFaultFlags);
    int rightFsRetVal = m_roboteqDevice.GetValue(_FS, 1, m_motorStatusFlags);

    anscer_msgs::MotorDiagnostics motorDiagnostics;

    motorDiagnostics.name = "motor_controller";
    setMotorFaultFlags(m_motorFaultFlags, motorDiagnostics.fault_flags);
    setMotorStatusFlags(m_motorStatusFlags, motorDiagnostics.status_flags);
    motorDiagnosticsArray.motor_diagnostics.push_back(motorDiagnostics);
}

/**
 * @brief Reading battery voltage from roboteq controller
 * @param readParameters struct
 */
void RoboteqController::readBatteryParameters(int &batteryVoltage,int &batteryCurrent)
{
	
    int voltage =m_roboteqDevice.GetValue(_VOLTS,2,batteryVoltage);
    int current =m_roboteqDevice.GetValue(_BATAMPS,batteryCurrent);

    batteryVoltage = batteryVoltage/10;
}
