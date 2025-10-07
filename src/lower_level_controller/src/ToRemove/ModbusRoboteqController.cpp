#include <ros/console.h>
#include "ModbusRoboteqController.h"
/**
* @brief  Constructor for the HighLevelController
*/

ModbusRoboteqController::ModbusRoboteqController(ros::NodeHandle &nh):nhp("~")
{
    ROS_INFO("ModbusRoboteqController Constructor called ");
    initialize();
}



/**
* @brief  Initializes the PLC controller
*/
void ModbusRoboteqController::initialize()
{
    initializeParameters();

    /*Connects to the MODBUS device and Roboteq Driver*/
    m_modbusDevice.connectToDevice(m_ipAddress, m_modbusPort);
    m_roboteqDevice.connectToDevice(m_roboteqPort);
}


/**
* @brief  Initializes parameters from the parameter server
*/
void ModbusRoboteqController::initializeParameters()
{
    /*Configurations for initializing modbus communication*/
    nhp.param<std::string>("ip_address", m_ipAddress, "192.168.1.105");
    nhp.param<std::string>("roboteq_port", m_roboteqPort, "/dev/roboteq_driver");

    nhp.param("modbus_port", m_modbusPort, 502);

    nhp.param("modbus_read/left_feedback", m_leftMotorFeedbackPin, 0);
    nhp.param("modbus_read/right_feedback", m_rightMotorFeedbackPin, 2);
    nhp.param("modbus_read/mode_selection", m_modeSelectionFeedbackPin, 11);
    nhp.param("modbus_read/left_speed", m_leftSpeedPin, 13);
    nhp.param("modbus_read/right_speed", m_rightSpeedPin, 14);
    nhp.param("modbus_read/confirmation_button", m_confirmationFeedbackPin, 14);
    nhp.param("modbus_read/lift_feedback", m_lifterFeedbackPin, 15);
    nhp.param("modbus_read/front_protective_field", m_frontLidarFeedbackPin, 16);
    nhp.param("modbus_read/emergency_feedback", m_emergencyFeedbackPin, 17);
    nhp.param("modbus_read/back_protective_field", m_rearLidarFeedbackPin, 19);

    // nh.getParam("modbus_write/left_command", m_leftCommandPin);
    // nh.getParam("modbus_write/right_command", m_rightCommandPin);
    // nh.getParam("modbus_write/left_direction", m_leftDirectionCommandPin);
    // nh.getParam("modbus_write/right_direction", m_rightDirectionCommandPin);
    nhp.param("modbus_write/left_brake", m_leftBrakeCommandPin, 27);
    nhp.param("modbus_write/right_brake", m_rightBrakeCommandPin, 27);
    nhp.param("modbus_write/lift_command", m_lifterCommandPin, 30);
    nhp.param("modbus_write/front_lidar_field", m_frontLidarCommandPin, 31);
    nhp.param("modbus_write/back_lidar_field", m_rearLidarCommandPin, 32);
    nhp.param("modbus_write/relay_state", m_relayCommandPin, 21);
    nhp.param("modbus_write/tower_light", m_towerLightCommandPin, 33);
}


/**
* @brief  Controlling the motor controller through motor commands
* @param  left and right RPS/CPS in double datatype
*/
void ModbusRoboteqController::writeCommands(writeParameters &writeParams)
{
    int leftMotorCommand    = m_roboteqDevice.SetCommand(_MOTVEL, 1, writeParams.m_leftMotorCommand);
    int rightMotorCommand   = m_roboteqDevice.SetCommand(_MOTVEL, 2, writeParams.m_rightMotorCommand);
}

/**
* @brief  Reading the encoder readings from controller 
* @param  left and right ticks from the controller in long data type
*/
void ModbusRoboteqController::readCommands(readParameters &readParam)
{
    anscer_msgs::MotorDiagnosticsArray motorDiagnosticsArray;
    /*Reads the entire read register data*/
    m_modbusDevice.readRegister();
   
    /*Reads value from Roboteq*/

    int leftMotorRead   = m_roboteqDevice.GetValue(_ABCNTR, 2,m_leftEncoderReading);
    int rightMotorRead  = m_roboteqDevice.GetValue(_ABCNTR, 1,m_rightEncoderReading);

    int leftMotorSpeed  = m_roboteqDevice.GetValue(_ABSPEED, 2,m_leftSpeed);
    int rightMotorSpeed = m_roboteqDevice.GetValue(_ABSPEED, 1,m_rightSpeed);

    readParam.m_leftMotorEncoder             = m_leftEncoderReading;
    readParam.m_rightMotorEncoder            = m_rightEncoderReading;
    readParam.m_leftMotorSpeed               = m_leftSpeed;
    readParam.m_rightMotorSpeed              = m_rightSpeed;

    readParam.m_emergencyFeedback       = m_modbusDevice.m_registerData[m_emergencyFeedbackPin];
    readParam.m_confirmationFeedback    = m_modbusDevice.m_registerData[m_confirmationFeedbackPin];
    readParam.m_frontLidarFeedback      = m_modbusDevice.m_registerData[m_frontLidarFeedbackPin];
    readParam.m_rearLidarFeedback       = m_modbusDevice.m_registerData[m_rearLidarFeedbackPin];
    readParam.m_motorErrorFeedback      = m_modbusDevice.m_registerData[m_motorErrorFeedbackPin];
    readParam.m_modeSelectionFeedback   = m_modbusDevice.m_registerData[m_modeSelectionFeedbackPin];

    readMotorControllerFlags(motorDiagnosticsArray);
    readParam.m_roboteqDiagnosticsFeedback = motorDiagnosticsArray;
}


/**
* @brief  Reading the motor fault flags from controller 
* @param  setfaultFlags is the fault flag value and motorFaultFlags is the converted data in anscer_msgs::MotorFaultFlags datatype
*/
void ModbusRoboteqController::setMotorFaultFlags(int setfaultFlags, anscer_msgs::MotorFaultFlags &motorFaultFlags)
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
void ModbusRoboteqController::setMotorStatusFlags(int setStatusFlags, anscer_msgs::MotorStatusFlags &motorStatusFlags)
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
void ModbusRoboteqController::readMotorControllerFlags(anscer_msgs::MotorDiagnosticsArray motorDiagnosticsArray)
{
    int rightFfRetVal = m_roboteqDevice.GetValue(_FF, 1, m_motorFaultFlags);
    int rightFsRetVal = m_roboteqDevice.GetValue(_FS, 1, m_motorStatusFlags);

    anscer_msgs::MotorDiagnostics motorDiagnostics;

    motorDiagnostics.name = "motor_controller";
    setMotorFaultFlags(m_motorFaultFlags, motorDiagnostics.fault_flags);
    setMotorStatusFlags(m_motorStatusFlags, motorDiagnostics.status_flags);
    motorDiagnosticsArray.motor_diagnostics.push_back(motorDiagnostics);
}

