/**
*@brief header file for MotorControl
*/
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "LowerLevelComputations.h"
#include "RoboteqDevice.h"


#define leftMotor 1
#define rightMotor 2

/**
*@brief class LowerLevelController
*@brief Lower level controller is for interfacing motordriver
*/
class MotorControl
{
public:
    /**
    *@brief constructor for LowerLevelController
    */
    MotorControl();
    /**
    *@brief destructor for LowerLevelController
    */
    ~MotorControl();

    int writeToMotor(float leftRpm,float rightRpm);

    LowerLevelComputations LowerLevelComputation;
    RoboteqDevice device;
    int initialiseConnection();

private:
    int m_motorReturnCommand;
    int m_status;
};
#endif
