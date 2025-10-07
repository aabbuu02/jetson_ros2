#ifndef ROBOTEQCONTROLLER_H
#define ROBOTEQCONTROLLER_H

#include <bitset>
#include "AHighLevelController.h"

#include "Constants.h"
#include "ErrorCodes.h"
#include "RoboteqDevice.h"
#include "anscer_msgs/MotorDiagnostics.h"
#include "anscer_msgs/MotorDiagnosticsArray.h"


using namespace Controller;

class RoboteqController : public AHighLevelController
{

public:
    RoboteqController(ros::NodeHandle &nh);
    ~RoboteqController();

    void writeMotorControl(writeParameters &writeParams) override;

    void readEncoderValues(readParameters &readParam) override;

    void robotTimerCallback(const ros::TimerEvent &time);

    anscer_msgs::MotorDiagnosticsArray getMotorDiagnosticsArray();

private:
    /*Member functions*/
    void initialize(ros::NodeHandle &nh) override;
    void readMotorControllerFlags();
    void setMotorFaultFlags(int setfaultFlags, anscer_msgs::MotorFaultFlags &motorFaultFlags);
    void setMotorStatusFlags(int setStatusFlags, anscer_msgs::MotorStatusFlags &motorStatusFlags);

    ros::Timer robotTimer;
    bool m_connected;
    ros::NodeHandle nh;
    /*Messages*/
    anscer_msgs::MotorDiagnostics m_leftDiag, m_rightDiag;
    anscer_msgs::MotorDiagnosticsArray m_motorDiagArr;

    /*Member variables*/
    int m_rightEncoderReading,m_leftEncoderReading;
    int m_leftSpeed,m_rightSpeed;
    int m_leftFaultFlags, m_leftStatusFlags;
    int m_rightFaultFlags, m_rightStatusFlags;

    RoboteqDevice device;
};

#endif
