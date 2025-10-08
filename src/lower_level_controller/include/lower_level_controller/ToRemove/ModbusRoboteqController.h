#ifndef MODBUS_ROBOTEQ_CONTROLLER_H
#define MODBUS_ROBOTEQ_CONTROLLER_H


#include "AHighLevelController.h"
#include "ModbusDevice.h"
#include "RoboteqDevice.h"

#include "anscer_msgs/MotorStatusFlags.h"
#include "anscer_msgs/MotorFaultFlags.h"
#include "anscer_msgs/MotorDiagnostics.h"
#include "anscer_msgs/MotorDiagnosticsArray.h"

class ModbusRoboteqController: public AHighLevelController
{

public:
    ModbusRoboteqController(ros::NodeHandle &nh);
    virtual ~ModbusRoboteqController(){}

    // void initialize(ros::NodeHandle &nh) override;
    void writeCommands(writeParameters &writeParams) override;
    void readCommands(readParameters &readParam) override;


private:


    ros::NodeHandle nhp;

    /*Member Functions*/
    void initialize();
    void initializeParameters();
    void setMotorFaultFlags(int setfaultFlags, anscer_msgs::MotorFaultFlags &motorFaultFlags);
    void setMotorStatusFlags(int setStatusFlags, anscer_msgs::MotorStatusFlags &motorStatusFlags);
    void readMotorControllerFlags(anscer_msgs::MotorDiagnosticsArray motorDiagnosticsArray);
    // void readRegister();
    // void writeRegister(int reg_add, int value);
    // void writeRegisters(int addr, int nb, uint16_t data[]);
    // uint32_t readEncoder(int encoder_num);


    /*Member variables*/


    std::string m_roboteqPort;
    
    int m_modbusPort;
    int m_motorFaultFlags = 0;
    int m_motorStatusFlags = 0;

    // int m_leftMotorFeedbackPin,m_rightMotorFeedbackPin;
    // int m_lifterFeedbackPin, m_emergencyFeedbackPin, m_confirmationPin;
    // int m_frontProtectiveFieldPin, m_backProtectiveFieldPin, m_motorErrorPin;



    int m_leftMotorFeedbackPin,m_rightMotorFeedbackPin;
    int m_leftSpeedPin,m_rightSpeedPin;
    int m_emergencyFeedbackPin,m_lifterFeedbackPin,m_confirmationFeedbackPin;
    int m_frontLidarFeedbackPin,m_rearLidarFeedbackPin;
    int m_modeSelectionFeedbackPin;
    int m_motorErrorFeedbackPin;


    int m_frontLidarCommandPin, m_rearLidarCommandPin;
    int m_leftBrakeCommandPin,m_rightBrakeCommandPin;
    int m_lifterCommandPin,m_relayCommandPin,m_towerLightCommandPin;

    int m_lifterCommand;

    int m_rightEncoderReading,m_leftEncoderReading;
    int m_leftSpeed,m_rightSpeed;
    // int m_leftCommandPin,m_rightCommandPin,m_leftDirectionCommandPin,m_rightDirectionCommandPin,m_relayStatePin;    


    std::string m_ipAddress;

    uint16_t writeRegData[WRITEREGISTERNUMBER] = {0,0}; //,0,0,0,0,0,0,0,0,0,0,0};
    // modbus_t *plc;

    ModbusDevice m_modbusDevice;
    RoboteqDevice m_roboteqDevice;
};

#endif
