#ifndef MODBUS_ROBOTEQ_CONTROLLER_H
#define MODBUS_ROBOTEQ_CONTROLLER_H


#include "AHighLevelController.h"
#include "RoboteqDevice.h"

#include "anscer_msgs/MotorStatusFlags.h"
#include "anscer_msgs/MotorFaultFlags.h"
#include "anscer_msgs/MotorDiagnostics.h"
#include "anscer_msgs/MotorDiagnosticsArray.h"

class RoboteqController: public AHighLevelController
{

public:
    RoboteqController(ros::NodeHandle &nh);
    virtual ~RoboteqController(){}

    ros::Publisher motorPub;

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
    void readMotorControllerFlags(anscer_msgs::MotorDiagnosticsArray &motorDiagnosticsArray);
    void readBatteryParameters(int &batteryVoltage,int &batteryCurrent);
    // void readRegister();
    // void writeRegister(int reg_add, int value);
    // void writeRegisters(int addr, int nb, uint16_t data[]);
    // uint32_t readEncoder(int encoder_num);


    /*Member variables*/


    std::string m_roboteqPort;
    
    int m_modbusPort;
    int m_motorFaultFlags = 0;
    int m_motorStatusFlags = 0;




    int m_rightEncoderReading,m_leftEncoderReading;
    int m_leftSpeed,m_rightSpeed;



    RoboteqDevice m_roboteqDevice;
};

#endif
