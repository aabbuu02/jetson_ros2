#ifndef RAINBOW_CONTROLLER_H
#define RAINBOW_CONTROLLER_H


#include "AHighLevelController.h"
#include "RainbowDevice.h"





class RainbowController: public AHighLevelController
{

public:
    RainbowController(ros::NodeHandle &nh);
    virtual ~RainbowController(){}

    void writeCommands(writeParameters &writeParams) override;
    void readCommands(readParameters &readParam) override;


private:


    ros::NodeHandle nhp;

    /*Member Functions*/

    void initialize();
    void initializeParameters();
    // void connectToDevice();
    // void watchDogTimer();
    // // void readRegister();
    // // void writeRegister(int reg_add, int value);
    // // void writeRegisters(int addr, int nb, uint16_t data[]);
    // // uint32_t readEncoder(int encoder_num);


    /*Member variables*/


    // int m_modbusPort;

    // int m_leftMotorFeedbackPin,m_rightMotorFeedbackPin;
    // int m_lifterFeedbackPin, m_emergencyFeedbackPin, m_confirmationPin;
    // int m_frontProtectiveFieldPin, m_backProtectiveFieldPin, m_motorErrorPin;



    // int m_leftMotorFeedbackPin,m_rightMotorFeedbackPin;
    // int m_leftSpeedPin,m_rightSpeedPin;
    // int m_emergencyFeedbackPin,m_lifterFeedbackPin,m_confirmationFeedbackPin;
    // int m_frontLidarFeedbackPin,m_rearLidarFeedbackPin;
    // int m_modeSelectionFeedbackPin;
    // int m_motorErrorFeedbackPin;


    // int m_frontLidarCommandPin, m_rearLidarCommandPin;
    // int m_leftBrakeCommandPin,m_rightBrakeCommandPin;
    // int m_lifterCommandPin,m_relayCommandPin,m_towerLightCommandPin;

    // int m_lifterCommand;
    // // int m_leftCommandPin,m_rightCommandPin,m_leftDirectionCommandPin,m_rightDirectionCommandPin,m_relayStatePin;    

    int m_baudRate;

    std::string m_leftMotor{"/dev/lft_motor"};
    std::string m_rightMotor{"/dev/rgt_motor"};

    // uint16_t writeRegData[WRITEREGISTERNUMBER] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
    // // modbus_t *plc;

    RainbowDevice m_rainbowDevice;

    // CommunicationSerial* p_communicationSerial;
    // std::vector<std::shared_ptr<ODriveDevice> >p_ODriveVector;
};

#endif
