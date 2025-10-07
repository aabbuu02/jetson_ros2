#ifndef MODBUS_CONTROLLER_H
#define MODBUS_CONTROLLER_H


#include "AHighLevelController.h"
#include "ModbusDevice.h"
#include "std_msgs/Int8.h"
#include <std_msgs/Float32.h>
#include "std_msgs/Bool.h"
#include "anscer_msgs/Feedback.h"
#include "anscer_msgs/PGVPose.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Int32.h"

#define ACR_DROPCOMMAND 0
#define ACR_PICKCOMMAND 1

#define LEFT_ORIENTATION 1
#define ACR_ORIENTATION 2
#define RIGHT_ORIENTATION 3

#define ROBOT_POWEROFF 2

class ModbusController: public AHighLevelController
{

public:
    ModbusController(ros::NodeHandle &nh);
    virtual ~ModbusController(){}

    void writeCommands(writeParameters &writeParams) override;
    void readCommands(readParameters &readParam) override;
    int32_t readEncoder(int encoder_num);
    void writeMotorControl(writeParameters &writeParams);
    void readEncoderValues(readParameters &readParam);
    

private:

    ros::NodeHandle nhp;
    ros::Publisher lifterFeedbackPub;
    // ros::Subscriber feedbackSub,statusSub,eStopSub;
    ros::ServiceClient powerSrvc;
    ros::Subscriber distSub,chargeSub;
    /*Member Functions*/

    void initialize();
    void initializeParameters();
    uint32_t convertDoubleInt(uint16_t low, uint16_t high);
    void distGoalCb(const std_msgs::Float32 &distMsg);
    void chargeCallback(const std_msgs::Int32 &msg);
    /*Member variables*/

    // int m_missionStatus,m_missionStatusPin;
    // int m_tagId,m_tagIdPin;

    // bool m_eStopUi;
    // int m_lidar = 0;
    // int m_powerPrev = 0;
    int m_modbusPort;

    int m_bufferFeedback;
    

    int m_powerOffPin, m_brakeCmdPin; 
    int m_leftMotorSpeedPin,m_rightMotorSpeedPin;
    int m_leftMotorEncoderPin,m_rightMotorEncoderPin;
    int m_leftMotorCommandPin,m_rightMotorCommandPin;
    int m_pickLocationCommandPin,m_pickOrientationCommandPin,m_dropLocationCommandPin,m_dropOrientationCommandPin;

    int m_batteryVoltagePin,m_batteryPercentagePin;
    int m_timeStampCurrentReg, m_timeStampUpdateReg;
    int m_emergencyFeedbackPin;
    int m_missionFeedbackPin,m_missionConfirmationCommandPin,m_missionTaskPin,m_homingDistancePin;
    int m_bufferFeedbackPin;
    int readCounter =0;


    int m_botShelfPin;
    bool m_charge = false;
    int m_chargeNumber = 0;
    int m_chargePin = 0;
    // int m_ledCommandPin,m_buzzerCommandPin;
    // int m_controllerModePin;
    // int m_frontLidarPin,m_rearLidarPin;
    // int m_dockStateFeedbackPin;
    // int m_leftSpeedPin,m_rightSpeedPin;
    // int m_emergencyFeedbackPin,m_confirmationFeedbackPin;
    // // int m_frontLidarFeedbackPin,m_rearLidarFeedbackPin;
    // int m_modeSelectionFeedbackPin;
    // int m_motorErrorFeedbackPin;
    // int m_batteryVoltageFeedbackPin, m_leftMotorCurrentFeedbackPin, m_rightMotorCurrentFeedbackPin;
    // int m_relayCommandPin;
    // int m_ledPin,m_buzzerPin, m_heartBeatPin;
    // int m_frontLidarCommandPin, m_rearLidarCommandPin;
    // int m_leftBrakeCommandPin,m_rightBrakeCommandPin;
    // int m_lifterCommandPin,m_relayCommandPin,m_towerLightCommandPin;
    // int m_lifterCommand;
    // int m_leftCommandPin,m_rightCommandPin,m_leftDirectionCommandPin,m_rightDirectionCommandPin;
    //int m_relayStatePin;    
    // int m_leftSpeed,m_rightSpeed;
    int32_t m_prevLeftEncoder  = 0;
    int32_t m_prevRightEncoder = 0;

    bool m_powerOffInitiated    = false;
    bool m_isBinRetracted       = true;

    bool m_missionUpdated  = false;
    //int m_missionUpdated = 1;
    bool m_missionCompleted  = false;
    bool m_topModuleInitiated=false;
    wms_data::Feedback feedbackMsg;
    uint16_t m_botShelfNo = 0;

    uint16_t m_homingDistance = 0;

    std::string m_lastId = "";
    std::string m_ipAddress;



    uint16_t writeRegData[WRITEREGISTERNUMBER] = {0};
    ModbusDevice m_modbusDevice;
    

};

#endif
