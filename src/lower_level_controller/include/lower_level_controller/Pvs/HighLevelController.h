#ifndef HIGHLEVELCONTROLLER_H
#define HIGHLEVELCONTROLLER_H

#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <modbus/modbus.h>
#include <chrono>
#include <std_msgs/ByteMultiArray.h>
#include "AHighLevelController.h"
#include "Utilities.h"
#include <std_msgs/Int32.h>

#include <actionlib/server/simple_action_server.h>
#include "anscer_msgs/LiftAction.h"
#include "anscer_msgs/LiftFeedback.h"
#include "anscer_msgs/LiftResult.h"

#include "anscer_msgs/DynamicFootprint.h"
#include "anscer_msgs/Safety.h"

#define READREGISTERNUMBER 20
#define STARTINGINDEX 100 /*Offset register index from which writing starts*/
#define WRITEREGISTERNUMBER 20


using namespace Controller;

class HighLevelController: public AHighLevelController
{

public:
    HighLevelController(ros::NodeHandle &nh,std::string name);
    ~HighLevelController();

    void writeMotorControl(writeParameters &writeParams) override;
    void readEncoderValues(readParameters &readParam) override;

	typedef actionlib::SimpleActionServer<anscer_msgs::LiftAction> liftServer;

	std::string action_name;

private:

    /**
    * @brief NodeHandle ROS
    */
    ros::NodeHandle nhp;

    ros::Timer heartBeatTimer;
    liftServer m_liftServer;
    ros::ServiceClient footprintClient;
    ros::ServiceClient safetyClient;
    ros::Publisher lidarFeedback;
    ros::Publisher modePublisher;

    /*Member functions*/
    void liftCallback(const anscer_msgs::LiftGoalConstPtr &goal);
    void heartBeatCallback(const ros::TimerEvent &e);
    void initialize(ros::NodeHandle &nh);
    void readRegister();
    void writeRegister(int reg_add, int value);
    void writeRegisters(int addr, int nb, uint16_t data[]);
    uint32_t readEncoder(int encoder_num);
    uint64_t millis();

    /*Member variables*/

    enum LifterStates
    {
        RETRACTED = 0,
        EXTENDED = 1
    };


    int m_leftMotorFeedbackPin,m_rightMotorFeedbackPin;
    int m_lifterFeedbackPin, m_emergencyFeedbackPin, m_confirmationPin;
    int m_frontProtectiveFieldPin, m_backProtectiveFieldPin, m_motorErrorPin;
    int m_frontLidarFieldPin, m_rearLidarFieldPin;
    int m_leftCommandPin,m_rightCommandPin,m_leftDirectionCommandPin,m_rightDirectionCommandPin,m_relayStatePin;
    int m_modeControlPin,m_lifterCommandPin, m_towerLightPin;
    int m_leftSpeedPin, m_rightSpeedPin;
    int m_leftBrakeCommandPin,m_rightBrakeCommandPin;
    int m_modeSelected = 0;
    int m_modeSelectionPin;

    int m_lifterFeedback = 0;
    int m_previousLifterFeedback = RETRACTED;
    int m_lifterCommand  = 0;
    int m_lifterLowerLimit;
    int m_lifterUpperLimit;
    int m_port;

    int8_t m_lifterState = 0; //Retracted
    int8_t m_frontLidarField = 0;
    int8_t m_rearLidarField = 0;


    uint64_t m_currentTime,m_lastTime,m_deltaTime;
    unsigned short int m_regData[READREGISTERNUMBER];
    uint16_t writeRegData[WRITEREGISTERNUMBER] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
    int m_heartBeatRate = 1;

    bool m_heartBeatStatus = false;
    std::string m_ipAddress;
    modbus_t *plc;
};

#endif
