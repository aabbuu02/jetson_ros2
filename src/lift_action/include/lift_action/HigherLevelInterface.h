/**
 * @file HigherLevelInterface.h
 * @author Jishnu (jishnu@anscer.com)
 * @brief communication node used for communicating between two  PLC s using modbus protocol
 *
 * @version 0.1
 * @date 2022-07-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */
/**
 * @brief include headerfiles
 */

#ifndef HIGHER_LEVEL_INTERFACE
#define HIGHER_LEVEL_INTERFACE

#include "Ui_Interface.h"

#include "ModbusCommunicator.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "lift_action/LiftAction.h"
#include "lift_action/LiftFeedback.h"
#include "lift_action/ErrorStatus.h"
#include <fstream>
#include<iostream>
#include <sstream>  // for string streams
#include <string>

class HigherLevelInterface
{
    public:
    HigherLevelInterface();
    ~HigherLevelInterface();

    int readLifterFeedbackData();
    int writeData(int m_writeRegistornumber,int m_writeValue);
    int writeArrayData(int m_writeRegstornumber,char m_arrayData[],int bytes);

    int failureFeedback(std::string uniqueId,int action);  //for sending confirmation to qr_mission_scheduler for box picking or dropping failure

    ModbusCommunicator *p_modbusCommunicator;
    float m_updateRate;
    lift_action::LiftFeedback m_lifterFeedbackData;
    std::string m_uniqueId;
    lift_action::ErrorStatus m_lifterErrorStatus;
    
    void robotFeedbackCallback(const lift_action::LiftAction &msg);
    void scannerDataCallback(const std_msgs::String &msg);
    void robotStateCallback(const std_msgs::Bool &msg);
    void robotLocationStatusCallback(const std_msgs::Bool &msg);
    void barcodeTriggerStatusCallback(const std_msgs::Bool &msg);
    void wmsStatusCallback(const std_msgs::Bool &msg);
    void bypassButtonCallback(const std_msgs::Bool &msg);
    int sendDataToExtractor();
     
    private:
    bool  checkParameters();
    int readParameters();
    bool m_parameterExistanceFlag;
    
    int m_numberOfReadRegisters;
    int m_writeRegistornumber;
    uint32_t m_writeValue;

    uint16_t* p_lifterFeedbackData;

    std::string m_subscriberTopic;
    std::string m_publisherTopic;
    std::string m_tasks;
    std::string m_tasksArray;
    std::string m_tasksFileName;
    std::string m_year;
    std::string m_date;
    std::string m_serialNo;
    
    std::string m_scannedData;
   // uint64_t m_scannedData;
    std::string m_rackNo;
    std::string m_boxNo;

    std::string m_prevUniqueID;
    
    std::string m_lastUniqueID ;
    


    int m_rack;
    int m_shelf;
    int m_acrShelf;
    int m_action;
    int m_bin;
    std::string m_core;
    int m_level;
    int m_prevAcrShelf;
    int m_lastAction;

    int m_triggerCount;
    int m_triggerThreshold;

    bool m_locationChangeFlag;
    bool m_agvStopFlag;
    bool m_reachedFlag;
    bool m_agvActionFlag;
    bool m_extractorStartFlag;
    bool m_lifterFeedbackFlag;
    bool m_failStatusFlag;
    bool m_barcodeTriggerStatusFlag;
    bool m_wmsUpdateFlag;
    bool m_operationFlag;
    uint16_t m_robotMovementFlag;
    int m_side;
    char m_binId[20];
    char m_putBinId[20];
    std_msgs::Bool robotInitialised;
    std::vector<uint16_t> m_taskValue;  
    
    ros::Subscriber robotFeedBack;
    ros::Subscriber scannerDataSubscriber;
    ros::Publisher  lifterFeedback;
    ros::Publisher robotInitialisedPublisher;
    ros::Publisher scannerTrigger;
    ros::Publisher brakeActivator;
    ros::Publisher errorStatus;
    ros::Publisher homingStatus;
    ros::Subscriber robotState;
    ros::Subscriber robotLocationChangeSubscriber;
    ros::Subscriber barcodeTriggerStatus;
    ros::Subscriber wmsUpdateStatus;
    ros::Subscriber bypassButton;



    UIInterface *p_UIInterface;

    

};


#endif
