/**
 * @file UIInterface.h
 * @author Jishnu (jishnur@anscer.com)
 * @brief interfacing UI for the ACR
 * @version 0.1
 * @date 2022-10-25
 * @copyright Copyright (c) 2022
 */
#ifndef UI_INTERFACE
#define UI_INTERFACE

/**
 * @brief include header files
*/
#include "ModbusCommunicator.h"
#include "std_msgs/Bool.h"
#include "lift_action/UI_Interface.h"
#include "lift_action/Homing_status.h"
#include "lift_action/Home_sensor.h"
#include "ros/ros.h"

class UIInterface
{
    private:
        std::string m_packageName;
        ros::NodeHandle nh;
        ModbusCommunicator *p_modbusCommunicatorPtr;

    public:
        UIInterface(ModbusCommunicator *mcPtr);
        ~UIInterface();
        
        lift_action::UI_Interface m_liftMsg;

        /**
         * @breif callback functions declarations
        */
        void eStopCallback(const std_msgs::Bool &msg);
        void homingCallback(const lift_action::UI_Interface &msg);
        void cycleResetCallback(const std_msgs::Bool &msg);
	void homingButtonCallback(const std_msgs::Bool &msg);

        int homingControl(lift_action::UI_Interface homingMsg);

        ros::Subscriber eStopDataSubscriber;
        ros::Subscriber homingDataSubscriber;
        ros::Subscriber cycleResetDataSubscriber;
        ros::Subscriber homingButtonSubscriber;
        ros::Publisher  homingStatusPublisher;


        lift_action::UI_Interface m_homingMsg;
        lift_action::Homing_status m_homingStatus;
        
    

};    
#endif        
