#ifndef UTILITIES_H
#define UTILITIES_H

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <chrono>
#include <fstream>
#include <mutex>
#include <thread>

#include "anscer_msgs/MotorDiagnostics.h"
#include "anscer_msgs/MotorDiagnosticsArray.h"



constexpr int ROBOTEQ_DEVICE = 1;


struct readParameters
{
    int m_leftEncoder;
    int m_rightEncoder;
    int m_leftSpeed;
    int m_rightSpeed;


    int m_leftDirection;
    int m_rightDirection;
    int m_lifterFeedback;
    
    int m_batteryVoltage;
    int m_batteryCurrent;

    bool m_emergencyFeedback;
    bool m_confirmationFeedback;
    bool m_frontLidarFeedback;
    bool m_rearLidarFeedback;
    bool m_motorErrorFeedback; //Not sure whether this is useful
    bool m_modeSelectionFeedback;

    anscer_msgs::MotorDiagnosticsArray m_roboteqDiagnosticsFeedback;

};

struct writeParameters
{
    int m_leftMotorCommand =0;
    int m_rightMotorCommand =0;
    int m_towerLightMode=0;
    int m_leftDirection =0;
    int m_rightDirection =0;
    int m_frontLidarField = 0;
    int m_rearLidarField = 0;
    int m_relayState = 0;
    int m_brakeState = 0;
};



namespace Utilities
{
    /**
    * @brief  Computes time in millis
    * @returns time in millis based on system clock
    */
    inline int64_t millis()
    {
        int64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        return ms;
    }
}

#endif



