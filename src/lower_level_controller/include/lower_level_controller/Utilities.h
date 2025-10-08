#ifndef UTILITIES_H
#define UTILITIES_H

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <chrono>
#include <fstream>
#include <mutex>
#include <thread>
#include "std_msgs/Int8.h"

#include "anscer_msgs/MotorDiagnostics.h"
#include "anscer_msgs/MotorDiagnosticsArray.h"


#include "lift_action/LiftAction.h"
#include "wms_data/Feedback.h"


constexpr int MODBUS_DEVICE         = 1;
constexpr int MODBUS_ROBOTEQ_DEVICE = 2;
constexpr int ODRIVE_DEVICE         = 3;
constexpr int RAINBOW_DEVICE        = 4;


struct readParameters
{ 
    int m_currentTimeStamp = 0;
    int m_leftMotorSpeed   = 0;
    int m_rightMotorSpeed  = 0;
    int m_leftDirection;
    int m_rightDirection;
    int32_t m_leftMotorEncoder  = 0;
    int32_t m_rightMotorEncoder = 0;
    int32_t m_botShelfNo = 0;
    double m_batteryVoltage,m_batteryPercentage;

  ///////////////////////////////////////////////////////////    



    bool m_emergencyFeedback;


};

struct writeParameters
{ 
    int m_updatedTimeStamp  = 0;
    int m_leftMotorCommand  = 0;
    int m_rightMotorCommand = 0;
    int m_leftDirection     = 0;
    int m_rightDirection    = 0;
    int m_heartBeat         = 0;

    lift_action::LiftAction m_lifterData;


      //PLC based mission initiation
      // int8_t m_taskRequestStatus = 0;

    int m_controllerMode;
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



