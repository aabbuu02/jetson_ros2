#ifndef LOWER_LEVEL_COMPUTATIONS_H
#define LOWER_LEVEL_COMPUTATIONS_H

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "anscer_msgs/PGVPose.h"
// #include "anscer_msgs/Debug.h"

#include "Utilities.h"

#define LEFT 0
#define RIGHT 1

#define CW 0
#define CCW 1

using namespace std;
/**
   * @class LowerLevelComputations
   * @brief Doing all the lower level computations and interact with the motor driver
   */

class LowerLevelComputations
{
public:

    LowerLevelComputations();
    ~LowerLevelComputations();

    void initializeComputation(const geometry_msgs::Twist &cmdVel, writeParameters &writeParam);
    void updateOdometry(uint32_t deltaTime,const readParameters &readParam,nav_msgs::Odometry &odometryData);

private:

    ros::NodeHandle nh;
    ros::NodeHandle nhp;
    ros::Subscriber barcodePoseSub;
    ros::Publisher debugCmdVelPub;

    /*Member functions*/

    void initializeParameters();
    void velocityLimiter(double &linearVel, double &angularVel);
    void motorDriveCommands(writeParameters &writeParam);
    void processEncoderValues(const uint32_t &leftEncoder, const uint32_t &rightEncoder);
    void barcodeCallback(const anscer_msgs::PGVPose &msg);
    double ticksToMeters(long ticks);

    /*Member variables*/

    int32_t m_lastReading[2] = {0,0};
    int32_t m_lastDiffReading[2] ={0,0};
    double m_wheelSeperation, m_wheelRadius,m_gearRatio ,m_ticksPerRevolutionBaseMotor;   //m_wheelRadius,m_gearRatio ,m_ticksPerRevolutionBaseMotor;  ** new update 
    double m_linearVelocityLimit, m_angularVelocityLimit;
    double m_linearVelocity, m_angularVelocity;
    double m_odomPose[3] = {0.0, 0.0, 0.0};
    double m_odomVel[3] = {0.0, 0.0, 0.0};
    double m_deltaLeftWheel,m_deltaRightWheel;
    double m_meterPerTick; // constant to convert tick to meters ( takes wheel radius , gear ratio, ticks per revolution)  ** new update 
    double m_TPR;
    bool m_initEncoder[2] = {false, false};
    bool m_barCodeDataReceived = false;
    std::string m_odomFrame, m_baseFrame,m_barcodeGlobalTopic,m_debugCmdVelTopic;
    anscer_msgs::PGVPose m_pgvGlobalPose;
};
#endif
