#ifndef LOWER_LEVEL_COMPUTATIONS_H
#define LOWER_LEVEL_COMPUTATIONS_H

#include <tf/tf.h>
#include <ros/package.h>
#include <unordered_map>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "anscer_msgs/PGVPose.h"

#include "Utilities.h"
#define MILLI_SECONDS 0.001

#define LEFT 0
#define RIGHT 1

#define CW 0
#define CCW 1

using namespace std;



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
    void barcodeCallback(const anscer_msgs::PGVPose &msg);

    void velocityLimiter(double &linearVel, double &angularVel);
    void motorDriveCommands(writeParameters &writeParam);
    void processEncoderValues(const int32_t &leftEncoder, const int32_t &rightEncoder);
    double ticksToMeters(long ticks);

    /*Member variables*/

    int m_leftDirection = 0;
    int m_rightDirection= 0;
    int m_tagId = -99;
    int32_t m_lastReading[2] = {0,0};
    int32_t m_lastDiffReading[2] ={0,0};
    int64_t m_currentTime, m_lastTime, m_deltaTime;

    double m_lastLinearVelocity = 0.0;
    double m_linearAccLimit__m_s2,m_linearDeccLimit__m_s2;
    double m_wheelSeperation, m_wheelRadius,m_gearRatio ,m_ticksPerRevolutionBaseMotor;   //m_wheelRadius,m_gearRatio ,m_ticksPerRevolutionBaseMotor;  ** new update 
    double m_linearVelocityLimit, m_angularVelocityLimit;
    double m_linearVelocity, m_angularVelocity;
    double m_currentTickLeft, m_currentTickRight;
    double m_odomPose[3] = {0.0, 0.0, 0.0};

    double m_odomVel[3] = {0.0, 0.0, 0.0};
    double m_deltaLeftWheel,m_deltaRightWheel;
    double m_meterPerTick; // constant to convert tick to meters ( takes wheel radius , gear ratio, ticks per revolution)  ** new update 
    double m_TPR;
    bool m_initEncoder[2] = {false, false};
    bool m_barCodeDataReceived = false;
    std::string m_odomFrame, m_baseFrame,m_debugCmdVelTopic;
    std::string m_barcodeTopic;
    anscer_msgs::PGVPose m_pgvGlobalPose;
    tf::TransformListener m_listener;
};
#endif
