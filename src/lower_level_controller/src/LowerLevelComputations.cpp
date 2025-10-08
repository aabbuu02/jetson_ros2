
#include "LowerLevelComputations.h"


/**
* @brief  Constructor for the LowerLevelComputations
*/
LowerLevelComputations::LowerLevelComputations():nhp("~")
{
    initializeParameters();
    debugCmdVelPub    = nh.advertise <geometry_msgs::Twist> (m_debugCmdVelTopic.c_str(), 1);
    barcodePoseSub    = nh.subscribe(m_barcodeTopic.c_str(), 1, & LowerLevelComputations::barcodeCallback, this);

    ROS_INFO("LowerLevelComputations Constructor called");
}


/**
* @brief  Destructor for the LowerLevelController
*/
LowerLevelComputations::~LowerLevelComputations()
{

}

/**
 * @brief  Reads parameters from parameter server
 */
void LowerLevelComputations::initializeParameters()
{
    nhp.param("robot/wheel_seperation", m_wheelSeperation, 0.39);
    nhp.param("robot/wheel_radius", m_wheelRadius, 0.10);
    nhp.param("robot/TPR", m_TPR, 16384.0);
    nhp.param("robot/linear_vel_limit", m_linearVelocityLimit, 2.0);
    nhp.param("robot/angular_vel_limit", m_angularVelocityLimit, 1.0);
    nhp.param("robot/gearRatio", m_gearRatio, 9.0);

    nhp.param<double>("robot/linear_acc_limit", m_linearAccLimit__m_s2, 0.1);
    nhp.param<double>("robot/linear_decc_limit", m_linearDeccLimit__m_s2, 0.1);

    nhp.param<std::string>("odom_frame", m_odomFrame, "odom_frame");
    nhp.param<std::string>("base_frame", m_baseFrame, "base_footprint");
    nhp.param<std::string>("barcode_topic", m_barcodeTopic, "barcode/global");
    nhp.param<std::string>("debug_cmd_topic", m_debugCmdVelTopic, "debug/cmd_vel");

    m_lastTime = m_currentTime = Utilities::millis();
}

/**
* @brief  Subscribes the Barcode data from PGV for the odometry correction
*/
void LowerLevelComputations::barcodeCallback(const anscer_msgs::PGVPose &msg)
{
    m_pgvGlobalPose       = msg;
    m_barCodeDataReceived = true;
}




/**
* @brief  Initializes the lower level computation by converting cmdVel to differential kinematics
* @param command velocity (geometry_msgs::Twist) ,left and right counts per second (double)
*/
void LowerLevelComputations::initializeComputation(const geometry_msgs::Twist &cmdVel, writeParameters &writeParam)
{
    m_meterPerTick = (2 * M_PI * m_wheelRadius) / (m_TPR * m_gearRatio); // computing the conversion factor of ticks to meter (from wheel radius , gearratio and no of ticks per revolution in  base motor)

    m_linearVelocity  = cmdVel.linear.x;
    m_angularVelocity = cmdVel.angular.z;

    velocityLimiter(m_linearVelocity, m_angularVelocity);

    if ((fabs(cmdVel.linear.x) <= 0.001 && fabs(cmdVel.angular.z) <= 0.001))
    {
        m_linearVelocity  = 0.0;
        m_angularVelocity = 0.0;
    }

    geometry_msgs::Twist cmdMsg;
    cmdMsg.linear.x  = m_linearVelocity;
    cmdMsg.angular.z = m_angularVelocity;
    debugCmdVelPub.publish(cmdMsg);
    motorDriveCommands(writeParam);
}


/**
* @brief  Ensuring the robot velocity stay within the bounds
* @param command linear and angular velocity (double)
*/
void LowerLevelComputations::velocityLimiter(double &linearVelocity, double &angularVelocity)
{

    m_currentTime = Utilities::millis();
    m_deltaTime   = m_currentTime - m_lastTime;

    /*Linear Velocity Smoothening*/
    /**
    * @brief Limiting velocity based on v = u +/- a*t
    */

    if (linearVelocity > (m_lastLinearVelocity + (m_linearAccLimit__m_s2 * m_deltaTime * MILLI_SECONDS)))
    {
        linearVelocity = (m_lastLinearVelocity + (m_linearAccLimit__m_s2 * m_deltaTime * MILLI_SECONDS));
    }
    else if (linearVelocity < (m_lastLinearVelocity - (m_linearDeccLimit__m_s2 * m_deltaTime * MILLI_SECONDS))) //m_lastLinearVel
    {
        linearVelocity = (m_lastLinearVelocity - (m_linearDeccLimit__m_s2 * m_deltaTime * MILLI_SECONDS));
    }
    else
    {
        linearVelocity = linearVelocity;
    }


    /*Clamping Linear Velocity*/

    if (linearVelocity > m_linearVelocityLimit)
    {
        linearVelocity = m_linearVelocityLimit;
    }
    else if (linearVelocity < -m_linearVelocityLimit)
    {
        linearVelocity = -m_linearVelocityLimit;
    }



    /*Clamping Angular Velocity */
   /* 
    if(abs(angularVelocity)<0.1 && abs(angularVelocity) > 0.01)
    {
          if(angularVelocity < 0.1 && angularVelocity > 0.0)
		 angularVelocity = 0.1;
	  else
		 angularVelocity = -0.1;
    }
    */

    if (angularVelocity > m_angularVelocityLimit)
    {
        angularVelocity = m_angularVelocityLimit;
    }
    else if (angularVelocity < -m_angularVelocityLimit)
    {
        angularVelocity = -m_angularVelocityLimit;
    }

    m_lastLinearVelocity = linearVelocity;
    m_lastTime = m_currentTime;

}


/**
* @brief  Compute the motor driver commands in counts per second
* @param left and right counts per second (double)
*/
void LowerLevelComputations::motorDriveCommands(writeParameters &writeParam)
{
	double leftSpeed  = m_linearVelocity + m_angularVelocity * m_wheelSeperation / 2.0;
    double rightSpeed = m_linearVelocity - m_angularVelocity * m_wheelSeperation / 2.0;

    double leftRPM    = 60 / (2 * M_PI * m_wheelRadius) * leftSpeed;
    double rightRPM   = 60 / (2 * M_PI * m_wheelRadius) * rightSpeed;



    /*Since the left-right motors need to rotate CW and CCW to make robot move straight/back*/

    // double leftRPS    = (leftRPM) / 60.0;
    // double rightRPS   = (rightRPM) / 60.0;

    /*IMPORTANT : For converting rotation per second to counts per second, multiply RPS value with TPR  (Ticks per rotation)*/
    // double leftCPS    = leftRPS * m_TPR;
    // double rightCPS   = rightRPS * m_TPR;

    /*Note : Use either of above combinations as per requirement.Developer needs to update the leftMotorCommand & rightMotorCommand accordingly*/

    /*TODO: Update proper according to PLC expected left/right command*/


    /*TODO: Might be required to change according to implementation*/
    
    writeParam.m_leftDirection  = (leftRPM < 0.0) ? CW : CCW;
    writeParam.m_rightDirection = (rightRPM < 0.0) ? CW : CCW;
    

    // if(leftRPM < 0.0)
    // {
    //     writeParam.m_leftDirection = CW;
    // }
    // else if(leftRPM > 0.0)
    // {
    //     writeParam.m_leftDirection = CCW;
    // }

    // if(rightRPM < 0.0)
    // {
    //     writeParam.m_rightDirection = CW;
    // }
    // else if(rightRPM > 0.0)
    // {
    //     writeParam.m_rightDirection = CCW;
    // }

    writeParam.m_leftMotorCommand  = int(abs(leftRPM * m_gearRatio));
    writeParam.m_rightMotorCommand = int(abs(rightRPM * m_gearRatio));

}

/**
 * @brief  Updates the odometry using delta time,and encoder ticks
 * @param delta time in ms(double),encoder ticks in left and right (long),odometry (nav_msgs::Odometry)
 */
/*TODO : Careful with data types*/
void LowerLevelComputations::updateOdometry(uint32_t deltaTime, const readParameters &readParam, nav_msgs::Odometry &odometryData)
{
    /*Note: To account for the arc and inplace turns*/
    int32_t rightEncoder = readParam.m_rightMotorEncoder;
    int32_t leftEncoder  = readParam.m_leftMotorEncoder;
    processEncoderValues(leftEncoder, rightEncoder);

    if (deltaTime == 0)
    {
        return;
    }

    /*Since the LeftRPM is already inverted,deltaLeftWheel also should be inverted so as to unaffect the kinematics calculations*/

    m_deltaLeftWheel  = m_lastDiffReading[LEFT] * m_meterPerTick;
    m_deltaRightWheel = m_lastDiffReading[RIGHT] * m_meterPerTick;


    if (std::isnan(m_deltaLeftWheel))
    {
        m_deltaLeftWheel = 0.0;
    }

    if (std::isnan(m_deltaRightWheel))
    {
        m_deltaRightWheel = 0.0;
    }

    double deltaTheta       = (-m_deltaLeftWheel + m_deltaRightWheel) / m_wheelSeperation;
    double deltaS           = (m_deltaRightWheel + m_deltaLeftWheel) / 2.0;


    double leftMotorRPM     = (readParam.m_leftMotorSpeed) / m_gearRatio;
    double rightMotorRPM    = (readParam.m_rightMotorSpeed) / m_gearRatio;

    double leftSpeed        = leftMotorRPM * 2 * M_PI * m_wheelRadius / 60;
    double rightSpeed       = rightMotorRPM * 2 * M_PI * m_wheelRadius / 60;

    double v                = double((leftSpeed + rightSpeed) / 2);                 // deltaS / (deltaTime * 0.001);
    double w                = double((leftSpeed - rightSpeed) / m_wheelSeperation); // deltaTheta / (deltaTime * 0.001);

    if (m_barCodeDataReceived)
    {
        // ROS_WARN_NAMED("LowerLevelComputations","Correcting global odometry using PGV reading");
        m_odomPose[0] = m_pgvGlobalPose.x;
        m_odomPose[1] = m_pgvGlobalPose.y;
        m_odomPose[2] = m_pgvGlobalPose.yaw;
    }
    else
    {
        m_odomPose[0] += deltaS * cos(m_odomPose[2] + (deltaTheta / 2.0));
        m_odomPose[1] += deltaS * sin(m_odomPose[2] + (deltaTheta / 2.0));
        m_odomPose[2] += deltaTheta;
    }

    m_odomVel[0] = v; // m_linearVelocity;//v;
    m_odomVel[1] = 0.0;
    m_odomVel[2] = w; // m_angularVelocity;//w;

    odometryData.header.stamp       = ros::Time::now();
    odometryData.header.frame_id    = m_odomFrame;
    odometryData.child_frame_id     = m_baseFrame;

    odometryData.pose.pose.position.x   = m_odomPose[0];
    odometryData.pose.pose.position.y   = m_odomPose[1];
    odometryData.pose.pose.position.z   = 0.0;
    odometryData.pose.pose.orientation  = tf::createQuaternionMsgFromYaw(m_odomPose[2]);
    odometryData.twist.twist.linear.x   = m_odomVel[0];
    odometryData.twist.twist.angular.z  = m_odomVel[2];

    m_barCodeDataReceived = false;

}

double LowerLevelComputations::ticksToMeters(long ticks)
{
    return (2 * M_PI * m_wheelRadius * ticks) / m_TPR;
}

/**
* @brief  Reads the encoder values from the controller
* @param ticks for left and right motor (long)
*/
void LowerLevelComputations::processEncoderValues(const int32_t &leftEncoder, const int32_t &rightEncoder)
{
    int32_t currentReading;

    currentReading = leftEncoder;
    if (!m_initEncoder[LEFT])
    {
        m_lastReading[LEFT]         = currentReading;
        m_lastDiffReading[LEFT]     = 0;
        m_initEncoder[LEFT]         = true;
    }

    m_lastDiffReading[LEFT]         = currentReading - m_lastReading[LEFT];
    m_lastReading[LEFT]             = currentReading;



    currentReading = rightEncoder;
    if (!m_initEncoder[RIGHT])
    {
        m_lastReading[RIGHT]        = currentReading;
        m_lastDiffReading[RIGHT]    = 0;
        m_initEncoder[RIGHT]        = true;
    }
    m_lastDiffReading[RIGHT]        = currentReading - m_lastReading[RIGHT];
    m_lastReading[RIGHT]            = currentReading;

   // ROS_INFO("last right diff : %d",m_lastDiffReading[RIGHT]);
    //std::cout<<""<<std::endl;
}

