
#include "LowerLevelComputations.h"



/**
* @brief  Constructor for the LowerLevelComputations
*/
LowerLevelComputations::LowerLevelComputations():nhp("~")
{
    ROS_INFO("LowerLevelComputations Constructor called");
    initializeParameters();

    barcodePoseSub    = nh.subscribe(m_barcodeGlobalTopic.c_str(), 1, & LowerLevelComputations::barcodeCallback, this);
    debugCmdVelPub    = nh.advertise <geometry_msgs::Twist> (m_debugCmdVelTopic.c_str(), 1);
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
    nhp.param("robot/wheel_seperation", m_wheelSeperation,0.39);
    nhp.param("robot/wheel_radius", m_wheelRadius,0.10);
    nhp.param("robot/TPR", m_TPR,16384.0);
    nhp.param("robot/linear_vel_limit", m_linearVelocityLimit,2.0);
    nhp.param("robot/angular_vel_limit", m_angularVelocityLimit,1.0);
    nhp.param("robot/gearRatio", m_gearRatio,9.0);

    nhp.param<std::string>("odom_frame", m_odomFrame,"odom_frame");
    nhp.param<std::string>("base_frame", m_baseFrame,"base_footprint");
    nhp.param<std::string>("barcode_global_topic", m_barcodeGlobalTopic,"barcode/global");
    nhp.param<std::string>("debug_cmd_topic", m_debugCmdVelTopic, "debug/cmd_vel");

    // nh.getParam("robot/ticksPerRev", m_ticksPerRevolutionBaseMotor);

}


/**
* @brief  Subscribes the Barcode data from PGV for the odometry correction
*/
void LowerLevelComputations::barcodeCallback(const anscer_msgs::PGVPose &msg)
{
    m_pgvGlobalPose = msg;
    m_barCodeDataReceived = true;
}


/**
* @brief  Initializes the lower level computation by converting cmdVel to differential kinematics
* @param command velocity (geometry_msgs::Twist) ,left and right counts per second (double)
*/
void LowerLevelComputations::initializeComputation(const geometry_msgs::Twist &cmdVel, writeParameters &writeParam)
{
    m_meterPerTick= (2*M_PI*m_wheelRadius)/(m_TPR*m_gearRatio); // computing the conversion factor of ticks to meter (from wheel radius , gearratio and no of ticks per revolution in  base motor)

    m_linearVelocity  = cmdVel.linear.x;
    m_angularVelocity = cmdVel.angular.z;

    velocityLimiter(m_linearVelocity, m_angularVelocity);

    geometry_msgs::Twist cmdMsg;
    cmdMsg.linear.x  = m_linearVelocity;
    cmdMsg.angular.z = m_angularVelocity;
    debugCmdVelPub.publish(cmdMsg);

    motorDriveCommands(writeParam);
    
    /*Debugging purpose*  t1*/

   // debugCmdVelPub.publish();


}

/**
* @brief  Ensuring the robot velocity stay within the bounds
* @param command linear and angular velocity (double)
*/
void LowerLevelComputations::velocityLimiter(double &linearVel, double &angularVel)
{
    if (linearVel > m_linearVelocityLimit)
    {
        linearVel = m_linearVelocityLimit;
    }
    else if (linearVel < -m_linearVelocityLimit)
    {
        linearVel = -m_linearVelocityLimit;
    }

    if (angularVel > m_angularVelocityLimit)
    {
        angularVel = m_angularVelocityLimit;
    }
    else if (angularVel < -m_angularVelocityLimit)
    {
        angularVel = -m_angularVelocityLimit;
    }
}

/**
* @brief  Compute the motor driver commands in counts per second
* @param left and right counts per second (double)
*/
void LowerLevelComputations::motorDriveCommands(writeParameters &writeParam)
{
    double leftSpeed    = m_linearVelocity + m_angularVelocity * m_wheelSeperation / 2.0;
    double rightSpeed   = m_linearVelocity - m_angularVelocity * m_wheelSeperation / 2.0;

    double leftRPM      = 60 / (2 * M_PI * m_wheelRadius) * leftSpeed;
    double rightRPM     = 60 / (2 * M_PI * m_wheelRadius) * rightSpeed;


    /*Debugging purpose t2*/
    //.left_rpm_command  = leftRPM;
    //.right_rpm_command = rightRPM;


    /*Roboteq expects leftRPM  or rightRPM*/
    writeParam.m_leftMotorCommand = int(leftRPM*m_gearRatio);
    writeParam.m_rightMotorCommand = int(rightRPM*m_gearRatio);
    
    //ROS_ERROR("Left RPM : %f    Right RPM   %f    Command : %d     %d",leftRPM,rightRPM,writeParam.m_leftMotorCommand,writeParam.m_rightMotorCommand);

 //  ROS_INFO("right_direcion :  %d left_direction : %d  ", writeParam.m_rightDirection,writeParam.m_leftDirection);

}

/**
* @brief  Updates the odometry using delta time,and encoder ticks
* @param delta time in ms(double),encoder ticks in left and right (long),odometry (nav_msgs::Odometry)
*/
/*TODO : Careful with data types*/
void LowerLevelComputations::updateOdometry(uint32_t deltaTime,const readParameters &readParam,nav_msgs::Odometry &odometryData)
{
     

    /*Note: To account for the arc and inplace turns*/
    uint32_t leftEncoder  = readParam.m_rightEncoder;
    uint32_t rightEncoder = readParam.m_leftEncoder;
    processEncoderValues(leftEncoder, rightEncoder);

    if (deltaTime == 0)
    {
        return;
    }


    /*Since the LeftRPM is already inverted,deltaLeftWheel also should be inverted so as to unaffect the kinematics calculations*/
    m_deltaLeftWheel   =  m_lastDiffReading[RIGHT]  * m_meterPerTick;         
    m_deltaRightWheel  =  m_lastDiffReading[LEFT] * m_meterPerTick;    



    if (std::isnan(m_deltaLeftWheel))
    {
        m_deltaLeftWheel = 0.0;
    }

    if (std::isnan(m_deltaRightWheel))
    {
        m_deltaRightWheel = 0.0;
    }


    double deltaTheta = (m_deltaRightWheel - m_deltaLeftWheel) / m_wheelSeperation;
    double deltaS     = (m_deltaRightWheel + m_deltaLeftWheel) / 2.0;


    double leftMotorRPM                = readParam.m_leftSpeed/m_gearRatio;
    double rightMotorRPM               = readParam.m_rightSpeed/m_gearRatio;

    /*Debugging purpose  t3  */
   // left_rpm_feedback          = leftMotorRPM;
   // right_rpm_feedback         = rightMotorRPM;

    double rightSpeed                  = leftMotorRPM*2*M_PI*m_wheelRadius/60;
    double leftSpeed                   = rightMotorRPM*2*M_PI*m_wheelRadius/60;

    //double v                           = double((leftSpeed + rightSpeed) / 2) ;
    //double w                           = double((-leftSpeed + rightSpeed)/m_wheelSeperation);	

    double v = deltaS/(deltaTime*0.001);    
    double w = deltaTheta/(deltaTime*0.001);

    ROS_WARN("v : %f w: %f", v,w);

    //std::cout << "Left speed  : " << readParam.m_leftSpeed << "   Right Speed  :  "<< readParam.m_rightSpeed<<std::endl;
    //ROS_WARN("Left speed : %f   Right Speed : %f",readParam.m_leftSpeed,readParam.m_rightSpeed);
    /*Uses the barcode based odometry correction or the encoder based correction as per requirement/scenario*/
    
    if(m_barCodeDataReceived)
    {
        //ROS_WARN_NAMED("LowerLevelComputations","Correcting global odometry using PGV reading");
        m_odomPose[0]                       = m_pgvGlobalPose.x;
        m_odomPose[1]                       = m_pgvGlobalPose.y;
        m_odomPose[2]                       = m_pgvGlobalPose.yaw;
    }
    else
    {
        m_odomPose[0]                      += deltaS * cos(m_odomPose[2] + (deltaTheta / 2.0));
        m_odomPose[1]                      += deltaS * sin(m_odomPose[2] + (deltaTheta / 2.0));
        m_odomPose[2]                      += deltaTheta;
    }



    m_odomVel[0]                        = v; //m_linearVelocity;//v;
    m_odomVel[1]                        = 0.0;
    m_odomVel[2]                        = w; //m_angularVelocity;//w;


    odometryData.header.stamp           = ros::Time::now();
    odometryData.header.frame_id        = m_odomFrame;
    odometryData.child_frame_id         = m_baseFrame;

    /*
    odometryData.pose.covariance[0]     = 0.1;
    odometryData.pose.covariance[7]     = 0.1;
    odometryData.pose.covariance[14]    = 1e10;
    odometryData.pose.covariance[21]    = 1e10;
    odometryData.pose.covariance[28]    = 1e10;
    odometryData.pose.covariance[35]    = 0.1;
    */

    odometryData.pose.pose.position.x   = m_odomPose[0];
    odometryData.pose.pose.position.y   = m_odomPose[1];
    odometryData.pose.pose.position.z   = 0.0;
    odometryData.pose.pose.orientation  = tf::createQuaternionMsgFromYaw(m_odomPose[2]);
    odometryData.twist.twist.linear.x   = m_odomVel[0];
    odometryData.twist.twist.angular.z  = m_odomVel[2];
   // ROS_WARN("odomPose      : %f",m_odomPose[2]);
    // ROS_DEBUG_STREAM("odom Pose : " << m_odomPose[2]);

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
void LowerLevelComputations::processEncoderValues(const uint32_t &leftEncoder, const uint32_t &rightEncoder)
{
    uint32_t currentReading;

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

    //ROS_INFO("last right : %ld  diff : %ld",currentReading,m_lastReading[RIGHT]);
    //std::cout<<""<<std::endl;
}

