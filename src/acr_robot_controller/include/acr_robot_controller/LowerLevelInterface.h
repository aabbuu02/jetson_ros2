#ifndef LOWER_LEVEL_INTERFACE_H
#define LOWER_LEVEL_INTERFACE_H


#include "RosClass.h"
#include "LowerLevelComputations.h"
#include "Client.h"

#define MACADDRESS "48:b0:2d:5f:86:30"


using namespace std;
/**
   * @class LowerLevelController
   * @brief Lower level controller for reading encoding and writing motor driver,publishing odometry values
   */

class LowerLevelInterface
{
public:
    /**
    * @brief  Constructor for the LowerLevelInterface
    */
    LowerLevelInterface();
    
    /**
    * @brief  Destructor for the LowerLevelInterface
    */
    ~LowerLevelInterface();
    void controlFlow();

private:

    ros::NodeHandle nh;
    ros::NodeHandle nhp;

    // /*Member function*/
    void initializeParameters();
    void authenticationProcess();
    void getMACAddress();
    void diagnoseMotorControllerHealth(const readParameters &readParams,geometry_msgs::Twist &cmdVel);
   // void controlFlow();
    void resetCmdVelocity(geometry_msgs::Twist &cmdVel);

    

    /*Member variables*/

    int64_t m_currentTime, m_lastTime, m_deltaTime;
    int m_controllerType =1;
    bool m_publishTF      = false;
    std::string m_macAddress;
    nav_msgs::Odometry m_odom;
    writeParameters m_writeParams;
    readParameters m_readParams;


    /*Class instances*/


    RosClass *p_getVariables;
    Client *p_client;
    AHighLevelController *p_highLevelController;
    LowerLevelComputations *p_lowerLevelComputation;   

};
#endif
