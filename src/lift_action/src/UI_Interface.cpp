/**
 * @file UIInterface.cpp
 * @author Jishnu (jishnur@anscer.com)
 * @brief interfacing UI for the ACR
 * @version 0.1
 * @date 2022-10-25
 * @copyright Copyright (c) 2022
 */
#include "lift_action/HigherLevelInterface.h"

/**
 * @brief constructor definition for UIInterface class
*/

UIInterface::UIInterface(ModbusCommunicator *mcPtr):
p_modbusCommunicatorPtr(mcPtr)
{

m_packageName="lift_action";
m_homingMsg.process="";
m_homingMsg.button=0;
m_homingMsg.direction=0;
eStopDataSubscriber = nh.subscribe("/e_stop", 1, &UIInterface::eStopCallback, this);
//homingDataSubscriber =nh.subscribe("/lifter/home_goal", 1, &UIInterface::homingCallback, this);
cycleResetDataSubscriber =nh.subscribe("/lifter/cycle_reset", 1, &UIInterface::cycleResetCallback, this);
homingButtonSubscriber =nh.subscribe("/lifter/homing_button",1,&UIInterface::homingButtonCallback, this);
//homingStatusPublisher=nh.advertise<lift_action::Homing_status>("/homing_status",1);



}

/**
 * @brief destructor definition for UIInterface class
*/
UIInterface::~UIInterface()
{

}
/**
 *@brief callback function for homingButton
 */
void UIInterface::homingButtonCallback(const std_msgs::Bool &msg)
{
	if(msg.data == true)
	{
		p_modbusCommunicatorPtr->writeDataToRegister(19,1);
		sleep(1);
		p_modbusCommunicatorPtr->writeDataToRegister(19,0);

	}
	else
	{
		p_modbusCommunicatorPtr->writeDataToRegister(19,0);

	}

}
/**
 * @brief callbackfunction for estop
*/
void UIInterface::eStopCallback(const std_msgs::Bool &msg)
{


}
/**
 * @brief callbackfunction for homing
*/
void UIInterface::homingCallback(const lift_action::UI_Interface &msg)
{
  m_homingMsg=msg;
 // homingControl(m_homingMsg);
}

/**
 * @brief callbackfunction for cycleReset
*/
void UIInterface::cycleResetCallback(const std_msgs::Bool &msg)
{
   if(msg.data == 1)
    {
        p_modbusCommunicatorPtr->writeDataToRegister(47,1);
        sleep(2);
         p_modbusCommunicatorPtr->writeDataToRegister(47,0);
    }
  else
    {
       p_modbusCommunicatorPtr->writeDataToRegister(47,0);
    }
}
/**
 * @brief homingControlFunction for doing homing
*/
int UIInterface::homingControl(lift_action::UI_Interface homingMsg)
{

  
  std::string process;
  int direction=0;
  process=homingMsg.process.c_str();
  direction=homingMsg.direction;

  lift_action::Homing_status homeSensor;
  //homeSensor.process=homingMsg.process.c_str();
  
  // m_homingStatus.lifterStatus=p_modbusCommunicatorPtr->readRegister(81);;
  // m_homingStatus.extractorSatus=p_modbusCommunicatorPtr->readRegister(82);
  // m_homingStatus.turnTableStatus=p_modbusCommunicatorPtr->readRegister(83);
  

 // homingStatusPublisher.publish(m_homingStatus);


}
