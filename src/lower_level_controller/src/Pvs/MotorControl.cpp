#include "MotorControl.h"

/**
*@brief constructor for MotorControl
*/
MotorControl::MotorControl()
{
  m_motorReturnCommand = 0;
  initialiseConnection();
}

MotorControl::~MotorControl()
{

}
int MotorControl::initialiseConnection()
{
    /**
    *@brief for connecting to roboteq device
    */
    m_status=device.Connect("/dev/ttyACM0");

    if (m_status != RQ_SUCCESS)
    {
        std::cout << "Error connecting to device: " << m_status << "." << std::endl;
        return -1;
    }
    else if (m_status == RQ_SUCCESS)
    {
        std::cout << "Roboteq device connected successfully " << std::endl;
    }
    //std::cout<<device.IsConnected()<<std::endl;
    return 0;
    
}

/**
*@brief function definition for writeToMotor()
*/
int MotorControl::writeToMotor(float leftRpm, float rightRpm)
{

  std::cout << "leftRPM:=" << leftRpm/10 << "     rightRpm:=" <<rightRpm/10 << std::endl;
  // 
 // device.Connect("/dev/ttyACM0");
  m_motorReturnCommand = device.SetCommand(_GO, 1, leftRpm);
  m_motorReturnCommand = device.SetCommand(_GO, 2, rightRpm);
  
	
 // m_motorReturnCommand = RoboteqDevice.SetCommand(_GO, 2, 0);
  // if (m_motorReturnCommand != RQ_SUCCESS)
  // {
  //   std::cout << "sending motor command failed" << m_motorReturnCommand << std::endl;
  // }
  // else

  // {
  //   std::cout << "motor command sending successfull" << std::endl;
  // }
}