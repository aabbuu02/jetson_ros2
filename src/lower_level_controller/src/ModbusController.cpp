#include <ros/console.h>
#include "ModbusController.h"
/**
* @brief  Constructor for the HighLevelController
*/

ModbusController::ModbusController(ros::NodeHandle &nh):nhp("~")
{

    ROS_INFO("ModbusController Constructor called ");
    powerSrvc          = nh.serviceClient<std_srvs::Trigger>("power_control/master/agx/poweroff");
    lifterFeedbackPub  = nh.advertise <wms_data::Feedback> ("lifterFeedback", 1);
    distSub = nh.subscribe("disatance_to_goal", 1, &ModbusController::distGoalCb, this);
    chargeSub = nh.subscribe("charge_battery", 1, &ModbusController::chargeCallback, this);

    initialize();
}

/**
* @brief  Initializes the PLC controller
*/
void ModbusController::initialize()
{
    initializeParameters();

    /*Connects to the MODBUS device*/
    m_modbusDevice.connectToDevice(m_ipAddress, m_modbusPort);
}

void ModbusController::distGoalCb(const std_msgs::Float32 &distMsg)
{
    if(distMsg.data > 1.8f) m_homingDistance = 1;
    else m_homingDistance = 0;
}

void ModbusController::chargeCallback(const std_msgs::Int32 &msg)
{
   m_chargeNumber = msg.data == 0 ? 0 : 1;
   ROS_ERROR_STREAM("Charge callback " << m_chargeNumber);

}
/**
* @brief  Initializes parameters from the parameter server
*/
void ModbusController::initializeParameters()
{
    /*Configurations for initializing modbus communication*/
    nhp.param<std::string>("ip_address", m_ipAddress, "192.168.1.5");
    nhp.param("port", m_modbusPort, 443);
    // ROS_INFO_STREAM("IP " << m_ipAddress << " PORT " << m_modbusPort);
   
    nhp.param("modbus_read/time_stamp_current", m_timeStampCurrentReg, 0);
    nhp.param("modbus_read/left_motor_speed", m_leftMotorSpeedPin, 2);
    nhp.param("modbus_read/right_motor_speed", m_rightMotorSpeedPin, 3);
    nhp.param("modbus_read/left_motor_encoder", m_leftMotorEncoderPin, 4);
    nhp.param("modbus_read/right_motor_encoder", m_rightMotorEncoderPin, 6);

    nhp.param("modbus_read/battery_voltage", m_batteryVoltagePin, 9);
    nhp.param("modbus_read/battery_percentage", m_batteryPercentagePin, 10);
    nhp.param("modbus_read/power_off", m_powerOffPin,11);
    nhp.param("modbus_read/mission_feedback", m_missionFeedbackPin,17);
    nhp.param("modbus_read/buffer_feedback", m_bufferFeedbackPin,18);
    nhp.param("modbus_read/bot_shelfno", m_botShelfPin, 67);



    // nhp.param("modbus_read/location_id", m_locationIdPin, 10);

    // nhp.param("modbus_read/barcode_tag", m_barcodeTagPin, 17);
    // nhp.param("modbus_read/barcode_x", m_barcodeXPin, 18);
    // nhp.param("modbus_read/barcode_y", m_barcodeYPin, 20);
    // nhp.param("modbus_read/barcode_theta", m_barcodeThetaPin, 22);    

    nhp.param("modbus_write/time_stamp_update", m_timeStampUpdateReg, 100);
    nhp.param("modbus_write/left_motor_command", m_leftMotorCommandPin,102);
    nhp.param("modbus_write/right_motor_command", m_rightMotorCommandPin,103);

    nhp.param("modbus_read/emergency_feedback", m_emergencyFeedbackPin, 17);

    nhp.param("modbus_write/pick_location", m_pickLocationCommandPin, 115);
    nhp.param("modbus_write/pick_orientation", m_pickOrientationCommandPin, 114);
    nhp.param("modbus_write/drop_location", m_dropLocationCommandPin, 117);
    nhp.param("modbus_write/drop_orientation", m_dropOrientationCommandPin, 116);
    nhp.param("modbus_write/mission_confirmation", m_missionConfirmationCommandPin, 105);
    nhp.param("modbus_write/tast_command", m_missionTaskPin, 142);
    nhp.param("modbus_write/homing_commmand", m_homingDistancePin,143);
    nhp.param("modbus_write/charge_pin",m_chargePin,145);
  
    nhp.param("modbus_write/brake_command", m_brakeCmdPin, 144);
    

}

uint32_t ModbusController::convertDoubleInt(uint16_t low, uint16_t high)
{
    return (static_cast<uint32_t>(high) << 16) | ((static_cast<uint32_t>(low)) & 0xFFFF);
}

/**
* @brief  Controlling the motor controller through motor commands
* @param  left and right RPS/CPS in double datatype
*/
void ModbusController::writeCommands(writeParameters &writeParams)
{
    writeRegData[m_timeStampUpdateReg - STARTINGINDEX]       = writeParams.m_updatedTimeStamp >> 16;
    writeRegData[(m_timeStampUpdateReg + 1) - STARTINGINDEX] = writeParams.m_updatedTimeStamp & 0xFFFF;
    writeRegData[m_homingDistancePin  - STARTINGINDEX] = m_homingDistance;
    bool stopCondition                          = ((writeParams.m_leftMotorCommand == 0) && (writeParams.m_rightMotorCommand == 0));
    lift_action::LiftAction lifterCommand       = writeParams.m_lifterData;

    if (stopCondition == false)
    {
        int leftDirection  = (writeParams.m_leftDirection == 0)? -1 : 1;
        int rightDirection = (writeParams.m_rightDirection == 0)? -1 : 1;



        writeRegData[m_leftMotorCommandPin - STARTINGINDEX]  = (writeParams.m_leftMotorCommand * leftDirection)/2;
        writeRegData[m_rightMotorCommandPin - STARTINGINDEX] = (writeParams.m_rightMotorCommand * rightDirection)/2;

          // writeRegData[m_leftBrakeCommandPin - STARTINGINDEX]        = 0;
          // writeRegData[m_rightBrakeCommandPin - STARTINGINDEX]       = 0;
    }
    else
    {
        writeRegData[m_leftMotorCommandPin - STARTINGINDEX]  = 0;
        writeRegData[m_rightMotorCommandPin - STARTINGINDEX] = 0;
    }

    //Brake activation
    writeRegData[m_brakeCmdPin - STARTINGINDEX] = writeParams.m_lifterData.reached ? 1 : 0;
    writeRegData[m_chargePin - STARTINGINDEX]  = m_chargeNumber;
 
   ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    feedbackMsg.unique_id = lifterCommand.unique_id;
    feedbackMsg.action    = lifterCommand.action;

   ROS_INFO_STREAM("Mission END : "<<m_missionCompleted  <<" TOP Module : "  <<m_topModuleInitiated);
    if ((m_botShelfNo > 0 || m_missionCompleted) && m_topModuleInitiated)
    {
        ROS_WARN("Hurrey...!! Current mission completed  ...!!");
        if (lifterCommand.action == ACR_PICKCOMMAND) /*Applicable only if placed To Robot*/
        {
        feedbackMsg.acr_shelf = m_bufferFeedback; /*Get the feedback from PLC and update in the message*/
        }

        feedbackMsg.status = true;
        m_topModuleInitiated=false;
        m_lastId = lifterCommand.unique_id;
    }


    bool initiateTopModule  = (lifterCommand.reached && (m_lastId !=lifterCommand.unique_id));   //&& !m_missionUpdated); 
  
    if(m_lastId != lifterCommand.unique_id) 
    {
	    feedbackMsg.status = false; 
            feedbackMsg.acr_shelf=0;

    }

    if(initiateTopModule) /*Command to TOP module shall be given on 
                                                            1.receiving feedback from lift_action  as reached AND
                                                            2 with a new unique_id AND   f
                                                            3.feedback from PLC on mission updation status as false (m_missionUpdated)  */
    {
        m_topModuleInitiated=true;
        if(lifterCommand.action == ACR_PICKCOMMAND)
        {
            writeRegData[m_pickLocationCommandPin - STARTINGINDEX]        = lifterCommand.shelf;
            writeRegData[m_pickOrientationCommandPin - STARTINGINDEX]     = ((lifterCommand.rack %2 == 0)?LEFT_ORIENTATION:RIGHT_ORIENTATION); /*If rack number is even LEFT side else RIGHT side*/
            writeRegData[m_dropLocationCommandPin - STARTINGINDEX]        = m_bufferFeedback;          /*While picking from shelf to robot,the acr shelf details shall be taken from PLC*/
            writeRegData[m_dropOrientationCommandPin - STARTINGINDEX]     = ACR_ORIENTATION;
            writeRegData[m_missionConfirmationCommandPin - STARTINGINDEX] = !m_missionUpdated;         /*This confirmation should be set only until the m_missionUpdated is resetted*/
    	    writeRegData[m_missionTaskPin - STARTINGINDEX] = m_missionUpdated ? 0 : 1;       /*O when mission updated and 1 for pick*/
	        ROS_INFO_STREAM("Pick command shelf  :"<<lifterCommand.shelf<<" Pick command : "<< writeRegData[m_pickOrientationCommandPin - STARTINGINDEX] << "  Confirmation command "<<  writeRegData[m_missionConfirmationCommandPin - STARTINGINDEX]);
    	}
        else if(lifterCommand.action == ACR_DROPCOMMAND)
        {
           writeRegData[m_pickLocationCommandPin - STARTINGINDEX]         = lifterCommand.acr_shelf;   /*While dropping from robot to shelf ,the acr_shelf shall be taken from WMS (lift_action)*/
            writeRegData[m_pickOrientationCommandPin - STARTINGINDEX]     = ACR_ORIENTATION;
            writeRegData[m_dropLocationCommandPin - STARTINGINDEX]        = lifterCommand.shelf;
            writeRegData[m_dropOrientationCommandPin - STARTINGINDEX]     = ((lifterCommand.rack %2 == 0)?LEFT_ORIENTATION:RIGHT_ORIENTATION);/*If rack number is even LEFT side else RIGHT side*/
            writeRegData[m_missionConfirmationCommandPin - STARTINGINDEX] = !m_missionUpdated;         /*This confirmation should be set only until the m_missionUpdated is resetted*/
            writeRegData[m_missionTaskPin - STARTINGINDEX] = m_missionUpdated ? 0 : 2;                 /*O when mission updated and 2 for pick*/
	        ROS_INFO_STREAM("Drop command shelf  :"<<lifterCommand.shelf<<" Drop  command : "<< writeRegData[m_pickOrientationCommandPin - STARTINGINDEX]<< "  Confirmation command "<< writeRegData[m_missionConfirmationCommandPin - STARTINGINDEX]);

        }
        else
        {
            ROS_FATAL_NAMED("ModbusController","Invalid ACR command passed .Please check the command again");
            //return;
        }





          /*Once the goal is reached,the PICK/DROP command should be given*/

        if(m_isBinRetracted)  /*From Robot -> After placing on rack;
                               To Robot   -> Post extraction
                               Also note that NOT to take a new command before completion of previous goal*/
        {
            /*failStatus is OBSOLETE in the current stack*/
        }

        /*if (m_missionCompleted)
        {

            if (readCounter > 2) //To ensure that the m_missionCompleted has got updated from readCommands() and 
                                    NO false switching of mission complete occurs during new goal//
            {
                ROS_WARN("Hurrey...!! Current mission completed  ...!!");
                if (lifterCommand.action == ACR_PICKCOMMAND) //Applicable only if placed To Robot//
                {
                    feedbackMsg.acr_shelf = m_bufferFeedback; //Get the feedback from PLC and update in the message
                }
                m_lastId = lifterCommand.unique_id;
            }
            readCounter++;
        }
        else
        {
            readCounter = 0;
        }*/
    }
    else
    {
        ROS_DEBUG_NAMED("ModbusController","Robot in moving state: resetting all the top module values");
    }



    lifterFeedbackPub.publish(feedbackMsg);



  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //WRITEREGISTERNUMBER
    for(int i = 0; i < WRITEREGISTERNUMBER; ++i)
    {
    	ROS_DEBUG_STREAM_NAMED("modbus_write","Write ["<<i<<"]: "<<writeRegData[i]);	    
    }

    m_modbusDevice.writeRegisters(STARTINGINDEX, WRITEREGISTERNUMBER, writeRegData);
}



/**
* @brief  ReadiModbusControllerng the encoder readings from controller 
* @param  left and right ticks from the controller in long data type
*/
void ModbusController::readCommands(readParameters &readParam)
{
    /*Reads the entire read register data*/
    m_modbusDevice.readRegister();
   
    /*Individual register data from PLC*/

    readParam.m_currentTimeStamp         = convertDoubleInt(m_modbusDevice.m_registerData[m_timeStampCurrentReg+1], m_modbusDevice.m_registerData[m_timeStampCurrentReg]);

    readParam.m_leftMotorSpeed           = (int16_t)m_modbusDevice.m_registerData[m_leftMotorSpeedPin];
    readParam.m_rightMotorSpeed          = (int16_t)m_modbusDevice.m_registerData[m_rightMotorSpeedPin];
    readParam.m_emergencyFeedback        = m_modbusDevice.m_registerData[m_emergencyFeedbackPin];
    readParam.m_botShelfNo               = m_modbusDevice.m_registerData[m_botShelfPin];
    m_botShelfNo = readParam.m_botShelfNo;
    std::bitset<16> missionFeedback       = m_modbusDevice.m_registerData[m_missionFeedbackPin];
    m_missionUpdated                     = missionFeedback[8]; /*Confirmation feedback that PLC accepted the given mission*/
    m_missionCompleted                   = missionFeedback[9]; /*Status of current mission from PLC*/
    // std::cout <<  "Total feedback byte "<< missionFeedback << " at " <<m_missionFeedbackPin<< std::endl;
    ROS_INFO_STREAM("Confirmation f/b :  "<<m_missionUpdated<<"  Mission status :  "<<m_missionCompleted <<"   Top_module :" << m_topModuleInitiated );
    m_bufferFeedback                     =  readParam.m_botShelfNo;
    // ROS_WARN_STREAM("Buffer f/b :  "<<m_bufferFeedback<<"  at  "<<m_bufferFeedbackPin);

    

    if(readParam.m_emergencyFeedback)
    {
        m_prevRightEncoder               = readEncoder(m_rightMotorEncoderPin);
        m_prevLeftEncoder                = readEncoder(m_leftMotorEncoderPin);
    }
    else
    {
        int32_t leftEncoder              = readEncoder(m_leftMotorEncoderPin);
        int32_t rightEncoder             = readEncoder(m_rightMotorEncoderPin);
        
        readParam.m_leftMotorEncoder     = readParam.m_leftMotorEncoder + (leftEncoder - m_prevLeftEncoder);
        readParam.m_rightMotorEncoder    = readParam.m_rightMotorEncoder + (rightEncoder - m_prevRightEncoder);
        m_prevRightEncoder               = readParam.m_rightMotorEncoder; 
        m_prevLeftEncoder                = readParam.m_leftMotorEncoder; 
    }

    readParam.m_batteryVoltage           = m_modbusDevice.m_registerData[m_batteryVoltagePin];
    readParam.m_batteryPercentage        = m_modbusDevice.m_registerData[m_batteryPercentagePin];
    int  robotPowerOff                   = m_modbusDevice.m_registerData[m_powerOffPin];


    for(int i = 0; i < READREGISTERNUMBER; ++i)
    {
        ROS_DEBUG_STREAM_NAMED("modbus_read","Read ["<<i<<"]: "<<(int)m_modbusDevice.m_registerData[i]);
    }
    


    /*Powering off the robot from the hardware button*/
    if((robotPowerOff == ROBOT_POWEROFF) && !m_powerOffInitiated)
    {
        m_powerOffInitiated = true;
        std_srvs::Trigger trigger;
        powerSrvc.call(trigger);
        ROS_ERROR_STREAM_NAMED("ModbusController","Powering off the Robot....!!  "<<robotPowerOff);
    }

}

/**
* @brief  Reading the encoder readings from controller and estimates the aggregated data
* @returns The aggregated encoder output data 
*/
int32_t ModbusController::readEncoder(int encoder_num)
{
    //ROS_INFO("Encoder %d  and %d ----   values %lu   %lu",encoder_num,encoder_num+1,m_modbusDevice.m_registerData[encoder_num],m_modbusDevice.m_registerData[encoder_num+1]);
    int32_t distanceInCm = (m_modbusDevice.m_registerData[encoder_num] * 65536) + m_modbusDevice.m_registerData[encoder_num+1];
    return distanceInCm;
}
