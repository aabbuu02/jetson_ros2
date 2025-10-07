/**
 * @file HigherLevelInterface.cpp
 * @author Jishnu (jishnu@anscer.com)
 * @brief communication node used for communicating between two  PLC s using modbus protocol
 *
 * @version 0.1
 * @date 2022-07-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */
/**
 * @brief include headerfiles
 */

#include "lift_action/HigherLevelInterface.h"

HigherLevelInterface::HigherLevelInterface()
{
    ROS_INFO("Higher_interface_constructor_called");
    ROS_INFO("setting up device");
    sleep(60);
    p_modbusCommunicator=new ModbusCommunicator();

    p_UIInterface=new UIInterface(p_modbusCommunicator);

    m_lifterErrorStatus.lifterFault=0;
    m_lifterErrorStatus.extractorFault=0;
    m_lifterErrorStatus.turnTableFault=0;
    m_lifterErrorStatus.swingArmFault=0;
    
    m_lifterErrorStatus.emergencyPBfault=0;
    m_lifterErrorStatus.swingArmSensorZeroFault=0;
    m_lifterErrorStatus.swingArmSensorNinetyFault=0;
    m_lifterErrorStatus.turntableSensorZeroFault=0;
    m_lifterErrorStatus.extractorSensorLaserFault=0;


    m_updateRate={0.00};
    m_numberOfReadRegisters={0};
    m_lifterFeedbackData.unique_id="";
    m_lifterFeedbackData.status={0};
    m_wmsUpdateFlag=1;
    m_agvStopFlag={0};
    m_agvActionFlag={0};
    m_reachedFlag={0};
    m_extractorStartFlag={0};
    m_lifterFeedbackFlag={1};
    m_locationChangeFlag={0};
    m_operationFlag={0};
    m_failStatusFlag={0};
    m_barcodeTriggerStatusFlag={0};
    m_prevAcrShelf={0};
    m_lastAction={0};
    
    m_triggerCount={0};
    m_triggerThreshold={0};
    m_scannedData="";
    m_uniqueId ="NONE";

    m_prevUniqueID="Data1";
    m_lastUniqueID="";
    m_parameterExistanceFlag=checkParameters();

    if(m_parameterExistanceFlag==0)
    {
        ROS_ERROR("failed to load HigherLevelInterface parameters");
        
        exit(0);
    }
    else
    {
        int feedBack=readParameters();
        if(feedBack !=0)
          {
            ROS_ERROR("parameter reading failed");
            
            
            exit(0);
            
          }
        else
          {
            ROS_INFO("All HigherLevelInterface parameter read successfully");
            
            
            
            
          }
    }
    
    /**
     * @brief publisher definition
     * 
     */
    lifterFeedback=p_modbusCommunicator->nh.advertise<lift_action::LiftFeedback>(m_publisherTopic,1);
    robotInitialisedPublisher=p_modbusCommunicator->nh.advertise<std_msgs::Bool>("/robot_initialized",1);
    robotFeedBack = p_modbusCommunicator->nh.subscribe(m_subscriberTopic, 1, &HigherLevelInterface::robotFeedbackCallback, this);
    brakeActivator=p_modbusCommunicator->nh.advertise<std_msgs::Bool>("/brake_activator",1);
    robotState=p_modbusCommunicator->nh.subscribe("/robot_state",1,&HigherLevelInterface::robotStateCallback,this);
    scannerTrigger=p_modbusCommunicator->nh.advertise<std_msgs::Bool>("/barcode_trigger",1);
    errorStatus=p_modbusCommunicator->nh.advertise<lift_action::ErrorStatus>("/lifter/error_status",1);
    homingStatus=p_modbusCommunicator->nh.advertise<std_msgs::Bool>("/lifter/homing_status",1);
    scannerDataSubscriber=p_modbusCommunicator->nh.subscribe("/barcode_data",1,&HigherLevelInterface::scannerDataCallback, this);
    robotLocationChangeSubscriber=p_modbusCommunicator->nh.subscribe("/robot_location_change",1,&HigherLevelInterface::robotLocationStatusCallback, this);
    barcodeTriggerStatus=p_modbusCommunicator->nh.subscribe("/barcode/trigger_status",1,&HigherLevelInterface::barcodeTriggerStatusCallback, this);
    wmsUpdateStatus=p_modbusCommunicator->nh.subscribe("/wms_status",1,&HigherLevelInterface::wmsStatusCallback, this);
    bypassButton=p_modbusCommunicator->nh.subscribe("/bypass",1,&HigherLevelInterface::bypassButtonCallback, this);

}
/**
 * @brief Destroy the Higher Level Interface:: Hiigher Level Interface object
 * 
 */
HigherLevelInterface::~HigherLevelInterface()
{

delete p_modbusCommunicator;
}
/**
 * @brief callback function for barcode trigger status
*/
void HigherLevelInterface::barcodeTriggerStatusCallback(const std_msgs::Bool &msg)
{
  m_barcodeTriggerStatusFlag=msg.data;
}

/**
*@brief callback function for wms status
*/
void HigherLevelInterface::wmsStatusCallback(const std_msgs::Bool &msg)
{
ROS_INFO("wms update called");
  if(msg.data==true)
   {
     m_wmsUpdateFlag=1;
    ROS_INFO("wms updated");
   }
  else
   {
   ROS_INFO("update failed command recived");
     m_wmsUpdateFlag=0;
   }
  
}
/**
*@brief callback function for robot location status
*/
void HigherLevelInterface::robotLocationStatusCallback(const std_msgs::Bool &msg)
{
  m_locationChangeFlag=msg.data;

    if(m_locationChangeFlag==1)
    {
	   ROS_WARN("deactivating brake ");
     std_msgs::Bool brake;
     brake.data=0;
     brakeActivator.publish(brake);
     sleep(1);
     robotInitialised.data=1;
     robotInitialisedPublisher.publish(robotInitialised);
      

    }
    
}


/**
 * @brief callback function for bypassbutton
*/
void HigherLevelInterface::bypassButtonCallback(const std_msgs::Bool &msg)
{
  ROS_INFO("bypassing button callback called");
  if(msg.data==true)
    {
      writeData(48,1);
    }
  else
    {
      writeData(48,0);
    }
}





/**
 * @brief callback function for scannerDataCallback
 * 
 */

void HigherLevelInterface::scannerDataCallback(const std_msgs::String &msg)
{
  // ROS_WARN("read data =%s",msg.data.c_str());
  std::string data=msg.data.c_str();
  m_scannedData=data;
  // ROS_WARN("scanned data = %s",m_scannedData.c_str());
}

/**
 * @brief callback function for robotFeedbackCallback
 * 
 */
void HigherLevelInterface::robotFeedbackCallback(const lift_action::LiftAction &msg)
{
  if(msg.reached==true)
  {
    std_msgs::Bool brake;
    brake.data=1;
    brakeActivator.publish(brake);
    m_uniqueId=msg.unique_id.c_str();
    ROS_ERROR("m_unique id recived %s ",m_uniqueId.c_str());
    m_action=msg.action;
    m_shelf=msg.shelf;
    m_acrShelf=msg.acr_shelf;
    m_boxNo=msg.putaway_barcode;
    m_rackNo=msg.whole_bin_no;
    int temp=msg.rack;
    int side=0;
    side=temp%10;
    if(temp>1052)
     {
      if(side==1)
       {
        m_side=2;
       }
      else if(side==2)
       {
       m_side=1;
       }

     }
    else
     {
       m_side=side;
     }
    
    m_reachedFlag=1;
    m_shelf=(m_shelf%10)+1;
  }
  else
  {
   m_reachedFlag=0;

  }




}
/**
 *@brief robotStateCallback function definition
 */
void HigherLevelInterface::robotStateCallback(const std_msgs::Bool &msg)
 {
   m_robotMovementFlag=uint16_t(msg.data);
 }



//////////////////////////////////////////////////////////////////
/**
 * @brief for sending confirmation if the picking or dropping fails
 * @brief function definition
*/

int HigherLevelInterface::failureFeedback(std::string uniqueId,int action)
{
  if(action==1)
    {
      writeData(5,0);
      writeData(6,1);
    }
  else
    {
      writeData(5,0);
      writeData(6,2);
    }

    m_lifterFeedbackData.unique_id=uniqueId;
    m_lifterFeedbackData.status=1;
	  m_lifterFeedbackData.acr_shelf=0;
    m_lifterFeedbackData.action=action;
    m_lifterFeedbackData.failStatus=1;

    lifterFeedback.publish(m_lifterFeedbackData);
    
    std_msgs::Bool brake;
           brake.data=0;
	  brakeActivator.publish(brake);
	  sleep(1);

	//robotInitialised.data=1;
	//robotInitialisedPublisher.publish(robotInitialised);
        m_agvStopFlag=0;

}

//////////////////////////////////////////////////////////////////




/**
 * @brief writing data to plc
 * 
 */
int HigherLevelInterface::sendDataToExtractor()
{
  /**
   * @brief checks register 64 ==0 means extractor not ready for cartonbox scanning
   * @brief check register 60 ==1 means extractor ready for picking
   * @brief checks register 61==0 means extractor is having no fault 
   */
  std_msgs::Bool homingData;
  homingData.data=p_modbusCommunicator->readRegister(84);
  homingStatus.publish(homingData);

  int regData=0;
  regData=p_modbusCommunicator->readRegister(87);
  if(regData==0)
  {
    m_lifterErrorStatus.lifterFault=0;
    m_lifterErrorStatus.extractorFault=0;
    m_lifterErrorStatus.turnTableFault=0;
    m_lifterErrorStatus.swingArmFault=0;
   m_lifterErrorStatus.emergencyPBfault=0;
   m_lifterErrorStatus.swingArmSensorZeroFault=0;
    m_lifterErrorStatus.swingArmSensorNinetyFault=0;
    m_lifterErrorStatus.turntableSensorZeroFault=0;
    m_lifterErrorStatus.extractorSensorLaserFault=0;

}

  else
  {
    if(regData==1)
      {
       m_lifterErrorStatus.emergencyPBfault=1;
      }
    else if(regData==2)
      {
       m_lifterErrorStatus.lifterFault=1;
      }
    else if(regData==3)
    {
      m_lifterErrorStatus.turnTableFault=1;
    }
    else if(regData==4)
    {
      m_lifterErrorStatus.extractorFault=1;
    }
    else if(regData==5)
    {
      m_lifterErrorStatus.swingArmFault=1;
    }
    else if(regData==6)
    {
      m_lifterErrorStatus.swingArmSensorZeroFault=1;
    }
    else if(regData==7)
    {
      m_lifterErrorStatus.swingArmSensorNinetyFault=1;
    }
    else if(regData==8)
    {
      m_lifterErrorStatus.turntableSensorZeroFault=1;
    }
    else if(regData==9)
    {
      m_lifterErrorStatus.extractorSensorLaserFault=1;
    }    
  }
 errorStatus.publish(m_lifterErrorStatus);
   
 //ROS_ERROR("m_action =%d  m_agvStopFlag=%d   m_lifterFeedbackFlag = %d ",m_action,m_agvStopFlag,m_lifterFeedbackFlag);
   if(m_reachedFlag==1 && m_agvStopFlag==0)
    {
       if(((m_uniqueId!=m_prevUniqueID) ||(m_lastAction != m_action)) && m_wmsUpdateFlag==1 )
          {
            
            ROS_INFO("new data recieved for action");
            m_prevUniqueID=m_uniqueId;


            m_agvStopFlag=1;
            m_agvActionFlag=0;
            m_extractorStartFlag=0;
	    
          }
    }

  //ROS_ERROR("m_action =%d  m_agvStopFlag=%d   m_lifterFeedbackFlag = %d ",m_action,m_agvStopFlag,m_lifterFeedbackFlag);
  else if(m_action==1 && m_agvStopFlag==1 && m_lifterFeedbackFlag==1)
  {
   
    if(p_modbusCommunicator->readRegister(60) == 1 && p_modbusCommunicator->readRegister(61) == 0 && m_agvActionFlag==0 && m_extractorStartFlag==0)
    {
      
      m_triggerCount=0;
      m_failStatusFlag=0;
      m_wmsUpdateFlag=0;


      m_robotMovementFlag=0;
      writeData(7,0);
      std_msgs::Bool brake;
      brake.data=1;
      brakeActivator.publish(brake);
      //sleep(1);
      writeData(1,m_shelf);
      ROS_INFO("m_shelf = %d",m_shelf);
      writeData(2,0);
      writeData(3,m_side);
      ROS_INFO("m_side = %d",m_side);
      writeData(4,1);      // start picking cycle
      m_agvActionFlag=1;
      sleep(1);
      writeData(4,0);
      m_prevAcrShelf=0;
      m_operationFlag=1;
    }
    /**
     * @brief once the extractor reaches the height for scanning the qrcode on carton box register 64 =1 
     * @brief scanner will be triggered for scanning
     * 
     */
    else if(p_modbusCommunicator->readRegister(64) == 1 &&  m_agvActionFlag==1 && m_extractorStartFlag==0 && m_barcodeTriggerStatusFlag==1)
    {
       if(m_triggerCount>=m_triggerThreshold)
       {
        failureFeedback(m_prevUniqueID,1);
        m_failStatusFlag=1;
       }
       ROS_WARN("scannning");
       std_msgs::Bool triger;
       triger.data=1;
       scannerTrigger.publish(triger);
       //scannerTrigger.publish(triger);
       //scannerTrigger.publish(triger);
      // sleep(1);
       m_triggerCount++;
    }
    /**
     * @brief checks whether read data is same as wms data
     * 
     */
    /*changed  m_boxNo to m_rackNo*/
   // if(m_scannedData==m_rackNo  && m_agvActionFlag==1 && m_failStatusFlag!=1)
   if( m_agvActionFlag==1 && m_failStatusFlag!=1)

     {
      ROS_WARN("scanned data matched with wms data");
      m_triggerCount=0;
      m_agvActionFlag=0;
      m_extractorStartFlag=1;
      writeData(5,1);      //carton box scanning complete 
      writeData(6,0); 
      std_msgs::Bool triger;
      triger.data=0;
      scannerTrigger.publish(triger);  
      m_lastUniqueID=m_prevUniqueID;
      sleep(1);
     
    }
  }


/**
   * @brief checks register 64 ==0 means extractor not ready for cartonbox scanning
   * @brief check register 60 ==1 means extractor ready for picking
   * @brief checks register 61==0 means extractor is having no fault 
   */
 else  if(m_action==0 && m_agvStopFlag==1)
  {
    if(p_modbusCommunicator->readRegister(66) == 1 && p_modbusCommunicator->readRegister(67) == 0 && m_agvActionFlag==0 && m_extractorStartFlag==0)
    {
     
     m_triggerCount=0;

     m_failStatusFlag=0;
     m_wmsUpdateFlag=0;


      m_robotMovementFlag=0;
      writeData(7,0);
      std_msgs::Bool brake;
      brake.data=1;
      brakeActivator.publish(brake);
      //sleep(1);

      m_lifterFeedbackFlag =0;
      writeData(1,m_acrShelf);
      writeData(2,m_shelf);
      writeData(3,m_side);
      writeData(4,2);      // start picking cycle
      m_agvActionFlag=1;
      sleep(1); //was 3 sec
      writeData(4,0);
      m_prevAcrShelf=0;
      m_operationFlag=1;
    }
    /**
     * @brief once the extractor reaches the height for scanning the qrcode on carton box register 64 =1 
     * @brief scanner will be triggered for scanning
     * 
     */
    else if(p_modbusCommunicator->readRegister(69) == 1 &&  m_agvActionFlag==1 && m_barcodeTriggerStatusFlag==1)
    {
      if(m_triggerCount>=m_triggerThreshold)
       {
        failureFeedback(m_prevUniqueID,0);
        m_failStatusFlag=1;
       }
        ROS_WARN("scannning");
       std_msgs::Bool triger;
       triger.data=1;
       scannerTrigger.publish(triger);

       sleep(1);

       m_triggerCount++;
      // writeData(5,2);

    }
    /**
     * @brief checks whether read data is same as wms data
     * 
     */
   // if(m_scannedData==m_rackNo  && m_agvActionFlag==1 && m_failStatusFlag!=1)
//edited to get rid of scanning
    if( m_agvActionFlag==1 && m_failStatusFlag!=1)
    
{
      m_triggerCount=0;
      m_agvActionFlag=0;
      writeData(5,2);      //bin  scanning complete 
      writeData(6,0);   
      m_prevAcrShelf=0;
      m_extractorStartFlag=1;
      std_msgs::Bool triger;
      triger.data=0;
      scannerTrigger.publish(triger); 
      sleep(1);
      m_lastUniqueID=m_prevUniqueID;     
    }
  }
  




}


/**
 * @brief for checking the availability of parameters
 * @brief returns true if all the parameters exists or return false
 */

bool HigherLevelInterface::checkParameters()
{
  ROS_INFO("checking existance of parameters");
  if(p_modbusCommunicator->nh.hasParam("/HigherLevelInterface/update_rate")!=1)
  {
    ROS_WARN("updateRate param misssing");
    return 0;
  }
 else if(p_modbusCommunicator->nh.hasParam("/HigherLevelInterface/subscriberTopic")!=1)
  {
    ROS_WARN("subscriberTopic param misssing");
    return 0;
  }
else  if(p_modbusCommunicator->nh.hasParam("/HigherLevelInterface/publisherTopic")!=1)
  {
    ROS_WARN("publisherTopic param misssing");
    return 0;
  }
else  if(p_modbusCommunicator->nh.hasParam("/HigherLevelInterface/numberOfReadRegisters")!=1)
  {
    ROS_WARN("numberOfReadRegisters param misssing");
    return 0;
  }
else  if(p_modbusCommunicator->nh.hasParam("/HigherLevelInterface/tasksFileName")!=1)
  {
    ROS_WARN("tasksFileName param misssing");
    return 0;
  }
else if(p_modbusCommunicator->nh.hasParam("/HigherLevelInterface/triggerFailureThreshold")!=1)
{
    ROS_WARN("triggerFailureThreshold param misssing");
    return 0; 
}

else
  {
    ROS_INFO("all parameters available");
    return 1;
    
  }
}

/**
 * @brief readParameters definition
 * 
 */
int HigherLevelInterface::readParameters()
{
 ROS_INFO("reading higher level interface parameters");
  
  
 p_modbusCommunicator->nh.getParam("/HigherLevelInterface/update_rate",m_updateRate);
 
 p_modbusCommunicator->nh.getParam("/HigherLevelInterface/subscriberTopic", m_subscriberTopic);
//  ROS_INFO(m_subscriberTopic);

 p_modbusCommunicator->nh.getParam("/HigherLevelInterface/publisherTopic",m_publisherTopic);
  
 p_modbusCommunicator->nh.getParam("/HigherLevelInterface/numberOfReadRegisters", m_numberOfReadRegisters);
 p_modbusCommunicator->nh.getParam("/HigherLevelInterface/tasksFileName", m_tasksFileName);

 p_modbusCommunicator->nh.getParam("/HigherLevelInterface/triggerFailureThreshold", m_triggerThreshold);
 return 0;
}

/**
 * @brief defintion for readData()
 * 
 */
int HigherLevelInterface::readLifterFeedbackData()
{


  p_lifterFeedbackData=p_modbusCommunicator->readData(m_numberOfReadRegisters);
  
  //  int k=60;
  //  for(int i=0;i<m_numberOfReadRegisters;i++)
  //  {

  //    ROS_INFO("register %d  ---> %d",k,p_lifterFeedbackData[i]);
  //    k++;
  //  }
if(m_operationFlag==1)
{
   
    if((p_lifterFeedbackData[2]==1 || p_lifterFeedbackData[8]==1))
     {
	//ROS_WARN("LIFTER FEEDBACK RECIVED READY FOR NEXT LOCATION last-id:  %s , current-id: %s reached_flag : %d ",m_lastUniqueID.c_str(),m_prevUniqueID.c_str(),m_reachedFlag);
      if(m_lastUniqueID==m_prevUniqueID && m_reachedFlag==1)
      {
	      m_lifterFeedbackData.unique_id=m_uniqueId;
        m_lastUniqueID=m_uniqueId;
        m_lifterFeedbackData.status=1;
	      m_lifterFeedbackData.failStatus=0;
	      m_lifterFeedbackData.acr_shelf=0;
        m_lifterFeedbackData.action=m_action;
        m_lastAction=m_action;
        



        lifterFeedback.publish(m_lifterFeedbackData);
        m_agvStopFlag=0;
	      if(m_action==1)
	      {
	        m_lifterFeedbackFlag=0;
        }
	      else if(m_action==0)
        {
	        m_lifterFeedbackFlag=1;
            m_operationFlag=0;
        }

      }


      else
      {

      writeData(1,0);
      writeData(2,0);
      writeData(3,0);
      writeData(4,0);
      writeData(5,0);
      writeData(6,0);


      }



     }

   else if(  p_lifterFeedbackData[3]  !=0 && m_prevAcrShelf != p_lifterFeedbackData[3] && m_action !=0 && m_lifterFeedbackFlag!=1)
   {
      //ROS_ERROR("updating acr shelf data  %d ,with uniqueID  %s",p_lifterFeedbackData[3],m_lastUniqueID);
      m_prevAcrShelf=p_lifterFeedbackData[3];
      m_lifterFeedbackData.unique_id=m_lastUniqueID;
      m_lifterFeedbackData.action=m_lastAction;
      m_lifterFeedbackData.acr_shelf=p_lifterFeedbackData[3];
      m_lifterFeedbackData.failStatus=0;
      lifterFeedback.publish(m_lifterFeedbackData);
      m_lifterFeedbackFlag=1;
      m_operationFlag=0;
      writeData(1,0);
      writeData(2,0);
      writeData(3,0);
      writeData(4,0);
      writeData(5,0);
      writeData(6,0);
      
            
   }
}
if(m_robotMovementFlag==1 && p_modbusCommunicator->readRegister(60) == 1 && p_modbusCommunicator->readRegister(66) == 1)
     {
      writeData(1,0);
      writeData(2,0);
      writeData(3,0);
      writeData(4,0);
      writeData(5,0);
      writeData(6,0);
      writeData(7,3);
     }
   else if(m_robotMovementFlag==0)
     {
      writeData(7,0);
     }

}
/**
 * @brief definition for writeData();
 * 
 */
int HigherLevelInterface::writeData(int m_writeRegistornumber,int m_writeValue)
{
    p_modbusCommunicator->writeDataToRegister(m_writeRegistornumber,m_writeValue);
}
/**
 * @brief definition for write array data
 * 
 */
int HigherLevelInterface::writeArrayData(int m_writeRegstornumber,char m_arrayData[],int bytes)
{
  p_modbusCommunicator->writeDataToRegisters(m_writeRegstornumber,m_arrayData,bytes);
}
/**
 *@brief main function
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "modbus_communicator");
  HigherLevelInterface HigherLevelInterface;
  ros::Rate loop_rate(HigherLevelInterface.m_updateRate); 
  
  while (ros::ok())
  {
   HigherLevelInterface.sendDataToExtractor();
   HigherLevelInterface.readLifterFeedbackData();
   ros::spinOnce();
   loop_rate.sleep();
  }

  

  return 0;
}
