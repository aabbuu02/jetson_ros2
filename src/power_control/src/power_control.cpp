#include "power_control.hpp"

/**
 * @brief Construct a new Power Control:: Power Control object
 * 
 * @param nh The node handle from the main function
 */
PowerControl::PowerControl(ros::NodeHandle &nh):
    nh(nh), nhp("~")
{
    getParameters();

    //Service servers
    std::stringstream sysTopicStringStream;
    sysTopicStringStream<<"power_control/"<<(m_isMasterSystem ? "master" : "slave")<<"/"<<m_systemName<<"/";
    poweroffSrvServer = nh.advertiseService(sysTopicStringStream.str()+"/poweroff", &PowerControl::poweroffSrvCallback, this);
    rebootSrvServer = nh.advertiseService(sysTopicStringStream.str()+"/reboot", &PowerControl::rebootSrvCallback, this);

    //Dynamic reconfigure
    pwrCtrlCfgCallbackType = boost::bind(&PowerControl::powerControlReconfigCallback, this, _1, _2);
    pwrCtrlDynCfgServer.setCallback(pwrCtrlCfgCallbackType);

    if(!m_isMasterSystem) return;

    //Service clients
    for(size_t i = 0; i < m_slaveSystemNames.size(); ++i)
    {
        std::stringstream slaveTopicStringStream;
        slaveTopicStringStream<<"power_control/slave/"<<m_slaveSystemNames.at(i)<<"/";
        
        ros::ServiceClient client;

        //For poweroff service clients
        client = nh.serviceClient<std_srvs::Trigger>(slaveTopicStringStream.str()+"/poweroff");
        slavePowerOffSrvClients.push_back(client);

        //For reboot service clients
        client = nh.serviceClient<std_srvs::Trigger>(slaveTopicStringStream.str()+"/reboot");
        slaveRebootSrvClients.push_back(client);
    }

    waitForServices();
}

/**
 * @brief Get the parameters from the ROS parameters server
 * 
 */
void PowerControl::getParameters()
{
    //Current system settings
    nhp.param<std::string>("system_name", m_systemName, "master_system");
    nhp.param("is_system_master", m_isMasterSystem, false);

    ROS_INFO_STREAM("System name: "<<m_systemName);

    if(!m_isMasterSystem) 
    {
        ROS_WARN_STREAM("Current system is set to be slave for power control! Slave(s) configuration will be ignored.");
        return;
    }

    ROS_WARN_STREAM("Current system is set to be master for power control");

    // Slave settings

    if(!nhp.hasParam("slave_system_names"))
    {
        ROS_WARN_STREAM("No slave systems are configured. This warning can be ignored if it was intentional");
        return;
    }

    XmlRpc::XmlRpcValue symbol;
    if(!nhp.getParam("slave_system_names", symbol))
    {
        ROS_ERROR_STREAM("Failed to read slave system names from parameters server");
        return;
    }

    if(symbol.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_INFO_STREAM("Configured slave system(s):");
        for(size_t i = 0; i < symbol.size(); ++i)
        {   
            
            m_slaveSystemNames.push_back(symbol[i]);
            ROS_INFO_STREAM(i<<": "<<m_slaveSystemNames.at(i));
        }
    }
    else
    {
        ROS_ERROR_STREAM("Cannot read slave system names as it is not specified in an array");
    }

    nhp.param("enable_system_power_control", m_enableSystemPwrCtrl, false);
    nhp.param("enable_slave_power_control", m_enableSlavePwrCtrl, false);

    ROS_INFO_STREAM("Enable current systems power control: "<<std::boolalpha<<m_enableSystemPwrCtrl);
    ROS_INFO_STREAM("Enable slave system(s) power control: "<<std::boolalpha<<m_enableSlavePwrCtrl);
}

/**
 * @brief Wait for the slave system(s) poweroff and reboot ROS servicess 
 * 
 */
void PowerControl::waitForServices() 
{
    if(slavePowerOffSrvClients.size() == 0) return;

    ROS_INFO_STREAM("Waiting for required slave system power control services");
    for(size_t i = 0; i < slavePowerOffSrvClients.size(); ++i)
    {
        slavePowerOffSrvClients.at(i).waitForExistence();
        slaveRebootSrvClients.at(i).waitForExistence();
    }
    ROS_INFO_STREAM("Required slave power control services are now available!");
}

/**
 * @brief The main run method to execute the slave's poweroff and reboot service calls along with
 * the current systems poweroff and reboot
 * 
 */
void PowerControl::run()
{
    ros::Rate rate(5.0f);

    while(ros::ok)
    {
        switch(m_powerCtrlCmd)
        {
            case WAIT: break;

            case POWEROFF:
            {
                if(poweroffSlaveSystems())
                {
                    ROS_WARN_STREAM("Powering down system "<<m_systemName);
                    powerOffSystem();
                }
                else
                {
                    ROS_ERROR_STREAM("Powering down of slave systems failed");
                }

                m_powerCtrlCmd = WAIT;
                
                break;
            }

            case REBOOT:
            {
                if(rebootSlaveSystems())
                {
                    ROS_WARN_STREAM("Rebooting system"<<m_systemName);
                    rebootSystem();
                }
                else
                {
                    ROS_ERROR_STREAM("Rebooting of slave systems failed");
                }

                m_powerCtrlCmd = WAIT;
                
                break;
            }

            default: 
            {
                ROS_ERROR_STREAM_THROTTLE(1, "Unknown power control command"); 
                break;
            }
        }
        
        ros::spinOnce();
        rate.sleep();
    }
}

/**
 * @brief Poweroff service call used in master and slave mode
 * 
 * @param req Trigger request
 * @param res Trigger response
 * @return true If the serive was executed successfully
 */
bool PowerControl::poweroffSrvCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    ROS_WARN_STREAM("Poweroff "<<m_systemName<<" system service called");

    m_powerCtrlCmd = POWEROFF;

    res.success = true;
    res.message = "Powering down "+m_systemName;

    return true;
}

/**
 * @brief Reboot service call used in master and slave mode
 * 
 * @param req Trigger request
 * @param res Trigger response
 * @return true If the service was executed successfully
 */
bool PowerControl::rebootSrvCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    ROS_WARN_STREAM("Reboot "<<m_systemName<<"system service called");

    m_powerCtrlCmd = REBOOT;

    res.success = true;
    res.message = "Rebooting "+m_systemName;
    return true;
}

/**
 * @brief To poweroff the current system
 * 
 * @return true If power off was successful or if it was not enabled
 * @return false If system power off failed
 */
bool PowerControl::powerOffSystem()
{
    if(!m_enableSystemPwrCtrl)
    {
        ROS_WARN_STREAM(m_systemName<<" cannot be powered off as system power control is disabled");
        return true;
    }

    sync(); //Prevent data loss by commiting changes to disk
    
    int retVal = system("sudo poweroff");

    if(retVal < 0)
    {
        ROS_ERROR_STREAM("System poweroff failed with error: "<<std::strerror(errno));
        return false;
    }

    return true;
}

/**
 * @brief To reboot the current system
 * 
 * @return true If reboot was successful or if it was not enabled
 * @return false If system reboot failed
 */
bool PowerControl::rebootSystem()
{
    if(!m_enableSystemPwrCtrl)
    {
        ROS_WARN_STREAM(m_systemName<<" cannot be rebooted as system power control is disabled");
        return true;
    }

    sync(); //Prevent data loss by commiting changes to disk
	
    int retVal = system("sudo reboot");

    if(retVal < 0)
    {
        ROS_ERROR_STREAM("System reboot failed with error: "<<std::strerror(errno));
        return false;
    }

    return true;
}

/**
 * @brief Function to poweroff slave system(s) by making ROS service calls to them
 * 
 * @return true If all service calls to slave(s) were made successfully
 * @return false If any of the service calls to slave(s) failed
 */
bool PowerControl::poweroffSlaveSystems()
{
    if(!m_isMasterSystem) return true;

    if(!m_enableSlavePwrCtrl)
    {
        ROS_WARN_STREAM("Slave poweroff cannot be done as slave power control is disabled");
        return true;
    }

    std_srvs::Trigger poweroffTrigger;

    bool slave_poweroffs_done = true;
    for(size_t i = 0; i < slavePowerOffSrvClients.size(); ++i)
    {
        if(!slavePowerOffSrvClients.at(i).call(poweroffTrigger))
        {
            ROS_ERROR_STREAM("Slave system poweroff failed for "<<m_slaveSystemNames.at(i));
            slave_poweroffs_done = false;
        }
        else
        {
            ROS_WARN_STREAM(poweroffTrigger.response.message);
        }
    }

    return slave_poweroffs_done;
}

/**
 * @brief Function to reboot slave system(s) by making ROS service calls to them
 * 
 * @return true If all service calls to slave(s) were made successfully
 * @return false If any of the service calls to slave(s) failed
 */
bool PowerControl::rebootSlaveSystems()
{
    if(!m_isMasterSystem) return true;

    if(!m_enableSlavePwrCtrl)
    {
        ROS_WARN_STREAM("Slave reboot cannot be done as slave power control is disabled");
        return true;
    }

    std_srvs::Trigger rebootTrigger;

    bool slave_reboots_done = true;
    for(size_t i = 0; i < slaveRebootSrvClients.size(); ++i)
    {
        if(!slaveRebootSrvClients.at(i).call(rebootTrigger))
        {
            ROS_ERROR_STREAM("Slave system reboot failed for "<<m_slaveSystemNames.at(i));
            slave_reboots_done = false;
        }
        else
        {
            ROS_WARN_STREAM(rebootTrigger.response.message);
        }
    }

    return slave_reboots_done;
}

/**
 * @brief Node's reconfigure call back
 * 
 * @param config The available configurations
 * @param level The level value used to set value in sections
 */
void PowerControl::powerControlReconfigCallback(power_control::PowerControlConfig &config, uint32_t level)
{
    m_enableSystemPwrCtrl = config.enable_system_power_control;
    m_enableSlavePwrCtrl = config.enable_slave_power_control;

    ROS_INFO_STREAM("System power control set to: "<<std::boolalpha<<m_enableSystemPwrCtrl);
    ROS_INFO_STREAM("Slave power control set to: "<<std::boolalpha<<m_enableSlavePwrCtrl);
}

/**
 * @brief The main entry point for the power control node
 * 
 * @param argc Only used for ROS commands
 * @param argv Only used for ROS commands
 * @return int Returns 0 on successful exit
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "power_control");
    ros::NodeHandle nh;

    PowerControl powerControl(nh);
    powerControl.run();

    return EXIT_SUCCESS;
}
