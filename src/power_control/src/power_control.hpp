#include <unistd.h>
#include <linux/reboot.h>
#include <sys/reboot.h>
#include <cerrno>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <xmlrpcpp/XmlRpc.h>
#include <dynamic_reconfigure/server.h>
#include <power_control/PowerControlConfig.h>

class PowerControl
{
    public:

    PowerControl(ros::NodeHandle &nh);
    void run();

    private:

    //Power control command
    enum PowerCtrlCommands
    {
        WAIT = 0,
        POWEROFF = 1,
        REBOOT = 2
    };

    uint8_t m_powerCtrlCmd = WAIT;

    //Current systems settings
    bool m_isMasterSystem;
    bool m_enableSystemPwrCtrl;
    bool m_enableSlavePwrCtrl;
    std::string m_systemName;
    
    //Slave sytem settings
    int m_noOfSlaves;
    std::vector<std::string> m_slaveSystemNames;

    //Nodehandle
    ros::NodeHandle nh, nhp;

    //Service servers
    ros::ServiceServer poweroffSrvServer;
    ros::ServiceServer rebootSrvServer;

    //Service callbacks
    bool poweroffSrvCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
    bool rebootSrvCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

    //Service clients
    std::vector<ros::ServiceClient> slavePowerOffSrvClients;
    std::vector<ros::ServiceClient> slaveRebootSrvClients;

    //Dynamic reconfigure
    dynamic_reconfigure::Server<power_control::PowerControlConfig> pwrCtrlDynCfgServer;
    dynamic_reconfigure::Server<power_control::PowerControlConfig>::CallbackType pwrCtrlCfgCallbackType;

    //Dynamic reconfigure callback
    void powerControlReconfigCallback(power_control::PowerControlConfig &config, uint32_t level);

    //Other functions
    void getParameters();
    void waitForServices();
    bool powerOffSystem();
    bool rebootSystem();
    bool poweroffSlaveSystems();
    bool rebootSlaveSystems();
};