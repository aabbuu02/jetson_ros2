#include "navigation_speed_control.h"
#include <ros/console.h>

/**
 * @brief  Constructor for the NavigationSpeedControl
 */
NavigationSpeedControl::NavigationSpeedControl() : nhp("~")
{
    initializeParameters();
    fieldSub = nh.subscribe(m_fieldTopic.c_str(), 1, &NavigationSpeedControl::fieldCallback, this);
    selectorSub = nh.subscribe(m_selectorSwitchTopic.c_str(), 1, &NavigationSpeedControl::selectorSwitchCallback, this);
    cmdVelSub = nh.subscribe(m_preNavigationTopic.c_str(), 1, &NavigationSpeedControl::preNavigateCallback, this);
    zoneInfoSub = nh.subscribe(m_zoneInfoTopic.c_str(), 1, &NavigationSpeedControl::zoneInfoCallback, this);

    cmdVelPub = nh.advertise<geometry_msgs::Twist>(m_navigationTopic.c_str(), 1);
    ROS_INFO("NavigationSpeedControl Constructor called");
}

/**
 * @brief  Destructor for the BatteryManagement
 */
NavigationSpeedControl::~NavigationSpeedControl()
{
}

/**
 * @brief  Initialize parameters for the NavigationSpeedControl
 */
void NavigationSpeedControl::initializeParameters()
{
    nhp.param<std::string>("controller_type", m_controller, "pure");
    nhp.param<std::string>("field_topic", m_fieldTopic, "lidar_fields");
    nhp.param<std::string>("zone_info_topic", m_zoneInfoTopic, "zone_info");
    nhp.param<std::string>("selector_switch_topic", m_selectorSwitchTopic, "mode_selection");
    nhp.param<std::string>("pre_navigation_topic", m_preNavigationTopic, "pre_navigation/cmd_vel");
    nhp.param<std::string>("navigation_topic", m_navigationTopic, "navigation/cmd_vel");
    int speedValue = 0.7;
    nhp.getParam("speed_factor", m_speedFactor);
    nhp.getParam("speed_value" , speedValue);
    if (m_controller == "pure")
    {
       if(nh.hasParam("qr_navigation_node/forward_max_vel")&& nh.hasParam("/qr_navigation_node/reverse_max_vel"))
        {
            nh.getParam("/qr_navigation_node/forward_max_vel", m_linearVel__m_s);
            nh.getParam("/qr_navigation_node/reverse_max_vel", m_reverseVel__m_s);
            ROS_INFO_STREAM("NAVIGAATION SPEED CONTROL Got Params" << m_linearVel__m_s << " and " << m_reverseVel__m_s);
        }
        else
        {
            ROS_ERROR("NAVIGAATION SPEED CONTROL PARAM NOT FOUND");
            m_linearVel__m_s = speedValue;
            m_reverseVel__m_s = speedValue;
        }

        m_controllerMaxVelocityParam = "forward_max_vel";
        m_controllerReversedVelocityParam = "reverse_max_vel";
    }
    else
    {
        nh.getParam("/tracking_controller/maxLinearVel__m_s", m_linearVel__m_s);
        m_controllerMaxVelocityParam = "maxLinearVel__m_s";
    }
}

/**
 * @brief  Subscriber callback for lidar field information
 */
void NavigationSpeedControl::fieldCallback(const anscer_msgs::FieldStatus &msg)
{
    m_warningCurrentStatus = msg.is_warning;
    if ((m_warningCurrentStatus != m_warningPreviousStatus) && (m_zoneSpeedFactor == 1.0))
    {
        reconfigureParameters(m_warningCurrentStatus);
        m_warningPreviousStatus = m_warningCurrentStatus;
    }
}

/**
 * @brief  Subscriber callback for zone information callback
 */
void NavigationSpeedControl::zoneInfoCallback(const anscer_msgs::ZoneInformation &msg)
{
    m_zoneSpeedFactor = msg.speed_factor;
    // ROS_INFO("Speed factor: %f", m_zoneSpeedFactor);
}

/**
 * @brief  Subscriber callback for selector switch information
 */
void NavigationSpeedControl::selectorSwitchCallback(const std_msgs::Int32 &msg)
{
    m_selectorSwitchStatus = msg.data;
}

/**
 * @brief  Subscriber callback for pre-navigation messages
 */
void NavigationSpeedControl::preNavigateCallback(const geometry_msgs::Twist &msg)
{
    if (m_selectorSwitchStatus == AUTO_MODE)
    {
        geometry_msgs::Twist cmdVel;
        cmdVel = msg;
        cmdVelPub.publish(cmdVel);
    }
}

/**
 * @brief  Reconfigure the velocity parameters based on warning status
 */
void NavigationSpeedControl::reconfigureParameters(bool warningStatus)
{

    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double speedFactor = (warningStatus == 1) ? m_speedFactor : 1.0;

    double_param.name = m_controllerMaxVelocityParam;
    double_param.value = m_linearVel__m_s * speedFactor;
    conf.doubles.push_back(double_param);
    double_param.name = m_controllerReversedVelocityParam;
    double_param.value = m_reverseVel__m_s * speedFactor;
    conf.doubles.push_back(double_param);
    srv_req.config = conf;

    if (m_controller == "pure")
    {
        ros::service::call("/qr_navigation_node/set_parameters", srv_req, srv_resp);
    }
    else if (m_controller == "lyapunov")
    {
        ros::service::call("/tracking_controller/set_parameters", srv_req, srv_resp);
    }
    ROS_DEBUG_NAMED("NavigationSpeedControl", "Dynamic reconfigure called");
}

/**
 * @brief Main function
 */
int main(int argc, char **argv)
{

    ros::init(argc, argv, "navigation_speed_control");
    NavigationSpeedControl navigationSpeedControl;

    ros::Rate rate(50);

    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
    }
    ros::shutdown();
    return 0;
}
