#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include "qr_mission_scheduler/mission_schedulerAction.h"
#include <wms_data/WMSTasks.h>
#include <wms_data/WMSData.h>
#include <lift_action/LiftFeedback.h>
#include <lift_action/LiftAction.h>
#include  "sensor_msgs/BatteryState.h"

#include <qr_mission_scheduler/goalMsg.h>
#include <qr_mission_scheduler/mission_schedulerGoal.h>
#include <std_msgs/Int16.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <std_msgs/Bool.h>

#include <boost/thread.hpp>

typedef actionlib::SimpleActionClient<qr_mission_scheduler::mission_schedulerAction> Client;
ros::Subscriber wms_drop_data_sub;
ros::Subscriber wms_pick_data_sub;

ros::Subscriber lift_feedback_sub;
ros::Subscriber battery_sub;
ros::Publisher lift_action_pub;
ros::Publisher robotLocationChangePub;
ros::Publisher enableBreakPub;
ros::ServiceServer charging_srvr;
bool m_is_battery_low{false};
int m_bin_number{1};
int m_rack_number{0};
std::string m_id{};
bool m_lifter_confirmation{false};
std::string m_lifter_id{};
lift_action::LiftAction m_lift_action_data{};
bool m_reached_goal{false};
bool m_failstatus{false}; 
std::string m_dropping_id{"Dropping"};
int m_picked_count{3};
int m_dropping_pos_count{0};
std::vector<int> droping_poses{5,6,7,8};
std::vector<std::string>conveyor_data{};
bool m_picked_from_conveyor;
std::string last_task_unique_id{};
std::string dropping_last_task_unique_id{};
bool reset_flag_{true};
bool dropped_flag{true};
int m_maximum_pick_size{9};


std::vector<std::string> data_vector;
std::vector<wms_data::WMSData> picking_location_data{};
std::vector<wms_data::WMSData> dropping_location_data{};
std::vector<std::vector<std::string>> pose_data{};
std::vector<std::vector<std::string>> conveyor_pose_data{};
std::vector<int> droppping_location{5,6,7,8};
int starting_location{1};
int source{48};\
float temp_source{1};
int destination{1};	
std::string goal_type {"NORMAL"};
int m_battery_percentage{0};
int m_charge_destination = 0;
bool m_is_charging_requested = false;
bool m_is_charge_initiated = false;
//void batteryCBData(const sensor_msgs::BatteryState& msg);

