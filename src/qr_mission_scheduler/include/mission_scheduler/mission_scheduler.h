#include <ros/ros.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include "qr_mission_scheduler/mission_schedulerAction.h"
#include "qr_mission_scheduler/GetGoal.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <qr_navigation/goal_msg.h>
#include <qr_mission_scheduler/goalMsg.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include "graph_msgs/GetGraphPlan.h"
#include "graph_msgs/GetGraphPlanResponse.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include  "sensor_msgs/BatteryState.h"



class MissionScheduler
	{
	
	
	protected:
			ros::NodeHandle nh;
			actionlib::SimpleActionServer<qr_mission_scheduler::mission_schedulerAction> as;
			qr_mission_scheduler::mission_schedulerFeedback feedback;
			qr_mission_scheduler::mission_schedulerActionGoal goal;
			qr_mission_scheduler::mission_schedulerResult action_result;
			
	public:
		MissionScheduler(std::string name);
		~MissionScheduler(void);
		ros::Subscriber battery_sub;
        ros::Subscriber battery_sub_;
		
        ros::Subscriber robot_odom_sub;
		ros::Subscriber robot_goal_result_sub;
		
		ros::Publisher charge_battery;
		ros::Publisher goal_msg_pub;
		ros::Publisher robot_status;
		
		
		ros::ServiceClient path_client;
		ros::ServiceServer m_goal_server;

		
		graph_msgs::GetGraphPlanResponse response_data;
		nav_msgs::Path planned_path_data;
		geometry_msgs::PoseStamped planned_path_poses;
		geometry_msgs::PoseStamped planned_path_poses_p,planned_path_poses_charge;


		//this vector contain the poses in order of turing that robot has to make
		std::vector<std::vector<std::vector<double>>> m_stripped_path_poses;
		std::vector<std::vector<double>> path_data_vec_2D;
		std::vector<double> path_data_vec_1D;
		std::vector<std::vector<double>> straight_line_vector;
		
		
		std::vector<double>  path_filtered_data_vec_1D;
		std::vector<std::vector<double>> path_filtered_data_vec_2D;

		double roll{0};
		double pitch{0};
		double yaw{0};
		double m_robot_odom_yaw{0};
		double previous_yaw{0};
		double next_yaw{0};
		int stripped_path_count{0};
		int path_data_count{0};
		int MAX_CHARGE{98};

		std::string action_name_{};
		qr_mission_scheduler::goalMsg goalValue{};
		qr_navigation::goal_msg pathData{};
		std_msgs::Bool m_robot_status_data;
		nav_msgs::Odometry m_robot_pose_data;
		std::string m_goal_result{};
		int m_pos_num{100};
		std::string m_reached_pos{0};
		bool m_robot_on_duty{false};
		bool m_undock{false};
		double goal_yaw_for_charging{};
		bool charging_mission_intiated{false};
		
		qr_mission_scheduler::goalMsg	 m_goal_data{};
		//qr_mission_scheduler::mission_schedulerGoal *m_goal_data;

		void execute_goal(const qr_mission_scheduler::goalMsg &goal_data);
		void executeCB(const qr_mission_scheduler::mission_schedulerGoalConstPtr &goal);
		void OdomCB(const nav_msgs::Odometry &odom);
		void GoalResultCB(const std_msgs::String &goal_result);
		//bool goalServerCB( qr_mission_scheduler::GetGoal::Request &req,  qr_mission_scheduler::GetGoal::Response &res );
		
		graph_msgs::GetGraphPlan get_path(const int source_id, const int target_id);
		void split_path(graph_msgs::GetGraphPlanResponse &response_data,std::vector<std::vector<std::vector<double>>> &stripped_path_poses);
		void pub_path(std::string operation,std::vector<std::vector<std::vector<double>>>  &stripped_path_poses,bool Dock, std::string Type );

		void Charge_battery();
		bool get_path_plan(const qr_mission_scheduler::goalMsg &goal);

		void reached_pos_check(int m_pos_num, qr_navigation::goal_msg & pathData );
	
		//void goalServerCB(qr_mission_scheduler::GetGoal::Request & req, qr_mission_scheduler::GetGoal::Response &res);


		void ignore_orientaion(std::vector<std::vector<std::vector<double>>>  &stripped_path_poses, bool robot_orientation=false);
		void reverse_orientation(std::vector<std::vector<std::vector<double>>>  &stripped_path_poses);
		void check_straight_line(graph_msgs::GetGraphPlanResponse &response_data);//,std::vector<std::vector<std::vector<double>>>  &stripped_path_poses);
		void correct_orientation(std::vector<std::vector<std::vector<double>>>  &stripped_path_poses);
		void check_inplace_rotation(std::vector<std::vector<std::vector<double>>>  &stripped_path_poses,bool robot_orientation);
		void charging_path_correction(std::vector<std::vector<std::vector<double>>>  &m_stripped_path_poses);


		float m_battery_percentage{100};
		float m_minimum_battery_threshold{48};

		void preemptCB();
		void goalCB();
		void batteryCB(const std_msgs::Float32 &msg);
        void batteryCBData(const sensor_msgs::BatteryState& msg);

		//void init(ros::NodeHandle &nh);

		
			
	};
