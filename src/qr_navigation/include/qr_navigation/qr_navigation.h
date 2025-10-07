#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "pose_struct.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float32.h>
#include  "pure_pursuit.h"
#include  "velocity_profile.h"
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "rotation_controller.h"
#include <qr_navigation/goal_msg.h>
#include <dynamic_reconfigure/server.h>
#include <qr_navigation/ParameterConfig.h>
#include <geometry_msgs/Pose2D.h>
#include <anscer_msgs/SafetyFields.h>


class QrNavigation
{
	public:
		QrNavigation();
		~QrNavigation();
		ros::Time time;
		ros::NodeHandle nh;
		ros::Subscriber odom_subscriber;
		ros::Subscriber path_subscriber;
		ros::Subscriber path_sequance_subscriber;
		ros::Subscriber obstacle_detection_subscriber;
        ros::Subscriber pause_robot;
		ros::Publisher  cmd_vel_pub;
		ros::Publisher  extractor_homing_pub;
		ros::Publisher  robot_pose_pub;
		ros::Publisher distance_to_goal_pub;
		ros::Publisher  result_pub;
		ros::Publisher	lateral_pub;
        ros::Publisher look_ahead_pub;
		ros::Subscriber robot_init;
        ros::Publisher robot_init_status;
        ros::Publisher robot_pause_status;

		pose_struct::PoseEuler robot_pose;
		pose_struct::PoseEuler start_pose;
		pose_struct::PoseEuler goal_pose;
		pose_struct::Matrix path_data;
		pose_struct::Matrix smooth_path_data;
		std_msgs::Float32 dist_to_goal;

		pose_struct::PathData start_and_goal_data;
		geometry_msgs::Twist control_velocity;

		pose_struct::ControlInput control_input;
		PurePursuit pure_pursuit;
		VelocityProfile velocity_profile;
		std::string last_goal_position,operation;
		bool is_initialized;
        bool is_paused;
		double m_homing_distance{0.60};
		bool is_extractor_home{false};
		double tolarance_distance;
		bool caution_front {false};
		bool caution_rear {false};
		bool stop_front {false};
		bool stop_rear {false};
		bool reached_position{false};
		bool old_goal{false};
 		double last_distance{0.0};
		double distance_to_goal{0.0};
        float goal_precision{0.005};

		RotationController rotation_control;
		bool in_place_rotation,new_goal;
		double desired_rot_angle;
		double position_tolerance {0.02};
		double reverse_max_speed {0.3};
		double angular_tolerance{0.05};
		double roll, pitch, yaw, goal_roll, goal_pitch,goal_yaw,start_roll,start_pitch,start_yaw;
		double find_desired_rot_angle(double current_theta, double desired_theta);
		std_msgs::String goal_reached;
		std_msgs::Float32 lateral_shift_data;
        std_msgs::Float32 look_ahead_data;
		std_msgs::Bool homing_data;
        std_msgs::Bool robot_init_status_;
        std_msgs::Bool robot_pause_status_;
		geometry_msgs::Pose2D pose_data;
		int m_inside_count{0};
		int path_array_size;	
		float current_linear_velocity,angular_velocity,linear_velocity,max_angular_vel;
        	void Odom_callback(const nav_msgs::OdometryConstPtr &odom_msg);
		void run();
		void Path_callback(const nav_msgs::PathConstPtr &path_msg);				
				
		void Goal_path_callback(const qr_navigation::goal_msg &Goal_path_msg);
		void Robot_init(std_msgs::Bool initialise_data);

		pose_struct::Matrix Path_smoothening(pose_struct::PoseEuler start,pose_struct::PoseEuler goal,double change_in_value,int x_);
		void Obstacle_node(const anscer_msgs::SafetyFields &obstacle_msg);

		void callback(qr_navigation::ParameterConfig &config, uint32_t level);
		void Update_params(double x_position_tolerance,double x_angular_tolerance,double reverse_max_speed_,double homing_distance);

		float check_operation(float m_linear_velocity);
		double desired_theta_correction(double desired_theta_);

        void Pause_robot(std_msgs::Bool pause_data);
		float find_distance_to_goal();



};

