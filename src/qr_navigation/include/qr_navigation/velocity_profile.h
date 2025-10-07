#ifndef VELOCITY_PROFILE_H
#define  VELOCITY_PROFILE_H


#include "pose_struct.h"

class VelocityProfile
{

	public:
		VelocityProfile(float vel_limit);
		~VelocityProfile();
		float max_velocity,path_length,distance_traveled,dist_to_goal;
		float max_n_vel {0.0};	
		float last_pos_x{0.0}, last_pos_y{0.0};
		bool trigger {false};
		double acc_point;
		double dec_point;
		double acceleration,deceleration;
		double linear_velocity {0.0};
		float acc_constant;
		float dec_constant;
		float inertia_constant;
		float dis_to_max_vel {0.0} , distance_for_acc{0.0}, distance_for_dec{0.0};
		float stabilize_dis;
		float slowdown_dis;
		float acceleration_const{0.0};
		bool goal_reached {false};
		double pull_velocity {0.06};
		float vmax;
	        void find_distance_segment(float path_dis);
		float Find_linear_velocity(const pose_struct::PathData &path_data,
				const pose_struct::PoseEuler &robot_pose,
				const float current_linear_velocity);

		double find_deceleration(double odom_max_vel);
		
		double find_acceleration();
		void Update_params(float vmax_, float acc_constant_, float dec_constant_,float inertia_constant_,float acceleration_const);

};












#endif
