#include <iostream>   
#include "velocity_profile.h"
#include <math.h>
#include <ros/console.h>


VelocityProfile::VelocityProfile(float vel_limit)
{

	path_length  = 0.0;
	distance_traveled= 0.0;
	dist_to_goal = 0.0;
	stabilize_dis = 0.8;
	slowdown_dis = 0.8;
	acc_constant = 0.2;
	dec_constant = 0.2;
	inertia_constant =0.8;
	acceleration_const = 0.2;
	vmax =vel_limit;
}

VelocityProfile::~VelocityProfile(){}




void VelocityProfile::Update_params(float vmax_, float acc_constant_, float dec_constant_,float inertia_constant_,float acceleration_)
{
	ROS_INFO("Updated params %f, %f, %f, %f ",vmax,acc_constant_,dec_constant_,inertia_constant_);
	vmax = vmax_;
	acc_constant = acc_constant_;
	dec_constant = dec_constant_;
	inertia_constant=inertia_constant_;
	acceleration_const = acceleration_;

}


void VelocityProfile::find_distance_segment(float path_dis)
{
	distance_for_acc = path_dis*(dec_constant/(acc_constant+dec_constant));
	
	distance_for_dec = path_dis*(acc_constant/(acc_constant+dec_constant));
	dis_to_max_vel = pow(vmax,2)/(2*acc_constant);
	ROS_INFO("distance_for_acc: %f , distance_for_dec: %f  , dis_to_max_vel: %f ",distance_for_acc,distance_for_dec,dis_to_max_vel);
	if (dis_to_max_vel > distance_for_acc)
	{
		stabilize_dis = distance_for_acc;
		slowdown_dis = distance_for_dec;
	}
	else
	{
		stabilize_dis = dis_to_max_vel;
		slowdown_dis = pow(vmax,2)/(2*dec_constant*inertia_constant);
	}
       ROS_INFO("slowdown_dis %f ---> ",slowdown_dis);
}

double VelocityProfile::find_acceleration()
{
	float acc;
		
	acc= pow(vmax, 2) / (2 * stabilize_dis);
	//ROS_INFO("accelertion %f, vmax : %f , stabilize_dis: %f ",acc,vmax,stabilize_dis);
	return acc;
}

double VelocityProfile::find_deceleration(double odom_max_vel)
{
	float dec = pow(odom_max_vel*0.03, 2) / (2 * slowdown_dis);
	//	std::cout << "deceleration is " << dec << std::endl;
	//ROS_INFO("deaccelerating");
	return dec;
}

float VelocityProfile::Find_linear_velocity(const pose_struct::PathData &path_data,
		const pose_struct::PoseEuler &robot_pose,
		float achieved_max_vel)
{
	//ROS_INFO("achived_max_vel -----> %f",achieved_max_vel);
	//ROS_INFO("pose_x : %f , pose_y : %f , pose_theta : %f ",robot_pose.position.x,robot_pose.position.y,robot_pose.orientation.yaw);
	path_length = sqrt(pow((path_data.goal_pose.position.x) - (path_data.init_pose.position.x), 2) + (pow((path_data.goal_pose.position.y) - (path_data.init_pose.position.y), 2)));

	distance_traveled = sqrt(pow((robot_pose.position.x - path_data.init_pose.position.x), 2) + pow((robot_pose.position.y - path_data.init_pose.position.y), 2));
	
	dist_to_goal = sqrt(pow((robot_pose.position.x - path_data.goal_pose.position.x), 2) + pow((robot_pose.position.y - path_data.goal_pose.position.y), 2));
	
	//ROS_INFO("path_length: %f ,distance_travelled: %f , dist_to_goal: %f ", path_length,distance_traveled,dist_to_goal);
		//ROS_INFO("achived_max_vel 1 -----> %f",achieved_max_vel);


	find_distance_segment(path_length);

	acc_point = stabilize_dis;
	////ROS_INFO("acc_point %f ",acc_point);
	dec_point = abs(path_length - slowdown_dis); 
    if(dec_point < path_length/2.5 )
    {
        ROS_INFO("dec_point : ---> %f ",dec_point);
        dec_point = path_length/2;
    }
	acceleration = find_acceleration();

	deceleration = find_deceleration(vmax);
	//ROS_INFO("achived_max_vel2 -----> %f",achieved_max_vel);

	ROS_INFO("path_length:  %f, slowdown_dis: %f , dec_point :%f ,acc_point : %f, distance_traveled %f :",path_length,slowdown_dis,dec_point,acc_point,distance_traveled);
	if (distance_traveled <= dec_point)
	{
		//	ROS_INFO("linear_velocity 3  -----> %f",linear_velocity);

		trigger = false;
		//if(distance_traveled < 0.1)
//{
	//		//ROS_WARN("INITIAL STATE");
	//		linear_velocity = 0.1;
	//		ROS_INFO("linear_velocity 4  -----> %f",linear_velocity);

	//	}
	//	else
	//	{
///std::cout << "acc" << std::endl;
				//	ROS_INFO("linear_velocity 5 -----> %f ,distance_traveled --->%f",linear_velocity,distance_traveled);
			//linear_velocity = linear_velocity + (acceleration_const * 0.025);

            linear_velocity = fabs(achieved_max_vel) + (acceleration_const * 0.04);

	//		linear_velocity = sqrt(2 * acc_constant * (distance_traveled));//distance_traveled);//acceleration * distance_traveled ;//
				//ROS_INFO("linear_velocity 6  -----> %f",linear_velocity);

		//linear_velocity = (vmax/pow(stabilize_dis,2))*pow(distance_traveled, 2);
			//ROS_INFO("linear_velocity : %f ",linear_velocity);
	//			ROS_INFO("Linear velocity -->%f , distance_traveled --> %f", achieved_max_vel,distance_traveled);

			if(linear_velocity < 0.15)
				linear_velocity = 0.15;

        if (abs(dist_to_goal) < 0.5 && linear_velocity > 0.2)
        {
            linear_velocity = 0.2;   
        } 
		//}
//				ROS_INFO("linear_velocity 7  -----> %f",linear_velocity);

	}

/*
	else if ((distance_traveled > acc_point) && (distance_traveled <= dec_point))
	{
		//std::cout << "const" << std::endl;	
				//	ROS_INFO("linear_velocity 8 -----> %f",achieved_max_vel);

		linear_velocity =abs(linear_velocity); //abs(achieved_max_vel);//sqrt(2*acc_constant*stabilize_dis);//vmax ;
		if(abs(achieved_max_vel) < 0.1) 
			{
			linear_velocity = 0.1;
			}
		//ROS_INFO("linear_velocity: ---> %f",linear_velocity);
		//tune_linear_vel(shift);
		trigger = false;
		//			ROS_INFO("linear_velocity 9 -----> %f",linear_velocity);

	}
*/
	else
	{
		if (!trigger)
        {
			max_n_vel = achieved_max_vel;
			last_pos_x = robot_pose.position.x;
			last_pos_y = robot_pose.position.y;
			trigger = true;
		}
	//	float length_seg = sqrt(pow((robot_pose.position.x - last_pos_x), 2) + pow((robot_pose.position.y - last_pos_y), 2));

		//linear_velocity = linear_velocity - (dec_constant*inertia_constant*dist_to_goal);
			//ROS_INFO("Linear velocity -->%f", achieved_max_vel);
		//				ROS_INFO("linear_velocity 10  -----> %f",linear_velocity);

			linear_velocity = sqrt(2 * dec_constant * (dist_to_goal));
			if(linear_velocity > fabs(achieved_max_vel) && fabs(achieved_max_vel) >0.1)
			{
				linear_velocity = fabs(achieved_max_vel);
			}
            if ((abs(dist_to_goal) < 0.5) && (linear_velocity > 0.2) && (abs(dist_to_goal) > 0.1))
            {
            linear_velocity = 0.2;
            }

        	if(abs(dist_to_goal) < 0.1) // && abs(dist_to_goal)>0.05) 
				{
					linear_velocity = 0.05;
					} 
						ROS_INFO("linear_velocity 11  -----> %f",linear_velocity);

            //if(abs(dist_to_goal < 0.05) )
              //  {

                //    linear_velocity = 0.01;
               // }
			//linear_velocity = (sqrt(pow(achieved_max_vel,2) - ( 2 * dec_constant * dist_to_goal )));//distance_traveled);//acceleration * distance_traveled ;//

	//	std::cout << "\n linear_dec vel	" << linear_velocity <<"  "<<dist_to_goal <<"\n";
		//if(linear_velocity <= pull_velocity)
		//	linear_velocity = pull_velocity;
	//	ROS_INFO("deaccelarating :%f ", linear_velocity);
		
	}
	if(linear_velocity > vmax)
		linear_velocity = vmax;
	/*if(linear_velocity > 0.001)
	  linear_velocity = 0.06;
	  else */
	if(linear_velocity <= 0.001)
	linear_velocity = 0.001;
	if(achieved_max_vel < 0)
		achieved_max_vel = 0;
	//}
	//if (goal_reached)
	//linear_velocity = 0.0;
	if(isnan(linear_velocity))
	linear_velocity = 0.0;       
	//if(linear_velocity > achieved_max_vel + .25)
	//linear_velocity = achieved_max_vel + .25 ;
	if (path_length - distance_traveled < -0.1)
	linear_velocity = 0.0;

	ROS_INFO("linear_velocity 12  -----> %f",linear_velocity);


	return linear_velocity;
}
