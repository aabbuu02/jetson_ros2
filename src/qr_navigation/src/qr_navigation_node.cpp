#include "qr_navigation.h"
#include <ros/console.h>



QrNavigation::QrNavigation():
	velocity_profile(0.2),
	rotation_control(0.3,-0.3,0.8)
{

	std_msgs::Bool data;

	//path_subscriber = nh.subscribe("path",1,&QrNavigation::Path_callback,this);
	path_sequance_subscriber = nh.subscribe("goal_path",1,&QrNavigation::Goal_path_callback,this);
	//obstacle_detection_subscriber = nh.subscribe("obstacle_detection",1,&QrNavigation::Obstacle_node,this);
	// Adding publisher for fms( fleet management) 
	//robot_pose_pub = nh.advertise<geometry_msgs::Pose2D>("pick_robot/robot_1/pose",10);

	odom_subscriber = nh.subscribe("odom",1,&QrNavigation::Odom_callback,this);
	distance_to_goal_pub = nh.advertise<std_msgs::Float32>("disatance_to_goal",1);
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("navigation/cmd_vel", 1);
	result_pub = nh.advertise<std_msgs::String>("/goal_result", 10);
	robot_init_status = nh.advertise<std_msgs::Bool>("/robot_init_status",1);
    robot_pause_status = nh.advertise<std_msgs::Bool>("/robot_pause_status",1);
    lateral_pub = nh.advertise<std_msgs::Float32>("/lateral_shift",10);
	look_ahead_pub = nh.advertise<std_msgs::Float32>("/look_ahead",10);

    extractor_homing_pub = nh.advertise<std_msgs::Bool>("/robot_state",1);
	robot_init = nh.subscribe("robot_initialized",1,&QrNavigation::Robot_init,this);
    pause_robot = nh.subscribe("pause_robot",1,&QrNavigation::Pause_robot,this);
	robot_pose.position.x = 0;
	robot_pose.position.y = 0;
	robot_pose.position.z = 0;
	robot_pose.orientation.yaw = 0;
	
	goal_pose.position.x =0;
	goal_pose.position.y =0;
	goal_pose.position.z =0;
	goal_pose.orientation.yaw = 0;

	start_pose.position.x =0;
	start_pose.position.y =0;
	start_pose.position.z =0;
	start_pose.orientation.yaw = 0;
	
	path_array_size  =0;
	path_data.rows   =1;
	path_data.columns =1;
	
	
	current_linear_velocity = 0.0;
	linear_velocity = 0.0;
	
	start_and_goal_data.init_pose.position.x = 0.0;
	start_and_goal_data.init_pose.position.y = 0.0;
	start_and_goal_data.init_pose.position.z = 0.0;
	start_and_goal_data.goal_pose.position.x = 0.0;
	start_and_goal_data.goal_pose.position.y = 0.0;
	start_and_goal_data.goal_pose.position.z = 0.0;
	start_and_goal_data.goal_pose.orientation.yaw = 0.0;
	in_place_rotation = false;
	new_goal = true;
	operation = "NONE";
	tolarance_distance = 0.02;
	is_initialized = false;
    is_paused = false;
}


QrNavigation::~QrNavigation(){}



void QrNavigation::Robot_init(std_msgs::Bool initialized_data)
	{
		is_initialized = initialized_data.data;
		//ROS_INFO("is_robot_initialized " );
	}

void QrNavigation::Pause_robot(std_msgs::Bool pause_data)
    {
        is_paused = pause_data.data;
    }

void QrNavigation::callback(qr_navigation::ParameterConfig &config, uint32_t level)
{

	velocity_profile.Update_params(config.forward_max_vel,config.acc_constant, config.dec_constant,config.inertia_constant,config.acceleration);

	
	pure_pursuit.Update_params(config.min_lookahead_dist,config.max_lookahead,config.wheel_base,config.kla,config.max_angular_vel);	
	
	Update_params(config.x_position_tolerance,config.x_angular_tolerance,config.reverse_max_vel,config.homing_distance);

	//rot_control.update_params(config.inplace_max_rot_vel,config.control_gain);
	//tracker_.update_params(config.min_lookahead,config.kla,config.kct,config.max_angular_vel_track,config.charge_vel,config.dock_vel);
		
}

void QrNavigation::Update_params(double x_position_tolerance,double x_angular_tolerance,double reverse_max_vel,double homing_distance )
{
	position_tolerance = x_position_tolerance;
	angular_tolerance = x_angular_tolerance;
	reverse_max_speed = reverse_max_vel;
	m_homing_distance= homing_distance;
	ROS_INFO("updated position_tolerance %f ",position_tolerance ); 
	
}




void QrNavigation::Odom_callback(const nav_msgs::OdometryConstPtr &odom_msg)
{
	
	robot_pose.position.x = odom_msg->pose.pose.position.x;
	robot_pose.position.y = odom_msg->pose.pose.position.y;
        
	current_linear_velocity = odom_msg->twist.twist.linear.x;
	tf::Quaternion q(odom_msg->pose.pose.orientation.x,
			odom_msg->pose.pose.orientation.y,
			odom_msg->pose.pose.orientation.z,
			odom_msg->pose.pose.orientation.w);	
	tf::Matrix3x3 m(q);
	m.getRPY(roll,pitch,yaw);
	robot_pose.orientation.yaw = yaw;
	
	//pose_data.x = robot_pose.position.x;
	//pose_data.y = robot_pose.position.y;
	//pose_data.theta = yaw;
	//robot_pose_pub.publish(pose_data);
	
	
	pure_pursuit.Intermediate_goal_point(robot_pose,current_linear_velocity);

	//ROS_INFO("odom_call_back");
}

pose_struct::Matrix QrNavigation::Path_smoothening(pose_struct::PoseEuler start,pose_struct::PoseEuler goal,double change_in_value,int x_)
{
			smooth_path_data.array = new double *[2];

			//ROS_INFO("chagne in value %f ",change_in_value);
			
			path_array_size = abs(change_in_value *100) + 1000;
			//This 1000 is added as a part of solving the problem of robot rotating when its about to stop, 
			//as per the calculations we need to plan ahead if we want to stop without making too much angular movement
			float start_value =start.position.x;
			//ROS_INFO("start_value %f ",start_value);
			float goal_value =start.position.y;


			float path_array[path_array_size];
			for (int i=0; i<2;i++)
			{
					smooth_path_data.array[i] = new double[path_array_size] ;
			}

			for (int i=0;i <path_array_size; i++)
			{
						if (change_in_value >0 and i ==0 )
						{
                                        //path_array[0] = start;

						*(smooth_path_data.array[0]+0) =start_value;
						*(smooth_path_data.array[1]+0) =goal_value;
			
                        }
                        if(change_in_value >0 and i>0)
						{
                                       // path_array[i]= path_array[i-1]+0.01;
					
						if(x_==0)
						{
							*(smooth_path_data.array[0]+i) = *(smooth_path_data.array[0]+i-1) +0.01;
								*(smooth_path_data.array[1]+i) = goal_value;	
									
						}
						if(x_==1)
						{
							*(smooth_path_data.array[0]+i) =start_value;
							//ROS_INFO("change_value ==1 and start value %f ",start_value);
							*(smooth_path_data.array[1]+i) = *(smooth_path_data.array[1]+i-1) +0.01;
						}


					
				
						}

						if (change_in_value <0 and i ==0 )
						{
                                       // path_array[0] = start;
							*(smooth_path_data.array[0]+0) = start_value;
							*(smooth_path_data.array[1]+0) =goal_value;
                    

						}
						if(change_in_value <0 and i>0 )
						{
						if (x_ ==0)
						{
					
                            *(smooth_path_data.array[0]+i) = *(smooth_path_data.array[0]+i-1) -0.01;   
							*(smooth_path_data.array[1]+i) = goal_value;
						}
					if(x_ ==1)
					{
						*(smooth_path_data.array[0]+i) =start_value;
						//ROS_INFO("change in value == 1 and in y start_value %f",start_value);
                                                *(smooth_path_data.array[1]+i) =*(smooth_path_data.array[1]+i-1) -0.01;
					}
					
			}


                        }
                      /*  for (int j = 0;j<path_array_size;j++)
                        {
                                ROS_INFO("path_data : %lf :, %f ", *(smooth_path_data.array[0]+j), *(smooth_path_data.array[1]+j));
                        }
*/
                       // ROS_INFO("length of the array %d ",path_array_size);



			return smooth_path_data;

}

void QrNavigation::Obstacle_node(const anscer_msgs::SafetyFields &obstacle_msg)
{

	caution_front = obstacle_msg.warning[0].is_front;
	caution_rear =  obstacle_msg.warning[0].is_rear;
	stop_front =    obstacle_msg.safety[0].is_front;
	stop_rear =     obstacle_msg.safety[0].is_rear;
	ROS_INFO("caution front:  %d , caution_rear: %d , stop_front %d , stop_rear: %d" , caution_front,caution_rear,stop_front,stop_rear);
}



void QrNavigation::Goal_path_callback(const qr_navigation::goal_msg &goal_path_msg)
{
	ROS_INFO("goal_path_msg_pose %s  operation : %s  ",goal_path_msg.pose.c_str(),goal_path_msg.operation.c_str());
	std::string current_goal;
	current_goal = std::string(goal_path_msg.pose);
	operation = std::string(goal_path_msg.operation);
	if(last_goal_position!= current_goal)
	{
		new_goal = true;
		old_goal = false;
		goal_pose.position.x =goal_path_msg.goal[0];
		goal_pose.position.y =goal_path_msg.goal[1];
		goal_pose.position.z =0;
		goal_pose.orientation.yaw =goal_path_msg.goal[2];
		start_pose.position.x =goal_path_msg.start[0];
               // ROS_INFO("StartPose %f ", start_pose.position.x );
		start_pose.position.y =goal_path_msg.start[1];
		start_pose.position.z = 0;

		
		double change_in_x,change_in_y;
		change_in_x = goal_path_msg.goal[0]  - goal_path_msg.start[0];
		change_in_y = goal_path_msg.goal[1] -  goal_path_msg.start[1];
		ROS_INFO("chnage_in_x %f , change in y %f ",change_in_x,change_in_y);
		if(fabs(change_in_x) > 0.1 && fabs(change_in_y) < 0.01)
		{

			path_data = Path_smoothening(start_pose,goal_pose,change_in_x,0);
			path_array_size = abs(change_in_x *100) +1000;
			ROS_WARN("change in x, path processing in progress");

			if(change_in_x <0)
			{
				start_and_goal_data.init_pose.position.z =3.14;
			}
			if(change_in_x > 0)
			{
				start_and_goal_data.init_pose.position.z = 0;
			}
		}
	

		if(fabs(change_in_y) > 0.1 && fabs(change_in_x) <0.01)
		{
		       path_data = Path_smoothening(start_pose,goal_pose,change_in_y,1);
		       path_array_size = abs(change_in_y *100) + 1000;
		       ROS_WARN("change in y, path processing in progress");
			if(change_in_y <0)
			{
				start_and_goal_data.init_pose.position.z=-1.57;
			}
			if(change_in_y > 0)
			{
				start_and_goal_data.init_pose.position.z =1.57;
			}


		}

	        /*goal_pose.position.x =goal_path_msg.goal[0];
        	goal_pose.position.y =goal_path_msg.goal[1];
        	goal_pose.position.z =0;
        	goal_pose.orientation.yaw =goal_path_msg.goal[2];
		start_pose.position.x =path_msg.start[0];
        	start_pose.position.y =path_msg.start[1];
        	start_pose.position.z = 0;



		*/


			for (int i=0; i<path_array_size;i++)
		{

			   float x_pose_test,y_pose_test;
			   bool x_finate,y_finate;
			   x_pose_test = *(path_data.array[0]+i);
       			   y_pose_test = *(path_data.array[1]+i);
			   x_finate = std::isinf(x_pose_test);
			   y_finate = std::isinf(y_pose_test);
			   if(x_finate == 1)
			   {
			 	*(path_data.array[0]+i) = 0.0;
			   
			   } 
			   if(y_finate ==1 and i !=0)
			   {
			   *(path_data.array[1]+i) = *(path_data.array[0]+i-1);
			   }

			//ROS_INFO("pathX: %f, pathY: %f ,finate : %d ", x_pose_test,y_pose_test,finate);
		}
		
		pure_pursuit.Update_path(start_pose,goal_pose,path_data,path_array_size);
		ROS_INFO("done pure pursuit");
		start_and_goal_data.init_pose.position.x = goal_path_msg.start[0];
       		start_and_goal_data.init_pose.position.y = goal_path_msg.start[1];

        	start_and_goal_data.goal_pose.position.x = goal_path_msg.goal[0];
        	start_and_goal_data.goal_pose.position.y = goal_path_msg.goal[1];

	
	
		last_goal_position = current_goal;

		
	//	ROS_INFO("New Goal Recived");
		unsigned int microsecond = 1000000;
		usleep(1 * microsecond);
		tolarance_distance = 0.01;		
		new_goal = false;
		reached_position = false;
		old_goal = true;

	}
	
	

}


void QrNavigation::Path_callback(const nav_msgs::PathConstPtr &path_msg)
{

	//ROS_INFO("size of array: %ld ",path_msg->poses.size());
	path_array_size = path_msg->poses.size();
	


	path_data.array = new double *[2];

	for (int i=0; i<2;i++)
	{	
		path_data.array[i] = new double[path_array_size] ;
	}	
	for (int j=0; j<path_array_size;j++)
	{
		*(path_data.array[0]+j) = path_msg->poses[j].pose.position.x;
		*(path_data.array[1]+j) = path_msg->poses[j].pose.position.y;
	}

	/*for (int k=0; k<path_array_size;k++)
        {
                ROS_INFO("path_data %f: , %f: ",*(path_data.array[0]+k),*(path_data.array[1]+k));
        }*/
	
	goal_pose.position.x =path_msg->poses[path_array_size-1].pose.position.x;
	goal_pose.position.y =path_msg->poses[path_array_size-1].pose.position.y;
	goal_pose.position.z =path_msg->poses[path_array_size-1].pose.position.z;
	
	tf::Quaternion q_goal(path_msg->poses[path_array_size-1].pose.orientation.x,
			 path_msg->poses[path_array_size-1].pose.orientation.y,
			 path_msg->poses[path_array_size-1].pose.orientation.z,
			 path_msg->poses[path_array_size-1].pose.orientation.w);
        tf::Matrix3x3 m_goal(q_goal);
        m_goal.getRPY(goal_roll,goal_pitch,goal_yaw);
	
	goal_pose.orientation.yaw = goal_yaw;
	//ROS_INFO("goal_pose orientation %f, %f ,%f ",goal_yaw,path_msg->poses[path_array_size-1].pose.orientation.z,path_msg->poses[path_array_size-1].pose.orientation.w);		
	start_and_goal_data.init_pose.position.x = path_msg->poses[0].pose.position.x;
	start_and_goal_data.init_pose.position.y = path_msg->poses[0].pose.position.y;

	start_and_goal_data.goal_pose.position.x = path_msg->poses[path_array_size-1].pose.position.x;
	start_and_goal_data.goal_pose.position.y = path_msg->poses[path_array_size-1].pose.position.y;
	start_and_goal_data.goal_pose.orientation.yaw = goal_yaw; 

        start_pose.position.x =path_msg->poses[0].pose.position.x;
        start_pose.position.y =path_msg->poses[0].pose.position.y;
        start_pose.position.z =path_msg->poses[0].pose.position.z;
        tf::Quaternion q_start(path_msg->poses[0].pose.orientation.x,
                         path_msg->poses[0].pose.orientation.y,
                         path_msg->poses[0].pose.orientation.z,
                         path_msg->poses[0].pose.orientation.w);
        tf::Matrix3x3 m_start(q_start);
        m_start.getRPY(start_roll,start_pitch,start_yaw);
        //goal_pose.orientation.yaw = start_yaw;

	start_pose.orientation.yaw = start_yaw;

	pure_pursuit.Update_path(start_pose,goal_pose,path_data,path_array_size);
	 /*here we are gonna update the pure pursuit with path to be followed, */


	//ROS_INFO("path start.x: %f, y: %f, z: %f, w: %f",path_msg->poses[0].pose.position.x,path_msg->poses[0].pose.position.y,path_msg->poses[0].pose.orientation.z,path_msg->poses[0].pose.orientation.w);
	//ROS_INFO("path goal.x: %f, y: %f, z: %f, w: %f",path_msg->poses[1].pose.position.x,path_msg->poses[1].pose.position.y,path_msg->poses[1].pose.orientation.z,path_msg->poses[1].pose.orientation.w);

}

double QrNavigation::desired_theta_correction(double desired_theta_)
{
	double corrected_theta;
	if(desired_theta_ > 2.5 | desired_theta_ < - 2.5)
		{
			corrected_theta = 3.1416;
		}
	if(desired_theta_ > 0.9 && desired_theta_ < 2.0)
		{
		corrected_theta = 1.5708;
		}
	if(desired_theta_ < - 0.9 && desired_theta_ > - 2.0)
		{
		corrected_theta = -1.5708;
		}
		
	if(desired_theta_ < 0.3 && desired_theta_ > -0.3)
		{
					corrected_theta = 0;

		}
	return corrected_theta;

}


double QrNavigation::find_desired_rot_angle(double current_theta, double desired_theta)
{
  double corrected_theta;
  corrected_theta = desired_theta_correction(desired_theta);

  double dif_angle = corrected_theta - current_theta;
 // ROS_INFO("diff angle : %f  current_theta: %f desired_theta %f ",dif_angle, current_theta,corrected_theta);
  double normalize_angle_positive = fmod((fmod(dif_angle, 2.0 * 3.1416)) + (2 * 3.1416), (2.0 * 3.1416));
  if (normalize_angle_positive > 3.1416)
	{
		normalize_angle_positive -= 2.0 * 3.1416;
	}
  return normalize_angle_positive;

}

float QrNavigation::check_operation(float m_linear_velocity)
{
				if (operation == "NORMAL" && caution_front == true)
					{
							m_linear_velocity = 0.2;
					}
		
				if(operation == "UNDOCK") 
				{
					m_linear_velocity = -1 * m_linear_velocity;
					if(m_linear_velocity < (-1 * reverse_max_speed))
					{
					//	linear_velocity = -0.5;
						m_linear_velocity = -1 * reverse_max_speed;
					}
				}
				if(operation == "REVERSE") 
				{
					m_linear_velocity = -1* m_linear_velocity;

					if (caution_rear == true )
					{
						if(m_linear_velocity < - 0.15)
						{
							m_linear_velocity = -0.15;
						}
					}
					//ROS_INFO("undock operation");

				}
				if(operation == "DOCK")
				{
					if (m_linear_velocity > 0.2)
					{
						m_linear_velocity = 0.2;
					}
				}


				if (operation == "CHARGE")
				{
					m_linear_velocity = -1 *m_linear_velocity;
					if (m_linear_velocity < -0.2)
					{ 
						m_linear_velocity = -0.1;
					}
				}
		return m_linear_velocity;
}

float QrNavigation::find_distance_to_goal()
{
		float change_in_x_direction,  change_in_y_direction,distance_to_goal_point;
		change_in_x_direction = start_and_goal_data.init_pose.position.x - start_and_goal_data.goal_pose.position.x;

		change_in_y_direction = start_and_goal_data.init_pose.position.y - start_and_goal_data.goal_pose.position.y;
		
		if(fabs(change_in_x_direction) <= 0.01 && change_in_y_direction!=0)
		{
			distance_to_goal_point = fabs(robot_pose.position.y - start_and_goal_data.goal_pose.position.y);
			ROS_INFO("robot travels in y direction --> distance to goal %f ",distance_to_goal_point );


		}
		if(change_in_x_direction !=0 && fabs(change_in_y_direction) <=0.1)
		{
			distance_to_goal_point = fabs(robot_pose.position.x - start_and_goal_data.goal_pose.position.x);
			ROS_INFO("robot travels in x direction --> distance to goal %f ",distance_to_goal_point );

		}
		
		return distance_to_goal_point;

}

void QrNavigation::run()
{
    robot_init_status_.data=is_initialized;
    robot_init_status.publish(robot_init_status_);
    robot_pause_status_.data= is_paused;
    robot_pause_status.publish(robot_pause_status_); 
 
	if(is_initialized == true && is_paused == false)   // check whether robot is initialized (place the robot phisically on top of a qr code so that the robot understand the current position)
	{
	
		double desired_start_orientaion = find_desired_rot_angle(robot_pose.orientation.yaw, start_and_goal_data.init_pose.position.z);	
	//	ROS_INFO("desired start orientation %f ", abs(desired_start_orientaion));
		
		desired_rot_angle = find_desired_rot_angle(robot_pose.orientation.yaw, goal_pose.orientation.yaw);

	//	ROS_INFO("Desired rot angle %f ",abs(desired_rot_angle));
	//	distance_to_goal = sqrt(pow((robot_pose.position.x - start_and_goal_data.goal_pose.position.x), 2) + pow((robot_pose.position.y - start_and_goal_data.goal_pose.position.y), 2));
		distance_to_goal = find_distance_to_goal();
		
		if(distance_to_goal > m_homing_distance)
		{ 
			//ROS_INFO("homing distance %d , extractor home: %d ,distance to goal %d ",m_homing_distance,is_extractor_home,distance_to_goal);
			is_extractor_home = true;

		}
		else
		{

		//	ROS_INFO("homing distance %d , extractor home: %d ",m_homing_distance,is_extractor_home);
			is_extractor_home = false;

		}
		homing_data.data = is_extractor_home;
		extractor_homing_pub.publish(homing_data);


		if (new_goal == false) 
		{ 
		//ROS_INFO("new_goal:  %d ", new_goal);
		if(fabs(desired_start_orientaion) < 0.3 || fabs(desired_rot_angle) > 0.015 && new_goal == false && distance_to_goal < 0.5 || operation =="UNDOCK" || operation == "CHARGE")
		{
				//ROS_INFO_ONCE("started navigation");
		//		ROS_INFO("current_linear_velocity before ---> %f ",current_linear_velocity);

				linear_velocity = velocity_profile.Find_linear_velocity(start_and_goal_data,robot_pose,current_linear_velocity);
				linear_velocity =check_operation(linear_velocity);
		
				ROS_INFO("linear_velocity %f : ",linear_velocity);
				distance_to_goal = find_distance_to_goal();
				//distance_to_goal = sqrt(pow((robot_pose.position.x - start_and_goal_data.goal_pose.position.x), 2) + pow((robot_pose.position.y - start_and_goal_data.goal_pose.position.y), 2));
				//ROS_INFO("distance to goal %f",distance_to_goal);
	


				if (distance_to_goal < position_tolerance && reached_position ==false )
				{
			//		ROS_INFO("last_distance --> :  %f, current distance ---> %f, position_tolerance ---> %f, reached_position --->%d  ",last_distance,distance_to_goal,position_tolerance,reached_position);
					//if (last_distance < distance_to_goal || distance_to_goal < 0.02)
					///	if((last_distance < distance_to_goal) && (fabs(last_distance - distance_to_goal) > 0.005) || fabs(distance_to_goal) < 0.01)
					    if(operation == "NORMAL")
                        {
                            goal_precision = 0.005;
                        }
                        else
                        {
                            goal_precision = 0.015;    


                        }
                        if(fabs(distance_to_goal) < goal_precision)
                        	{
							ROS_INFO("reached position ");
							reached_position = true;
							
						    }
                        if(last_distance < distance_to_goal)
                        {
                            ROS_INFO("last distance %f , distane to goal %f ",last_distance,distance_to_goal);
                            ROS_ERROR("Some issue with position please check  dont run the robot");
                            //is_paused = true;
                        }
				}
				
			//	distance_to_goal = sqrt(pow((robot_pose.position.x - start_and_goal_data.goal_pose.position.x), 2) + pow((robot_pose.position.y - start_and_goal_data.goal_pose.position.y), 2));
		
				ROS_INFO("distance to goal %f ,position_tolerance %f , reached_position %d ",distance_to_goal,position_tolerance ,reached_position);
				
				if (fabs(distance_to_goal) <= position_tolerance && reached_position == true && old_goal == true || in_place_rotation==true)
				{
					
					if(m_inside_count <1)
					{				
					in_place_rotation = true;
					control_velocity.linear.x =0;
					control_velocity.angular.z =0;
					m_inside_count +=1;
				//	ROS_INFO("publishing angluar vlocity ----->>>>%f",control_velocity.angular.z);
					cmd_vel_pub.publish(control_velocity);
					reached_position = true;
					ros::Rate rate_(0.5);
					rate_.sleep();
					}
					if (old_goal==true)
						{
					desired_rot_angle = find_desired_rot_angle(robot_pose.orientation.yaw, goal_pose.orientation.yaw);
					ROS_INFO("desired_rot_angle --> %f ",desired_rot_angle);
						}
					rotation_control.rotate_to_angle(desired_rot_angle, control_input);
					control_input.linear.x = 0.0;
					//Added rotation tolerance 
					//if(abs(desired_rot_angle)<0.05)
					if(fabs(desired_rot_angle)<angular_tolerance)
					{
					goal_reached.data = "reached_" + last_goal_position;
					result_pub.publish(goal_reached);
					in_place_rotation = false;
					m_inside_count =0;
					control_input.linear.x = 0.0;
					control_input.angular.z = 0.0;
					cmd_vel_pub.publish(control_velocity);
					ros::Rate rate_(10);
					rate_.sleep();
					old_goal =false;
					new_goal = true;

					}
					
					tolarance_distance = 0.1;		
						
				}

				else if(distance_to_goal > tolarance_distance and in_place_rotation == false && reached_position == false  || reached_position == false )
				{

						control_input= pure_pursuit.Pure_pursuit_controller(linear_velocity);
						lateral_shift_data.data = control_input.linear.y;
                        look_ahead_data.data=control_input.linear.z; 
						lateral_pub.publish(lateral_shift_data);
					    look_ahead_pub.publish(look_ahead_data);	
				}
				else
				{
					//	std::cout << "idle state" << std::endl;
						control_input.linear.x = 0.0;
						control_input.angular.z = 0.0;
				 }

				
				control_velocity.linear.x =control_input.linear.x;
				
				control_velocity.angular.z = control_input.angular.z;
				//ROS_INFO("control velocity for rotation --> %f,",control_velocity.angular.z);
                if (operation=="UNDOCK" || operation =="REVERSE")
				{
					
					if(stop_rear == true)
					{
										
					control_velocity.linear.x = 0; 
					control_velocity.angular.z = 0;
					}
				}
				if (operation=="NORMAL")
				{
					
					if(stop_front == true)
					{
										
					control_velocity.linear.x = 0; 
					control_velocity.angular.z = 0;
					}
				}
				//ROS_INFO("Velocity before publishing --> %f ",control_velocity.linear.x);
				cmd_vel_pub.publish(control_velocity);
				dist_to_goal.data = distance_to_goal;

				distance_to_goal_pub.publish(dist_to_goal);
				last_distance = distance_to_goal;


			//	ROS_INFO("distance to goal : %f ",distance_to_goal);
		}
		else 
		{
			control_velocity.linear.x =0;
			control_velocity.angular.z =0;

			cmd_vel_pub.publish(control_velocity);	
		}
		}
			else 
		{
			control_velocity.linear.x =0;
			control_velocity.angular.z =0;

			cmd_vel_pub.publish(control_velocity);	
		}
	}

	else
		{
			ROS_DEBUG_THROTTLE(3,"waiting for robot initialization");
			control_velocity.linear.x =0;
			control_velocity.angular.z =0;
			cmd_vel_pub.publish(control_velocity);	

		}
}


int main(int argc, char** argv)
{
	ros::init(argc,argv, "qr_navigation_node");
	QrNavigation *qr_navigation = new QrNavigation();
	dynamic_reconfigure::Server<qr_navigation::ParameterConfig> server;
	dynamic_reconfigure::Server<qr_navigation::ParameterConfig>::CallbackType f;
	f = boost::bind(&QrNavigation::callback,qr_navigation, _1, _2);
	server.setCallback(f);
	ros::Rate rate(25);
	ROS_INFO("started navigation node");
	while ( ros::ok())
	{
		qr_navigation->run();
		ros::spinOnce();
		rate.sleep();	
	
	}
	delete qr_navigation;
	return 0;
}

