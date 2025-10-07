#include "mission_scheduler.h"
#include <ros/console.h>
#include <string>
#include <unistd.h>

MissionScheduler::MissionScheduler(std::string name)
	: // as(nh,name,false),
		as(nh, name, boost::bind(&MissionScheduler::executeCB, this, _1), false)
		,

		action_name_(name)
{
	//   battery_sub = nh.subscribe("/battery", 1, &MissionScheduler::batteryCB, this);
	// battery_sub_ = nh.subscribe("/battery_data",1,&MissionScheduler::batteryCBData, this);
	charge_battery = nh.advertise<std_msgs::Int32>("/charge_battery", 10);
	goal_msg_pub = nh.advertise<qr_navigation::goal_msg>("goal_path", 10);
	robot_status = nh.advertise<std_msgs::Bool>("robot_status", 10);
	robot_odom_sub = nh.subscribe("/odom", 1, &MissionScheduler::OdomCB, this);
	//m_goal_server = nh.advertiseService("/goal", &MissionScheduler::goalServerCB, this);
	robot_goal_result_sub = nh.subscribe("/goal_result", 1, &MissionScheduler::GoalResultCB, this);
	path_client = nh.serviceClient<graph_msgs::GetGraphPlan>("make_graph_plan");
	as.start();
}

MissionScheduler::~MissionScheduler()
{
}

void MissionScheduler::preemptCB()
{
	ROS_INFO("Mission cancelled");
	as.setPreempted();
}

void MissionScheduler::GoalResultCB(const std_msgs::String& goal_result)
{
	m_goal_result = goal_result.data;
	//ROS_INFO("goal result %s ", m_goal_result.c_str());
}



void MissionScheduler::executeCB(const qr_mission_scheduler::mission_schedulerGoalConstPtr& goal)

{

	ROS_INFO("mission_server called");

	m_goal_data.goalType = goal->goalType.goalType;
	m_goal_data.source = goal->goalType.source;
	m_goal_data.destination = goal->goalType.destination;
	execute_goal(m_goal_data);
}



graph_msgs::GetGraphPlan MissionScheduler::get_path(const int source_id, const int target_id)
{
	ROS_INFO("Get path called");
	graph_msgs::GetGraphPlan path_plan;
	path_plan.request.source_id = source_id;
	path_plan.request.target_id = target_id;
	path_plan.request.ignore_path_orientation = false;
	path_plan.request.ignore_goal_orientation = false;

	return path_plan;
}




void MissionScheduler::split_path(graph_msgs::GetGraphPlanResponse &response_data, std::vector<std::vector<std::vector<double>>>  &stripped_path_poses)
{


	ROS_INFO("path_filtered_data_vec_2D %ld --->" ,path_filtered_data_vec_2D.size());

	for(int k = 0; k < (int)path_filtered_data_vec_2D.size(); k++) 
	{
		//		 ROS_INFO("entering the loop");
		//			 ROS_INFO("%f ",path_filtered_data_vec_2D[k][0]);
		//			ROS_INFO("%f ",path_filtered_data_vec_2D[k][1]);
		//			ROS_INFO("%f ",path_filtered_data_vec_2D[k][2]);


		/*	planned_path_poses = response_data.plan.poses[k];
			tf::Quaternion q(planned_path_poses.pose.orientation.x, planned_path_poses.pose.orientation.y,
			planned_path_poses.pose.orientation.z, planned_path_poses.pose.orientation.w);
			tf::Matrix3x3 m(q);
			m.getRPY(roll, pitch, yaw);
			*/
		yaw = path_filtered_data_vec_2D[k][2];

		path_data_vec_1D.clear();
		path_data_vec_1D.push_back(path_filtered_data_vec_2D[k][0]);
		path_data_vec_1D.push_back(path_filtered_data_vec_2D[k][1]);
		path_data_vec_1D.push_back(path_filtered_data_vec_2D[k][2]);

		path_data_vec_2D.push_back(path_data_vec_1D);
		if(k != 0) 
		{
			//previous yaw 
			/*	planned_path_poses = response_data.plan.poses[k - 1];
				tf::Quaternion quat_(planned_path_poses.pose.orientation.x, planned_path_poses.pose.orientation.y,
				planned_path_poses.pose.orientation.z, planned_path_poses.pose.orientation.w);
				tf::Matrix3x3 matrix_(quat_);
				matrix_.getRPY(roll, pitch, previous_yaw);
				*/
			previous_yaw = path_filtered_data_vec_2D[k-1][2];
			ROS_INFO(" yaw --> %f  ", yaw);
			// 2 times previous yaw

			/*planned_path_poses_p = response_data.plan.poses[k + 1];

			  tf::Quaternion quat_p(planned_path_poses_p.pose.orientation.x, planned_path_poses_p.pose.orientation.y,
			  planned_path_poses_p.pose.orientation.z, planned_path_poses_p.pose.orientation.w);
			  tf::Matrix3x3 matrix_p(quat_p);
			  matrix_p.getRPY(roll, pitch, next_yaw);

*/


			/*	if((yaw - previous_yaw > 0.1 || yaw - previous_yaw < -0.1)) 
				{
				stripped_path_poses.push_back(path_data_vec_2D);
				path_data_vec_2D.clear();

				stripped_path_count += 1;
				path_data_count = 0;
				ROS_INFO("different yaw--> path stripped %d , %f current yaw : %f ", stripped_path_count, previous_yaw,yaw);
				}
				*/


			if(abs(yaw-previous_yaw)> 1.5 && abs(yaw-previous_yaw) < 1.7 
					|| abs(yaw-previous_yaw)> 4.6 && abs(yaw-previous_yaw) < 4.8 ) // assuming the difference in yaw is exacly oposite direction 
			{

				stripped_path_poses.push_back(path_data_vec_2D);
				path_data_vec_2D.clear();

				stripped_path_count += 1;
				path_data_count = 0;
				ROS_INFO("different yaw--> path stripped %d , %f current yaw : %f ", stripped_path_count, previous_yaw,yaw);

			}


		}
		// ROS_INFO("current yaw %f ",yaw);
		if(k == 0 && (int)path_filtered_data_vec_2D.size() > 0) 
		{
			//planned_path_poses = response_data.plan.poses[k + 1];

			/*	tf::Quaternion quat_(planned_path_poses.pose.orientation.x, planned_path_poses.pose.orientation.y,
				planned_path_poses.pose.orientation.z, planned_path_poses.pose.orientation.w);
				tf::Matrix3x3 matrix_(quat_);
				matrix_.getRPY(roll, pitch, previous_yaw);
				*/
			previous_yaw = path_filtered_data_vec_2D[k+1][2];

			//	if((yaw - previous_yaw > 0.1 || yaw - previous_yaw < -0.1)) 
			if(abs(yaw-previous_yaw)> 1.5 && abs(yaw-previous_yaw) < 1.7 || abs(yaw-previous_yaw)> 4.6 && abs(yaw-previous_yaw) < 4.8 ) // assuming the difference in yaw is exacly oposite direction 
			{
				stripped_path_poses.push_back(path_data_vec_2D);
				path_data_vec_2D.clear();

				stripped_path_count += 1;
				path_data_count = 0;
				ROS_INFO("different yaw--> path stripped %d , %f , current_yaw : %f  ", stripped_path_count, previous_yaw,yaw);
			}

			//	stripped_path_poses.push_back(path_data_vec_2D);
		}
		if(k == (int)path_filtered_data_vec_2D.size() - 1) 
		{
			if(path_data_vec_2D.size() > 0) 
			{
				stripped_path_poses.push_back(path_data_vec_2D);
				//ROS_INFO("pushing last path data");
			}
		}

		path_data_count += 1;
	}



}

void MissionScheduler::check_straight_line(graph_msgs::GetGraphPlanResponse &response_data) 	//,std::vector<std::vector<std::vector<double>>>  &stripped_path_poses)
{

	ROS_INFO("check straight line size of path recived --> %ld ",response_data.plan.poses.size());
	// for(int m_=0;  m_<response_data.plan.poses.size();m_++)
	// {
	// 	ROS_INFO("z value %f ",response_data.plan.poses[m_].pose.orientation.z);
	// }

	path_filtered_data_vec_1D.clear();
	path_filtered_data_vec_2D.clear();

	bool reset_flag{false};
	planned_path_poses = response_data.plan.poses[0];
	tf::Quaternion q(planned_path_poses.pose.orientation.x, planned_path_poses.pose.orientation.y,
			planned_path_poses.pose.orientation.z, planned_path_poses.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);


	path_filtered_data_vec_1D.push_back(planned_path_poses.pose.position.x);
	path_filtered_data_vec_1D.push_back(planned_path_poses.pose.position.y);
	path_filtered_data_vec_1D.push_back(yaw);

	path_filtered_data_vec_2D.push_back(path_filtered_data_vec_1D);
	for(int k = 1; k < (int)response_data.plan.poses.size()-1 ; k++) 
	{

		planned_path_poses = response_data.plan.poses[k];
		tf::Quaternion q(planned_path_poses.pose.orientation.x, planned_path_poses.pose.orientation.y,
				planned_path_poses.pose.orientation.z, planned_path_poses.pose.orientation.w);
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		//	ROS_INFO("current_yaw -->	%f ",yaw);


		planned_path_poses_p = response_data.plan.poses[k + 1];

		tf::Quaternion quat_p(planned_path_poses_p.pose.orientation.x, planned_path_poses_p.pose.orientation.y,
				planned_path_poses_p.pose.orientation.z, planned_path_poses_p.pose.orientation.w);
		tf::Matrix3x3 matrix_p(quat_p);
		matrix_p.getRPY(roll, pitch, next_yaw);
		//ROS_INFO("next_yaw -->	%f ",next_yaw);
		goal_yaw_for_charging = next_yaw;

		planned_path_poses = response_data.plan.poses[k - 1];
		tf::Quaternion quat_(planned_path_poses.pose.orientation.x, planned_path_poses.pose.orientation.y,
				planned_path_poses.pose.orientation.z, planned_path_poses.pose.orientation.w);
		tf::Matrix3x3 matrix_(quat_);
		matrix_.getRPY(roll, pitch, previous_yaw);

		//	ROS_INFO("previous_yaw -->	%f ",previous_yaw);


		ROS_INFO("goal_yaw_for_charging %f ",goal_yaw_for_charging);

		if(yaw!=next_yaw && yaw!=previous_yaw &&previous_yaw==next_yaw)
		{
			ROS_INFO("yaw to be removed -->	%f, previous yaw--> %f ",yaw, previous_yaw);
			reset_flag = true;
			//ROS_INFO("yaw -->	%f ",yaw);
			path_filtered_data_vec_1D.clear();
			path_filtered_data_vec_1D.push_back(planned_path_poses.pose.position.x);
			path_filtered_data_vec_1D.push_back(planned_path_poses.pose.position.y);
			path_filtered_data_vec_1D.push_back(previous_yaw);
			//	ROS_INFO("yaw -->	%f,%f,%f ",path_filtered_data_vec_1D[0],path_filtered_data_vec_1D[1],path_filtered_data_vec_1D[2]);

			path_filtered_data_vec_2D.push_back(path_filtered_data_vec_1D);
		}
		else 
		{
			if(reset_flag == true)
			{
				previous_yaw = yaw;
				ROS_INFO("resetting previous yaw %f with yaw %f  ",previous_yaw,yaw);
			}	
			reset_flag = false;
			//		ROS_INFO("yaw -->	%f ",yaw);
			path_filtered_data_vec_1D.clear();
			path_filtered_data_vec_1D.push_back(planned_path_poses.pose.position.x);
			path_filtered_data_vec_1D.push_back(planned_path_poses.pose.position.y);
			path_filtered_data_vec_1D.push_back(previous_yaw);
			//	ROS_INFO("yaw -->	%f,%f,%f ",path_filtered_data_vec_1D[0],path_filtered_data_vec_1D[1],path_filtered_data_vec_1D[2]);

			path_filtered_data_vec_2D.push_back(path_filtered_data_vec_1D);
		}
	}



	if( (int)response_data.plan.poses.size() >0)
	{
		planned_path_poses = response_data.plan.poses[response_data.plan.poses.size()-1];
		ROS_INFO(" x --> %f ,y --> %f ",planned_path_poses.pose.position.x,planned_path_poses.pose.position.y);
		//	planned_path_poses = response_data.plan.poses[(int)response_data.plan.poses.size()-1];
		tf::Quaternion q(planned_path_poses.pose.orientation.x, planned_path_poses.pose.orientation.y,
				planned_path_poses.pose.orientation.z, planned_path_poses.pose.orientation.w);
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);

		path_filtered_data_vec_1D.clear();

		path_filtered_data_vec_1D.push_back(planned_path_poses.pose.position.x);
		path_filtered_data_vec_1D.push_back(planned_path_poses.pose.position.y);
		path_filtered_data_vec_1D.push_back(previous_yaw);

		path_filtered_data_vec_2D.push_back(path_filtered_data_vec_1D);
		ROS_INFO("path_filtered_data_vec_2D size ->	%ld ",path_filtered_data_vec_2D.size());;
	}


}

void MissionScheduler::ignore_orientaion(std::vector<std::vector<std::vector<double>>>  &stripped_path_poses,bool robot_orientation)
{
	int x{0},y{0};
	x = stripped_path_poses.size();
	y = stripped_path_poses[x-1].size();
	//ROS_INFO("x pos %d y pos: %d ",x,y);
	for (int i{0};i<x;i++)
	{
		for (int j{0};j<stripped_path_poses[i].size();j++)
		{
			if (robot_orientation)
			{
				stripped_path_poses[i][j][2] = m_robot_odom_yaw; 
			}
			else 
			{
				stripped_path_poses[i][j][2] =  stripped_path_poses[x-1][y-1][2];
			}

		}
	}
}

void MissionScheduler::check_inplace_rotation(std::vector<std::vector<std::vector<double>>>  &stripped_path_poses,bool robot_orientation)
{
	int x{0},y{0};
	float yaw_ {0};
	float next_yaw{0};

	x = stripped_path_poses.size();
	ROS_INFO("x %i",x);
	if(x >=1) 
	{
		y = stripped_path_poses[0].size();
		ROS_INFO("y : %i",y);
	}
	else {
		y = stripped_path_poses[0].size();
		ROS_INFO("y-- :%i",y);
	}
	yaw_ = stripped_path_poses[0][0][2];
	//next_yaw  = stripped_path_poses[1][0][2];
	ROS_INFO("x -->: %i , y :%i, yaw : %f ,m_robot_odom_yaw : %f, next_yaw : %f ",stripped_path_poses.size(),y,yaw_,m_robot_odom_yaw,next_yaw);


	if(fabs(fabs(m_robot_odom_yaw) - fabs(yaw_)) > 1.8 || (m_robot_odom_yaw + yaw_) < 0.2 && (m_robot_odom_yaw + yaw_) > -0.2 ) 
	{
		
 //if(abs(m_robot_odom_yaw - yaw_) > 1.8)
         double v_r_x = std::cos(m_robot_odom_yaw);
         double v_r_y = std::sin(m_robot_odom_yaw);
         // Calculate vector from robot to point
        double v_p_x = stripped_path_poses[0][y-1][0] - m_robot_pose_data.pose.pose.position.x;
        double v_p_y = stripped_path_poses[0][y-1][1] - m_robot_pose_data.pose.pose.position.y;

        // Calculate the dot product
        double dotProduct = (v_r_x * v_p_x) + (v_r_y * v_p_y);

        ROS_WARN("Robot X : %f, Robot Y : %f, Robot Yaw : %f, path X : %f, Path Y : %f , Path Yaw : %f ",m_robot_pose_data.pose.pose.position.x,m_robot_pose_data.pose.pose.position.y,m_robot_odom_yaw,stripped_path_poses[0][y-1][0],stripped_path_poses[0][y-1][1]);
        ROS_INFO(" --------------------------------");
        if(dotProduct <0)
        {
                ROS_WARN("path is behaind");
   
		ROS_INFO("inplace rotaion found , doing reverse, size of path data %d ",x );
		for (int i{0};i<x;i++)
		{
			m_undock = true;
			for (int j{0};j<stripped_path_poses[i].size();j++)
			{
				if (robot_orientation)
				{
					stripped_path_poses[i][j][2] = m_robot_odom_yaw;
				}
				ROS_INFO("path_pose %f",stripped_path_poses[i][j][2]);

			}
			ROS_INFO("done reversing 1st path");
			/*	if(abs(m_robot_odom_yaw - next_yaw ) > 1.8)
				{
				for (int j{0};j<stripped_path_poses[i+1].size();j++)
				{
				if (robot_orientation)
				{
				stripped_path_poses[i+1][j][2] = m_robot_odom_yaw;
				}
				ROS_INFO("path_pose %f",stripped_path_poses[i+1][j][2]);

				}


				}*/
			if(i==0)
			{
				break;
			}		
		
		}
	}
	if(dotProduct >0) 
	{
	ROS_INFO("Path is infront setting undock false");
	m_undock = false;
	}

	}
}

void MissionScheduler::correct_orientation(std::vector<std::vector<std::vector<double>>>  &stripped_path_poses)
{
	int x{0},y{0};
	float x_ {0}, y_ {0}, yaw_ {0}, next_x{0},next_y{0},next_yaw{0},yaw_difference{0},diff_in_x{0},diff_in_y{0};

	x = stripped_path_poses.size();
	y = stripped_path_poses[x-1].size();
	ROS_INFO("x pos %d y pos: %d ",x,y);
	for (int i{0};i<x;i++)
	{
		for (int j{0};j<stripped_path_poses[i].size();j++)
		{
			x_ = stripped_path_poses[i][j][0];
			y_ = stripped_path_poses[i][j][1];
			yaw_ = stripped_path_poses[i][j][2];
			//	next_yaw = stripped_path_poses[i][j+1][2];
			//	yaw_difference  = (yaw_ - next_yaw);

			//ROS_INFO("diff_x : %f, diff_y : %f , diff_yaw %f",x_,y_,yaw_);

		}
	}
}


/** here we are gonna reverse the orientation of the poses so we can travel through it without much trouble in reverse direction**/
void MissionScheduler::reverse_orientation(std::vector<std::vector<std::vector<double>>>  &stripped_path_poses)
{
	int x{0},y{0};
	x = stripped_path_poses.size();
	y = stripped_path_poses[x-1].size();
	//ROS_INFO("x pos %d y pos: %d ",x,y);
	for (int i{0};i<x;i++)
	{
		for (int j{0};j<stripped_path_poses[i].size();j++)
		{


			stripped_path_poses[i][j][2] +=3.14 ;


		}

	}
}
void MissionScheduler::reached_pos_check(int m_pos_num, qr_navigation::goal_msg & pathData )
{
	ros::Rate rate(0.5);
	m_reached_pos = "reached_" + std::to_string(m_pos_num);
	std::cout << "m_reached_pos: " << m_reached_pos << " : " << m_pos_num << std::endl;

	//Check the robot reached last goal given,	
	while(m_goal_result != m_reached_pos) 
	{

		goal_msg_pub.publish(pathData);
		ROS_INFO("m_goal_result %s, m_reache pos %s ",m_goal_result.c_str(),m_reached_pos.c_str());
		if(as.isPreemptRequested() || !ros::ok()) 
		{
			ROS_INFO("%s: Preempted", action_name_.c_str());
			// set the action state to preempted
			as.setPreempted();
			break;
		}
		rate.sleep();
	}

}


void MissionScheduler::pub_path(std::string operation, std::vector<std::vector<std::vector<double>>>  &stripped_path_poses, bool Dock, std::string Type  )
{

	int z, y, x;


	std::string temp_operation{};
	//	check_straight_line(stripped_path_poses);
	check_inplace_rotation(stripped_path_poses,true);
	x = stripped_path_poses.size();//(stripped_path_poses.size() > 1) ? stripped_path_poses.size() - 1 : stripped_path_poses.size()-1;
	ROS_INFO("nubmer of poses %d ", x);
	ROS_INFO("m_undock status %d ", m_undock);	
	//if (operation =="UNDOCK" )
	//	ignore_orientaion(stripped_path_poses,true);
	//reverse_orientation(stripped_path_poses);
	int m_undock_count{0};
	for(int i {0}; i < x; i++) 
	{


		if(Dock  &&  i == (x-1))
		{
			ROS_INFO("operation : %s ",operation.c_str());
			temp_operation=Type;
			ignore_orientaion(stripped_path_poses);

		}	
		else if(Dock && i < (x-1))
		{
			ROS_INFO("operation1 : %s ",operation.c_str());

			temp_operation = "NORMAL";
		}
		else if(m_undock)
		{
			temp_operation = "UNDOCK";
			m_undock_count +=1;
			if(m_undock_count >0)	
			{
				m_undock = false;
				Type = "NORMAL";
			}										
			ROS_INFO("operation m_undock : %s ",operation.c_str());

		}
		else if(Type=="UNDOCK")
		{
			temp_operation = "UNDOCK";
			m_undock_count +=1;
			if(m_undock_count>0)
			{
				Type = "NORMAL";
			}
		}
		else  
		{
			ROS_INFO("operation2 : %s ",operation.c_str());

			temp_operation = Type;
		}


		y = stripped_path_poses[i].size() - 1;

		//	ROS_INFO("x,y,z %d : %d : ", x, y);
		ROS_INFO("path array %f : %f : %f", m_stripped_path_poses[i][y][0], m_stripped_path_poses[i][y][1],m_stripped_path_poses[i][y][2]);
		//ROS_INFO("size of stripped path %f ",m_stripped_path_poses[i][0][2]);


		pathData.pose = std::to_string(m_pos_num);
		pathData.start = { stripped_path_poses[i][0][0], stripped_path_poses[i][0][1] };
		if(i<(x-1) && x>1) 
		{
			double	next_array_size {0};

			next_array_size = stripped_path_poses[i+1].size() - 1;
			ROS_INFO( "size of the array %f ", next_array_size);
			if(next_array_size==0 && i <(x-2))
			{
				pathData.goal = { stripped_path_poses[i][y][0], stripped_path_poses[i][y][1],
					stripped_path_poses[i+2][0][2] };
			}
			pathData.goal = { stripped_path_poses[i][y][0], stripped_path_poses[i][y][1],
				stripped_path_poses[i+1][0][2] };
		}
		//	if(i==0 && x==1) 
		else
		{
			pathData.goal = { stripped_path_poses[i][y][0], stripped_path_poses[i][y][1],
				stripped_path_poses[i][y][2] };
		}
		ROS_INFO("operation %s ",temp_operation.c_str() );	
		pathData.operation = temp_operation;
		ROS_INFO("charging_mission_intiated %d current_loop %d , maxcount %d",charging_mission_intiated,i,x-1);
		if(charging_mission_intiated ==true && i ==(x-1))
		{
			ROS_INFO("undocking for charging");
			pathData.operation = "UNDOCK";
		}			
		reached_pos_check(m_pos_num,pathData);
		m_pos_num += 1;
		m_pos_num = (m_pos_num > 110) ? 100 : m_pos_num;
	}
	ROS_INFO("Done");
	m_goal_result = "";


}



void MissionScheduler::Charge_battery()
{
	ros::Rate rate(1);
	std_msgs::Int32 charge_battery_data;
	charge_battery_data.data = 1;
	charge_battery.publish(charge_battery_data);
	while(m_battery_percentage < 100 && as.isActive()) 
	{
		ROS_INFO("Charging --> battery %f", m_battery_percentage);
		// feedback.data = m_battery_percentage;
		m_robot_on_duty = 1; // robot is kept for charging which makes it not available for any other tasks
		if(m_battery_percentage > MAX_CHARGE) 
		{
			m_robot_on_duty = 0; // robot is kept for charging which makes it not available for any other tasks
			std::string status { "success" };
			ROS_INFO("fully Charged");
			// action_result.result = status;
			// as.setSucceeded();
			ROS_INFO("Goal executed successfully");
			action_result.success = "true";
			as.setSucceeded(action_result);
			break;
		}
		rate.sleep();
		// if (!as.isActive())
		// break;
		if(as.isPreemptRequested() || !ros::ok()) 
		{
			ROS_INFO("%s: Preempted", action_name_.c_str());
			// set the action state to preempted
			as.setPreempted();
			break;
		}
	}
	m_robot_on_duty = 0;
	charge_battery_data.data = 0;
	charge_battery.publish(charge_battery_data);

}
void MissionScheduler::charging_path_correction(std::vector<std::vector<std::vector<double>>>  &m_stripped_path_poses)
{
	ROS_INFO("size of stripped path poses %d ",m_stripped_path_poses.size());
	int total_num_path_ = m_stripped_path_poses.size();
	int last_path_size = m_stripped_path_poses[total_num_path_-1].size();
	for(int j =0; j<last_path_size; j++)
	{
		m_stripped_path_poses[total_num_path_-1][j][2] = goal_yaw_for_charging;
	}
	ROS_INFO("total_num_path_ %d ,last_path_size %d ",total_num_path_,last_path_size);
	//m_stripped_path_poses[path_size_ ][m_stripped_path_poses[ m_stripped_path_poses.size()-1].size()][2] = goal_yaw_for_charging;
	for(int i =0; i < m_stripped_path_poses.size(); i++)
	{
		for (int y=0;y< m_stripped_path_poses[i].size(); y++)
		{
			ROS_INFO("charging path currection x: %f, y: %f, yaw: %f", m_stripped_path_poses[i][y][0],m_stripped_path_poses[i][y][1],m_stripped_path_poses[i][y][2]);
		}
	}		
}

bool MissionScheduler::get_path_plan( const qr_mission_scheduler::goalMsg &goal)

{
	graph_msgs::GetGraphPlan robot_path_plan;
	robot_path_plan = get_path(int(goal.source),int(goal.destination));
	if(path_client.call(robot_path_plan)) 
	{
		ROS_INFO("recived path");
		response_data = robot_path_plan.response;
		ROS_INFO("recived path size : %ld ", response_data.plan.poses.size());
		if(response_data.plan.poses.size() ==0) return false;
		//HERE We Are splitting the path in to multiple path 
		check_straight_line(response_data);//,m_stripped_path_poses);

		MissionScheduler::split_path(response_data,m_stripped_path_poses);
		//Here we are gonna update the yaw with next pose yaw
		MissionScheduler::correct_orientation(m_stripped_path_poses);
		if(goal.goalType == "CHARGING")
		{
			charging_path_correction(m_stripped_path_poses);
			ROS_INFO("Charing is called, we have to modify the last path");
		}

		//TODO We have to update the path planner with the current location of the robot
		std_msgs::Int32 charge_battery_data;

		return true;
	}
	else 
	{
		ROS_WARN("Haven't recived any path, make sure path planner node is up ");
		return false;
	}

}

void MissionScheduler::execute_goal(const qr_mission_scheduler::goalMsg &goal)

{
	ROS_INFO("executing goal action ");
	ros::Rate rate(1);

	m_stripped_path_poses.clear(); // clearing the path array
	path_data_vec_1D.clear();
	path_data_vec_2D.clear();

	ROS_INFO("goalType %s ", goal.goalType.c_str());

	if(goal.goalType == "CHARGING") 
	{
		ROS_INFO(" Charging Action initiated ");

		if (get_path_plan(goal))
		{
			ROS_INFO("recived path, good to go");
			std::string operation_mode = goal.goalType;
			//ROS_INFO("size of stripped path %f ",m_stripped_path_poses[12][0][2]);
			charging_mission_intiated = true;
			pub_path(operation_mode,m_stripped_path_poses,false,"NORMAL");
			Charge_battery();

		}
		else 
		{
			ROS_WARN("Path Not Found --> Check the source and destination are correct");
		}


	}

	if(goal.goalType=="NORMAL")
	{
		if(m_battery_percentage > m_minimum_battery_threshold) 
		{
			ROS_INFO("Moving from source : %f ---> Target : %f ",goal.source,goal.destination);
			if (get_path_plan(goal))
			{
				ROS_INFO("recived path, good to go");
				std::string operation_mode = goal.goalType;
				//ROS_INFO("size of stripped path %f ",m_stripped_path_poses[12][0][2]);
				charging_mission_intiated = false;
				pub_path(operation_mode,m_stripped_path_poses,false,"NORMAL");
				std::string status { "success" };

				ROS_INFO("Goal executed successfully");
				action_result.success = "true";
				as.setSucceeded(action_result);

			}
		}
		else 
		{
			ROS_WARN("Not enough battery for the task please send the robot for charging");
			as.setPreempted();
			m_robot_on_duty = 0;
		}
		m_robot_on_duty = 0;
	}


	if(goal.goalType=="MNP")
	{
		//Goal is Move to a loction and pick

		ROS_INFO("Move and pick initiated");
		if(m_battery_percentage > m_minimum_battery_threshold) 
		{

			if (get_path_plan(goal))
			{
				ROS_INFO("recived path, good to go");
				std::string operation_mode = goal.goalType;
				//ROS_INFO("size of stripped path %f ",m_stripped_path_poses[12][0][2]);

				pub_path(operation_mode,m_stripped_path_poses,true,"UNDOCK");
				std::string status { "success" };

				ROS_INFO("Goal executed successfully");
				action_result.success = "true";
				as.setSucceeded(action_result);

			}
			else 
			{
				ROS_WARN("Path Not Found --> Check the source and destination are correct");
			}

		}
	}

	if(goal.goalType =="UNDOCK")
	{
		if (get_path_plan(goal))
		{
			//get robot orientation
			ROS_INFO("robot yaw --> %f ", m_robot_odom_yaw);

			std::string operation_mode = goal.goalType;
			//ROS_INFO("size of stripped path %f ",m_stripped_path_poses[12][0][2]);
			charging_mission_intiated = false;
			pub_path(operation_mode,m_stripped_path_poses,false,"UNDOCK");
			std::string status { "success" };

			ROS_INFO("Goal executed successfully");
			action_result.success = "true";
			as.setSucceeded(action_result);

		}


	}
	/*
	   if (goal.goalType == "MND")
	   {

	   }

	   if (goal.goalType =="PND")
	   {
	   if(m_battery_percentage > m_minimum_battery_threshold) 
	   {
	   ROS_INFO("Moving from source : %f ---> Target : %f ",goal.source,goal.destination);
	//TODO this functionality now work as a normal Move function, 
	//Once the palnner is updated to give path from the current location of the robot,
	//then this functionality can be updated in such a way that this follows a sequance of motion
	// go to pickup location
	// Dock --> Lift 
	// undock 
	// go to the drop location --> Drop
	// undock 
	//move to waiting location or next goal. 
	// This functions to be implemented
	//plan_and_execute("NORMAL",goal);
	ROS_INFO("Goal executed successfully");
	action_result.success = "success";
	as.setSucceeded(action_result);
	}
	else 
	{
	ROS_WARN("Not enough battery for the task please send the robot for charging");
	as.setPreempted();
	m_robot_on_duty = 0;
	}
	m_robot_on_duty = 0;

	}
	*/




	   else 
	   {
		   ROS_INFO("Invalid Goal Type");
	   }


	   if(!as.isActive() || as.isPreemptRequested())
		   return;

	   rate.sleep();
}


void MissionScheduler::OdomCB(const nav_msgs::Odometry& odom)
{
	m_robot_pose_data = odom;
	tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
			odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll{0},pitch{0};
	m.getRPY(roll, pitch, m_robot_odom_yaw);
	if(as.isActive()) {
		feedback.pose.clear();
		feedback.pose.push_back(float(m_robot_pose_data.pose.pose.position.x));
		feedback.pose.push_back(float(m_robot_pose_data.pose.pose.position.y));
		as.publishFeedback(feedback);

	}

	// ROS_INFO("odom pose %f , %f ", odom.pose.pose.position.x, odom.pose.pose.position.y);
}

void MissionScheduler::batteryCB(const std_msgs::Float32& msg)
{
	// if (!as.isActive())
	// return;
	// ROS_INFO("GOAL TYPE %s ",goalValue.goalType.c_str());

	// ROS_INFO("battery: %f ",msg.data);
	// ROS_INFO("odom pose %f , %f ", m_robot_pose_data.pose.pose.position.x, m_robot_pose_data.pose.pose.position.y);

	m_battery_percentage = msg.data;
	usleep(1000);
	if(m_battery_percentage > m_minimum_battery_threshold && m_robot_on_duty == 0) {
		m_robot_status_data.data = true;
	} 
	else 
	{
		m_robot_status_data.data = false;
	}
	robot_status.publish(m_robot_status_data);
	// feedback.data = m_battery_percentage;
}

void MissionScheduler::batteryCBData(const sensor_msgs::BatteryState& msg)
{

	m_battery_percentage = msg.percentage * 100;
	usleep(1000);
	if(m_battery_percentage > m_minimum_battery_threshold && m_robot_on_duty == 0) {
		m_robot_status_data.data = true;
	}
	else
	{
		m_robot_status_data.data = false;
	}
	ROS_INFO("m_robot_status_data --> %i ",m_robot_status_data);
	robot_status.publish(m_robot_status_data);

	//    if(m_battery_percentage > 0)
	//   {
	//      m_battery_percentage = m_battery_percentage*100;
	// }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mission_scheduler_server");
	ROS_INFO("Starting mission scheduler server");
	MissionScheduler mission_scheduler("mission_server");
	ros::Rate rate(10); // 10
	while(ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
