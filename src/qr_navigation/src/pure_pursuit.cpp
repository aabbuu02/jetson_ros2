#include "pure_pursuit.h"
#include <math.h>
#include <unistd.h>
PurePursuit::PurePursuit()

{

	robot_pose_.position.x = 0;
	robot_pose_.position.y = 0;
	robot_pose_.orientation.yaw = 0;


	goal_pose_.position.x =0;
	goal_pose_.position.y =0;
	goal_pose_.orientation.yaw = 0;

	start_pose_.position.x =0;
	start_pose_.position.y =0;
	start_pose_.orientation.yaw = 0;

	path_array_size_  =0;
	path_data_.rows   =1;
	path_data_.columns =1;
	min_look_ahead_distance = 0.4;
	max_look_ahead_distance = 1.0;
	min_distance_index = 0;
	previous_index = 0;
	wheel_base  = 0.50;
	steering_angle = 0.0;
	curvature = 0.0;
	angular_velocity = 0.0;
	max_angular_vel = 0.4;
	kla = 1.1;
	velocity_change = 0.0;
}



PurePursuit::~PurePursuit(){}


void PurePursuit::Update_params(float min_look_ahead_distance_,float max_look_ahead_distance_, float wheel_base_, float kla_,float max_angular_vel_)
{
	//ROS_INFO("updated purepursuit parameters ");
	min_look_ahead_distance = min_look_ahead_distance_;
	max_look_ahead_distance = max_look_ahead_distance_;
	wheel_base  = wheel_base_;
	max_angular_vel = max_angular_vel_;
	kla = kla_;

}



void PurePursuit::Update_path( const pose_struct::PoseEuler &start_pose,
                                const pose_struct::PoseEuler &goal_pose,
                                const pose_struct::Matrix &path_data,
				const int &path_array_size)
{

		ROS_INFO("recived updated path, updated path_array_size %d :",path_array_size);
		
		start_pose_ = start_pose;
	
		goal_pose_  = goal_pose;
		
		path_data_ = path_data;	
		path_array_size_ = path_array_size;
    if(path_array_size > 1000)
        {
		min_distance_index = 500;
        }
    else 
        {
        min_distance_index = 100;
        }
}

int PurePursuit::Index_Of_Smallest_Element(float array[], int size)
{
  int index = 0 ;
  double n = array[0] ;
  for (int i = 1; i < size; ++i)
  {
    if (array[i] < n)
    {
        n = array[i] ;
        index = i ;
    }
  }
 return index;
}



void PurePursuit::Intermediate_goal_point(const pose_struct::PoseEuler &robot_pose,const float current_linear_velocity)
{
	
	//ROS_INFO("pose_x : %f , pose_y : %f , pose_theta : %f ",robot_pose.position.x,robot_pose.position.y,robot_pose.orientation.yaw);	
//	ROS_INFO("intermediate_goal_posint");
	robot_pose_=robot_pose;
	float distance_array[path_array_size_];
 	 
	for (int i=0;i<path_array_size_;i++)
	{
		float id_x,id_y;
		id_x = robot_pose.position.x - *(path_data_.array[0]+i);
		id_y = robot_pose.position.y - *(path_data_.array[1]+i);	
	

		distance= sqrt(pow(id_x,2)+pow(id_y,2));
		distance_array[i] = distance;
	}
 	
	min_distance_index = Index_Of_Smallest_Element(distance_array,path_array_size_);
//	ROS_INFO("min_distance_index %d ",min_distance_index);
	locus = 0.0;
	double look_ahead_distance;
	look_ahead_distance =  dynamic_look_ahead(current_linear_velocity);

	while (look_ahead_distance > locus && (min_distance_index+1) < path_array_size_ )
		{
			dx = *(path_data_.array[0]+(min_distance_index+1)) - *(path_data_.array[0]+min_distance_index); 
			dy = *(path_data_.array[1]+(min_distance_index+1)) - *(path_data_.array[1]+min_distance_index);
			locus = locus + sqrt(pow(dx,2) + pow(dy,2));
			min_distance_index +=1;
//			ROS_INFO("Locus %f ",locus);
				
		}	
//	ROS_INFO("min_distance_index : %d :", min_distance_index);
	
}


float PurePursuit::dynamic_look_ahead(double linear_velocity)
{
	
	velocity_change = min_look_ahead_distance /kla;
	
	if (fabs(linear_velocity)  < velocity_change)
		dynamic_look_ahead_distance = min_look_ahead_distance;
	else
		dynamic_look_ahead_distance = 1.4 *kla *fabs(linear_velocity);

	if (dynamic_look_ahead_distance > max_look_ahead_distance)
		dynamic_look_ahead_distance = max_look_ahead_distance;
	if (dynamic_look_ahead_distance < min_look_ahead_distance)
		dynamic_look_ahead_distance = min_look_ahead_distance;
	//std::cout<< "dynamic " <<dynamic_look_ahead_distance<<"  velocity --> "<<abs(linear_velocity)<<std::endl;
	return dynamic_look_ahead_distance;
}




pose_struct::ControlInput  PurePursuit::Pure_pursuit_controller(double linear_velocity)
{

	//ROS_INFO("min_lad :%f, max_lad :%f, wheel_base: %f, max_angular_vel: %f, kla %f: ",min_look_ahead_distance,max_look_ahead_distance,wheel_base,max_angular_vel,kla);

	float x_pose,y_pose,alpha;
	/*if (previous_index >=min_distance_index)
	{
		min_distance_index = previous_index;

	}
*/
	if(path_array_size_ !=0)
	{
//	ROS_INFO("path_data %ld ", min_distance_index);
	x_pose = *(path_data_.array[0]+min_distance_index);
	y_pose = *(path_data_.array[1]+min_distance_index);
	
//	float diff_in_y {0.0}; 
	//float diff_in_x {0.0};

	//diff_in_y = y_pose - robot_pose_.position.y;
	//diff_in_x = x_pose - robot_pose_.position.x;
	
	//if( abs(diff_in_y) < 0.02 && abs(diff_in_x) <0.02 ) // or less than minimum lookahead distance
	//{
		
		
	//}
	
	//ROS_INFO("pathX: %f, pathY: %f, robotX: %f, robotY: %f, robotYaw: %f",x_pose,y_pose, robot_pose_.position.x,robot_pose_.position.y,robot_pose_.orientation.yaw );


	alpha  = atan2(y_pose - robot_pose_.position.y, x_pose - robot_pose_.position.x)-robot_pose_.orientation.yaw;
 	//ROS_INFO("alpha data : %f , robot orientation %f",alpha,robot_pose_.orientation.yaw);	
/*---------------------------------------------*/
	lateral_shift = min_look_ahead_distance * sin(alpha);
	//control_input.linear.y= lateral_shift;
/* -------------------------------------------*/
	//steering_angle  = atan2(2.0 * wheel_base * lateral_shift / look_ahead_distance, 1.0);
	//ROS_INFO("steering_angle: %f",steering_angle);
	double d_la_distance;

	d_la_distance = dynamic_look_ahead(linear_velocity);
//    lateral_shift = d_la_distance * sin(alpha);
    control_input.linear.y= lateral_shift;
    control_input.linear.z = d_la_distance;	
    ROS_INFO("look ahead distance %f ",d_la_distance);
	curvature =    (2*lateral_shift) / pow(d_la_distance,2);
	//ROS_INFO("curvature : %f , lateral_shift:  %f , dynamic look_ahead_distance: %f ", curvature,lateral_shift,d_la_distance);

	
	
	
        angular_velocity = linear_velocity * curvature;
        //ROS_INFO("curvature %f ", curvature);

        if (angular_velocity > max_angular_vel)
                angular_velocity = max_angular_vel;
        else if(angular_velocity < -max_angular_vel)
                angular_velocity = -max_angular_vel;

       control_input.linear.x = linear_velocity; //control_input.linear_vel;

       control_input.angular.z = angular_velocity;

	//ROS_INFO("linear velocity : %f , angular_velocity:  %f ", linear_velocity,angular_velocity);



	
	previous_index = min_distance_index;

	}
	return control_input;
}
