#include "pose_struct.h"
#include <iostream>



class RotationController
{
	public:
		RotationController(double max_angular_vel, double min_angular_vel, double gain);
		~RotationController();
		double max_angular_velocity;
		double min_angular_velocity;
		double control_gain;
		bool rotate_to_angle(double desired_theta, pose_struct::ControlInput &control);


};


