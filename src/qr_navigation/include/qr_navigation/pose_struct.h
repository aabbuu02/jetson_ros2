#ifndef POSE_STRUCT_H
#define POSE_STRUCT_H
namespace pose_struct
{
	//pose container
	struct Position
	{
		double x;
		double y;
		double z;
	};
	
	//orientation container
	struct Euler
	{
		double roll;
		double pitch;
		double yaw;
	};
	
	//Quaternion contatiner
	struct Quaternion
	{
		double x;
		double y;
		double z;
		double w;
	};
	
	//container for holding pose informations
	struct PoseQuaternion
	{
		Position position;
		Quaternion orientation;
	};
	
	struct PoseEuler
	{
		Position position;
		Euler orientation;
	};
	
	// path data container of  the robot
	struct PathData
	{
		PoseEuler init_pose;
		PoseEuler goal_pose;
	};
	
	//container for holding frame relations
	struct Transform
	{
		Position origin;
		Euler rotation;
	};
	
	struct LinearVelocity
	{
		double x;
		double y; 
		double z;
	};
	
	struct AngularVelocity
	{
		double x;
		double y;
		double z;
	};
	
	struct ControlInput
	{
		LinearVelocity linear;
		AngularVelocity angular;
	};

    struct Matrix
    {
        double **array;
        int rows;
        int columns;
    };
}
#endif
