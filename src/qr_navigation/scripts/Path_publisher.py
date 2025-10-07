#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from agv_navigation_qr.msg import goal_msg
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
from geometry_msgs.msg import Pose2D

from std_msgs.msg import String



goal_msg_pub = rospy.Publisher('/goal_path',goal_msg,queue_size=10)
goal_segment_completion_ack  = rospy.Publisher('segment_completion_ack',Pose2D,queue_size=10)

position_x =0 
position_y = 0
yaw = 0
poses = []
flag = False
current_pose =[]






def GoalResultCallback(msg):
	print(msg)
	global flag
	global count
	global poses
	global current_pose
	print(poses)
	print("len fo poses ",len(poses))
	for i in range(0,len(poses)):
		data_message = "reached_pos"+str(i)	
		print("data_message ", data_message)
		if (str(msg.data) ==data_message):
	
			if i >= len(poses)-1:
				print(poses[0])
				next_pose = "next_pos"+str(0+1)
				current_pose = poses[0]
				#print(next_pose)
				flag = True
			else:
				goal_ack = Pose2D()
				goal_ack.x = current_pose[i][1]
				goal_ack.y = current_pose[i][2]
				goal_ack.theta = current_pose[i][3]
				goal_segment_completion_ack.publish(goal_ack)
				print("default looop")
				print(poses[i])
				current_pose = poses[i+1] 
				next_pose ="next_pose"+str(i+1)
				print(next_pose)
				flag = True
		else:
			flag = False





def Path_publisher():
	global position_x
	global position_y
	global yaw
	fleet_path = rospy.Subscriber('/pick_robot/robot_1/path', Path, Path_Callback )
	goal_result = rospy.Subscriber('/goal_result', String, GoalResultCallback )
		

	# Initialize node
	rospy.init_node('fleet_path_publisher', anonymous=True)
	
	# Set rate
	r = rospy.Rate(1) # 10 Hz
	# Wait for goal pose
	rospy.loginfo("Path sequance publisher started")

	# Publish the path continuously
	while not rospy.is_shutdown():
		#pathPub.publish(path)	
		#marker_pub(0,0)
	
		#goal_msg_ = goal_msg()
		#goal_msg_.pose = "pos0"
		#goal_msg_.start    = [16.7,0.86]
		#goal_msg_.goal  = [float(position_x),float(position_y),float(yaw)]
		#goal_msg_.operation = "NORMAL"
		#time.sleep(2)
		#goal_msg_pub.publish(goal_msg_)
                    
		#print(poses)	
		#print(current_pose)
		if(current_pose):
                    print(current_pose)
                    
                    goal_msg_ = goal_msg()
                    goal_msg_.pose = str(current_pose[0])
                    goal_msg_.start    = [16.7,0.86]
                    goal_msg_.goal  = [float(current_pose[1]),float(current_pose[2]),float(current_pose[3])]
                    goal_msg_.operation = str(current_pose[4])
                    time.sleep(2)
                    goal_msg_pub.publish(goal_msg_)
		
		r.sleep()
		
    
def Path_Callback(msg):
	print(msg)
	#print(len(msg.poses))
	global position_x
	global position_y
	global yaw
	m = len(msg.poses)
	n = 5
	global poses
	pose_array = [[0 for x in range(n)] for x in range(m)]
	print(msg.poses[0].pose.position.y)
	print(len(msg.poses))
	for i in range(0,len(msg.poses)):
		#print(i)
		#print(len(msg.poses))
		pose_array[i][0] = "pos"+str(i)
		pose_array[i][1] = msg.poses[i].pose.position.x
		pose_array[i][2] =  msg.poses[i].pose.position.y
		orientation_z = msg.poses[i].pose.orientation.z
		orientation_w = msg.poses[i].pose.orientation.w
		#print("position_x : ",msg.poses[i].pose.position.x)
		#print("position_y : ",msg.poses[i].pose.position.y)
		
		orientation_list = [msg.poses[i].pose.orientation.x,msg.poses[i].pose.orientation.y, msg.poses[i].pose.orientation.z, msg.poses[i].pose.orientation.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		#print(yaw)
		pose_array[i][3] = yaw
		pose_array[i][4] = "NORMAL"
	
	#print(pose_array)
	poses = pose_array
	




if __name__ == "__main__":
	try:
		Path_publisher()
	except rospy.ROSInterruptException:
		pass
