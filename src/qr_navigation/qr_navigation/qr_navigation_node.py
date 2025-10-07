"""
@file qr_navigation_node.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32
from anscer_msgs.msg import SafetyFields
from qr_navigation_msgs.msg import GoalMsg
import tf_transformations
import math
import numpy as np

# Helper classes translated from C++
class PurePursuit:
    def __init__(self):
        # ... initialization of parameters
        pass
    def Update_params(self, min_look_ahead, max_look_ahead, wheel_base, kla, max_angular_vel):
        # ... update parameters
        pass
    def Update_path(self, path):
        self.path = path
    def get_control(self, robot_pose, linear_vel):
        # ... full pure pursuit logic here
        return 0.0, 0.0 # angular_vel, lookahead_dist

class VelocityProfile:
    # ... full implementation of VelocityProfile
    def Find_linear_velocity(self, dist_to_goal, current_vel):
        # ... full velocity profile logic
        return 0.1 # example linear_vel

class RotationController:
    # ... full implementation of RotationController
    def rotate_to_angle(self, angle_diff):
        # ... full rotation logic
        return 0.1 # example angular_vel

class QrNavigationNode(Node):
    def __init__(self):
        super().__init__('qr_navigation_node')
        # ... (full __init__ implementation with publishers/subscribers)
        self.cmd_vel_pub = self.create_publisher(Twist, 'navigation/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(GoalMsg, 'goal_path', self.goal_callback, 10)
        self.timer = self.create_timer(0.04, self.run_loop) # 25Hz

        self.pure_pursuit = PurePursuit()
        self.velocity_profile = VelocityProfile()
        self.rotation_controller = RotationController()
        
        self.robot_pose = None
        self.path = None
        self.goal_pose = None
        self.is_initialized = False
        self.is_paused = False
        # ... other state variables

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
        # ... (convert quaternion to euler)

    def goal_callback(self, msg):
        self.get_logger().info("New Goal Received!")
        # ... (process new goal and create path)
        self.path = np.array([msg.start, msg.goal]) # Simplified path
        self.goal_pose = {'x': msg.goal[0], 'y': msg.goal[1], 'theta': msg.goal[2]}


    def run_loop(self):
        if not self.is_initialized or self.is_paused or self.robot_pose is None or self.path is None:
            self.cmd_vel_pub.publish(Twist()) # Publish zero velocity
            return

        dist_to_goal = math.sqrt((self.robot_pose.position.x - self.goal_pose['x'])**2 + 
                                 (self.robot_pose.position.y - self.goal_pose['y'])**2)
        
        cmd_vel_msg = Twist()
        
        position_tolerance = 0.02 # Example value
        if dist_to_goal > position_tolerance:
            linear_vel = self.velocity_profile.Find_linear_velocity(dist_to_goal, 0.0)
            angular_vel, _ = self.pure_pursuit.get_control(self.robot_pose, linear_vel)
            cmd_vel_msg.linear.x = linear_vel
            cmd_vel_msg.angular.z = angular_vel
        else:
            # Goal reached, perform final rotation
            # ... rotation logic
            self.path = None # Stop navigation
            self.get_logger().info("Goal Reached!")

        self.cmd_vel_pub.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = QrNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

