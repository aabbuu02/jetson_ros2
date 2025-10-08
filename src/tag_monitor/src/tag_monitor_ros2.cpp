/**
 * @file tag_monitor_ros2.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief ROS2 tag monitor matching ROS1 functionality
 */

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <memory>

class TagMonitor : public rclcpp::Node
{
public:
  TagMonitor()
  : Node("tag_monitor_node")
  {
    // Declare parameters
    this->declare_parameter("e_stop_topic", "/e_stop/tag_detection");
    this->declare_parameter("deinit_robot_topic", "/robot_initialized");
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("detected_tag_topic", "/detected_tag");
    this->declare_parameter("init_robot_topic", "/initialize_robot");
    this->declare_parameter("max_distance_without_tag", 5.0);
    this->declare_parameter("loop_rate", 10.0);

    // Get parameters
    m_e_stop_topic = this->get_parameter("e_stop_topic").as_string();
    m_deinit_robot_topic = this->get_parameter("deinit_robot_topic").as_string();
    m_odom_topic = this->get_parameter("odom_topic").as_string();
    m_det_tag_topic = this->get_parameter("detected_tag_topic").as_string();
    m_init_robot_topic = this->get_parameter("init_robot_topic").as_string();
    m_max_distance_without_tag = this->get_parameter("max_distance_without_tag").as_double();
    m_loop_rate = this->get_parameter("loop_rate").as_double();

    // Create publishers
    m_e_stop_pub = this->create_publisher<std_msgs::msg::Bool>(m_e_stop_topic, 10);
    m_init_robot_pub = this->create_publisher<std_msgs::msg::Bool>(m_deinit_robot_topic, 10);

    // Create subscribers
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      m_odom_topic, 10,
      std::bind(&TagMonitor::odometryCallback, this, std::placeholders::_1));
    
    m_detected_tag_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      m_det_tag_topic, 10,
      std::bind(&TagMonitor::detectedTagCallback, this, std::placeholders::_1));
    
    m_init_robot_sub = this->create_subscription<std_msgs::msg::Bool>(
      m_init_robot_topic, 10,
      std::bind(&TagMonitor::initializeRobotCallback, this, std::placeholders::_1));

    // Initialize TF2
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer);

    // Initialize variables
    m_is_robot_initialized = false;
    m_cur_dist_travelled = 0.0;
    m_e_stop_msg.data = false;

    RCLCPP_INFO(this->get_logger(), "Tag Monitor node has been started.");
  }

private:
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    // Update current pose
    m_cur_pose_vec.x = odom_msg->pose.pose.position.x;
    m_cur_pose_vec.y = odom_msg->pose.pose.position.y;
    m_cur_pose_vec.z = odom_msg->pose.pose.position.z;

    // Publish e_stop status
    m_e_stop_pub->publish(m_e_stop_msg);

    if (!m_is_robot_initialized) {
      m_prev_pose_vec = m_cur_pose_vec;
      m_cur_dist_travelled = 0.0;
      return;
    }

    // Calculate distance travelled
    double dx = m_cur_pose_vec.x - m_prev_pose_vec.x;
    double dy = m_cur_pose_vec.y - m_prev_pose_vec.y;
    double distance = sqrt(dx * dx + dy * dy);
    m_cur_dist_travelled += distance;

    // Check if robot has travelled too far without tag detection
    if (m_cur_dist_travelled > m_max_distance_without_tag) {
      m_e_stop_msg.data = true;
      RCLCPP_WARN(this->get_logger(), "Robot travelled too far without tag detection! Emergency stop activated.");
    }

    // Update previous pose
    m_prev_pose_vec = m_cur_pose_vec;
  }

  void detectedTagCallback(const geometry_msgs::msg::PoseStamped::SharedPtr tag_msg)
  {
    // Reset distance counter when tag is detected
    m_cur_dist_travelled = 0.0;
    m_e_stop_msg.data = false;
    
    RCLCPP_DEBUG(this->get_logger(), "Tag detected, resetting distance counter.");
  }

  void initializeRobotCallback(const std_msgs::msg::Bool::SharedPtr init_msg)
  {
    m_is_robot_initialized = init_msg->data;
    
    if (m_is_robot_initialized) {
      RCLCPP_INFO(this->get_logger(), "Robot initialized for tag monitoring.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Robot deinitialized from tag monitoring.");
    }
  }

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_e_stop_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_init_robot_pub;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_detected_tag_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_init_robot_sub;

  // TF2
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;

  // Member variables
  std::string m_e_stop_topic;
  std::string m_deinit_robot_topic;
  std::string m_odom_topic;
  std::string m_det_tag_topic;
  std::string m_init_robot_topic;
  double m_max_distance_without_tag;
  double m_loop_rate;

  bool m_is_robot_initialized = false;
  double m_cur_dist_travelled = 0.0;
  std_msgs::msg::Bool m_e_stop_msg;

  struct Pose3D {
    double x, y, z;
  } m_cur_pose_vec, m_prev_pose_vec;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TagMonitor>());
  rclcpp::shutdown();
  return 0;
}