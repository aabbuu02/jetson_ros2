/**
 * @file acr_robot_controller_ros2.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief ROS2 ACR robot controller matching ROS1 functionality
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "anscer_msgs/msg/motor_diagnostics_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <memory>
#include <string>

class ACRRobotController : public rclcpp::Node
{
public:
  ACRRobotController()
  : Node("acr_robot_controller_node")
  {
    // Declare parameters
    this->declare_parameter("cmd_topic", "/cmd_vel");
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("emergency_stop_topic", "/e_stop");
    this->declare_parameter("emergency_button_topic", "/e_stop/button");
    this->declare_parameter("controller_type", 1);
    this->declare_parameter("roboteq_port", "/dev/roboteq");
    this->declare_parameter("roboteq_baudrate", 115200);
    this->declare_parameter("plc_ip", "192.168.1.125");
    this->declare_parameter("plc_port", 502);
    this->declare_parameter("wheel_separation", 0.90);
    this->declare_parameter("wheel_radius", 0.101);
    this->declare_parameter("gear_ratio", 9.0);
    this->declare_parameter("linear_vel_limit", 1.0);
    this->declare_parameter("angular_vel_limit", 1.0);
    this->declare_parameter("publish_tf", false);
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_footprint");

    // Get parameters
    m_cmd_topic = this->get_parameter("cmd_topic").as_string();
    m_odom_topic = this->get_parameter("odom_topic").as_string();
    m_emergency_stop_topic = this->get_parameter("emergency_stop_topic").as_string();
    m_emergency_button_topic = this->get_parameter("emergency_button_topic").as_string();
    m_controller_type = this->get_parameter("controller_type").as_int();
    m_roboteq_port = this->get_parameter("roboteq_port").as_string();
    m_roboteq_baudrate = this->get_parameter("roboteq_baudrate").as_int();
    m_plc_ip = this->get_parameter("plc_ip").as_string();
    m_plc_port = this->get_parameter("plc_port").as_int();
    m_wheel_separation = this->get_parameter("wheel_separation").as_double();
    m_wheel_radius = this->get_parameter("wheel_radius").as_double();
    m_gear_ratio = this->get_parameter("gear_ratio").as_double();
    m_linear_vel_limit = this->get_parameter("linear_vel_limit").as_double();
    m_angular_vel_limit = this->get_parameter("angular_vel_limit").as_double();
    m_publish_tf = this->get_parameter("publish_tf").as_bool();
    m_odom_frame = this->get_parameter("odom_frame").as_string();
    m_base_frame = this->get_parameter("base_frame").as_string();

    // Create publishers
    m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(m_odom_topic, 10);
    m_emergency_pub = this->create_publisher<std_msgs::msg::Bool>(m_emergency_button_topic, 10);
    
    if (m_controller_type == 1) { // Modbus/Roboteq
      m_motor_diag_pub = this->create_publisher<anscer_msgs::msg::MotorDiagnosticsArray>("/motor_diagnostics", 10);
    }

    // Create subscribers
    m_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      m_cmd_topic, 10,
      std::bind(&ACRRobotController::cmdVelCallback, this, std::placeholders::_1));
    
    m_e_stop_sub = this->create_subscription<std_msgs::msg::Bool>(
      m_emergency_stop_topic, 10,
      std::bind(&ACRRobotController::eStopCallback, this, std::placeholders::_1));

    // Create TF broadcaster
    if (m_publish_tf) {
      m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

    // Initialize motor controller based on type
    initializeMotorController();

    // Create timer for odometry publishing
    m_odom_timer = this->create_wall_timer(
      std::chrono::milliseconds(50), // 20Hz
      std::bind(&ACRRobotController::publishOdometry, this));

    RCLCPP_INFO(this->get_logger(), "ACR Robot Controller node has been started.");
  }

private:
  void initializeMotorController()
  {
    switch (m_controller_type) {
      case 1: // Modbus/Roboteq
        RCLCPP_INFO(this->get_logger(), "Initializing Modbus/Roboteq controller");
        // Initialize Modbus connection
        break;
      case 2: // ODrive
        RCLCPP_INFO(this->get_logger(), "Initializing ODrive controller");
        // Initialize ODrive connection
        break;
      case 3: // Direct Roboteq
        RCLCPP_INFO(this->get_logger(), "Initializing Direct Roboteq controller");
        // Initialize direct Roboteq connection
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown controller type: %d", m_controller_type);
        break;
    }
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (m_e_stop_active) {
      RCLCPP_WARN(this->get_logger(), "Emergency stop active, ignoring velocity commands");
      return;
    }

    // Apply velocity limits
    double linear_vel = std::max(-m_linear_vel_limit, std::min(m_linear_vel_limit, msg->linear.x));
    double angular_vel = std::max(-m_angular_vel_limit, std::min(m_angular_vel_limit, msg->angular.z));

    // Convert to motor commands based on controller type
    switch (m_controller_type) {
      case 1: // Modbus/Roboteq
        sendModbusCommands(linear_vel, angular_vel);
        break;
      case 2: // ODrive
        sendODriveCommands(linear_vel, angular_vel);
        break;
      case 3: // Direct Roboteq
        sendDirectRoboteqCommands(linear_vel, angular_vel);
        break;
    }
  }

  void eStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    m_e_stop_active = msg->data;
    
    if (m_e_stop_active) {
      RCLCPP_WARN(this->get_logger(), "Emergency stop activated!");
      // Stop all motors immediately
      stopAllMotors();
    } else {
      RCLCPP_INFO(this->get_logger(), "Emergency stop deactivated");
    }
  }

  void sendModbusCommands(double linear_vel, double angular_vel)
  {
    // Convert to wheel velocities
    double left_wheel_vel = (linear_vel - angular_vel * m_wheel_separation / 2.0) / m_wheel_radius;
    double right_wheel_vel = (linear_vel + angular_vel * m_wheel_separation / 2.0) / m_wheel_radius;

    // Convert to motor RPM
    double left_motor_rpm = left_wheel_vel * m_gear_ratio * 60.0 / (2.0 * M_PI);
    double right_motor_rpm = right_wheel_vel * m_gear_ratio * 60.0 / (2.0 * M_PI);

    // Send Modbus commands (implement actual Modbus communication)
    RCLCPP_DEBUG(this->get_logger(), "Sending Modbus commands: L=%.2f, R=%.2f RPM", left_motor_rpm, right_motor_rpm);
  }

  void sendODriveCommands(double linear_vel, double angular_vel)
  {
    // Convert to wheel velocities
    double left_wheel_vel = (linear_vel - angular_vel * m_wheel_separation / 2.0) / m_wheel_radius;
    double right_wheel_vel = (linear_vel + angular_vel * m_wheel_separation / 2.0) / m_wheel_radius;

    // Send ODrive commands (implement actual ODrive communication)
    RCLCPP_DEBUG(this->get_logger(), "Sending ODrive commands: L=%.2f, R=%.2f", left_wheel_vel, right_wheel_vel);
  }

  void sendDirectRoboteqCommands(double linear_vel, double angular_vel)
  {
    // Convert to wheel velocities
    double left_wheel_vel = (linear_vel - angular_vel * m_wheel_separation / 2.0) / m_wheel_radius;
    double right_wheel_vel = (linear_vel + angular_vel * m_wheel_separation / 2.0) / m_wheel_radius;

    // Convert to motor RPM
    double left_motor_rpm = left_wheel_vel * m_gear_ratio * 60.0 / (2.0 * M_PI);
    double right_motor_rpm = right_wheel_vel * m_gear_ratio * 60.0 / (2.0 * M_PI);

    // Send direct Roboteq commands (implement actual serial communication)
    RCLCPP_DEBUG(this->get_logger(), "Sending Direct Roboteq commands: L=%.2f, R=%.2f RPM", left_motor_rpm, right_motor_rpm);
  }

  void stopAllMotors()
  {
    // Stop all motors immediately
    RCLCPP_WARN(this->get_logger(), "Stopping all motors due to emergency stop");
  }

  void publishOdometry()
  {
    // Publish odometry (implement actual odometry calculation)
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = m_odom_frame;
    odom_msg.child_frame_id = m_base_frame;
    
    // Set pose and twist (dummy values for now)
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;
    
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;
    
    m_odom_pub->publish(odom_msg);
  }

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_emergency_pub;
  rclcpp::Publisher<anscer_msgs::msg::MotorDiagnosticsArray>::SharedPtr m_motor_diag_pub;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_e_stop_sub;

  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

  // Timer
  rclcpp::TimerBase::SharedPtr m_odom_timer;

  // Member variables
  std::string m_cmd_topic;
  std::string m_odom_topic;
  std::string m_emergency_stop_topic;
  std::string m_emergency_button_topic;
  int m_controller_type;
  std::string m_roboteq_port;
  int m_roboteq_baudrate;
  std::string m_plc_ip;
  int m_plc_port;
  double m_wheel_separation;
  double m_wheel_radius;
  double m_gear_ratio;
  double m_linear_vel_limit;
  double m_angular_vel_limit;
  bool m_publish_tf;
  std::string m_odom_frame;
  std::string m_base_frame;
  bool m_e_stop_active = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ACRRobotController>());
  rclcpp::shutdown();
  return 0;
}