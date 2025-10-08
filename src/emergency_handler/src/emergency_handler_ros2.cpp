/**
 * @file emergency_handler_ros2.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief ROS2 emergency handler matching ROS1 functionality
 */

#include "rclcpp/rclcpp.hpp"
#include "anscer_msgs/msg/safety_fields.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include <memory>

class EmergencyHandler : public rclcpp::Node
{
public:
  EmergencyHandler()
  : Node("emergency_handler_node")
  {
    // Create publishers
    e_stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/e_stop", 10);
    e_stop_lidar_pub_ = this->create_publisher<std_msgs::msg::Bool>("/e_stop/lidar", 10);
    e_stop_button_pub_ = this->create_publisher<std_msgs::msg::Bool>("/e_stop/buttons", 10);
    emergency_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/emergency_cmd_vel", 10);

    // Create subscribers - matching ROS1 implementation
    detection_sub_ = this->create_subscription<anscer_msgs::msg::SafetyFields>(
      "/obstacle_detection", 10, 
      std::bind(&EmergencyHandler::detectionCallback, this, std::placeholders::_1));
    
    button_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/e_stop/button", 10,
      std::bind(&EmergencyHandler::eStopButtonCallback, this, std::placeholders::_1));
    
    webapp_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/e_stop/webApp", 10,
      std::bind(&EmergencyHandler::eStopWebAppCallback, this, std::placeholders::_1));
    
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&EmergencyHandler::cmdVelCallback, this, std::placeholders::_1));
    
    key_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/joy/cmd_vel", 10,
      std::bind(&EmergencyHandler::keycmdVelCallback, this, std::placeholders::_1));
    
    tag_detection_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/e_stop/tag_detection", 10,
      std::bind(&EmergencyHandler::tagDetFailCallback, this, std::placeholders::_1));

    // Create timer for publishing messages (matching ROS1 rate)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), // 20Hz like ROS1
      std::bind(&EmergencyHandler::publishMessages, this));

    RCLCPP_INFO(this->get_logger(), "Emergency Handler Node has been started with full ROS1 functionality.");
  }

private:
  // Callback functions
  void detectionCallback(const anscer_msgs::msg::SafetyFields::SharedPtr msg)
  {
    if (!msg->safety.empty()) {
      m_lidar_forward = msg->safety[0].is_front;
      m_lidar_backward = msg->safety[0].is_rear;
    }
  }

  void eStopButtonCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    m_e_stop_button = msg->data;
  }

  void eStopWebAppCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    m_e_stop_webapp = msg->data;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    m_linear_vel = msg->linear.x;
  }

  void keycmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    m_key_linear_vel = msg->linear.x;
    m_key_angular_vel = msg->angular.z;
  }

  void tagDetFailCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    m_e_stop_tag_detection = msg->data;
  }

  void publishMessages()
  {
    std_msgs::msg::Bool e_stop_msg, e_stop_lidar_msg, e_stop_button_msg;
    geometry_msgs::msg::Twist emergency_cmd_vel_msg;
    
    bool estop_lidar = false;
    bool cmd_vel_lidar = ((m_lidar_forward && (m_linear_vel > 0.0)) || 
                         (m_lidar_backward && (m_linear_vel < 0.0)));
    bool stopped_cmd_vel_lidar = ((m_lidar_forward && (m_linear_vel == 0.0)) || 
                                 (m_lidar_backward && (m_linear_vel == 0.0)));
    bool key_cmd_vel_lidar = ((m_lidar_forward && (m_key_linear_vel < 0.0)) || 
                             (m_lidar_backward && (m_key_linear_vel > 0.0)));
    
    bool key_cmd_tag_detection = ((m_key_linear_vel != 0.0) || (m_key_angular_vel != 0.0));

    if (cmd_vel_lidar || stopped_cmd_vel_lidar) {
      estop_lidar = true;
    }

    if (key_cmd_vel_lidar) {
      estop_lidar = false;
    }

    if (key_cmd_tag_detection) {
      m_e_stop_tag_detection = false;
    }

    // Button emergency stop
    if (m_e_stop_button || m_e_stop_webapp) {
      e_stop_button_msg.data = true;
    } else {
      e_stop_button_msg.data = false;
    }

    // Lidar emergency stop
    if (estop_lidar) {
      e_stop_lidar_msg.data = true;
    } else {
      e_stop_lidar_msg.data = false;
    }

    // Overall emergency stop
    if (estop_lidar || m_e_stop_button || m_e_stop_webapp || m_e_stop_tag_detection) {
      e_stop_msg.data = true;
      // Send zero velocity command
      emergency_cmd_vel_msg.linear.x = 0.0;
      emergency_cmd_vel_msg.angular.z = 0.0;
    } else {
      e_stop_msg.data = false;
    }

    // Publish all messages
    e_stop_pub_->publish(e_stop_msg);
    e_stop_button_pub_->publish(e_stop_button_msg);
    e_stop_lidar_pub_->publish(e_stop_lidar_msg);
    emergency_cmd_vel_pub_->publish(emergency_cmd_vel_msg);
  }

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr e_stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr e_stop_lidar_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr e_stop_button_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr emergency_cmd_vel_pub_;

  // Subscribers
  rclcpp::Subscription<anscer_msgs::msg::SafetyFields>::SharedPtr detection_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr webapp_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr key_cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tag_detection_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Member variables
  bool m_lidar_forward = false;
  bool m_lidar_backward = false;
  bool m_e_stop_button = false;
  bool m_e_stop_webapp = false;
  bool m_e_stop_tag_detection = false;
  double m_key_linear_vel = 0.0;
  double m_key_angular_vel = 0.0;
  double m_linear_vel = 0.0;
  std_msgs::msg::Bool e_stop_msg;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmergencyHandler>());
  rclcpp::shutdown();
  return 0;
}