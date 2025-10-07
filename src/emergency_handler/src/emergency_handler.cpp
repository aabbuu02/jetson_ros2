/**
 * @file emergency_handler.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "rclcpp/rclcpp.hpp"
#include "anscer_msgs/msg/safety_field.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>

// In ROS 2, nodes are classes that inherit from rclcpp::Node
class EmergencyHandler : public rclcpp::Node
{
public:
  // The constructor is where we initialize the node, publishers, and subscribers
  EmergencyHandler()
  : Node("emergency_handler_node")
  {
    // Create the publisher. The API is slightly different from ROS 1.
    // CORRECTED: Changed '/' to '::' for the message type
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/emergency_cmd_vel", 10);

    // Create the subscriber. The syntax uses a template and std::bind.
    // CORRECTED: Changed '/' to '::' for the message type
    subscription_ = this->create_subscription<anscer_msgs::msg::SafetyField>(
      "/safety_field_status", 10, std::bind(&EmergencyHandler::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Emergency Handler Node has been started.");
  }

private:
  // The callback function now takes a shared pointer to the message
  void topic_callback(const anscer_msgs::msg::SafetyField::SharedPtr msg)
  {
    // TODO: This logic is temporary. It should be replaced with a parameter
    // to select which field(s) trigger the emergency stop (e.g., front, rear, or both).
    if (msg->is_front)
    {
      // Create a Twist message to publish
      auto stop_msg = geometry_msgs::msg::Twist();
      
      // All fields are zero by default, so we don't need to set them.
      // Publish the stop message
      publisher_->publish(stop_msg);
      RCLCPP_WARN(this->get_logger(), "Emergency stop triggered by front field!");
    }
  }

  // Declare the publisher and subscriber as member variables
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<anscer_msgs::msg::SafetyField>::SharedPtr subscription_;
};

// The main function is where the ROS 2 system is initialized
int main(int argc, char * argv[])
{
  // Initialize rclcpp
  rclcpp::init(argc, argv);

  // Create an instance of our node and spin it, which allows callbacks to be processed
  rclcpp::spin(std::make_shared<EmergencyHandler>());

  // Shut down rclcpp
  rclcpp::shutdown();
  return 0;
}


