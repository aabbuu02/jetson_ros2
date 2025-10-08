/**
 * @file shelf_reader_ros2.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief ROS2 shelf reader matching ROS1 functionality
 */

#include "rclcpp/rclcpp.hpp"
#include "anscer_msgs/msg/pgv_pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include <memory>

class ShelfReader : public rclcpp::Node
{
public:
  ShelfReader()
  : Node("shelf_reader_node")
  {
    // Declare parameters
    this->declare_parameter("shelf_topic", "/shelf_detection");
    this->declare_parameter("loop_rate", 10.0);

    // Get parameters
    m_shelf_topic = this->get_parameter("shelf_topic").as_string();
    m_loop_rate = this->get_parameter("loop_rate").as_double();

    // Create publishers
    m_shelf_detection_pub = this->create_publisher<std_msgs::msg::Bool>(m_shelf_topic, 10);

    // Create timer for shelf reading
    m_shelf_timer = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / m_loop_rate)),
      std::bind(&ShelfReader::shelfReadCallback, this));

    RCLCPP_INFO(this->get_logger(), "Shelf Reader node has been started.");
  }

private:
  void shelfReadCallback()
  {
    // Simulate shelf reading (replace with actual shelf reading logic)
    std_msgs::msg::Bool shelf_detected;
    shelf_detected.data = false; // Dummy detection
    
    m_shelf_detection_pub->publish(shelf_detected);
  }

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_shelf_detection_pub;

  // Timer
  rclcpp::TimerBase::SharedPtr m_shelf_timer;

  // Member variables
  std::string m_shelf_topic;
  double m_loop_rate;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShelfReader>());
  rclcpp::shutdown();
  return 0;
}