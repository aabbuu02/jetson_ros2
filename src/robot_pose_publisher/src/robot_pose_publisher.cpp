/**
 * @file robot_pose_publisher.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include <chrono>
#include <memory>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class RobotPosePublisher : public rclcpp::Node
{
public:
    explicit RobotPosePublisher(const rclcpp::NodeOptions & options)
    : Node("robot_pose_publisher", options)
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<double>("publish_frequency", 10.0);

        map_frame_ = this->get_parameter("map_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        double publish_frequency = this->get_parameter("publish_frequency").as_double();

        // Initialize TF listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose", 10);

        // Initialize timer for the main loop
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_frequency),
            std::bind(&RobotPosePublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Robot Pose Publisher has been started.");
        RCLCPP_INFO(this->get_logger(), "Publishing pose from '%s' to '%s' at %.2f Hz.", map_frame_.c_str(), base_frame_.c_str(), publish_frequency);
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }

        auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
        pose_msg->header.stamp = this->get_clock()->now();
        pose_msg->header.frame_id = map_frame_;

        pose_msg->pose.position.x = transform.transform.translation.x;
        pose_msg->pose.position.y = transform.transform.translation.y;
        pose_msg->pose.position.z = transform.transform.translation.z;
        pose_msg->pose.orientation = transform.transform.rotation;

        publisher_->publish(std::move(pose_msg));
    }

    std::string map_frame_;
    std::string base_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<RobotPosePublisher>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
