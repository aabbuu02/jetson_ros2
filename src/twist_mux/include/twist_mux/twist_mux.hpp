/**
 * @file twist_mux.hpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef TWIST_MUX_HPP
#define TWIST_MUX_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include <vector>
#include <string>
#include <memory>

namespace twist_mux
{

// A helper struct to manage each velocity topic subscriber
struct VelocityTopic
{
    std::string name;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;
    rclcpp::Time last_msg_time;
    double timeout;
    int priority;
    geometry_msgs::msg::Twist last_twist;
};

// A helper struct to manage each lock topic subscriber
struct LockTopic
{
    std::string name;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub;
    rclcpp::Time last_msg_time;
    double timeout;
    int priority;
    bool is_locked = false;
};

class TwistMux : public rclcpp::Node
{
public:
    TwistMux();

private:
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg, const std::string& topic_name);
    void lock_callback(const std_msgs::msg::Bool::SharedPtr msg, const std::string& topic_name);
    void publish_loop();
    int get_lock_priority();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::shared_ptr<VelocityTopic>> velocity_topics_;
    std::vector<std::shared_ptr<LockTopic>> lock_topics_;
};

} // namespace twist_mux

#endif // TWIST_MUX_HPP

