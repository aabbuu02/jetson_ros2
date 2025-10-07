/**
 * @file twist_mux.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "twist_mux/twist_mux.hpp"

namespace twist_mux
{

TwistMux::TwistMux() : Node("twist_mux")
{
    // --- Parameters ---
    this->declare_parameter<std::vector<std::string>>("topics", {});
    this->declare_parameter<std::vector<std::string>>("locks", {});
    double timer_frequency = this->declare_parameter<double>("publish_frequency", 10.0);

    // --- Publisher ---
    cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);

    // --- Velocity Topic Subscribers ---
    auto topic_names = this->get_parameter("topics").as_string_array();
    for (const auto& name : topic_names) {
        auto topic = std::make_shared<VelocityTopic>();
        topic->name = name;

        // Declare parameters for this specific topic
        this->declare_parameter<std::string>(name + ".topic", "");
        this->declare_parameter<double>(name + ".timeout", 0.5);
        this->declare_parameter<int>(name + ".priority", 0);
        
        std::string topic_path = this->get_parameter(name + ".topic").as_string();
        topic->timeout = this->get_parameter(name + ".timeout").as_double();
        topic->priority = this->get_parameter(name + ".priority").as_int();
        
        auto cb = [this, name](const geometry_msgs::msg::Twist::SharedPtr msg) {
            this->velocity_callback(msg, name);
        };
        topic->sub = this->create_subscription<geometry_msgs::msg::Twist>(topic_path, 1, cb);
        
        velocity_topics_.push_back(topic);
        RCLCPP_INFO(this->get_logger(), "Subscribed to velocity topic '%s' on path '%s' with priority %d",
                    name.c_str(), topic_path.c_str(), topic->priority);
    }

    // --- Lock Topic Subscribers ---
    auto lock_names = this->get_parameter("locks").as_string_array();
    for (const auto& name : lock_names) {
        auto lock = std::make_shared<LockTopic>();
        lock->name = name;
        
        this->declare_parameter<std::string>(name + ".topic", "");
        this->declare_parameter<double>(name + ".timeout", 0.5);
        this->declare_parameter<int>(name + ".priority", 0);
        
        std::string topic_path = this->get_parameter(name + ".topic").as_string();
        lock->timeout = this->get_parameter(name + ".timeout").as_double();
        lock->priority = this->get_parameter(name + ".priority").as_int();

        auto cb = [this, name](const std_msgs::msg::Bool::SharedPtr msg) {
            this->lock_callback(msg, name);
        };
        lock->sub = this->create_subscription<std_msgs::msg::Bool>(topic_path, 1, cb);

        lock_topics_.push_back(lock);
        RCLCPP_INFO(this->get_logger(), "Subscribed to lock topic '%s' on path '%s' with priority %d",
                    name.c_str(), topic_path.c_str(), lock->priority);
    }

    // --- Main Loop Timer ---
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / timer_frequency)),
        std::bind(&TwistMux::publish_loop, this));
}

void TwistMux::velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg, const std::string& topic_name)
{
    for (auto& topic : velocity_topics_) {
        if (topic->name == topic_name) {
            topic->last_msg_time = this->get_clock()->now();
            topic->last_twist = *msg;
            return;
        }
    }
}

void TwistMux::lock_callback(const std_msgs::msg::Bool::SharedPtr msg, const std::string& topic_name)
{
    for (auto& lock : lock_topics_) {
        if (lock->name == topic_name) {
            lock->last_msg_time = this->get_clock()->now();
            lock->is_locked = msg->data;
            return;
        }
    }
}

int TwistMux::get_lock_priority()
{
    int max_priority = 0;
    rclcpp::Time now = this->get_clock()->now();
    for (const auto& lock : lock_topics_) {
        if (lock->is_locked && (now - lock->last_msg_time).seconds() < lock->timeout) {
            if (lock->priority > max_priority) {
                max_priority = lock->priority;
            }
        }
    }
    return max_priority;
}

void TwistMux::publish_loop()
{
    int lock_priority = get_lock_priority();
    
    int best_priority = -1;
    std::shared_ptr<VelocityTopic> best_topic = nullptr;
    rclcpp::Time now = this->get_clock()->now();

    for (auto& topic : velocity_topics_) {
        if ((now - topic->last_msg_time).seconds() < topic->timeout) {
            if (topic->priority > best_priority && topic->priority > lock_priority) {
                best_priority = topic->priority;
                best_topic = topic;
            }
        }
    }
    
    if (best_topic) {
        cmd_vel_out_pub_->publish(best_topic->last_twist);
    } else {
        // Publish a zero twist if no topic is active
        geometry_msgs::msg::Twist zero_twist;
        cmd_vel_out_pub_->publish(zero_twist);
    }
}

} // namespace twist_mux

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<twist_mux::TwistMux>());
    rclcpp::shutdown();
    return 0;
}

