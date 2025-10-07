/**
 * @file qr_mission_client.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "mission_scheduler/qr_mission_client.h"

using namespace std::placeholders;

QRMissionClient::QRMissionClient(const rclcpp::NodeOptions & options)
: Node("qr_mission_client", options)
{
    this->client_ptr_ = rclcpp_action::create_client<Mission>(this, "mission_scheduler");
    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&QRMissionClient::send_goal, this));
}

void QRMissionClient::send_goal()
{
    this->timer_->cancel(); // Send goal only once

    if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        return;
    }

    auto goal_msg = Mission::Goal();
    goal_msg.order_id = "test_order";

    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto send_goal_options = rclcpp_action::Client<Mission>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&QRMissionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&QRMissionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&QRMissionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void QRMissionClient::goal_response_callback(const GoalHandleMission::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void QRMissionClient::feedback_callback(
    GoalHandleMission::SharedPtr,
    const std::shared_ptr<const Mission::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Received feedback: %.2f %%", feedback->percentage);
}

void QRMissionClient::result_callback(const GoalHandleMission::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }
    RCLCPP_INFO(this->get_logger(), "Result received: %s", result.result->success ? "true" : "false");
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QRMissionClient>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
