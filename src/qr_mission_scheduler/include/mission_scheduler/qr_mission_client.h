/**
 * @file qr_mission_client.h
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef QR_MISSION_CLIENT_H
#define QR_MISSION_CLIENT_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "qr_mission_scheduler/action/mission_scheduler.hpp" // This path stays snake_case

class QRMissionClient : public rclcpp::Node
{
public:
    // Use the PascalCase name for the action type
    using Mission = qr_mission_scheduler::action::MissionScheduler;
    using GoalHandleMission = rclcpp_action::ClientGoalHandle<Mission>;

    explicit QRMissionClient(const rclcpp::NodeOptions & options);
    void send_goal();

private:
    rclcpp_action::Client<Mission>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(const GoalHandleMission::SharedPtr & goal_handle);
    void feedback_callback(
        GoalHandleMission::SharedPtr,
        const std::shared_ptr<const Mission::Feedback> feedback);
    void result_callback(const GoalHandleMission::WrappedResult & result);
};
#endif // QR_MISSION_CLIENT_H
