/**
 * @file mission_scheduler.h
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef MISSION_SCHEDULER_H
#define MISSION_SCHEDULER_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "qr_mission_scheduler/action/mission_scheduler.hpp" // This path stays snake_case
#include "qr_mission_scheduler/srv/get_goal.hpp"

class MissionScheduler : public rclcpp::Node
{
public:
    // Use the PascalCase name for the action type
    using Mission = qr_mission_scheduler::action::MissionScheduler;
    using GoalHandleMission = rclcpp_action::ServerGoalHandle<Mission>;

    explicit MissionScheduler(const rclcpp::NodeOptions & options);

private:
    rclcpp_action::Server<Mission>::SharedPtr action_server_;
    rclcpp::Service<qr_mission_scheduler::srv::GetGoal>::SharedPtr get_goal_service_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Mission::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMission> goal_handle);

    void handle_accepted(
        const std::shared_ptr<GoalHandleMission> goal_handle);

    void execute(
        const std::shared_ptr<GoalHandleMission> goal_handle);

    void get_goal_callback(
        const std::shared_ptr<qr_mission_scheduler::srv::GetGoal::Request> request,
        std::shared_ptr<qr_mission_scheduler::srv::GetGoal::Response> response);
};

#endif // MISSION_SCHEDULER_H
