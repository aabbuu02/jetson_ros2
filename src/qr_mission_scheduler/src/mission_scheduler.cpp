/**
 * @file mission_scheduler.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "mission_scheduler/mission_scheduler.h"

MissionScheduler::MissionScheduler(const rclcpp::NodeOptions & options)
: Node("mission_scheduler_server", options)
{
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Mission>(
        this,
        "mission_scheduler",
        std::bind(&MissionScheduler::handle_goal, this, _1, _2),
        std::bind(&MissionScheduler::handle_cancel, this, _1),
        std::bind(&MissionScheduler::handle_accepted, this, _1));
    
    this->get_goal_service_ = this->create_service<qr_mission_scheduler::srv::GetGoal>(
        "get_goal",
        std::bind(&MissionScheduler::get_goal_callback, this, _1, _2));
    
    RCLCPP_INFO(this->get_logger(), "Mission Scheduler Action Server and GetGoal Service are ready.");
}

rclcpp_action::GoalResponse MissionScheduler::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Mission::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with order id %s", goal->order_id.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MissionScheduler::handle_cancel(
    const std::shared_ptr<GoalHandleMission> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MissionScheduler::handle_accepted(
    const std::shared_ptr<GoalHandleMission> goal_handle)
{
    std::thread{std::bind(&MissionScheduler::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void MissionScheduler::execute(
    const std::shared_ptr<GoalHandleMission> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Mission::Feedback>();
    auto result = std::make_shared<Mission::Result>();

    for (int i = 1; i <= 5 && rclcpp::ok(); ++i) {
        if (goal_handle->is_canceling()) {
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        feedback->percentage = i * 20.0f;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publishing feedback: %.2f %%", feedback->percentage);
        loop_rate.sleep();
    }

    if (rclcpp::ok()) {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}

void MissionScheduler::get_goal_callback(
    const std::shared_ptr<qr_mission_scheduler::srv::GetGoal::Request> request,
    std::shared_ptr<qr_mission_scheduler::srv::GetGoal::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "GetGoal service called for current position: %d", request->current_pos[0]);
    response->unique_id = "sample_id_123";
    response->goal_pos = {10, 20}; // Dummy response
    RCLCPP_INFO(this->get_logger(), "Responding with goal for unique_id: %s", response->unique_id.c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionScheduler>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
