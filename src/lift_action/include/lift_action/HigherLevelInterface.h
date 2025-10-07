/**
 * @file HigherLevelInterface.h
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef HIGHER_LEVEL_INTERFACE_H
#define HIGHER_LEVEL_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "lift_action/msg/lift_action.hpp"
#include "lift_action/msg/lift_feedback.hpp"
#include "lift_action/msg/error_status.hpp"
#include "lift_action/ModbusCommunicator.h"
#include "lift_action/Ui_Interface.h"
#include <memory>

class HigherLevelInterface : public rclcpp::Node {
public:
    explicit HigherLevelInterface(const rclcpp::NodeOptions & options);

private:
    void main_loop();
    void readParameters();
    void robotFeedbackCallback(const lift_action::msg::LiftAction::SharedPtr msg);

    std::shared_ptr<ModbusCommunicator> p_modbusCommunicator;
    std::unique_ptr<UIInterface> p_UIInterface;
    
    rclcpp::Publisher<lift_action::msg::LiftFeedback>::SharedPtr lifterFeedback_pub;
    rclcpp::Subscription<lift_action::msg::LiftAction>::SharedPtr robotFeedBack_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    // Member variables
    std::string m_uniqueId, m_prevUniqueID;
    int m_action, m_shelf, m_acrShelf, m_side;
    bool m_reachedFlag;
};

#endif // HIGHER_LEVEL_INTERFACE_H
