/**
 * @file HigherLevelInterface.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "lift_action/HigherLevelInterface.h"
#include <chrono>

using namespace std::chrono_literals;

HigherLevelInterface::HigherLevelInterface(const rclcpp::NodeOptions & options)
    : Node("higher_level_interface_node", options) {
    RCLCPP_INFO(this->get_logger(), "HigherLevelInterface constructor called");
    readParameters();

    try {
        std::string ip = this->get_parameter("modbus_ip").as_string();
        int port = this->get_parameter("modbus_port").as_int();
        p_modbusCommunicator = std::make_shared<ModbusCommunicator>(ip, port, this->get_logger());
    } catch (const std::runtime_error &e) {
        RCLCPP_FATAL(this->get_logger(), "Shutting down: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    p_UIInterface = std::make_unique<UIInterface>(this, p_modbusCommunicator);

    lifterFeedback_pub = this->create_publisher<lift_action::msg::LiftFeedback>("/lifter_feedback", 10);
    robotFeedBack_sub = this->create_subscription<lift_action::msg::LiftAction>(
        "/lifter_goal", 10, std::bind(&HigherLevelInterface::robotFeedbackCallback, this, std::placeholders::_1));
    
    double update_rate = this->get_parameter("update_rate").as_double();
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0/update_rate), std::bind(&HigherLevelInterface::main_loop, this));
}

void HigherLevelInterface::readParameters() {
    this->declare_parameter<double>("update_rate", 10.0);
    this->declare_parameter<std::string>("modbus_ip", "127.0.0.1");
    this->declare_parameter<int>("modbus_port", 502);
}

void HigherLevelInterface::robotFeedbackCallback(const lift_action::msg::LiftAction::SharedPtr msg) {
    m_reachedFlag = msg->reached;
    if (m_reachedFlag) {
        m_uniqueId = msg->unique_id;
        m_action = msg->action;
        m_shelf = msg->shelf;
        m_acrShelf = msg->acr_shelf;
        m_side = msg->rack % 10; // Simplified side calculation
        RCLCPP_INFO(this->get_logger(), "Goal received: %s", m_uniqueId.c_str());
    }
}

void HigherLevelInterface::main_loop() {
    if (!p_modbusCommunicator->isConnected() || !m_reachedFlag) {
        return;
    }

    if (m_uniqueId != m_prevUniqueID) {
        RCLCPP_INFO(this->get_logger(), "Processing new goal: %s", m_uniqueId.c_str());
        
        p_modbusCommunicator->writeDataToRegister(1, m_shelf);
        p_modbusCommunicator->writeDataToRegister(2, (m_action == 0) ? m_acrShelf : 0);
        p_modbusCommunicator->writeDataToRegister(3, m_side);
        p_modbusCommunicator->writeDataToRegister(4, (m_action == 1) ? 1 : 2);
        
        m_prevUniqueID = m_uniqueId;
    }

    int cycle_complete = p_modbusCommunicator->readRegister(60);

    if (cycle_complete == 1) {
        RCLCPP_INFO(this->get_logger(), "Cycle complete for goal: %s", m_uniqueId.c_str());
        auto feedback_msg = lift_action::msg::LiftFeedback();
        feedback_msg.unique_id = m_uniqueId;
        feedback_msg.status = 1;
        lifterFeedback_pub->publish(feedback_msg);

        m_reachedFlag = false;
        p_modbusCommunicator->writeDataToRegister(4, 0);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<HigherLevelInterface>(options);
    if(rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
