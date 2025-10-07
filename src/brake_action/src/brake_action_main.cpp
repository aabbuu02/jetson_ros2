/**
 * @file brake_action_main.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "rclcpp/rclcpp.hpp"
#include "brake_action/HigherLevelInterface.h"
#include <memory>

// This main function creates and runs the HigherLevelInterface node.
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HigherLevelInterface>());
    rclcpp::shutdown();
    return 0;
}


