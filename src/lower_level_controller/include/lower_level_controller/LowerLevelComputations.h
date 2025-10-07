/**
 * @file LowerLevelComputations.h
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef LOWER_LEVEL_COMPUTATIONS_H
#define LOWER_LEVEL_COMPUTATIONS_H

#include <cmath>
#include <tuple>
#include "rclcpp/rclcpp.hpp"

class LowerLevelComputations
{
public:
    LowerLevelComputations(double wheel_radius, double wheel_separation, int encoder_cpr);
    std::tuple<double, double> twistToRPM(double linear_v, double angular_v);
    std::tuple<double, double, double, double, double> updateOdometry(long left_encoder, long right_encoder, rclcpp::Time current_time);

private:
    double wheel_radius_;
    double wheel_separation_;
    int encoder_cpr_;
    double distance_per_count_;
    
    long last_left_encoder_;
    long last_right_encoder_;
    rclcpp::Time last_time_;

    double x_pos_;
    double y_pos_;
    double theta_;
};
#endif //LOWER_LEVEL_COMPUTATIONS_H
