/**
 * @file LowerLevelComputations.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "lower_level_controller/LowerLevelComputations.h"

LowerLevelComputations::LowerLevelComputations(double wheel_radius, double wheel_separation, int encoder_cpr)
    : wheel_radius_(wheel_radius), wheel_separation_(wheel_separation), encoder_cpr_(encoder_cpr)
{
    distance_per_count_ = (2 * M_PI * wheel_radius_) / encoder_cpr_;
    last_left_encoder_ = 0;
    last_right_encoder_ = 0;
    x_pos_ = 0.0;
    y_pos_ = 0.0;
    theta_ = 0.0;
    last_time_ = rclcpp::Clock().now();
}

std::tuple<double, double> LowerLevelComputations::twistToRPM(double linear_v, double angular_v)
{
    double v_left = linear_v - (angular_v * wheel_separation_ / 2.0);
    double v_right = linear_v + (angular_v * wheel_separation_ / 2.0);
    double left_rpm = (v_left / wheel_radius_) * (60.0 / (2.0 * M_PI));
    double right_rpm = (v_right / wheel_radius_) * (60.0 / (2.0 * M_PI));
    return {left_rpm, right_rpm};
}

std::tuple<double, double, double, double, double> LowerLevelComputations::updateOdometry(long left_encoder, long right_encoder, rclcpp::Time current_time)
{
    long delta_left = left_encoder - last_left_encoder_;
    long delta_right = right_encoder - last_right_encoder_;
    double dt = (current_time - last_time_).seconds();

    double dist_left = delta_left * distance_per_count_;
    double dist_right = delta_right * distance_per_count_;
    double delta_dist = (dist_left + dist_right) / 2.0;
    double delta_theta = (dist_right - dist_left) / wheel_separation_;

    x_pos_ += delta_dist * cos(theta_);
    y_pos_ += delta_dist * sin(theta_);
    theta_ += delta_theta;

    double linear_velocity = (dt > 0) ? (delta_dist / dt) : 0.0;
    double angular_velocity = (dt > 0) ? (delta_theta / dt) : 0.0;
    
    last_left_encoder_ = left_encoder;
    last_right_encoder_ = right_encoder;
    last_time_ = current_time;

    return {x_pos_, y_pos_, theta_, linear_velocity, angular_velocity};
}
