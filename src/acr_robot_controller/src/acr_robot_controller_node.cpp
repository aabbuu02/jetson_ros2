/**
 * @file acr_robot_controller_node.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

// ROS 2 Headers
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "anscer_msgs/msg/motor_diagnostics_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Standard C++ and System Headers
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

// --- Merged Content from ErrorCodes.h and Constants.h ---
#define RQ_SUCCESS 0
#define RQ_ERR_OPEN_PORT 1
#define RQ_ERR_NOT_CONNECTED 2
#define RQ_ERR_TRANSMIT_FAILED 3
#define RQ_ERR_SERIAL_IO 4
#define RQ_INVALID_COMMAND 5
#define RQ_INVALID_QUERY 6
#define RQ_INVALID_INDEX 7
#define RQ_UNEXPECTED_RESPONSE 8
const int _GO = 3;
const int _MOTVEL = 30;
const int _ABCNTR = 34;

// --- RoboteqDevice Class Definition and Implementation ---
class RoboteqDevice
{
public:
	RoboteqDevice() { fd = -1; }
	~RoboteqDevice() { Disconnect(); }

	int Connect(std::string port) {
		if(fd != -1) return RQ_ERR_OPEN_PORT;
		if((fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) == -1) return RQ_ERR_OPEN_PORT;
		struct termios options;
		tcgetattr(fd, &options);
		options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		options.c_oflag &= ~OPOST;
		cfsetospeed(&options, B115200);
		cfsetispeed(&options, B115200);
		tcsetattr(fd, TCSANOW, &options);
		return RQ_SUCCESS;
	}

	void Disconnect() {
		if(fd != -1) close(fd);
		fd = -1;
	}

	int SetCommand(int command, int index, int value) {
		std::string cmd_str = "!G " + std::to_string(index) + " " + std::to_string(value) + "\r";
        if (command == _MOTVEL) {
            cmd_str = "!S " + std::to_string(index) + " " + std::to_string(value) + "\r";
        }
		return write_port(cmd_str);
	}

	int GetValue(int query, int index, int &value) {
		std::string query_str = "?C " + std::to_string(index) + "\r";
        if (query == _ABCNTR) {
            query_str = "?C " + std::to_string(index) + "\r";
        }
		if(write_port(query_str) != RQ_SUCCESS) return RQ_ERR_TRANSMIT_FAILED;
		std::string response = "";
		usleep(10000);
		if(read_port(response) != RQ_SUCCESS) return RQ_ERR_SERIAL_IO;
		size_t pos = response.find("=");
		if(pos == std::string::npos) return RQ_UNEXPECTED_RESPONSE;
		try { value = std::stoi(response.substr(pos + 1)); }
        catch (...) { return RQ_UNEXPECTED_RESPONSE; }
		return RQ_SUCCESS;
	}

private:
	int fd;
	int write_port(const std::string &str) {
		if (fd == -1) return RQ_ERR_NOT_CONNECTED;
		int count = write(fd, str.c_str(), str.length());
		if (count < 0) return RQ_ERR_TRANSMIT_FAILED;
		return RQ_SUCCESS;
	}
	int read_port(std::string &response) {
		if (fd == -1) return RQ_ERR_NOT_CONNECTED;
		char buf[2048] = "";
		int count = read(fd, buf, 2048);
		if(count < 0) return RQ_ERR_SERIAL_IO;
		response = buf;
		return RQ_SUCCESS;
	}
};


// --- Main ROS 2 Node Class ---
class AcrRobotControllerNode : public rclcpp::Node
{
public:
    AcrRobotControllerNode() : Node("acr_robot_controller_node")
    {
        // Parameters
        this->declare_parameter<std::string>("roboteq_port", "/dev/ttyACM0");
        this->declare_parameter<double>("wheel_base", 0.5);
        this->declare_parameter<double>("wheel_radius", 0.1);
        this->declare_parameter<int>("encoder_cpr", 1024);
        this->declare_parameter<double>("cmd_vel_timeout", 0.25);

        roboteq_port_ = this->get_parameter("roboteq_port").as_string();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        encoder_cpr_ = this->get_parameter("encoder_cpr").as_int();
        cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();

        // Hardware Init
        roboteq_device_ = std::make_unique<RoboteqDevice>();
        if (roboteq_device_->Connect(roboteq_port_) != 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to connect to Roboteq device on port %s", roboteq_port_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully connected to Roboteq device.");
        }

        // ROS Communications
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&AcrRobotControllerNode::cmd_vel_callback, this, std::placeholders::_1));
        estop_sub_ = this->create_subscription<std_msgs::msg::Bool>("e_stop", 10, std::bind(&AcrRobotControllerNode::estop_callback, this, std::placeholders::_1));
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&AcrRobotControllerNode::control_loop, this));
        last_odom_time_ = this->get_clock()->now();
        last_cmd_vel_time_ = this->get_clock()->now();
    }

    ~AcrRobotControllerNode()
    {
        if (roboteq_device_) { roboteq_device_->Disconnect(); }
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_vel_ = *msg;
        last_cmd_vel_time_ = this->get_clock()->now();
    }

    void estop_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        estop_active_ = msg->data;
    }

    void control_loop()
    {
        geometry_msgs::msg::Twist current_cmd = last_cmd_vel_;
        if ((this->get_clock()->now() - last_cmd_vel_time_).seconds() > cmd_vel_timeout_ || estop_active_) {
            current_cmd.linear.x = 0.0;
            current_cmd.angular.z = 0.0;
        }

        double v = current_cmd.linear.x;
        double w = current_cmd.angular.z;
        double right_vel = (v + (w * wheel_base_ / 2.0)) / wheel_radius_;
        double left_vel = (v - (w * wheel_base_ / 2.0)) / wheel_radius_;
        int right_rpm = static_cast<int>(right_vel * 60.0 / (2.0 * M_PI));
        int left_rpm = static_cast<int>(left_vel * 60.0 / (2.0 * M_PI));

        roboteq_device_->SetCommand(_MOTVEL, 1, right_rpm);
        roboteq_device_->SetCommand(_MOTVEL, 2, left_rpm);

        int left_encoder_raw = 0, right_encoder_raw = 0;
        roboteq_device_->GetValue(_ABCNTR, 2, left_encoder_raw);
        roboteq_device_->GetValue(_ABCNTR, 1, right_encoder_raw);

        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_odom_time_).seconds();
        if (dt <= 0) return;

        long delta_left_enc = left_encoder_raw - last_left_encoder_;
        long delta_right_enc = right_encoder_raw - last_right_encoder_;
        double dist_left = (2.0 * M_PI * wheel_radius_ * delta_left_enc) / encoder_cpr_;
        double dist_right = (2.0 * M_PI * wheel_radius_ * delta_right_enc) / encoder_cpr_;
        double delta_dist = (dist_left + dist_right) / 2.0;
        double delta_theta = (dist_right - dist_left) / wheel_base_;

        x_pos_ += delta_dist * cos(theta_ + delta_theta / 2.0);
        y_pos_ += delta_dist * sin(theta_ + delta_theta / 2.0);
        theta_ += delta_theta;

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = x_pos_;
        odom_msg.pose.pose.position.y = y_pos_;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);
        odom_msg.twist.twist.linear.x = delta_dist / dt;
        odom_msg.twist.twist.angular.z = delta_theta / dt;
        odom_pub_->publish(odom_msg);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x_pos_;
        t.transform.translation.y = y_pos_;
        t.transform.rotation = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(t);
        
        last_left_encoder_ = left_encoder_raw;
        last_right_encoder_ = right_encoder_raw;
        last_odom_time_ = current_time;
    }

    std::unique_ptr<RoboteqDevice> roboteq_device_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist last_cmd_vel_;
    rclcpp::Time last_cmd_vel_time_;
    bool estop_active_ = false;
    std::string roboteq_port_;
    double wheel_base_, wheel_radius_;
    int encoder_cpr_;
    double cmd_vel_timeout_;
    double x_pos_ = 0.0, y_pos_ = 0.0, theta_ = 0.0;
    rclcpp::Time last_odom_time_;
    long last_left_encoder_ = 0, last_right_encoder_ = 0;
};

// --- Main function ---
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AcrRobotControllerNode>());
    rclcpp::shutdown();
    return 0;
}


