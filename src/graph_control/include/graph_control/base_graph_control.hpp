#pragma once

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

namespace graph_control
{
    class BaseGraphControl
    {
        public:

        virtual void initialize() = 0;
        virtual void sendGoal(const std::string &vertex_name, uint32_t vertex_id, const geometry_msgs::Pose &vertex_pose) = 0;
        virtual void cancelGoal() = 0;

        virtual ~BaseGraphControl(){}

        protected:

        BaseGraphControl(){}
    };
}