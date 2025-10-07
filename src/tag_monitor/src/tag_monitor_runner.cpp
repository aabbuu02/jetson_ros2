#include <ros/ros.h>

#include <tag_monitor/tag_monitor.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "tag_monitor");
    
    TagMonitor tm;
    ros::spin();
  
    return EXIT_SUCCESS;  
} 
