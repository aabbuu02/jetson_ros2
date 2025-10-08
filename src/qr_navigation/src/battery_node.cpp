#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <future>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>


class Battery 
{
	private:
		ros::NodeHandle nh_;
		double battery_percentage ;	
		bool charging;
		ros::Subscriber charge_battery ;
		ros::Publisher charge_pub ;
	public:
		Battery(ros::NodeHandle *nodehandle): nh_ (*nodehandle)
		{
			battery_percentage = 100;
			charging = false;
			charge_battery = nh_.subscribe("charge_battery",1,&Battery::charge_callback,this);
			charge_pub  = nh_.advertise<std_msgs::Float32>("battery",10);
		}
		void charge_callback(const std_msgs::Int32ConstPtr &msg)
		{	
			if(msg->data == 0)
				charging = false;
			else if( msg->data == 1)
				charging = true;
		}
		void increase_charge()
		{
			if(battery_percentage < 5)
				battery_percentage = 10 ;
			if(battery_percentage <= 100)
				battery_percentage += battery_percentage / 10000;
			std_msgs::Float32 value ;
			value.data = battery_percentage;
			charge_pub.publish(value);
		}
		void decrease_charge()
		{
			if(battery_percentage >= 0)
                                battery_percentage -= battery_percentage / 10000;
			std_msgs::Float32 value;
			value.data = battery_percentage;
			charge_pub.publish(value);
		}
		bool get_status()
		{
			return charging ;
		}
		
};
bool publish = true ;
std::vector <std::future<void> > m;
void publish_percentage(Battery *battery)
{
	while(publish)
	{
		if(battery->get_status())
			battery->increase_charge();
		else
			battery->decrease_charge();
		usleep(25000);
	}
}
int main(int argc,char * argv[])
{
	ros::init(argc, argv, "battery_publisher");
	ros::NodeHandle nh;

	Battery *battery = new Battery(&nh);
	m.push_back(std::async (std::launch::async,publish_percentage,battery));
	ros::spin();
	publish = false ;
	delete battery;
	return 0;
}