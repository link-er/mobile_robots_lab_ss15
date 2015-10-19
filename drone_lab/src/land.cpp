#include <ros/ros.h>
#include <std_msgs/Empty.h>

std_msgs::Empty emp_msg;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "manual_land");
	ros::NodeHandle node;
	ros::Rate loop_rate(50);
	ros::Publisher pub_empty_land;
	
	pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
	while (ros::ok()) {
		pub_empty_land.publish(emp_msg);
	
		ros::spinOnce();
		loop_rate.sleep();

	}//ros::ok
}//main

