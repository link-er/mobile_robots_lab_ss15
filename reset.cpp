#include <ros/ros.h>
#include <std_msgs/Empty.h>

std_msgs::Empty emp_msg;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "manual_reset");
	ros::NodeHandle node;
	ros::Rate loop_rate(50);
	ros::Publisher pub_empty_reset;
	
	pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); /* Message queue length is just 1 */
	while (ros::ok()) {
		pub_empty_reset.publish(emp_msg); //kills the drone
	
		ros::spinOnce();
		loop_rate.sleep();

	}//ros::ok
}//main

