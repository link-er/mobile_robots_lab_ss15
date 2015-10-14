#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ardrone_autonomy/Navdata.h"

int state, battery;
float h;

char* stateToString(int state) {
	switch(state){
	case 0: return "Unknown";
	case 1: return "Inited";
	case 2: return "Landed";
	case 3: return "Flying";
	case 4: return "Hovering";
	case 5: return "Test";
	case 6: return "Taking off";
	case 7: return "Flying";
	case 8: return "Landing";
	case 9: return "Looping";
	}
}

void control(const ardrone_autonomy::Navdata& msg_in)
{
  battery = msg_in.batteryPercent;
  state = msg_in.state;
  h = msg_in.altd;
  ROS_INFO("battery: %d%%, state: %s", battery, stateToString(state));
  ROS_INFO("Height: %.3f", h);
}

geometry_msgs::Twist twist_msg;
geometry_msgs::Twist twist_msg_hover;
geometry_msgs::Twist twist_msg_pshover;
std_msgs::Empty emp_msg;

int main(int argc, char** argv)
{
	ros::init(argc, argv,"ARDrone_manual_test");
	ros::NodeHandle node;
	ros::Rate loop_rate(50);

	ros::Publisher pub_empty_land;
	ros::Publisher pub_twist;
	ros::Publisher pub_empty_takeoff;
	
	double time;

	//hover message
	twist_msg_hover.linear.x=0.0; 
	twist_msg_hover.linear.y=0.0;
	twist_msg_hover.linear.z=0.0;
	twist_msg_hover.angular.x=0.0; 
	twist_msg_hover.angular.y=0.0;
	twist_msg_hover.angular.z=0.0;

	//command message
	float wait_time=20.0;	

	twist_msg.linear.x=0.0; 
	twist_msg.linear.y=0.0;
	twist_msg.linear.z=0.0;
	twist_msg.angular.x=0.0; 
	twist_msg.angular.y=0.0;
	twist_msg.angular.z=0.0;

	pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
	pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
	
	ros::NodeHandle n;
  	ros::Subscriber sub = n.subscribe("/ardrone/navdata", 1, control);
	
	time =(double)ros::Time::now().toSec();	

	while (ros::ok()) {
		if ((double)ros::Time::now().toSec() < time + 5.0) {
			pub_empty_takeoff.publish(emp_msg); //launches the drone
			pub_twist.publish(twist_msg_hover); //drone is flat
		}//takeoff before t+5

		else if (((double)ros::Time::now().toSec() > time + wait_time)) {
			pub_twist.publish(twist_msg_hover); //drone is flat
			pub_empty_land.publish(emp_msg); //lands the drone
		}//land after t+15
		
		else {	
			pub_twist.publish(twist_msg);
		}//fly according to desired twist

		ros::spinOnce();
		loop_rate.sleep();

	}//ros::ok
}//main

