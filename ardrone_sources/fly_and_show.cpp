#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallbackFront(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("front", cv_bridge::toCvShare(msg, "bgr8")->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void imageCallbackBottom(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("bottom", cv_bridge::toCvShare(msg, "bgr8")->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

geometry_msgs::Twist twist_msg;
geometry_msgs::Twist twist_msg_hover;
geometry_msgs::Twist twist_msg_pshover;
std_msgs::Empty emp_msg;

int main(int argc, char** argv)
{
	printf("Manual Test Node Starting");
	
	ros::init(argc, argv,"ARDrone_manual_test");
	ros::NodeHandle node;
	ros::Rate loop_rate(50);

	ros::Publisher pub_empty_land;
	ros::Publisher pub_twist;
	ros::Publisher pub_empty_takeoff;
	ros::Publisher pub_empty_reset;
	
	double time;

	//hover message
	twist_msg_hover.linear.x=0.0; 
	twist_msg_hover.linear.y=0.0;
	twist_msg_hover.linear.z=0.0;
	twist_msg_hover.angular.x=0.0; 
	twist_msg_hover.angular.y=0.0;
	twist_msg_hover.angular.z=0.0;

	//command message
	float fly_time=3.0;
	float land_time=5.0;
	float kill_time =2.0;	

	twist_msg.linear.x=0.0; 
	twist_msg.linear.y=-0.0001;
	twist_msg.linear.z=0.0;
	twist_msg.angular.x=0.0; 
	twist_msg.angular.y=0.0;
	twist_msg.angular.z=0.0;

	pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
	pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
	pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); /* Message queue length is just 1 */

	sleep(5);//waiting while driver is up	
	
	system( "rosservice call /ardrone/togglecam" );
	system( "rosservice call /ardrone/togglecam" );

	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh_front;
	cv::namedWindow("front");
	cv::startWindowThread();
	image_transport::ImageTransport it_front(nh_front);
	image_transport::Subscriber sub_front = it_front.subscribe("ardrone/front/image_raw", 1, imageCallbackFront);

	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh_bottom;
	cv::namedWindow("bottom");
	cv::startWindowThread();
	image_transport::ImageTransport it_bottom(nh_bottom);
	image_transport::Subscriber sub_bottom = it_bottom.subscribe("ardrone/bottom/image_raw", 1, imageCallbackBottom);
	
	time =(double)ros::Time::now().toSec();	
	ROS_INFO("Starting ARdrone_test loop");

	while (ros::ok()) {
		if ((double)ros::Time::now().toSec() < time+5.0) {
			pub_empty_takeoff.publish(emp_msg); //launches the drone
			pub_twist.publish(twist_msg_hover); //drone is flat
			ROS_INFO("Taking off");
		}//takeoff before t+5

		else if ((double)ros::Time::now().toSec() > time+fly_time+land_time+kill_time) {
			ROS_INFO("Closing Node");
			pub_empty_reset.publish(emp_msg); //kills the drone		
			break;
		}//kill node

		else if (((double)ros::Time::now().toSec() > time+fly_time+land_time)) {
			pub_twist.publish(twist_msg_hover); //drone is flat
			pub_empty_land.publish(emp_msg); //lands the drone
			ROS_INFO("Landing");
		}//land after t+15
	
		else {	
			pub_twist.publish(twist_msg_hover);
			ROS_INFO("Flying");
		}//fly according to desired twist

		ros::spinOnce();
		loop_rate.sleep();

	}//ros::ok

	pub_empty_reset.publish(emp_msg);
	
	cv::destroyWindow("front");
	cv::destroyWindow("bottom");
}//main

