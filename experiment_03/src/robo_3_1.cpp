// experiment_03
// robo_3_1.cpp
// Using AMCL
//
// Author: University of Bonn, Autonomous Intelligent Systems
//

// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159265

// Class definition
class Navigator {
public:
	Navigator();
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
	void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
	void emergencyStop();
	void calculateCommand();
	void mainLoop();
	float estimateX;
	float estimateY;
	float estimateTheta;

protected:

	// Nodehandle for Navigator robot
	ros::NodeHandle m_nodeHandle;

	// Subscriber and membervariables for our laserscanner
	ros::Subscriber m_laserSubscriber;
	sensor_msgs::LaserScan m_laserscan;
	// Subscriber and membervariables for our amcl
	ros::Subscriber m_robotposeSubscriber;
	geometry_msgs::PoseWithCovarianceStamped m_robotpose;
	// Publisher and membervariables for the driving commands
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

};

// constructor
Navigator::Navigator() {
	// Node will be instantiated in root-namespace
	ros::NodeHandle m_nodeHandle("/");

	// Initialising the node handle
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &Navigator::laserCallback, this);
	m_robotposeSubscriber = m_nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 20, &Navigator::poseCallback, this);
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);

}// end of Navigator constructor

void Navigator::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    estimateX=msg->pose.pose.position.x;
    estimateY=msg->pose.pose.position.y;
    estimateTheta=tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("X:%+6.2f  Y:%+6.2f  Theta:%+6.2f[rad/s]", estimateX, estimateY, estimateTheta);
    m_robotpose.pose.pose = msg->pose.pose;
}

// callback for getting laser values
void Navigator::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {
	if( (&m_laserscan)->ranges.size() < scanData->ranges.size() ) {
		m_laserscan.ranges.resize((size_t)scanData->ranges.size());
  }

	for(unsigned int i = 0; i < scanData->ranges.size(); i++) {
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}// end of laserCallback

// here we go
// this is the place where we will generate the commands for the robot
void Navigator::calculateCommand() {
	// goal coordinates fixed, but might be changed to desired
	double goalX = 23.30;
	double goalY = -38;
	double static goalAngle = 100;

	if(m_robotpose.pose.pose.position.x!=0 && m_robotpose.pose.pose.position.y!=0) {
		if(goalAngle==100) {
			// to get correct angle need to find point coordinates in robots system, i.e. with robot at 0;0
			// use atan2 to get right angle
	    goalAngle = atan2 ((goalY-m_robotpose.pose.pose.position.y), (goalX-m_robotpose.pose.pose.position.x));
	  }

	  double distance = sqrt( (goalX-m_robotpose.pose.pose.position.x)*(goalX-m_robotpose.pose.pose.position.x) +
	  	(goalY-m_robotpose.pose.pose.position.y)*(goalY-m_robotpose.pose.pose.position.y) );

		m_roombaCommand.linear.x = distance/10.0;
		if(distance < 0.4)
			m_roombaCommand.linear.x  = 0;

		m_roombaCommand.angular.z = goalAngle - estimateTheta;
		if(m_roombaCommand.angular.z < 0.2 && m_roombaCommand.angular.z > -0.2 )
			m_roombaCommand.angular.z = 0;

	  ROS_INFO("Goal angle:%+6.2f Distance:%+6.2f Velocity:%+6.2f Angular velocity:%+6.2f", goalAngle, distance, m_roombaCommand.linear.x, m_roombaCommand.angular.z);
	}
}// end of calculateCommands

// robot shall stop, in case anything is closer than ...
void Navigator::emergencyStop() {
	// see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0)
	{
		for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
		{
			if( m_laserscan.ranges[i] <= 0.30) {
				m_roombaCommand.linear.x = 0.0;
				m_roombaCommand.angular.z = 0.0;
			}// end of if too close
		}// end of for all laser beams
	} // end of if we have laser data
}// end of emergencyStop

void Navigator::mainLoop() {
	// determines the number of loops per second
	ros::Rate loop_rate(20);

	// as long as all is O.K : run
	// terminate if the node get a kill signal
	while (m_nodeHandle.ok())
	{
		calculateCommand();
		emergencyStop();

		// send the command to the roomrider for execution
		m_commandPublisher.publish(m_roombaCommand);

		// spinOnce, just make the loop happen once
		ros::spinOnce();
		// and sleep, to be aligned with the 50ms loop rate
		loop_rate.sleep();
	}
}// end of mainLoop

int main(int argc, char** argv) {
	// initialize
	ros::init(argc, argv, "Navigator");

	// get an object of type Navigator and call it dude
	Navigator dude;

	// main loop
	// make dude execute it's task
	dude.mainLoop();

	return 0;

}// end of main

// end of file: robo_3_1.cpp
