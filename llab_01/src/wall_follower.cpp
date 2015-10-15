//
//
// llab_01
// wall_follower
//
// Robot follows right wall
//
// Author: University of Bonn, Autonomous Intelligent Systems
//

// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

// Class definition
class Wall_follower {
public:
	Wall_follower();
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
	void emergencyStop();
	void calculateCommand();
	void mainLoop();
	const static double DANGEROUS = 0.2;
	const static double FOLLOW = 0.4;
	const static double EPSILON = 0.05;

protected:

	// Nodehandle for Just_move robot
	ros::NodeHandle m_nodeHandle;

	// Subscriber and membervariables for our laserscanner
	ros::Subscriber m_laserSubscriber;
	sensor_msgs::LaserScan m_laserscan;

	// Publisher and membervariables for the driving commands
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

};

// constructor
Wall_follower::Wall_follower() {
	// Node will be instantiated in root-namespace
	ros::NodeHandle m_nodeHandle("/");

	// Initialising the node handle
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &Wall_follower::laserCallback, this);
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);
}// end of Just_move constructor



// callback for getting laser values
void Wall_follower::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {

	if( (&m_laserscan)->ranges.size() < scanData->ranges.size() ) {
		m_laserscan.ranges.resize((size_t)scanData->ranges.size());
    	}

	for(unsigned int i = 0; i < scanData->ranges.size(); i++)
	{
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}// end of laserCallback



// robot shall stop, in case anything is closer than ...
void Wall_follower::emergencyStop() {

        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0)
	{
		for(unsigned int i=2; i < (&m_laserscan)->ranges.size() - 2; i++)
		{
			if( m_laserscan.ranges[i] < DANGEROUS) {
				m_roombaCommand.linear.x = 0.0;
				m_roombaCommand.angular.z = 0.0;
				ROS_INFO(" EMERGENCY!");
			}// end of if too close
		}// end of for all laser beams
	} // end of if we have laser data
}// end of emergencyStop



// here we go
// this is the place where we will generate the commands for the robot
void Wall_follower::calculateCommand() {

	// see if we have laser data
	if( (&m_laserscan)->ranges.size() > 0)
	  {

			// set the roomba velocities
			// the linear velocity (front direction is x axis) is measured in m/sec
			// the angular velocity (around z axis, yaw) is measured in rad/sec
			m_roombaCommand.linear.x  = 0.2;
			m_roombaCommand.angular.z = -0.15;

			if (m_laserscan.ranges[90] < FOLLOW + EPSILON && m_laserscan.ranges[90] > FOLLOW - EPSILON) {
				m_roombaCommand.angular.z = 0.0;
				ROS_INFO("----------- Following the right side -----------");
			}

			if (m_roombaCommand.angular.z == -0.15) {
				for(unsigned int i=90; i <= 360; i++)
				{
					if( m_laserscan.ranges[i] < FOLLOW - EPSILON) {
						m_roombaCommand.angular.z = 0.2;
						m_roombaCommand.linear.x  = 0;
						ROS_INFO("----------- Turning of the right side -----------");
					}
				}
			}

			if ( m_roombaCommand.angular.z == -0.15 ) {
				for(unsigned int i=2; i < 90; i++)
				{
					if( m_laserscan.ranges[i] < FOLLOW) {
						m_roombaCommand.angular.z = -0.2;
						m_roombaCommand.linear.x  = 0.1;
						ROS_INFO("----------- Turning to the right side -----------");
					}
				}
			}

			if ( m_roombaCommand.angular.z == -0.15 ) {
				for(unsigned int i=361; i <= 538; i++)
				{
					if( m_laserscan.ranges[i] < FOLLOW) {
						m_roombaCommand.angular.z = -0.2;
						ROS_INFO("----------- Going and turning to the right side -----------");
					}
				}
			}

			if ( m_roombaCommand.angular.z == -0.15 )
				ROS_INFO("----------- Just moving slightly to right - nothing is near -----------");

	  } // end of if we have laser values
} // end of calculateCommands


//
void Wall_follower::mainLoop() {
	// determines the number of loops per second
	ros::Rate loop_rate(20);

	// als long as all is O.K : run
	// terminate if the node get a kill signal
	while (m_nodeHandle.ok())
	{
		calculateCommand();
		emergencyStop();

		ROS_INFO(" robot is moving with speed: forward=%f and turn=%f", m_roombaCommand.linear.x, m_roombaCommand.angular.z);

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
	ros::init(argc, argv, "Just_move");

	// get an object of type Just_move and call it robbi
	Wall_follower robbi;

	// main loop
	// make robbi do whatever robbi wants to do
	robbi.mainLoop();

	return 0;

}// end of main

// end of file: wall_follower.cpp
