//
//
// llab_01
// scanner
//
// Robot scans the 540 beams for distances
//
// Author: University of Bonn, Autonomous Intelligent Systems
//

// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

// Class definition
class Scanner {
public:
	Scanner();
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
	void emergencyStop();
	void calculateCommand();
	void mainLoop();
	const static double DANGEROUS = 0.2;

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
Scanner::Scanner() {
	// Node will be instantiated in root-namespace
	ros::NodeHandle m_nodeHandle("/");

	// Initialising the node handle
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &Scanner::laserCallback, this);
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);
}// end of Just_move constructor



// callback for getting laser values
void Scanner::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {

	if( (&m_laserscan)->ranges.size() < scanData->ranges.size() ) {
		m_laserscan.ranges.resize((size_t)scanData->ranges.size());
    	}

	for(unsigned int i = 0; i < scanData->ranges.size(); i++)
	{
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}// end of laserCallback



// robot shall stop, in case anything is closer than ...
void Scanner::emergencyStop() {

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
void Scanner::calculateCommand() {
	static bool have_scan = false;

	// see if we have laser data
	if( (&m_laserscan)->ranges.size() > 0 && !have_scan)
	  {
		FILE* scan = fopen("scan_result.txt", "w");
		for(unsigned int i=0; i<540; i++)
			fprintf(scan, "%d %f\n", i, m_laserscan.ranges[i]);
		fclose(scan);
		have_scan = true;
		ROS_INFO(" WROTE SCAN RESULT!");
	  } // end of if we have laser values
} // end of calculateCommands


//
void Scanner::mainLoop() {
	// determines the number of loops per second
	ros::Rate loop_rate(20);

	// als long as all is O.K : run
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
	ros::init(argc, argv, "Scanner");

	// get an object of type Just_move and call it robbi
	Scanner robbi;

	// main loop
	// make robbi do whatever robbi wants to do
	robbi.mainLoop();

	return 0;

}// end of main

// end of file: scanner.cpp
