//
//
// llab_01
// smooth_derivative
//
// Robot scans the 540 beams for distances, smoothes it with 1/3[1,1,1] filter
// and finds derivative with 1/2[-1,0,1] filter
//
// Author: University of Bonn, Autonomous Intelligent Systems
//

// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

// Class definition
class Smooth_derivative {
public:
	Smooth_derivative();
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
Smooth_derivative::Smooth_derivative() {
	// Node will be instantiated in root-namespace
	ros::NodeHandle m_nodeHandle("/");

	// Initialising the node handle
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &Smooth_derivative::laserCallback, this);
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);
}// end of Just_move constructor



// callback for getting laser values
void Smooth_derivative::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {

	if( (&m_laserscan)->ranges.size() < scanData->ranges.size() ) {
		m_laserscan.ranges.resize((size_t)scanData->ranges.size());
    	}

	for(unsigned int i = 0; i < scanData->ranges.size(); i++)
	{
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}// end of laserCallback



// robot shall stop, in case anything is closer than ...
void Smooth_derivative::emergencyStop() {

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
void Smooth_derivative::calculateCommand() {
	static bool have_scan = false;

	// see if we have laser data
	if( (&m_laserscan)->ranges.size() > 0 && !have_scan)
	  {
		FILE* scan = fopen("r.dat", "w");
		//for(unsigned int i=0; i<540; i++)
		//	fprintf(scan, "%d %f\n", i, m_laserscan.ranges[i]);
		//fclose(scan);
		ROS_INFO(" WROTE SCAN RESULT!");
		
		//FILE* smooth_scan = fopen("s.dat", "w");
		double smoothed[540];
		smoothed[0] = m_laserscan.ranges[0];
		smoothed[539] = m_laserscan.ranges[539];
		for(unsigned int i=1; i<539; i++) {
			smoothed[i] = (1.0/3.0)*(m_laserscan.ranges[i-1] + m_laserscan.ranges[i] + m_laserscan.ranges[i+1]);
		//	fprintf(smooth_scan, "%d %f\n", i, smoothed[i]);
		}
		//fclose(smooth_scan);
		ROS_INFO(" WROTE SMOOTHED SCAN RESULT!");
		
		//FILE* derivative_scan = fopen("d.dat", "w");
		double derivative[540];
		derivative[0] = smoothed[0];
		derivative[539] = smoothed[539];
		for(unsigned int i=1; i<539; i++) {
			derivative[i] = (1.0/2.0)*(-smoothed[i-1] + smoothed[i+1]);
		//	fprintf(derivative_scan, "%d %f\n", i, derivative[i]);
		}
		//fclose(derivative_scan);
		
		for(unsigned int i=0; i<540; i++) {
			fprintf(scan, "%d %f %f %f\n", i, m_laserscan.ranges[i], smoothed[i], derivative[i]);
		}
		fclose(scan);
		ROS_INFO(" WROTE DERIVATIVE OF SCAN RESULT!");
		have_scan = true;
	  } // end of if we have laser values
} // end of calculateCommands


//
void Smooth_derivative::mainLoop() {
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
	Smooth_derivative robbi;

	// main loop
	// make robbi do whatever robbi wants to do
	robbi.mainLoop();

	return 0;

}// end of main

// end of file: smooth_derivative.cpp
