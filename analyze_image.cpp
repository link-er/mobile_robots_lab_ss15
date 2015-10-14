#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#include "ardrone_autonomy/Navdata.h"

#include <camera_calibration_parsers/parse.h>


int g_count = 0;
std::string encoding;
std::string codec;
int fps;
std::string filename;

int state, battery;
float x, y, z, h;

int rate;
double linear_coefficient, turn_coefficient, yaw_coefficient;
int counter = 0;

double x_history[10], y_history[10], x_average=0, y_average=0;

/*char* stateToString(int state) {
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
}*/

void control(const ardrone_autonomy::Navdata& msg_in)
{
  battery = msg_in.batteryPercent;
  state = msg_in.state;
  x = msg_in.rotX;
  y = msg_in.rotY;
  z = msg_in.rotZ;
  h = msg_in.altd;
  //ROS_INFO("battery: %d%%, state: %s", battery, stateToString(state));
  //if(counter%50 == 0)
  //	ROS_INFO("RotX: %.3f, RotY: %.3f, RotZ: %.3f, Height: %.3f", x, y, z, h);
}

geometry_msgs::Twist twist_msg;
geometry_msgs::Twist twist_msg_hover;
std_msgs::Empty emp_msg;

CvPoint imageCenter;
CvPoint spotCenter;
FILE* points;

void setMoveMsg() {
	//imageCenter;
	//spotCenter;
	twist_msg.linear.x=0.0; 
	twist_msg.linear.y=0.0;
	twist_msg.linear.z=0.0;
	twist_msg.angular.z=0.0;
	
	double forward = (- spotCenter.y + imageCenter.y);
	double turn = (- spotCenter.x + imageCenter.x);
	double yaw = turn;
	
	double prev_y = y_history[counter%10], prev_x = x_history[counter%10];
	
  	x_history[counter%10] = spotCenter.x;
  	y_history[counter%10] = spotCenter.y;
  	
  	if(counter >= 10) {
  		x_average = ((double)(x_history[0]+x_history[1]+x_history[2]+x_history[3]+x_history[4]))/5.0;
  		y_average = ((double)(y_history[0]+y_history[1]+y_history[2]+y_history[3]+y_history[4]))/5.0;
  	}
  	
	fprintf(points, "%d %d %f %f\n", spotCenter.x, spotCenter.y, x_average, y_average);
  	
	if(spotCenter.x < 0 && spotCenter.y < 0)
		return;
	
	if(!(forward < 5 && forward > -5))
		twist_msg.linear.x = linear_coefficient * (imageCenter.y - prev_y);// forward;
		
	if(!(turn < 10 && turn > -10))
		twist_msg.linear.y = turn_coefficient * (imageCenter.x - prev_x);// turn;
		
		//twist_msg.angular.z = yaw * yaw_coefficient;
}

void do_some_magic(cv::Mat* img, int hue, int saturation, int value) {
    	int width = img->cols;
    	int height = img->rows;
    	
	spotCenter.x = width / 2;
	spotCenter.y = height / 2;
	imageCenter.x = width / 2;
	imageCenter.y = height / 2;
	
	cv::Mat img_hsv;
	cv::cvtColor(*img, img_hsv, CV_RGB2HSV);
	
	/*cv::Vec3b pixel = img_hsv.at<cv::Vec3b>(height / 2, width / 2);
	int h = pixel[0];
	int s = pixel[1];
	int v = pixel[2];
	
	ROS_INFO("=======h: %d, s: %d, v: %d", h, s, v);*/

    	int radius = 10 ;

    	int channels = img_hsv.channels();
    	int step = img_hsv.step;

    	unsigned char hueP = 0, saturationP = 0, valueP = 0;
    	double S_x = 0.0 ;
    	double S_y = 0.0 ;
    	int hue_difference, saturation_difference, value_difference;

    	int C_C = 0 ;

    	for(int y=0; y<height; y++) {
        	for(int x=0; x<width; x++) {
          		hueP = img_hsv.data[y*step+x*channels+0] ;
          		saturationP = img_hsv.data[y*step+x*channels+1] ;
          		valueP = img_hsv.data[y*step+x*channels+2] ;

	  		hue_difference = fabs(hue - (int)hueP);
	  		saturation_difference = fabs(saturation - (int)saturationP);
	  		value_difference = fabs(value - (int)valueP);

	  		if ( (hue_difference < 20) && (saturation_difference < 100) && (value_difference < 100) ) {
	     			C_C++ ;
	     			S_x += x;
	     			S_y += y;
	    		}
	
        	}
     	}

	if(C_C > 40) {
	    	S_x = S_x / (C_C);
	    	S_y = S_y / (C_C);
    
		// draw a circle to into the control image 
		spotCenter.x = S_x;
		spotCenter.y = S_y;
	}
	cv::ellipse( *img, spotCenter, cv::Size(radius,radius), 2, 0, 360, cv::Scalar(255, 0, 0), 3, 0 );
    	cv::imshow("bottom", *img);
    
    	setMoveMsg();
}

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
    cv::Mat frame;
    cv::Mat frame_copy;
    
    //ROS_INFO(typeid(cv_bridge::toCvShare(msg, "bgr8")->image).name());

    frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    frame_copy = frame.clone();
    
    do_some_magic( &frame_copy, 56, 149, 203);
    //cv::imshow("bottom", frame_copy);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

//1 - loop_rate
//2 - linear coefficient
//3 - yaw coefficient or turn coefficient
//4 - log file name
int main(int argc, char **argv)
{
	system( "rosservice call /ardrone/togglecam" );
	system( "rosservice call /ardrone/togglecam" );
  
  	ros::init(argc, argv, "analyze_image");
  	
  	rate = atoi(argv[1]);
  	linear_coefficient = atof(argv[2]);
  	turn_coefficient = atof(argv[3]);
  	points = fopen(argv[4], "w");
  	
  	for(int i=0; i<10; i++) {
  		x_history[i] = 0;
  		y_history[i] = 0;
  	}
  	
	ros::NodeHandle node;
	ros::Rate loop_rate(rate);
  
	ros::Publisher pub_empty_land;
	ros::Publisher pub_twist;
	ros::Publisher pub_empty_takeoff;
	
	float wait_time=120.0;	

	//hover message
	twist_msg_hover.linear.x=0.0; 
	twist_msg_hover.linear.y=0.0;
	twist_msg_hover.linear.z=0.0;
	twist_msg_hover.angular.x=0.0; 
	twist_msg_hover.angular.y=0.0;
	twist_msg_hover.angular.z=0.0;

	twist_msg.linear.x=0.0; 
	twist_msg.linear.y=0.0;
	twist_msg.linear.z=0.0;
	twist_msg.angular.x=0.0; 
	twist_msg.angular.y=0.0;
	twist_msg.angular.z=0.0;

	pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
	pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */

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
	
	ros::NodeHandle n;
  	ros::Subscriber sub = n.subscribe("/ardrone/navdata", 1, control);
  
 double time = (double)ros::Time::now().toSec();

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
			if(counter%100 == 0)
				ROS_INFO("command: %.3f, %.3f, %.3f, %.3f", twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z, twist_msg.angular.z);
			pub_twist.publish(twist_msg);
		}//fly according to desired twist

		ros::spinOnce();
		loop_rate.sleep();
		counter++;

	}//ros::ok
	
	cv::destroyWindow("front");
	cv::destroyWindow("bottom");
	fclose(points);
}

