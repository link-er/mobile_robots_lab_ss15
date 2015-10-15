#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#include "ardrone_autonomy/Navdata.h"

#include <camera_calibration_parsers/parse.h>

cv::VideoWriter outputVideo;

int g_count = 0;
std::string encoding;
std::string codec;
int fps;
std::string filename;

int state, battery;
float x, y, z, h;

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
  x = msg_in.rotX;
  y = msg_in.rotY;
  z = msg_in.rotZ;
  h = msg_in.altd;
  ROS_INFO("battery: %d%%, state: %s", battery, stateToString(state));
  ROS_INFO("RotX: %.3f, RotY: %.3f, RotZ: %.3f, Height: %.3f", x, y, z, h);
}

geometry_msgs::Twist twist_msg;
geometry_msgs::Twist twist_msg_hover;
std_msgs::Empty emp_msg;

CvPoint imageCenter;
CvPoint bottomCenter;
CvPoint triangleCenter;
	
/**
 * Helper function to find euclidean distance
 * from pt0->pt1 and pt1->pt2
 */
static double distance(cv::Point pt1, cv::Point pt2) {
	double dx = pt1.x - pt2.x;
	double dy = pt1.y - pt2.y;
	return sqrt(dx * dx + dy * dy);
}

/**
 * Helper function to display circle in the center of a contour
 * return center point of the contour
 */
CvPoint getCenterOfContour(cv::Mat* img, std::vector<cv::Point>& contour) {
	cv::Moments m = moments(contour, true);
	//getting center point of the contour
	cv::Point center(m.m10 / m.m00, m.m01 / m.m00);

	int radius = 5; //radius of the circle
	int brush_width = 2;

	// draw a circle to into the control image
	CvPoint cv_center;
	cv_center.x = center.x;
	cv_center.y = center.y;

	cv::ellipse(*img, cv_center, cv::Size(radius,radius), 2, 0, 360, cv::Scalar(255, 0, 0), brush_width, 0);
	return cv_center;
}

void setMoveMsg() {
	//imageCenter;
	//bottomCenter;
	//triangleCenter;
	twist_msg.linear.x=0.0; 
	twist_msg.linear.y=0.0;
	twist_msg.linear.z=0.0;
	twist_msg.angular.z=0.0;
	
	ROS_INFO("points are: %d, %d, %d, %d", bottomCenter.x, bottomCenter.y, triangleCenter.x, triangleCenter.y);
	
	double rotate = bottomCenter.x - triangleCenter.x;
	/*if(rotate > 10)
		twist_msg.angular.z = -0.1;
	if(rotate < -10)
		twist_msg.angular.z = 0.1;*/
}

void detect_sign(cv::Mat* src) {
	//getting image center coordinates
	imageCenter.x = (src->cols) / 2;
	imageCenter.y = (src->rows) / 2;

	// Convert to grayscale
	cv::Mat gray;
	cv::cvtColor(*src, gray, CV_BGR2GRAY);

	// Use Canny instead of threshold to catch squares with gradient shading
	cv::Mat canny_result;
	cv::Canny(gray, canny_result, 500, 1500, 5);
	//cv::dilate(canny_result,canny_result, cv::Mat(), cv::Point(-1,-1),2,1,1);
	//cv::erode(canny_result,canny_result, cv::Mat(), cv::Point(-1,-1),2,1,1);
	cv::imshow("filtered", canny_result);

	// Find contours
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(canny_result.clone(), contours, CV_RETR_EXTERNAL,
			CV_CHAIN_APPROX_SIMPLE);

	std::vector<cv::Point> approx;

	for (int i = 0; i < contours.size(); i++) {
		// Approximate contour with accuracy proportional
		// to the contour perimeter
		cv::approxPolyDP(cv::Mat(contours[i]), approx,
				cv::arcLength(cv::Mat(contours[i]), true) * 0.1, true);

		// Skip small or non-convex objects
		if (std::fabs(cv::contourArea(contours[i])) < 50
				|| !cv::isContourConvex(approx))
			continue;

		if (approx.size() == 3) {
			triangleCenter = getCenterOfContour(src, contours[i]); // Triangles

			int vtc = approx.size();

			// Get the cosines of all corners
			double shortestDistance = RAND_MAX;

			for (int j = 0; j < vtc; j++) {
				double tempDistance = distance(approx[j],
						approx[(j + 1) % vtc]);
				if (tempDistance < shortestDistance) {
					shortestDistance = tempDistance;
					bottomCenter.x = (approx[j].x + approx[(j + 1) % vtc].x)
							/ 2;
					bottomCenter.y = (approx[j].y + approx[(j + 1) % vtc].y)
							/ 2;

				}
			}
			//drawing circle around bottom side center
			cv::ellipse(*src, bottomCenter, cv::Size(5, 5), 2, 0, 360, cv::Scalar(0, 255, 0), 2, 0);
		}
	}

	//original image with circle
	cv::imshow("bottom", *src);
	
	setMoveMsg();
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
    
    detect_sign(&frame_copy);
    cv::imshow("bottom", frame_copy);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


void camera_callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info)
{
    if (!outputVideo.isOpened() && !info) return;
    else if (!outputVideo.isOpened() && info) {

        cv::Size size(info->width, info->height);

        outputVideo.open(filename, 
                CV_FOURCC(codec.c_str()[0],
                          codec.c_str()[1],
                          codec.c_str()[2],
                          codec.c_str()[3]), 
                fps,
                size,
                true);

        if (!outputVideo.isOpened())
        {
            ROS_ERROR("Could not create the output video! Check filename and/or support for codec.");
            exit(-1);
        }

        ROS_INFO_STREAM("Starting to record " << codec << " video at " << size << "@" << fps << "fps. Press Ctrl+C to stop recording." );

    }
    else if (outputVideo.isOpened() && info) return;

    cv::Mat image;
    try
    {
        image = cv_bridge::toCvShare(image_msg, encoding)->image;
    } catch(cv_bridge::Exception)
    {
        ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding.c_str());
        return;
    }

    if (!image.empty()) {
        outputVideo << image;
        ROS_INFO_STREAM("Recording frame " << g_count << "\x1b[1F");
        g_count++;
    } else {
        ROS_WARN("Frame skipped, no data!");
    }
}


int main(int argc, char **argv)
{
	system( "rosservice call /ardrone/togglecam" );
	system( "rosservice call /ardrone/togglecam" );
  
	ros::init(argc, argv, "move_by_sign");
	ros::NodeHandle node;
	ros::Rate loop_rate(50);

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
	
	cv::namedWindow("filtered");
	
	ros::NodeHandle n;
  	ros::Subscriber sub = n.subscribe("/ardrone/navdata", 1, control);
  	
	/*ros::init(argc, argv, "recorder");
    	ros::NodeHandle nh;
    	image_transport::ImageTransport it(nh);
    	std::string topic = nh.resolveName("ardrone/bottom/image_raw");
    	image_transport::CameraSubscriber sub_camera = it.subscribeCamera(topic, 1, &camera_callback);
    	image_transport::Subscriber sub_image = it.subscribe(topic, 1,
            boost::bind(camera_callback, _1, sensor_msgs::CameraInfoConstPtr()));

    	ros::NodeHandle local_nh("~");
    	local_nh.param("filename", filename, std::string("/home/ropra/ros_world/LAB_SS15_GR05/out.avi"));
    	local_nh.param("fps", fps, 15);
    	local_nh.param("codec", codec, std::string("MJPG"));
    	local_nh.param("encoding", encoding, std::string("bgr8"));

    	if (codec.size() != 4) {
        	ROS_ERROR("The video codec must be a FOURCC identifier (4 chars)");
        	exit(-1);
    	}*/
	
	double time = (double)ros::Time::now().toSec();

	while (ros::ok()) {
		if ((double)ros::Time::now().toSec() < time + 10.0) {
			pub_empty_takeoff.publish(emp_msg); //launches the drone
			pub_twist.publish(twist_msg_hover); //drone is flat
		}//takeoff before t+5
		
		else if (((double)ros::Time::now().toSec() > time + wait_time)) {
			pub_twist.publish(twist_msg_hover); //drone is flat
			pub_empty_land.publish(emp_msg); //lands the drone
		}//land after t+15
		
		else {	
			ROS_INFO("command is: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z, twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z);
			pub_twist.publish(twist_msg);
		}//fly according to desired twist

		ros::spinOnce();
		loop_rate.sleep();

	}//ros::ok
	
	cv::destroyWindow("front");
	cv::destroyWindow("bottom");
	cv::destroyWindow("filtered");
	outputVideo.release();
}

