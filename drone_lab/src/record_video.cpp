#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_calibration_parsers/parse.h>

cv::VideoWriter outputVideo;

int g_count = 0;
std::string encoding;
std::string codec;
int fps;
std::string filename;

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

void callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info)
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

int main(int argc, char** argv)
{	
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
	
	ros::init(argc, argv, "recorder");
    	ros::NodeHandle nh;
    	image_transport::ImageTransport it(nh);
    	std::string topic = nh.resolveName("ardrone/bottom/image_raw");
    	image_transport::CameraSubscriber sub_camera = it.subscribeCamera(topic, 1, &callback);
    	image_transport::Subscriber sub_image = it.subscribe(topic, 1,
            boost::bind(callback, _1, sensor_msgs::CameraInfoConstPtr()));

    	ros::NodeHandle local_nh("~");
    	local_nh.param("filename", filename, std::string("/home/ropra/ros_world/LAB_SS15_GR05/out.avi"));
    	local_nh.param("fps", fps, 15);
    	local_nh.param("codec", codec, std::string("MJPG"));
    	local_nh.param("encoding", encoding, std::string("bgr8"));

    	if (codec.size() != 4) {
        	ROS_ERROR("The video codec must be a FOURCC identifier (4 chars)");
        	exit(-1);
    	}
    	
	while (ros::ok())
    		ros::spin();
	
	cv::destroyWindow("front");
	cv::destroyWindow("bottom");
	outputVideo.release();
}//main

