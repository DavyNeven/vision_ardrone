#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Empty.h>
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"
#include "Track.h"
#include "ObjectDetector.h"
#include <stdio.h>

using namespace cv;
using namespace std; 

const char *windowName = "Image window";
const char *faceCascadeFilename = "/usr/openCV/opencv/data/haarcascades/haarcascade_upperbody.xml";
const int DETECTION_WIDTH = 320;
ObjectDetector bodyDetector;
	

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    Mat img = cv_ptr->image; 
    Rect detectRect; 
    bodyDetector.detectBiggestObject(img, detectRect,DETECTION_WIDTH);
	if(detectRect.width > 0)
    		rectangle(img, detectRect, cv::Scalar(255,0,0), 2); 
    
    imshow(windowName, img);
    waitKey(1);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "upperbody_detector");
	ros::NodeHandle nh; 
	image_transport::ImageTransport it(nh); 
	image_transport::Subscriber img_sub; 
	img_sub = it.subscribe("/ardrone/image_raw", 1,&imageCb);
	bodyDetector.initDetector(faceCascadeFilename);
	cv::namedWindow(windowName);
	ros::spin(); 
	return 0; 
}
