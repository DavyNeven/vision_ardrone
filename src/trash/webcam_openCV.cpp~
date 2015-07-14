#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ObjectDetector.h"
#include <stdio.h>

using namespace std; 

static const std::string OPENCV_WINDOW = "Image window";
//const char *faceCascadeFilename = "/usr/openCV/opencv/data/lbpcascades/lbpcascade_frontalface.xml";
const char *faceCascadeFilename = "/usr/openCV/opencv/data/haarcascades/haarcascade_fullbody.xml";
const int FACE_DETECTION_WIDTH = 640;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ObjectDetector faceDetector;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ardrone/image_raw", 1, 
      &ImageConverter::imageCb, this);
    cv::namedWindow(OPENCV_WINDOW);
    faceDetector.initDetector(faceCascadeFilename);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

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

   
    cv::Rect faceRect;
	faceDetector.detectBiggestObject(cv_ptr->image, faceRect,FACE_DETECTION_WIDTH);
	cout << "detecting" << endl; 
	if(faceRect.width > 0) //if face detected
	{
		cv::rectangle(cv_ptr->image,faceRect, Scalar(0,255,0), 2);
		cout << "body detected" << endl; 
	}
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(20);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
