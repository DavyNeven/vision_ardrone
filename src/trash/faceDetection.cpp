#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Empty.h>
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/opencv.h>
#include "LKT_Tracker.h"
#include <stdio.h>

using namespace cv;
using namespace std; 

const char *windowName = "Image window";	
dlib::frontal_face_detector detector; 
LKT_Tracker *tracker; 
ros::Publisher pub;
ros::Subscriber navdata_sub;

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

    Mat webcamImage = cv_ptr->image; 
    cout << "size : " << webcamImage.size() << endl;
    dlib::cv_image<dlib::bgr_pixel> cv(webcamImage);
	dlib::array2d<dlib::rgb_pixel> imgDlib;
	dlib::assign_image(imgDlib,cv);
	//pyramid_up(img);
	std::vector<dlib::rectangle> dets = detector(imgDlib);
	
	if(dets.size() > 0)
	{
		Rect trac = Rect(dets[0].left(), dets[0].top(), dets[0].width(), 
								dets[0].height()) & Rect(0, 0, 640,360); 
		tracker->updateTracker(webcamImage, trac);
		rectangle(webcamImage, trac, CV_RGB(0,255,0),2);
	}
	else
	{
		tracker->track(webcamImage);
	}
	if(tracker->isLocked())		// Face trackable
	{
		//Calculate center of boundingbox
		Point *p = tracker->getPosition(); 
		Point center = (p[0] + p[1] + p[2] + p[3]);
		center.x = center.x /4; center.y = center.y/4;

		//Calculate size of boundingbox
		double area = tracker->getArea();

		//Send message
		geometry_msgs::Twist pos;
		pos.linear.x = center.x; pos.linear.y = center.y; pos.linear.z = 1;
		pos.angular.x = area;
		pub.publish(pos); 
	}
	else
	{
		// Face not trackable
		geometry_msgs::Twist pos;
		pos.linear.z = 0; 
		pub.publish(pos); 
	
	}
    
    imshow(windowName, webcamImage);
    waitKey(1);
}

void navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
	cout << "Battery level: " << (float)navdataPtr->batteryPercent << endl; 
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "faceDetector");
	ros::NodeHandle nh; 
	image_transport::ImageTransport it(nh); 
	image_transport::Subscriber img_sub; 
	img_sub = it.subscribe("/ardrone/image_raw", 1,&imageCb);
	pub = nh.advertise<geometry_msgs::Twist>("/face/position", 1000);
	navdata_sub = nh.subscribe(nh.resolveName("ardrone/navdata"),50, &navdataCb);
	
	detector = dlib::get_frontal_face_detector();
	tracker = new LKT_Tracker(25);
	cv::namedWindow(windowName);
	ros::spin(); 
	return 0; 
}
