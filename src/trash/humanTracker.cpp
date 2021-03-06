#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/opencv.h>
#include <std_msgs/Empty.h>
#include "ardrone_autonomy/Navdata.h"
#include "Track.h"
#include <stdio.h>

using namespace dlib;
using namespace std; 

const char *windowName = "Image window";

dlib::frontal_face_detector detector;
Track track(0.5,0.001,0.001);


void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Update GUI Window

	cv_image<bgr_pixel> cv(cv_ptr->image);
	array2d<rgb_pixel> img; 
	assign_image(img, cv);
	pyramid_up(img);

	std::vector<rectangle> dets = detector(img); 

 	//if face detected: update track;
	cv::Rect trackRec; 
        if(dets.size() > 0) //face detected;
        {
        	cv::Rect face(dets[0].left()/2, dets[0].top()/2, dets[0].width()/2, dets[0].height()/2);
		track.predict_update(&face);
        	cv::rectangle(cv_ptr->image, face, CV_RGB(0, 255, 0),2);
        }
	else
		track.predict_update(); 
	
	//cv::rectangle(cv_ptr->image, track.getBoundingBox(), CV_RGB(255, 255, 0));

	cv::imshow(windowName, cv_ptr->image);
	cv::waitKey(1);
}

enum modes {takeOff= 0, land}; 
modes mode = takeOff; 
const char* MODE_NAMES[] = {"takeoff", "land"}; 

void do_modes()
{
	switch(mode)
	{
		case takeOff: 
			cout << "take off" << endl;
			while(ros::ok())
				ros::spinOnce(); 
			break; 
		case land:
			cout << "land" << endl; 
			break; 
		default:
			cout << "unknown" << endl; 
			break; 
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "human_tracker");
	ros::NodeHandle nh; 
	image_transport::ImageTransport it(nh); 
	image_transport::Subscriber img_sub; 
	img_sub = it.subscribe("/ardrone/image_raw", 1,&imageCb);
	detector = dlib::get_frontal_face_detector();
	ros::Publisher takeoff_pub;
	ros::Publisher land_pub;
	ros::Subscriber land_sub; 
	ros::Subscriber nav_data; 	
	cv::namedWindow(windowName);
	while(ros::ok())
	{
		do_modes(); 
		ros::spinOnce(); 
	}
	return 0; 
}
