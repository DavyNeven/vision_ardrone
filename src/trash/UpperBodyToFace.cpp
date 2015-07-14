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
#include "ObjectDetector.h"

using namespace cv;
using namespace std; 

const char *windowName = "Image window";	
dlib::frontal_face_detector detector; 
LKT_Tracker *faceTracker;
LKT_Tracker *UB_tracker;
ros::Publisher pub;
ros::Subscriber navdata_sub;
ObjectDetector bodyDetector1, bodyDetector2;
const char *faceCascadeFilename1 = "/usr/openCV/opencv/data/haarcascades/haarcascade_upperbody.xml";
const char *faceCascadeFilename2 = "/home/davy/HS.xml";

enum STATES {UB_DECTECTION=0, FACE_TRACKING, UB_TRACKING};
const char* STATE_NAMES[] = {"UpperBody detection", "Face tracking",
							 "UpperBody tracking"};

STATES state = UB_DECTECTION;


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


	switch(state)
	{
		case UB_DECTECTION:
		{
			Rect rect;
            bodyDetector2.detectBiggestObject(webcamImage, rect, 640);
            cout << "Mode: UB_DETECTION" << endl;
            if (rect.width > 0) // Upperbody detected!
            {
                Rect rect1;
                bodyDetector1.detectBiggestObject(webcamImage,rect1,640);
                Rect intersect = rect1 & rect;
                if(intersect.area() > rect.area()/2)
                {
                    //valid upperbody!
                    //Start tracking!
                    UB_tracker->updateTracker(webcamImage, rect);
                    state = UB_TRACKING;
                }
                rectangle(webcamImage, rect1, cv::Scalar(0,0,255), 2);
            }
            rectangle(webcamImage, rect, cv::Scalar(255,0,0), 2); 
		}
			break;
		case FACE_TRACKING:
		{
			faceTracker->track(webcamImage);
			// Try to update the tracker with a face detection //will do this every 4th frame
			static int counter = 0;
			if(counter++%4 == 0)
			{
				dlib::cv_image<dlib::bgr_pixel> cv(webcamImage);
				dlib::array2d<dlib::rgb_pixel> imgDlib;
				dlib::assign_image(imgDlib, cv);
				std::vector<dlib::rectangle> dets = detector(imgDlib);

				if (dets.size() > 0) { // Face detected -> update tracker
					Rect faceRect =
							Rect(dets[0].left(), dets[0].top(), dets[0].width(),
								 dets[0].height()) & Rect(0, 0, 640, 480);
					// Start tracker
					faceTracker->updateTracker(webcamImage, faceRect);
				}
			}
			if(faceTracker->isLocked())		// Face trackable
			{
				//Calculate center of boundingbox
				Point *p = faceTracker->getPosition();
				Point center = (p[0] + p[1] + p[2] + p[3]);
				center.x = center.x /4; center.y = center.y/4;

				//Calculate size of boundingbox
				double area = faceTracker->getArea();

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
		}    break;
		case UB_TRACKING:
		{
				cout << "Mode: UB_tracking" << endl;
                //Try to update tracker;
                Rect rect1;
                bodyDetector2.detectBiggestObject(webcamImage,rect1,640);
                if(((Rect)(rect1 & UB_tracker->getRect())).area() > UB_tracker->getRect().area()*0.8 && rect1.area() < UB_tracker->getRect().area()*1.2)
                {
                    UB_tracker->updateTracker(webcamImage, rect1);
                }
                else {
                    UB_tracker->track(webcamImage);
                }
                // Try to update the tracker with a face detection //will do this every 4th frame
                static int counter = 0;
                if (counter++ % 4 == 0) {
                    dlib::cv_image<dlib::bgr_pixel> cv(webcamImage);
                    dlib::array2d<dlib::rgb_pixel> imgDlib;
                    dlib::assign_image(imgDlib, cv);
                    std::vector<dlib::rectangle> dets = detector(imgDlib);

                    if (dets.size() > 0) { // Face detected -> update tracker
                        Rect faceRect =
                                Rect(dets[0].left(), dets[0].top(), dets[0].width(),
                                     dets[0].height()) & Rect(0, 0, 640, 480);
                        // Start tracker
                        faceTracker->updateTracker(webcamImage, faceRect);
                        state = FACE_TRACKING;
                    }
                }

			if(UB_tracker->isLocked())		// Face trackable
			{
				//Calculate center of boundingbox
				Point *p = UB_tracker->getPosition();
				Point center = (p[0] + p[1] + p[2] + p[3]);
				center.x = center.x /4; center.y = center.y/4;

				//Calculate size of boundingbox
				double area = UB_tracker->getArea();

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

		}    break;
	};



    
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
	bodyDetector1.initDetector(faceCascadeFilename1);
	bodyDetector2.initDetector(faceCascadeFilename2);
	faceTracker = new LKT_Tracker(40);
	UB_tracker = new LKT_Tracker(100);
	cv::namedWindow(windowName);
	ros::spin(); 
	return 0; 
}
