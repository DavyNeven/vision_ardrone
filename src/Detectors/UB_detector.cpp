//
// Created by davy on 4/26/15.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Empty.h>
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"
#include "../tools/LKT_Tracker.h"
#include <stdio.h>
#include "../tools/ObjectDetector.h"
#include "vision_ardrone/ControlCommand.h"
#include "vision_ardrone/Detection.h"
#include "vision_ardrone/Status.h"

using namespace cv;
using namespace std;

const char *windowName = "Image window";
LKT_Tracker *UB_tracker;

ros::Subscriber sub_control_commands;
ros::Publisher pub_detection;
ros::Publisher pub_status;

ObjectDetector bodyDetector1, bodyDetector2;
const char *faceCascadeFilename1 = "/opt/ros/hydro/share/OpenCV/haarcascades/haarcascade_upperbody.xml";
const char *faceCascadeFilename2 = "/home/ardrone/HS.xml";

enum STATES {IDLE=0, UB_DECTECTION, UB_TRACKING};
STATES state = IDLE;



void publish_status(const ros::TimerEvent& event)
{
    vision_ardrone::Status c;
    c.type = vision_ardrone::Status::UB_D;
    switch(state)
    {
        case IDLE:
            c.status = vision_ardrone::Status::IDLE;
            break;
        case UB_DECTECTION:
            c.status = vision_ardrone::Status::DETECTING;
            break;
        case UB_TRACKING:
            c.status = vision_ardrone::Status::TRACKING;
            break;
        default:
            c.status = vision_ardrone::Status::IDLE;
            break;
    }
    pub_status.publish(c);

}

void ControlCb(const vision_ardrone::ControlCommand c)
{
    switch(c.command)
    {
        case vision_ardrone::ControlCommand::START_DETECTION:
            cout << "received: Start detection" << endl;
            state = UB_DECTECTION;
            break;
        case vision_ardrone::ControlCommand::STOP_DETECTION:
            cout << "received: Stop detection" << endl;
            state = IDLE;
            break;
        default:
            cout << "received: Unkown " << endl;
            break;
    }
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

    Mat webcamImage = cv_ptr->image;


    switch(state)
    {
        case IDLE:
        {
            vision_ardrone::Detection d;
            d.status = vision_ardrone::Detection::NOT_LOCKED;
            d.type = vision_ardrone::Detection::UB;
            pub_detection.publish(d);
        }  break;
        case UB_DECTECTION:
        {
            Rect rect;
            bodyDetector2.detectBiggestObject(webcamImage, rect, 640);
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
            vision_ardrone::Detection d;
            d.status = vision_ardrone::Detection::NOT_LOCKED;
            d.type = vision_ardrone::Detection::UB;
            pub_detection.publish(d);
            rectangle(webcamImage, rect, cv::Scalar(255,0,0), 2);
        }
            break;

        case UB_TRACKING:
        {
            //Try to update tracker;
            Rect rect1;
            bodyDetector2.detectBiggestObject(webcamImage,rect1,640);
            if(((Rect)(rect1 & UB_tracker->getRect())).area() > UB_tracker->getRect().area()*0.8 && rect1.area() < UB_tracker->getRect().area()*1.2)
            {
                UB_tracker->updateTracker(webcamImage, rect1);
            }
            else
            {
                UB_tracker->track(webcamImage);
            }

            if(UB_tracker->isLocked())		// Face trackable
            {
                vision_ardrone::Detection d;
                Rect trac = UB_tracker->getRect();
                d.x = trac.x; d.y = trac.y;
                d.width = trac.width; d.height = trac.height;
                d.size = trac.area();
                d.status = vision_ardrone::Detection::LOCKED;
                d.type = vision_ardrone::Detection::UB;

                pub_detection.publish(d);
            }
            else
            {
                vision_ardrone::Detection d;
                d.status = vision_ardrone::Detection::NOT_LOCKED;
                d.type = vision_ardrone::Detection::UB;
                pub_detection.publish(d);
                state=UB_DECTECTION;
            }

        }    break;
    };
    imshow(windowName, webcamImage);
    waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ardrone_UB_detector");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub;
    img_sub = it.subscribe("/ardrone/image_raw", 1,&imageCb);
    pub_detection = nh.advertise<vision_ardrone::Detection>("/ardrone_UB_detector/detection", 1000);
    pub_status =  nh.advertise<vision_ardrone::Status>("/ardrone_UB_detector/Status", 1000);
    sub_control_commands = nh.subscribe(nh.resolveName("/ardrone_master/UBDetectorCommands"), 1000, &ControlCb);

    // Publish status every 200 ms
    ros::Timer timer = nh.createTimer(ros::Duration(0.2), &publish_status);

    bodyDetector1.initDetector(faceCascadeFilename1);
    bodyDetector2.initDetector(faceCascadeFilename2);

    UB_tracker = new LKT_Tracker(100);
    cv::namedWindow(windowName);
    ros::spin();
    return 0;
}
