//
// Created by davy on 4/28/15.
//

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
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/opencv.h>
#include "vision_ardrone/ControlCommand.h"
#include "vision_ardrone/Detection.h"
#include "vision_ardrone/Status.h"

using namespace cv;
using namespace std;

const char *windowName = "Image window";
LKT_Tracker *face_tracker;
dlib::frontal_face_detector detector;

ros::Subscriber sub_control_commands;
ros::Publisher pub_detection;
ros::Publisher pub_status;

enum STATES {IDLE=0, FACE_DECTECTION, FACE_TRACKING};
STATES state = IDLE;


void publish_status(const ros::TimerEvent& event)
{
    vision_ardrone::Status c;
    c.type = vision_ardrone::Status::FACE_D;
    switch(state)
    {
        case IDLE:
            c.status = vision_ardrone::Status::IDLE;
            break;
        case FACE_DECTECTION:
            c.status = vision_ardrone::Status::DETECTING;
            break;
        case FACE_TRACKING:
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
            state = FACE_DECTECTION;
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

    dlib::cv_image<dlib::bgr_pixel> cv(webcamImage);
    dlib::array2d<dlib::rgb_pixel> imgDlib;
    dlib::assign_image(imgDlib,cv);
    //pyramid_up(img);

    switch(state)
    {
        case IDLE:
        {
            vision_ardrone::Detection d;
            d.status = vision_ardrone::Detection::NOT_LOCKED;
            d.type = vision_ardrone::Detection::FACE;
            pub_detection.publish(d);
        }  break;
        case FACE_DECTECTION:
        {

            std::vector<dlib::rectangle> dets = detector(imgDlib);
            if(dets.size() > 0)
            {
                Rect trac = Rect(dets[0].left(), dets[0].top(), dets[0].width(),
                                 dets[0].height()) & Rect(0, 0, 640,360);
                face_tracker->updateTracker(webcamImage, trac);
                state = FACE_TRACKING;
                rectangle(webcamImage, trac, CV_RGB(0,255,0),5);
            }

            vision_ardrone::Detection d;
            d.status = vision_ardrone::Detection::NOT_LOCKED;
            d.type = vision_ardrone::Detection::FACE;
            pub_detection.publish(d);
        }
            break;

        case FACE_TRACKING:
        {
            //Try to update tracker;
            std::vector<dlib::rectangle> dets = detector(imgDlib);
            if(dets.size() > 0)
            {
                for(int i = 0; i< dets.size(); i++)
                {
                    Rect rec = Rect(dets[i].left(), dets[i].top(), dets[i].width(),
                                    dets[i].height()) & Rect(0, 0, 640,360);
                    if((rec & face_tracker->getRect()).area() > 0)
                    {
                        face_tracker->updateTracker(webcamImage, rec);
                        break;
                    }
                }
            }

            face_tracker->track(webcamImage);

            if(face_tracker->isLocked())		// Face trackable
            {
                vision_ardrone::Detection d;
                Rect trac = face_tracker->getRect();
                d.x = trac.x; d.y = trac.y;
                d.width = trac.width; d.height = trac.height;
                d.size = trac.area();
                d.status = vision_ardrone::Detection::LOCKED;
                d.type = vision_ardrone::Detection::FACE;

                pub_detection.publish(d);
            }
            else
            {
                vision_ardrone::Detection d;
                d.status = vision_ardrone::Detection::NOT_LOCKED;
                d.type = vision_ardrone::Detection::FACE;
                pub_detection.publish(d);
                state=FACE_DECTECTION;
            }

        }    break;
    };
    imshow(windowName, webcamImage);
    waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ardrone_FACE_detector");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub;
    img_sub = it.subscribe("/ardrone/image_raw", 1,&imageCb);
    pub_detection = nh.advertise<vision_ardrone::Detection>("/ardrone_FACE_detector/detection", 1000);
    pub_status =  nh.advertise<vision_ardrone::Status>("/ardrone_FACE_detector/Status", 1000);
    sub_control_commands = nh.subscribe(nh.resolveName("/ardrone_master/FaceDetectorCommands"), 1000, &ControlCb);

    ros::Timer timer = nh.createTimer(ros::Duration(0.2), &publish_status);


    detector = dlib::get_frontal_face_detector();

    face_tracker = new LKT_Tracker(30);
    cv::namedWindow(windowName);
    ros::spin();
    return 0;
}
