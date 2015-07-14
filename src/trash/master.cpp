#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include "vision_ardrone/ControlCommand.h"
#include "vision_ardrone/Status.h"

using namespace cv;
using namespace std;

double batteryLvl = 0;

const char *windowName = "Ardrone_Master";

enum STATUS {ST_TAKING_OFF=0, ST_LANDING, ST_HOVERING, ST_TRACKING_UB, ST_TRACKING_FACE, ST_CIRCLING, ST_DETECTING, ST_TRACKING, ST_IDLE};
const string STATUS_NAMES[] = {"Taking off", "Landing", "Hovering", "Tracking UB", "Tracking FACE", "Circling", "Detecting", "Tracking", "Idle"};
STATUS controller_status = ST_IDLE;
STATUS UB_detector_status = ST_IDLE;
STATUS FACE_detector_status = ST_IDLE;

enum STATES {TAKE_OFF=0, START_UB_DETECTION,STOP_UB_DETECTION,START_FACE_DETECTION, STOP_FACE_DETECTION, WAITING, START_UB_TRACKING, START_FACE_TRACKING, HOVERING, LANDING,
				FLY_CIRCLE, RECOGNIZE};
STATES state = WAITING;

/************** PUBS AND SUBS *********************/

ros::Publisher pub_controller_commands;
ros::Publisher pub_ub_detector_commands;
ros::Publisher pub_face_detector_commands;
ros::Publisher pub_recognizer_commands;


ros::Subscriber sub_controller_status;
ros::Subscriber sub_ub_detector_status;
ros::Subscriber sub_face_detector_status;

ros::Subscriber sub_navdata;

/**************************************************/

void statusCb(vision_ardrone::Status s)
{

	switch(s.status) {
		case vision_ardrone::Status::TAKING_OFF:
				switch(s.type)
				{
					case vision_ardrone::Status::CONTROLLER:
						controller_status = ST_TAKING_OFF;
						break;
					case vision_ardrone::Status::UB_D:
						UB_detector_status = ST_TAKING_OFF;
						break;
					case vision_ardrone::Status::FACE_D:
						FACE_detector_status = ST_TAKING_OFF;
						break;
				}
			break;
		case vision_ardrone::Status::HOVERING:
			switch(s.type)
			{
				case vision_ardrone::Status::CONTROLLER:
					controller_status = ST_HOVERING;
					break;
				case vision_ardrone::Status::UB_D:
					UB_detector_status = ST_HOVERING;
					break;
				case vision_ardrone::Status::FACE_D:
					FACE_detector_status = ST_HOVERING;
					break;
			}
			break;
		case vision_ardrone::Status::LANDING:
			switch(s.type)
			{
				case vision_ardrone::Status::CONTROLLER:
					controller_status = ST_LANDING;
					break;
				case vision_ardrone::Status::UB_D:
					UB_detector_status = ST_LANDING;
					break;
				case vision_ardrone::Status::FACE_D:
					FACE_detector_status = ST_LANDING;
					break;
			}
			break;
		case vision_ardrone::Status::IDLE:
			switch(s.type)
			{
				case vision_ardrone::Status::CONTROLLER:
					controller_status = ST_IDLE;
					break;
				case vision_ardrone::Status::UB_D:
					UB_detector_status = ST_IDLE;
					break;
				case vision_ardrone::Status::FACE_D:
					FACE_detector_status = ST_IDLE;
					break;
			}
			break;
		case vision_ardrone::Status::TRACKING_UB:
			switch(s.type)
			{
				case vision_ardrone::Status::CONTROLLER:
					controller_status = ST_TRACKING_UB;
					break;
				case vision_ardrone::Status::UB_D:
					UB_detector_status = ST_TRACKING_UB;
					break;
				case vision_ardrone::Status::FACE_D:
					FACE_detector_status = ST_TRACKING_UB;
					break;
			}
			break;
		case vision_ardrone::Status::TRACKING_FACE:
			switch(s.type)
			{
				case vision_ardrone::Status::CONTROLLER:
					controller_status = ST_TRACKING_FACE;
					break;
				case vision_ardrone::Status::UB_D:
					UB_detector_status = ST_TRACKING_FACE;
					break;
				case vision_ardrone::Status::FACE_D:
					FACE_detector_status = ST_TRACKING_FACE;
					break;
			}
			break;
		case vision_ardrone::Status::DETECTING:
			switch(s.type)
			{
				case vision_ardrone::Status::CONTROLLER:
					controller_status = ST_DETECTING;
					break;
				case vision_ardrone::Status::UB_D:
					UB_detector_status = ST_DETECTING;
					break;
				case vision_ardrone::Status::FACE_D:
					FACE_detector_status = ST_DETECTING;
					break;
			}
			break;
		case vision_ardrone::Status::TRACKING:
			switch(s.type)
			{
				case vision_ardrone::Status::CONTROLLER:
					controller_status = ST_TRACKING;
					break;
				case vision_ardrone::Status::UB_D:
					UB_detector_status = ST_TRACKING;
					break;
				case vision_ardrone::Status::FACE_D:
					FACE_detector_status = ST_TRACKING;
					break;
			}
			break;
		case vision_ardrone::Status::CIRCLING:
			switch(s.type)
			{
				case vision_ardrone::Status::CONTROLLER:
					controller_status = ST_CIRCLING;
					break;
				case vision_ardrone::Status::UB_D:
					UB_detector_status = ST_CIRCLING;
					break;
				case vision_ardrone::Status::FACE_D:
					FACE_detector_status = ST_CIRCLING;
					break;
			}
			break;
		default:
			break;
	}
}

void imageCb(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	Mat webcamImage = cv_ptr->image;
	string text = STATUS_NAMES[controller_status];
	Point coord_upper (40,40);
	Point coord_under(40,350);
	putText(webcamImage, text, coord_upper, FONT_HERSHEY_COMPLEX, 1.0f, cv::Scalar(0,255,0), 1, CV_AA);
	std::ostringstream buff;
	buff<<"Bat: " << batteryLvl;
	putText(webcamImage,buff.str() , coord_under, FONT_HERSHEY_COMPLEX, 1.0f, cv::Scalar(0,255,0), 1, CV_AA);

	imshow(windowName, webcamImage);
}

void send_command(vision_ardrone::ControlCommand c, ros::Publisher *pub)
{
	pub->publish(c);
}
void navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
	batteryLvl = (float)navdataPtr->batteryPercent;
}




void control_state()
{
	ros::Rate loop_rate(50);
	while(ros::ok())
	{
		vision_ardrone::ControlCommand c;
		switch(state)
		{
			case TAKE_OFF:
				//send take-off command to controller
				c.command = vision_ardrone::ControlCommand::TAKE_OFF;
				send_command(c, &pub_controller_commands);
				state = WAITING;
				break;
			case START_UB_DETECTION:
				//Start the upper-body detector
				c.command = vision_ardrone::ControlCommand::START_DETECTION;
				send_command(c,&pub_ub_detector_commands);
				state = WAITING;
				break;
			case STOP_UB_DETECTION:
				//Stop the upper-body detector
				c.command = vision_ardrone::ControlCommand::STOP_DETECTION;
				send_command(c,&pub_ub_detector_commands);
				state = WAITING;
				break;
			case START_FACE_DETECTION:
				//Start the upper-body detector
				c.command = vision_ardrone::ControlCommand::START_DETECTION;
				send_command(c,&pub_face_detector_commands);
				state = WAITING;
				break;
			case STOP_FACE_DETECTION:
				//Stop the upper-body detector
				c.command = vision_ardrone::ControlCommand::STOP_DETECTION;
				send_command(c,&pub_face_detector_commands);
				state = WAITING;
				break;
			case WAITING:
					// Waiting state
					state = WAITING;
				break;
			case START_UB_TRACKING:
					// Send start_tracking to controller
				c.command = vision_ardrone::ControlCommand::TRACK_UB;
				send_command(c, &pub_controller_commands);
				state = WAITING;
				break;
			case START_FACE_TRACKING:
				c.command = vision_ardrone::ControlCommand::TRACK_FACE;
				send_command(c, &pub_controller_commands);
				state = WAITING;
				break;
			case HOVERING:
					// Send hovering command to controller
				c.command = vision_ardrone::ControlCommand::HOVER;
				send_command(c, &pub_controller_commands);
				state = WAITING;
				break;
			case LANDING:
					// Send landing command to controller
				c.command = vision_ardrone::ControlCommand::LAND;
				send_command(c,&pub_controller_commands);
				state = WAITING;
				break;
			case FLY_CIRCLE:
				c.command = vision_ardrone::ControlCommand::FLY_CIRCLE;
				send_command(c,&pub_controller_commands);
				state = WAITING;
				break;
			case RECOGNIZE:
				c.command = vision_ardrone::ControlCommand::RECOGNIZE;
				send_command(c,&pub_recognizer_commands);
				state = WAITING;
				break;
		}

		switch((char)cv::waitKey(1))
		{
			case 'l':
				state = LANDING;
				break;
			case 't':
				state= TAKE_OFF;
				break;
			case 'o':
				state = START_UB_DETECTION;
				break;
			case 'p':
				state = STOP_UB_DETECTION;
				break;
			case 'f':
				state = START_FACE_DETECTION;
				break;
			case 'g':
				state = STOP_FACE_DETECTION;
				break;
			case 's':
				state = START_UB_TRACKING;
				break;
			case 'd':
				state = START_FACE_TRACKING;
				break;
			case 'h':
				state = HOVERING;
				break;
			case 'c':
				state = FLY_CIRCLE;
				break;
			case 'r':
				state = RECOGNIZE;
			default:
				break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ardrone_master");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber img_sub;
	img_sub = it.subscribe("/ardrone/image_raw", 1,&imageCb);

	pub_controller_commands = nh.advertise<vision_ardrone::ControlCommand>("/ardrone_master/ControllerCommands", 1000);
	pub_ub_detector_commands = nh.advertise<vision_ardrone::ControlCommand>("/ardrone_master/UBDetectorCommands", 1000);
	pub_face_detector_commands = nh.advertise<vision_ardrone::ControlCommand>("/ardrone_master/FaceDetectorCommands", 1000);
	pub_recognizer_commands = nh.advertise<vision_ardrone::ControlCommand>("/ardrone_master/RecognizerCommands", 1000);

	sub_controller_status = nh.subscribe(nh.resolveName("/ardrone_controller/Status"), 1000, &statusCb);
	sub_ub_detector_status = nh.subscribe(nh.resolveName("/ardrone_UB_detector/Status"), 1000, &statusCb);
	sub_face_detector_status = nh.subscribe(nh.resolveName("/ardrone_FACE_detector/Status"), 1000, &statusCb);
	sub_navdata = nh.subscribe(nh.resolveName("ardrone/navdata"),50, &navdataCb);

	cv::namedWindow(windowName);
	control_state();

	return 0; 
}
