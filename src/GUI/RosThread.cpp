
#include "RosThread.h"

pthread_mutex_t RosThread::send_CS = PTHREAD_MUTEX_INITIALIZER;
RosThread::RosThread()
{
	gui = NULL;
	keepRunning = true;
	identity = -1;
}

RosThread::~RosThread(void)
{


}

void RosThread::startSystem()
{
	keepRunning = true;
	start();
}

void RosThread::stopSystem()
{
	keepRunning = false;
	join();
}

void RosThread::run()
{
	std::cout << "Starting ROS Thread" << std::endl;

	pub_controller_commands = nh.advertise<vision_ardrone::ControlCommand>("/ardrone_master/ControllerCommands", 1000);
	pub_ub_detector_commands = nh.advertise<vision_ardrone::ControlCommand>("/ardrone_master/UBDetectorCommands", 1000);
	pub_face_detector_commands = nh.advertise<vision_ardrone::ControlCommand>("/ardrone_master/FaceDetectorCommands", 1000);
	pub_recognizer_commands = nh.advertise<vision_ardrone::ControlCommand>("/ardrone_master/RecognizerCommands", 1000);

	sub_controller_status = nh.subscribe(nh.resolveName("/ardrone_controller/Status"), 1000, &RosThread::controller_statusCb, this);
	sub_ub_detector_status = nh.subscribe(nh.resolveName("/ardrone_UB_detector/Status"), 1000, &RosThread::ub_statusCb, this);
	sub_face_detector_status = nh.subscribe(nh.resolveName("/ardrone_FACE_detector/Status"), 1000, &RosThread::face_statusCb, this);
	sub_recognizer_status = nh.subscribe(nh.resolveName("/ardrone_recognizer/Status"), 1000, &RosThread::recognizer_statusCb, this);

	sub_navdata = nh.subscribe(nh.resolveName("ardrone/navdata"),50, &RosThread::navdataCb, this);

	sub_recognizer_identity = nh.subscribe(nh.resolveName("/ardrone_recognizer/Identity"), 1000, &RosThread::setIdentity,this);

	ros::Rate loop_rate(50);

	while(keepRunning && ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	if(nh.ok()) ros::shutdown();
	std::cout << "Exiting ROS Thread " << std::endl;
}

void RosThread::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
	batteryStatus = (navdataPtr->batteryPercent);
}

void RosThread::controller_statusCb(vision_ardrone::Status s)
{
	switch(s.status) {
		case vision_ardrone::Status::TAKING_OFF:
			gui->setControllerStatus("taking off");
			break;
		case vision_ardrone::Status::HOVERING:
			gui->setControllerStatus("hovering");
			break;
		case vision_ardrone::Status::LANDING:
			gui->setControllerStatus("landing");
			break;
		case vision_ardrone::Status::IDLE:
			gui->setControllerStatus("idle");
			break;
		case vision_ardrone::Status::TRACKING_UB:
			gui->setControllerStatus("Tracking UB");
			break;
		case vision_ardrone::Status::TRACKING_FACE:
			gui->setControllerStatus("Tracking FACE");
			break;
		case vision_ardrone::Status::CIRCLING:
			gui->setControllerStatus("circling");
			break;
		default:
			gui->setControllerStatus("unknown");
			break;
	}
}

void RosThread::ub_statusCb(vision_ardrone::Status s)
{
	switch(s.status) {
		case vision_ardrone::Status::IDLE:
			gui->setUBStatus("idle");
			break;
		case vision_ardrone::Status::TRACKING:
			gui->setUBStatus("tracking");
			break;
		case vision_ardrone::Status::DETECTING:
			gui->setUBStatus("detecting");
			break;
		default:
			gui->setUBStatus("unknown");
			break;
	}
}

void RosThread::face_statusCb(vision_ardrone::Status s)
{
	switch(s.status) {
		case vision_ardrone::Status::IDLE:
			gui->setFaceStatus("idle");
			break;
		case vision_ardrone::Status::TRACKING:
			gui->setFaceStatus("tracking");
			break;
		case vision_ardrone::Status::DETECTING:
			gui->setFaceStatus("detecting");
			break;
		default:
			gui->setFaceStatus("unknown");
			break;
	}
}

void RosThread::recognizer_statusCb(vision_ardrone::Status s)
{
	switch(s.status) {
		case vision_ardrone::Status::IDLE:
			gui->setRecogStatus("idle");
			break;
		case vision_ardrone::Status::COLLECT_FACES:
			gui->setRecogStatus("collecting");
			break;
		case vision_ardrone::Status::RECOGNIZE:
			gui->setRecogStatus("recognize");
			break;
		default:
			gui->setRecogStatus("unknown");
			break;
	}
}

void RosThread::setIdentity(vision_ardrone::Identity id) {
	identity = id.identity;
	cout << "received label: " << identity << endl;
}

void RosThread::land()
{
	//send take-off command to controller
	vision_ardrone::ControlCommand c;
	c.command = vision_ardrone::ControlCommand::LAND;
	pthread_mutex_lock(&send_CS);
	pub_controller_commands.publish(c);
	pthread_mutex_unlock(&send_CS);
}
void RosThread::takeOff()
{
	//send take-off command to controller
	vision_ardrone::ControlCommand c;
	c.command = vision_ardrone::ControlCommand::TAKE_OFF;
	pthread_mutex_lock(&send_CS);
	pub_controller_commands.publish(c);
	pthread_mutex_unlock(&send_CS);
}

void RosThread::startUBDetection()
{
	vision_ardrone::ControlCommand c;
	c.command = vision_ardrone::ControlCommand::START_DETECTION;
	pthread_mutex_lock(&send_CS);
	pub_ub_detector_commands.publish(c);
	pthread_mutex_unlock(&send_CS);
}

void RosThread::stopUBDetection()
{
	vision_ardrone::ControlCommand c;
	c.command = vision_ardrone::ControlCommand::STOP_DETECTION;
	pthread_mutex_lock(&send_CS);
	pub_ub_detector_commands.publish(c);
	pthread_mutex_unlock(&send_CS);
}

void RosThread::startFaceDetection()
{
	vision_ardrone::ControlCommand c;
	c.command = vision_ardrone::ControlCommand::START_DETECTION;
	pthread_mutex_lock(&send_CS);
	pub_face_detector_commands.publish(c);
	pthread_mutex_unlock(&send_CS);
}

void RosThread::stopFaceDetection()
{
	vision_ardrone::ControlCommand c;
	c.command = vision_ardrone::ControlCommand::STOP_DETECTION;
	pthread_mutex_lock(&send_CS);
	pub_face_detector_commands.publish(c);
	pthread_mutex_unlock(&send_CS);
}

void RosThread::startUBTracking()
{
	vision_ardrone::ControlCommand c;
	c.command = vision_ardrone::ControlCommand::TRACK_UB;
	pthread_mutex_lock(&send_CS);
	pub_controller_commands.publish(c);
	pthread_mutex_unlock(&send_CS);
}

void RosThread::startFaceTracking()
{
	vision_ardrone::ControlCommand c;
	c.command = vision_ardrone::ControlCommand::TRACK_FACE;
	pthread_mutex_lock(&send_CS);
	pub_controller_commands.publish(c);
	pthread_mutex_unlock(&send_CS);
}

void RosThread::hover()
{
	vision_ardrone::ControlCommand c;
	c.command = vision_ardrone::ControlCommand::HOVER;
	pthread_mutex_lock(&send_CS);
	pub_controller_commands.publish(c);
	pthread_mutex_unlock(&send_CS);
}

void RosThread::changeControl(bool X, bool Yaw, bool Altitude)
{
	vision_ardrone::ControlCommand c;
	c.command = vision_ardrone::ControlCommand::CHANGE_CONTROL;
	c.controlX = X;
	c.controlAltitude = Altitude;
	c.controlYaw = Yaw;
	pthread_mutex_lock(&send_CS);
	pub_controller_commands.publish(c);
	pthread_mutex_unlock(&send_CS);
}

void RosThread::flyCircle()
{
	vision_ardrone::ControlCommand c;
	c.command = vision_ardrone::ControlCommand::FLY_CIRCLE;
	pthread_mutex_lock(&send_CS);
	pub_controller_commands.publish(c);
	pthread_mutex_unlock(&send_CS);
}

void RosThread::recognize()
{
	vision_ardrone::ControlCommand c;
	c.command = vision_ardrone::ControlCommand::RECOGNIZE;
	pthread_mutex_lock(&send_CS);
	pub_recognizer_commands.publish(c);
	pthread_mutex_unlock(&send_CS);
}
