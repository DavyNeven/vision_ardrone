#ifndef __ROSTHREAD_H
#define __ROSTHREAD_H
 
 

#include "cvd/thread.h"
#include "ros/ros.h"
#include "mainwindow.h"
#include "vision_ardrone/ControlCommand.h"
#include "vision_ardrone/Identity.h"
#include "vision_ardrone/Status.h"
#include "ardrone_autonomy/Navdata.h"

class MainWindow;

class RosThread : private CVD::Thread
{
private:
	// the associated thread's run function.
	void run();

	// keep Running
	bool keepRunning;

	// ros stuff

	ros::Publisher pub_controller_commands;
	ros::Publisher pub_ub_detector_commands;
	ros::Publisher pub_face_detector_commands;
	ros::Publisher pub_recognizer_commands;


	ros::Subscriber sub_controller_status;
	ros::Subscriber sub_ub_detector_status;
	ros::Subscriber sub_face_detector_status;
	ros::Subscriber sub_recognizer_status;

	ros::Subscriber sub_navdata;

	ros::Subscriber sub_recognizer_identity;


	ros::NodeHandle nh;

	static pthread_mutex_t send_CS;

public:
	RosThread(void);
	~RosThread(void);

	// start and stop system and respective thread.
	// to be called externally
	void startSystem();
	void stopSystem();

	void land();

	void takeOff();

	void startUBDetection();

	void stopUBDetection();

	void startFaceDetection();

	void stopFaceDetection();

	void startUBTracking();

	void startFaceTracking();

	void hover();

	void changeControl(bool X, bool Yaw, bool Altitude);

	void flyCircle();

	void recognize();

	void setIdentity(vision_ardrone::Identity id);

	void navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr);

	void controller_statusCb(vision_ardrone::Status s);


	void ub_statusCb(vision_ardrone::Status s);


	void face_statusCb(vision_ardrone::Status s);


	void recognizer_statusCb(vision_ardrone::Status s);


	int identity;
	int batteryStatus;

	MainWindow* gui;
};

#endif /* __ROSTHREAD_H */
