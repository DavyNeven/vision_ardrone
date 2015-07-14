#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"
#include <opencv2/opencv.hpp>
#include <stdio.h>


using namespace std; 

ros::Publisher takeoff_pub;
ros::Publisher land_pub;
ros::Publisher vel_pub; 
ros::Subscriber face_sub; 
geometry_msgs::Twist position;
geometry_msgs::Twist twist_msg_hover;
geometry_msgs::Twist twist_msg_up;
geometry_msgs::Twist command;
bool tracker_enabled = false; 

const char *windowName = "Image window";

void init()
{
	//hover msg
	twist_msg_hover.linear.x=0.0;
	twist_msg_hover.linear.y=0.0;
	twist_msg_hover.linear.z=0.0;
	twist_msg_hover.angular.x=0.0;
	twist_msg_hover.angular.y=0.0;
	twist_msg_hover.angular.z=0.0; 
	
	//up msg

	twist_msg_up.linear.x=0;
	twist_msg_up.linear.y=0.0;
	twist_msg_up.linear.z=1;
	twist_msg_up.angular.x=0.0;
	twist_msg_up.angular.y=0.0;
}

void takeOff()
{
	ros::Rate loop_rate(50); 
	double time = (double)ros::Time::now().toSec();
	cout << "Taking off" << endl; 
	
	
	while((double)ros::Time::now().toSec() < time + 2 && ros::ok())
	{ 		
		takeoff_pub.publish(std_msgs::Empty());
		vel_pub.publish(twist_msg_hover);
		ros::spinOnce();
		loop_rate.sleep(); 
	}
	while((double)ros::Time::now().toSec() < time + 4 && ros::ok())
	{ 
							
		vel_pub.publish(twist_msg_up);
		ros::spinOnce();
		loop_rate.sleep(); 
	}	
}

void land()
{
	ros::Rate loop_rate(50); 
	double time = (double)ros::Time::now().toSec();
	cout << "Landing" << endl; 

	while((double)ros::Time::now().toSec() < time + 4 && ros::ok())
	{ 		
				
		land_pub.publish(std_msgs::Empty());
		vel_pub.publish(twist_msg_hover);
		ros::spinOnce();
		loop_rate.sleep(); 
	}
}

double prev_error = 0;
double prev_time = 0;
double getCommand(double error, double Kp, double Kd)
{

	double error_diff = error - prev_error;
	prev_error = error;

	double time = ros::Time::now().toSec();
	double time_diff = time - prev_time;
	prev_time = time;

	cout << "time diff" << time_diff << endl;
	cout << "error_diff" << error_diff << endl;
	double Ed = error_diff/time_diff;
	cout << "diff term :" << Kd*Ed << endl;
	double command = Kp*error + Kd*Ed;
	//output boundaries
	if(command > 1) command = 1;
	if(command < -1 ) command = -1;
	cout << "Control command : ====   " << command << "    =====" << endl;
	return command;
}

geometry_msgs::Twist calculateCommand(geometry_msgs::Twist position)
{
	if(position.linear.z) // Face trackable -> control ardrone
	{
		cout << "Caculating command" << endl;
		geometry_msgs::Twist msg;
		msg.linear.x = msg.linear.y = msg.linear.z = 0;
		msg.angular.x = msg.angular.y = msg.angular.z = 0;

		// proportion control rotation
		msg.angular.z = (320 - position.linear.x)/320;
		// proportion control altitude
		msg.linear.z = ((180 - position.linear.y)/180)*0.5;
		// proportion control x-direction

		//	msg.linear.x = (10000 - position.angular.x) /20000;
		// !For face in real word use 20000!
		double error = (40000 - position.angular.x)/40000;
		//msg.linear.x = getCommand(error,  0.09, 0.06);
		msg.linear.x = getCommand(error,  0.6, 0.01);
		return msg;
	}
	else // Face not trackable -> hover
	{
		cout << "hovering" << endl;
		return twist_msg_hover;
	}
}

void faceCb(const geometry_msgs::TwistConstPtr vel)
{
	command = calculateCommand(*vel);
	tracker_enabled = true; 
}

void control()
{
	ros::Rate loop_rate(50);
	geometry_msgs::Twist msg;
	while(ros::ok())
	{
		if(tracker_enabled)
		{
			vel_pub.publish(command);
		}
		else
		{
			cout << "hovering" << endl; 
			vel_pub.publish(twist_msg_hover);
		}
		ros::spinOnce();
		if(cv::waitKey(1) >= 0)
			break; 
		loop_rate.sleep();
	}
}

void hover()
{
	ros::Rate loop_rate(50); 
	while(ros::ok())
	{
			geometry_msgs::Twist msg;
			msg.linear.x = msg.linear.y = msg.linear.z = 0; 
			msg.angular.x = msg.angular.y  = msg.angular.z = 0; 
			
			vel_pub.publish(msg);
		
		ros::spinOnce();
		if(cv::waitKey(1) >= 0)
			break; 
		loop_rate.sleep();
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "ardrone_controller");
	ros::NodeHandle nh;
	init();  
	takeoff_pub = nh.advertise<std_msgs::Empty>(nh.resolveName("ardrone/takeoff"),1);
	land_pub = nh.advertise<std_msgs::Empty>(nh.resolveName("ardrone/land"),1);
	vel_pub = nh.advertise<geometry_msgs::Twist>(nh.resolveName("cmd_vel"),1);
	face_sub = nh.subscribe(nh.resolveName("face/position"), 1000, &faceCb);

	cv::namedWindow(windowName);
	
	ros::Rate loop_rate(10);
	loop_rate.sleep();

	takeOff(); 
	//hover();
	
	control(); 
	
	land(); 

	return 0; 
}
