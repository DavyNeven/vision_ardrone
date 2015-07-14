#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>


using namespace std; 

ros::Publisher takeoff_pub;
ros::Publisher land_pub;
ros::Publisher vel_pub; 
ros::Subscriber pedestrian_sub; 
geometry_msgs::Twist position;
geometry_msgs::Twist twist_msg_hover;
geometry_msgs::Twist twist_msg_up;
geometry_msgs::Twist twist_msg_down;
geometry_msgs::Twist twist_msg_forward;
geometry_msgs::Twist twist_msg_backward;



int tracking = 0; 
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

	//down msg

	twist_msg_down.linear.x=0;
	twist_msg_down.linear.y=0.0;
	twist_msg_down.linear.z=-1;
	twist_msg_down.angular.x=0.0;
	twist_msg_down.angular.y=0.0;
	
	//forward msg
	twist_msg_forward.linear.x=0.5;
	twist_msg_forward.linear.y=0.0;
	twist_msg_forward.linear.z=0.0;
	twist_msg_forward.angular.x=0.0;
	twist_msg_forward.angular.y=0.0;
	twist_msg_forward.angular.z=0.0; 

	//backward msg

	twist_msg_backward.linear.x=-0.5;
	twist_msg_backward.linear.y=0.0;
	twist_msg_backward.linear.z=0.0;
	twist_msg_backward.angular.x=0.0;
	twist_msg_backward.angular.y=0.0;

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
void pedestrianCb(const geometry_msgs::TwistConstPtr vel)
{
	tracking = 1;
	position = *vel;
}
void track()
{
	cout << position.angular.y << endl; 
	geometry_msgs::Twist msg;
	if(position.linear.x > (320 - (position.angular.x)/2))
		msg.linear.y = -0.2;
	else
		msg.linear.y = 0.2; 
	if(position.linear.y +  (position.angular.y)  >  320)
		msg.linear.z = -0.2;
	else
		msg.linear.z = 0.2;

	if(position.angular.y > 150)
		msg.linear.x = -0.2;
	else
		msg.linear.x = 0.2;
	msg.angular.x = 0; 
	msg.angular.y = 0; 
	msg.angular.z = 0; 
	vel_pub.publish(msg); 
}
void patrol()
{
	// move forward
	cout << "moving forward" << endl; 
	vel_pub.publish(twist_msg_forward);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ardrone_controller");
	ros::NodeHandle nh;
	init();  
	takeoff_pub = nh.advertise<std_msgs::Empty>(nh.resolveName("ardrone/takeoff"),1);
	land_pub = nh.advertise<std_msgs::Empty>(nh.resolveName("ardrone/land"),1);
	vel_pub = nh.advertise<geometry_msgs::Twist>(nh.resolveName("cmd_vel"),1);
	pedestrian_sub = nh.subscribe(nh.resolveName("pedestrian/position"), 1000, &pedestrianCb);
	

	ros::Rate loop_rate(10);
	loop_rate.sleep();

	takeOff(); 

	
	while(ros::ok())
	{	
		if(tracking)
			track(); 
		else
			patrol();
		ros::spinOnce(); 
		loop_rate.sleep();
	}
	return 0; 
}
