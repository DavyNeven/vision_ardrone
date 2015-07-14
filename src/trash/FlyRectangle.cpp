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
geometry_msgs::Twist twist_msg_left;
geometry_msgs::Twist twist_msg_right;



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
	twist_msg_forward.linear.x=1.0;
	twist_msg_forward.linear.y=0.0;
	twist_msg_forward.linear.z=0.0;
	twist_msg_forward.angular.x=0.0;
	twist_msg_forward.angular.y=0.0;
	twist_msg_forward.angular.z=0.0; 

	//backward msg

	twist_msg_backward.linear.x=-1.0;
	twist_msg_backward.linear.y=0.0;
	twist_msg_backward.linear.z=0.0;
	twist_msg_backward.angular.x=0.0;
	twist_msg_backward.angular.y=0.0;
	
	//left
	twist_msg_left.linear.x=0.0;
	twist_msg_left.linear.y=0.8;
	twist_msg_left.linear.z=0.0;
	twist_msg_left.angular.x=0.0;
	twist_msg_left.angular.y=0.0;
	twist_msg_left.angular.z=-0.3; 
	//right
	twist_msg_right.linear.x=0.0;
	twist_msg_right.linear.y=-1.0;
	twist_msg_right.linear.z=0.0;
	twist_msg_right.angular.x=0.0;
	twist_msg_right.angular.y=0.0;
	twist_msg_right.angular.z=0.0; 

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
	while((double)ros::Time::now().toSec() < time + 6 && ros::ok())
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

void fly_rectangle()
{
	ros::Rate loop_rate(50); 
	double time = (double)ros::Time::now().toSec();
	cout << "Fly left" << endl; 
	while((double)ros::Time::now().toSec() < time + 21 && ros::ok())
	{ 		
		vel_pub.publish(twist_msg_left);
		ros::spinOnce();
		loop_rate.sleep(); 
	}
	/*cout << "Fly forward" << endl; 
	while((double)ros::Time::now().toSec() < time + 6 && ros::ok())
	{ 		
		vel_pub.publish(twist_msg_forward);
		ros::spinOnce();
		loop_rate.sleep(); 
	}
	cout << "Fly right" << endl; 
	while((double)ros::Time::now().toSec() < time + 12 && ros::ok())
	{ 		
		vel_pub.publish(twist_msg_right);
		ros::spinOnce();
		loop_rate.sleep(); 
	}
	cout << "Fly backward" << endl; 
	while((double)ros::Time::now().toSec() < time + 15 && ros::ok())
	{ 		
		vel_pub.publish(twist_msg_backward);
		ros::spinOnce();
		loop_rate.sleep(); 
	}
	cout << "Fly left" << endl; 
	while((double)ros::Time::now().toSec() < time + 18 && ros::ok())
	{ 		
		vel_pub.publish(twist_msg_left);
		ros::spinOnce();
		loop_rate.sleep(); 
	}*/
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "fly_rectangle");
	ros::NodeHandle nh;
	init();  
	takeoff_pub = nh.advertise<std_msgs::Empty>(nh.resolveName("ardrone/takeoff"),1);
	land_pub = nh.advertise<std_msgs::Empty>(nh.resolveName("ardrone/land"),1);
	vel_pub = nh.advertise<geometry_msgs::Twist>(nh.resolveName("cmd_vel"),1);
	

	ros::Rate loop_rate(10);
	loop_rate.sleep();

	takeOff(); 
	fly_rectangle();
	land();

	return 0; 
}
