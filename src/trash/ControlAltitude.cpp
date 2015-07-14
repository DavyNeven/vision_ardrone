//
// Created by davy on 4/30/15.
//

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"
#include <opencv2/opencv.hpp>
#include "PDcontroller.h"
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

PDcontroller *pd_y, *pd_x, *pd_alt;

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
    while((double)ros::Time::now().toSec() < time + 5 && ros::ok())
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

geometry_msgs::Twist calculateCommand(geometry_msgs::Twist position, ros::Duration dt)
{
    if(position.linear.z) // Face trackable -> control ardrone
    {
        cout << "Caculating command" << endl;
        geometry_msgs::Twist msg;
        msg.linear.x = msg.linear.y = msg.linear.z = 0;
        msg.angular.x = msg.angular.y = msg.angular.z = 0;

        double error;

        //PD-control altitude
        error =(180.0 - position.linear.y)/180;
        msg.linear.z = pd_alt->computeCommand(error, dt);

        //PD-control y pos
        error = (320.0 - position.linear.x)/320;
        msg.linear.y = pd_y->computeCommand(error, dt);

        //PD-control x pos
        error = (20000 - position.angular.x)/20000;
        msg.linear.x = pd_x->computeCommand(error, dt);

        // proportion control rotation
        //error = (320 - position.linear.x)/320;
        //msg.linear.z = error;

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
    static ros::Time last_time = ros::Time::now();
    ros::Time time = ros::Time::now();
    command = calculateCommand(*vel, time - last_time);
    tracker_enabled = true;
    last_time = time;
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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ardrone_altitude_controller");
    ros::NodeHandle nh;
    init();
    takeoff_pub = nh.advertise<std_msgs::Empty>(nh.resolveName("ardrone/takeoff"),1);
    land_pub = nh.advertise<std_msgs::Empty>(nh.resolveName("ardrone/land"),1);
    vel_pub = nh.advertise<geometry_msgs::Twist>(nh.resolveName("cmd_vel"),1);
    face_sub = nh.subscribe(nh.resolveName("face/position"), 1000, &faceCb);

    cv::namedWindow(windowName);

    ros::Rate loop_rate(10);
    loop_rate.sleep();

    //pd_x = new PDcontroller(0.09, 0.06);
    //pd_y = new PDcontroller(0.2,0.25);
    //pd_alt = new PDcontroller(0.6,0.06);

    pd_x = new PDcontroller(0.6, 0.06);
    pd_y = new PDcontroller(0.6,0.25);
    pd_alt = new PDcontroller(0.6,0.06);


    takeOff();

    control();

    land();

    return 0;
}


