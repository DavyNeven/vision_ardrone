//
// Created by davy on 4/25/15.
//

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"
#include "vision_ardrone/ControlCommand.h"
#include "vision_ardrone/Detection.h"
#include "vision_ardrone/Status.h"
#include "../tools/PDcontroller.h"
#include <stdio.h>


using namespace std;

ros::Publisher takeoff_pub;
ros::Publisher land_pub;
ros::Publisher vel_pub;
ros::Publisher status_pub;

ros::Subscriber sub_control_commands;
ros::Subscriber sub_ub_detector;
ros::Subscriber sub_face_detector;

geometry_msgs::Twist twist_msg_hover;
geometry_msgs::Twist twist_msg_up;
geometry_msgs::Twist twist_msg_turn;

geometry_msgs::Twist velCommand;

PDcontroller *pd_x, *pd_alt;

bool controlAltitude = false;
bool controlX = false;
bool controlYaw = false;

enum STATES {TAKE_OFF=0,HOVERING, LANDING, IDLE, TRACK_UB, TRACK_FACE, FLY_CIRCLE};

STATES state = IDLE;

void init() {
    //hover msg
    twist_msg_hover.linear.x = 0.0;
    twist_msg_hover.linear.y = 0.0;
    twist_msg_hover.linear.z = 0.0;
    twist_msg_hover.angular.x = 0.0;
    twist_msg_hover.angular.y = 0.0;
    twist_msg_hover.angular.z = 0.0;

    //up msg

    twist_msg_up.linear.x = 0;
    twist_msg_up.linear.y = 0.0;
    twist_msg_up.linear.z = 1;
    twist_msg_up.angular.x = 0.0;
    twist_msg_up.angular.y = 0.0;

    //turn msg

    twist_msg_turn.linear.x = 0.0;
    twist_msg_turn.linear.y = 0.5;
    twist_msg_turn.linear.z = 0.0;
    twist_msg_turn.angular.x = 0.0;
    twist_msg_turn.angular.y = 0.0;
    twist_msg_turn.angular.z = -0.3;
}
void publish_status(const ros::TimerEvent&)
{
    vision_ardrone::Status s;
    s.type = vision_ardrone::Status::CONTROLLER;
    switch(state)
    {
        case TAKE_OFF:
            s.status = vision_ardrone::Status::TAKING_OFF;
            break;
        case HOVERING:
            s.status = vision_ardrone::Status::HOVERING;
            break;
        case LANDING:
            s.status = vision_ardrone::Status::LANDING;
            break;
        case IDLE:
            s.status = vision_ardrone::Status::IDLE;
            break;
        case TRACK_UB:
            s.status = vision_ardrone::Status::TRACKING_UB;
            break;
        case TRACK_FACE:
            s.status = vision_ardrone::Status::TRACKING_FACE;
            break;
        case FLY_CIRCLE:
            s.status = vision_ardrone::Status::CIRCLING;
            break;
        default:
            break;
    }
    status_pub.publish(s);
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
     time = (double)ros::Time::now().toSec();
    while((double)ros::Time::now().toSec() < time + 3 && ros::ok())
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

void hover()
{
    vel_pub.publish(twist_msg_hover);
}


geometry_msgs::Twist calculateVelCommand(vision_ardrone::Detection d, ros::Duration dt)
{
    geometry_msgs::Twist msg;
    msg.linear.x = msg.linear.y = msg.linear.z = 0;
    msg.angular.x = msg.angular.y = msg.angular.z = 0;

    int centerX = d.x + d.width/2;
    int centerY = d.y + d.height/2;

    if(centerX > 640) centerX = 640;
    if(centerX < 0) centerX = 0;

    if(centerY > 360 ) centerY = 360;
    if(centerY < 0) centerY = 0;

    cout << "centerX: " << centerX << "CenterY: " << centerY << endl;

    double error;

    //PD-control altitude
    if(controlAltitude)
    {
        error =(180.0 - centerY)/180;
        msg.linear.z = pd_alt->computeCommand(error, dt)*0.8;
    }
    cout << "Altitude: " << msg.linear.z << endl;

    //PD-control x pos
    // Take sqrt of size to make it linear with the horizontal speed
    int size = sqrt(d.size);
    if(controlX)
    {
        if(state == TRACK_FACE){
            if(size > 200)
                size = 200;
            error = (100.0 - size)/100.0;
        }
        else {
            if(size > 560)
                size = 560;
            error = (280.0 - size)/280.0;
        }
        msg.linear.x = pd_x->computeCommand(error, dt)*0.7;
    }

    cout << "X-control: " << msg.linear.x << endl;

    // proportion control rotation
    if(controlYaw)
    {
        error = (320.0 - centerX)/320;
        msg.angular.z = error;
    }

    cout << "Yaw: " << msg.angular.z << endl;

    return msg;
}

void detectorCb(vision_ardrone::Detection d)
{
    static ros::Time last_time = ros::Time::now();
    ros::Time time = ros::Time::now();
    switch(d.type) {
        case vision_ardrone::Detection::UB:
            if(state == TRACK_UB  || state == FLY_CIRCLE)
            {
                if(d.status == vision_ardrone::Detection::NOT_LOCKED)
                {
                    velCommand = twist_msg_hover;
                }
                else
                {
                    velCommand = calculateVelCommand(d, time - last_time);
                }
                last_time = time;
            }
            break;
        case vision_ardrone::Detection::FACE:
            if(state == TRACK_FACE)
            {
                if(d.status == vision_ardrone::Detection::NOT_LOCKED)
                {
                    velCommand = twist_msg_hover;
                }
                else
                {
                    velCommand = calculateVelCommand(d, time - last_time);
                }
                last_time = time;
            }
            break;
        default:
            break;
    }

}

void controlCb(const vision_ardrone::ControlCommand c)
{
    switch(c.command)
    {
        case vision_ardrone::ControlCommand::TAKE_OFF:
            cout << "received: Taking off" << endl;
            state = TAKE_OFF;
            break;
        case vision_ardrone::ControlCommand::LAND:
            cout << "received: land" << endl;
            state = LANDING;
            break;
        case vision_ardrone::ControlCommand::HOVER:
            cout << "received: hover"<< endl;
            state = HOVERING;
            break;
        case vision_ardrone::ControlCommand::TRACK_UB:
            cout << "received: start tracking UB" << endl;
            state = TRACK_UB;
            break;
        case vision_ardrone::ControlCommand::TRACK_FACE:
            cout << "received: start tracking face" << endl;
            state = TRACK_FACE;
            break;
        case vision_ardrone::ControlCommand::FLY_CIRCLE:
            cout << "received: fly circle" << endl;
            state = FLY_CIRCLE;
            break;
        case vision_ardrone::ControlCommand::CHANGE_CONTROL:
            cout << "received: changing control" << endl;
            controlYaw = c.controlYaw;
            controlAltitude = c.controlAltitude;
            controlX = c.controlX;
            break;
        default:
            cout << "received: Unkown" << endl;
            break;
    }
}

void control_state()
{
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        switch(state)
        {
            case TAKE_OFF:
                takeOff();
                state = HOVERING;
                break;
            case HOVERING:
                hover();
                break;
            case LANDING:
                land();
                state = IDLE;
                break;
            case TRACK_UB:
                vel_pub.publish(velCommand);
                break;
            case TRACK_FACE:
                vel_pub.publish(velCommand);
                break;
            case FLY_CIRCLE:
            	{
            	geometry_msgs::Twist turn;
            	turn.linear.y = 0.35; 
            	turn.linear.x = velCommand.linear.x; 
            	turn.angular.z = velCommand.angular.z; 
                vel_pub.publish(turn);
                }
                break;
            case IDLE:
                hover();
                break;
        }
        ros::spinOnce();
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
    status_pub = nh.advertise<vision_ardrone::Status>("/ardrone_controller/Status", 1000);

    sub_control_commands = nh.subscribe(nh.resolveName("/ardrone_master/ControllerCommands"), 1000, &controlCb);
    sub_ub_detector = nh.subscribe(nh.resolveName("/ardrone_UB_detector/detection"), 1000, &detectorCb);
    sub_face_detector = nh.subscribe(nh.resolveName("/ardrone_FACE_detector/detection"), 1000, &detectorCb);

    // Publish status every 200 ms
    ros::Timer timer = nh.createTimer(ros::Duration(0.2), &publish_status);

    pd_x = new PDcontroller(0.3, 0.15);
    pd_alt = new PDcontroller(0.9,0.03);

    /*pd_x = new PDcontroller(0.6, 0.06);
    pd_alt = new PDcontroller(0.6,0.06);*/

    ros::Rate loop_rate(10);
    loop_rate.sleep();

    control_state();

    return 0;
}
