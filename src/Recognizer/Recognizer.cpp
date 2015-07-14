//
// Created by davy on 5/1/15.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Empty.h>
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/opencv.h>
#include "Face_Recognizer.h"
#include "../tools/PreprocessFace.h"
#include "vision_ardrone/ControlCommand.h"
#include "vision_ardrone/Identity.h"
#include "vision_ardrone/Status.h"

const char *shapePredictorFileName = "/home/ardrone/shape_predictor_68_face_landmarks.dat";
const char* faceModel_filename = "/home/ardrone/RecognitionDatabase/FaceModel.yml";
const int numFacesToCollect = 10;
const int faceWidth = 70;
const float UNKNOWN_PERSON_THRESHOLD = 0.35f;

using namespace cv;
using namespace std;

Face_Recognizer *recognizer;
PreprocessFace *facePreprocessor;
dlib::frontal_face_detector detector;

image_transport::Subscriber img_sub;
image_transport::ImageTransport *it;

ros::Subscriber sub_control_commands;

ros::Publisher pub_identity;
ros::Publisher status_pub;

vector<Mat> faces;
int numFaces = 0;

bool collectFaces = false;

enum STATES {IDLE=0, COLLECT_FACES, RECOGNIZE};
STATES state = IDLE;

void publish_status(const ros::TimerEvent&)
{
    vision_ardrone::Status s;
    s.type = vision_ardrone::Status::RECOGNIZER;
    switch(state)
    {
        case IDLE:
            s.status = vision_ardrone::Status::IDLE;
            break;
        case COLLECT_FACES:
            s.status = vision_ardrone::Status::COLLECT_FACES;
            break;
        case RECOGNIZE:
            s.status = vision_ardrone::Status::RECOGNIZE;
            break;
        default:
            break;
    }
    status_pub.publish(s);
}


void ControlCb(const vision_ardrone::ControlCommand c)
{
    switch(c.command)
    {
        case vision_ardrone::ControlCommand::RECOGNIZE:
            ROS_ERROR("received: recognize");
            state = COLLECT_FACES;
            break;
        default:
            ROS_ERROR("received: unkown");
            break;
    }
}

void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    if(collectFaces) {

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        Mat webcamImage = cv_ptr->image;

        dlib::cv_image<dlib::bgr_pixel> cv(webcamImage);
        dlib::array2d<dlib::rgb_pixel> imgDlib;
        dlib::assign_image(imgDlib, cv);

        std::vector<dlib::rectangle> dets = detector(imgDlib);
        if (dets.size() > 0) {
            Rect bb = Rect(dets[0].left(), dets[0].top(), dets[0].width(),
                           dets[0].height()) & Rect(0, 0, 640, 360);
            Mat face = webcamImage(bb);
            Mat preprocessedFace = facePreprocessor->getPreprocessedFace(face, faceWidth);
            faces.push_back(preprocessedFace);
            numFaces++;
        }
        if (numFaces == numFacesToCollect){
            state = RECOGNIZE;
            collectFaces = false;
        }

    }
}

void publish_identity(int id)
{
    ROS_ERROR("Identity is: %d", id);
    vision_ardrone::Identity i;
    i.identity = id;
    pub_identity.publish(i);
}

void do_states()
{
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        switch(state)
        {
            case IDLE:

               break;
            case COLLECT_FACES:
                numFaces = 0;
                faces.clear();
                collectFaces = true;
                img_sub = it->subscribe("/ardrone/image_raw", 1,&imageCb);
                state = IDLE;
                break;
            case RECOGNIZE:
                img_sub.shutdown();
                vector<Mat> reconstructedFaces;
                // Calculate reconstructed faces
                for(int i = 0; i< faces.size(); i++)
                {
                    reconstructedFaces.push_back(recognizer->reconstructFace(faces[i]));
                }
                // Calculate L2-error to see if the person is in the database
                vector<bool> inDatabase;
                for(int i = 0; i< faces.size(); i++)
                {
                    double L2 = recognizer->calculateL2error(faces[i], reconstructedFaces[i]);
                    inDatabase.push_back(L2 < UNKNOWN_PERSON_THRESHOLD);
                }
                // Predict person only on faces with OK L2-error
                vector<int> labels;
                for(int i = 0; i<faces.size(); i++)
                {
                    if(inDatabase[i])
                    {
                        labels.push_back(recognizer->predict(faces[i]));
                    }
                }
                if(labels.size() == 0){
                    publish_identity(-1);
                    ROS_INFO("Identity unknown! Not in database");
                    publish_identity(-1);
                    state = IDLE;
                    break;
                }
                vector<int> votes(10);

                // Take the majority vote
                for(int i = 0; i< labels.size(); i++)
                {
                    int pos = labels[i];
                    votes[pos] = votes[pos] + 1;
                }
                int identity = -1;
                int maxVotes = -1;
                for(int i = 0; i< votes.size(); i++)
                {
                    if(votes[i] > maxVotes)
                    {
                        identity = i;
                        maxVotes = votes[i];
                    }
                }
                publish_identity(identity);
                state = IDLE;
                break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "ardrone_FACE_recognizer");
    ros::NodeHandle nh;

    it = new image_transport::ImageTransport(nh);

    sub_control_commands = nh.subscribe(nh.resolveName("/ardrone_master/RecognizerCommands"), 1000, &ControlCb);
    pub_identity =  nh.advertise<vision_ardrone::Identity>("/ardrone_recognizer/Identity", 1000);
    status_pub = nh.advertise<vision_ardrone::Status>("/ardrone_recognizer/Status", 1000);

    recognizer = new Face_Recognizer();
    if(!recognizer->loadModel(faceModel_filename))
    {
        ROS_ERROR("Can't load face Model! ");
        cout << "Can't load face Model: " << faceModel_filename << "does not exist!" << endl;
    }
    detector = dlib::get_frontal_face_detector();
    facePreprocessor = new PreprocessFace(shapePredictorFileName);

    // Publish status every 200 ms
    ros::Timer timer = nh.createTimer(ros::Duration(0.2), &publish_status);

    do_states();

    ros::spin();
    return 0;
}
