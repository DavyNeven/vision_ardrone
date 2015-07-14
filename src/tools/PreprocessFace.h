//
// Created by davy on 5/1/15.
//

#ifndef VISION_ARDRONE_PREPROCESSFACE_H
#define VISION_ARDRONE_PREPROCESSFACE_H


#include "opencv2/opencv.hpp"

//include dlib facedetector

#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>



using namespace cv;
using namespace std;

#define DESIRED_LEFT_EYE_X 0.16     // Controls how much of the face is visible after preprocessing.
#define DESIRED_LEFT_EYE_Y 0.14
#define FACE_ELLIPSE_CY 0.40
#define FACE_ELLIPSE_W 0.50         // Should be atleast 0.5
#define FACE_ELLIPSE_H 0.80         // Controls how tall the face mask is.

class PreprocessFace {

protected:
    dlib::shape_predictor shapePredictor;

public:

    PreprocessFace(const char *shapePredictorFileName);

    void detectBothEyes(const Mat &face, Point &leftEye, Point &rightEye);

    void equalizeLeftAndRightHalves(Mat &faceImg);

    Mat getPreprocessedFace(Mat &faceImg, int desiredFaceWidth);

};


#endif //VISION_ARDRONE_PREPROCESSFACE_H
