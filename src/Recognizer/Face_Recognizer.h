//
// Created by davy on 5/1/15.
//

#ifndef VISION_ARDRONE_FACERECOGNIZER_H
#define VISION_ARDRONE_FACERECOGNIZER_H


#include <stdio.h>
#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;


class Face_Recognizer {

protected:
    Ptr<FaceRecognizer> facesModel;

public:
    Face_Recognizer(const char *facerecAlgorithm= "FaceRecognizer.Eigenfaces");

    //Reconstructs the face by backprojecting the eigenfaces
    // for the given face. If the person is in the database then this
    // should yield a good reconstruction
    Mat reconstructFace(const Mat face);

    // Calculates the square root of the sum of squared errors between
    // the 2 faces
    double calculateL2error(const Mat faceA, const Mat faceB);

    int predict(const Mat face);

    Mat getImageFrom1DFloatMat(const Mat matrixRow, int height);

    bool loadModel(const char* fileName);

};


#endif //VISION_ARDRONE_FACERECOGNIZER_H