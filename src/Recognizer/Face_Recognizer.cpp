//
// Created by davy on 5/1/15.
//

#include "Face_Recognizer.h"

Face_Recognizer::Face_Recognizer(char const *facerecAlgorithm) {
    facesModel = Algorithm::create<FaceRecognizer>(facerecAlgorithm);
    if (facesModel.empty()) {
        cout << "ERROR: The required faceRecognitionAlgorithm is not available in this version of OpenCV. Update to OpenCV v2.4.1 or newer." << endl;
        exit(1);
    }
}

Mat Face_Recognizer::reconstructFace(const Mat face){

    Mat eigenvectors = facesModel->get<Mat>("eigenvectors");
    Mat averageFaceRow = facesModel->get<Mat>("mean");

    int faceHeight = face.rows;

    // Project the input image onto the PCA subspace.
    Mat projection = subspaceProject(eigenvectors, averageFaceRow, face.reshape(1,1));

    // Generate the reconstructed face back from the PCA subspace.
    Mat reconstructionRow = subspaceReconstruct(eigenvectors, averageFaceRow, projection);


    // reshape it to the image size
    Mat reconstructionMat = reconstructionRow.reshape(1, faceHeight);
    // Convert the floating-point pixels to regular 8-bit uchar pixels.
    Mat reconstructedFace = Mat(reconstructionMat.size(), CV_8U);
    reconstructionMat.convertTo(reconstructedFace, CV_8U, 1, 0);

    return reconstructedFace;
}



double Face_Recognizer::calculateL2error(const Mat faceA, const Mat faceB){

    if (faceA.rows > 0 && faceA.rows == faceB.rows && faceA.cols > 0 && faceA.cols == faceB.cols) {
        // Calculate the L2 relative error between the 2 images.
        double errorL2 = norm(faceA, faceB, CV_L2);
        // Convert to a reasonable scale, since L2 error is summed across all pixels of the image.
        double similarity = errorL2 / (double)(faceA.rows * faceA.cols);
        return similarity;
    }
    else {
        //cout << "WARNING: Images have a different size in 'getSimilarity()'." << endl;
        return 100000000.0;  // Return a bad value
    }
}

int Face_Recognizer::predict(const Mat face){
    return facesModel->predict(face);
}
Mat Face_Recognizer::getImageFrom1DFloatMat(const Mat matrixRow, int height){
    // Reshape the image
    Mat rectangularMat = matrixRow.reshape(1, height);
    // Scale values from float to 8bit
    Mat dst;
    normalize(rectangularMat, dst, 0, 255, NORM_MINMAX, CV_8UC1);
    return dst;
}

bool Face_Recognizer::loadModel(const char *fileName) {
    try {
        facesModel->load(fileName);
        return true;
    }
    catch (cv::Exception &e) {}
    return false;
}




