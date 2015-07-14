#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <iostream>
#include "RosThread.h"
#include <opencv2/opencv.hpp>

class RosThread;

using namespace cv;
using namespace std;

namespace Ui {
class MainWindow;
}

const string xmlFilePath = "/home/davy/RecognitionDatabase/database.xml";

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:

    RosThread* rosThread;

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void loadProfilePics();
    void setBatLvl(float lvl);
    void setControllerStatus(string status);
    void setUBStatus(string status);
    void setFaceStatus(string status);
    void setRecogStatus(string status);
    
private slots:


    void setIdentity();

    void on_Land_clicked();

    void on_TakeOff_clicked();

    void on_DetectFace_clicked();

    void on_DetectUB_clicked();

    void on_Hover_clicked();

    void on_TrackFace_clicked();

    void on_TrackUB_clicked();

    void on_Circling_clicked();

    void on_recognize_clicked();

    void on_Altitude_toggled(bool checked);

    void on_Yaw_toggled(bool checked);

    void on_Xdirect_toggled(bool checked);

protected:
    vector<Mat> profilePics;
    Mat unknownPic;

    void keyPressEvent( QKeyEvent * key);

private:
    Ui::MainWindow *ui;

    QTimer *timer;




};

#endif // MAINWINDOW_H
