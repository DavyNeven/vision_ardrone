#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtCore>
#include <QKeyEvent>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    loadProfilePics();
    unknownPic = imread("/home/ardrone/RecognitionDatabase/unknown.jpg", CV_LOAD_IMAGE_COLOR);
    cvtColor(unknownPic, unknownPic, CV_BGR2RGB);
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(setIdentity()));
    timer->start(500);
}

void MainWindow::setBatLvl(float lvl)
{
    ui->batteryStatus->setText(QString::number(lvl));
}

void MainWindow::setControllerStatus(string status)
{
    QString s = QString::fromStdString(status);
    ui->controllerStatus->setText(s);
}
void MainWindow::setUBStatus(string status)
{
    QString s = QString::fromStdString(status);
    ui->ubStatus->setText(s);
}
void MainWindow::setFaceStatus(string status)
{
    QString s = QString::fromStdString(status);
    ui->faceStatus->setText(s);
}
void MainWindow::setRecogStatus(string status)
{
    QString s = QString::fromStdString(status);
    ui->recognizerStatus->setText(s);
}

void MainWindow::loadProfilePics()
{
    FileStorage fs(xmlFilePath, FileStorage::READ);
    if(!fs.isOpened())
    {
        cerr << "xml file does not exist!" << endl;
        return;
    }
    // reading images
    FileNode n = fs["profile_images"];
    if (n.type() != FileNode::SEQ)
    {
        cerr << "profile_images is not a sequence! FAIL" << endl;
    }
    FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
    for (; it != it_end; ++it)
    {
        string fileName = (string)*it;
        Mat img = imread(fileName, CV_LOAD_IMAGE_ANYCOLOR);
        cvtColor(img, img, CV_BGR2RGB);
        profilePics.push_back(img);
    }
}

void MainWindow::setIdentity() {
    int label = rosThread->identity;
    if(label == -1 || label >= profilePics.size()) {
        QImage qImage((uchar*)unknownPic.data, unknownPic.cols, unknownPic.rows, unknownPic.step, QImage::Format_RGB888);
        ui->Identity->setPixmap(QPixmap::fromImage(qImage));
        ui->Identity->show();
    }
    else {
        QImage qImage((uchar *) profilePics[label].data, profilePics[label].cols, profilePics[label].rows, profilePics[label].step,
                      QImage::Format_RGB888);
        ui->Identity->setPixmap(QPixmap::fromImage(qImage));
        ui->Identity->show();
   }
    setBatLvl(rosThread->batteryStatus);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_Land_clicked()
{
    rosThread->land();
}

void MainWindow::on_TakeOff_clicked()
{
    rosThread->takeOff();
}

void MainWindow::on_DetectFace_clicked()
{
    rosThread->startFaceDetection();
}

void MainWindow::on_DetectUB_clicked()
{
    rosThread->startUBDetection();
}

void MainWindow::on_Hover_clicked()
{
    rosThread->hover();
}

void MainWindow::on_TrackFace_clicked()
{
    rosThread->startFaceTracking();
}

void MainWindow::on_TrackUB_clicked()
{
    rosThread->startUBTracking();
}

void MainWindow::on_recognize_clicked()
{
    rosThread->recognize();
}

void MainWindow::on_Altitude_toggled(bool checked)
{
    rosThread->changeControl(ui->Xdirect->isChecked(), ui->Yaw->isChecked(), checked);
}

void MainWindow::on_Yaw_toggled(bool checked)
{
    rosThread->changeControl(ui->Xdirect->isChecked(), checked, ui->Altitude->isChecked());
}

void MainWindow::on_Xdirect_toggled(bool checked)
{
    rosThread->changeControl(checked,ui->Yaw->isChecked(), ui->Altitude->isChecked());
}

void MainWindow::on_Circling_clicked()
{
    rosThread->flyCircle();
}

void MainWindow::keyPressEvent( QKeyEvent * key)
{
    switch((char)(key->key()))
    {
        case Qt::Key_H:
            rosThread->hover();
            break;
        case Qt::Key_T:   //t
           rosThread->takeOff();
            break;
        case Qt::Key_L:   //l
            rosThread->land();
            break;
        case Qt::Key_O:   //o
            rosThread->startUBDetection();
            break;
        case Qt::Key_F:   //f
            rosThread->startFaceDetection();
            break;
        case Qt::Key_S:   //s
            rosThread->startUBTracking();
            break;
        case Qt::Key_D:   //d:
            rosThread->startFaceTracking();
            break;
        case Qt::Key_C:   //c:
            rosThread->flyCircle();
            break;
        default:
            rosThread->hover();
            break;
    }
}




