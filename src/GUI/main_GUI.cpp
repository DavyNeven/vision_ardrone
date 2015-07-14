//
// Created by davy on 5/2/15.
//



#include <QtGui>
#include <QApplication>
#include "ros/ros.h"
#include "mainwindow.h"
#include "RosThread.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "drone_gui");

    RosThread t;

    QApplication a(argc, argv);
    MainWindow w;
    w.rosThread = &t;

    t.gui = &w;

    t.startSystem();
    w.show();

    return a.exec();
}