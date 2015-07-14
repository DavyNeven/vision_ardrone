/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Mon May 4 12:47:36 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGroupBox *groupBox;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLineEdit *controllerStatus;
    QLineEdit *faceStatus;
    QLineEdit *ubStatus;
    QLineEdit *recognizerStatus;
    QLabel *label_5;
    QLineEdit *batteryStatus;
    QGroupBox *groupBox_2;
    QPushButton *Land;
    QPushButton *TakeOff;
    QPushButton *DetectFace;
    QPushButton *TrackFace;
    QPushButton *DetectUB;
    QPushButton *TrackUB;
    QPushButton *Hover;
    QPushButton *Circling;
    QGroupBox *groupBox_3;
    QLabel *Identity;
    QPushButton *recognize;
    QGroupBox *groupBox_4;
    QCheckBox *Altitude;
    QCheckBox *Yaw;
    QCheckBox *Xdirect;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(674, 455);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(30, 20, 271, 211));
        QFont font;
        font.setPointSize(16);
        font.setBold(true);
        font.setItalic(false);
        font.setWeight(75);
        font.setStrikeOut(false);
        groupBox->setFont(font);
        groupBox->setAcceptDrops(false);
        groupBox->setAutoFillBackground(true);
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(20, 40, 121, 31));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(20, 70, 121, 31));
        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(20, 100, 121, 31));
        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(20, 130, 121, 31));
        controllerStatus = new QLineEdit(groupBox);
        controllerStatus->setObjectName(QString::fromUtf8("controllerStatus"));
        controllerStatus->setGeometry(QRect(150, 40, 113, 27));
        QFont font1;
        font1.setPointSize(12);
        font1.setBold(false);
        font1.setWeight(50);
        controllerStatus->setFont(font1);
        controllerStatus->setReadOnly(true);
        faceStatus = new QLineEdit(groupBox);
        faceStatus->setObjectName(QString::fromUtf8("faceStatus"));
        faceStatus->setGeometry(QRect(150, 70, 113, 27));
        faceStatus->setFont(font1);
        faceStatus->setReadOnly(true);
        ubStatus = new QLineEdit(groupBox);
        ubStatus->setObjectName(QString::fromUtf8("ubStatus"));
        ubStatus->setGeometry(QRect(150, 100, 113, 27));
        ubStatus->setFont(font1);
        ubStatus->setReadOnly(true);
        recognizerStatus = new QLineEdit(groupBox);
        recognizerStatus->setObjectName(QString::fromUtf8("recognizerStatus"));
        recognizerStatus->setGeometry(QRect(150, 130, 113, 27));
        recognizerStatus->setFont(font1);
        recognizerStatus->setReadOnly(true);
        label_5 = new QLabel(groupBox);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(20, 160, 121, 31));
        batteryStatus = new QLineEdit(groupBox);
        batteryStatus->setObjectName(QString::fromUtf8("batteryStatus"));
        batteryStatus->setGeometry(QRect(150, 160, 113, 27));
        batteryStatus->setFont(font1);
        batteryStatus->setReadOnly(true);
        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(30, 230, 271, 191));
        QFont font2;
        font2.setPointSize(16);
        font2.setBold(true);
        font2.setItalic(false);
        font2.setWeight(75);
        groupBox_2->setFont(font2);
        groupBox_2->setAutoFillBackground(true);
        Land = new QPushButton(groupBox_2);
        Land->setObjectName(QString::fromUtf8("Land"));
        Land->setGeometry(QRect(40, 40, 98, 27));
        QFont font3;
        font3.setPointSize(12);
        font3.setBold(false);
        font3.setItalic(false);
        font3.setWeight(50);
        Land->setFont(font3);
        TakeOff = new QPushButton(groupBox_2);
        TakeOff->setObjectName(QString::fromUtf8("TakeOff"));
        TakeOff->setGeometry(QRect(150, 40, 98, 27));
        TakeOff->setFont(font3);
        DetectFace = new QPushButton(groupBox_2);
        DetectFace->setObjectName(QString::fromUtf8("DetectFace"));
        DetectFace->setGeometry(QRect(40, 70, 98, 27));
        DetectFace->setFont(font3);
        TrackFace = new QPushButton(groupBox_2);
        TrackFace->setObjectName(QString::fromUtf8("TrackFace"));
        TrackFace->setGeometry(QRect(150, 70, 98, 27));
        TrackFace->setFont(font3);
        DetectUB = new QPushButton(groupBox_2);
        DetectUB->setObjectName(QString::fromUtf8("DetectUB"));
        DetectUB->setGeometry(QRect(40, 100, 98, 27));
        DetectUB->setFont(font3);
        TrackUB = new QPushButton(groupBox_2);
        TrackUB->setObjectName(QString::fromUtf8("TrackUB"));
        TrackUB->setGeometry(QRect(150, 100, 98, 27));
        TrackUB->setFont(font3);
        Hover = new QPushButton(groupBox_2);
        Hover->setObjectName(QString::fromUtf8("Hover"));
        Hover->setGeometry(QRect(40, 130, 98, 27));
        Hover->setFont(font3);
        Circling = new QPushButton(groupBox_2);
        Circling->setObjectName(QString::fromUtf8("Circling"));
        Circling->setGeometry(QRect(150, 130, 98, 27));
        Circling->setFont(font3);
        groupBox_3 = new QGroupBox(centralWidget);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setGeometry(QRect(360, 20, 291, 191));
        groupBox_3->setFont(font2);
        groupBox_3->setAutoFillBackground(true);
        Identity = new QLabel(groupBox_3);
        Identity->setObjectName(QString::fromUtf8("Identity"));
        Identity->setGeometry(QRect(150, 40, 131, 141));
        Identity->setAutoFillBackground(true);
        Identity->setScaledContents(true);
        recognize = new QPushButton(groupBox_3);
        recognize->setObjectName(QString::fromUtf8("recognize"));
        recognize->setGeometry(QRect(0, 40, 121, 27));
        QFont font4;
        font4.setPointSize(12);
        recognize->setFont(font4);
        groupBox_4 = new QGroupBox(centralWidget);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setGeometry(QRect(360, 230, 271, 191));
        groupBox_4->setFont(font2);
        groupBox_4->setAutoFillBackground(true);
        Altitude = new QCheckBox(groupBox_4);
        Altitude->setObjectName(QString::fromUtf8("Altitude"));
        Altitude->setGeometry(QRect(40, 40, 161, 22));
        Yaw = new QCheckBox(groupBox_4);
        Yaw->setObjectName(QString::fromUtf8("Yaw"));
        Yaw->setGeometry(QRect(40, 70, 141, 22));
        Xdirect = new QCheckBox(groupBox_4);
        Xdirect->setObjectName(QString::fromUtf8("Xdirect"));
        Xdirect->setGeometry(QRect(40, 100, 141, 22));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 674, 25));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainWindow", "Status", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-style:normal;\">Controller :</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-style:normal;\">Face Detector :</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-style:normal;\">UB Detector :</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt; font-style:normal;\">Recognizer :</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:12pt;\">Battery :</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "Actions", 0, QApplication::UnicodeUTF8));
        Land->setText(QApplication::translate("MainWindow", "Land", 0, QApplication::UnicodeUTF8));
        TakeOff->setText(QApplication::translate("MainWindow", "Take Off", 0, QApplication::UnicodeUTF8));
        DetectFace->setText(QApplication::translate("MainWindow", "Detect Face", 0, QApplication::UnicodeUTF8));
        TrackFace->setText(QApplication::translate("MainWindow", "Track Face", 0, QApplication::UnicodeUTF8));
        DetectUB->setText(QApplication::translate("MainWindow", "Detect UB", 0, QApplication::UnicodeUTF8));
        TrackUB->setText(QApplication::translate("MainWindow", "Track UB", 0, QApplication::UnicodeUTF8));
        Hover->setText(QApplication::translate("MainWindow", "Hover", 0, QApplication::UnicodeUTF8));
        Circling->setText(QApplication::translate("MainWindow", "Circle", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("MainWindow", "Identity", 0, QApplication::UnicodeUTF8));
        Identity->setText(QString());
        recognize->setText(QApplication::translate("MainWindow", "Recognize", 0, QApplication::UnicodeUTF8));
        groupBox_4->setTitle(QApplication::translate("MainWindow", "Control", 0, QApplication::UnicodeUTF8));
        Altitude->setText(QApplication::translate("MainWindow", "Altitude", 0, QApplication::UnicodeUTF8));
        Yaw->setText(QApplication::translate("MainWindow", "Yaw", 0, QApplication::UnicodeUTF8));
        Xdirect->setText(QApplication::translate("MainWindow", "X direction", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
