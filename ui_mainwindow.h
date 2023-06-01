/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *lbLeftImage;
    QLabel *lbRightImage;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QSpacerItem *verticalSpacer_4;
    QPushButton *btCameras;
    QSpacerItem *verticalSpacer_6;
    QPushButton *btCapture;
    QSpacerItem *verticalSpacer_3;
    QPushButton *btCalibrateCheckerBoard;
    QSpacerItem *verticalSpacer;
    QPushButton *btCalibrateSquareBar;
    QSpacerItem *verticalSpacer_7;
    QPushButton *btFetchRailCrown;
    QSpacerItem *verticalSpacer_2;
    QPushButton *btExit;
    QSpacerItem *verticalSpacer_5;
    QWidget *horizontalLayoutWidget_3;
    QHBoxLayout *horizontalLayout_3;
    QLabel *lbLeftImage_2;
    QLabel *lbRightImage_2;
    QLabel *lbFused3DImage;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(840, 770);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setMaximumSize(QSize(16777215, 16777215));
        QFont font;
        font.setFamily(QString::fromUtf8("Copperplate Gothic Bold"));
        font.setPointSize(12);
        MainWindow->setFont(font);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayoutWidget_2 = new QWidget(centralwidget);
        horizontalLayoutWidget_2->setObjectName(QString::fromUtf8("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setGeometry(QRect(20, 10, 631, 261));
        horizontalLayout_2 = new QHBoxLayout(horizontalLayoutWidget_2);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        lbLeftImage = new QLabel(horizontalLayoutWidget_2);
        lbLeftImage->setObjectName(QString::fromUtf8("lbLeftImage"));
        lbLeftImage->setFrameShape(QFrame::Panel);
        lbLeftImage->setFrameShadow(QFrame::Sunken);
        lbLeftImage->setLineWidth(3);
        lbLeftImage->setMidLineWidth(0);

        horizontalLayout_2->addWidget(lbLeftImage);

        lbRightImage = new QLabel(horizontalLayoutWidget_2);
        lbRightImage->setObjectName(QString::fromUtf8("lbRightImage"));
        lbRightImage->setFrameShape(QFrame::Panel);
        lbRightImage->setFrameShadow(QFrame::Sunken);
        lbRightImage->setLineWidth(3);

        horizontalLayout_2->addWidget(lbRightImage);

        verticalLayoutWidget = new QWidget(centralwidget);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(664, 10, 161, 751));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_4);

        btCameras = new QPushButton(verticalLayoutWidget);
        btCameras->setObjectName(QString::fromUtf8("btCameras"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::MinimumExpanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(btCameras->sizePolicy().hasHeightForWidth());
        btCameras->setSizePolicy(sizePolicy1);

        verticalLayout->addWidget(btCameras);

        verticalSpacer_6 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_6);

        btCapture = new QPushButton(verticalLayoutWidget);
        btCapture->setObjectName(QString::fromUtf8("btCapture"));
        sizePolicy1.setHeightForWidth(btCapture->sizePolicy().hasHeightForWidth());
        btCapture->setSizePolicy(sizePolicy1);
        btCapture->setMinimumSize(QSize(0, 0));
        btCapture->setMaximumSize(QSize(16777215, 16777215));
        btCapture->setFont(font);

        verticalLayout->addWidget(btCapture);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_3);

        btCalibrateCheckerBoard = new QPushButton(verticalLayoutWidget);
        btCalibrateCheckerBoard->setObjectName(QString::fromUtf8("btCalibrateCheckerBoard"));
        sizePolicy1.setHeightForWidth(btCalibrateCheckerBoard->sizePolicy().hasHeightForWidth());
        btCalibrateCheckerBoard->setSizePolicy(sizePolicy1);

        verticalLayout->addWidget(btCalibrateCheckerBoard);

        verticalSpacer = new QSpacerItem(20, 30, QSizePolicy::Minimum, QSizePolicy::Minimum);

        verticalLayout->addItem(verticalSpacer);

        btCalibrateSquareBar = new QPushButton(verticalLayoutWidget);
        btCalibrateSquareBar->setObjectName(QString::fromUtf8("btCalibrateSquareBar"));
        sizePolicy1.setHeightForWidth(btCalibrateSquareBar->sizePolicy().hasHeightForWidth());
        btCalibrateSquareBar->setSizePolicy(sizePolicy1);

        verticalLayout->addWidget(btCalibrateSquareBar);

        verticalSpacer_7 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_7);

        btFetchRailCrown = new QPushButton(verticalLayoutWidget);
        btFetchRailCrown->setObjectName(QString::fromUtf8("btFetchRailCrown"));
        sizePolicy1.setHeightForWidth(btFetchRailCrown->sizePolicy().hasHeightForWidth());
        btFetchRailCrown->setSizePolicy(sizePolicy1);

        verticalLayout->addWidget(btFetchRailCrown);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);

        btExit = new QPushButton(verticalLayoutWidget);
        btExit->setObjectName(QString::fromUtf8("btExit"));
        sizePolicy1.setHeightForWidth(btExit->sizePolicy().hasHeightForWidth());
        btExit->setSizePolicy(sizePolicy1);
        btExit->setFont(font);

        verticalLayout->addWidget(btExit);

        verticalSpacer_5 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_5);

        verticalLayout->setStretch(0, 4);
        verticalLayout->setStretch(1, 3);
        verticalLayout->setStretch(2, 4);
        verticalLayout->setStretch(3, 3);
        verticalLayout->setStretch(4, 4);
        verticalLayout->setStretch(5, 3);
        verticalLayout->setStretch(6, 4);
        verticalLayout->setStretch(7, 3);
        verticalLayout->setStretch(8, 4);
        verticalLayout->setStretch(9, 3);
        verticalLayout->setStretch(10, 4);
        verticalLayout->setStretch(11, 3);
        verticalLayout->setStretch(12, 4);
        horizontalLayoutWidget_3 = new QWidget(centralwidget);
        horizontalLayoutWidget_3->setObjectName(QString::fromUtf8("horizontalLayoutWidget_3"));
        horizontalLayoutWidget_3->setGeometry(QRect(20, 280, 631, 261));
        horizontalLayout_3 = new QHBoxLayout(horizontalLayoutWidget_3);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        lbLeftImage_2 = new QLabel(horizontalLayoutWidget_3);
        lbLeftImage_2->setObjectName(QString::fromUtf8("lbLeftImage_2"));
        lbLeftImage_2->setFrameShape(QFrame::Panel);
        lbLeftImage_2->setFrameShadow(QFrame::Sunken);
        lbLeftImage_2->setLineWidth(3);

        horizontalLayout_3->addWidget(lbLeftImage_2);

        lbRightImage_2 = new QLabel(horizontalLayoutWidget_3);
        lbRightImage_2->setObjectName(QString::fromUtf8("lbRightImage_2"));
        lbRightImage_2->setFrameShape(QFrame::Panel);
        lbRightImage_2->setFrameShadow(QFrame::Sunken);
        lbRightImage_2->setLineWidth(3);

        horizontalLayout_3->addWidget(lbRightImage_2);

        lbFused3DImage = new QLabel(centralwidget);
        lbFused3DImage->setObjectName(QString::fromUtf8("lbFused3DImage"));
        lbFused3DImage->setGeometry(QRect(20, 560, 631, 201));
        lbFused3DImage->setFrameShape(QFrame::Panel);
        lbFused3DImage->setFrameShadow(QFrame::Sunken);
        lbFused3DImage->setLineWidth(3);
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 840, 24));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MetGauge Box-GUI for Straightness Gauge", nullptr));
        lbLeftImage->setText(QCoreApplication::translate("MainWindow", "L", nullptr));
        lbRightImage->setText(QCoreApplication::translate("MainWindow", "R", nullptr));
        btCameras->setText(QCoreApplication::translate("MainWindow", "Parameters", nullptr));
#if QT_CONFIG(tooltip)
        btCapture->setToolTip(QCoreApplication::translate("MainWindow", "Capture photos and image-process them.", nullptr));
#endif // QT_CONFIG(tooltip)
        btCapture->setText(QCoreApplication::translate("MainWindow", "Capture", nullptr));
        btCalibrateCheckerBoard->setText(QCoreApplication::translate("MainWindow", "Calibrate\n"
"Checkerboard", nullptr));
        btCalibrateSquareBar->setText(QCoreApplication::translate("MainWindow", "Calibrate\n"
"Straight Edge", nullptr));
        btFetchRailCrown->setText(QCoreApplication::translate("MainWindow", "Fetch V && H\n"
"of Rail Head", nullptr));
        btExit->setText(QCoreApplication::translate("MainWindow", "Exit", nullptr));
        lbLeftImage_2->setText(QCoreApplication::translate("MainWindow", "L - midline", nullptr));
        lbRightImage_2->setText(QCoreApplication::translate("MainWindow", "R - midline", nullptr));
        lbFused3DImage->setText(QCoreApplication::translate("MainWindow", "Fused 3D profile \n"
"on the laser plane", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
