#include "mainwindow.h"

//-nogui -MODE=VH
//C:\Users\user\Documents\ComputerVision\Working_Code\build-StraightServer-Desktop_Qt_5_14_2_MSVC2017_64bit-Debug

//QT_PLUGIN_PATH   C:\Users\jeffrey\RunStraightServer\Qt5.14 VC17_64 Plugins

/*
 *  download the final package
 *  install anydesk , make it always open
 *  install vnc & video-key, but not re-instate the configuration
 *  copy documents to mobile
 *
  *  set unsatisfied thresholds for calibrations?
  *  seg. fault?
  *  calibration
  *         - remember to clean the rest files
  */


#include <QApplication>
#include <QCommandLineParser>
#include <QCommandLineOption>

#include "cameras.h"

using namespace Pylon;
using namespace std;
using namespace cv;
using namespace nsCameras;

void drawText(Mat & image);

QCoreApplication* createApplication(int argc, char *argv[])
{
        for (int i = 1; i < argc; ++i)
                if (!qstrcmp(argv[i], "--no_gui"))
                        return new QCoreApplication(argc, argv);

        return new QApplication(argc, argv);
}
#include <iostream>
#include <cstring>

int main(int argc, char *argv[])
{
        cout << "NextSense - MetGauge - StraightGauge - Box-AP \n\n" ;
#if defined(_MISSION_BS)
           cout << "This is a version dedicated to British Steel at Scunthrope.\n\n" ;
           cout << "Sensor Boxes need to cooperate with the Box-Driver(MBD) on Main-PC(MPC). \n\n" ;
#elif defined(_MISSION_XS)
           cout << "This is a version dedicated to XiangTan Iron&Steel in HuNan.\n\n" ;
           cout << "Sensor Boxes need to cooperate with the Main-AP on Main-PC(MPC)\n\n" ;
#endif

#ifdef SG_BOX_RD
           cout << "WATCH: This is a special version for RD Box-PC running at the Taipei Office.\n\n" ;
#endif
#ifndef _DEBUG
    #ifdef _DEBUG_FILE
            cout << "Watch: _DEBUG_FILE flag stays ON in Release Mode!\n\n";
    #endif
#endif

    int execCode;

    try {
            QApplication a(argc, argv);

           MainWindow fmMain;
           cout << "The system is running in SG-Box-" << fmMain.BOX_ID << "...\n\n" ;
           fmMain.setWindowTitle( fmMain.BasicMainWndTitle + QString::fromStdString(to_string(fmMain.BOX_ID)) );

           if (argc > 1)
           {
                    QString arg1=*(argv + 1);
                    arg1 = arg1.toUpper();
                    if (arg1.contains("NOGUI"))
                    {
                            fmMain.NO_GUI_MODE = true, cout << "Learned GUI NOT operatable. " ;

                            EOperateMode opMode = OM_FETCH_RAIL_CROWN;  //??
                            fmMain.mvCallbackContext.OP_Mode = opMode;
                            if (argc == 2)
                                    cout << "No mode specified, defaulted to VH Mode.\n";
                            else {
                                if (!QDir(QString::fromStdString(fmMain.WorkDirBox1_5)).exists())
                                {
                                        cout << "\n\nThe calibrated dir " <<  fmMain.CalibrateDir << "does not exist, will abort. \n\n" ;
                                        return -7;
                                }
                                    QString arg2 = *(argv + 2);
                                    arg2 = arg2.toUpper();
                                    if (arg2.contains("MODE="))
                                            if (arg2.contains("VH"))
                                            {
                                            } else if (arg2.contains("SV"))
                                            {
                                                   cout  << "Enter File-Saving Mode.\n";
                                            }
                            }
                            switch (opMode)
                            {
                            case OM_FETCH_RAIL_CROWN:
                                                    cout  << "Entered V-H Packet Mode.\n\n";
                                                    fmMain.on_btFetchRailCrown_clicked();
                                                    fmMain.fmExposures.ui->kbSavePhotos->setCheckState(Qt::Unchecked);
                                             #ifdef SG_BOX_RD
                                                     fmMain.fmExposures.ui->kbHWTrigger->setCheckState(Qt::Unchecked);
                                                     //fmMain.SimulateTibboOn();
                                             #else
                                                    fmMain.fmExposures.ui->kbLoadPhotos->setCheckState(Qt::Unchecked);
                                                    fmMain.fmExposures.ui->kbHWTrigger->setCheckState(Qt::Checked);
                                             #endif
                                            break;
                            case OM_IDLE:
                                break;

                            }

                    }
           }

            QCommandLineParser parser;
            parser.setApplicationDescription("Test helper");
            parser.addHelpOption();
            parser.addVersionOption();
               QCommandLineOption targetDirectoryOption(QStringList() << "t" << "target-directory",
                       QCoreApplication::translate("main", "Copy all source files into <directory>."),
                       QCoreApplication::translate("main", "directory"));
               parser.addOption(targetDirectoryOption);
               //parser.process(a);

               fmMain.show();

                throw a.exec();
/*
            int rc;
            QScopedPointer<QCoreApplication> AP(createApplication(argc, argv));
            MainWindow fmMain;
            if ( qobject_cast<QApplication *>(AP.data()) )
            {
                    fmMain.show();
                    //nsUtility::IsGUIAP = true;

                    rc = AP->exec();
            } else {
            }
                    //? delete AP.data();
            throw  rc;
        /*
*           QScopedPointer<QCoreApplication> app(createApplication(argc, argv));
            if (qobject_cast<QApplication *>(app.data())) {
               // start GUI version...
                MainWindow fmMain;
                fmMain.show();
            } else {
               // start non-GUI version...
            }           // if (execCode = InitializeCameras()) throw execCode ;
            throw app.data()->exec();  //a.exec();
/**/

            //fmMain.fmExposures.SetInitializedGUIToVariables();
           // if (execCode = InitializeCameras()) throw execCode ;

 /*
             QApplication a(argc, argv);

            MainWindow fmMain;

                            //fmMain.fmExposures.SetInitializedGUIToVariables();

            fmMain.show();

            throw a.exec();
*/

            cout << "Built with OpenCV " << CV_VERSION << endl;
            Mat image;
            VideoCapture capture;
            capture.open(0);
            if (capture.isOpened())
            {
                    cout << "Capture is opened" << endl;
                    for(;;)
                    {
                        capture >> image;
                        if (image.empty())
                            break;
                        drawText(image);
                        imshow("Sample", image);
                        if (waitKey(10) >= 0)
                            break;
                }
            } else {
                    cout << "No capture" << endl;
                    image = Mat::ones(480, 640, CV_8UC1);  // ::zeros(
                    drawText(image);
                    imshow("Sample", image);
                    waitKey(0);
            }
    } catch (int thrown) {
            execCode = thrown;
    }


    cout << "The program is terminated by the user. \n\n";
    return execCode;
}

void drawText(Mat & image)
{
    putText(image, "Hello OpenCV",
            Point(20, 50),
            FONT_HERSHEY_COMPLEX, 1, // font face and scale
            Scalar(255, 255, 255), // white
            1, LINE_AA); // line thickness and type
}
