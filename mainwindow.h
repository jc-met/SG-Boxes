#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ui_mainwindow.h"
#include "cameras.h"
#include "ui_exposures.h"
#include "exposures.h"

#include <QMainWindow>
#include <QTcpSocket>
#include <QTcpServer>
#include <QFileInfo>

#include <mutex>
#include <thread>

#include "opencv2/opencv.hpp"
//using namespace cv;

//#include "opencv2/core/core.hpp"
//using namespace cv;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


/*enum OperateMode {   OM_IDLE,
                                            OM_CALIBRATE_CHECKERBOARD,
                                            OM_CALIBRATE_SQUARE_BAR,
                                            OM_FETCH_RAIL_CROWN
                 };
/**/
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    //Ui::MainWindow *ui;

    Exposures fmExposures;

    String WorkingDir;
    QElapsedTimer railTimer;
    //QElapsedTimer startStopTimer;
    ulong TriggerStart = 0; //railTimer.nsecsElapsed();
    int LastRailGap = -1;
    //int TriggerCount = -1;

    uint BOX_ID = 6; //1-based
    bool NO_GUI_MODE = false;
    ETriggerMode eTriggerMode;  //need a lv for threading

    //String  WorkDirBox6  = "/home/user/Code/SG/SG_Box/data/box" + to_string(BOX_ID) + "/KB/"; //2/KB/"; //
    const String  WorkDirBox1_5 = "/home/user/downloads/centralize/calib/";

    const QString BasicMainWndTitle ="MetGauge Box-AP for Straightness Gauge - Box ";
#ifdef _TEST_LOCALHOST  //_CALIB_AT_NEIHU
    const String ipTibbo = "127.0.0.1"; //"192.168.0.157";
    const String ipPanelPC = "192.168.0.000";
#else
    const String ipTibbo = "192.168.50.101";
    const String ipPanelPC = "192.168.50.19";  //16 changed to 19
#endif
    const uint portPanelPC = 1002;
    //constexpr uint MPCPortNumber[6] = { 10000, 10010, 10015, 10020, 10030, 10060 };
    //const uint portTibbo = 1003;
    //const uint TibboPortNumber[6] = { 10000, 10010, 10015, 10020, 10030, 1003 };
    const uint TibboPortNumber[ C_TotalPCs] = { 10001, 10002, 10003, 10004, 10005
                                              #ifdef _MISSION_XS
                                                  , 10006 //, 10007, 10008, 10009, 10010, 10011, 10012
                                              #endif
                                              #ifdef SG_BOX_RD
                                                       , 10001
                                              #endif
                                                     };
    //obsolete const uint MainAPPortNumber[6 /*C_TotalPCs*/] = { 10010, 10020, 10030, 10040, 10050, 10010 };
    const uint MainBDPortNumber[C_TotalPCs] = { 10100, 10200, 10300, 10400, 10500
                                                    #ifdef _MISSION_XS
                                                        , 10600  //, 10700, 10800, 10900, 11000, 11100, 11200
                                                    #endif
                                                    #ifdef SG_BOX_RD
                                                        , 10100
                                                    #endif
                                                      };
    String BOX_IP;   //redundant
    /*const QString Box_IPs[6] = { "192.168.50.21",
                                                                                  "192.168.50.22",
                                                                                  "192.168.50.23",
                                                                                  "192.168.50.24",
                                                                                  "192.168.50.25",
                                                                             #ifdef _TEST_LOCALHOST  //230208 _CALIB_AT_NEIHU
                                                                                    QHostAddress::LocalHost //230208 "127.0.0.1" //"192.168.0.157"
                                                                             #else
                                                                                 "192.168.50.60"
                                                                             #endif
                                                                                  }; */

    const uint EXP_CALIB_CHECKERBOARD_ms[C_MaxCamerasToUse] = {600, 550}; //ambient light 0707  385000; 0621
    const double EXP_CALIB_SQUARE_BAR_ms[C_MaxCamerasToUse] = {2.5, 2.0}; //2000, 1500}; //laser light 105000 black stone;  //100000 for square-bar 3500
    const uint EXP_MEASURE_RAIL_CROWN_ms[C_MaxCamerasToUse] = {20, 15}; // {2000, 1500}; //105000 black stone;  //100000 for square-bar 3500
    array<pair<double, double>, sizeof(EOperateMode)> CameraExposures = {
                make_pair(40, 40),
                make_pair(EXP_CALIB_CHECKERBOARD_ms[Camera_Left_0], EXP_CALIB_CHECKERBOARD_ms[Camera_Right_1]),
                make_pair(EXP_CALIB_SQUARE_BAR_ms[Camera_Left_0], EXP_CALIB_SQUARE_BAR_ms[Camera_Right_1]),
                make_pair(EXP_MEASURE_RAIL_CROWN_ms[Camera_Left_0], EXP_MEASURE_RAIL_CROWN_ms[Camera_Right_1])
            };
    array<ETriggerMode, sizeof(EOperateMode)> TriggerModes = {
                    TriggerMode_Software,
                    TriggerMode_Software,
        #ifdef _MISSION_BS
                    TriggerMode_Software, // TriggerMode_Hardware, worse for loading
        #endif
        #ifdef _MISSION_XS
                    TriggerMode_Hardware,
        #endif
                    TriggerMode_Hardware };
    array<bool, sizeof(EOperateMode)> SaveCaptured = {
                    true,
                    true,
                    true,
                    false
                };
    array<bool, sizeof(EOperateMode)> LoadCaptured = {
                    false,
                    false,
                    false,
                    false
        };
    //bool TibboOn = false;

    EOperateMode opMode;
    String CalibrateDir;
    sCallbackContext mvCallbackContext;  //shared across cameras?!
    //sCallbackContext bslCallbackContext;

    int LoadSystemParameters();
    int SaveSystemParameters();
    void SetManualCapture();
    void StartLiveShootTimer(uint msInterval);
    void StopLiveShootTimer();
    void StartHWLiveShootTimer(uint msInterval);
    void StopHWLiveShootTimer();
    void tibboStartReceived(uint);
    void tibboStopReceived(uint);
    void TuneRotatedPhotoDim();
    //std::mutex mtxPhoto;
    //cv::Mat sPhotoes[2];
    void SimulateTibboOn();
//#ifndef _BOX_PLAY_CLIENT
    bool SvrSktForMPCStartListening(const uint nthBox);
    bool SvrSktForMBDStartListening(const uint nthBox);
//#endif
    void CloseTibboSocket();
    void CloseMBDSocket();
    void VoidLoadedRctfMaps();
    inline bool IsAutoSEing()   {  return IsAutoSECalibrating || IsAutoSEMeasuring;     }

private slots:

    void on_btCapture_clicked();

    void on_btExit_clicked();

    //void on_btCalibrate_clicked();

    void on_btCameras_clicked();

    //void on_btFetchLine_clicked();

    void tmSWLiveCaptureHandler();
    void tmHWTLiveCaptureHandler();
    int HWTProcessHandler(uint THREAD_ID=0);
    void HWTSaveFileHandler();
    void HWTSECaptureHandler();
    void tmHWTProcessHandler0();
    void tmHWTProcessHandler1();
    void tmHWTProcessHandler2();
    void tmHWTProcessHandler3();
    void tmHWTProcessHandler4();
    void tmHWTProcessHandler5();
    void tmHWTProcessHandler6();
    void tmHWTProcessHandler7();
    void tmHouseKeepHandler();

    //void thdTCPHandshake(String IP, uint portNo);
    void tibboSktReadHandler();
//    void on_btRectify_clicked();
//    void on_pushButton_clicked();
//    void on_pushButton_2_clicked();
//    void MainWindow::displayCanvasLabel(QLabel &canvas, Mat &photo);

    void on_btCalibrateSquareBar_clicked();

    void on_btCalibrateCheckerBoard_clicked();

public slots:
        void on_btFetchRailCrown_clicked();
        void ReadMBDServerStr_CMD();

protected:
        void closeEvent (QCloseEvent *event);
        void keyPressEvent(QKeyEvent* keyEvent);

public slots:
    void onMPCServerConnecting(const uint nthBox);
    void onMPCServerConnecting0();
    void onMPCServerConnecting1();
    void onMPCServerConnecting2();
    void onMPCServerConnecting3();
    void onMPCServerConnecting4();

    void onConnectedToMBDServer_VH(const uint nthBox); //const uint nthBox);
    void onConnectedToMBDServer_CMD(const uint nthBox);
#ifndef _BOX_PLAY_CLIENT
    void onConnectedToMBDServer_VH0();
    void onConnectedToMBDServer_VH1();
    void onConnectedToMBDServer_VH2();
    void onConnectedToMBDServer_VH3();
    void onConnectedToMBDServer_VH4();
#endif


private:
        enum ETibboSignal {   Tibbo_None, Tibbo_Start, Tibbo_Stop, Tibbo_Calibrate_SE, Tibbo_Save_Round };
        const uint Tibbo_Packet_Size = 45;
        const uint Tibbo_Packet_Size_1 = Tibbo_Packet_Size - 1;
        //const BYTE StartSum = 0xAD; //QByteArray StartSum{"0xAD"};
        //const BYTE  StopSum = 0xAC; //QByteArray StopSum{"0xAC"}; //(1, static_cast<char>(0xAC));
        const QString CalibrateCheckerBoardCaption = "Calibrate\nCheckerboard";
        const QString CalibrateSquareBarCaption = "Calibrate\nStraight Edge"; // Square Bar";
        const QString FetchRailCrownCaption = "Fetch V && H\nof Rail Head";
        Ui::MainWindow *ui;
        uint PhotoPairStartNo;
        QLabel* pnTopPhoto[2];
        QLabel* pnMidPhoto[2];
        QLabel* pnFused3DImage[2];
        cv::Point3d GaugeAxisY;
        cv::Point3d GaugeAxisX[2];
        double AxisYSlant;  //10

        QTimer *tmLiveShootShowBoth;
        uint swLiveShootInterval;
        QTimer *tmHWTLiveShoot;
        uint hwLiveShootInterval;
        QTimer *tmHWTLiveProcess[C_ParallelProcessPhotos];
        QTimer *tmHWTLiveSave;
        QTimer *tmHWTSECapture;
        QTimer *tmHouseKeeper;
        //QTimer *tmHardwareTrigger;
        std::thread thTibbo;
        QTcpSocket *sktTibbo /*tibboSocket*/ = nullptr;
#ifdef _BOX_PLAY_CLIENT
        QTcpSocket *sktClientMBD_VH = nullptr;
        QTcpSocket *sktClientMBD_CMD = nullptr;
#else
        QTcpServer *serverForMBD[C_TotalPCs] = { nullptr , nullptr, nullptr, nullptr, nullptr };
        QTcpSocket *sktMBD[C_TotalPCs] = { nullptr , nullptr, nullptr, nullptr, nullptr };
        QTcpServer *serverForMPC[C_TotalPCs] = { nullptr , nullptr, nullptr, nullptr, nullptr };
        QTcpSocket *sktMPC[C_TotalPCs] = { nullptr , nullptr, nullptr, nullptr, nullptr };
#endif
        std::thread thHWTLiveShoot;
        bool isClosing = false;
        QByteArray TibboBuffer = "";
        bool IsAutoSECalibrating = false;
        bool IsAutoSEMeasuring = false;
        bool IsAbortingAutoSE=false;
        chrono::system_clock::time_point AutoSEStart;
        EOperateMode orgModeBeforeACSE;

        void GUIRestoreCalibrate();
        void showCrop(int, int = 0, int = 0);
        bool loadRectifiedMaps();
        bool loadSECResults();
        bool saveSECResults();
        int CaptureProcessDual(ETriggerMode, bool, bool, const uint = THREAD_0, const bool = false);
        void ConcludeAutoSE(const uint rc);
        int StartAutoSE(bool &flag);
        void OnCalibrateDirChanged();
        bool GetCalibrateDir();
        int OnBothFilesLoaded();
        int determinePacket(const uint sumIdx);
        void removeHeartbeats();
        bool TryConnectMBD();
        void setCaptureButtonCaption();

};
#endif // MAINWINDOW_H
