#ifndef CAMERAS_H
#define CAMERAS_H

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <QDebug>
#include <QThread>
#include <QMessageBox>
#include <QElapsedTimer>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/calib3d/calib3d.hpp"


#include "pylon/PylonIncludes.h"
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif
#include <pylon/BaslerUniversalInstantCamera.h>

#include "CameraApi.h"  //MV

using namespace std;
using namespace Pylon;
using namespace cv;

#ifdef OS_LINUX
    //220914 const String DEBUG_PATH = "/home/user/Code/SG/SG_Box/execute/debug/";
#else
    //220914 const String DEBUG_PATH = "C:/Users/jeffrey/RunStraightServer/debug/";
#endif
    const String CAPTURE_SUBDIR = "captured";


enum EOperateMode {     OM_IDLE, // FILE_SAVING
                                                OM_CALIBRATE_CHECKERBOARD,
                                                OM_CALIBRATE_SQUARE_BAR,
                                                OM_FETCH_RAIL_CROWN
                 };

enum ReturnCode {   RC_OK,
                                        RC_WRONG_MODE,
                                        RC_INVALID_DATA,
                                        RC_FAIL_TO_SAVE,
                                        RC_FAIL_TO_LOAD,
                                        RC_FAIL_TO_READ,
                                        RC_FAIL_TO_WRITE,  //6
                                        RC_FAIL_TO_FIND,
                                        RC_FAIL_TO_MAP,
                                        RC_FAIL_TO_OPEN,
                                        RC_FAIL_TO_REGISTER,  //10
                                        RC_FAIL_TO_ALLOCATE,
                                        RC_LACK_CAMERAS,
                                        RC_NO_IMAGE,   //13
                                        RC_LOCK_GUARDED,
                                        RC_BAD_IMAGE,   //15
                                        RC_IS_BUSY,   //
                                        RC_ABORTED,   //
                                        RC_NO_TRIGGER,   //time out
                                        RC_IS_CLOSING,   //
                                        RC_Exception
                                };

enum MVTriggerMode {      MTM_CONTINUOUS,
                                                    MTM_SOFTWARE_TRIGGER,
                                                   MTM_HARDWARE_TRIGGER
                                            };

struct SKBGrids {
    uint horzBlackGrdNum;  //black-to-black
    uint vertBlackGrdNum;  //black-to-black
    double mmGridArrayWidth;    //left of the left black to right of the right
    double mmGridArrayHeight;    //top of the top black to bottom of the bottom

    uint horzTotalGrdNum;  //black-to-black
    uint vertTotalGrdNum;  //black-to-black
    double mmOneGridWidth;    //left of the left black to right of the right
    double mmOneGridHeight;    //top of the top black to bottom of the bottom
};

namespace nsUtility
{
        constexpr uint ns2ms = 1000000;
        constexpr uint ns2us100 = 100000;
        int SaveMatBinary(Mat& mat, String filename);
        int LoadMatBinary(String filename, Mat& mat);
        //bool DoesDirExist(String dir);
        bool ProjectPointToAxisX(const Point3d&, const Point3d&, const Point3d&, const Point3d&, Point2d&);
        int GetCoordinatesRange(const vector<Point2d>& , Range&, Range&, const uint = 1);
        double AngleBtnVectors(const Vec3d&, const Vec3d& );
        String GetFileStamp(String path, uint count, uint nthCamera);
        String GetDirStamp(String path);
        void DumpHexBytes(const char *data, const int size);
        int TrimStereoMap(  Mat &, Mat &, const Rect &, Rect &, Mat &);
        int ReverseRoundMap(    Mat &orgImage, // const Size mappedImageSize,
                                                            Mat &reverseMapTableX, Mat &reverseMapTableY,
                                                            Rect *pReverseMapTOI, Mat *pReverseMapMask,
                                                            Mat &mappedImage );
        bool RotateBaroundA(cv::Point3d &A, cv::Point3d &B,  double angleDegree);
}

namespace nsCameras
{
    constexpr double LENS_FOCUS_mm = 25.0; //220812 16.0;
    constexpr uint MAX_EXPOSURE_ms = 3200; //MV 1600(Basler) * 2; //1.6 sec
    const uint32_t C_countOfImagesToGrab = 2;
    constexpr int CAPTURED_OFFSET_TLRN_HWT = 100; //20, 10 test dropping pair 5;// * nsUtility::ns2ms;  //5ms
    constexpr int CAPTURED_OFFSET_TLRN_SWT = 100; //20, 10 test dropping pair 5;// * nsUtility::ns2ms;  //5ms
    constexpr uint IMAGE_QUEUE_SIZE = 32; //256; //256; //256-4G /   512-6.9G  //IS111         2; //RAM limits to 800?! 32768; //25 * 100; //1000/40=25 1200/1.2=100
    // It is important to manage the available bandwidth when grabbing with multiple cameras.
    // This applies, for instance, if two GigE cameras are connected to the same network adapter via a switch.
    // To manage the bandwidth, the GevSCPD interpacket delay parameter and the GevSCFTD transmission delay
    // parameter can be set for each GigE camera device.
    // The "Controlling Packet Transmission Timing with the Interpacket and Frame Transmission Delays on Basler GigE Vision Cameras"
    // Application Notes (AW000649xx000) provide more information about this topic.
    // The bandwidth used by a FireWire camera device can be limited by adjusting the packet size.
#ifndef SMLT_HWT
    #define HWT_INTERVAL (50)
#endif

#if defined(_MISSION_BS)
    const uint C_WorkBoxes = 5;
    const uint C_WBperPC = 1;
#elif defined(_MISSION_XS)
    const uint C_WorkBoxes = 12;  //C_GalvanometerSet
    const uint C_WBperPC = 2;
#endif
    const uint C_WorkPCs = C_WorkBoxes / C_WBperPC;
    const uint C_TotalBoxes = C_WorkBoxes;

#ifdef SG_BOX_RD
    const uint C_TotalPCs = C_WorkPCs + 1; //C_WorkBoxes;
    //const uint C_TotalBoxes = C_WorkPCs + 1; //C_WorkBoxes + 1;
    const uint C_ParallelProcessPhotos = 8; //4 cores  M#   //IS111
#else
    const uint C_TotalPCs = C_WorkPCs; //C_WorkBoxes;
    //const uint C_TotalBoxes = C_WorkPCs; //C_WorkBoxes;
    const uint C_ParallelProcessPhotos = 2; //8; //4 cores  M#   //IS111
#endif
    const uint HEARTBEAT_SIZE = 6; //9 for port 1006
    const uint C_MaxCamerasToUse = 2;
    constexpr uint THREAD_0 = 0; //, THREAD_1 = 1, THREAD_2 = 2, THREAD_3 = 3;  //IS111
    const bool WILL_PROCESS = true;
    const bool CAPTURE_ONLY = false;
    const bool WILL_DISPLAY = true;
    const bool NO_DISPLAY = false;
    const int Camera_Unknown = -1;
#ifdef ROI_IMAGE   //RAW_LASER_ROI_WIDTH
    const Rect CAMERA_PHOTO_RECT = {0, 0, 2447, 2047 };  //for sDummyFullImage

    const uint ACCOMMODATE_185mm = 80;
    const uint ACCOMMODATE_BOTH_185_and_137 = 100; //ACCOMMODATE_185mm * 2;
    constexpr uint RAW_LASER_ROI_WIDTH = 1900; //1600;
    constexpr uint RAW_LASER_ROI_HEIGHT = 800; //600 + ACCOMMODATE_BOTH_185_and_137;
#ifdef _CALIB_AT_NEIHU
//    constexpr uint RAW_LASER_ROI_HEIGHT = 900;
#else
//    constexpr uint RAW_LASER_ROI_HEIGHT = 600;
#endif
    const Point RAW_LASER_ROI[C_TotalBoxes][2] = {  //M#  combined with RAW_LASER_ROI_WIDTH, RAW_LASER_ROI_HEIGHT
                                 { {500, 750},
                                   {0, 600} }, //{ {250, 550 - ACCOMMODATE_185mm },  {500, 500 - ACCOMMODATE_185mm } }, //Box1  { {950, 900},  {0, 700} }, //Box1

                                 {{200, 900},  //230407:X200
                                  {0, 850} },  //230407:X0  { {230, 1000 - ACCOMMODATE_185mm },  {300, 1000 - ACCOMMODATE_185mm } },  //Box2

                                 { {500, 900},  //230407:X500  230327:X300  200,
                                   {100, 0} },  // 230407:X100  230327:X200  230323L:0->200 X:540->300 !2448-1900=548 { {200, 1100 - ACCOMMODATE_185mm },  {800, 150 - ACCOMMODATE_185mm } },  //Box3

                                 { {300, 950},  //230407:X300  230327:X200, 230205:X50 0, 950
                                   {0, 0} //230407:X0  230327:X150, 230205:X300 {300, 100}
                                                    }, //{ {0, 1000 - ACCOMMODATE_185mm },  {700, 200 - ACCOMMODATE_185mm } }, //Box4
                                     #ifdef _CALIB_AT_NEIHU
                                                    { {650, 1050}, {400, 600} }
                                     #else
                                 { {0, 750},  {250, 450} }, //{ {0, 800 - ACCOMMODATE_185mm },   //800 too low for 185mm
                                                      // {500, 550 - ACCOMMODATE_185mm } }
                                     #endif
                                                    }; //B4: 3600  5200;
    /*Rect LASER_ROI_RIGHT[5] = {
                                                        {0, 0, 1000, 800 }, //Box1
                                                        {0, 0, 0, 0 }, //Box2
                                                        {0, 0, 50, 0 },
                                                        {700, 200, 2447-700, 600 }, //BOX4
                                                        {0, 0, 3700, 0 }
                                                       };  //= 0; //B4: 01400;   */
#endif


    enum CameraBrands { uk1 = -1, Camera_Basler = 0, Camera_MV,  Camera_Mars };
    enum CameraEnum { uk2 = -1, Camera_Left_0 = 0, Camera_Right_1 }; //Camera_Unknown = -1,
    enum XYEnum { X_0 = 0, Y_1 };
    enum ETriggerMode { TriggerMode_Software, TriggerMode_Hardware, TriggerMode_Files };
    struct sCallbackBuf {
            BYTE *alignedBuf;  //mv only?!
            uint frameCols, frameRows, frameSize;
            bool uncopied;
    };
    struct TQueuePhoto {
            Mat photoImage; //from q or mem
            ulong timeStamp;
    };
    struct TProcessPhoto {
            Mat *pPhotoImage; //from q or mem
            ulong *pTimeStamp;
    };
    struct sCallbackContext {
            //camera
            int cameraBrand;
            double cameraExposures[C_MaxCamerasToUse] = {50, 50} ;
            //array<pair<double, double>, sizeof(EOperateMode)> *cameraExposures;
            //system
            uint Box_ID;
            EOperateMode OP_Mode;
            bool TibboOn = false;
            //Q
            TQueuePhoto *pPhotoQueues; //[IMAGE_QUEUE_SIZE][C_MaxCamerasToUse];
            uint *pPhotoQHead;
            uint *pPhotoQTail;
            mutex *pPhotoQMtx;

            //rail
            QElapsedTimer *pRailTimer;
            ulong *pTriggerStart;
            int *pLastRailGap;

            //MV
            sCallbackBuf  mvCallbackBuf[C_MaxCamerasToUse];
    };
#ifdef CAMERAS_CPP
    CameraBrands CameraBrand;
    //CInstantCameraArray cameras[C_MaxCamerasToUse];
    CBaslerUniversalInstantCamera  cameras[C_MaxCamerasToUse];
    IPylonDevice *pylonDevices[C_MaxCamerasToUse] = {nullptr, nullptr};

    tSdkCameraDevInfo mvCameraList[C_MaxCamerasToUse];
    CameraHandle mvCamerahandles[C_MaxCamerasToUse];
    tSdkCameraCapbility mvCameraCaps[C_MaxCamerasToUse]; // camera characterization
    // sCallbackContext mvCallbackContext;
    // sCallbackBuf  mvCallbackBuf[C_MaxCamerasToUse];
    // bool TibboOn = false;
    std::mutex mtxCallback;
    String DEBUG_PATH;
    String CAPTURE_PATH;
    //bool IsGUIAP;
#ifdef ROI_IMAGE
    Rect CUR_PHOTO_RECT;// = {0, 0, 2447, 2047 };
    uint CUR_LASER_ROI_WIDTH;
    uint CUR_LASER_ROI_HEIGHT;
    Point CUR_LASER_ROI[5][2];
#endif
    int ROI_SE_LEFT_X[C_TotalBoxes] = {
                                                        6210, //23Jan 6300, // budged  0.41  0.427: 6100, //Box1  16-13.14   M#
                                                        4000, //23Jan 3600, //
                                                        4350, //230323: 4350  4300, //23Jan 4300, //0920   0915:4750, //Box3  16.34-15.43
                                                        6050, //230205 4900, //23Jan 5000, //Box4   15.91, 12.2
                                               #ifdef _CALIB_AT_NEIHU
                                                        4300 xx
                                               #else
                                                        6750  //230205:6750   6735, //23Jan  6730 //0920   //4500 6250, //
                                               #endif
                                                   };
    /*RectifyRatioX == 3.1*/
    //RectifyRatioX==5
    int ROI_SE_RIGHT_X[C_TotalBoxes] = {
                                                            2690, //23Jan 2400, // budged  2700(Mark fixed)
                                                            530,  //23Jan 570,  //Box2 350, //
                                                            500, //230407:800!!  230320:500  850, // 700, //750, //23Jan  770, // 100, //850, //1200, //Box3
                                                            2450, //230205 1450, //23Jan  1400,  //Box4
                                                     #ifdef _CALIB_AT_NEIHU
                                                            1000 xx
                                                     #else
                                                             3000 //230205:3000 3200, //23Jan  3200  //0920  //500 2750 //
                                                     #endif
                                                    };
    /*RectifyRatioX == 3.1
*/
    //RectifyRatioX==5
    int ROI_SE_Y_TOP[C_TotalBoxes] = {
                                                        730, //23Jan 400, // budged  810, //0.41  0.427: 800, //Box1
                                                        700, //23Jan 510, //470, //Box2
                                                        700, //23Jan  650, // 225, //250, //0920 0915:200,  //Box3
                                                        620, //230205 700, //23Jan  220,  //Box4
                                                   #ifdef _CALIB_AT_NEIHU
                                                            200 xx
                                                   #else
                                                            200 //230205:200 510, //23Jan  250 //0920  //500
                                                   #endif
                                                };
    /*RectifyRatioX == 3.1
    int ROI_Y_TOP[C_TotalBoxes][C_ParallelProcessPhotos] = {
                                                    {0, 0, 820, 600 }, //Box1
                                                    {0, 0, 500, 250 }, //Box2
                                                    {0, 0, 220, 0 }, //Box3
                                                    {0, 0, 240, 0 }, //BOX4
                                                    {0, 0, 3700, 0 }
                                                   }; */
    int ROI_SE_WIDTH =  300;
    //int ROI_WIDTH[4] = { 0, 0, 300, 600 };
#ifdef _CALIB_AT_NEIHU
    int ROI_SE_HEIGHT = 1000;  //white CB
#else
    int ROI_SE_HEIGHT = 450; //23Jan Box-3  400;
#endif
    //int ROI_HEIGHT[4] = { 0, 0, 400, 1150 }; //= 1150;
#else
    extern CameraBrands CameraBrand;
    extern CBaslerUniversalInstantCamera  cameras[C_MaxCamerasToUse];
    extern tSdkCameraDevInfo mvCameraList[C_MaxCamerasToUse];
    extern CameraHandle mvCamerahandles[C_MaxCamerasToUse];
    extern tSdkCameraCapbility mvCameraCaps[C_MaxCamerasToUse];
    //extern sCallbackContext mvCallbackContext;
            //extern sCallbackBuf  mvCallbackBuf[C_MaxCamerasToUse];
            //extern bool TibboOn;
    extern std::mutex mtxCallback;
    extern String DEBUG_PATH;
    extern String CAPTURE_PATH;
#ifdef ROI_IMAGE   //CUR_PHOTO_RECT
    extern Rect CUR_PHOTO_RECT;
    extern uint CUR_LASER_ROI_WIDTH;
    extern uint CUR_LASER_ROI_HEIGHT;
    extern Point CUR_LASER_ROI[C_TotalBoxes][2];
#endif
    extern int ROI_SE_LEFT_X[C_TotalBoxes];
    extern int ROI_SE_RIGHT_X[C_TotalBoxes];
    extern int ROI_SE_Y_TOP[C_TotalBoxes];
    extern int ROI_SE_WIDTH ;
    extern int ROI_SE_HEIGHT;
#endif


    static int initializeCallbackBuffer(sCallbackBuf *, int, int);
    int InitializeCameras(void *); //Mat *, PVOID);
    int FinalizeCameras(void *);
    int StartCapturing(int nthDevice = Camera_Unknown);
    int GrabOneImage(int nthDevice, Mat &OCvImage, mutex& mtxPhoto, bool HWTriggered, void *callbackContext);
    int GrabOnePairImage(TProcessPhoto *, mutex& mtxPhoto, bool HWTriggered, sCallbackContext *callbackContext, bool CaptureLastInQ=false);
    int TuneCameraExposure(int nthDevice, double usNewExp, double* usResultExp = nullptr);
    int GetCameraExposure(int nthDevice, double& usNewExp);
    int SetCameraTriggerMode(int nthDevice, const ETriggerMode tm);
    void mvGrabCallback0(CameraHandle hCamera, BYTE *pFrameBuffer, tSdkFrameHead* pFrameHead,PVOID pContext); //_stdcall
    void mvGrabCallback1(CameraHandle hCamera, BYTE *pFrameBuffer, tSdkFrameHead* pFrameHead,PVOID pContext); //_stdcall
    bool BaslerGrabOne(uint theDevice, CGrabResultPtr pGrabbedResult);

    inline string GetXYChar(int XY)  {  return (XY == 0 ? "X" : "Y"); }
    string GetCameraChar(int nthCamera);
    string GetCameraStr(int nthCamera);

    void mvGrabCallback(int theDevice, CameraHandle hCamera, BYTE *pFrameBuffer, tSdkFrameHead* pFrameHead,PVOID pContext);
    int OnPhotosCaptured(
            int theDevice, int cameraHandle,
            void *sdkCallbackBuffer, sCallbackContext *callbackContext, void *photoBufferHeader = nullptr);

} // namespace nsCameras

namespace nsImageProcess {
    /*constexpr uint CB_INT_CNR_ROW = 4;
    constexpr uint CB_INT_CNR_COL = 6;
    const Size CB_INT_CNRS (CB_INT_CNR_COL, CB_INT_CNR_ROW); //patternSize =
    cv::Size(points_per_row,points_per_colum) = cv::Size(columns,rows) ).
    */
    constexpr int SE_VH_OFFSET_X = 100;
    constexpr int SE_VH_OFFSET_Y = 200; //230205 for Box-4 200;
    constexpr int SE_VH_OFFSET_WIDTH = 300;
    constexpr int SE_VH_OFFSET_HEIGHT = 750; //230205 750;

    constexpr int RECTIFY_MAP_TYPE_ID = CV_32FC1;
    typedef  float RECTIFY_MAP_TYPE;
    //.elemSize() constexpr uint RECTIFY_MAP_ELMT_SIZE = 32/8;
    constexpr double RectifyAlpha = -1;   // M#
    constexpr double RectifyRatioX = 5.0; // Box2 3.1; //1.5;  //M# Box-2?!
    constexpr double RectifyRatioY = 0.7; // 0.6;  //M# 0.7 better
    int FindMidline(Mat &OCvImage, const uint nthBox, int nthDevice = nsCameras::Camera_Unknown,
                    EOperateMode = OM_FETCH_RAIL_CROWN, Mat *extWork = nullptr);
    int CalibrateOneCamera(const SKBGrids &, const uint, const string,
                           const uint, const bool, Size &,
                           vector<vector<cv::Point3f>> &,
                           vector<vector<cv::Point2f>> &, Mat &, Mat &, Rect &,
                           double &);
    void GeneratePhysicalCorners(vector<Point3f> &, const SKBGrids &);
    int GetStereo3DFromRectifiedPair(const Mat &, Rect *, const Mat &, Rect *, int,
                                     int, const double, const Point2d,
                                     vector<Point3d> &, Point &, Point &);
    int DetermineXAxisOfCrossSquareBar(const vector<Point3d> &, Point3d &,
                                       Point3d &, Point3d &);
    int MapRail3DCoordsTo2DCS(vector<Point3d> &, const Point3d &,
                              const Point3d &, const Point3d &,
                              vector<Point2d> &);
    int GetRailHeadVH(const uint nthBox, const EOperateMode, const vector<Point2d> &, double VHVerticalOffset,
                      Point2d &V, Point2d &H);
    Rect GetRectifiedROI(const uint , const uint , const EOperateMode );
    int SimpleBinaryThrshold(Mat &image, const uint threadVal, const bool keepHigh = true);
} // namespace nsImageProcess

namespace nsGalvanometer {
    int SendAnglesToGalvanometer();

}
#endif // CAMERAS_H
