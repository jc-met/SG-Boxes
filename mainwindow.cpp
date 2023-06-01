#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QCloseEvent>
#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include <QProcess>
#include <QTimer>

#include<thread>
//#include <pylon/PylonIncludes.h>
/*
 * 3,2,3,2,3,2,3,2,3
 * 4,3,4,3,4,3,4,3,4
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif

// Namespace for using pylon objects.
using namespace Pylon;

*/

#include "cameras.h"

using namespace std;
using namespace Pylon;
using namespace chrono;
using namespace cv;

using namespace nsCameras;
using namespace nsImageProcess;
using namespace nsUtility;

//typedef void (*FSockerHandler)(void);
//MainWindow *a;

static Mat sPhotoMem[C_ParallelProcessPhotos] [ C_MaxCamerasToUse ];
static ulong sTimeStamp[C_ParallelProcessPhotos][ C_MaxCamerasToUse ] ; //IS111
  /*=
                                                            { {10000, 10001},
                                                              {20000, 20001},
                                                              {30000, 30001},
                                                              {40000, 40001}
                                                            } ; */
static TProcessPhoto sPhotoes[C_ParallelProcessPhotos] [ C_MaxCamerasToUse ] = { {nullptr, nullptr }, {nullptr, nullptr }} ;
                        //={ {&sPhotoMem[0], &sTimeStamp[0]}, {&sPhotoMem[1], &sTimeStamp[1]} };
//static Mat sPhotoes[ C_MaxCamerasToUse ];
//static ulong sTimeStamps[ C_MaxCamerasToUse ];
static int TriggerCount;  //TriggerCount++ observe idx sPhotoes
static mutex mtxPhoto;

static TQueuePhoto sPhotoQueues[IMAGE_QUEUE_SIZE][C_MaxCamerasToUse];
//static Mat sPhotoQueues[IMAGE_QUEUE_SIZE][C_MaxCamerasToUse];
//static ulong sPhotoTimestamps[IMAGE_QUEUE_SIZE][C_MaxCamerasToUse];
static uint sPhotoQHead;
static uint sPhotoQTail;
static mutex mtxPhotoQ;
static condition_variable cndtVrbl;
//static bool TibboOn = false;

static Mat sStereoMap[C_MaxCamerasToUse][2];
static Mat sProjectionMatrices_P[C_MaxCamerasToUse];;  //3x4
static Mat sRctfMidlines[C_ParallelProcessPhotos][C_MaxCamerasToUse];
static Point sMidlineRectStart[C_MaxCamerasToUse];
static Rect sReverseMapTOI[C_MaxCamerasToUse];
static Mat sReverseMapMask[C_MaxCamerasToUse];

#ifdef ROI_IMAGE
static Mat sDummyFullImage[C_ParallelProcessPhotos][C_MaxCamerasToUse];
#endif
static bool rectifyMapLoaded = false;
//#define _AUTO_CS_XY
#ifndef _AUTO_CS_XY
static Mat sWorkCS_XY[C_ParallelProcessPhotos];
#endif
static Mat sWorkRctfML[C_ParallelProcessPhotos][C_MaxCamerasToUse];

void thdCapture(int theDevice, bool HWTriggered, void *callbackContext, const uint THREAD_ID)
{
        BOOL opened;
        switch(CameraBrand)
        {
        case Camera_Basler:     opened = cameras[theDevice].IsOpen();      break;
        case Camera_MV:         //thdCapture
                                            //CameraIsOpened(&mvCameraList[theDevice], &opened) ;
                                            #ifdef OS_LINUX
                                                opened = true;
                                            #endif
                                            break;
        }

        int rc;
        if (opened)  //
        {
                sPhotoes[THREAD_ID][theDevice].pPhotoImage = &sPhotoMem[THREAD_ID][theDevice];
                sPhotoes[THREAD_ID][theDevice].pTimeStamp = &sTimeStamp[THREAD_ID][theDevice];
                GrabOneImage(theDevice, *sPhotoes[THREAD_ID][theDevice].pPhotoImage , //thdCapture   sPhotoes[theDevice],
                                                mtxPhoto, HWTriggered, callbackContext);  // thdCapture //thdCapture
        }
                /*for(;;) {
                        bool leave = true;
XX                        if ((rc = GrabOneImage(theDevice, sPhotoes[theDevice], mtxPhoto, HWTriggered)) != RC_OK)
                                if (HWTriggered)
XX                                        if (TibboOn)
                                                leave = false;
                        if (leave) break;
                }*/
}

using namespace nsCameras;
void thdFetchLine(int nthDevice, EOperateMode opMode, int BoxID,  uint THREAD_ID)
{
        if (!rectifyMapLoaded)  return;
        duration<double, milli> d(0);
        auto t1 = high_resolution_clock::now();

#ifdef _DEBUG_FILE
        //qDebug()  << "DMBT: "  << nthDevice << ", "  << opMode << ", "  << BoxID << ", "  << THREAD_ID;
        imwrite(DEBUG_PATH + "$input_" + to_string(nthDevice) +".bmp",  *sPhotoes[THREAD_ID][nthDevice].pPhotoImage); //sPhotoes[nthDevice]); //*photo );
#endif

#ifdef FIND_LINE_FIRST
        FindMidline( *sPhotoes[THREAD_ID][nthDevice].pPhotoImage, //sPhotoes[nthDevice],
                                    nthDevice, opMode);
                        //imwrite("C:/Users/jeffrey/debug/$FM 7_" + to_string(nthDevice) +".bmp",  sPhotoes[nthDevice]); //*photo );
        #ifdef _DEBUG
                d = high_resolution_clock::now() - t1;
                cout << "FT " << to_string(nthDevice) << " FML: " << d.count() << endl;
        #endif
#endif

        Mat *pFullImage;
#ifdef ROI_IMAGE   //thdFetchLine
            Mat *pPhotoImage = sPhotoes[THREAD_ID][nthDevice].pPhotoImage;
            Rect topRoi( Point(0, 0),  pPhotoImage->size() );
            sDummyFullImage[THREAD_ID][nthDevice].setTo(Scalar::all(0));  //IS556 230319
            Rect bottomRoi( CUR_LASER_ROI[BoxID - 1][nthDevice],  pPhotoImage->size() );
            int offsetY = sDummyFullImage[THREAD_ID][nthDevice].rows - bottomRoi.y - pPhotoImage->rows;
            if (offsetY < 0)
                    bottomRoi.height += offsetY, topRoi.height += offsetY;
            int offsetX = sDummyFullImage[THREAD_ID][nthDevice].cols - bottomRoi.x - pPhotoImage->cols;
            if (offsetX < 0)
                    bottomRoi.width += offsetX, topRoi.width += offsetX;
            (*pPhotoImage)(topRoi).copyTo( sDummyFullImage[THREAD_ID][nthDevice] (bottomRoi) );
            pFullImage = &sDummyFullImage[THREAD_ID][nthDevice];
#else
            pFullImage = sPhotoes[THREAD_ID][nthDevice].pPhotoImage;
#endif

        //if (rectifyMapLoaded)
        {
            auto t1 = high_resolution_clock::now();
                #ifdef _DEBUG_FILE
                        //imwrite(DEBUG_PATH + "$beforeRemap_" + to_string(nthDevice) +".bmp",  *sPhotoes[THREAD_ID][nthDevice].pPhotoImage); //sPhotoes[nthDevice]); //*photo );
                #endif
                   /*
                remap( *pFullImage, //sPhotoes[nthDevice], thdFetchLine
                                sRctfMidlines[THREAD_ID][nthDevice],
XX                                        sStereoMap[nthDevice][0], sStereoMap[nthDevice][1],
                                        //INTER_NEAREST   //*3 nearest neighbor interpolation
                                        //INTER_LINEAR      //*4~5  bilinear interpolation
                                        //INTER_CUBIC        //*6  bicubic interpolation
                                        //INTER_AREA
                                                                    //*5 resampling using pixel area relation. It may be a preferred method
                                                                     //*  for image decimation, as it gives moire'-free results. But when the image
                                                                     //* is zoomed, it is similar to the INTER_NEAREST method.
                                        INTER_LANCZOS4    //*9 but 1:1 if Debug Lanczos interpolation over 8x8 neighborhood
                                    );
/**/
                //*
                Rect targetROI = Rect( CUR_LASER_ROI[BoxID-1][nthDevice], Size(CUR_LASER_ROI_WIDTH, CUR_LASER_ROI_HEIGHT));
                switch (opMode)
                {
                case OM_CALIBRATE_SQUARE_BAR:
                                        if (nsUtility::TrimStereoMap( sStereoMap[nthDevice][0], sStereoMap[nthDevice][1],
                                                                        targetROI, sReverseMapTOI[nthDevice], sReverseMapMask[nthDevice]) == RC_OK)
                                        {
                                                cout << "targetROI _" << nsCameras::GetCameraChar(nthDevice) << "=" << targetROI  << endl;
                                                cout << "RMTOI_" << nsCameras::GetCameraChar(nthDevice) << "=" << sReverseMapTOI[nthDevice] << endl;
                                                #ifdef _DEBUG                                            
                                                d = high_resolution_clock::now() - t1;
                                                cout << "FT " << to_string(nthDevice) << " TSM: " << d.count() << endl;
                                                t1 = high_resolution_clock::now();
                                                #endif
                                        }
                                        break;
                case OM_FETCH_RAIL_CROWN:
                                        //ReverseMapTOI = %%%%%
                                        break;
                }
            //#ifdef CHECK_LKG
//return ;//LKG_51
#ifdef _DEBUG_FILE
            imwrite(DEBUG_PATH + "$beforeRRP_" + to_string(nthDevice) +".bmp",  *pFullImage);
#endif
                int rc = nsUtility::ReverseRoundMap(   *pFullImage, //sPhotoes[nthDevice], thdFetchLine
                                                            sStereoMap[nthDevice][0], sStereoMap[nthDevice][1],
                                                            &sReverseMapTOI[nthDevice],
                                                    //230206 cancelled (BoxID == 4) ? nullptr :
                                                   &sReverseMapMask[nthDevice],  //,//Box-1 budged  and turn the masks invalid
                                                            sRctfMidlines[THREAD_ID][nthDevice] );
            //#endif
//return ;//LKG_53

                /**/
#ifdef _DEBUG
                d = high_resolution_clock::now() - t1;
                cout << "FT " << to_string(nthDevice) << " MAP: " << d.count() << endl;
                //for (uint i = 0; i < 2; i++) dilate(sRctfMidlines[nthDevice], sRctfMidlines[nthDevice], Mat());
#endif
#ifndef _DEBUG_FILE
            if (opMode == OM_CALIBRATE_SQUARE_BAR) // || fmExposures.willLoadFromFiles())
#endif
                   imwrite(DEBUG_PATH + "$FL-AR_" + to_string(nthDevice) +".bmp",  sRctfMidlines[THREAD_ID][nthDevice]); //*photo );
                //Mat work = Mat::zeros(OCvImage.rows, OCvImage.cols, CV_8UC1);

                t1 = high_resolution_clock::now();
#ifdef FIND_LINE_FIRST
                threshold( sRctfMidlines[THREAD_ID][nthDevice], sRctfMidlines[THREAD_ID][nthDevice],
                                            50, 255, THRESH_BINARY);  //1 100 M#
                #ifdef _DEBUG2
                                //d = high_resolution_clock::now() - t1;
                                        //cout << "FT " << to_string(nthDevice) << " THD: " << d.count() << endl;
                                //t1 = high_resolution_clock::now();
                #endif
                sMidlineRectStart[nthDevice] = Point(0,0);
#else
                //if (opMode == OM_CALIBRATE_SQUARE_BAR)
                //            imwrite(DEBUG_PATH + "$BeforeFML_" + to_string(nthDevice) +".bmp",  sRctfMidlines[THREAD_ID][nthDevice]); //M#
                /*Rect roi(CUR_LASER_ROI[BoxID-1][nthDevice], Size(CUR_LASER_ROI_WIDTH, CUR_LASER_ROI_HEIGHT));
                roi.height = min(roi.height, sRctfMidlines[THREAD_ID][nthDevice].rows - roi.y),
                roi.width = min(roi.width, sRctfMidlines[THREAD_ID][nthDevice].cols - roi.x); */
                /*Rect rectifiedROI[2] = { {ROI_SE_LEFT_X[BoxID-1], // ROI_LEFT_X[BOX_ID-1][opMode], // 5200, //RectifyRatioY=0.7  5100 if RectifyRatioY=0.6 7100,
                                                                 XXROI_SE_Y_TOP[BoxID-1], //ROI_Y_TOP[BOX_ID-1][opMode],
                                                                ROI_SE_WIDTH, //[opMode],
                                                                ROI_SE_HEIGHT }, //[opMode]); //RectifyRatioX=3.1;  4200 as RectifyRatioX =1.5
                                                                {ROI_SE_RIGHT_X[BoxID-1], //[opMode], //1400, //RectifyRatioY=0.7 1600 as RectifyRatioY=0.6 ,  //3500,
                                                                ROI_SE_Y_TOP[BoxID-1], //[opMode],
                                                                ROI_SE_WIDTH, //[opMode],
                                                                ROI_SE_HEIGHT } }; //[opMode]); //RectifyRatioX=3.1;  4200 as RectifyRatioX =1.5
                */
                Rect rectifiedROI = GetRectifiedROI(BoxID-1, nthDevice, opMode);
                sMidlineRectStart[nthDevice] = rectifiedROI.tl();
                int offset = sRctfMidlines[THREAD_ID][nthDevice].rows - rectifiedROI.y;
                if (offset < rectifiedROI.height)  rectifiedROI.height = offset;
                offset = sRctfMidlines[THREAD_ID][nthDevice].cols - rectifiedROI.x;
                if (offset < rectifiedROI.width)  rectifiedROI.width = offset;
                sRctfMidlines[THREAD_ID][nthDevice] = sRctfMidlines[THREAD_ID][nthDevice]( rectifiedROI ); //rectifiedROI[nthDevice]);
#ifdef _DEBUG_FILE
                                imwrite(DEBUG_PATH + "$FL-BFM_" + to_string(nthDevice) +".bmp",  sRctfMidlines[THREAD_ID][nthDevice]); //*photo );
#endif

//return ;//LKG_55//
                FindMidline( sRctfMidlines[THREAD_ID][nthDevice],
                                            BoxID - 1,
                                            nthDevice,
                                            opMode,
                                            &sWorkRctfML[THREAD_ID][nthDevice]
                             );
//return ;//LKG_57
#ifdef _DEBUG_FILE
                                //imwrite(DEBUG_PATH + "$FM 7_" + to_string(nthDevice) +".bmp",  sRctfMidlines[THREAD_ID][nthDevice]); //*photo );
#endif
                #ifdef _DEBUG
                        d = high_resolution_clock::now() - t1;
                        cout << "FT " << to_string(nthDevice) << " FML: " << d.count() << endl;
                #endif
#endif //FIND_LINE_FIRST

                t1 = high_resolution_clock::now();
                constexpr uint GAP = 2; //3 M#
                for (uint i = 0; i < GAP; i++)
                        dilate(sRctfMidlines[THREAD_ID][nthDevice], sRctfMidlines[THREAD_ID][nthDevice], Mat());
                for (uint i = 0; i < GAP; i++)
                        erode(sRctfMidlines[THREAD_ID][nthDevice], sRctfMidlines[THREAD_ID][nthDevice], Mat());
                #ifdef _DEBUG
                                //d = high_resolution_clock::now() - t1;
                                //cout << "FT " << to_string(nthDevice) << " CLS: " << d.count() << endl;
                #endif

#ifndef _DEBUG_FILE
            if (opMode == OM_CALIBRATE_SQUARE_BAR) // || fmExposures.willLoadFromFiles())
#endif
                        imwrite(DEBUG_PATH + "$RctfMidline_" + to_string(nthDevice) +".bmp",  sRctfMidlines[THREAD_ID][nthDevice]); //M#
        }
}

void thdDisplayPhoto(Mat* photo, QLabel* panel, bool autoDilate)
{
        if (!photo) return;
        if (photo->empty()) return;

        Mat work = Mat::zeros(photo->rows, photo->cols, CV_8UC1);
        if (autoDilate)
        {
            dilate(*photo, work, Mat()); //, Point(-1, -1), 2, 1, 1);
                        //qDebug() << "LG-4-0";
#ifdef LCK_GRD
                        lock_guard<std::mutex> lgMtxPhoto{mtxPhoto};
#else
                        mtxPhoto.lock();
#endif
#ifdef _DEBUG_MUTEX   //LG-4-1
                        qDebug() << "LG-4-1";
#endif
            dilate(work, *photo, Mat());
#ifndef LCK_GRD
                                    mtxPhoto.unlock();
#endif
        } else
        {
#ifdef OS_LINUX //o.w. one of the two won't go alive normally
            //*
            dilate(*photo, work, Mat()); //, Point(-1, -1), 2, 1, 1);
                        //qDebug() << "LG-5-0";
#ifdef LCK_GRD
                    lock_guard<std::mutex> lgMtxPhoto{mtxPhoto};
#else
                    mtxPhoto.lock(); //thdDisplayPhoto
#endif
#ifdef _DEBUG_MUTEX   //LG-5-1
                    qDebug() << "LG-5-1";
#endif

            erode(work, *photo, Mat());
#ifndef LCK_GRD
                                    mtxPhoto.unlock();
#endif
            /**/
#endif
        }
        QImage img(      photo->data,
                                        photo->size().width,  //frameCols,
                                        photo->size().height,  //frameRows,
                                        photo->step,
                                        QImage::Format_Indexed8
                                    );
        QPixmap pixmap = QPixmap::fromImage(img);
        panel->setPixmap(pixmap);
        //panel->setScaledContents(true);  //0111
        if (panel->objectName() == "lbRightImage")
        {
                int a=2;
                //panel->update();
                //panel->clear();
        }
        work.release();    work.deallocate();//221024
}

int MainWindow::StartAutoSE(bool &flag)
{
        flag = true;
        IsAbortingAutoSE = false;
        AutoSEStart = high_resolution_clock::now();
        sPhotoQHead = sPhotoQTail = 0;
        return RC_OK;
}

void MainWindow::ConcludeAutoSE(const uint rc)
{
        mvCallbackContext.TibboOn = false;

        if (sktClientMBD_CMD)
                if(sktClientMBD_CMD->state() == QAbstractSocket::ConnectedState)
                        sktClientMBD_CMD->write( ("SE, " + QString::number(rc)+ "\n").toStdString().c_str() );

        IsAutoSECalibrating = IsAutoSEMeasuring = false;

        //qDebug() << "btCSB to be called - 1 \n";
        if (opMode == OM_CALIBRATE_SQUARE_BAR)
        {
                if (orgModeBeforeACSE != OM_CALIBRATE_SQUARE_BAR)
                        on_btCalibrateSquareBar_clicked();

                if (fmExposures.LFBeforeASOR >= -1)  //ow ignored
                        fmExposures.ui->kbLoadPhotos->setCheckState((fmExposures.LFBeforeASOR) ? Qt::Checked : Qt::Unchecked),
                        fmExposures.LFBeforeASOR = -1;
                if (fmExposures.FSBeforeASOR >= -1)  //ow ignored
                        fmExposures.ui->kbSavePhotos->setCheckState((fmExposures.FSBeforeASOR) ? Qt::Checked : Qt::Unchecked),
                        fmExposures.FSBeforeASOR = -1;
                if (fmExposures.HWTBeforeACSE >= -1)  //ow ignored
                        fmExposures.ui->kbHWTrigger->setCheckState((fmExposures.HWTBeforeACSE) ? Qt::Checked : Qt::Unchecked),
                        fmExposures.HWTBeforeACSE = -1;
        }
        //QThread::msleep(10);
        if (orgModeBeforeACSE == OM_FETCH_RAIL_CROWN)
                on_btFetchRailCrown_clicked();

}

//tm can use eTriggerMode since 220918
int MainWindow::CaptureProcessDual(ETriggerMode tm, bool willProcess, bool willDisplay, const uint THREAD_ID, const bool CaptureLastInQ)
{
#ifdef ROI_IMAGE
        const bool IsROIImage = true;
#else
        const bool IsROIImage = false;
#endif

//return RC_OK;//LKG_31
        int rc;
        //0731 if (!tmLiveShootShowBoth->isActive()) // tmLiveCaptureHandler->  bLiveCapturing)
        if (tm == TriggerMode_Software)
        {
                /*std::thread cleft(thdCapture, Camera_Left_0, false, &mvCallbackContext, THREAD_ID);  //SW-trigger
                std::thread cright(thdCapture, Camera_Right_1, false, &mvCallbackContext, THREAD_ID);
                cleft.join(), cright.join();

          XX      lock_guard<std::mutex> lgMtxPhoto{mtxPhoto};
                if ( sPhotoQTail != sPhotoQHead)  //
XX                        sPhotoQTail  = (sPhotoQTail + 1) % IMAGE_QUEUE_SIZE;  //after thdCapture  */
                for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                        sPhotoes[THREAD_ID][nthDevice].pPhotoImage = &sPhotoMem[THREAD_ID][nthDevice],
                        sPhotoes[THREAD_ID][nthDevice].pTimeStamp = &sTimeStamp[THREAD_ID][nthDevice];
                if ((rc = GrabOnePairImage(&sPhotoes[THREAD_ID][0], mtxPhoto, false, &mvCallbackContext, CaptureLastInQ)) != RC_OK)// thdCapture //thdCapture
                        return rc;
        }

        if (willDisplay)
        {
                std::thread dleft(thdDisplayPhoto, sPhotoes[THREAD_ID][Camera_Left_0].pPhotoImage, //&sPhotoes[Camera_Left_0],
                                                    pnTopPhoto[Camera_Left_0], false);
                std::thread dright(thdDisplayPhoto, sPhotoes[THREAD_ID][Camera_Right_1].pPhotoImage, //&sPhotoes[Camera_Right_1],
                                                        pnTopPhoto[Camera_Right_1], false);
                dleft.join(), dright.join();
        }

        if (fmExposures.IsSavingCapturedPhoto())
        {
                String path;
                switch(opMode)
                {
                case OM_IDLE:   //must be handled by HWTSaveFileHandler?!
                case OM_FETCH_RAIL_CROWN:
                        path = CAPTURE_PATH;   break;
                //case OM_CALIBRATE_CHECKERBOARD:  disabled
                case OM_CALIBRATE_SQUARE_BAR:
                        path = DEBUG_PATH;   break;
                }
                for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                        if (!sPhotoes[THREAD_ID][nthDevice].pPhotoImage->empty())
                                imwrite( nsUtility::GetFileStamp(path, 0, nthDevice),
                                                        *sPhotoes[THREAD_ID][nthDevice].pPhotoImage //sPhotoes[nthDevice]
                                                );
        }
        if (!willProcess)
                return RC_OK;

        if (!sPhotoes[THREAD_ID][Camera_Left_0].pPhotoImage || !sPhotoes[THREAD_ID][Camera_Right_1].pPhotoImage)
                return RC_NO_IMAGE;
        if (sPhotoes[THREAD_ID][Camera_Left_0].pPhotoImage->empty() || sPhotoes[THREAD_ID][Camera_Right_1].pPhotoImage->empty())
            //if (sPhotoes[Camera_Left_0].empty() || sPhotoes[Camera_Right_1].empty())
                return RC_NO_IMAGE;
        //constexpr uint ROI_Y_TOP = 0;   //0
        //constexpr uint ROI_HEIGHT = 1150; //220812 RectifyRatioY=0.7 1300 as RectifyRatioY=0.6; // 1800;  //
        //constexpr uint ROI_WIDTH = 600; //800;  //
//return RC_OK;//LKG_21  jam the RAM soon!

        Rect rectifiedROILeft = GetRectifiedROI(BOX_ID - 1, Camera_Left_0,  opMode);
        Rect rectifiedROIRight = GetRectifiedROI(BOX_ID - 1, Camera_Right_1,  opMode);
               /*(ROI_SE_LEFT_X[BOX_ID-1], // ROI_LEFT_X[BOX_ID-1][opMode], // 5200, //RectifyRatioY=0.7  5100 if RectifyRatioY=0.6 7100,
                                                    ROI_SE_Y_TOP[BOX_ID-1], //ROI_Y_TOP[BOX_ID-1][opMode],
                                                    ROI_SE_WIDTH, //[opMode],
                                                    ROI_SE_HEIGHT ); //[opMode]); //RectifyRatioX=3.1;  4200 as RectifyRatioX =1.5
        if (opMode == OM_FETCH_RAIL_CROWN)
                rectifiedROILeft.x -= fmExposures.SE_VH_OFFSET_X,
                rectifiedROILeft.y -= fmExposures.SE_VH_OFFSET_Y,
                rectifiedROILeft.width += fmExposures.SE_VH_OFFSET_WIDTH,
                rectifiedROILeft.height += fmExposures.SE_VH_OFFSET_HEIGHT;

        Rect rectifiedROIRight(ROI_SE_RIGHT_X[BOX_ID-1], //[opMode], //1400, //RectifyRatioY=0.7 1600 as RectifyRatioY=0.6 ,  //3500,
                                                        ROI_SE_Y_TOP[BOX_ID-1], //[opMode],
                                                        ROI_SE_WIDTH, //[opMode],
                                                        ROI_SE_HEIGHT ); //[opMode]); //RectifyRatioX=3.1;  4200 as RectifyRatioX =1.5
        if (opMode == OM_FETCH_RAIL_CROWN)
                        rectifiedROIRight.x -= fmExposures.SE_VH_OFFSET_X,
                        rectifiedROIRight.y -= fmExposures.SE_VH_OFFSET_Y,
                        rectifiedROIRight.width += fmExposures.SE_VH_OFFSET_WIDTH,
                        rectifiedROIRight.height += fmExposures.SE_VH_OFFSET_HEIGHT;
        */
        duration<double, milli> d1(0), d2(0), d3(0);

//#ifdef CHECK_LKG
        if (fmExposures.areCameras90DgrRotated)
                for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
                        if (! sPhotoes[THREAD_ID][nthCamera].pPhotoImage->empty()) //sPhotoes[nthCamera].empty())
                                switch(nthCamera)
                                {
                                case nsCameras::Camera_Left_0:
                                                                        //imwrite(DEBUG_PATH + "$BeforeRotate_" + to_string(nthCamera) +".bmp",  *sPhotoes[THREAD_ID][nthCamera].pPhotoImage);
                                                                        rotate( *sPhotoes[THREAD_ID][nthCamera].pPhotoImage,  //sPhotoes[nthCamera],
                                                                                        *sPhotoes[THREAD_ID][nthCamera].pPhotoImage,  //sPhotoes[nthCamera],
                                                                                        ROTATE_90_CLOCKWISE);
                                                                        //imwrite(DEBUG_PATH + "$AfterRotate_" + to_string(nthCamera) +".bmp",  *sPhotoes[THREAD_ID][nthCamera].pPhotoImage); //sPhotoes[nthDevice]); //*photo );
                                                                        break;
                                case nsCameras::Camera_Right_1:
                                                                        rotate( *sPhotoes[THREAD_ID][nthCamera].pPhotoImage,  //sPhotoes[nthCamera],
                                                                                        *sPhotoes[THREAD_ID][nthCamera].pPhotoImage,  //sPhotoes[nthCamera],
                                                                                        ROTATE_90_COUNTERCLOCKWISE);
                                                                        break;
                                }
//#endif

        //Point leftROIStart, rightROIStart;
        switch(opMode)
        {
        case OM_CALIBRATE_CHECKERBOARD:
                            {
#ifdef _DEBUG_MUTEX   //LG-6-0
            qDebug() << "LG-6-0";
#endif
                            #ifdef LCK_GRD
                                        lock_guard<std::mutex> lgMtxPhoto{mtxPhoto};
                            #else
                                        mtxPhoto.lock();
                            #endif
                                        qDebug() << "LG-6-1";
                                    imwrite(CalibrateDir + "/" + to_string(PhotoPairStartNo) + "L.bmp",  *sPhotoes[THREAD_ID][Camera_Left_0].pPhotoImage);  //sPhotoes[Camera_Left_0] );
                                    imwrite(CalibrateDir + "/" + to_string(PhotoPairStartNo) + "R.bmp",  *sPhotoes[THREAD_ID][Camera_Right_1].pPhotoImage);  //sPhotoes[Camera_Right_1] );
                            #ifndef LCK_GRD
                                    mtxPhoto.unlock();
                            #endif
                            }
                            PhotoPairStartNo++;
                            rc = RC_OK;
                            break;
        case OM_CALIBRATE_SQUARE_BAR:
        case OM_FETCH_RAIL_CROWN:
                        {
                /*#ifdef ROI_IMAGE
                            leftROIStart = GetRectifiedROI(BOX_ID - 1, Camera_Left_0,  opMode);
                            rightROIStart = GetRectifiedROI(BOX_ID - 1, Camera_Right_1,  opMode);
                #else
                                leftROIStart = {0, 0}, rightROIStart ={0, 0};
                #endif*/
                                const String COMMA = ",";
                                //get the middle lines
                                auto t1 = high_resolution_clock::now();
                                //thdFetchLine(Camera_Left_0, opMode);
                                //thdFetchLine(Camera_Right_1, opMode);
//#ifdef CHECK_LKG
                                std::thread fleft(thdFetchLine, Camera_Left_0, opMode, BOX_ID, THREAD_ID); //&sPhotoes[Camera_Left_0]);
                                std::thread fright(thdFetchLine, Camera_Right_1, opMode, BOX_ID, THREAD_ID); //&sPhotoes[Camera_Right_1]);
                                fleft.join(), fright.join(); /**/
//#endif
                                d1 = high_resolution_clock::now() - t1;
                #ifdef _DEBUG_FILE
                                    //imwrite(DEBUG_PATH + "$FM 9_0.bmp",  *sPhotoes[THREAD_ID][Camera_Left_0].pPhotoImage);
                                    //imwrite(DEBUG_PATH + "$FM 9_1.bmp",  *sPhotoes[THREAD_ID][Camera_Right_1].pPhotoImage);
                #endif
                                if (opMode == OM_CALIBRATE_SQUARE_BAR)
                                        if (!saveSECResults())
                                {
                                                rc = RC_FAIL_TO_SAVE;
                #ifdef MOVED
                                        FileStorage FS(CalibrateDir + "last/RPTOI", FileStorage::WRITE);  XX
                                        if (FS.isOpened() )
                                        {
                                                for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                                                {
                                                        String cameraID = nsCameras::GetCameraChar(nthDevice);
                                                        String key = cameraID + "_RPTOI_X";
                                                        FS << key << sReverseMapTOI[nthDevice].x;
                                                        key = cameraID + "_RPTOI_Y", FS << key << sReverseMapTOI[nthDevice].y;
                                                        key = cameraID + "_RPTOI_W", FS << key << sReverseMapTOI[nthDevice].width;
                                                        key = cameraID + "_RPTOI_H", FS << key << sReverseMapTOI[nthDevice].height;
                                                }
                                                FS.release();
                                                //QMessageBox::information(this, "Calibration Stage II", "The SE(Straight Edge) is calibrated.");
                                        }
                #endif
                                }
                                if (rectifyMapLoaded)
                                {
                                        if (willDisplay) {
                                                std::thread dleft(thdDisplayPhoto, &sRctfMidlines[THREAD_ID][Camera_Left_0], pnMidPhoto[Camera_Left_0], true);
                                                std::thread dright(thdDisplayPhoto, &sRctfMidlines[THREAD_ID][Camera_Right_1], pnMidPhoto[Camera_Right_1], true);
                                                dleft.join(), dright.join();
                                        }
                                        t1 = high_resolution_clock::now();
                                        vector<Point3d> fusedCoords3D;
                                        //*
//break;//LKG_11
                                        constexpr double RATIO = 1.3;
                                        rc = GetStereo3DFromRectifiedPair (
                                                                sRctfMidlines[THREAD_ID][Camera_Left_0], (IsROIImage) ? nullptr : &rectifiedROILeft,
                                                                sRctfMidlines[THREAD_ID][Camera_Right_1], (IsROIImage) ? nullptr : &rectifiedROIRight,
                                                                sProjectionMatrices_P[Camera_Right_1].at<double>(0, 3), // int pxlCameraOffset,  //Tx, pixels
                                                                sProjectionMatrices_P[Camera_Right_1].at<double>(0, 0), // pxlRectifiedFocus,  //Tx, pixels
                                                                LENS_FOCUS_mm
                                                                        //* ((RectifyAlpha > 0) ? RectifyAlpha : RectifyRatioY)   // RectifiedFocus_mm, //mm
                                                                        * fmExposures.FOCUS_TUNE[BOX_ID - 1] //0.404// .427  // * 0.428 //RectifyRatioY=0.7 0.66 //0.66<->RectifyRatioY=0.6//RATIO   // 0.907 M#
                                                                ,
                                                                Point2d(sProjectionMatrices_P[Camera_Right_1].at<double>(0, 2), sProjectionMatrices_P[Camera_Right_1].at<double>(1, 2)), //principalPoint,
                                                                fusedCoords3D,  //in Camera's view
                                                                sMidlineRectStart[Camera_Left_0],  sMidlineRectStart[Camera_Right_1] //leftROIStart, rightROIStart
                                                        );  /**/
                                        d2 = high_resolution_clock::now() - t1;
                                        switch(opMode)
                                        {
                                        case OM_CALIBRATE_SQUARE_BAR:
                                                    if (!IsAutoSEMeasuring)
                                                    {
                                                            if ((rc = DetermineXAxisOfCrossSquareBar(fusedCoords3D, GaugeAxisX[0], GaugeAxisX[1], GaugeAxisY)) == RC_OK)
                                                            {
                                                                    double axisXLen = norm(GaugeAxisX[1] - GaugeAxisX[0]);
                                                                    cout << "AxisX Len=" << axisXLen << endl;  //#DEBUGDEBUG

                                                                    double axisYLen = norm(GaugeAxisX[1] - GaugeAxisY);
                                                                    cout << "AxisY Len=" <<  axisYLen << endl;  //#DEBUGDEBUG

                                                                    if (abs(fmExposures.SBWidth - axisXLen) > fmExposures.BarSectEstTrn)
                                                                            if  (axisXLen > fmExposures.SBWidth)
                                                                                    qDebug() << "*** estimated too long\n";
                                                                            else
                                                                                    qDebug() << "*** estimated too short\n" ;

                                                                    FileStorage FS(CalibrateDir + "last/LaserCS", FileStorage::WRITE);
                                                                    if (FS.isOpened() )
                                                                    {
                                                                            FS << "AxisX_0" << GaugeAxisX[0];
                                                                            FS << "AxisX_1" << GaugeAxisX[1];
                                                                            FS << "AxisY_1" << GaugeAxisY;
                                                                            FS.release();
                                                                            //QMessageBox::information(this, "Calibration Stage II", "The SE(Straight Edge) is calibrated.");
                                                                    }
                                                            }
                                                            if (IsAutoSECalibrating)
                                                                    ConcludeAutoSE(rc);

                                                    } else {  //IsAutoSEMeasuring
                                                            if (rc == RC_OK)
                                                            {
                                                                    Point3d measuredAxisX[2], measuredAxisY;
                                                                    if ((rc = DetermineXAxisOfCrossSquareBar(fusedCoords3D, measuredAxisX[0], measuredAxisX[1], measuredAxisY)) == RC_OK)
                                                                    {
                                                                                vector<Point3d> measuredAxisPoints = {/*measuredAxisX[0], */measuredAxisX[1], measuredAxisY};
                                                                                vector<Point2d> mapped2CS_XY2D;
                                                                                //moved into loadSECResults
                                                                                if ((rc = MapRail3DCoordsTo2DCS(    measuredAxisPoints,
                                                                                                                                                        GaugeAxisX[0], GaugeAxisX[1], GaugeAxisY,
                                                                                                                                                        mapped2CS_XY2D)) == RC_OK)
                                                                                {
                                                                                        stringstream packetStr, packetStrBD;
                                                                                        packetStr << mapped2CS_XY2D[0].x << COMMA
                                                                                                                         << mapped2CS_XY2D[0].y << COMMA
                                                                                                                         << mapped2CS_XY2D[1].x << COMMA
                                                                                                                         << mapped2CS_XY2D[1].y << "\n";
                                                                                        cout << "SE CSC VH: " << packetStr.str();
                                                                                        packetStrBD << packetStr.rdbuf();

                                                                                        if (sktClientMBD_VH) //sktMBD[nthBox])
                                                                                                if (sktClientMBD_VH->state() == QAbstractSocket::ConnectedState)
                                                                                                {
                                                                                                        sktClientMBD_VH->write(packetStrBD.str().c_str());
                                                                                                        sktClientMBD_VH->waitForBytesWritten();
                                                                                                }
                                                                                }
                                                                     }
                                                            }
                                                            ConcludeAutoSE(rc);
                                                    }
                                                    //221021 break;
                                        case OM_FETCH_RAIL_CROWN:
                                                    {
//break;//LKG_9
                                                            t1 = high_resolution_clock::now();
                                                            vector<Point2d> mapped2CS_XY2D;
                                                            //moved into loadSECResults
                                                            if ((rc = MapRail3DCoordsTo2DCS(    fusedCoords3D,
                                                                                                                                    GaugeAxisX[0], GaugeAxisX[1],
                                                                                                                                    GaugeAxisY,
                                                                                                                                mapped2CS_XY2D)) == RC_OK)
                                                            {
                                                                                                    //qDebug() << "mapped 2 2D \n";
//mapped2CS_XY2D.clear();break;//LKG_7
                                                                    constexpr double SPATIAL_RATIO = 0.2;
                                                                    constexpr uint RESOLUTION_RATIO = 10;  //1 for 1mm per x-uint, 10 for 0.1mm
                                                                    Range rangeX, rangeY;
                                                                    GetCoordinatesRange(mapped2CS_XY2D, rangeX, rangeY, RESOLUTION_RATIO);
                                                                                            cout << "rangeX=" << rangeX << ", rangeY=" << rangeY <<"\n";
                                                                    const uint xr = rangeX.size();
                                                                    const uint yr = rangeY.size();
                                                                    uint HORZ_SPACE = round(xr * SPATIAL_RATIO);
                                                                    uint VERT_SPACE = round(yr * SPATIAL_RATIO);
                                                                    //cout << "rangeX.size=" <<  rangeX.size() << endl;
                                                                    //cout << "rangeY.size=" <<  rangeY.size() << endl;
 //mapped2CS_XY2D.clear();break;//LKG_5
                                                                      const  uint width = HORZ_SPACE * 2 + xr + 1;
                                                                      const  uint height = VERT_SPACE * 2 + yr + 1;
                                                        #ifdef _AUTO_CS_XY
                                                                    Mat work = Mat::zeros(  height , //VERT_SPACE * 2 + yr + 1,
                                                                                                                    width, //HORZ_SPACE * 2 + xr + 1,
                                                                                                                    CV_8UC1);
//work.release();work.deallocate();mapped2CS_XY2D.clear();break;//LKG_3
                                                        #else
                                                                     //sWorkCS_XY[THREAD_ID] = Mat::zeros(  height, width, CV_8UC1);
                                                                    ///221104
                                                                    if (sWorkCS_XY[THREAD_ID].empty())
                                                                            sWorkCS_XY[THREAD_ID] = Mat::zeros(  height, width, CV_8UC1);
                                                                    else {
                                                                          //if ( sWorkCS_XY[THREAD_ID].empty()) sWorkCS_XY[THREAD_ID].create(height, width, CV_8UC1);
                                                                          if  ((sWorkCS_XY[THREAD_ID].rows != height) || (sWorkCS_XY[THREAD_ID].cols != width) )
                                                                                  sWorkCS_XY[THREAD_ID].create(height, width, CV_8UC1);
                                                                          sWorkCS_XY[THREAD_ID].setTo(0);
                                                                    }
                                                        #endif

                                                                    for (auto p : mapped2CS_XY2D)
                                                                    {
                                                                            double d = rangeY.end - RESOLUTION_RATIO * p.y;
                                                                            int r = round(VERT_SPACE + d); //rangeY.start upsidedown
                                                                            d = RESOLUTION_RATIO * p.x - rangeX.start;
                                                                            int c = round(HORZ_SPACE + d);  //777 MUST be anchored with GaugeAxisX[1]
                                                        #ifdef _AUTO_CS_XY
                                                                            work
                                                        #else
                                                                            sWorkCS_XY[THREAD_ID]
                                                        #endif

                                                                                    .at<uchar>( r, c ) = 255;
                                                                    }
                                                                    d3 = high_resolution_clock::now() - t1;
#ifdef _DEBUG
                                                                            imwrite(CalibrateDir + "/rail.bmp",
                                                                                    #ifdef _AUTO_CS_XY
                                                                                                        work
                                                                                    #else
                                                                                                        sWorkCS_XY[THREAD_ID]
                                                                                    #endif
                                                                                    );
                                                                    //for (uint i = 0; i < 1; i++)  dilate(work, work, Mat());
                                                                    if (!fmExposures.IsHardwareTriggered())
                                                                    {
                                                                            std::thread showFused(thdDisplayPhoto,
                                                                                      #ifdef _AUTO_CS_XY
                                                                                                          &work,
                                                                                      #else
                                                                                                          &sWorkCS_XY[THREAD_ID],
                                                                                      #endif
                                                                                                  pnFused3DImage[Camera_Right_1], true); //ui->  //pnMidPhoto[Camera_Right_1]); //ui->
                                                                            showFused.join();
                                                                    }
#endif
                                                                    Point2d V;  Point2d H;
//work.release(); work.deallocate();mapped2CS_XY2D.clear();break;//LKG_1
                                                                    if ((rc = GetRailHeadVH(BOX_ID-1,  //for cstm
                                                                                                                opMode,
                                                                                                                mapped2CS_XY2D,
                                                                                                                fmExposures.VHVertOffset, V, H)) == RC_OK)
                                                                    {
                                                                            if (opMode == OM_CALIBRATE_SQUARE_BAR)
                                                                                    if (V.x > H.x)
                                                                                    {
                                                                                            AxisYSlant = V.x - H.x;
                                                                                            FileStorage FS(CalibrateDir + "last/LaserCS", FileStorage::APPEND);   //WRITE
                                                                                            if (FS.isOpened() )
                                                                                            {
                                                                                                    FS << "AxisYSlant" << AxisYSlant;
                                                                                                    FS.release();
                                                                                            }
                                                                                    }
                                                                            //ulong elapsed = railTimer.nsecsElapsed() - TriggerStart ;
                                                                            ulong elapsed = (*sPhotoes[THREAD_ID][0].pTimeStamp + *sPhotoes[THREAD_ID][1].pTimeStamp)  //(sTimeStamps[0]  + sTimeStamps[1] )
                                                                                                            / 2; // - TriggerStart ;
                                                                            //ulong elapsed = railTimer.nsecsElapsed() - TriggerStart ;
                                                                            if (TriggerCount == 3)
                                                                            {
                                                                                    const double DEFAULT_EXPOSURES[2][C_TotalBoxes][2] = {
                                                                                                        { {20, 20},  {20, 20},  {18, 18}, {15, 15}, {20, 20} },
                                                                                                        { {24, 24},  {22, 22},  {22, 22}, {16, 16}, {25, 25} }
                                                                                                    }
                                                                                                    ;
                                                                                    //const double AutoTuneRatios[C_TotalBoxes][2] = { {0.2, 0.5},  {0.2, 0.5},  {0.2, 0.6}, {0.2, 0.5}, {0.2, 0.6} };

                                                                                    const uint heightLevel = (V.y > 16.0)  ?  0 : 1;
                                                                                    for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
                                                                                    {
                                                                                                               //double offset = V.y - 13.0;  //M#
                                                                                                               //double ratio = (offset > 0) ? AutoTuneRatios[BOX_ID-1][0] : AutoTuneRatios[BOX_ID-1][1];
                                                                                                               //fmExposures.setupCameraExposure(nthCamera, DEFAULT_EXPOSURES[BOX_ID-1][nthCamera] - ratio * offset );
                                                                                               //fmExposures.setupCameraExposure(nthCamera, DEFAULT_EXPOSURES[heightLevel][BOX_ID-1][nthCamera]);
                                                                                    }
                                                                                    //suspected to stop
                                                                                    //fmExposures.on_edLeftCameraExpo_returnPressed();
                                                                                    //fmExposures.on_edRightCameraExpo_returnPressed();
                                                                            }
                                                                            //elapsed /= ns2ms;
                                                                            stringstream packetStr, packetStrBD;
                                                                            packetStr << V.x << COMMA << (V.y + fmExposures.SEVOffset) * fmExposures.VFineTune << COMMA
                                                                                                << (H.x + AxisYSlant * fmExposures.VHVertOffset / 10 + fmExposures.SEHOffset) * fmExposures.HFineTune << COMMA
                                                                                                << H.y << COMMA
                                                                                                << TriggerCount << COMMA
                                                                                                <<  elapsed //us  start() invalidate()
                                                                                                << "\n";
                                                                            cout << "V-H: " << packetStr.str();
                                                                            packetStrBD << packetStr.rdbuf();
                                                                            //PackVHString
                                                                            if ((opMode == OM_FETCH_RAIL_CROWN))
                                                                            {
#ifdef _RALPH_SIMULATE
                                                                                //while not rail end => Controlled by START/STOP signals from Tibbo

                                                                                      int Hi=0;
                                                                                      int Vi=0;
                                                                                      for (int j = 0; j <= 9; j++)  // for j=0 to 9
                                                                                              for (int i = 1; i <= 5; i++)  // for i=1 to 5
                                                                                                    if (sktMPC[i-1])
                                                                                                        if (sktMPC[i-1]->state() == QAbstractSocket::ConnectedState)
                                                                                                        {
                                                                                                                Hi =Hi + i; //H(i) =H(i) + i
                                                                                                                Vi = Vi + i; //V(i) = V(i) - i
                                                                                                                //send H(i), V(i)
                                                                                                                packetStr.clear();
                                                                                                                packetStr << V.x << COMMA << Vi << COMMA
                                                                                                                                    << Hi << COMMA << H.y << COMMA
                                                                                                                                    << TriggerCount << COMMA <<  elapsed << "\n";
                                                                                                                sktMPC[i-1]->write(packetStr.str().c_str());
                                                                                                                sktMPC[i-1]->waitForBytesWritten();
                                                                                                                QThread::msleep(40);
                                                                                                        }
                                                                                      for (int j = 10; j <= 19; j++)  //  for j=10 to 19
                                                                                            for (int i = 1; i <= 5; i++)  // for i=1 to 5
                                                                                                   if (sktMPC[i-1])
                                                                                                       if (sktMPC[i-1]->state() == QAbstractSocket::ConnectedState)
                                                                                                       {
                                                                                                           Hi =Hi - i;  //H(i) =H(i) - i
                                                                                                           Vi = Vi - i;  //V(i) = V(i) - i
                                                                                                            //send H(i), V(i)
                                                                                                           packetStr.clear();
                                                                                                           packetStr << V.x << COMMA << Vi << COMMA
                                                                                                                               << Hi << COMMA << H.y << COMMA
                                                                                                                               << TriggerCount << COMMA <<  elapsed << "\n";
                                                                                                           sktMPC[i-1]->write(packetStr.str().c_str());
                                                                                                           sktMPC[i-1]->waitForBytesWritten();
                                                                                                           QThread::msleep(40);
                                                                                                   }
#else
#ifdef _SIMULATE_ALL_MPC_SEVERS
                                                                                for (uint nthBox = 0; nthBox < C_TotalPCs; nthBox++)
                                                                                {
#else
                                                                                {
                                                                                    uint nthBox = BOX_ID - 1;
#endif
#ifdef _BOX_PLAY_CLIENT
                                                                                    if (sktClientMBD_VH) //sktMBD[nthBox])
                                                                                            if (sktClientMBD_VH->state() == QAbstractSocket::ConnectedState)
                                                                                            {
                                                                                                    sktClientMBD_VH->write(packetStrBD.str().c_str());
                                                                                                    sktClientMBD_VH->waitForBytesWritten();
                                                                                            }
#else
                                                                                    if (sktMPC[nthBox])
                                                                                            if (sktMPC[nthBox]->state() == QAbstractSocket::ConnectedState)
                                                                                            {
                                                                                                    sktMPC[nthBox]->write(packetStr.str().c_str());
                                                                                                    sktMPC[nthBox]->waitForBytesWritten();
//cout << packetStr.str().c_str() <<"\n";
                                                                                            }
#endif
                                                                                }
#endif
                                                                          }
                                                                    }
                                                            #ifdef _AUTO_CS_XY
                                                                    work.release();   work.deallocate();//221024
                                                            #endif
                                                            }
                                                            mapped2CS_XY2D.clear();
//break;//LKG_0
                                                            if (rc != RC_OK)
                                                                    if ( (opMode == OM_FETCH_RAIL_CROWN) )
#ifdef _SIMULATE_ALL_MPC_SEVERS
                                                                        for (uint nthBox = 0; nthBox < C_TotalPCs; nthBox++)
                                                                        {
#else
                                                                        {
                                                                            uint nthBox = BOX_ID - 1;
#endif


#ifdef _BOX_PLAY_CLIENT
                                                                                    if (sktClientMBD_VH) //sktMBD[nthBox])
                                                                                            if (sktClientMBD_VH->state() == QAbstractSocket::ConnectedState)
                                                                                            {
                                                                                                    ulong elapsed = railTimer.nsecsElapsed() - TriggerStart ;
                                                                                                    elapsed /= ns2ms;
                                                                                                    String packetStr = COMMA + COMMA + COMMA + COMMA
                                                                                                                                        + to_string(TriggerCount) + COMMA
                                                                                                                                        + to_string(elapsed) + "\n";
                                                                                                    sktClientMBD_VH->write(packetStr.c_str());  //(rc != RC_OK)
                                                                                                    sktClientMBD_VH->waitForBytesWritten();
                                                                                            }
#else
                                                                            if (sktMPC[nthBox])
                                                                                    if (sktMPC[nthBox]->state() == QAbstractSocket::ConnectedState)
                                                                                    {
                                                                                            ulong elapsed = railTimer.nsecsElapsed() - TriggerStart ;
                                                                                            elapsed /= ns2ms;
                                                                                            String packetStr = COMMA + COMMA + COMMA + COMMA
                                                                                                                                + to_string(TriggerCount) + COMMA
                                                                                                                                + to_string(elapsed) + "\n";
                                                                                            sktMPC[nthBox]->write(packetStr.c_str());
                                                                                            sktMPC[nthBox]->waitForBytesWritten();
                                                                                    }
#endif
                                                                    }
                                                    } //OM_FETCH_RAIL_CROWN
                                                    break;
                                        }
                                } else  {//if (rectifyMapLoaded)
                                        if (willDisplay) {
                                                std::thread dleft(thdDisplayPhoto, sPhotoes[THREAD_ID][Camera_Left_0].pPhotoImage, //&sPhotoes[Camera_Left_0],
                                                                                        pnMidPhoto[Camera_Left_0], true);
                                                std::thread dright(thdDisplayPhoto, sPhotoes[THREAD_ID][Camera_Right_1].pPhotoImage, //&sPhotoes[Camera_Right_1],
                                                                                            pnMidPhoto[Camera_Right_1], true);
                                                dleft.join(), dright.join();
                                        }
                                        rc = RC_FAIL_TO_LOAD;
                                }
                                /**/
                        }
            break;

                   break;
        }  //case

        if (opMode == OM_FETCH_RAIL_CROWN)
        {
#ifdef _DEBUG
                cout << "(Find mid-line + Remap) x 2 =" << d1.count() << endl;
                cout << "Fuse rectified pair + Convert to CS XY plane =" << d2.count() + d3.count()  << endl;
                //cout << "convert to CS XY plane =" << d3.count() << endl;
#else
            cout << "Cost " << d1.count() + d3.count() << "ms\n" ;
#endif
        }
        for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                if (sPhotoes[THREAD_ID][nthDevice].pPhotoImage) //sPhotoMem[nthDevice].release();  //123
                        if (!sPhotoes[THREAD_ID][nthDevice].pPhotoImage->empty())
                        {
                                sPhotoes[THREAD_ID][nthDevice].pPhotoImage->release();
                                sPhotoes[THREAD_ID][nthDevice].pPhotoImage = nullptr;
                                *sPhotoes[THREAD_ID][nthDevice].pTimeStamp = 0;
                                sPhotoes[THREAD_ID][nthDevice].pTimeStamp = nullptr;
                        }

        return rc;

}


void MainWindow::HWTSECaptureHandler()
{
        tmHWTSECapture->stop();
        if (HWTProcessHandler(100) == RC_NO_IMAGE)
                tmHWTSECapture->start(1);
}

void MainWindow::HWTSaveFileHandler()
{
        // if (!fmExposures.SaveCapturedPhoto()) return;

        while (!mtxPhotoQ.try_lock())  //HWTSaveFileHandler
        {
                //qDebug() << "HWT SFH  FoL";
                QThread::msleep(10);
                QApplication::processEvents();
        }
        //mtxPhotoQ.lock(); //HWTSaveFileHandler
        //qDebug() << "HWT SFH  LK";
        if ( sPhotoQTail != sPhotoQHead)  //empty
        {
                uint idx = sPhotoQTail;
                sPhotoQTail  = (sPhotoQTail + 1) % IMAGE_QUEUE_SIZE;  //HWTSaveFileHandler
                uint curCount = TriggerCount++;  //HWTSaveFileHandler
                mtxPhotoQ.unlock();

                if (fmExposures.IsSavingCapturedPhoto())
                        for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                                if (!sPhotoQueues[idx][nthDevice].photoImage.empty())
                                {
                                        String filename = nsUtility::GetFileStamp((fmExposures.SOR_Dir == "") ? CAPTURE_PATH : fmExposures.SOR_Dir,
                                                                                  curCount, nthDevice);
                                        qDebug() << QString::fromStdString(filename) << " @ "
                                                            << sPhotoQueues[idx][nthDevice].timeStamp; // sPhotoTimestamps[idx][nthDevice] ;
                                        imwrite( filename , //nsUtility::GetFileStamp(CAPTURE_SUBDIR, curCount, nthDevice),  //M# will jam
                                                            sPhotoQueues[idx][nthDevice].photoImage );
                                        sPhotoQueues[idx][nthDevice].photoImage.release();  //123
                                        sPhotoQueues[idx][nthDevice].timeStamp = 0;
                                }
        } else
                mtxPhotoQ.unlock();

        //qDebug() << "HWT SFH  UL";
}

int MainWindow::HWTProcessHandler(uint THREAD_ID)  //BLOCKING, all SHARED!
{
//Rgs-M3
        THREAD_ID %= C_ParallelProcessPhotos;
        //qDebug() << "HWT Hdl-0" << ID;
        //if (!mvCallbackContext.TibboOn)  //221003 if (!fmExposures.IsHardwareTriggered())
         //       return RC_WRONG_MODE;

        static bool emptyShown = false;
        int rc = RC_NO_IMAGE;
        /*unique_lock<std::mutex> ulMtxPhotoQ(mtxPhotoQ);
        while (cndtVrbl.wait_until(ulMtxPhotoQ, high_resolution_clock::now() + chrono::milliseconds(200)) == cv_status::timeout)
                qDebug() << "HWT " << ID << " got no lock!";
*/

        //qDebug() << "TLQ-12-0";
        while (!mtxPhotoQ.try_lock())  //HWTProcessHandler
        {
                //qDebug() << "HWT PH" << THREAD_ID << "FoL";
                QThread::msleep(1);
                QApplication::processEvents();
        }
//qDebug() << "HWT PH" << THREAD_ID << "LK";
        try {
                //mtxPhotoQ.lock(); //HWTProcessHandler
                //qDebug() << "TLQ-12-1";
        #ifdef _DEBUG_SHOT_TmSmp
                        if (mvCallbackContext.TibboOn)
                              qDebug() << "HWT QH=" << sPhotoQHead  << ", QT=" <<  sPhotoQTail;
        #endif
                const uint idxTail = sPhotoQTail;
                if ( sPhotoQTail != sPhotoQHead)
                {
                      emptyShown = false;
                      sPhotoQTail  = (sPhotoQTail + 1) % IMAGE_QUEUE_SIZE;  //HWTProcessHandler
                      TriggerCount++;
                      mtxPhotoQ.unlock(); //cndtVrbl.notify_all(); //
qDebug() << "HWT PH" << THREAD_ID << "UL-1";
                      //qDebug() << "TLQ-12-2";

                      for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                      {
                              sPhotoes[THREAD_ID][nthDevice].pPhotoImage  = &sPhotoQueues[idxTail][nthDevice].photoImage;
                              sPhotoes[THREAD_ID][nthDevice].pTimeStamp = &sPhotoQueues[idxTail][nthDevice].timeStamp;
                                      //* sTimeStamps[nthDevice] = sPhotoTimestamps[idx][nthDevice];
                      }
                                      qDebug() << "HWT" << THREAD_ID << "grabs" << idxTail; // << sPhotoQueues[idxTail][0].photoImage.rows;
                      rc = CaptureProcessDual(TriggerMode_Hardware, WILL_PROCESS, NO_DISPLAY, THREAD_ID);  //HWTProcessHandler
                      if (rc != RC_OK)
                              qDebug() << "HWT" << THREAD_ID << "rc="  << rc << "\n";
                } else {
                        mtxPhotoQ.unlock(); //cndtVrbl.notify_all(); //
//qDebug() << "HWT PH" << THREAD_ID << "UL-2";
                        if (!emptyShown)
                                qDebug() << "__________________ PQ-Empty H=T=" << idxTail <<"_________________", emptyShown = true;
                }
                //qDebug() << "HWT Hdl-1" << ID;
                //if (rc == RC_NO_IMAGE) qDebug() << "PQ-Empty ? " ;
        /*
                if ( sPhotoQTail != sPhotoQHead)  //empty
                {
                        uint idx = sPhotoQTail;
                        sPhotoQTail  = (sPhotoQTail + 1) % IMAGE_QUEUE_SIZE;  //cmnted
                        TriggerCount++;
                        mtxPhotoQ.unlock(); //cndtVrbl.notify_all(); //
                        qDebug() << "TLQ-12-2";

                        for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                        {
        XX                        sPhotoes[-1][nthDevice].pPhotoImage = &sPhotoQueues[idx][nthDevice].photoImage;
                                sPhotoes[-1][nthDevice].pTimeStamp = &sPhotoQueues[idx][nthDevice].timeStamp;
                                        //* sTimeStamps[nthDevice] = sPhotoTimestamps[idx][nthDevice];
                        }
XX                                        qDebug() << "HWT " << ID << "x grabs photo " << idx ;
        XX                int rc = CaptureProcessDual(TriggerMode_Hardware, WILL_PROCESS, NO_DISPLAY); //XXX
                        if (rc != RC_OK)
                                qDebug() << "HWT" << ID << "XXprocessed code = "  << rc << "\n";
                } else {
                        mtxPhotoQ.unlock(); //cndtVrbl.notify_all(); //
                        qDebug() << "x TLQ-12-3";
            }
            qDebug() << "HWT Hdl-1" << ID; */
        } catch (const GenericException& e) {
                     qDebug() << "HWT PH:" << e.GetDescription();
                     mtxPhotoQ.unlock(); //cndtVrbl.notify_all(); //
qDebug() << "HWT PH" << THREAD_ID << "UL-3";
         }
        //mtxPhotoQ.unlock(); //cndtVrbl.notify_all(); //
        return rc;
}


#define tmHWTProcessHandler(TIMER_ID)\
{\
         tmHWTLiveProcess[TIMER_ID]->stop();\
        int rc = HWTProcessHandler(TIMER_ID);\
        if (mvCallbackContext.TibboOn || (rc != RC_NO_IMAGE))\
                tmHWTLiveProcess[TIMER_ID]->start(hwLiveShootInterval);\
}

void MainWindow::tmHWTProcessHandler0() {
        const uint TIMER_ID = 0;
        tmHWTProcessHandler(TIMER_ID);
        /*QTimer * timer = qobject_cast<QTimer*>(sender());
        qDebug() << "HWT 0" << timer->timerId();
            tmHWTLiveProcess[0]->stop();
            int rc = HWTProcessHandler(0);  XX
            if (mvCallbackContext.TibboOn || (rc != RC_NO_IMAGE))
XX                    tmHWTLiveProcess[0]->start(hwLiveShootInterval); */
 }
void MainWindow::tmHWTProcessHandler1() {
    const uint TIMER_ID = 1;
    tmHWTProcessHandler(TIMER_ID);
}
void MainWindow::tmHWTProcessHandler2() {
    const uint TIMER_ID = 2;
    tmHWTProcessHandler(TIMER_ID);
}
void MainWindow::tmHWTProcessHandler3() {
    const uint TIMER_ID = 3;
    tmHWTProcessHandler(TIMER_ID);
}
void MainWindow::tmHWTProcessHandler4() {
    const uint TIMER_ID = 4;
    tmHWTProcessHandler(TIMER_ID);
}
void MainWindow::tmHWTProcessHandler5() {
    const uint TIMER_ID = 5;
    tmHWTProcessHandler(TIMER_ID);
}
void MainWindow::tmHWTProcessHandler6() {
    const uint TIMER_ID = 6;
    tmHWTProcessHandler(TIMER_ID);
}
void MainWindow::tmHWTProcessHandler7() {
    const uint TIMER_ID = 7;
    tmHWTProcessHandler(TIMER_ID);
}


void MainWindow::StartHWLiveShootTimer(uint msInterval)
{
#if (!DIRECT_QUEUE)
        if (!tmHWTLiveShoot->isActive())
                tmHWTLiveShoot->start(hwLiveShootInterval = msInterval);  //IS134 save queue
#endif
        switch(opMode)
        {
        case OM_IDLE: // FILE_SAVING
                if (fmExposures.IsSavingCapturedPhoto())
                        tmHWTLiveSave->start(hwLiveShootInterval = msInterval);
                else
                        tmHWTLiveSave->stop();
                break;
        case OM_FETCH_RAIL_CROWN:
                for (int i = 0; i < C_ParallelProcessPhotos; i++)
                        tmHWTLiveProcess[i]->start(hwLiveShootInterval = msInterval);
                break;
        case OM_CALIBRATE_CHECKERBOARD:
        case OM_CALIBRATE_SQUARE_BAR:
            break;
        }
}
void MainWindow::StopHWLiveShootTimer()
{
        tmHWTLiveShoot->stop();

        switch(opMode)
        {
        case OM_IDLE: // FILE_SAVING
                tmHWTLiveSave->stop();
                break;
        case OM_FETCH_RAIL_CROWN:
                for (int i = 0; i < C_ParallelProcessPhotos; i++)
                        tmHWTLiveProcess[i]->stop();
                break;
        }
}
void MainWindow::tmHWTLiveCaptureHandler()  //BLOCKING
{
//triggered   #if (!DIRECT_QUEUE)
    static bool isin;
    if (isin)
    {
            qDebug() << "last not over \n" ;
            exit;
    }
    isin = true;

/*        basler grabonepage here
                sPhotoQueues?
            if (opened)  //
            {
XX                    sPhotoes[THREAD_ID][theDevice].pPhotoImage = &sPhotoMem[THREAD_ID][theDevice];
                    sPhotoes[THREAD_ID][theDevice].pTimeStamp = &sTimeStamp[THREAD_ID][theDevice];
XX                    GrabOneImage(theDevice, *sPhotoes[THREAD_ID][theDevice].pPhotoImage , //thdCapture   sPhotoes[theDevice],
                                                    mtxPhoto, HWTriggered, callbackContext);  // thdCapture //thdCapture
            }
        /*std::thread cleft(thdCapture, Camera_Left_0, true);
        std::thread cright(thdCapture, Camera_Right_1, true);
        cleft.join(), cright.join();*/
        //tmHWLiveShoot->stop();
        bool captured[C_MaxCamerasToUse] = {false, false};
        for( ;  mvCallbackContext.TibboOn && fmExposures.IsHardwareTriggered()
                    && (!captured[Camera_Left_0] || !captured[Camera_Right_1]); )
        {
            //for (int repeat = 0; repeat < 2; repeat++)
                for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                        if (!captured[nthDevice])
                                if (        (mvCallbackContext.mvCallbackBuf[nthDevice].frameSize > 0)  //230503
//for alignedBuf?                                         && mvCallbackContext.mvCallbackBuf[nthDevice].uncopied
                                            )
                                {
                                        if ((GrabOneImage(nthDevice,  //tmHWTLiveCaptureHandler
                                                          sPhotoQueues[sPhotoQHead][nthDevice].photoImage,
                                                          mtxPhoto, true, &mvCallbackContext)) == RC_OK)
                                        {
                                                ulong elapsed  = railTimer.nsecsElapsed() - TriggerStart;
                                                elapsed /= ns2ms;
                                                sPhotoQueues[sPhotoQHead][nthDevice].timeStamp  //sPhotoTimestamps[sPhotoQHead][nthDevice]
                                                        =  elapsed;
                                                captured[nthDevice] = true;
                                                    qDebug() << "Image " << nthDevice << " into Q\n";
                                        }
                                } else
                                            captured[nthDevice] = true;  //230503
               QThread::msleep(1);
               QApplication::processEvents();
        }
        if (captured[Camera_Left_0] && captured[Camera_Right_1])
        {
                constexpr uint tmCAPTURED_OFFSET = 5; // * ns2ms;  //5ms   XXXXXXXXXXXXXXXXXXXXXXX
                int offset =  sPhotoQueues[sPhotoQHead][Camera_Left_0].timeStamp - sPhotoQueues[sPhotoQHead][Camera_Right_1].timeStamp;
                        // (sPhotoTimestamps[sPhotoQHead][Camera_Left_0] - sPhotoTimestamps[sPhotoQHead][Camera_Right_1]) ;
                offset = abs(offset);
                        qDebug() << "L-R offset=" << offset << "\n";
                if ( offset  < tmCAPTURED_OFFSET ) //XXXXXXXXXXXXXXXXXX
                {
                        uint next = (sPhotoQHead + 1) % IMAGE_QUEUE_SIZE;
                        {
#ifdef _DEBUG_MUTEX   //LG-7-0
                            qDebug() << "LG-7-0";
#endif
#ifdef LCK_GRD_Q
                                        lock_guard<std::mutex> lgMtxPhotoQ{mtxPhotoQ};
#else
                                        mtxPhotoQ.lock();  //CAPTURED_OFFSET
#endif
                                        qDebug() << "LG-7-1";
                                if ( next != sPhotoQTail) //full
                                        sPhotoQHead = next;

#ifndef LCK_GRD
                                mtxPhotoQ.unlock();
#endif
                        }
                        ulong elapsed  = railTimer.nsecsElapsed() - TriggerStart;
                        elapsed /= ns2ms;
                        String str =  "Head=" + to_string(next) + "@" + to_string(elapsed);  + "\n";
                        //qDebug() << QString::fromStdString(str);

                        if ( next == sPhotoQTail) //full
                                qDebug() << "***** sPhotoQ full ******* \n";
                }
        }
        //tmHWLiveShoot->start(hwLiveShootInterval);

        isin = false;
}
void thdHWTLiveCaptureHandler(  bool *terminated, ETriggerMode *triggerMode, QElapsedTimer *pRailTimer,
                                                                        ulong *pTriggerStart, void *callbackContext)
{
    sCallbackContext *mvCallbackContext = (sCallbackContext *) callbackContext;
    QElapsedTimer &railTimer = *pRailTimer;
    ulong &TriggerStart = *pTriggerStart;
    while (!*terminated)
    {
            bool captured[C_MaxCamerasToUse] = {false, false};
            for( ;  mvCallbackContext->TibboOn && (*triggerMode == TriggerMode_Hardware) //fmExposures.IsHardwareTriggered()
                        && (!captured[Camera_Left_0] || !captured[Camera_Right_1]); )
            {
                    //for (int repeat = 0; repeat < 2; repeat++)
                    for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                            if (!captured[nthDevice])
                                    if (mvCallbackContext->mvCallbackBuf[nthDevice].uncopied)
                                            if ((GrabOneImage(nthDevice,  //thdHWTLiveCaptureHandler
                                                              sPhotoQueues[sPhotoQHead][nthDevice].photoImage,
                                                              mtxPhoto, true, callbackContext)) == RC_OK)
                                            {
                                                    ulong elapsed  = railTimer.nsecsElapsed() - TriggerStart;
                                                    elapsed /= ns2ms;
                                                    sPhotoQueues[sPhotoQHead][nthDevice].timeStamp // sPhotoTimestamps[sPhotoQHead][nthDevice]
                                                                                =  elapsed;
                                                    captured[nthDevice] = true;
                                                        qDebug() << "Image " << nthDevice << " into Q\n";
                                            }
                   QThread::msleep(1);
                   QApplication::processEvents();
            }
            if (captured[Camera_Left_0] && captured[Camera_Right_1])
            {
                    constexpr uint thdCAPTURED_OFFSET = 5 * ns2ms;  //5ms
                    int offset =  sPhotoQueues[sPhotoQHead][Camera_Left_0].timeStamp - sPhotoQueues[sPhotoQHead][Camera_Right_1].timeStamp;
                            // (sPhotoTimestamps[sPhotoQHead][Camera_Left_0] - sPhotoTimestamps[sPhotoQHead][Camera_Right_1]) ;
                    offset = abs(offset);
                            qDebug() << "L-R offset=" << offset << "\n";
                    if ( offset  < thdCAPTURED_OFFSET )
                    {
                            uint next = (sPhotoQHead + 1) % IMAGE_QUEUE_SIZE;
                            {
                                        qDebug() << "LG-8-0";
#ifdef LCK_GRD_Q
                                        lock_guard<std::mutex> lgMtxPhotoQ{mtxPhotoQ};
#else
                                        mtxPhotoQ.lock();  //CAPTURED_OFFSET
#endif
#ifdef _DEBUG_MUTEX   //LG-8-1
                                        qDebug() << "LG-8-1";
#endif
                                    if ( next != sPhotoQTail) //full
                                            sPhotoQHead = next;
#ifndef LCK_GRD
                                    mtxPhotoQ.unlock();
#endif
                            }
                            ulong elapsed  = railTimer.nsecsElapsed() - TriggerStart;
                            elapsed /= ns2ms;
                            String str =  "Head=" + to_string(next) + "@" + to_string(elapsed);  + "\n";
                            //qDebug() << QString::fromStdString(str);

                            if ( next == sPhotoQTail) //full
                                    qDebug() << "***** sPhotoQ full ******* \n";
                    }
            }

            QThread::msleep(5);  //M# assue at least one trigger
    }
}


void MainWindow::onMPCServerConnecting(const uint nthBox)
{
#ifndef _BOX_PLAY_CLIENT
       sktMPC[nthBox] = serverForMPC[nthBox]->nextPendingConnection();
       if (sktMPC[nthBox])
                qDebug() << "\nPanel PC connects to Box-Server" << nthBox+1 << "....\n";
#endif
}

void MainWindow::onMPCServerConnecting0()  {  onMPCServerConnecting(0);  }
void MainWindow::onMPCServerConnecting1()  {  onMPCServerConnecting(1);  }
void MainWindow::onMPCServerConnecting2()  {  onMPCServerConnecting(2);  }
void MainWindow::onMPCServerConnecting3()  {  onMPCServerConnecting(3);  }
void MainWindow::onMPCServerConnecting4()  {  onMPCServerConnecting(4);  }

void MainWindow::onConnectedToMBDServer_VH(const uint nthBox)
{
#ifdef _BOX_PLAY_CLIENT
       //uint nthBox = BOX_ID - 1;
       //sktMBD[nthBox] = serverForMBD[nthBox]->nextPendingConnection();
       if (sktClientMBD_VH)
                qDebug() << "\nHas connected to the Main Box Driver VH....\n";
#endif
}

void MainWindow::onConnectedToMBDServer_CMD(const uint nthBox)
{
#ifdef _BOX_PLAY_CLIENT
       if (sktClientMBD_CMD)
                qDebug() << "\nHas connected to the command port of MBD....\n";
#endif
}


#ifndef _BOX_PLAY_CLIENT
w
void MainWindow::onConnectedToMBDServer_VH0()  {  onConnectedToMBDServer_VH(0);  }
void MainWindow::onConnectedToMBDServer_VH1()  {  onConnectedToMBDServer_VH(1);  }
void MainWindow::onConnectedToMBDServer_VH2()  {  onConnectedToMBDServer_VH(2);  }
void MainWindow::onConnectedToMBDServer_VH3()  {  onConnectedToMBDServer_VH(3);  }
void MainWindow::onConnectedToMBDServer_VH4()  {  onConnectedToMBDServer_VH(4);  }
#endif

void thdTCPHandshake(QTcpSocket  *socket, String IP, uint portNo) //, FSockerHandler receiveHandler)
{
        try {
                socket->connectToHost(QString::fromStdString(IP), portNo);
                 if(!socket->waitForConnected(5))  //5 seconds
                 {
                            qDebug() << "Failed to connect Tibbo!";
                            goto over;
                 }
                 qDebug() << "Connect successfully!";

        over:;
        } catch (int thrown) {
                //execCode = thrown;
        }
        socket->abort();        //Cancel existing connections
        socket->disconnectFromHost(); //Close connection
}

void MainWindow::SimulateTibboOn()
{
        qDebug() << "SML Tibbo ON";
        StartHWLiveShootTimer(5) ;// so slow for single thread?! 0);
        sPhotoQTail  = sPhotoQHead;   //keyPressEvent
        mvCallbackContext.TibboOn = true;  //SimulateTibboOn
}

void MainWindow::tibboStartReceived(uint sender)
{
        mvCallbackContext.TibboOn = true;  //tibboStartReceived
        qDebug() << (sender ? "Tibbo " : "Simulated " ) << "START from Tibbo!\n";

                qDebug() << "LG-9-0";
#ifdef LCK_GRD_Q
                lock_guard<std::mutex> lgMtxPhotoQ{mtxPhotoQ};
#else
                mtxPhotoQ.lock();  //tibboStartReceived
#endif
#ifdef _DEBUG_MUTEX   //LG-9-1
                qDebug() << "LG-9-1";
#endif
        if (!LastRailGap)  //o.w callback first
                LastRailGap = railTimer.nsecsElapsed() - TriggerStart;  //IS834 try
        TriggerStart = railTimer.nsecsElapsed();  //IS834 try
        TriggerCount = 0;  //TriggerCount++  reset
        for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                sPhotoQHead = sPhotoQTail = 0;
#ifndef LCK_GRD
        mtxPhotoQ.unlock();
#endif
        StartHWLiveShootTimer(5) ;// so slow for single thread?! 0);
}
void MainWindow::tibboStopReceived(uint sender)
{
        mvCallbackContext.TibboOn = false;
        ulong start2Stop = (railTimer.nsecsElapsed() - TriggerStart)  / ns2ms;  //IS834 try
        qDebug() << (sender ? "Tibbo " : "Simulated " ) << "STOP from Tibbo after" << start2Stop << "ms!\n";
                qDebug() << "LG-10-0";
#ifdef LCK_GRD_Q
                lock_guard<std::mutex> lgMtxPhotoQ{mtxPhotoQ};
#else
                mtxPhotoQ.lock();  //tibboStopReceived
#endif
#ifdef _DEBUG_MUTEX   //LG-10-1
                qDebug() << "LG-10-1";
#endif
        TriggerStart = railTimer.nsecsElapsed();  //IS834 try
        TriggerCount = -1;    LastRailGap = 0; //TriggerCount++ STOP
        if (fmExposures.FSBeforeASOR != -1)  //ow ignored
        {
                fmExposures.ui->kbSavePhotos->setCheckState((fmExposures.FSBeforeASOR) ? Qt::Checked : Qt::Unchecked);
                //compress SOR_Dir & pack!
                fmExposures.FSBeforeASOR = -1,    fmExposures.SOR_Dir = "";
        }

        if (fmExposures.LFBeforeASOR != -1)  //ow ignored
        {
                fmExposures.ui->kbLoadPhotos->setCheckState((fmExposures.LFBeforeASOR) ? Qt::Checked : Qt::Unchecked);
                fmExposures.LFBeforeASOR = -1;
        }
#ifndef LCK_GRD
        mtxPhotoQ.unlock();
#endif
}

//beacon: 0x25  57 30 30 30 30 26 0x0D 0x0A
void MainWindow::removeHeartbeats()
{
        int hbIdx;
        while (true)
        {
                hbIdx = TibboBuffer.indexOf(0x25); //25 59 33 26 => 25 57 30 30 30 30 26
                int N = TibboBuffer.size();
                if (hbIdx < 0) break;
                if ((N > (hbIdx + HEARTBEAT_SIZE - 1)) && ((uchar)TibboBuffer[hbIdx + HEARTBEAT_SIZE - 1] == 0x0A))
                       TibboBuffer.remove(hbIdx, HEARTBEAT_SIZE);
                else if ((N > (hbIdx + 1)) && ((uchar)TibboBuffer[hbIdx + 1] == 0x59)) //0x57))  //
                                if ((N > (hbIdx + 2)) && ((uchar)TibboBuffer[hbIdx + 2] == 0x33))  //0x30))  //
        //                        if ((N > (hbIdx + 3)) && ((uchar)TibboBuffer[hbIdx + 3] == 0x30))  //
        //                        if ((N > (hbIdx + 4)) && ((uchar)TibboBuffer[hbIdx + 4] == 0x30))  //
        //                        if ((N > (hbIdx + 5)) && ((uchar)TibboBuffer[hbIdx + 5] == 0x30))  //
                                        if ((N > (hbIdx + HEARTBEAT_SIZE - 3)) && ((uchar)TibboBuffer[hbIdx + HEARTBEAT_SIZE - 3] == 0x26)) //3] == 0x26))
                                                if ((N > (hbIdx + HEARTBEAT_SIZE - 2)) && ((uchar)TibboBuffer[hbIdx + HEARTBEAT_SIZE - 2] == 0x0D))  //4] == 0x0D))
                                                        TibboBuffer.remove(hbIdx, HEARTBEAT_SIZE - 1); //5);
                                                else
                                                        TibboBuffer.remove(hbIdx, HEARTBEAT_SIZE - 2); //4);
                                        else
                                                TibboBuffer.remove(hbIdx, HEARTBEAT_SIZE - 3); //3);
        //                        else TibboBuffer.remove(hbIdx, 5);
        //                        else TibboBuffer.remove(hbIdx, 4);
        //                        else TibboBuffer.remove(hbIdx, 3);
                                else TibboBuffer.remove(hbIdx, 2);
                      else
                                TibboBuffer.remove(hbIdx, 1);
        }
}

//beacon: 0x25  0x59  0x33  0x26 0x0D 0x0A
/*void MainWindow::removeHeartbeatsOld()
{
        int hbIdx;
        while (true)
        {
                hbIdx = TibboBuffer.indexOf(0x25);
                if (hbIdx < 0) break;
                if (TibboBuffer[hbIdx + 5] == 0x0A)
                       TibboBuffer.remove(hbIdx, 6);
                else if (TibboBuffer[hbIdx + 1] == 0x59)
                        if (TibboBuffer[hbIdx + 2] == 0x33)
                                if (TibboBuffer[hbIdx + 3] == 0x26)
                                        if (TibboBuffer[hbIdx + 4] == 0x0D)
                                                TibboBuffer.remove(hbIdx, 5);
                                        else
                                                TibboBuffer.remove(hbIdx, 4);
                                else
                                        TibboBuffer.remove(hbIdx, 3);
                        else
                                TibboBuffer.remove(hbIdx, 2);
                else
                        TibboBuffer.remove(hbIdx, 1);
        }
} */

/*
START event:
04 00 00 00 01 00 00 00 3B 00
00 00 2b 00 00 00 09 00 00 00
1C 00 00 00  07 00 00 00 16 00
00 00 00 00 00 00 00 00 00 00
00 00 00 00 AD

STOP event:
04 00 00 00 00 00 00 00 3B 00
00 00 2b 00 00 00 09 00 00 00
1C 00 00 00  07 00 00 00 16 00
00 00 00 00 00 00 00 00 00 00
00 00 00 00 AC
*/
int MainWindow::determinePacket(const uint sumIdx)
{
        ETibboSignal result = Tibbo_None;
        int startIdx = sumIdx - Tibbo_Packet_Size_1;
        if (startIdx >= 0)
                if (        (TibboBuffer[startIdx] != 0x04)
                            // timer || (TibboBuffer[startIdx + 8] != 0x3B)
                            //|| (TibboBuffer[startIdx + 20] != 0x1C)  //12?!
                            //|| (TibboBuffer[startIdx + 28] != 0x16) //01
                    ) ;
                else
                        if ( //((BYTE) TibboBuffer[sumIdx] == StartSum) &&
                             (TibboBuffer[startIdx + 4] == 0x01) )
                                    result = Tibbo_Start;
                        else if ( //((BYTE) TibboBuffer[sumIdx] == StopSum) &&
                                  (TibboBuffer[startIdx + 4] == 0x00) )
                                    result = Tibbo_Stop;
        return result;
}

void MainWindow::tibboSktReadHandler()
{
    //static bool isin;
    //if (isin) return  else  isin = true;
    QByteArray buffer = sktTibbo->readAll();
    const uint buflen = buffer.length();
#ifdef _SHOW_TIBBO_MSG
        if ((buflen != 6))
                if (fmExposures.IsHardwareTriggered())
                        qDebug( ) << "Received" << buflen << "from Tibbo:"  << buffer.toHex(' ');
        //cout( ) << "Tibbo sent " << buflen << " bytes: \n";
        //qDebug( ) << ;
        //DumpHexBytes(buffer.data(), buflen);
#endif

        TibboBuffer.append(buffer);
        /*
        int sumIdx = TibboBuffer.indexOf(StartSum);
        if (sumIdx < 0)
                sumIdx = TibboBuffer.indexOf(StopSum);
        if (sumIdx < 0)
        {
                if (TibboBuffer.size() >= Tibbo_Packet_Size)
                        TibboBuffer.remove(0, TibboBuffer.size() - Tibbo_Packet_Size);
                return;
        } */
        removeHeartbeats();
        int sumIdx = TibboBuffer.size() - 1 ;

#ifdef _SHOW_TIBBO_MSG
        if (TibboBuffer.size() > 0)
                        qDebug( ) << "TibboBuffer = " << TibboBuffer.toHex(' ');
#endif
        switch (determinePacket(sumIdx))
        {
        case  Tibbo_None:       break;
        case Tibbo_Start:       tibboStartReceived(1);      break;  //HW Tibbo
        case Tibbo_Stop:        tibboStopReceived(1);      break;  //HW Tibbo
        case Tibbo_Calibrate_SE:
                                                //AutomaticCalibrateSE();
                            if (opMode != OM_CALIBRATE_SQUARE_BAR)
                            {
                                    switch(opMode)
                                    {
                                    case OM_FETCH_RAIL_CROWN:   on_btFetchRailCrown_clicked(); break;
                                    case OM_CALIBRATE_CHECKERBOARD:   on_btExit_clicked(); break;
                                    }

qDebug() << "btCSB to be called - 2 \n";
                                    on_btCalibrateSquareBar_clicked();  //tibboSktReadHandler  trigger entry
                            }

                            if (opMode == OM_CALIBRATE_SQUARE_BAR)
                            {
                                    //obsolete fmExposures.LFBeforeASOR = (fmExposures.ui->kbLoadPhotos->checkState() == Qt::Checked) ? 1 : 0;
                                    //fmExposures.ui->kbLoadPhotos->setCheckState(Qt::Checked);

                                    fmExposures.FSBeforeASOR = (fmExposures.ui->kbSavePhotos->checkState() == Qt::Checked) ? 1 : 0;
                                    fmExposures.ui->kbSavePhotos->setCheckState(Qt::Checked);

                                    fmExposures.HWTBeforeACSE = (fmExposures.ui->kbHWTrigger->checkState() == Qt::Checked) ? 1 : 0;
                                    fmExposures.ui->kbHWTrigger->setCheckState(Qt::Checked);

                                    on_btCapture_clicked();  //tibboSktReadHandler
                            }
                            break;
        case Tibbo_Save_Round:
                                if (fmExposures.FSBeforeASOR == -1)  //ow ignored
                                {
                                        fmExposures.FSBeforeASOR = (fmExposures.ui->kbSavePhotos->checkState() == Qt::Checked) ? 1 : 0;
                                        fmExposures.ui->kbSavePhotos->setCheckState(Qt::Checked);
                                        fmExposures.SOR_Dir = GetDirStamp(CAPTURE_PATH) + "/";
                                }
                                break;
        }
        TibboBuffer.clear();
        return;

    if (buflen == Tibbo_Packet_Size)
    {
            char *bufData = buffer.data();
            char *pData = bufData ;
            BYTE sum = 0;
            for (int i = 0;  i < Tibbo_Packet_Size_1; i++)
                    sum += *pData++;
            if (sum == (BYTE) *pData)
            {
                    if (bufData[4] == 1)
                    {       //START
                            tibboStartReceived(1);   //HW Tibbo
                            /*TibboOn = true;
                            qDebug() << "START from Tibbo!\n";
                            TriggerStart = railTimer.nsecsElapsed();
                            mtxPhotoQ.lock();  cmt
                            TriggerCount = 0;  //TriggerCount++  reset
                            for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                                    sPhotoQHead = sPhotoQTail = 0;
                            mtxPhotoQ.unlock();*/
                    } else if (bufData[4] == 0) {  //STOP
                            tibboStopReceived(1);
                            /*TibboOn = false;
                            qDebug() << "STOP from Tibbo!\n";
                            mtxPhotoQ.lock();  cmt
                            TriggerCount = -1;  //TriggerCount++ STOP
                            mtxPhotoQ.unlock();*/
                    } else
                            qDebug() << "Unknown commad in Tibbo packet: " <<  bufData[4] << "\n";
            } else
                        qDebug() << "Packet from Tibbo sized " <<  buffer.length() << "\n";
    }
    //isin = false;
}


void MainWindow::tmHouseKeepHandler()
{
        tmHouseKeeper->stop();
        setCaptureButtonCaption(); //

        if (sktTibbo)
                if(sktTibbo->state()==QAbstractSocket::UnconnectedState)
                        try {
                               //qDebug() << "Found Tibbo not connected!";
                                sktTibbo->connectToHost( QString::fromStdString(ipTibbo), TibboPortNumber[BOX_ID-1]);
                                //sktTibbo->connectToHost( QString::fromStdString("192.168.50.101"), 1003);
                                 if(sktTibbo->waitForConnected(5))  //5 seconds
                                        qDebug() << "\t* 👌 Has connected to Tibbo Controller. \n";
                        } catch (int thrown) {
                                //execCode = thrown;
                        }

#ifdef _SIMULATE_ALL_MPC_SEVERS
        for (uint nthBox = 0; nthBox < C_TotalPCs; nthBox++)
        {
#else
        {
            uint nthBox = BOX_ID - 1;
#endif
#ifdef _BOX_PLAY_CLIENT
                bool tryConnect = !sktClientMBD_VH || !sktClientMBD_CMD;
                if (sktClientMBD_VH)
                        tryConnect = (sktClientMBD_VH->state() != QAbstractSocket::ConnectedState)
                                            || (sktClientMBD_CMD->state() != QAbstractSocket::ConnectedState);
                if (tryConnect)
                {
                        TryConnectMBD();
        /**/
                //if (!sktClientMBD_VH->waitForConnected(10))
                        /*
                if (!sktClientMBD_VH->isValid())
                        qDebug() << "Connect to MBD server time-out! \n";
                else
                        qDebug() << "Has connected to Main Box Server. \n";
        /**/
                }
#else
                if (sktMPC[nthBox])
                        if(sktMPC[nthBox]->state()==QAbstractSocket::UnconnectedState)
                        {
                                qDebug() << "Box-Server" << nthBox + 1 << "has lost the connection to MPC...";
                                delete sktMPC[nthBox];
                                sktMPC[nthBox] = nullptr;
                        }
#endif
        }

        //const bool isAutoSEing = IsAutoSECalibrating || IsAutoSEMeasuring;
        if (IsAutoSEing())
        {
                auto past = std::chrono::duration_cast<std::chrono::milliseconds>(high_resolution_clock::now() - AutoSEStart);
                if (past.count() > 3000)
                        ConcludeAutoSE(RC_NO_TRIGGER);
        }

        tmHouseKeeper->start(500); //every second?!
}


void MainWindow::GUIRestoreCalibrate()
{
        StopLiveShootTimer();

        this->ui->btExit->setText("Exit");
        this->ui->btCalibrateCheckerBoard->setText(CalibrateCheckerBoardCaption);
        this->ui->btCalibrateCheckerBoard->setEnabled(true);
        this->ui->btCalibrateSquareBar->setEnabled(true);
        this->ui->btFetchRailCrown->setEnabled(true);
        if (fmExposures.isVisible())
                fmExposures.close();
        mvCallbackContext.OP_Mode = opMode = OM_IDLE;

}

void MainWindow::VoidLoadedRctfMaps()
{
        rectifyMapLoaded = false;
}

void MainWindow::CloseTibboSocket()
{
        if (sktTibbo)
                if(sktTibbo->state() == QAbstractSocket::ConnectedState)
                {
                        sktTibbo->abort();
                        sktTibbo->close();
                        QThread::msleep(100);
                        qDebug() << "Tibbo socket has been closed...";
                }
}

#ifdef _BOX_PLAY_CLIENT
void MainWindow::CloseMBDSocket()
{
        if (sktClientMBD_VH)
                if(sktClientMBD_VH->state() == QAbstractSocket::ConnectedState)
                {
                        sktClientMBD_VH->abort();
                        sktClientMBD_VH->close();
                        QThread::msleep(100);
                        qDebug() << "MBD socket has been closed...";
                }
        if (sktClientMBD_CMD)
                if(sktClientMBD_CMD->state() == QAbstractSocket::ConnectedState)
                {
                        sktClientMBD_CMD->abort();
                        sktClientMBD_CMD->close();
                        QThread::msleep(100);
                        qDebug() << "MBD socket has been closed...";
                }
}
#endif

bool MainWindow::SvrSktForMPCStartListening(const uint nthBox)
{
#ifndef _BOX_PLAY_CLIENT
        if (!serverForMPC[nthBox])  return false;

        if (serverForMPC[nthBox]->isListening())
                serverForMPC[nthBox]->close();
        QHostAddress ipPanelPC(BOX_IP); //Box_IPs[nthBox]); //BOX_IP); //"192.168.50.25");
        if (!serverForMPC[nthBox]->listen(QHostAddress::AnyIPv4,
                                                                    MainAPPortNumber[nthBox]  //10002 //10050 //)) //10002)
                                        ))
        {
                qDebug() << "\t* 😱 Box-Server" << nthBox + 1 << "for MPC failed to start:" << serverForMPC[nthBox]->errorString() << "\n\n";
                return false;
        } else {
                qDebug() << "\t* 👋 Box-Server" << nthBox + 1 << "for MPC starts listening to port" << MainAPPortNumber[nthBox] << "...\n";
                return true;
        }
#endif
}

bool MainWindow::SvrSktForMBDStartListening(const uint nthBox)
{
#ifndef _BOX_PLAY_CLIENT
        if (!serverForMBD[nthBox])  return false;

        if (serverForMBD[nthBox]->isListening())
                serverForMBD[nthBox]->close();
        //QHostAddress ipPanelPC(Box_IPs[nthBox]); //BOX_IP); //"192.168.50.25");
        if (!serverForMBD[nthBox]->listen(QHostAddress::AnyIPv4,
                                                                    MainBDPortNumber[nthBox]  //10002 //10050 //)) //10002)
                                        ))
        {
                qDebug() << "\t* 😱 Box-Server" << nthBox + 1 << "for MBD failed to start:" << serverForMPC[nthBox]->errorString() << "\n\n";
                return false;
        } else {
                qDebug() << "\t* 👋 Box-Server" << nthBox + 1 << "for MBD starts listening to port" << MainAPPortNumber[nthBox] << "...\n";
                return true;
        }
#endif
}

MainWindow::MainWindow(QWidget *parent)
        : QMainWindow(parent) , ui(new Ui::MainWindow)
{
        //fmExposures.setParent(this);
        fmExposures.pMainForm = this;

        ui->setupUi(this);
        ui->statusbar = new QStatusBar(this);

        setFixedSize(width(), height());
        setWindowFlags( Qt::Window
                                        | Qt::CustomizeWindowHint
                                        | Qt::WindowTitleHint
                                        //| Qt::WindowSystemMenuHint
                                        //| Qt::WindowMinimizeButtonHint
                                        //| Qt::WindowCloseButtonHint
                        );
        //setCaptureButtonCaption(); //ui->btCapture->setText("&Preview");
#ifdef _MISSION_BS
        ui->btFetchRailCrown->setText("Fetch V && H\nof Rail Head");
#elif _MISSION_XS
        ui->btFetchRailCrown->setText("Fetch\nPlate Contour");
#endif


        WorkingDir = QDir::currentPath().toStdString() + "/";
        CAPTURE_PATH = WorkingDir + CAPTURE_SUBDIR + "/";
        QString strDir = QString::fromStdString(CAPTURE_PATH); //"./" + CAPTURE_SUBDIR);
        if (!QDir(strDir).exists())
                QDir().mkdir(strDir);
#ifdef _APPLY_FIXED_DIRXX
        #ifdef SG_BOX_RDXX
                //220911
        //WorkingDir = "/home/user/Code/SG/SG_Box/data/box4/KB";
         #else
                //WorkingDir = "C:/Users/jeffrey/RunStraightServer/";
        #endif
#endif
        opMode = OM_IDLE;  //MUST precede
        LoadSystemParameters();

        pnTopPhoto[0] = ui->lbLeftImage,  pnTopPhoto[1] = ui->lbRightImage;
        pnMidPhoto[0] = ui->lbLeftImage_2,  pnMidPhoto[1] = ui->lbRightImage_2;
        pnFused3DImage[1] = ui->lbFused3DImage;
        for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
        {
                pnTopPhoto[nthDevice]->setScaledContents(true);
                pnMidPhoto[nthDevice]->setScaledContents(true);
        }
        pnFused3DImage[1]->setScaledContents(true);

                //bLiveCapturing = false;
        tmLiveShootShowBoth = new QTimer(this);
        connect(    tmLiveShootShowBoth,
                            SIGNAL(timeout()),
                            this,
                            SLOT(tmSWLiveCaptureHandler()));
        tmHWTLiveShoot = new QTimer(this);
        connect(    tmHWTLiveShoot,
                            SIGNAL(timeout()),
                            this,
                            SLOT(tmHWTLiveCaptureHandler()));

        for (int i = 0; i < C_ParallelProcessPhotos; i++)
        {
                tmHWTLiveProcess[i] = new QTimer(this);
#ifdef ROI_IMAGE
                for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                {
                        sDummyFullImage[i][nthDevice] = Mat::zeros( CAMERA_PHOTO_RECT.height,
                                                                                                                    CAMERA_PHOTO_RECT.width, //photo->cols,
                                                                                                                    CV_8UC1);
                        if ( !sDummyFullImage[i][nthDevice].isContinuous() )
                                sDummyFullImage[i][nthDevice]= sDummyFullImage[i][nthDevice].clone();
                }
#endif
        }
        TuneRotatedPhotoDim();
        connect(tmHWTLiveProcess[0], SIGNAL(timeout()), this, SLOT(tmHWTProcessHandler0()));
        if (C_ParallelProcessPhotos >= 2)
                connect(tmHWTLiveProcess[1], SIGNAL(timeout()), this, SLOT(tmHWTProcessHandler1()));
        if (C_ParallelProcessPhotos >= 3)
                connect(tmHWTLiveProcess[2], SIGNAL(timeout()), this, SLOT(tmHWTProcessHandler2()));
        if (C_ParallelProcessPhotos >= 4)
                connect(tmHWTLiveProcess[3], SIGNAL(timeout()), this, SLOT(tmHWTProcessHandler3()));
        if (C_ParallelProcessPhotos >= 5)
                connect(tmHWTLiveProcess[4], SIGNAL(timeout()), this, SLOT(tmHWTProcessHandler4()));
        if (C_ParallelProcessPhotos >= 6)
                connect(tmHWTLiveProcess[5], SIGNAL(timeout()), this, SLOT(tmHWTProcessHandler5()));
        if (C_ParallelProcessPhotos >= 7)
                connect(tmHWTLiveProcess[6], SIGNAL(timeout()), this, SLOT(tmHWTProcessHandler6()));
        if (C_ParallelProcessPhotos >= 8)
                connect(tmHWTLiveProcess[7], SIGNAL(timeout()), this, SLOT(tmHWTProcessHandler7()));

        tmHWTLiveSave = new QTimer(this);
        connect(tmHWTLiveSave, SIGNAL(timeout()), this, SLOT(HWTSaveFileHandler()));

        tmHWTSECapture = new QTimer(this);
        connect(tmHWTSECapture , SIGNAL(timeout()), this, SLOT(HWTSECaptureHandler()));
    /*tmHardwareTrigger = new QTimer(this);
    connect(    tmHardware






Trigger,
                        SIGNAL(timeout()),
                        this,
                        SLOT(tmHWTriggerHandler()));*/

        tmHouseKeeper = new QTimer(this);
        connect(    tmHouseKeeper,
                            SIGNAL(timeout()),
                            this,
                            SLOT(tmHouseKeepHandler()));
        tmHouseKeeper->start(1000); //every second?!

        sktTibbo = new QTcpSocket();
        QObject::connect( sktTibbo,
                                            &QTcpSocket::readyRead,
                                            this,
                                            &MainWindow::tibboSktReadHandler );
        //thTibbo = std::thread(thdTCPHandshake, socket, tibboIP, tibboPort); //, FSockerHandler(&MainWindow::tibboSktReadHandler));

#ifdef _SIMULATE_ALL_MPC_SEVERS
#else
#endif
#ifdef _SIMULATE_ALL_MPC_SEVERS
        for (int i = 0; i < C_TotalPCs; i++)
                serverForMPC[i] = new QTcpServer(this), serverForMBD[i] = new QTcpServer(this);

        connect( serverForMPC[0], SIGNAL(newConnection()), this, SLOT(onMPCServerConnecting0()));
        connect( serverForMPC[1], SIGNAL(newConnection()), this, SLOT(onMPCServerConnecting1()));
        connect( serverForMPC[2], SIGNAL(newConnection()), this, SLOT(onMPCServerConnecting2()));
        connect( serverForMPC[3], SIGNAL(newConnection()), this, SLOT(onMPCServerConnecting3()));
        connect( serverForMPC[4], SIGNAL(newConnection()), this, SLOT(onMPCServerConnecting4()));

        connect( serverForMBD[0], SIGNAL(newConnection()), this, SLOT(onConnectedToMBDServer_VH0()));
        connect( serverForMBD[1], SIGNAL(newConnection()), this, SLOT(onConnectedToMBDServer_VH1()));
        connect( serverForMBD[2], SIGNAL(newConnection()), this, SLOT(onConnectedToMBDServer_VH2()));
        connect( serverForMBD[3], SIGNAL(newConnection()), this, SLOT(onConnectedToMBDServer_VH3()));
        connect( serverForMBD[4], SIGNAL(newConnection()), this, SLOT(onConnectedToMBDServer_VH4()));

        for (int i = 0; i < C_TotalPCs; i++)
                SvrSktForMPCStartListening(i),  SvrSktForMBDStartListening(i);

#else
#ifdef _BOX_PLAY_CLIENT
        sktClientMBD_VH /*serverForMBD[BOX_ID - 1]*/ = new QTcpSocket();
        sktClientMBD_CMD = new QTcpSocket();
        // connect( sktClientMBD_VH , SIGNAL(newConnection()), this, SLOT(onConnectedToMBDServer_VH()));
#else
        serverForMPC[BOX_ID - 1] = new QTcpServer(this);
        // whenever a user connects, it will emit signal
        //connect( serverForMPC[BOX_ID - 1], SIGNAL(newConnection()),
         //                       this, SLOT(onMPCServerConnecting()));
        switch(BOX_ID)
        {
        case 1:   connect( serverForMPC[0], SIGNAL(newConnection()), this, SLOT(onMPCServerConnecting0()));
                        //connect( serverForMBD[0], SIGNAL(newConnection()), this, SLOT(onConnectedToMBDServer_VH0()));
                        break;
        case 2:   connect( serverForMPC[1], SIGNAL(newConnection()), this, SLOT(onMPCServerConnecting1()));
                        //connect( serverForMBD[1], SIGNAL(newConnection()), this, SLOT(onConnectedToMBDServer_VH1()));
                        break;
        case 3:   connect( serverForMPC[2], SIGNAL(newConnection()), this, SLOT(onMPCServerConnecting2()));
                        //connect( serverForMBD[2], SIGNAL(newConnection()), this, SLOT(onConnectedToMBDServer_VH2()));
                        break;
        case 4:   connect( serverForMPC[3], SIGNAL(newConnection()), this, SLOT(onMPCServerConnecting3()));
                        //connect( serverForMBD[3], SIGNAL(newConnection()), this, SLOT(onConnectedToMBDServer_VH3()));
                        break;
        case 5:   connect( serverForMPC[4], SIGNAL(newConnection()), this, SLOT(onMPCServerConnecting4()));
                        //connect( serverForMBD[4], SIGNAL(newConnection()), this, SLOT(onConnectedToMBDServer_VH4()));
                        break;
        }

        SvrSktForMPCStartListening(BOX_ID-1);
#endif
#endif
        /*
        //QHostAddress ipPanelPC(ipPanelPC ) ;// taipei "192.168.0.140");
        QHostAddress ipPanelPC(Box_IPs[BOX_ID-1]); //BOX_IP); //"192.168.50.25");
XX         if (!serverForMPC->listen(QHostAddress::AnyIPv4,
                                                    PanelPCPortNumber[BOX_ID-1]  //10002 //10050 //)) //10002)
                //*    ipPanelPC , //QHostAddress::AnyIPv4,  //
              //                                              PanelPCPortNumber[BOX_ID-1] // 10060
            //                                            )) //portPanelPC)) // MPCPortNumber[BOX_ID]))

                                                ))
        {
XX                qDebug() << "\t* Server for MPC failed to start: " << serverForMPC->errorString() << "\n\n";
        } else
XX                qDebug() << "\t* Server for MPC starts listening to port " << PanelPCPortNumber[BOX_ID-1] << "...\n";
 */
        CameraBrand = CameraBrands(Camera_Unknown);
        mvCallbackContext.Box_ID = BOX_ID;
        mvCallbackContext.OP_Mode = opMode;
        mvCallbackContext.TibboOn = false;

        mvCallbackContext.pPhotoQueues = &sPhotoQueues[0][0];
        mvCallbackContext.pPhotoQHead = &sPhotoQHead;
        mvCallbackContext.pPhotoQTail = &sPhotoQTail;
        mvCallbackContext.pRailTimer = &railTimer;
        mvCallbackContext.pTriggerStart = &TriggerStart;
        mvCallbackContext.pLastRailGap = &LastRailGap;
        mvCallbackContext.pPhotoQMtx = &mtxPhotoQ;
        if (InitializeCameras(&mvCallbackContext/*&sPhotoes[0], this*/) != RC_OK)
                QApplication::quit();//throw execCode ;
        mvCallbackContext.cameraBrand = CameraBrand;
        mvCallbackContext.cameraExposures[Camera_Left_0] = CameraExposures[fmExposures.ui->cbMode->currentIndex()].first;
        mvCallbackContext.cameraExposures[Camera_Right_1] = CameraExposures[fmExposures.ui->cbMode->currentIndex()].second;
        //@@@HWT
#if defined(HWT_CP_THD) && (!DIRECT_QUEUE)
        thHWTLiveShoot = std::thread(thdHWTLiveCaptureHandler,
                                                    &isClosing,
                                                    &eTriggerMode,
                                                    &railTimer,
                                                    &TriggerStart,
                                                    &mvCallbackContext);
#endif

}

MainWindow::~MainWindow()
{
        isClosing = true;
        if (thHWTLiveShoot.joinable())
            thHWTLiveShoot.join();

        if (thTibbo.joinable())
                thTibbo.join();
        if (sktTibbo)
        {
                sktTibbo->abort();        //Cancel existing connections
                sktTibbo->disconnectFromHost(); //Close connection
                delete sktTibbo;
        }
#ifdef _BOX_PLAY_CLIENT
        if (sktClientMBD_VH)
        {
                if (sktClientMBD_VH->state() == QAbstractSocket::ConnectedState)
                        sktClientMBD_VH->close();
                delete sktClientMBD_VH;
        }
        if (sktClientMBD_CMD)
        {
                if (sktClientMBD_CMD->state() == QAbstractSocket::ConnectedState)
                        sktClientMBD_CMD->close();
                delete sktClientMBD_CMD;
        }
#else
        if (sktMPC[BOX_ID - 1])
        {
                sktMPC[BOX_ID - 1]->abort();        //Cancel existing connections
                delete sktMPC[BOX_ID - 1];
        }
#endif
        FinalizeCameras(&mvCallbackContext);
        for (int idx = 0; idx < IMAGE_QUEUE_SIZE; idx++)
                for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                {
                        if (!sPhotoQueues[idx][nthDevice].photoImage.empty())
                                sPhotoQueues[idx][nthDevice].photoImage.release();  //123
                }
        for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
        {
                for (int idx = 0; idx < 2; idx ++)
                        sStereoMap[nthDevice][idx].release();
                sReverseMapMask[nthDevice].release();
                if (!sProjectionMatrices_P[nthDevice].empty())
                        sProjectionMatrices_P[nthDevice].release();
        }
        delete tmLiveShootShowBoth;
        delete tmHWTLiveShoot;
        for (int i = 0; i < C_ParallelProcessPhotos; i++)
        {
                delete tmHWTLiveProcess[i];
                sWorkCS_XY[i].release();
#ifdef ROI_IMAGE
                for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                {
                        sDummyFullImage[i][nthDevice].release();
                        sRctfMidlines[i][nthDevice].release();
                        sWorkRctfML[i][nthDevice].release();
                }
#endif
        }
        delete tmHWTLiveSave;
        delete tmHWTSECapture;
        delete tmHouseKeeper;
        delete ui->statusbar;
        //delete tmHardwareTrigger;
        delete ui;
}

void MainWindow::TuneRotatedPhotoDim()
{
#ifdef ROI_IMAGE   //TuneRotatedPhotoDim
        if (    ( fmExposures.areCameras90DgrRotated && (sDummyFullImage[0][0].cols > sDummyFullImage[0][0].rows))
              || (!fmExposures.areCameras90DgrRotated &&  (sDummyFullImage[0][0].cols < sDummyFullImage[0][0].rows))  )
                for (int i = 0; i < C_ParallelProcessPhotos; i++)
                        for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                                sDummyFullImage[i][nthDevice] = sDummyFullImage[i][nthDevice].t();

        if (! fmExposures.areCameras90DgrRotated )
        {
                CUR_LASER_ROI_WIDTH = RAW_LASER_ROI_WIDTH;
                CUR_LASER_ROI_HEIGHT = RAW_LASER_ROI_HEIGHT;

                for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                        CUR_LASER_ROI[BOX_ID-1][nthDevice ] = RAW_LASER_ROI[BOX_ID-1][nthDevice ];
        } else {
                CUR_LASER_ROI_WIDTH = RAW_LASER_ROI_HEIGHT;
                CUR_LASER_ROI_HEIGHT = RAW_LASER_ROI_WIDTH;
                //for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                {
                        const uint nthBox = BOX_ID - 1;
                        CUR_LASER_ROI[nthBox][Camera_Left_0].y = RAW_LASER_ROI[nthBox][Camera_Left_0].x;
                        CUR_LASER_ROI[nthBox][Camera_Left_0].x = sDummyFullImage[THREAD_0][Camera_Left_0].cols - 1
                                                                                                        - RAW_LASER_ROI[nthBox][Camera_Left_0].y
                                                                                                        - RAW_LASER_ROI_HEIGHT;

                        CUR_LASER_ROI[nthBox][Camera_Right_1].x = RAW_LASER_ROI[nthBox][Camera_Right_1].y;
                        CUR_LASER_ROI[nthBox][Camera_Right_1].y = sDummyFullImage[THREAD_0][Camera_Right_1].rows - 1
                                                                                                        - RAW_LASER_ROI[nthBox][Camera_Right_1].x
                                                                                                        - RAW_LASER_ROI_WIDTH;
                        /*
                         if (nthBox == 3)  //230206
                        {
                                CUR_LASER_ROI[nthBox][Camera_Right_1].y -= 250; //266; //%%%%
                                if (CUR_LASER_ROI[nthBox][Camera_Right_1].y < 0)
                                        CUR_LASER_ROI[nthBox][Camera_Right_1].y = 0;
                        } /**/
                }
        }
#endif

}


int MainWindow::LoadSystemParameters()
 {
       int rc = RC_FAIL_TO_OPEN;
     try {
         QFileInfo check_file (QString::fromStdString(WorkingDir + "SGBoxParams.ini"));
        if (check_file.exists())
        {
                FileStorage FS(WorkingDir + "SGBoxParams.ini", FileStorage::READ);
                if (FS.isOpened())
                {
                        double r;
                        /*FS["IdleLeftExp"] >> r;
                        if (r > 0)
                            CameraExposures[OM_IDLE].first;*/
                        int n;
                        FileNode fn = FS["BOX_ID"];
                        if (!fn.empty())
                        {
                                FS["BOX_ID"] >> n,  BOX_ID = n; //& rectidied roi
                                BOX_IP = "192.168.50.2" + to_string(BOX_ID);  //230208
                        }
#ifdef _CALIB_AT_NEIHU
                        if (BOX_ID != 5)
                                cout << "WARNING: The calibration at NeiHu applies only to Box-5! \n\n" ;
#endif
                        FS["IdleLeftExp"] >> CameraExposures[OM_IDLE].first;
                        FS["IdleRightExp"] >> CameraExposures[OM_IDLE].second;
                        FS["CalibKBLeftExp"] >> CameraExposures[OM_CALIBRATE_CHECKERBOARD].first;
                        FS["CalibKBRightExp"] >> CameraExposures[OM_CALIBRATE_CHECKERBOARD].second;
                        FS["CalibSBLeftExp"] >> CameraExposures[OM_CALIBRATE_SQUARE_BAR].first;
                        FS["CalibSBRightExp"] >> CameraExposures[OM_CALIBRATE_SQUARE_BAR].second;
                        FS["FetchVHLeftExp"] >> CameraExposures[OM_FETCH_RAIL_CROWN].first;
                        FS["FetchVHRightExp"] >> CameraExposures[OM_FETCH_RAIL_CROWN].second;

                        FS["IdleTriggerMode"] >> TriggerModes[OM_IDLE];
                        //FS["CalibKBTriggerMode"] >> TriggerModes[OM_CALIBRATE_CHECKERBOARD];
                        TriggerModes[OM_CALIBRATE_CHECKERBOARD] = TriggerMode_Software;
                        FS["CalibSETriggerMode"] >> TriggerModes[OM_CALIBRATE_SQUARE_BAR];
                        FS["FetchVHTriggerMode"] >> TriggerModes[OM_FETCH_RAIL_CROWN];

                        FS["IdleSaveCaptured"] >> SaveCaptured[OM_IDLE];
                        FS["CalibKBSaveCaptured"] >> SaveCaptured[OM_CALIBRATE_CHECKERBOARD];
                        FS["CalibSESaveCaptured"] >> SaveCaptured[OM_CALIBRATE_SQUARE_BAR];
                        FS["FetchVHSaveCaptured"] >> SaveCaptured[OM_FETCH_RAIL_CROWN];

                        FS["IdleLoadCaptured"] >> LoadCaptured[OM_IDLE];
                        FS["CalibKBLoadCaptured"] >> LoadCaptured[OM_CALIBRATE_CHECKERBOARD];
                        FS["CalibSELoadCaptured"] >> LoadCaptured[OM_CALIBRATE_SQUARE_BAR];
                        FS["FetchVHLoadCaptured"] >> LoadCaptured[OM_FETCH_RAIL_CROWN];

                        FS["CamerasRotatedBy90Degree"] >> fmExposures.areCameras90DgrRotated;
#if defined(_MISSION_BS)
                        if (!fmExposures.areCameras90DgrRotated)
                        {
                                qDebug() << "Cameras forced to rotate 90 degree in the BS case\n";
                                fmExposures.areCameras90DgrRotated = true;
                        }
#elif defined(_MISSION_XS)
                        if (fmExposures.areCameras90DgrRotated)
                        {
                                qDebug() << "Cameras no need to rotate 90 degree in the XS case\n";
                                fmExposures.areCameras90DgrRotated = false;
                        }
#endif

                        FS["CheckerboardHorzBlackGridNum"] >> n;
                        if (n > 0) fmExposures.KBGrids.horzBlackGrdNum /* .gridColNum*/ = n;

                        FS["CheckerboardVertBlackGridNum"] >> n;
                        if (n > 0) fmExposures.KBGrids.vertBlackGrdNum /*gridRowNum*/ = n;

                        FS["CheckerboardGridArrayWidth"] >>  r;
                        if (r > 0) fmExposures.KBGrids.mmGridArrayWidth /* .mmGridWidth*/ = r;

                        FS["CheckerboardGridArrayHeight"] >> r;
                        if (r > 0) fmExposures.KBGrids.mmGridArrayHeight = r;

                        FS["SquareBarCrossWidth"] >> r;
                        if (r > 0)  fmExposures.SBWidth = r;

                        FS["SquareBarCrossLength"] >> r;
                        if (r > 0)  fmExposures.SBLength = r;

                        //BS-specific
#ifdef _MISSION_BS
                        FS["VFT"] >> r;  if (r > 0) fmExposures.VFineTune = r;
                        FS["HFT"] >> r;  if (r > 0) fmExposures.HFineTune = r;
                        FS["RFT"] >> r;  if (r) fmExposures.SERFineTune = r;
                        FS["YFT"] >> r;  if (r) fmExposures.SEVOffset = r;
                        FS["XFT"] >> r;  if (r) fmExposures.SEHOffset = r;

                        FS["VHVerticalOffset"] >> r;
                        if (r > 0) fmExposures.VHVertOffset = r;

                        FS["BarSectEstimateTolerance"] >> r;  if (r > 0) fmExposures.BarSectEstTrn = r;

                        FS["RctSERoiLeftX"] >> n; if  (n > 0) ROI_SE_LEFT_X[BOX_ID-1] = n; //[OM_CALIBRATE_SQUARE_BAR] = n;
                        //FS["RctVHRoiLeftX"] >> n; if  (n > 0)  fmExposures.ROI_LEFT_X[BOX_ID-1][OM_FETCH_RAIL_CROWN] = n;
                        FS["RctSERoiRightX"] >> n;  if  (n > 0) ROI_SE_RIGHT_X[BOX_ID-1] = n;
                        //FS["RctVHRoiRightX"] >> n;  if  (n > 0) fmExposures.ROI_RIGHT_X[BOX_ID-1][OM_FETCH_RAIL_CROWN] = n;
                        FS["RctSERoiSharedTop"] >> n;  if  (n > 0) ROI_SE_Y_TOP[BOX_ID-1] = n;
                        //FS["RctVHRoiSharedTop"] >> n;  if  (n > 0) fmExposures.ROI_Y_TOP[BOX_ID-1][OM_FETCH_RAIL_CROWN] = n;
                        FS["RctSERoiSharedWidth"] >> n;  if  (n > 0) ROI_SE_WIDTH = n; //[OM_CALIBRATE_SQUARE_BAR] = n;
                        //FS["RctVHRoiSharedWidth"] >> n;  if  (n > 0) fmExposures.ROI_WIDTH[OM_FETCH_RAIL_CROWN] = n;
                        FS["RctSERoiSharedHeight"] >> n;  if  (n > 0) ROI_SE_HEIGHT = n;
                        //FS["RctVHRoiSharedHeight"] >> n;  if  (n > 0) fmExposures.ROI_HEIGHT[OM_FETCH_RAIL_CROWN] = n;
#endif

                        //XS
#ifdef _MISSION_XS
                        FS["GvMFrom4BCC"] >> r;
                        if (r > 0)  fmExposures.GvMSwingFrom4BCC = r;
                        FS["GvMTo4BCC"] >> r;
                        if (r > 0)  fmExposures.GvMSwingTo4BCC = r;
                        FS["GvMSteps4BCC"] >> r;
                        if (r > 0)  fmExposures.GvMSwingSteps4BCC = r;
                        FS["GvMFromPlate"] >> r;
                        if (r > 0)  fmExposures.GvMSwingFromPlate = r;
                        FS["GvMToPlate"] >> r;
                        if (r > 0)  fmExposures.GvMSwingToPlate = r;
                        FS["GvMStepsPlate"] >> r;
                        if (r > 0)  fmExposures.GvMSwingStepsPlate = r;
#endif

                        FS["ReprojectErrorTolerance"] >> r;  if (r > 0) fmExposures.ReprojErrTrn = r;

                        FS.release();

                        //saved the swapped ?
                        //fmExposures.OnRotate90CheckBoxChanged(fmExposures.areCameras90DgrRotated);
                        rc = RC_OK;
                }
        }
        if (rc != RC_OK)
        {
                fmExposures.SetInitializedGUIToVariables(true); // fmExposures.OnRotate90CheckBoxChanged(fmExposures.areCameras90DgrRotated);
        }
        fmExposures.KBGrids.horzTotalGrdNum = fmExposures.KBGrids.horzBlackGrdNum * 2 - 2; ///*CB_INT_CNR_COL
        fmExposures.KBGrids.vertTotalGrdNum = fmExposures.KBGrids.vertBlackGrdNum * 2 - 2;  //CB_INT_CNR_ROW
        fmExposures.KBGrids.mmOneGridWidth = fmExposures.KBGrids.mmGridArrayWidth / (fmExposures.KBGrids.horzTotalGrdNum/* gridColNum */ + 1); //GRID_DIM_X
        fmExposures.KBGrids.mmOneGridHeight = fmExposures.KBGrids.mmGridArrayHeight / (fmExposures.KBGrids.vertTotalGrdNum /*.gridRowNum*/ + 1); //GRID_DIM_Y
        fmExposures.SetupBoxID(BOX_ID);
     } catch (const GenericException& e) {
            rc = RC_FAIL_TO_LOAD;
     }
     return rc;
}
int MainWindow::SaveSystemParameters()
 {
     try {
            FileStorage FS(WorkingDir + "SGBoxParams.ini", FileStorage::WRITE);
            if (FS.isOpened())
            {
                    int n = BOX_ID;
                    FS << "BOX_ID" <<  n;
                    FS << "IdleLeftExp" << CameraExposures[OM_IDLE].first;
                    FS << "IdleRightExp" << CameraExposures[OM_IDLE].second;
                    FS << "CalibKBLeftExp" << CameraExposures[OM_CALIBRATE_CHECKERBOARD].first;
                    FS << "CalibKBRightExp" << CameraExposures[OM_CALIBRATE_CHECKERBOARD].second;
                    FS << "CalibSBLeftExp" << CameraExposures[OM_CALIBRATE_SQUARE_BAR].first;
                    FS << "CalibSBRightExp" << CameraExposures[OM_CALIBRATE_SQUARE_BAR].second;
                    FS << "FetchVHLeftExp" << CameraExposures[OM_FETCH_RAIL_CROWN].first;
                    FS << "FetchVHRightExp" << CameraExposures[OM_FETCH_RAIL_CROWN].second;

                    FS << "IdleTriggerMode" << TriggerModes[OM_IDLE];
                    //FS << "CalibKBTriggerMode" << TriggerModes[OM_CALIBRATE_CHECKERBOARD];


                    FS << "CalibSETriggerMode" << TriggerModes[OM_CALIBRATE_SQUARE_BAR];
                    FS << "FetchVHTriggerMode" << TriggerModes[OM_FETCH_RAIL_CROWN];

                    FS << "IdleSaveCaptured" << SaveCaptured[OM_IDLE];
                    FS << "CalibKBSaveCaptured" << SaveCaptured[OM_CALIBRATE_CHECKERBOARD];
                    FS << "CalibSESaveCaptured" << SaveCaptured[OM_CALIBRATE_SQUARE_BAR];
                    FS << "FetchVHSaveCaptured" << SaveCaptured[OM_FETCH_RAIL_CROWN];

                    FS << "IdleLoadCaptured" << LoadCaptured[OM_IDLE];
                    FS << "CalibKBLoadCaptured" << LoadCaptured[OM_CALIBRATE_CHECKERBOARD];
                    FS << "CalibSELoadCaptured" << LoadCaptured[OM_CALIBRATE_SQUARE_BAR];
                    FS << "FetchVHLoadCaptured" << LoadCaptured[OM_FETCH_RAIL_CROWN];

                    FS << "CamerasRotatedBy90Degree" << fmExposures.areCameras90DgrRotated;

                    n = fmExposures.KBGrids.horzBlackGrdNum; // .gridColNum;
                    FS << "CheckerboardHorzBlackGridNum" << n;
                    n = fmExposures.KBGrids.vertBlackGrdNum; // .gridRowNum;
                    FS << "CheckerboardVertBlackGridNum" << n;
                    FS << "CheckerboardGridArrayWidth" << fmExposures.KBGrids.mmGridArrayWidth;
                    FS << "CheckerboardGridArrayHeight" << fmExposures.KBGrids.mmGridArrayHeight;
                    FS << "SquareBarCrossWidth" << fmExposures.SBWidth;
                    FS << "SquareBarCrossLength" << fmExposures.SBLength;

                    //BS-specific
#ifdef _MISSION_BS
                    //if (fmExposures.VFineTune != 1.0)
                            FS << "VFT" << fmExposures.VFineTune;
                    //if (fmExposures.HFineTune != 1.0)
                            FS << "HFT" << fmExposures.HFineTune;
                            FS << "RFT" << fmExposures.SERFineTune;
                            FS << "YFT" << fmExposures.SEVOffset;
                            FS << "XFT" << fmExposures.SEHOffset;
                    FS << "VHVerticalOffset" << fmExposures.VHVertOffset;
/* not save for now
                    FS << "RctSERoiLeftX" << ROI_SE_LEFT_X[BOX_ID-1]; //[OM_CALIBRATE_SQUARE_BAR];
                    //FS << "RctVHRoiLeftX" << fmExposures.

                    //ROI_LEFT_X[BOX_ID-1][OM_FETCH_RAIL_CROWN];
                    FS << "RctSERoiRightX" << ROI_SE_RIGHT_X[BOX_ID-1];
                    //FS << "RctVHRoiRightX" << fmExposures.ROI_RIGHT_X[BOX_ID-1][OM_FETCH_RAIL_CROWN];
                    FS << "RctSERoiSharedTop" << ROI_SE_Y_TOP[BOX_ID-1];
                    //FS << "RctVHRoiSharedTop" << fmExposures.ROI_Y_TOP[BOX_ID-1][OM_FETCH_RAIL_CROWN];
                    FS << "RctSERoiSharedWidth" << ROI_SE_WIDTH; //[OM_CALIBRATE_SQUARE_BAR];
                    //FS << "RctVHRoiSharedWidth" << fmExposures.ROI_WIDTH[OM_FETCH_RAIL_CROWN];
                    FS << "RctSERoiSharedHeight" << ROI_SE_HEIGHT;
                    //FS << "RctVHRoiSharedHeight" << fmExposures.ROI_HEIGHT[OM_FETCH_RAIL_CROWN];
/**/

                    FS << "BarSectEstimateTolerance" << fmExposures.BarSectEstTrn;
#endif
                    //XS
#ifdef _MISSION_XS
                    FS << "GvMFrom4BCC" << fmExposures.GvMSwingFrom4BCC;
                    FS << "GvMToBCC" << fmExposures.GvMSwingTo4BCC;
                    FS << "GvMSteps4BCC" << fmExposures.GvMSwingSteps4BCC;
                    FS << "GvMFromPlate" << fmExposures.GvMSwingFromPlate;
                    FS << "GvMToBCC" << fmExposures.GvMSwingToPlate;
                    FS << "GvMStepsPlate" << fmExposures.GvMSwingStepsPlate;
#endif

                    FS << "ReprojectErrorTolerance" << fmExposures.ReprojErrTrn;

                    FS.release();
                    return RC_OK;
            }
     } catch (const GenericException& e) {
     }
     return RC_FAIL_TO_WRITE;
}

void MainWindow::keyPressEvent(QKeyEvent* e)
{
        if(e->key() == Qt::Key_F7)
        {
                SimulateTibboOn();
/*                    qDebug() << "SML Tibbo ON";
XX                StartHWLiveShootTimer(5) ;// so slow for single thread?! 0);
                sPhotoQTail  = sPhotoQHead;   //keyPressEvent
                mvCallbackContext.TibboOn = true; */
        } else if(e->key() == Qt::Key_F6)
        {
                    qDebug() << "SML Tibbo OFF";
            //finish the rest StopHWLiveShootTimer() ;
            mvCallbackContext.TibboOn = false;
        } else if(e->key() == Qt::Key_Escape)      //if(e->key()==Qt::Key_Enter)
             on_btExit_clicked(); //QCoreApplication::quit();
        else
            QWidget::keyPressEvent(e);
}

void MainWindow::closeEvent (QCloseEvent *event)
{
        //if (!IsGUIAP )
        QMessageBox::StandardButton resBtn = QMessageBox::Yes;
        if (!NO_GUI_MODE)
                resBtn = QMessageBox::question( this, "Confirm to close...",
                                                                    tr("Are you sure to leave?\n"),
                                                                    QMessageBox::No | QMessageBox::Yes,
                                                                    QMessageBox::Yes);

        if (resBtn == QMessageBox::Yes)
        {
                event->accept();
                fmExposures.close();
        } else
                event->ignore();
}

bool MainWindow::TryConnectMBD()
{
#ifdef _BOX_PLAY_CLIENT
        bool result = false;
        if (sktClientMBD_VH)
            try {
                    //if (sktClientMBD_VH->state() == QAbstractSocket::ConnectedState)  return RC_OK;
                    sktClientMBD_VH->abort();
                    //const string BOX_IP = "192.168.50.2" + to_string(BOX_ID);
                    //const string NEIHU_LOCAL_IP = "192.168.0.140";  //157
                    sktClientMBD_VH->connectToHost(
                                                                        #ifdef _TEST_LOCALHOST
                                                                                    QHostAddress::LocalHost // NEIHU_LOCAL_IP.c_str()  //"192.168.0.157"  //QHostAddress::LocalHost//"127.0.0.1" //"192.168.0.157"
                                                                        #else
                                                                                    ipPanelPC.c_str()
                                                                        #endif
                                                                        , 10000 + (BOX_ID) * 111
                                                                        );

                    if(!sktClientMBD_VH->waitForConnected(1000))
                    {
                             //qDebug() << "Box" << BOX_ID+1 << " connects MBD time-out!";
                             result = false;
                    } else  {
                            qDebug() << "\t* 👋 " << "Has connected to the VH port of the MBD server.\n";
                            result = true;
                    }
            } catch (const GenericException& e) {
            }

        if (!result)
                return result;
        if (sktClientMBD_CMD)
            try {
                    //if (sktClientMBD_CMD->state() == QAbstractSocket::ConnectedState)  return RC_OK;
                    sktClientMBD_CMD->abort();
                    //const string BOX_IP = "192.168.50.2" + to_string(BOX_ID);
                    //const string NEIHU_LOCAL_IP = "192.168.0.140";  //157
                    sktClientMBD_CMD->connectToHost(
                                                                        #ifdef _TEST_LOCALHOST
                                                                                    QHostAddress::LocalHost // NEIHU_LOCAL_IP.c_str()  //"192.168.0.157"  //QHostAddress::LocalHost//"127.0.0.1" //"192.168.0.157"
                                                                        #else
                                                                                    ipPanelPC.c_str()
                                                                        #endif
                                                                        , 10001 + (BOX_ID) * 111
                                                                        );

                    if(!sktClientMBD_CMD->waitForConnected(1000))
                    {
                             //qDebug() << "Box" << BOX_ID+1 << " connects MBD time-out!";
                             result = false;
                    } else  {
                            qDebug() << "\t* 👋 " << "Has connected to the command port of the MBD server.\n";
                            QObject::connect( sktClientMBD_CMD, &QTcpSocket::readyRead, this, &MainWindow::ReadMBDServerStr_CMD);
                            result = true;
                    }
            } catch (const GenericException& e) {
            }
        return result;
        /*
                    //if (!sktClientMBD_VH->waitForConnected(10))
                    //if (!sktClientMBD_VH->isValid())
                    if (true)
                    {
                            //qDebug()
                        cout << "Connect to MBD server time-out! \n";
                             //return false;
                    } else  {
                            //qDebug()
                        cout << "Has connected to Main Box Server. \n";
                            //return true;
                    }
        */
        //qDebug()<<"FINE HERE!!";
#endif
}

void MainWindow::ReadMBDServerStr_CMD()
{
        QByteArray buffer = sktClientMBD_CMD->readAll();
        if (!buffer.isEmpty())
        {
                QString str(buffer.data());
                str = str.split("\n")[0];
                qDebug() << "MBD Command:" << str;
                QStringList cmd = str.split(",");
                if (cmd[0] == "SE")
                {
                        if (cmd.size() < 2)
                        {
                                sktClientMBD_CMD->write( ("SE, " + QString::number(RC_INVALID_DATA)+ "\n").toStdString().c_str() );  //Erro code =
                                //sktClientMBD_CMD->write("SE, " needs a second argument: 1 for calibrate, 2 for measure.\n");
                                return;
                        }
                        int purpose = cmd[1].toInt();
                        switch(purpose)
                        {
                        case -1:      IsAbortingAutoSE = true;
                                            qDebug() << "MBD notifies to abort...";
                                            break;
                        case 1:       StartAutoSE(IsAutoSEMeasuring);
                                            qDebug() << "MBD asks to measure...";
                                            break;
                        case 2:       StartAutoSE(IsAutoSECalibrating);
                                            qDebug() << "MBD asks to calibrate...";
                                            break;
                        }
                        orgModeBeforeACSE = opMode;
                        if (!IsAbortingAutoSE)
                                switch(opMode)
                                {
                                case OM_FETCH_RAIL_CROWN:
                                    qDebug() << "btCSB to be called - 3 \n";
                                                                        on_btFetchRailCrown_clicked();
                                                                        QThread::msleep(10);
                                case OM_IDLE:
                                                                        on_btCalibrateSquareBar_clicked();  //ReadMBDServerStr_CMD  trigger, entry
                                                                        QThread::msleep(10);

                                case OM_CALIBRATE_SQUARE_BAR:
                                                                        if (opMode == OM_CALIBRATE_SQUARE_BAR)
                                                                        {
                                                                                fmExposures.LFBeforeASOR = (fmExposures.ui->kbLoadPhotos->checkState() == Qt::Checked) ? 1 : 0;
                                                                                if (fmExposures.ui->kbLoadPhotos->checkState() == Qt::Checked)
                                                                                {
                                                                                        fmExposures.ui->kbLoadPhotos->setCheckState(Qt::Unchecked);
                                                                                        fmExposures.on_kbLoadPhotos_clicked(fmExposures.ui->kbLoadPhotos->checkState());
                                                                                }

                                                                                fmExposures.HWTBeforeACSE = (fmExposures.ui->kbHWTrigger->checkState() == Qt::Checked) ? 1 : 0;
                                                                        //* changed to SW triggering 230528 !?
                                                                        #ifdef DIRECT_QUEUE
                                                                                if (fmExposures.ui->kbHWTrigger->checkState() == Qt::Unchecked)
                                                                                {
                                                                                        fmExposures.ui->kbHWTrigger->setCheckState(Qt::Checked);
                                                                                }
                                                                        #else
                                                                                if (fmExposures.ui->kbHWTrigger->checkState() == Qt::Checked)
                                                                                {
                                                                                        fmExposures.ui->kbHWTrigger->setCheckState(Qt::Unchecked);
                                                                                }
                                                                        #endif
                                                                                fmExposures.on_kbHWTrigger_clicked(fmExposures.ui->kbHWTrigger->checkState());
                                                                    //#ifdef _SAVE_TO_MONITOR
                                                                                fmExposures.FSBeforeASOR = (fmExposures.ui->kbSavePhotos->checkState() == Qt::Checked) ? 1 : 0;
                                                                                if (fmExposures.ui->kbSavePhotos->checkState() != Qt::Checked)
                                                                                {
                                                                                        fmExposures.ui->kbSavePhotos->setCheckState(Qt::Checked);
                                                                                        fmExposures.on_kbSavePhotos_clicked(fmExposures.ui->kbSavePhotos->checkState());
                                                                                }
                                                                    //#endif
                                                                                on_btCapture_clicked();  //ReadMBDServerStr_CMD

    //qDebug() << "set TbOn";
                                                                                mvCallbackContext.TibboOn = true;
                                                                        }
                                                                        break;
                                case OM_CALIBRATE_CHECKERBOARD:
                                                                        sktClientMBD_CMD->write( ("SE, " + QString::number(RC_IS_BUSY)+ "\n").toStdString().c_str() );  //Erro code =
                                                                        //sktClientMBD_CMD->write("SE, is executing checkerboard calibration.\n");
                                                                        break;
                                }
                }
                else if (cmd[0] == "RT")
                {
                        if (cmd.size() < 2)
                        {
                                sktClientMBD_CMD->write("RT, needs a second argument for rail type.\n");
                                return;
                        }
                }
        }

}


void MainWindow::on_btCameras_clicked()
{
        //QString pylonExeFile = "C:/Program Files/Basler/pylon 6/Applications/x64/bin/pylonviewer.exe";
        //QProcess::startDetached(pylonExeFile , QStringList() << "any_argument");
        //fmExposures.setParent(this);

        fmExposures.SetupMode(opMode); //230319 moved to each mode

        //fmExposures.RefreshParameterGUIs(); //230319 moved to

        fmExposures.show();
}

int MainWindow::OnBothFilesLoaded()
{
        for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
        {
                Rect roi(RAW_LASER_ROI[BOX_ID - 1][nthCamera ], Size(RAW_LASER_ROI_WIDTH, RAW_LASER_ROI_HEIGHT)); //imread before rotate
                        roi.height = min(roi.height, sPhotoMem[THREAD_0][nthCamera].rows - roi.y),
                        roi.width = min(roi.width, sPhotoMem[THREAD_0][nthCamera].cols - roi.x);
                if ((roi.height <= 0) || (roi.width <= 0))
                        return RC_BAD_IMAGE;
                sPhotoMem[THREAD_0][nthCamera] = sPhotoMem[THREAD_0][nthCamera](roi);

                sPhotoes[THREAD_0][nthCamera ].pPhotoImage = &sPhotoMem[THREAD_0][nthCamera];
                sPhotoes[THREAD_0][nthCamera ].pTimeStamp = &sTimeStamp[THREAD_0][nthCamera ];
        }
        return RC_OK;
}

void MainWindow::setCaptureButtonCaption()
{
        QString caption;
        if (tmLiveShootShowBoth->isActive())
                caption = "&Freeze";
        else if  (fmExposures.willLoadFromFiles())
                caption = "&Load";
        else
                switch(opMode)
                {
                case OM_IDLE:
                case OM_CALIBRATE_CHECKERBOARD:
                                                    caption = "&Preview";   break;
                case OM_CALIBRATE_SQUARE_BAR:
                        #ifdef _MISSION_BS
                                                    caption = "&Capture";
                        #endif
                        #ifdef _MISSION_XS
                                                    caption = "&Scan";
                        #endif
                                                    break;
                case OM_FETCH_RAIL_CROWN:
                                                    caption = "&Capture";   break;
                }
        ui->btCapture->setText(caption);
}
void MainWindow::StartLiveShootTimer(uint msInterval)
{
        if (!tmLiveShootShowBoth->isActive())
                tmLiveShootShowBoth->start(msInterval);

        /*switch(opMode)
        {
        case OM_IDLE:
        case OM_CALIBRATE_CHECKERBOARD:
                                    ui->btCapture->setText("&Freeze");
                                    break;
        case OM_CALIBRATE_SQUARE_BAR:
        case OM_FETCH_RAIL_CROWN:
                                    ui->btCapture->setText("&Capture");
                                    break;
        }*/
}
void MainWindow::StopLiveShootTimer()
{
        if (tmLiveShootShowBoth->isActive())
                tmLiveShootShowBoth->stop();
        /*switch(opMode)
        {
        case OM_IDLE:
        case OM_CALIBRATE_CHECKERBOARD:
                                    ui->btCapture->setText("&Preview");
                                    break;
        case OM_CALIBRATE_SQUARE_BAR:
        case OM_FETCH_RAIL_CROWN:
                                    ui->btCapture->setText("&Capture");
                                    break;
        }*/
}
void MainWindow::tmSWLiveCaptureHandler()
{
        const int interval = tmLiveShootShowBoth->interval();
    //CGrabResultPtr pGrabbedResult;
    //bool res = BaslerGrabOne(0, pGrabbedResult);
            /*
             * CGrabResultPtr pGrabbedResult;
                try {
                            cameras[0].StartGrabbing(1);
                            cameras[0].RetrieveResult//GrabOne
                                    ( MAX_EXPOSURE_ms, pGrabbedResult, TimeoutHandling_ThrowException);
               } catch (const GenericException& e) {
                        cerr << "An exception 12-1 occurred." << endl << e.GetDescription() << endl;
                }
    /**/
        /*while (!mtxPhotoQ.try_lock())
        {
                qDebug() << "SWT Tmr FoL";
                QThread::msleep(10);
                QApplication::processEvents();
        }  */
        tmLiveShootShowBoth->stop();
        switch(opMode)
        {
        case OM_CALIBRATE_SQUARE_BAR:
                        CaptureProcessDual(TriggerMode_Software, WILL_PROCESS, WILL_DISPLAY, THREAD_0, true);  //on_btCapture_clicked
                        //ui->btCapture->setText("&Capture");
                        break;
        default:
                        CaptureProcessDual(TriggerMode_Software, CAPTURE_ONLY, WILL_DISPLAY);  //tmSWLiveCaptureHandler
                        tmLiveShootShowBoth->start(interval);
                        break;
        }
        //mtxPhotoQ.unlock(); //cndtVrbl.notify_all(); //

}

void MainWindow::on_btCapture_clicked()
{
        if (fmExposures.IsHardwareTriggered())
        {
                if (opMode == OM_CALIBRATE_SQUARE_BAR)
                {
#ifdef _MISSION_XS
                        nsGalvanometer::SendAnglesToGalvanometer();
#endif
        #ifdef _MISSION_BS
                        if (!IsAutoSEing())
                        {
                                QMessageBox::warning(this, "Aborted", "Due to the limit of Tibbo, Boxes cannot trigger laser. \n\nUse S/W Triggering.");
                                return;
                         }
                        tmHWTSECapture->start(1);
                        //* notify Tibbo to send HWT
#ifdef TIBBO_CLIENT_NOT_LIMIT
                        char CMD_TRIGGER_ON[12] = { 0x25, 0x43, 0x4C, 0x42, 0x31, 0x26, 0x0D };
                        char CMD_TRIGGER_OFF[12] = { 0x25, 0x43, 0x4C, 0x42, 0x30, 0x26, 0x0D };
                        sktTibbo->write((char *) CMD_TRIGGER_ON, 6);
                        if (sktTibbo->waitForBytesWritten())
                        {
                                QThread::msleep(100);  //M# assue at least one trigger
                                sktTibbo->write((char *) CMD_TRIGGER_OFF, 6);
                                if (!sktTibbo->waitForBytesWritten())
                                        tmHWTSECapture->stop();
                        }
#endif
#endif
                } else
                        QMessageBox::information(this, "Cancelled", "Cannot manually trigger cameras while they are set hardware-triggered.");
                return;
        }
        if  (fmExposures.willLoadFromFiles())
        {
            static QString lastLoadDir = QString::fromStdString(DEBUG_PATH);
#ifdef _APPLY_FIXED_DIR
            switch(opMode)
            {
            case OM_CALIBRATE_SQUARE_BAR:
                                        #ifdef SG_BOX_RD
                                                lastLoadDir  = "/home/user/Code/SG/SG_Box/data/box" + QString::fromStdString(to_string(BOX_ID)) +"/SE/";
                                        #else
                                                lastLoadDir  = "/home/user/downloads/centralize/captured/calib/SE";
                                        #endif
                                        break;
            case OM_FETCH_RAIL_CROWN:
                                        #ifdef SG_BOX_RD
                                                #ifdef _CALIB_AT_NEIHU
                                                        lastLoadDir  = "/home/user/Code/SG/SG_Box/execute/VH/";
                                                #else
                                                        lastLoadDir  = "/home/user/Code/SG/SG_Box/data/box4/VH/20ms/";
                                                        lastLoadDir  = QString::fromStdString("/home/user/Code/SG/SG_Box/data/box" + to_string(BOX_ID) + "/VH/");
                                                 #endif
                                        #else
                                                lastLoadDir  = "/home/user/downloads/centralize/captured/calib/";
                                        #endif
                                        break;
            default:         break;
            }
#endif
            QString filename = QFileDialog::getOpenFileName(this,
                                                             "Select the file of the LEFT camera",
                                                            lastLoadDir ,
                                                             "Bmp files (*.bmp)"); //Text files (*.txt);; All files (*.*)");
            QFileInfo check_file (filename);
           if (!check_file.exists())  return;
            sPhotoMem[THREAD_0][Camera_Left_0] = imread(filename.toStdString(), IMREAD_GRAYSCALE);
            if (sPhotoMem[THREAD_0][Camera_Left_0].empty())  return;

            QFileInfo qfile(filename);
            lastLoadDir = qfile.absoluteFilePath();
            filename = QFileDialog::getOpenFileName(this,
                                                             "Select the file of the RIGHT camera",
                                                            lastLoadDir, // QString::fromStdString(DEBUG_PATH),
                                                             "Bmp files (*.bmp)"); //Text files (*.txt);; All files (*.*)");
            check_file.setFile(filename);
           if (!check_file.exists())  return;
            //sPhotoes[Camera_Right_1] = imread(filename.toStdString(), IMREAD_GRAYSCALE);
            sPhotoMem[THREAD_0][Camera_Right_1] = imread(filename.toStdString(), IMREAD_GRAYSCALE);
            if (sPhotoMem[THREAD_0][Camera_Right_1].empty())  return;

            if (OnBothFilesLoaded() != RC_OK)
                    return;

            CaptureProcessDual(TriggerMode_Files, WILL_PROCESS,   //on_btCapture_clicked
                   #ifdef _DEBUG
                                                        WILL_DISPLAY
                   #else
                                                        NO_DISPLAY
                   #endif
                                                    );  //on_btCapture_clicked

            if (fmExposures.IsHardwareTriggered())
                    if (opMode == OM_CALIBRATE_SQUARE_BAR)
                    {
                            //cnt restore for hwt LFBeforeASOR
                            if (fmExposures.FSBeforeASOR != -1)  //ow ignored
                            {
                                    fmExposures.ui->kbSavePhotos->setCheckState((fmExposures.FSBeforeASOR) ? Qt::Checked : Qt::Unchecked);
                                    fmExposures.FSBeforeASOR = -1;
                            }
                            if (fmExposures.HWTBeforeACSE != -1)  //ow ignored
                            {
                                    fmExposures.ui->kbHWTrigger->setCheckState((fmExposures.HWTBeforeACSE) ? Qt::Checked : Qt::Unchecked);
                                    fmExposures.HWTBeforeACSE = -1;
                            }
                    }
            return;
        }
        ///* in case MV adopt callback mechanism for software-triggering
        //CameraSdkStatus status = CameraSoftTriggerEx(mvCamerahandles[0], CAMERA_ST_CLEAR_BUFFER_BEFORE );  /**/
        //CameraSoftTriggerEx(mvCamerahandles[1], CAMERA_ST_CLEAR_BUFFER_BEFORE );  /**/
        //CameraPlay(mvCamerahandles[0]);
        /*StopLiveShootTimer();
XX        int rc = CaptureProcessDual(TriggerMode_Software, WILL_PROCESS, WILL_DISPLAY, THREAD_0, true);  //on_btCapture_clicked
        if (rc != RC_OK)
                QMessageBox::warning(this, "Replied", QString::fromStdString(to_string(rc)) );
*/
        int lag ;
        switch(opMode)
        {
        case OM_IDLE:
        case OM_CALIBRATE_CHECKERBOARD:
                                    lag= 50;  break;
        case OM_CALIBRATE_SQUARE_BAR:
        case OM_FETCH_RAIL_CROWN:
                                    lag= 0;  break;
        }
                                    if (tmLiveShootShowBoth->isActive())
                                            StopLiveShootTimer();
                                    else
                                            StartLiveShootTimer(lag); //on_btCapture_clicked
        return;
                                    //break;
                                    StopLiveShootTimer();
                                    int rc = CaptureProcessDual(TriggerMode_Software, WILL_PROCESS, WILL_DISPLAY, THREAD_0, true);  //on_btCapture_clicked
                                    if (rc != RC_OK)
                                            QMessageBox::warning(this, "Replied", QString::fromStdString(to_string(rc)) );
                                    StartLiveShootTimer(50); //tmLiveShootShowBoth->start(50);
                                    //break;

        //}

}

void MainWindow::on_btExit_clicked()
{
        switch(opMode)
        {
        case OM_CALIBRATE_CHECKERBOARD:
                                    GUIRestoreCalibrate();
                                    break;
        //case OM_FETCH_RAIL_CROWN: break;
        default:         close();
        }
}

void MainWindow::OnCalibrateDirChanged()
 {
         //why?? WorkingDir =  CalibrateDir;  //220820
         QString qdir = QString::fromStdString(CalibrateDir + "last/");
         if (!QDir(qdir ).exists())   //calibDir + "/last"
                 QDir().mkdir(qdir );
        //#ifdef _DEBUG  220914
         DEBUG_PATH = CalibrateDir + "debug/";
         qdir = QString::fromStdString(DEBUG_PATH);
         if (!QDir(qdir).exists())   //calibDir + "/debug"
                QDir().mkdir(qdir);
        //#endif
}
void MainWindow::showCrop(int nthCamera, int x0, int y0)  //_DEBUG
{
        constexpr int W = 6;
        constexpr int H = 2;
        bool dimShown = false;
        for (int i = X_0; i <= Y_1; i++)
        {
                if (!sStereoMap[nthCamera][i].data)
                        continue;

                if (!dimShown)
                {
#ifdef _DISPLAY_LOADED
                        cout << "SM W=" << sStereoMap[nthCamera][i].cols <<  ", H=" << sStereoMap[nthCamera][i].rows << endl;
#endif
                        dimShown = true;
                }
                int x = max(x0, 0), y = max(y0, 0);
                x = min(x, sStereoMap[nthCamera][i].cols - 1 - W),
                y = min(y, sStereoMap[nthCamera][i].rows - 1 - H);

                const Rect rectCrop (x, y, W, H);
                Mat submap = sStereoMap[nthCamera][i](rectCrop);                                        //uchar *p = stereoMap[nthCamera][i].ptr();
#ifdef _DISPLAY_LOADED
                cout << "SM_" + GetCameraChar(nthCamera) + "[" + to_string(i) + "]="
                            << submap << endl; ;
#endif
        }
}

bool MainWindow::GetCalibrateDir()
{
        bool dirExists = false; // = (CalibrateDir != "") && QDir(QString::fromStdString(CalibrateDir)).exists();
#ifdef _APPLY_FIXED_DIR
        #ifdef SG_BOX_RD
            //debug  fixed KB
                //WorkDirBox6  = "/home/user/Code/SG/SG_Box/data/box" + to_string(BOX_ID) + "/KB/"; //2/KB/"; //
                #ifdef _CALIB_AT_NEIHU
                        CalibrateDir = "/home/user/Code/SG/SG_Box/execute/";
                #else
                        CalibrateDir = "/home/user/Code/SG/SG_Box/data/box" + to_string(BOX_ID) + "/KB/";
                                            //WorkDirBox6; //"/home/user/Code/SG/SG_Box/data/box" + to_string(BOX_ID) + "/KB/";
                #endif
        #else
                CalibrateDir  = WorkDirBox1_5; //"/home/user/downloads/centralize/calib/";
        #endif
        #ifdef OS_WINDOWS   //DESKTOP_WIN
            CalibrateDir  = "C:/Users/jeffrey/RunStraightServer/data/box4/KB/";
        #endif
#endif
        dirExists = (CalibrateDir != "") && QDir(QString::fromStdString(CalibrateDir)).exists();

        #ifdef _DEBUG2  //220914
        if (! dirExists )
        #ifdef OS_LINUX
xx                    CalibrateDir  = "/home/user/Code/SG/SG_Box/execute/";
        #endif
        dirExists = (CalibrateDir != "") && QDir(QString::fromStdString(CalibrateDir)).exists();
    #endif
        if (! dirExists ) // DoesDirExist(CalibrateDir))
        {
                QString  calibDir = QFileDialog::getExistingDirectory(this,
                                                                            "Choose the folder for last calibration",
                                                                             QString::fromStdString(WorkingDir + "calib/"),
                                                                            QFileDialog::ShowDirsOnly);
                if (calibDir.isEmpty())
                        return false;  //goto over;
                CalibrateDir =  calibDir.toStdString()+ "/";
        }
        return true;

}

void MainWindow::on_btCalibrateCheckerBoard_clicked()
{
        if (tmLiveShootShowBoth->isActive())
                StopLiveShootTimer();
        if (fmExposures.IsHardwareTriggered())
        {
                QMessageBox::information(this, "Rejected", "Cannot calibrate while cameras are hardware-triggered, \nplease tune to software-triggered in the parameters dialog first.");
                return;
        }

        const String OriginalWorkingDir = WorkingDir;
        if (this->ui->btCalibrateCheckerBoard->text() == CalibrateCheckerBoardCaption )
        {
                if (!GetCalibrateDir())
                        return;
                OnCalibrateDirChanged();

                this->ui->btCalibrateCheckerBoard->setText("Execute");
                this->ui->btCalibrateSquareBar->setEnabled(false);
                this->ui->btFetchRailCrown->setEnabled(false);
                this->ui->btExit->setText("Stop");

                if (fmExposures.isVisible())
                        fmExposures.close();
                mvCallbackContext.OP_Mode = opMode = OM_CALIBRATE_CHECKERBOARD;
                fmExposures.SetupMode(opMode); // 230319

                vector<cv::String> imageFilenames ;
                cv::glob(CalibrateDir , imageFilenames , false);
                const uint pairNum = imageFilenames .size() / 2;  //01L/01R  02L/02R
                PhotoPairStartNo = 1;
                if (pairNum > 0)
                {
                        QMessageBox::StandardButton reply =
                                QMessageBox::question(
                                                    this, "Overwrite or append",
                                                       "There already are " + QString::fromStdString(to_string(pairNum)) + " pairs of images in, do you want to overwrite or append? Click Yes to overwrite or No to append.",
                                                       QMessageBox::Yes|QMessageBox::No);

                         if (reply == QMessageBox::No) {
                                PhotoPairStartNo = pairNum + 1;
                               //qDebug() << "Yes was clicked";
                               //QApplication::quit();
                         }
                }
                //constexpr uint CALIB_KB_EXPOSURE = 525000; //ambient light 0707  385000; 0621
                TuneCameraExposure(Camera_Left_0, CameraExposures[OM_CALIBRATE_CHECKERBOARD].first * 1000 );
                TuneCameraExposure(Camera_Right_1, CameraExposures[OM_CALIBRATE_CHECKERBOARD].second * 1000);

                StartLiveShootTimer(50); //tmLiveShootShowBoth->start(50);
                //bLiveCapturing = true;
                //std::thread liveCapture(thdLiveShootShowBoth);

        } else if (this->ui->btCalibrateCheckerBoard->text() == "Execute")
        {
                tmLiveShootShowBoth->stop(); // bLiveCapturing = false;  //stop the live-capturing
                this->ui->btCalibrateCheckerBoard->setEnabled(false);

                vector<cv::String> imageFilenames ;
                cv::glob(CalibrateDir , imageFilenames , false);
                uint pairNum = imageFilenames .size() / 2;  //01L/01R  02L/02R
                Size unifiedImageSize(0, 0);

                // Creating vector to store vectors of 3D points for each checkerboard image
                vector<vector<cv::Point3f> > aryPhyCorners; //shared by both
                aryPhyCorners.clear();
                // Creating vector to store vectors of 2D points for each checkerboard image
                vector<vector<cv::Point2f> > aryImgCorners[2];
                 aryImgCorners[0].clear(),  aryImgCorners[1].clear();

                Mat intrinsicMatrices[2]; // = { Mat(3, 3, CV_32FC1), Mat(3, 3, CV_32FC1) };
                Mat distortMatrices[2];
                distortMatrices[0].setTo(0),  distortMatrices[1].setTo(0);
                Rect validPxlROI[2];
                double reprojError[2];
                //TBI: check if Answer NO files are there first!  CalibrateDir + "/last/Calibrate_" + CAMERA_CHAR
                //for (int nthPair = 0; nthPair < pairNum; nthPair++)  //if, say 2 pairs, shoule be 1L / 1R / 2L / 2R
               switch (QMessageBox::question(this, "Calibrate single or load",
                                       "Calibrate singles or load the calibrated? Click Yes to calibrate or No to load.",
                                       QMessageBox::Yes| QMessageBox::No | QMessageBox::Cancel ) )
               {
               case QMessageBox::Cancel:
                                                                WorkingDir = OriginalWorkingDir, GUIRestoreCalibrate();
                                                                return; //goto over;
                case QMessageBox::Yes:
                        if (!pairNum)
                        {
                                QMessageBox::warning(this, "Aborted", "There are no photo pairs to calibrate.");
                                this->ui->btCalibrateCheckerBoard->setEnabled(true);
                                return;
                        } else
                                for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
                                        if (CalibrateOneCamera(fmExposures.KBGrids,
                                                                        nthCamera,
                                                                        CalibrateDir,  // + "/" + to_string(nthPair + 1) + ((nthCamera == (int) Camera_Left_0) ? "L" : "R") +  ".bmp",
                                                                        pairNum,
                                                                        fmExposures.areCameras90DgrRotated,
                                                                        unifiedImageSize,
                                                                        aryPhyCorners,
                                                                        aryImgCorners[nthCamera],
                                                                        intrinsicMatrices[nthCamera], //rawImages[nthCamera]
                                                                        distortMatrices[nthCamera],
                                                                        validPxlROI[nthCamera],
                                                                        reprojError[nthCamera]
                                                                        ) == RC_OK)
                                        {
                                                if (reprojError[nthCamera] > fmExposures.ReprojErrTrn)
                                                        qDebug() << "*** reprojError too large!\n";
                                                QMessageBox::information(this, QString::fromStdString(GetCameraStr(nthCamera) + " Camera"), QString::fromStdString(GetCameraStr(nthCamera) + " e=") + QString::number(reprojError[nthCamera]));
                                                //220720 cancel if (reprojError[nthCamera] < 2)
                                                {
                                                        FileStorage FS(CalibrateDir + "last/Calibrate_" + GetCameraChar(nthCamera), FileStorage::WRITE);
                                                        if (FS.isOpened() )
                                                        {
                                                                FS << "p" << to_string(pairNum);
                                                                FS << "e" << reprojError[nthCamera];
                                                                FS << "c" << to_string(aryImgCorners[nthCamera].at(0).size());
                                                                FS << "uw" << unifiedImageSize.width  << "uh" <<  unifiedImageSize.height;
                                                                FS << "x" << validPxlROI[nthCamera].x << "y" << validPxlROI[nthCamera].y;
                                                                FS << "rw" << validPxlROI[nthCamera].width << "rh" << validPxlROI[nthCamera].height;
                                                                FS.release();
                                                        }
                                                        std::ofstream OF(CalibrateDir + "last/Corners_" + GetCameraChar(nthCamera));
                                                        for (uint nthPair = 0; nthPair < pairNum; nthPair++)
                                                                for (const auto &e : aryImgCorners[nthCamera].at(nthPair))
                                                                        OF << e.x << " " << e.y << " ";
                                                        OF.close();
                                                        SaveMatBinary(intrinsicMatrices[nthCamera],
                                                                            CalibrateDir + "last/CameraMatrix_" + GetCameraChar(nthCamera));
                                                        SaveMatBinary(distortMatrices[nthCamera],
                                                                            CalibrateDir + "last/DistortMatrix_" + GetCameraChar(nthCamera));
                                                        /**/
                                                }
                                        } else {
                                                WorkingDir = OriginalWorkingDir, GUIRestoreCalibrate();
                                                return; //goto over;
                                        }
                        break; //} else
               case QMessageBox::No:
                        for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
                        {
                                   uint cornerNum;
                                   string token;
                                   Point2f coord;
                                   vector<Point2f> imgCorners;
                                   const string CAMERA_CHAR = GetCameraChar(nthCamera);
                                   FileStorage FS(CalibrateDir + "last/Calibrate_" + CAMERA_CHAR, FileStorage::READ);
                                   if (FS.isOpened())
                                   {
                                            FS["p"] >> token;  pairNum = stoi(token);
                                            FS["e"] >> reprojError[nthCamera];
                                            FS["x"] >> validPxlROI[nthCamera].x;
                                            FS["y"] >> validPxlROI[nthCamera].y;
                                            FS["rw"] >> validPxlROI[nthCamera].width;
                                            FS["rh"] >> validPxlROI[nthCamera].height;
                                            FS["uw"] >> unifiedImageSize.width;
                                            FS["uh"] >> unifiedImageSize.height;
                                            FS["c"] >> token;  cornerNum = stoi(token);
                                    }

                                   if (nthCamera == Camera_Left_0)
                                   {
                                           vector<Point3f> phyCorners;

                                           GeneratePhysicalCorners(phyCorners, fmExposures.KBGrids);
                                           for (uint i = 0; i < pairNum; i++)
                                                    aryPhyCorners.push_back(phyCorners);
                                    }
                                   std::ifstream IF(CalibrateDir + "last/Corners_" + CAMERA_CHAR);
                                   for (uint nthPair = 0; nthPair < pairNum; nthPair++)
                                   {
                                            imgCorners.clear();
                                            for (uint i = 0; i < cornerNum; i++)
                                            {
                                                    IF >> token;
                                                    if (!token.empty())
                                                            coord.x = stoi(token);
                                                    IF >> token;
                                                    if (!token.empty())
                                                            coord.y = stoi(token);
                                                    imgCorners.push_back(coord);
                                            }
                                            aryImgCorners[nthCamera].push_back(imgCorners);
                                   }
                                   IF.close();

                                   LoadMatBinary(CalibrateDir + "last/CameraMatrix_" + CAMERA_CHAR, intrinsicMatrices[nthCamera]);
                                   LoadMatBinary(CalibrateDir + "last/DistortMatrix_" +CAMERA_CHAR, distortMatrices[nthCamera]);
                        }
                        break;
                } //switch

               for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
               {
                        cout << GetCameraStr( nthCamera) << ":" << endl;
                        cout << "e=" << reprojError[nthCamera] << endl;
                        cout << "cameraMatrix=\n" << intrinsicMatrices[nthCamera] << endl;
                        cout << "distCoeffs=" << distortMatrices[nthCamera] << endl;
                        cout << "validPxlROI=" << validPxlROI[nthCamera] << endl;
               }

                        /* This function finds the intrinsic parameters for each of the two cameras and
                         * the extrinsic parameters between the two cameras.
                          */
                        Mat rotationMatrix_R; //Output rotation matrix between the 1st and the 2nd camera coordinate systems.
                        Mat translationVector_T; //Output translation vector between the coordinate systems of the cameras.
                        Mat essentialMatrix;
                        Mat fundamentalMatrix;
                        TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 1000, 1e-6);
                        double d = stereoCalibrate/*Extended*/( aryPhyCorners,
                                                                aryImgCorners[Camera_Left_0], aryImgCorners[Camera_Right_1],
                                                                intrinsicMatrices[Camera_Left_0], distortMatrices[Camera_Left_0],
                                                                intrinsicMatrices[Camera_Right_1], distortMatrices[Camera_Right_1],
                                                                unifiedImageSize, //Size of the image used only to initialize the camera intrinsic matrices.
                                                                rotationMatrix_R, translationVector_T, essentialMatrix, fundamentalMatrix,
                                                                0,// CALIB_RATIONAL_MODEL + CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
                                                                //CALIB_FIX_INTRINSIC |CALIB_FIX_FOCAL_LENGTH,  //YT python
                                                                //CALIB_SAME_FOCAL_LENGTH, long black
                                                                //CALIB_FIX_INTRINSIC,  fast all black
                                                                //0, //fast, LEFT correct, Right curvly distorted
                                                                //CALIB_USE_INTRINSIC_GUESS, long all black
                                                                //CALIB_FIX_FOCAL_LENGTH too close , // CALIB_FIX_INTRINSIC |
                                                                //CALIB_FIX_INTRINSIC +  CALIB_FIX_FOCAL_LENGTH , CALIB_USE_EXTRINSIC_GUESS(crash) |
                                                                // CALIB_FIX_ASPECT_RATIO + CALIB_ZERO_TANGENT_DIST +CALIB_USE_INTRINSIC_GUESS(crash) + +  CALIB_SAME_FOCAL_LENGTH
                                                                // CALIB_SAME_FOCAL_LENGTH + CALIB_RATIONAL_MODEL + CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
                                                                criteria );
                        cout << "\n\nstereoCalibrate d=" << d << endl;
                        cout << "R=" << rotationMatrix_R << endl;
                        cout << "T=" << translationVector_T << endl;
                        cout << "E=" << essentialMatrix << endl;
                        cout << "F=" << fundamentalMatrix << endl;

    /* function stereoRectify computes the rotation matrices for each camera that (virtually) make both camera image planes the same plane.
    * Consequently, this makes all the epipolar lines parallel and thus simplifies the dense stereo correspondence problem.
    * The function takes the matrices computed by stereoCalibrate() as input.
    * As output, it provides two rotation matrices and also two projection matrices in the new coordinates.
    */
                        Mat rectificationTransform_R[2];  //3x3
                        //Mat projectionMatrices_P[2];  //3x4
                        Mat Q; //disparity-to-depth mapping matrix
                        Size rectifiedSize(unifiedImageSize.width * RectifyRatioX, unifiedImageSize.height * RectifyRatioY);  // Size(0, 0), M#
                        stereoRectify( intrinsicMatrices[Camera_Left_0], distortMatrices[Camera_Left_0],
                                                    intrinsicMatrices[Camera_Right_1], distortMatrices[Camera_Right_1],
                                                    unifiedImageSize, rotationMatrix_R, translationVector_T,
                                                    rectificationTransform_R[Camera_Left_0], rectificationTransform_R[Camera_Right_1],
                                                    sProjectionMatrices_P[Camera_Left_0], sProjectionMatrices_P[Camera_Right_1],
                                                    Q,
                                                    CALIB_ZERO_DISPARITY,
                                                     nsImageProcess::RectifyAlpha, //
                                                    //0, // get T_x
                                                    //CALIB_USE_EXTRINSIC_GUESS,
    /*
    *  If set, the function makes the principal points of each camera have the same pixel coordinates in the rectified views.
    * if not set, the function may still shift the images in the horizontal or vertical direction (depending on the orientation of epipolar lines)
    * to maximize the useful image area. */
                                                     //-1, both enlarged, leaving small areas overlapped
                                                    //1, //left recified, shifted up , right distorted curvly
    /*  alpha=0 means that the rectified images are zoomed and shifted so that only valid pixels are visible (no black areas after rectification).
    * alpha=1 means that the rectified image is decimated and shifted so that all the pixels from the original images from the cameras
    * are retained in the rectified images (no source image pixels are lost). */

                                                    rectifiedSize, // Size(0, 0), // IN, set to the original imageSize) newImageSize = Size(),
                                                     &validPxlROI[Camera_Left_0], &validPxlROI[Camera_Right_1]
    //Optional output rectangles inside the rectified images where all the pixels are valid.
                                                    );
    #ifdef _DISPLAY_LOADED
                        cout << "RR_L=" << rectificationTransform_R[Camera_Left_0] << endl;
                        cout << "RR_R=" << rectificationTransform_R[Camera_Right_1] << endl;
                        cout << "PP_L=" << sProjectionMatrices_P[Camera_Left_0] << endl;
                        cout << "PP_R=" << sProjectionMatrices_P[Camera_Right_1] << endl;
                        cout << "PP_R type=" << sProjectionMatrices_P[Camera_Right_1].type() << endl;
                        cout << "Q=" << Q << endl;
                        cout << "ROI_L=" << validPxlROI[Camera_Left_0] << endl;
                        cout << "ROI_R=" << validPxlROI[Camera_Right_1] << endl;
    #endif

                        SaveMatBinary(sProjectionMatrices_P[Camera_Right_1],
                                                            CalibrateDir + "last/RectifiedPPR");

    /* Computes the undistortion and rectification transformation map.  The function computes
     * the joint undistortion and rectification transformation and represents the result in the form
     * of maps for #remap.
                        */
                        //Mat stereoMap[2][2];
                        for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
                        {
                                initUndistortRectifyMap(
                                                    intrinsicMatrices[nthCamera], distortMatrices[nthCamera],
                                                    rectificationTransform_R[nthCamera],
                                                    sProjectionMatrices_P[nthCamera],
                                                    rectifiedSize,  //       unifiedImageSize,
                                                    RECTIFY_MAP_TYPE_ID, //CV_32FC1, // CV_16SC2, //CV_32FC1,  // CV_32FC1, CV_32FC2 or CV_16SC2, s
                                                    sStereoMap[nthCamera][0], sStereoMap[nthCamera][1] );

                                for (int nthXY = 0; nthXY < 2; nthXY++)  //221018
                                {
                                        if ( !sStereoMap[nthCamera][nthXY].isContinuous() )
                                                sStereoMap[nthCamera][nthXY] = sStereoMap[nthCamera][nthXY].clone();

                                        SaveMatBinary(sStereoMap[nthCamera][nthXY],
                                                      CalibrateDir + "last/RectifyMap" + GetXYChar(nthXY) + "_"  + GetCameraChar(nthCamera));
                                //SaveMatBinary(sStereoMap[nthCamera][1], CalibrateDir + "last/RectifyMapY_" + GetCameraChar(nthCamera));
                                }
                                rectifyMapLoaded = true;
                                showCrop(nthCamera);
                        }
                        for(uint i{1}; i <= pairNum; i++)
                        {
                                for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
                                {
                                        const string side = GetCameraChar((CameraEnum) nthCamera);
                                        Mat mapImage, orgImage = imread(CalibrateDir + "/" + to_string(i) + side + ".bmp", IMREAD_GRAYSCALE);

                                        /*if (fmExposures.isCamera90DgrRotated)
                                                switch(nthCamera)
                                                {
                                                case nsCameras::Camera_Left_0:
                                                                                        rotate(orgImage , orgImage , ROTATE_90_CLOCKWISE);
                                                                                        break;
                                                case nsCameras::Camera_Right_1:
                                                                                        rotate(orgImage , orgImage , ROTATE_90_COUNTERCLOCKWISE);
                                                                                        break;
                                                } */

                                        remap( orgImage, mapImage,  //on_btCalibrateCheckerBoard_clicked
                                                        sStereoMap[nthCamera][0], sStereoMap[nthCamera][1],
                                                        INTER_LANCZOS4 //
                                                        //INTER_LINEAR
                                                    );
    //#ifdef _DEBUG
                                        imwrite(DEBUG_PATH + "rctd_" + to_string(i) + side  + ".bmp",  mapImage);
    //#endif
                                }
                        }
        over:;
                 WorkingDir = OriginalWorkingDir;
                GUIRestoreCalibrate();
                QMessageBox::information(this, "Calibration Stage I", "The checkerboard is calibrated.");
        }

}


bool MainWindow::loadRectifiedMaps()
{
        //if (rectifyMapLoaded) return true;
        if (!GetCalibrateDir())
                return false;
        //const bool isAutoSEing = IsAutoSECalibrating || IsAutoSEMeasuring;
        OnCalibrateDirChanged();
        String lastCalibFile = CalibrateDir + "last/RectifiedPPR";
        QFileInfo check_file (QString::fromStdString(lastCalibFile ));
        if (!check_file.exists())
        {
                if (!NO_GUI_MODE && !IsAutoSEing())
                        QMessageBox::information(this, "Aborted", "Failed to find rectified maps, the checkboards must have been calibrated beforehand.");
                return false;
        }
        if (LoadMatBinary(lastCalibFile , sProjectionMatrices_P[Camera_Right_1]) != RC_OK)
        {
                if (!NO_GUI_MODE && !IsAutoSEing())
                        QMessageBox::information(this, "Aborted", "Failed to load rectified maps, please make sure the checkboards have been properly calibrated.");
                return false;
        }
#ifdef _DEBUG
            if (!QDir(QString::fromStdString(CalibrateDir + "debug")).exists())
                   QDir().mkdir(QString::fromStdString(CalibrateDir + "debug"));
#endif
#ifdef _DISPLAY_LOADED
                cout << "PP_R type=" << sProjectionMatrices_P[Camera_Right_1].type() << endl;
                cout << sProjectionMatrices_P[Camera_Right_1] << endl;
#endif
        for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
        {
                //stereoMap[nthCamera ][0].setTo(0);
                //int res = LoadMatBinary(CalibrateDir + "/last/RectifyMapX_" + GetCameraChar(nthCamera), stereoMap[nthCamera ][0]);
                for (int nthXY = 0; nthXY <= 1; nthXY++)
                        if (LoadMatBinary(
                                    CalibrateDir + "last/RectifyMap" + GetXYChar(nthXY) + "_" + GetCameraChar(nthCamera),
                                    sStereoMap[nthCamera ][nthXY]) != RC_OK)
                                return false;  //goto over;
                /*if (LoadMatBinary(CalibrateDir + "last/RectifyMapY_" + GetCameraChar(nthCamera), sStereoMap[nthCamera ][1]) != RC_OK)
                        return false;  //goto over; */
                showCrop(nthCamera, 500, 10);
        }
            cout << "The rectifed map just loaded\n\n";
        rectifyMapLoaded = true;
        return true;
}
void MainWindow::on_btCalibrateSquareBar_clicked()
{
        if (tmLiveShootShowBoth->isActive())
                StopLiveShootTimer();

        //const bool isAutoSEing = IsAutoSECalibrating || IsAutoSEMeasuring;
        //if (!isAutoSEing)
                if (fmExposures.IsHardwareTriggered())
                {
                        fmExposures.ui->kbHWTrigger->setEnabled(false);
                        //... QMessageBox::information(this, "Rejected", "Hardware-triggering supports only the auto function from Main-AP, \noff-line please tune to software-triggered in the parameters dialog first.");
                        //return;
                }
        if (this->ui->btCalibrateSquareBar->text() == CalibrateSquareBarCaption)
        {
                //constexpr uint CALIB_CB_EXPOSURE[2] = {2500, 2000}; //2000, 1500}; //laser light 105000 black stone;  //100000 for square-bar 3500
                //for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
                //       TuneCameraExposure(nthCamera , CALIB_CB_EXPOSURE[nthCamera ] );
                TuneCameraExposure(Camera_Left_0, CameraExposures[OM_CALIBRATE_SQUARE_BAR].first * 1000 );
                TuneCameraExposure(Camera_Right_1, CameraExposures[OM_CALIBRATE_SQUARE_BAR].second * 1000);

                if (!loadRectifiedMaps())
                {
                        if (IsAutoSEing())
                                sktClientMBD_CMD->write( ("SE, " + QString::number(RC_FAIL_TO_LOAD)+ "\n").toStdString().c_str() );
                        return;
                }

                this->ui->btCalibrateCheckerBoard->setEnabled(false);
                this->ui->btFetchRailCrown->setEnabled(false);
                if (fmExposures.isVisible())
                        fmExposures.close();
                mvCallbackContext.OP_Mode = opMode = OM_CALIBRATE_SQUARE_BAR;
                this->ui->btCalibrateSquareBar->setText("Stop");
           } else {
                this->ui->btCalibrateSquareBar->setText(CalibrateSquareBarCaption);
                //this->ui->btFetchLine->setEnabled(true);
                this->ui->btCalibrateCheckerBoard->setEnabled(true);
                this->ui->btFetchRailCrown->setEnabled(true);
                if (fmExposures.isVisible())
                        fmExposures.close();
                mvCallbackContext.OP_Mode = opMode = OM_IDLE;
        }
        fmExposures.SetupMode(opMode); // 230319
}

bool MainWindow::saveSECResults()
{

        FileStorage FS(CalibrateDir + "last/RPTOI", FileStorage::WRITE);
        if (!FS.isOpened())
                return false;
        {
                for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                {
                        String cameraID = nsCameras::GetCameraChar(nthDevice);
                        String key = cameraID + "_RPTOI_X";
                        FS << key << sReverseMapTOI[nthDevice].x;
                        key = cameraID + "_RPTOI_Y", FS << key << sReverseMapTOI[nthDevice].y;
                        key = cameraID + "_RPTOI_W", FS << key << sReverseMapTOI[nthDevice].width;
                        key = cameraID + "_RPTOI_H", FS << key << sReverseMapTOI[nthDevice].height;
                }
                FS.release();
                //QMessageBox::information(this, "Calibration Stage II", "The SE(Straight Edge) is calibrated.");
        }
        for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
        {
                String filename = CalibrateDir + "last/RMapMask" + "_" + GetCameraChar(nthCamera);
#ifdef _DISPLAY_LOADED
                    //cout << "Save RMM type =" << sReverseMapMask[nthCamera].type() << endl; // CalibrateDir + "last/RMapMask" + "_" + GetCameraChar(nthCamera) << endl;
                    //imwrite(DEBUG_PATH + "$S SEC-RMM_" + to_string(nthCamera) +".bmp",  sReverseMapMask[nthCamera]);
#endif
                /*if
                 *  ( !sReverseMapMask[nthCamera].isContinuous() )
                        sReverseMapMask[nthCamera] = sReverseMapMask[nthCamera].clone(); */
                SaveMatBinary(sReverseMapMask[nthCamera], filename);

#ifdef _DISPLAY_LOADED
                //*
                sReverseMapMask[nthCamera].setTo(100);
                 if (LoadMatBinary(filename, sReverseMapMask[nthCamera]) == RC_OK)
                {
                    //cout << "Save-Load RMM type =" << sReverseMapMask[nthCamera].type() << endl; //:" << CalibrateDir + "last/RMapMask" + "_" + GetCameraChar(nthCamera) << endl;
                    //imwrite(DEBUG_PATH + "$LS SEC-RMM_" + to_string(nthCamera) +".bmp",  sReverseMapMask[nthCamera]);
                } /**/
#endif
                        //return false;  //goto over;
        }
        return true;
}
bool MainWindow::loadSECResults()
{
        //keep loading the X-axis
        GaugeAxisX[0] = GaugeAxisX[1] = Point3d(-1, -1, -1);
        String cbClb = CalibrateDir + "last/LaserCS";
        QFileInfo check_file (QString::fromStdString(cbClb));
        if (!check_file.exists()) return false;

        String cbClb2 = CalibrateDir + "last/RPTOI";
        QFileInfo check_file2 (QString::fromStdString(cbClb2));
        if (!check_file2.exists()) return false;

        FileStorage FS(cbClb, FileStorage::READ);
        if (!FS.isOpened() )
                return false;
        try {
                FS["AxisX_0"] >> GaugeAxisX[0];
                FS["AxisX_1"] >> GaugeAxisX[1];
                FS["AxisY_1"] >> GaugeAxisY;
                FS["AxisYSlant"] >> AxisYSlant;
                //if (BOX_ID == 1)
                {
                        const double angle = fmExposures.SERFineTune; //-20;
                        //+2(8, 46~52)
                        //+3(8, 47~52, tail 15,55 x 40)
                        //+4(8, 47~52, tail 15,55 x 20)
                        //+6(8, 47~52, tail 15,55 x 40)
                        //-5 +5  -0.5 -0.2(1, 52)  -0.1 (44, -20)  -0.3(15, 54)
                        if (angle)
                        {
                                RotateBaroundA(GaugeAxisX[0], GaugeAxisX[1], angle);
                                RotateBaroundA(GaugeAxisX[0], GaugeAxisY, angle);
                        }
                        /*double a = -0.25 * M_PI / 180;
                        double dx = GaugeAxisX[1].x - GaugeAxisX[0].x;
                        double dy = GaugeAxisX[1].y - GaugeAxisX[0].y;
                        GaugeAxisX[1].x = (dx * cos(a)) - (dy * sin(a)) + GaugeAxisX[0].x;
                        GaugeAxisX[1].y = (dx * sin(a)) + (dy * cos(a)) + GaugeAxisX[0].y;
                        dx = GaugeAxisY.x - GaugeAxisX[0].x;
                        dy = GaugeAxisY.y - GaugeAxisX[0].y;
                        GaugeAxisY.x = (dx * cos(a)) - (dy * sin(a)) + GaugeAxisX[0].x;
                        GaugeAxisY.y = (dx * sin(a)) + (dy * cos(a)) + GaugeAxisX[0].y;
                        //AxisYSlant supposed to stay the same;
                        */
                }
        } catch (const GenericException& e) {
                FS.release();
                return false;
        }
        FS.release();

        FileStorage FS2(cbClb2, FileStorage::READ);
        if (!FS2.isOpened() )
                return false;
        try {
                FS2["L_RPTOI_X"] >> sReverseMapTOI[Camera_Left_0].x;
                FS2["L_RPTOI_Y"] >> sReverseMapTOI[Camera_Left_0].y;
                FS2["L_RPTOI_W"] >> sReverseMapTOI[Camera_Left_0].width;
                FS2["L_RPTOI_H"] >> sReverseMapTOI[Camera_Left_0].height;

                FS2["R_RPTOI_X"] >> sReverseMapTOI[Camera_Right_1].x;
                FS2["R_RPTOI_Y"] >> sReverseMapTOI[Camera_Right_1].y;
                FS2["R_RPTOI_W"] >> sReverseMapTOI[Camera_Right_1].width;
                FS2["R_RPTOI_H"] >> sReverseMapTOI[Camera_Right_1].height;
        } catch (const GenericException& e) {
                FS2.release();
                return false;
        }
        FS2.release();

        if (       (GaugeAxisX[0].z < 0) || (GaugeAxisX[1].z < 0)
                || ((GaugeAxisY.x <= 0) && (GaugeAxisY.y <= 0) && (GaugeAxisY.z <= 0))
                )
                return false;

        for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
        {
                sReverseMapMask[nthCamera].setTo(150);
                if (LoadMatBinary(CalibrateDir + "last/RMapMask" + "_" + GetCameraChar(nthCamera), sReverseMapMask[nthCamera]) == RC_OK)
                {
#ifdef _DISPLAY_LOADED
                    //cout << "Load RMM:" << CalibrateDir + "last/RMapMask" + "_" + GetCameraChar(nthCamera) << endl;
                    //imwrite(DEBUG_PATH + "$LSEC-RMM_" + to_string(nthCamera) +".bmp",  sReverseMapMask[nthCamera]);
#endif
                } else
                        return false;  //goto over;
        }
        return true;
}

void MainWindow::on_btFetchRailCrown_clicked()
{
        if (tmLiveShootShowBoth->isActive())
                StopLiveShootTimer();
    //cout << this->ui->btFetchRailCrown->text().toStdString() << endl;
        if (this->ui->btFetchRailCrown->text() == FetchRailCrownCaption)
        {
                //constexpr uint LASER_EXPOSURE_ON_RAIL_CROWN[2] = {20000, 15000}; // {2000, 1500}; //105000 black stone;  //100000 for square-bar 3500
                /* for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
                        TuneCameraExposure(nthCamera , LASER_EXPOSURE_ON_RAIL_CROWN[nthCamera] ); */
                TuneCameraExposure(Camera_Left_0, CameraExposures[OM_FETCH_RAIL_CROWN].first * 1000 );
                TuneCameraExposure(Camera_Right_1, CameraExposures[OM_FETCH_RAIL_CROWN].second * 1000);

                if (!loadRectifiedMaps())
                {
                        QMessageBox::information(this, "Aborted", "Failed to load rectified maps, please make sure the checkboard has been properly calibrated.");
                        return;
                }
                if (!loadSECResults())
                {
                        QMessageBox::information(this, "Aborted", "Failed to load the calibrative results of SE(streight edge), please make sure it has been properly calibrated.");
                        return;
                }
#ifdef MOVED
                //keep loading the X-axis
                GaugeAxisX[0] = GaugeAxisX[1] = Point3d(-1, -1, -1);
                String cbClb = CalibrateDir + "last/LaserCS";
                QFileInfo check_file (QString::fromStdString(cbClb));
               if (!check_file.exists()) return;

               String cbClb2 = CalibrateDir + "last/RPTOI";
               QFileInfo check_file2 (QString::fromStdString(cbClb2));
              if (!check_file2.exists()) return;

                FileStorage FS(cbClb, FileStorage::READ);
                if (!FS.isOpened() )
                        return;
                try {
                        FS["AxisX_0"] >> GaugeAxisX[0];  XX
                        FS["AxisX_1"] >> GaugeAxisX[1];
                        FS["AxisY_1"] >> GaugeAxisY;
                } catch (const GenericException& e) {
                        return;
                }
                FS.release();

                FileStorage FS2(cbClb2, FileStorage::READ);
                if (!FS2.isOpened() )
                        return;
                try {
                        FS2["L_RPTOI_X"] >> sReverseMapTOI[Camera_Left_0].x;
                        FS2["L_RPTOI_Y"] >> sReverseMapTOI[Camera_Left_0].y;
                        FS2["L_RPTOI_W"] >> sReverseMapTOI[Camera_Left_0].width;
                        FS2["L_RPTOI_H"] >> sReverseMapTOI[Camera_Left_0].height;

                        FS2["R_RPTOI_X"] >> sReverseMapTOI[Camera_Right_1].x;
                        FS2["R_RPTOI_Y"] >> sReverseMapTOI[Camera_Right_1].y;
                        FS2["R_RPTOI_W"] >> sReverseMapTOI[Camera_Right_1].width;
                        FS2["R_RPTOI_H"] >> sReverseMapTOI[Camera_Right_1].height;
                } catch (const GenericException& e) {
                        return;
                }
                FS2.release();

                if ((GaugeAxisX[0].z < 0) || (GaugeAxisX[1].z < 0))
                        return;;
#endif
                this->ui->btCalibrateCheckerBoard->setEnabled(false);
                this->ui->btCalibrateSquareBar->setEnabled(false);
                if (fmExposures.isVisible())
                        fmExposures.close();
                mvCallbackContext.OP_Mode = opMode = OM_FETCH_RAIL_CROWN;
                this->ui->btFetchRailCrown->setText("Stop");
       } else {
                this->ui->btFetchRailCrown->setText(FetchRailCrownCaption);
                this->ui->btCalibrateCheckerBoard->setEnabled(true);
                this->ui->btCalibrateSquareBar->setEnabled(true);
                if (fmExposures.isVisible())
                        fmExposures.close();
                mvCallbackContext.OP_Mode = opMode = OM_IDLE;
        }
        fmExposures.SetupMode(opMode); // 230319
}

/*
  double euclidean_distance_3d(double x, double y)
  {
        return ( (x[0] - y[0]) ^ 2+ (x[1] - y[1]) ^ 2 + (x[2] - y[2]) ^ 2) ^ 0.5;
  }*/


void MainWindow::SetManualCapture()
{
        ui->btCapture->setEnabled(!fmExposures.IsHardwareTriggered());
}
void displayCanvasLabel(QLabel *canvas, Mat *photo)
{
        QPixmap pixmap;
        switch (photo->type())
        {
        case CV_8UC1:
                                    {
                                            // Set the color table (used to translate colour indexes to qRgb values)

                                            QVector<QRgb> colorTable;
                                            for (int i = 0 ; i < 256; i++)
                                                    colorTable.push_back(qRgb(i, i, i));
                                            // Copy input Mat
                                            //const uchar *qImageBuffer = (const uchar*) photo->data;
                                            // Create QImage with same dimensions as input Mat
                                            QImage img( photo->data, //qImageBuffer,
                                                                    photo->cols, photo->rows, photo->step,
                                                                    QImage::Format_Indexed8);
                                            img.setColorTable(colorTable);
                                            pixmap = QPixmap::fromImage(img);
                                    }
                                    break;
        // 8-bits unsigned, NO. OF CHANNELS=3
        case CV_8UC3:
                                        {
                                            /*
                                             * resize(photo, photo, photo-> (photo->shape[1]//4*4
                                                            , img.shape[0]//4*4), fx=0, fy=0, interpolation=cv2.INTER_NEAREST)
                                            Rect myROI(10, 10, 100, 100);
                                            Mat photo2 = photo(Range(80,280), Range(150,330));
                                            photo = cvtColor(photo2, COLOR_BGR2RGB)
                                                */
                                            // Copy input Mat
                                           // QImage img;
                                            //const uchar *qImageBuffer = (const uchar*) photo->data;
                                            // Create QImage with same dimensions as input Mat
                                            QImage img(     photo->data, //qImageBuffer,
                                                                            photo->cols, photo->rows, photo->step,
                                                                            QImage::Format_RGB888);
                                            //img = img.rgbSwapped();
                                            pixmap = QPixmap::fromImage(img); //.rgbSwapped());
                                        }
                                    break;
        default:
                                        {
                                            qDebug() << "ERROR: Mat could not be converted to QImage or QPixmap.";
                                            return;
                                        }
        }
        canvas->setPixmap(pixmap);
        canvas->setScaledContents(true);
}
//void MainWindow::on_pushButton_clicked() {}



/*
//while not rail end => Controlled by START/STOP signals from Tibbo

  int Hi=0;
  int Vi=0;
  for (int j = 0; j <= 9; j++)  // for j=0 to 9
          for (int i = 1; i <= 5; i++)  // for i=1 to 5
                if (sktMPC[i-1])
                    if (sktMPC[i-1]->state() == QAbstractSocket::ConnectedState)
                    {
                            Hi =Hi + i; //H(i) =H(i) + i
                            Vi = Vi + i; //V(i) = V(i) - i
                            //send H(i), V(i)
                            packetStr.clear();
                            packetStr << V.x << COMMA << Vi << COMMA
                                                << Hi << COMMA << H.y << COMMA
                                                << TriggerCount << COMMA <<  elapsed << "\n";
                            sktMPC[i-1]->write(packetStr.str().c_str());
                            sktMPC[i-1]->waitForBytesWritten();
                            QThread::msleep(40);
                    }
  for (int j = 10; j <= 19; j++)  //  for j=10 to 19
        for (int i = 1; i <= 5; i++)  // for i=1 to 5
               if (sktMPC[i-1])
                   if (sktMPC[i-1]->state() == QAbstractSocket::ConnectedState)
                   {
                       Hi =Hi - i;  //H(i) =H(i) - i
                       Vi = Vi - i;  //V(i) = V(i) - i
                        //send H(i), V(i)
                       packetStr.clear();
                       packetStr << V.x << COMMA << Vi << COMMA
                                           << Hi << COMMA << H.y << COMMA
                                           << TriggerCount << COMMA <<  elapsed << "\n";
                       sktMPC[i-1]->write(packetStr.str().c_str());
                       sktMPC[i-1]->waitForBytesWritten();
                       QThread::msleep(40);
               }
*/
