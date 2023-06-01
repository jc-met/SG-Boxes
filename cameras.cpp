#define CAMERAS_CPP

#include "cameras.h"

using namespace std;
using namespace Pylon;
using namespace Basler_UniversalCameraParams;
using namespace std;
using namespace chrono;
using namespace cv;

//namespace nsCameras {

string nsCameras::GetCameraChar(int nthCamera)
{
        switch(nthCamera)
        {
        case Camera_Left_0: return "L";
        case Camera_Right_1: return "R";
        default:  return "";  //unknown
        }
}
string nsCameras::GetCameraStr(int  nthCamera)
{
        switch(nthCamera)
        {
        case Camera_Left_0: return "LEFT";
        case Camera_Right_1: return "RIGHT";
        }
}

static int nsCameras::initializeCallbackBuffer(sCallbackBuf *pCallbackBuf, int width, int height)
{
        if (CameraBrand < 0)
                return RC_LACK_CAMERAS;
        pCallbackBuf->frameCols = width;
        pCallbackBuf->frameRows = height;
        pCallbackBuf->frameSize = pCallbackBuf->frameCols * pCallbackBuf->frameRows;

        switch(CameraBrand)
        {
        case Camera_Basler:    pCallbackBuf->alignedBuf = NULL;  //initializeCallbackBuffer
                                                    break;
        case Camera_MV:          pCallbackBuf->alignedBuf = CameraAlignMalloc( pCallbackBuf->frameSize  * 4, 16);  //initializeCallbackBuffer
                                                    if (!pCallbackBuf->alignedBuf)
                                                            return RC_FAIL_TO_ALLOCATE;
                                                    break;
        }
        pCallbackBuf->uncopied = false;
        return RC_OK;
}

bool nsCameras::BaslerGrabOne(uint theDevice, CGrabResultPtr pGrabbedResult)
{
            bool result = false;
            //if (cameras[theDevice].IsOpen())
            //if (cameras[theDevice].IsPylonDeviceAttached())
                //return RC_LACK_CAMERAS;
            if (cameras[theDevice].IsGrabbing())
            try {
                     cameras[theDevice].RetrieveResult//GrabOne  //
                             ( MAX_EXPOSURE_ms, pGrabbedResult, TimeoutHandling_ThrowException);
                     result = true;
           } catch (const GenericException& e) {
                        qDebug() << "BGO:" << e.GetDescription();
            }
            return result;
}

int nsCameras::InitializeCameras(void *contextForCallback) //Mat *matPhotoes, PVOID pMainForm)
{
        //moved CameraBrand = CameraBrands(Camera_Unknown);
//qDebug()  << "InitializeCameras";

        int exitCode = RC_OK;
        int iCameraNums = 0;

#ifdef _CAMERA_BASLER
        bool swap;
        try //pylon
        {
                sCallbackContext *mvCallbackContext;

                PylonInitialize();  //initialize pylon runtime

                // Get the transport layer factory.
                CTlFactory& tlFactory = CTlFactory::GetInstance();

                // Get all attached devices and exit application if no device is found.
                DeviceInfoList_t devices;
                //String_t SN[C_MaxCamerasToUse];
                string SN[C_MaxCamerasToUse] = { "", ""};
                iCameraNums = tlFactory.EnumerateDevices( devices );
                if (iCameraNums <
#ifdef _DEBUG_ONE_CAMERA
                        1
#else
                        2
#endif
                        )
                {
    //                    /if (execCode == RC_CAMERA_NOT_OPEN) QMessageBox::critical(&w, "title",  "Must connect to two Basler cameras.");
                        //QMessageBox::critical(this, "title",  "Must connect to two Basler cameras.");
                        //qDebug() << "Must connect to two Basler cameras." << endl;
                        //RUNTIME_EXCEPTION( "Must connect to two Basler cameras.");
                        exitCode = RC_LACK_CAMERAS;
                        goto over1;
                }
                CameraBrand = Camera_Basler;  //InitializeCameras
                //sn0 = devices[0].GetSerialNumber();
                //sn1 = devices[1].GetSerialNumber();

                mvCallbackContext = (sCallbackContext *) contextForCallback;
                for (size_t nthDevice = 0; nthDevice <C_MaxCamerasToUse; nthDevice++)  // and attach all Pylon Devices.
                        memset(&mvCallbackContext->mvCallbackBuf[nthDevice], 0, sizeof(sCallbackBuf)) ;  //230503
                //mvCallbackContext->mvCallbackBuf[nthDevice].frameSize = 0;   //230503  uncopied true

                for (size_t nthDevice = 0; nthDevice < iCameraNums /*cameras.GetSize()*/; nthDevice++)  // and attach all Pylon Devices.
                        SN[nthDevice] = devices[nthDevice].GetSerialNumber();
                swap = (iCameraNums > 1) && (SN[0].compare(SN[1]) > 0) ;  //the Left camera come with the smaller SN swap
                // Create an array of instant cameras for the found devices and avoid exceeding a maximum number of devices.
                //cameras.Initialize( C_MaxCamerasToUse );
                for (size_t nthDevice = 0; nthDevice < iCameraNums /*cameras.GetSize()*/; nthDevice++)  // and attach all Pylon Devices.
                {
                        //IPylonDevice *pd
                        pylonDevices[nthDevice]  = tlFactory.CreateDevice( devices[ (swap) ? 1- nthDevice : nthDevice] );
                        cameras[nthDevice].Attach(pylonDevices[nthDevice]);  //attach the device to a camera
                        if (!cameras[nthDevice].IsPylonDeviceAttached())
                            return RC_LACK_CAMERAS;

                        /*
                                ShutterMode_Global,  //!< The shutter opens and closes at the same time for all pixels - Applies to: CameraLink, GigE and ace USB
                                ShutterMode_GlobalResetRelease,  //!< The shutter opens at the same time for all rows but closes in a sequential manner - Applies to: CameraLink, GigE and ace USB
                                ShutterMode_Rolling  //!< The shutter opens and closes sequentially for groups of rows - Applies to: CameraLink, GigE and ace USB
                        */
                        cameras[nthDevice].Open();

                        if (cameras[nthDevice].IsOpen())
                        {
                                SetCameraTriggerMode(nthDevice, TriggerMode_Software );
                                cout << "Camera "<< nthDevice <<  " " << cameras[nthDevice].GetDeviceInfo().GetModelName() << cameras[nthDevice].GetDeviceInfo().GetSerialNumber() ;
                                cout << ", W/H: " << cameras[nthDevice].Width.GetValue() << " / " << cameras[nthDevice].Height.GetValue() << endl << endl;
//After open basler
//cameras[nthDevice].ExposureAuto.SetValue(ExposureAuto_Off);
//int rc = cameras[nthDevice].ExposureAuto.GetValue();
                                //qDebug() << "Basler" << nthDevice << "width:" << cameras[nthDevice].Width.GetValue() << cameras[nthDevice].Height.GetValue() << endl;
                                /*cameras[nthDevice].ShutterMode.SetValue(
                                #ifdef _CAMERA_GLOBAL
                                                        ShutterMode_Global
                                #endif
                                #ifdef _CAMERA_ROLLING
                                                        ShutterMode_Rolling
                                #endif
                                    ); */
                                mvCallbackContext = (sCallbackContext *) contextForCallback;
                                sCallbackBuf *pCallbackBuf = &mvCallbackContext->mvCallbackBuf[nthDevice];
                                /*CGrabResultPtr  pGrabbedResult;
                                cameras[nthDevice].GrabOne( 50, //MAX_EXPOSURE_ms, //  M# 1600 for MV?!  2000, //1000, //500 not enough for expossure 350ms?
                                                                                                pGrabbedResult,
                                                                                                TimeoutHandling_ThrowException); */
                                CGrabResultPtr pGrabbedResult;
                                bool res ;
                                try {
                                //?? if (!HWTriggered)
                               cameras[nthDevice].StartGrabbing();
                                // Switch on image acquisition
                               //cameras[nthDevice].AcquisitionStart.Execute();
                               res = BaslerGrabOne(0, pGrabbedResult);
                               res = BaslerGrabOne(0, pGrabbedResult);
                                        initializeCallbackBuffer(pCallbackBuf,
                                                                            cameras[nthDevice].Width.GetValue(), //pGrabbedResult->GetWidth(),
                                                                            cameras[nthDevice].Height.GetValue()  //pGrabbedResult->GetHeight()
                                                                    );
                                } catch (const GenericException& e) {
                                           cerr << "An exception 1-1 occurred." << endl << e.GetDescription() << endl;
                                      }
                               res = BaslerGrabOne(0, pGrabbedResult);
                                /**/
                                double d=2;
                        }
                        //cameras[i].GetDeviceInfo().
                }
                exitCode = RC_OK;
        over1:;
        } catch (const GenericException& e) {
                cerr << "Basler: " << e.GetDescription() << endl << endl;
                exitCode = RC_Exception;
        }

        //if (exitCode == RC_OK) goto finished; //return exitCode;

#endif  //Basler


#ifdef _CAMERA_MV
        tSdkCameraDevInfo swap ;
        try //MV
        {
                uint cnt;
                CameraSdkInit(1);   //sdk初始化  0 English 1中文

                iCameraNums = C_MaxCamerasToUse;
                if (CameraEnumerateDevice( mvCameraList, &iCameraNums) != CAMERA_STATUS_SUCCESS || iCameraNums == 0)
                {
                        exitCode = RC_LACK_CAMERAS;
                        goto over2;
                }
                if (iCameraNums  < C_MaxCamerasToUse)
                {
                        exitCode = RC_LACK_CAMERAS;
                        goto over2;
                }
                swap = mvCameraList[0];
                mvCameraList[0] = mvCameraList[1];
                mvCameraList[1] = swap;

                for (size_t i = 0; i < C_MaxCamerasToUse; i++)  // and attach all Pylon Devices.
                {
                        if ((CameraInit(&mvCameraList[i], -1, -1,&mvCamerahandles[i])) != CAMERA_STATUS_SUCCESS)
                        {
                                exitCode = RC_FAIL_TO_OPEN;
                                goto over2;
                        }
                        if (CameraGetCapability(mvCamerahandles[i], &mvCameraCaps[i]) != CAMERA_STATUS_SUCCESS)
                        {
                                exitCode = RC_FAIL_TO_OPEN;
                                goto over2;
                        }
                        CameraBrand = Camera_MV;

                        if (mvCameraCaps[i].sIspCapacity.bMonoSensor)
                                CameraSetIspOutFormat(mvCamerahandles[i], CAMERA_MEDIA_TYPE_MONO8);

                        sCallbackContext *mvCallbackContext = (sCallbackContext *) contextForCallback;
                        sCallbackBuf *pCallbackBuf = &mvCallbackContext->mvCallbackBuf[i];

                        if ((exitCode = initializeCallbackBuffer(pCallbackBuf, mvCameraCaps[i].sResolutionRange.iWidthMax, mvCameraCaps[i].sResolutionRange.iHeightMax)) != RC_OK)
                                    goto over2;
                        /*pCallbackBuf->frameCols = mvCameraCaps[i].sResolutionRange.iWidthMax;
                        pCallbackBuf->frameRows = mvCameraCaps[i].sResolutionRange.iHeightMax;
                        pCallbackBuf->frameSize = pCallbackBuf->frameCols * pCallbackBuf->frameRows;

                        pCallbackBuf->alignedBuf = CameraAlignMalloc( pCallbackBuf->frameSize  * 4, 16);
                        if (!pCallbackBuf->alignedBuf)
                        {
                                exitCode = RC_FAIL_TO_ALLOCATE;
                                goto over2;
                        }
                        pCallbackBuf->uncopied = false; */
                }
                if (CameraSetCallbackFunction(
                                    mvCamerahandles[0],
                                    mvGrabCallback0,  //watch that all cameras share the same callback
                                    (PVOID)contextForCallback, // &mvCallbackContext, //mvCallbackBuf[0], // (matPhotoes + i), //mvInfoList[intendedID]),
                                    nullptr
                                    ) != CAMERA_STATUS_SUCCESS )
                {
                        exitCode = RC_FAIL_TO_REGISTER;
                        goto over2;
                }
                if (CameraSetCallbackFunction(
                                    mvCamerahandles[1],
                                    mvGrabCallback1,
                                    (PVOID) contextForCallback, //&mvCallbackContext, //mvCallbackBuf[1],
                                    nullptr
                                    ) != CAMERA_STATUS_SUCCESS )
                {
                        exitCode = RC_FAIL_TO_REGISTER;
                        goto over2;
                }

                        /*if (CameraSetCallbackFunction(
                                            mvCamerahandles[i],
                                            mv
GrabCallback,  //watch that all cameras share the same callback
                                            (PVOID) &mvCallbackBuf[i], // (matPhotoes + i), //mvInfoList[intendedID]),
                                            nullptr
                                            ) != CAMERA_STATUS_SUCCESS )
                        {
                                exitCode = RC_FAIL_TO_REGISTER;
                                goto over2;
                        } */
                cout << "\t* 📹 " ;
                for (size_t i = 0; i < C_MaxCamerasToUse; i++)  // and tune all MV Devices.
                {
                        CameraSetAeState(mvCamerahandles[i], false);
                        if (CameraPlay(mvCamerahandles[i]) == CAMERA_STATUS_SUCCESS )
                        {
                                CameraSetTriggerMode(mvCamerahandles[i], MTM_CONTINUOUS) ; //initialized  IS457
                                cout << "Camera "<< i << " is ON. \t" ;
                                cnt++;
                        }
                        //cameras[i].Open();
                        //cameras[i].ShutterMode.SetValue(ShutterMode_Rolling);
                }
                cout << "\n\n" ;
                //CameraSetTriggerMode(mvCamerahandles[0], MTM_CONTINUOUS) ; //initialized  IS457
                //CameraSetTriggerMode(mvCamerahandles[1], MTM_CONTINUOUS) ; //initialized  IS457
                if (cnt == C_MaxCamerasToUse)
                        exitCode = RC_OK;

        over2:;
        } catch (const GenericException& e) {
                //cerr << "An exception 1 occurred." << endl << e.GetDescription() << endl;
        }
#endif  //MV

finished:
        if (iCameraNums < C_MaxCamerasToUse)
        {
                cout << "Warn: Sensor Box needs two cameras to operate normally!!\n" ;
                if (iCameraNums)
                        cout << "Only single camera is found, the Box would not behave normally. \n\n" ;
                else
                        cout << "No cameras are found, the Box cannot run normally. \n\n" ;
        }

        return exitCode ;
    }

int nsCameras::FinalizeCameras(void *contextForCallback)
{
    sCallbackContext *mvCallbackContext = (sCallbackContext *) contextForCallback;
    for (size_t i = 0; i < C_MaxCamerasToUse ; ++i)  // Create and attach all Pylon Devices.
        switch(CameraBrand)
        {
        case Camera_Basler:  //FinalizeCameras

                   if (cameras[i].IsOpen())
                        cameras[i].Close();
                    if (cameras[i].IsPylonDeviceAttached())
                        cameras[i].DetachDevice();
            break;
         case Camera_MV:  //FinalizeCameras
                    if (mvCallbackContext->mvCallbackBuf[i].alignedBuf)
                            CameraAlignFree(mvCallbackContext->mvCallbackBuf[i].alignedBuf);
                    if (mvCamerahandles[i])
                            CameraUnInit(mvCamerahandles[i]);
            break;
            }
    /**/
    // Releases all pylon resources.~
    PylonTerminate();

    return 0;
}

int nsCameras::OnPhotosCaptured(  //put into Q
        int theDevice,
        int cameraHandle,
        void *sdkCallbackBuffer,
        sCallbackContext *callbackContext,
        void *photoBufferHeader
        )
{
        if (!sdkCallbackBuffer || !callbackContext)  //pAlignBuf ||  !cameraHandle ||
                    return RC_INVALID_DATA;

        int rc = RC_FAIL_TO_REGISTER;
#ifdef MISSION_BS
        if (!callbackContext->TibboOn)   //IS777  redundant?!
                return RC_NO_IMAGE;
#endif
         //for Basler
        CGrabResultPtr *pGrabbedResult = NULL;
        //for MV
        BYTE *pMvFrameBuffer = NULL;
        CameraHandle hMvCamera = 0;
        tSdkFrameHead *pMvFrameHead = NULL;
        TProcessPhoto *pPhotopair = NULL;  //used by no DIRECT_QUEUE
#ifdef DIRECT_QUEUE
        bool saveIntoQueue; // = true;
#else
        bool saveIntoQueue = false;
#endif
//cout <<  "Brnd:" <<  callbackContext->cameraBrand << endl;
        switch(callbackContext->cameraBrand)
        {
        case Camera_Basler:   //OnPhotosCaptured-1
                                                pGrabbedResult = (CGrabResultPtr *) sdkCallbackBuffer;
                                                pPhotopair = (TProcessPhoto *) photoBufferHeader;
                                                saveIntoQueue = false;  //not supported                                                
                                                break;
        case Camera_MV:     //OnPhotosCaptured-1
                                                hMvCamera = cameraHandle;
                                                pMvFrameHead = (tSdkFrameHead *) photoBufferHeader;
                                                pMvFrameBuffer = (BYTE *) sdkCallbackBuffer;
                                                saveIntoQueue = true;
                                                break;
        }
//if (!saveIntoQueue) qDebug() << "SiQ becomes False-1";
//cout <<  "SiQ-1:" <<  saveIntoQueue << endl;
        //check mode?!
        sCallbackBuf *pCallbackBuf = &callbackContext->mvCallbackBuf[theDevice];
        QElapsedTimer &railTimer = *callbackContext->pRailTimer;
        ulong &TriggerStart = *callbackContext->pTriggerStart;
        int &LastRailGap = *callbackContext->pLastRailGap;
        ulong elapsed;
        if (LastRailGap >= 0)  //o.w
        {
                if (!LastRailGap)  //o.w precede handler
                        LastRailGap = railTimer.nsecsElapsed() - TriggerStart;  //IS834 try
                elapsed  = LastRailGap;
                LastRailGap = -1;
        } else
                elapsed  = (railTimer.nsecsElapsed() - TriggerStart);
//#ifdef  OS_LINUX
        elapsed  /= nsUtility::ns2ms;
//#endif
if (!saveIntoQueue) qDebug() << "SiQ becomes False-2";
#ifdef CB_NEED_MTX
        while (!callbackContext->pPhotoQMtx->try_lock())  //mvContext->pPhotoQMtx->lock(); //mvCallbackBuf
        {
                switch(callbackContext->OP_Mode)
                {
                case OM_FETCH_RAIL_CROWN:   QThread::msleep(5);  break;
                case OM_CALIBRATE_CHECKERBOARD: //moved up here 230117
                case OM_CALIBRATE_SQUARE_BAR:
                case OM_IDLE:
                                                QThread::msleep(20);  break;
                }
                //QApplication::processEvents();
        }
#endif
//callbackContext->pPhotoQMtx->unlock();
//return -999;  //@@@@@@@@@@
//goto over2; //@@@@@@@@@
//qDebug() << "OnPhC LK";
        try {
                TQueuePhoto *pQueueEntity;
                int thisHead, nextHead;
               pQueueEntity = NULL;
                thisHead = *callbackContext->pPhotoQHead;
                nextHead = (thisHead + 1) % IMAGE_QUEUE_SIZE;

                switch(callbackContext->OP_Mode)
                {
                case OM_CALIBRATE_CHECKERBOARD: //moved up here 230117
                case OM_CALIBRATE_SQUARE_BAR:

                case OM_IDLE:
                case OM_FETCH_RAIL_CROWN:
                                    if ( *callbackContext->pPhotoQHead != *callbackContext->pPhotoQTail) //empty
                                    {
                                            thisHead = (thisHead + IMAGE_QUEUE_SIZE - 1) % IMAGE_QUEUE_SIZE;
                                            nextHead = (nextHead + IMAGE_QUEUE_SIZE - 1) % IMAGE_QUEUE_SIZE;
                                    }
                                        break;
        //                                break;
                }
                if (!saveIntoQueue) qDebug() << "SiQ becomes False-3";
                static bool fullShown = false;
                if ( nextHead == *callbackContext->pPhotoQTail) //full
                {
                        if (!fullShown)
                                qDebug() << "************** PQ-Full H=>T=" << nextHead << "******************", fullShown = true;
                } else {
                        fullShown = false;
                        Mat *pQueueEntityImage;
    //qDebug() <<  "SiQ-2:" <<  saveIntoQueue;
    #ifdef _MISSION_BS
            //assert(saveIntoQueue == true);
    #endif
                        if (saveIntoQueue)
                        {
                                pQueueEntity = callbackContext->pPhotoQueues
                                                                    + 2 * thisHead/* *mvContext->pPhotoQHead */
                                                                    + theDevice;
                                pQueueEntityImage = &pQueueEntity->photoImage;
                        } else
                                pQueueEntityImage = (pPhotopair + theDevice)->pPhotoImage;

                        duration<double, milli> d(0);
                        auto t1 = high_resolution_clock::now();
                        CameraSdkStatus rss;
    #ifdef ROI_IMAGE                 //mvGrabCallback
                        const  bool IsRoiImage = true;
    #else
                        const  bool IsRoiImage = false;
    #endif
                        switch(callbackContext->OP_Mode)
                        {
                        //220111  case OM_CALIBRATE_CHECKERBOARD:
                        case OM_CALIBRATE_SQUARE_BAR:
                        case OM_FETCH_RAIL_CROWN:  //221110
                                    if (IsRoiImage)
                                    {
                                            Mat rawFrameFull(pCallbackBuf->frameRows, pCallbackBuf->frameCols, CV_8UC1);

                                            bool capturedFine = false;
                                            switch(callbackContext->cameraBrand)
                                            {
                                            case Camera_Basler:  //OnPhotosCaptured-2
                                                            capturedFine =  memcpy(rawFrameFull.ptr(), (uint8_t *) (*pGrabbedResult)->GetBuffer(), pCallbackBuf->frameSize);
                                                            break;
                                            case Camera_MV:  //OnPhotosCaptured-2
                                                            capturedFine =  (rss = CameraImageProcess(hMvCamera, pMvFrameBuffer, rawFrameFull.ptr(), pMvFrameHead)) == CAMERA_STATUS_SUCCESS;
                                                            break;
                                            }
                                            if (capturedFine)
                                            {
//qDebug() << "OnPhC-3";
                                                        //if (!theDevice) imwrite(DEBUG_PATH + "$Q0Org_" + to_string(thisHead) +".bmp",  rawFrameFull); //
                                                            //Mat rawFrameFull(pCallbackBuf->frameRows, pCallbackBuf->frameCols, CV_8UC1, pFrameBuffer);
                                                    pQueueEntityImage->/*pQueueEntity->photoImage.*/create(RAW_LASER_ROI_HEIGHT, RAW_LASER_ROI_WIDTH, CV_8UC1);
                                                    pQueueEntityImage->/*pQueueEntity->photoImage.*/setTo(0);
                                                    //rawFrameROI .create(RAW_LASER_ROI_HEIGHT, RAW_LASER_ROI_WIDTH, CV_8UC1);
                                                    Rect roi(RAW_LASER_ROI[callbackContext->Box_ID - 1][theDevice], Size(RAW_LASER_ROI_WIDTH, RAW_LASER_ROI_HEIGHT)); //mvGrabCallback before rotate
                                                    roi.height = min(roi.height, rawFrameFull.rows - roi.y); //  (int) RAW_LASER_ROI_HEIGHT); ///*rawFrameROI.rows*/ ),
                                                    roi.width = min(roi.width, rawFrameFull.cols - roi.x); //(int) RAW_LASER_ROI_WIDTH); //rawFrameROI.cols);
                                                    rawFrameFull(roi).copyTo(*pQueueEntityImage); //pQueueEntity->photoImage);
                                            }
                                            rawFrameFull.release();
                                            break;
                                    }
                        case OM_CALIBRATE_CHECKERBOARD:  //220111
                        case OM_IDLE:
                                            pQueueEntityImage->/*pQueueEntity->photoImage.*/create(pCallbackBuf->frameRows, pCallbackBuf->frameCols, CV_8UC1);
    //LKG_81
                                            switch(callbackContext->cameraBrand)
                                            {
                                            case Camera_Basler:           //OnPhotosCaptured-3  not necessary
                                                                rss =  memcpy(pQueueEntityImage->ptr(), (uint8_t *) (*pGrabbedResult)->GetBuffer(), pCallbackBuf->frameSize)
                                                                            ? CAMERA_STATUS_SUCCESS :CAMERA_STATUS_NO_MEMORY ;  //230308
                                                                break;
                                            case Camera_MV: //OnPhotosCaptured-3
                                                                rss = CameraImageProcess(hMvCamera, pMvFrameBuffer, pQueueEntityImage->/*pQueueEntity->photoImage.*/ptr(), pMvFrameHead);  // CameraSetIspOutFormat 8=>8
                                                                break;
                                             }
                                            break;
                        }
    //#endif
                        #ifdef _DEBUG
                                d = high_resolution_clock::now() - t1;
                                //qDebug() << "MVGC CIP: " << d.count();
                                t1 = high_resolution_clock::now();
                        #endif
                        if ( rss  == CAMERA_STATUS_SUCCESS)
                        {
                                ulong queueEntityTimeStamp, twinEntityTimeStamp;
                                TQueuePhoto *pQTwinEntity = NULL;
    //qDebug() <<  "SiQ-3:" <<  saveIntoQueue;
    #ifdef _MISSION_BS
            //assert(saveIntoQueue == true);
    #endif
                                if (saveIntoQueue)
                                {
                                        pQueueEntity->timeStamp  =  elapsed;
                                        queueEntityTimeStamp = pQueueEntity->timeStamp;
                                        pQTwinEntity = (!theDevice) ? pQueueEntity  + 1 : pQueueEntity - 1;
                                        twinEntityTimeStamp = pQTwinEntity->timeStamp;
                                } else {
                                            *(pPhotopair + theDevice)->pTimeStamp = elapsed;
                                            queueEntityTimeStamp = *(pPhotopair + theDevice)->pTimeStamp;
                                            twinEntityTimeStamp = *(pPhotopair + 1 -  theDevice)->pTimeStamp;
                                }
//qDebug() << "OnPhC-5";

                                int offset =  abs(queueEntityTimeStamp - twinEntityTimeStamp); //pQueueEntity->timeStamp - pQTwinEntity->timeStamp;
                                offset = abs(offset);
    #ifdef _DEBUG_SHOT_TmSmp
                                        //*
                                /*TQueuePhoto *pQTwinEntityP1 = pQueueEntity  + 1;
                                TQueuePhoto *pQTwinEntityN1 = pQueueEntity - 1;*/
                                        //qDebug() << "C" << theDevice <<", P1=" << pQTwinEntityP1->timeStamp << ", N1=" << pQTwinEntityN1->timeStamp;
                                        qDebug() << "C" << theDevice <<", QH" << *mvContext->pPhotoQHead << ":"
                                                 << pQueueEntity->timeStamp << "," << pQTwinEntity ->timeStamp << "," << offset;
                                        /**/
                                        //qDebug() << "L-R offset=" << offset << "\n";
    #endif
    #ifdef _MISSION_BS
            //assert(saveIntoQueue == true);
    #endif
    //qDebug() <<  "SiQ-4:" <<  saveIntoQueue;
                                if (saveIntoQueue)
                                        if (  (callbackContext->OP_Mode == OM_CALIBRATE_CHECKERBOARD)  //0111
                                             ||
                                                (offset  <
                #ifdef _DEBUG
                                                CAPTURED_OFFSET_TLRN_SWT
                #else
                                                CAPTURED_OFFSET_TLRN_HWT
                #endif
                                            ) )
                                        {
                                                switch(callbackContext->OP_Mode)
                                                {
                                                case OM_IDLE:
                                                case OM_FETCH_RAIL_CROWN:
                                                                if (*callbackContext->pPhotoQHead != *callbackContext->pPhotoQTail)  //keep 1
                                                                        break;
                                                case OM_CALIBRATE_CHECKERBOARD:
                                                case OM_CALIBRATE_SQUARE_BAR:
                                                                *callbackContext->pPhotoQHead = nextHead; //(*mvContext->pPhotoQHead + 1) % IMAGE_QUEUE_SIZE;
            #ifdef _DEBUG_SHOT_TmSmp
                                                   qDebug() << "Q Head =>" << *mvContext->pPhotoQHead  << "@"  << pQueueEntity->timeStamp << ", Tail=" << *mvContext->pPhotoQTail;
            #endif
                                                }
                                        } else if (offset < 111)  //dump both
                                        {
                                                    pQueueEntity->timeStamp  = pQTwinEntity ->timeStamp = 11;
                                                    qDebug() << "C" << theDevice <<", QH" << *callbackContext->pPhotoQHead << ":" << offset << ", dropped....";
                                        }
                        }
    #ifdef ROI_IMAGE
    //                       rawFrameROI.release();
    #endif
                }
over:;
        } catch (const GenericException& e) {
                     qDebug() << "OPC:" << e.GetDescription();
         }
over2:
#ifdef CB_NEED_MTX
       callbackContext->pPhotoQMtx->unlock();
//qDebug() << "OnPhC UL";
#endif

       switch(callbackContext->cameraBrand)
       {
       case Camera_Basler:           break;   //OnPhotosCaptured
       case Camera_MV:
//@@@@@@@
                                CameraReleaseImageBuffer(hMvCamera, pMvFrameBuffer);     //OnPhotosCaptured
                                break;
        }
       return rc;
}

//wait beforehand
int nsCameras::GrabOneImage(
        int theDevice,
        Mat &OCvImage,
        mutex& mtxPhoto,
        bool HWTriggered,
        void *callbackContext)
{
    //sCallbackContext *mvCallbackContext = (sCallbackContext *) callbackContext;
    //if (isClosing) return RC_IS_CLOSING;

    int rc = RC_NO_IMAGE;
    CGrabResultPtr pGrabbedResult;
    switch(CameraBrand)
    {
    case Camera_Basler:  //GrabOneImage
                    try {
                        //TuneCameraExposure(nthDevice, 3000);

                        if (HWTriggered)
                                cameras[theDevice].RetrieveResult(  MAX_EXPOSURE_ms,
                                                                                                    pGrabbedResult,
                                                                                                    TimeoutHandling_ThrowException);
                        else
                                cameras[theDevice].GrabOne( MAX_EXPOSURE_ms, //  M# 1600 for MV?!  2000, //1000, //500 not enough for expossure 350ms?
                                                                                                pGrabbedResult,
                                                                                                TimeoutHandling_ThrowException);
                    } catch (const GenericException& e) {
                            cerr << "An exception 2-1 occurred." << endl << e.GetDescription() << endl;
                            return RC_Exception;
                    }

                    try {
                        rc  = OnPhotosCaptured(  //GrabOneImage-Basler
                                            theDevice, 0, (void *) &pGrabbedResult, (sCallbackContext *) callbackContext);
            #ifdef PYLON_WIN_BUILD
                       // Pylon::DisplayImage( 1, pGrabbedResult);
            #endif
                        int frameCols = pGrabbedResult->GetWidth();
                        int frameRows = pGrabbedResult->GetHeight();
                        int frameSize = frameCols * frameRows;
#ifdef _DEBUG_MUTEX   //GOI-1-0
                        qDebug() << "GOI-1-0";
#endif
#ifdef LCK_GRD
                                lock_guard<std::mutex> lgMtxPhoto{mtxPhoto};
#else
                                mtxPhoto.lock();  //basler pGrabbedResult
#endif

                                qDebug() << "LG-1-1";
                        if (OCvImage .empty())
                            OCvImage  = cv::Mat(frameRows, frameCols, CV_8UC1);
                       memcpy(OCvImage.ptr(), (uint8_t *) pGrabbedResult->GetBuffer(), frameSize);
#ifndef LCK_GRD
                                mtxPhoto.unlock();
#endif

                      // imwrite("C:\\GrabbedImageCV.png",  OCvImage ); //theFrame);

                       rc = RC_OK;

                    } catch (const GenericException& e) {
                        cerr << "An exception 2-2 occurred." << endl << e.GetDescription() << endl;
                        return RC_Exception;
                    }
                    break;
        case Camera_MV:  //GrabOneImage
                    try {
                            sCallbackContext *mvCallbackContext = (sCallbackContext *) callbackContext;
                            if (HWTriggered)
                            {  //polled
                                    if (mvCallbackContext->mvCallbackBuf[theDevice].uncopied)
                                    {
#ifdef _DEBUG_MUTEX   //GOI-3-0
                                        qDebug() << "GOI-3-0";
#endif
#ifdef LCK_GRD
                                                lock_guard<std::mutex> lgMtxPhoto{mtxPhoto};
#else
                                                mtxPhoto.lock(); //mvCallbackBuf
#endif
                                                    qDebug() << "LG-2-1";
                                            sCallbackBuf *pCallbackBuf = &mvCallbackContext->mvCallbackBuf[theDevice];
                                            if (OCvImage .empty())
                                                    OCvImage  = cv::Mat(pCallbackBuf->frameRows, pCallbackBuf->frameCols, CV_8UC1);
                                           memcpy(OCvImage.ptr(),
                                                                pCallbackBuf->alignedBuf,
                                                                pCallbackBuf->frameSize);
#ifndef LCK_GRD
                                        mtxPhoto.unlock();
#endif
                                           pCallbackBuf->uncopied = false;
                                           rc = RC_OK;
                                    }
                            }
                            else {   // restore 220919  if (HWTriggered)
                                    //IS457
                                    lock_guard<std::mutex> lgMtxPhoto{mtxPhoto};
#ifdef _DEBUG_MUTEX   //GOI-5-0
                                    qDebug() << "GOI-5-0";
#endif
                                    if (mvCallbackContext->pPhotoQTail != mvCallbackContext->pPhotoQHead)
                                    {
                                            //int lastHead = (*mvCallbackContext->pPhotoQHead + IMAGE_QUEUE_SIZE - 1) % IMAGE_QUEUE_SIZE;
                                            TQueuePhoto *pLastPhoto = mvCallbackContext->pPhotoQueues + *mvCallbackContext->pPhotoQTail * 2 + theDevice;
                                            if (!pLastPhoto->photoImage.empty())
                                            {
                                                    pLastPhoto->photoImage.copyTo(OCvImage);
                                                    rc = RC_OK;
                                            }
                                            //sPhotoQTail  = (sPhotoQTail + 1) % IMAGE_QUEUE_SIZE;  //HWTSaveFileHandler
                                    }
                                    //MTM_SOFTWARE_TRIGGER,
                                    tSdkFrameHead  frameHead;       //图像帧头信息
                                    BYTE *pRawBuffer = nullptr, *pRgbBuffer = nullptr;

                                    goto skip;
/*
                                    memset(&frameHead, 0, sizeof(tSdkFrameHead));
                                    if ((rc = CameraGetImageBuffer(mvCamerahandles[theDevice],
                                                                                    &frameHead, &pRawBuffer,
                                                                                    MAX_EXPOSURE_ms)) == CAMERA_STATUS_SUCCESS)
                                    {
                                           if ((rc = CameraImageProcess(mvCamerahandles[theDevice],
                                                                                        pRawBuffer, pRgbBuffer,
                                                                                        &frameHead)) == CAMERA_STATUS_SUCCESS)
                                                   if(frameHead.uiMediaType==CAMERA_MEDIA_TYPE_MONO8)
                                                   {
                                                           //QImage img(pRgbBuffer, frameHead.iWidth, frameHead.iHeight,QImage::Format_Indexed8);
#ifdef _DEBUG_MUTEX   //GOI-7-0
                                                       qDebug() << "GOI-7-0";
#endif
#ifdef LCK_GRD
                                                                lock_guard<std::mutex> lgMtxPhoto{mtxPhoto};
#else
                                                                mtxPhoto.lock();  //cmt
#endif
XX                                                                    qDebug() << "LG-3-1";
                                                            if (OCvImage .empty())
                                                                    OCvImage  = cv::Mat(frameHead.iHeight, frameHead.iWidth, CV_8UC1);
                                                            memcpy(OCvImage.ptr(), pRgbBuffer, frameHead.iWidth * frameHead.iHeight);
#ifndef LCK_GRD
                                                            mtxPhoto.unlock();
#endif
                                                   }
                                           CameraReleaseImageBuffer(mvCamerahandles[theDevice], pRawBuffer);
                                   }
/**/
                                   skip:;
                            }
                    } catch (const GenericException& e) {
                        cerr << "An exception 2-2 occurred." << endl << e.GetDescription() << endl;
                        return -1;
                    }
                    break;
        case Camera_Mars:  break;
        }
        return rc;
}

int nsCameras::GrabOnePairImage(
        TProcessPhoto *photopair,
        mutex& mtxPhoto,
        bool HWTriggered,
        sCallbackContext *callbackContext,
        bool CaptureLastInQ
        )
{
        int rc = RC_LOCK_GUARDED;
        //sCallbackContext *mvCallbackContext = (sCallbackContext *) callbackContext;

        if (HWTriggered)
        {
                lock_guard<std::mutex> lgMtxPhoto{mtxPhoto};
#ifdef _DEBUG_MUTEX   //GOP-1
                qDebug() << "GOP-1-0";
#endif
                int qTail = -1;
                if (*callbackContext->pPhotoQTail != *callbackContext->pPhotoQHead)
                        qTail = *callbackContext->pPhotoQTail;
                else if (CaptureLastInQ)
                        qTail = (*callbackContext->pPhotoQTail +  IMAGE_QUEUE_SIZE - 1) % IMAGE_QUEUE_SIZE;
                if (qTail >= 0)
                {
                        TQueuePhoto *pLastPhoto = callbackContext->pPhotoQueues + qTail * 2; //LR
                        bool bothOK = true;
                        for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++, pLastPhoto++)
                                if (pLastPhoto->photoImage.empty())
                                {
                                        bothOK = false;
                                        qDebug() << "QTl:" << qTail << ", C" << nthDevice << "empty!";
                                } else
                                        pLastPhoto->photoImage.copyTo(*((photopair + nthDevice)->pPhotoImage));

                        if (*callbackContext->pPhotoQTail != *callbackContext->pPhotoQHead)
                                 *callbackContext->pPhotoQTail  = (*callbackContext->pPhotoQTail + 1) % IMAGE_QUEUE_SIZE;  //HWTSaveFileHandler
                        if (bothOK)
                        {
                            //moved up 0117 *mvCallbackContext->pPhotoQTail  = (*mvCallbackContext->pPhotoQTail + 1) % IMAGE_QUEUE_SIZE;  //HWTSaveFileHandler
//if (*mvCallbackContext->pPhotoQTail  > *mvCallbackContext->pPhotoQHead)     qDebug() << "T>H";
                                rc = RC_OK;
                        }
                        else
                                rc = RC_NO_IMAGE;
                }

        }

#ifdef MISSION_XS
need to re-write this  part, to fullfil the direct-queue of MV
#endif
        CGrabResultPtr pGrabbedResult;
        switch(CameraBrand)
        {
        case Camera_Basler:  //GrabOnePairImage
                for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice ++)
                {
                    try {
                            if (cameras[nthDevice].IsGrabbing()) //Open())
                            {
                                    // interfere with timer ?
                                    if (!HWTriggered)
                                            //cameras[nthDevice].StartGrabbing(1);
                                            cameras[nthDevice].RetrieveResult(  MAX_EXPOSURE_ms,
                                                                                                        pGrabbedResult,
                                                                                                        TimeoutHandling_ThrowException);
                            /* else
                                    cameras[nthDevice].GrabOne( MAX_EXPOSURE_ms, //  M# 1600 for MV?!  2000, //1000, //500 not enough for expossure 350ms?
                                                                                                pGrabbedResult,
                                                                                                TimeoutHandling_ThrowException);
                            */
                            }
                            else
                                    rc = RC_NO_IMAGE;
                    } catch (const GenericException& e) {
                            cerr << "An exception 2-1 occurred." << endl << e.GetDescription() << endl;
                            return RC_Exception;
                    }

                    try {
                            if (pGrabbedResult->GrabSucceeded())
                                    rc = OnPhotosCaptured(  //GrabOnePairImage-Basler
                                                        nthDevice, 0,
                                                            (void *) &pGrabbedResult, (sCallbackContext *) callbackContext,
                                                            photopair);
#ifdef SKIP_OLD
        #ifdef PYLON_WIN_BUILD
                       // Pylon::DisplayImage( 1, pGrabbedResult);
            #endif
                        int frameCols = pGrabbedResult->GetWidth();
                        int frameRows = pGrabbedResult->GetHeight();
                        int frameSize = frameCols * frameRows;
                                qDebug() << "LG-1-0";
#ifdef LCK_GRD
                                xx lock_guard<std::mutex> lgMtxPhoto{mtxPhoto};
#else
                                mtxPhoto.lock();  //basler pGrabbedResult
#endif

                                qDebug() << "LG-1-1";
                        if (OCvImage .empty())
                            OCvImage  = cv::Mat(frameRows, frameCols, CV_8UC1);
                       memcpy(OCvImage.ptr(), (uint8_t *) pGrabbedResult->GetBuffer(), frameSize);
#ifndef LCK_GRD
                                mtxPhoto.unlock();
#endif
                      // imwrite("C:\\GrabbedImageCV.png",  OCvImage ); //theFrame);
                                rc = RC_OK;
#endif
                            } catch (const GenericException& e) {
                                cerr << "An exception 2-2 occurred." << endl << e.GetDescription() << endl;
                                return RC_Exception;
                            }
                    } //for
                    break;
        case Camera_MV:  //GrabOnepairImage
                    try {
                            sCallbackContext *mvCallbackContext = (sCallbackContext *) callbackContext;
#ifdef MOVED_ABOVE
                                    //IS457
                                    //230117
                                lock_guard<std::mutex> lgMtxPhoto{mtxPhoto};
#ifdef _DEBUG_MUTEX   //GOP-1
                                    qDebug() << "GOP-1-0";
#endif
                                    int qTail = -1;
                                    if (*mvCallbackContext->pPhotoQTail != *mvCallbackContext->pPhotoQHead)
                                            qTail = *mvCallbackContext->pPhotoQTail;
                                    else if (CaptureLastInQ)
                                            qTail = (*mvCallbackContext->pPhotoQTail +  IMAGE_QUEUE_SIZE - 1) % IMAGE_QUEUE_SIZE;
                                    if (qTail >= 0)
                                    {
                                            TQueuePhoto *pLastPhoto = mvCallbackContext->pPhotoQueues + qTail * 2; //LR
                                            bool bothOK = true;
                                            for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++, pLastPhoto++)
                                                    if (pLastPhoto->photoImage.empty())
                                                    {
                                                            bothOK = false;
                                                            qDebug() << "QTl:" << qTail << ", C" << nthDevice << "empty!";
                                                    } else
                                                            pLastPhoto->photoImage.copyTo(*((photopair + nthDevice)->pPhotoImage));

                                            if (*mvCallbackContext->pPhotoQTail != *mvCallbackContext->pPhotoQHead)
                                                     *mvCallbackContext->pPhotoQTail  = (*mvCallbackContext->pPhotoQTail + 1) % IMAGE_QUEUE_SIZE;  //HWTSaveFileHandler
                                            if (bothOK)
                                            {
                                                //moved up 0117 *mvCallbackContext->pPhotoQTail  = (*mvCallbackContext->pPhotoQTail + 1) % IMAGE_QUEUE_SIZE;  //HWTSaveFileHandler
//if (*mvCallbackContext->pPhotoQTail  > *mvCallbackContext->pPhotoQHead)     qDebug() << "T>H";
                                                    rc = RC_OK;
                                            }
                                            else
                                                    rc = RC_NO_IMAGE;
                                    }
                                    goto skip;
#endif
//#ifndef DIRECT_QUEUE
                            for (int nthDevice = 0; nthDevice < C_MaxCamerasToUse; nthDevice++)
                            {
                                    //MTM_SOFTWARE_TRIGGER,
                                    tSdkFrameHead  frameHead;       //图像帧头信息
                                    BYTE *pRawBuffer = nullptr, *pRgbBuffer = nullptr;

                                    memset(&frameHead, 0, sizeof(tSdkFrameHead));
                                    if ((rc = CameraGetImageBuffer(mvCamerahandles[nthDevice],
                                                                                    &frameHead, &pRawBuffer,
                                                                                    mvCallbackContext->cameraExposures[nthDevice] * 2
                                                                                    )) == CAMERA_STATUS_SUCCESS)
                                    {
                                            if (photopair[nthDevice].pPhotoImage->empty())
                                                    switch(frameHead.uiMediaType)
                                                    {
                                                    case CAMERA_MEDIA_TYPE_MONO8:
                                                                    *photopair[nthDevice].pPhotoImage  = cv::Mat(frameHead.iHeight, frameHead.iWidth, CV_8UC1);
                                                                    break;
                                                    }

                                            //qDebug() << "LG-3-0";
#ifdef LCK_GRD
                                            lock_guard<std::mutex> lgMtxPhoto{mtxPhoto};
#else
                                            mtxPhoto.lock();  //cmt
#endif
                                               // qDebug() << "LG-3-1";
                                           rc = CameraImageProcess(mvCamerahandles[nthDevice],
                                                                                        pRawBuffer,
                                                                                        photopair[nthDevice].pPhotoImage->ptr(), //pRgbBuffer,
                                                                                        &frameHead); // == CAMERA_STATUS_SUCCESS)
                                                   /*if(frameHead.uiMediaType==CAMERA_MEDIA_TYPE_MONO8)
                                                   {
                                                           //QImage img(pRgbBuffer, frameHead.iWidth, frameHead.iHeight,QImage::Format_Indexed8);
                                                            memcpy(photopair[nthDevice].pPhotoImage->ptr(), pRgbBuffer, frameHead.iWidth * frameHead.iHeight);
                                                   }*/
#ifndef LCK_GRD
                                                            mtxPhoto.unlock();
#endif
                                           CameraReleaseImageBuffer(mvCamerahandles[nthDevice], pRawBuffer);
                                   }
                            }
   skip:;
//#endif
                                    /**/
                    } catch (const GenericException& e) {
                        cerr << "An exception 2-2 occurred." << endl << e.GetDescription() << endl;
                        return -1;
                    }
                    break;
        //case Camera_Mars:  break;
        default:
                    rc = RC_LACK_CAMERAS;
        }
        return rc;

}

void nsCameras::mvGrabCallback(
            int theDevice,
            CameraHandle hCamera,
            BYTE *pFrameBuffer,
            tSdkFrameHead* pFrameHead,
            PVOID pContext)
    {
            //if (!theDevice) qDebug() << "LCB\n"; else qDebug() << "RCB\n";
            if (!hCamera || !pFrameBuffer || !pFrameHead)  //pAlignBuf ||
                        return;
            sCallbackContext *mvContext= (sCallbackContext *) pContext;

//#ifndef SMLT_HWT
            if (!mvContext->TibboOn)   //IS777
            {
//qDebug() << "Tibbo NOT ON";
                    return;
            }
//qDebug() << "Tibbo ON";


            OnPhotosCaptured( //mvGrabCallback
                        theDevice, hCamera, pFrameBuffer, mvContext, pFrameHead);
            return ;

            //mvContext->pPhotoQMtx->unlock();
//#endif
            //sCallbackBuf *pAlignBuf=  &mvContext->mvCallbackBuf[theDevice];// (sCallbackBuf *) pContext;

                // The original data obtained will be converted into RGB format data, and meanwhile through the ISP module, the image will be processed with noise reduction, edge enhancement and color correction.
                // Most of our company's cameras, the original data are Bayer format
            //CameraSdkStatus status = CameraImageProcess(hCamera, pFrameBuffer, pAlignBuf->alignedBuf, pFrameHead);
            sCallbackBuf *pCallbackBuf = &mvContext->mvCallbackBuf[theDevice];
            QElapsedTimer &railTimer = *mvContext->pRailTimer;
            ulong &TriggerStart = *mvContext->pTriggerStart;
            int &LastRailGap = *mvContext->pLastRailGap;
            ulong elapsed;
            if (LastRailGap >= 0)  //o.w
            {
                    if (!LastRailGap)  //o.w precede handler
                            LastRailGap = railTimer.nsecsElapsed() - TriggerStart;  //IS834 try
                    elapsed  = LastRailGap;
                    LastRailGap = -1;
            } else
                    elapsed  = (railTimer.nsecsElapsed() - TriggerStart);
//#ifdef  OS_LINUX
            elapsed  /= nsUtility::ns2ms;
//#endif
#ifdef CB_NEED_MTX
//XX            while (!mvContext->pPhotoQMtx->try_lock())  //mvContext->pPhotoQMtx->lock(); //mvCallbackBuf
            {
                    qDebug() << "MVGC FoL";
                    QThread::msleep(5);
                    //QApplication::processEvents();
            }
#endif
            TQueuePhoto *pQueueEntity;
            int thisHead = *mvContext->pPhotoQHead;
            int nextHead = (thisHead + 1) % IMAGE_QUEUE_SIZE;
            switch(mvContext->OP_Mode)
            {
            case OM_CALIBRATE_CHECKERBOARD: //moved up here 230117
            case OM_CALIBRATE_SQUARE_BAR:

            case OM_IDLE:
            case OM_FETCH_RAIL_CROWN:
                                if ( *mvContext->pPhotoQHead != *mvContext->pPhotoQTail) //empty
                                {
                                        thisHead = (thisHead + IMAGE_QUEUE_SIZE - 1) % IMAGE_QUEUE_SIZE;
                                        nextHead = (nextHead + IMAGE_QUEUE_SIZE - 1) % IMAGE_QUEUE_SIZE;
                                }
                                    break;
//                                break;
            }
                static bool fullShown = false;
                if ( nextHead == *mvContext->pPhotoQTail) //full
                {
                        if (!fullShown)
                                qDebug() << "************** PQ-Full H=>T=" << nextHead << "******************", fullShown = true;
                } else {
                        fullShown = false;
                        pQueueEntity = mvContext->pPhotoQueues
                                                            + 2 * thisHead/* *mvContext->pPhotoQHead */
                                                            + theDevice;

                        duration<double, milli> d(0);
                        auto t1 = high_resolution_clock::now();
                        CameraSdkStatus rss;
#ifdef ROI_IMAGE                 //mvGrabCallback
                        const  bool IsRoiImage = true;
#else
                        const  bool IsRoiImage = false;
#endif
                        switch(mvContext->OP_Mode)
                        {
                        //220111  case OM_CALIBRATE_CHECKERBOARD:
                        case OM_CALIBRATE_SQUARE_BAR:
                        case OM_FETCH_RAIL_CROWN:  //221110
                                    if (IsRoiImage)
                                    {
                                            Mat rawFrameFull(pCallbackBuf->frameRows, pCallbackBuf->frameCols, CV_8UC1);
                                                        //pQueueEntity->photoImage.create(pCallbackBuf->frameRows, pCallbackBuf->frameCols, CV_8UC1);
                                            if ((rss = CameraImageProcess(hCamera, pFrameBuffer, rawFrameFull.ptr(), pFrameHead)) == CAMERA_STATUS_SUCCESS)
                                            {
                                                        //if (!theDevice) imwrite(DEBUG_PATH + "$Q0Org_" + to_string(thisHead) +".bmp",  rawFrameFull); //
                                                            //Mat rawFrameFull(pCallbackBuf->frameRows, pCallbackBuf->frameCols, CV_8UC1, pFrameBuffer);
                                                    pQueueEntity->photoImage.create(RAW_LASER_ROI_HEIGHT, RAW_LASER_ROI_WIDTH, CV_8UC1);
                                                    pQueueEntity->photoImage.setTo(0);
                                                    //rawFrameROI .create(RAW_LASER_ROI_HEIGHT, RAW_LASER_ROI_WIDTH, CV_8UC1);
                                                    Rect roi(RAW_LASER_ROI[mvContext->Box_ID - 1][theDevice], Size(RAW_LASER_ROI_WIDTH, RAW_LASER_ROI_HEIGHT)); //mvGrabCallback before rotate
                                                    roi.height = min(roi.height, rawFrameFull.rows - roi.y); //  (int) RAW_LASER_ROI_HEIGHT); ///*rawFrameROI.rows*/ ),
                                                    roi.width = min(roi.width, rawFrameFull.cols - roi.x); //(int) RAW_LASER_ROI_WIDTH); //rawFrameROI.cols);
                                                    rawFrameFull(roi).copyTo(pQueueEntity->photoImage);
                        /*
                                                                    Mat rawFrameROI(RAW_LASER_ROI_HEIGHT, RAW_LASER_ROI_WIDTH, CV_8UC1);// = Mat::zeros(RAW_LASER_ROI_HEIGHT, RAW_LASER_ROI_WIDTH, CV_8UC1);
                                                                    rawFrameROI.setTo(0);
                        XX                                            rawFrameFull(roi).copyTo(rawFrameROI);

                                                                    ///if (pQueueEntity->photoImage.empty()) pQueueEntity->photoImage = cv::Mat(RAW_LASER_ROI_HEIGHT, RAW_LASER_ROI_WIDTH, CV_8UC1);
                                                                    tSdkFrameHead frameRoiHead = *pFrameHead;
                                                                    frameRoiHead.iWidth = RAW_LASER_ROI_WIDTH, frameRoiHead.iHeight = RAW_LASER_ROI_HEIGHT;
                                                                    rss = CameraImageProcess(hCamera, rawFrameROI.ptr(), pQueueEntity->photoImage.ptr(), &frameRoiHead);
                                                                    rawFrameROI.release();
                        */
                                            }
                                            rawFrameFull.release();
//#ifdef _DEBUG_FILE
                                    //if (!theDevice)
                                    //        imwrite(DEBUG_PATH + "$Q"+ to_string(theDevice) + "_Roi_" + to_string(thisHead) +".bmp",  pQueueEntity->photoImage); //sPhotoes[nthDevice]); //*photo );
//#endif
                                            break;
                                    }
                        case OM_CALIBRATE_CHECKERBOARD:  //220111
                        case OM_IDLE:
                                            //if (pQueueEntity->photoImage.empty())
                                            //        pQueueEntity->photoImage = cv::Mat(pCallbackBuf->frameRows, pCallbackBuf->frameCols, CV_8UC1);
                                            pQueueEntity->photoImage.create(pCallbackBuf->frameRows, pCallbackBuf->frameCols, CV_8UC1);
//LKG_81
                                            rss = CameraImageProcess(hCamera, pFrameBuffer, pQueueEntity->photoImage.ptr(), pFrameHead);  // CameraSetIspOutFormat 8=>8
                                            break;
                        }
//#endif
                        #ifdef _DEBUG
                                d = high_resolution_clock::now() - t1;
                                //qDebug() << "MVGC CIP: " << d.count();
                                t1 = high_resolution_clock::now();
                        #endif
                        if ( rss  == CAMERA_STATUS_SUCCESS)
                        {
                    /*
XX                     * #ifdef _DEBUG <2ms
                             duration<double, milli> d = high_resolution_clock::now() - t1;
                            qDebug() << "CB " << theDevice << " CIP: " << d.count();
                    //#endif */
                                pQueueEntity->timeStamp  =  elapsed;

                                TQueuePhoto *pQTwinEntity = (!theDevice) ? pQueueEntity  + 1 : pQueueEntity - 1;
                                int offset =  pQueueEntity->timeStamp - pQTwinEntity ->timeStamp;  offset = abs(offset);
#ifdef _DEBUG_SHOT_TmSmp
                                        //*
                                /*TQueuePhoto *pQTwinEntityP1 = pQueueEntity  + 1;
                                TQueuePhoto *pQTwinEntityN1 = pQueueEntity - 1;*/
                                        //qDebug() << "C" << theDevice <<", P1=" << pQTwinEntityP1->timeStamp << ", N1=" << pQTwinEntityN1->timeStamp;
                                        qDebug() << "C" << theDevice <<", QH" << *mvContext->pPhotoQHead << ":"
                                                 << pQueueEntity->timeStamp << "," << pQTwinEntity ->timeStamp << "," << offset;
                                        /**/
                                        //qDebug() << "L-R offset=" << offset << "\n";
#endif
                                if (  (mvContext->OP_Mode == OM_CALIBRATE_CHECKERBOARD)  //0111
                                     ||
                                        (offset  <
        #ifdef _DEBUG
                                        CAPTURED_OFFSET_TLRN_SWT
        #else
                                        CAPTURED_OFFSET_TLRN_HWT
        #endif
                                    ) )
                                {
                                        switch(mvContext->OP_Mode)
                                        {
                                        case OM_IDLE:
                                        case OM_FETCH_RAIL_CROWN:
                                                        if (*mvContext->pPhotoQHead != *mvContext->pPhotoQTail)  //keep 1
                                                                break;
                                        case OM_CALIBRATE_CHECKERBOARD:
                                        case OM_CALIBRATE_SQUARE_BAR:
                                                        *mvContext->pPhotoQHead = nextHead; //(*mvContext->pPhotoQHead + 1) % IMAGE_QUEUE_SIZE;
#ifdef _DEBUG_SHOT_TmSmp
                                           qDebug() << "Q Head =>" << *mvContext->pPhotoQHead  << "@"  << pQueueEntity->timeStamp << ", Tail=" << *mvContext->pPhotoQTail;
#endif
                                        }
                                } else if (offset < 111)  //dump both
                                {
                                            pQueueEntity->timeStamp  = pQTwinEntity ->timeStamp = 11;
                                            qDebug() << "C" << theDevice <<", QH" << *mvContext->pPhotoQHead << ":" << offset << ", dropped....";
                                }
                        }
#ifdef ROI_IMAGE
//                       rawFrameROI.release();
#endif
                }
#ifdef CB_NEED_MTX
           mvContext->pPhotoQMtx->unlock();

#endif
           //if (!fullShown)
                //CameraSdkStatus status = CameraImageProcess(hCamera, pFrameBuffer, pQueueEntity->photoImage.ptr(), pFrameHead);
           //if (status == CAMERA_STATUS_SUCCESS)
                   //qDebug() << "Image " << theDevice << " into Q @ " <<  pQueueEntity->timeStamp  ;

            CameraReleaseImageBuffer(hCamera, pFrameBuffer);

            //if (status == CAMERA_STATUS_SUCCESS)
             //       pAlignBuf->uncopied = true;

            //mtxCallback.unlock();
    }

void nsCameras::mvGrabCallback0(CameraHandle hCamera, BYTE *pFrameBuffer, tSdkFrameHead* pFrameHead, PVOID pContext)
{
        //qDebug() << "LCB\n";;
        mvGrabCallback(0, hCamera, pFrameBuffer, pFrameHead, pContext);
}
void nsCameras::mvGrabCallback1(CameraHandle hCamera, BYTE *pFrameBuffer, tSdkFrameHead* pFrameHead,PVOID pContext)
{
        //qDebug() << "RCB\n";;
        mvGrabCallback(1, hCamera, pFrameBuffer, pFrameHead, pContext);
        return;

        sCallbackContext *mvContext= (sCallbackContext *) pContext;
        if (!mvContext->TibboOn)
                return;
        qDebug() << "RCB\n";
        sCallbackBuf *pAlignBuf=  &mvContext->mvCallbackBuf[1];// (sCallbackBuf *) pContext;

        if (!hCamera || !pFrameBuffer || !pAlignBuf || !pFrameHead)
                    return;
        CameraSdkStatus status = CameraImageProcess(hCamera, pFrameBuffer, pAlignBuf->alignedBuf, pFrameHead);
        //GrabOneImage();
        CameraReleaseImageBuffer(hCamera, pFrameBuffer);
        if (status == CAMERA_STATUS_SUCCESS)
                pAlignBuf->uncopied = true;
}

int nsCameras::StartCapturing(int theDevice)
{
        if ((theDevice < 0) || (theDevice > C_MaxCamerasToUse-1))
            return -1;
        /* Starts grabbing for all cameras starting with index 0.
            The grabbing is started for one camera after the other.
            That's why the images of all cameras are NOT taken at the same time.
            However, a hardware trigger setup can be used to cause all cameras to grab images synchronously.
            According to their default configuration, the cameras are set up for free-running continuous acquisition.
         */
        switch(CameraBrand)
        {
        case Camera_Basler: //StartCapturing
                    try {
                            if (!cameras[theDevice].IsPylonDeviceAttached())
                                return RC_LACK_CAMERAS;

                            cameras[theDevice].StartGrabbing();
                    } catch (const GenericException& e) {
                        cerr << "An exception 2-2 occurred." << endl << e.GetDescription() << endl;
                        return -1;
                    }
                    break;
        case Camera_MV:  //StartCapturing
                    try {
                            /* CameraPlay moved earlier
                            BOOL opened;
                            if (CameraIsOpened(&mvCameraList[theDevice], &opened) == CAMERA_STATUS_SUCCESS)
                                    if (opened)
                                            CameraPlay(mvCamerahandles[theDevice]);
                             */
                    } catch (...) {
                            return RC_FAIL_TO_OPEN;
                    }
                    break;
        }

        return 0;
}

int nsCameras::GetCameraExposure(int theDevice, double& usNewExp)
{
    switch(CameraBrand)
    {
    case Camera_Basler: //GetCameraExposure
                    try {
                            usNewExp = cameras[theDevice].ExposureTimeRaw.GetValue();
                            return RC_OK;
                    } catch (...) {
                            return RC_FAIL_TO_OPEN;
                    }
                    break;
      case Camera_MV: //GetCameraExposure
                    try {
                            if (CameraGetExposureTime(mvCamerahandles[theDevice], &usNewExp) == CAMERA_STATUS_SUCCESS)
                                    usNewExp *= 1000;
                                    return RC_OK;
                    } catch (...) {
                                    return RC_FAIL_TO_OPEN;
                    }
                break;
       }
    return RC_FAIL_TO_OPEN;
}


int nsCameras::TuneCameraExposure(int theDevice, double usNewExp, double* usResultExp)
{
    int rc = RC_FAIL_TO_WRITE; //basler not handled yet
    switch(CameraBrand)
    {
    case Camera_Basler: //TuneCameraExposure
                if (pylonDevices[theDevice])
                        if (cameras[theDevice].IsOpen())
                                try {
                                        //int rc = cameras[theDevice].ExposureMode.GetValue();
                                        //if (rc != ExposureMode_Timed)
                                                //cameras[theDevice].ExposureMode.SetValue(ExposureMode_Timed);
                                                    /*
                                                            ExposureMode_Off,  //!< Sets the exposure mode to off - Applies to: CameraLink and GigE
                                                            ExposureMode_Timed,  //!< The exposure mode is set to Timed - Applies to: CamEmu, CameraLink, GigE, ace 2 GigE, ace 2 USB, ace USB, boost CoaXPress, dart 2 USB, dart BCON, dart USB and pulse USB
                                                            ExposureMode_TriggerControlled,  //!< The exposure mode is set to Trigger Controlled - Applies to: CameraLink, GigE, ace 2 GigE, ace 2 USB, boost CoaXPress and dart 2 USB
                                                            ExposureMode_TriggerWidth  //!< The exposure mode is set to Trigger Width - Applies to: CameraLink, GigE, ace 2 GigE, ace 2 USB, ace USB, boost CoaXPress, dart 2 USB, dart BCON and dart USB
                                                      */

                                        //if (cameras[theDevice].IsOpen())
                                        //        rc = cameras[theDevice].ExposureAuto.GetValue();
                                        //if (rc != ExposureAutoEnums::ExposureAuto_Off)
                                                cameras[theDevice].ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Off);
                                                    /* ExposureAuto_Continuous,  //!< The exposure time is adjusted continuously while images are being acquired - Applies to: CameraLink, GigE, ace 2 GigE, ace 2 USB, ace USB, boost CoaXPress, dart 2 USB, dart BCON, dart USB and pulse USB
                                                        ExposureAuto_Off,  //!< Automatic exposure time adjustment is disabled - Applies to: CameraLink, GigE, ace 2 GigE, ace 2 USB, ace USB, boost CoaXPress, dart 2 USB, dart BCON, dart USB and pulse USB
                                                        ExposureAuto_Once  //!< The exposure time is adjusted automatically to reach the specified target value - Applies to: CameraLink, GigE, ace 2 GigE, ace 2 USB, ace USB, boost CoaXPress, dart 2 USB, dart BCON, dart USB and pulse USB
                                                      */

                                        usNewExp = trunc(usNewExp / 35) * 35;
                                        cameras[theDevice].ExposureTimeRaw.TrySetValue(usNewExp);  //Raw Abs

                                        if (usResultExp)
                                            GetCameraExposure(theDevice, *usResultExp);

                                        //double d = cameras[nthDevice].ExposureTimeRaw.GetValue();
                                        //                qDebug() << d ;
                                                    /*
                                                        ExposureTimeMode_Standard,  //!< The exposure time mode is set to Standard - Applies to: GigE and ace USB
                                                        ExposureTimeMode_UltraShort  //!< The exposure time mode is set to Ultra Short - Applies to: GigE and ace USB
                                                      */
                                                    //cameras[nthDevice].ExposureTimeMode.SetValue(ExposureTimeMode_UltraShort);

                                        rc = RC_OK;
                                } catch (const GenericException& e) {
                                        cerr << "An exception 3 occurred." << endl << e.GetDescription() << endl;
                                }
            break;
    case Camera_MV: //TuneCameraExposure
                try {
                        if ((rc = CameraSetExposureTime(mvCamerahandles[theDevice], usNewExp)) == CAMERA_STATUS_SUCCESS) // / 1000);
                                if (usResultExp)
                                        rc = CameraGetExposureTime(mvCamerahandles[theDevice], usResultExp);
                } catch (const GenericException& e) {
                        cerr << "An exception 3 occurred." << endl << e.GetDescription() << endl;
                }
    break;
    }
    return rc;
}

int nsCameras::SetCameraTriggerMode(int theDevice, const ETriggerMode tm)
{  //if stop to debug here, SetValue(TriggerSelector_FrameStart) will close the camera!!
    switch(CameraBrand)
    {
    case Camera_Basler:  //SetCameraTriggerMode
                    if (pylonDevices[theDevice])
                            if (cameras[theDevice].IsOpen())
                                    try {
                                                //cameras[theDevice].StopGrabbing();
                                                cameras[theDevice].TriggerSelector.SetValue(TriggerSelector_FrameStart);
                                                    /*
                                                        TriggerSelector_AcquisitionActive,  //!< Selects the acquisition active trigger for configuration - Applies to: CameraLink and GigE
                                                        TriggerSelector_AcquisitionEnd,  //!< Selects the acquisition end trigger for configuration - Applies to: CameraLink and GigE
                                                        TriggerSelector_AcquisitionStart,  //!< The Acquisition Start trigger can be configured - Applies to: CamEmu, CameraLink and GigE
                                                        TriggerSelector_ExposureActive,  //!< The Exposure Active trigger can be configured - Applies to: CameraLink, GigE, ace 2 GigE, ace 2 USB, boost CoaXPress and dart 2 USB
                                                        TriggerSelector_ExposureEnd,  //!< The Exposure End trigger can be configured - Applies to: CameraLink, GigE, ace 2 GigE, ace 2 USB, boost CoaXPress and dart 2 USB
                                                        TriggerSelector_ExposureStart,  //!< The Exposure Start trigger can be configured - Applies to: CameraLink, GigE, ace 2 GigE, ace 2 USB, boost CoaXPress and dart 2 USB
                                                        TriggerSelector_FrameActive,  //!< The Frame Active trigger can be configured - Applies to: CameraLink, GigE, ace 2 GigE, ace 2 USB, boost CoaXPress and dart 2 USB
                                                        TriggerSelector_FrameBurstActive,  //!< The Frame Burst Active trigger can be configured - Applies to: ace 2 GigE, ace 2 USB, boost CoaXPress and dart 2 USB
                                                        TriggerSelector_FrameBurstEnd,  //!< The Frame Burst End trigger can be configured - Applies to: ace 2 GigE, ace 2 USB, boost CoaXPress and dart 2 USB
                                                        TriggerSelector_FrameBurstStart,  //!< The Frame Burst Start trigger can be configured - Applies to: ace 2 GigE, ace 2 USB, ace USB, boost CoaXPress and dart 2 USB
                                                        TriggerSelector_FrameEnd,  //!< The Frame End trigger can be configured - Applies to: CameraLink, GigE, ace 2 GigE, ace 2 USB, boost CoaXPress and dart 2 USB
                                                        TriggerSelector_FrameStart,  //!< The Frame Start trigger can be configured - Applies to: CameraLink, GigE, ace 2 GigE, ace 2 USB, ace USB, blaze, boost CoaXPress, dart 2 USB, dart BCON, dart USB and pulse USB
                                                        TriggerSelector_LineStart  //!< The Line Start trigger can be configured - Applies to: CameraLink and GigE
                                                 */

                //                                    cameras[theDevice].TriggerMode.SetValue(TriggerMode_On);

                                                switch(tm)
                                                {
                                                TriggerMode_Software:
                                                                                                        cameras[theDevice].AcquisitionMode.SetValue(AcquisitionMode_Continuous);
                                                                                                        cameras[theDevice].TriggerSource.SetValue("Trigger Software"); //("Action1");
                                                                                                        break;
                                                TriggerMode_Hardware:       cameras[theDevice].TriggerSource.SetValue("Line1");   break;
                                                }
                                    } catch (const GenericException& e) {
                                            cerr << "An exception 4 occurred." << endl << e.GetDescription() << endl;
                                            return -1;
                                    }
                    break;
    case Camera_MV:  //SetCameraTriggerMode
                try {
                        /*  [in] iModeSel   模式选择索引号。
                         *          0: 连续采集；
                         *          1: 软件触发；
                         *          2: 硬件触发（线阵为帧触发）；
                         *          3: 行触发（编码器触发）（仅线阵）；
                         *          4: 条件行触发（仅线阵） */
                        int curTMVal = -1, newTMVal =  (tm == TriggerMode_Hardware) ?  MTM_HARDWARE_TRIGGER
                                                                                                                              : MTM_CONTINUOUS;
                        CameraGetTriggerMode(mvCamerahandles[theDevice],  &curTMVal);
                        if (curTMVal != newTMVal)
                                CameraSetTriggerMode(mvCamerahandles[theDevice], newTMVal) ;

                    } catch (const GenericException& e) {
                        cerr << "An exception 4 occurred." << endl << e.GetDescription() << endl;
                        return -1;
                    }
                    break;
    }
    return 0;
}

using namespace  nsCameras;
Rect nsImageProcess::GetRectifiedROI(const uint nthBox, const uint nthCamera, const EOperateMode opMode)
{
        Rect roi;
        switch(nthCamera)
        {
        case nsCameras::Camera_Left_0:  roi = Rect(     ROI_SE_LEFT_X[nthBox], // ROI_LEFT_X[BOX_ID-1][opMode], // 5200, //RectifyRatioY=0.7  5100 if RectifyRatioY=0.6 7100,
                                                                                                    ROI_SE_Y_TOP[nthBox], //ROI_Y_TOP[BOX_ID-1][opMode],
                                                                                                    ROI_SE_WIDTH, ROI_SE_HEIGHT );
                                                                        break;
        case nsCameras::Camera_Right_1: roi = Rect(    ROI_SE_RIGHT_X[nthBox], //[opMode], //1400, //RectifyRatioY=0.7 1600 as RectifyRatioY=0.6 ,  //3500,
                                                                                                    ROI_SE_Y_TOP[nthBox], //[opMode],
                                                                                                    ROI_SE_WIDTH, ROI_SE_HEIGHT);
                                                                        break;
        }
        if (opMode == OM_FETCH_RAIL_CROWN)
        {
                roi.x -= SE_VH_OFFSET_X;
                roi.y -= SE_VH_OFFSET_Y,
                roi.x = max(0, roi.x), roi.y = max(0, roi.y); //230320, sorten roi.width?
                roi.width += SE_VH_OFFSET_WIDTH,
                roi.height += SE_VH_OFFSET_HEIGHT;
        }
        return roi;
}

int nsImageProcess::SimpleBinaryThrshold(Mat &image,
                                            const uint threadVal, const bool keepHigh)
{
        if (image.empty())
                return RC_NO_IMAGE;

        if (image.elemSize() > 1)
                return RC_INVALID_DATA;

        int nRows = image.rows;
        int nCols = image.cols;
        if (image.isContinuous())
                nCols *= nRows,  nRows = 1;

        for( int i = 0; i < nRows; i++)
        {
                BYTE *p = image.ptr<uchar>(i);
                for ( int j = 0; j < nCols; j++)
                        if (p[j])
                                p[j] = (p[j] >= threadVal) ? 255 : 0;
        }
        return RC_OK;
        /*
        BYTE *pData = image.ptr( .data;
        if (keepHigh)
                for (size_t i = 0; i < image.total(); i++, pData++ )
                        *pData = (*pData >= threadVal) ? 255 : 0;
        else
                for (size_t i = 0; i < image.total(); i++, pData++ )
                        *pData = (*pData <= threadVal) ? 255 : 0;
                        **/

}
//find the mid-line of an almost-binary laser line,
int nsImageProcess::FindMidline(
                                                                    Mat &OCvImage,
                                                                    const uint nthBox,
                                                                    int nthCamera,
                                                                    EOperateMode opMode,
                                                                    Mat *extWork
                                                                )
{
        constexpr uint SE_THRESH[C_TotalBoxes] = {100, 150, 200, 150,
                               #ifdef _CALIB_AT_NEIHU
                                       100        //white CB
                               #else
                                       150
                               #endif
                                      };
        if (OCvImage.empty())
                return RC_INVALID_DATA;
        String cameraID = nsCameras::GetCameraChar(nthCamera);
        //constexpr THRESHOLD = 200;
#ifdef _DEBUG_FILE
                    imwrite( nsCameras::DEBUG_PATH + "$FM 1_" + cameraID + ".bmp",  OCvImage );
#endif
        uint height = OCvImage.rows;
        uint width = OCvImage.cols;
        Mat *pwork, lwork; // = Mat::zeros(OCvImage.rows, OCvImage.cols, CV_8UC1);
        if (extWork)
        {
#ifdef ORG_THRD
                pwork = extWork;
                //if (!pwork->empty()) pwork->release(), pwork->deallocate();
                if ( (pwork->rows != height) || (pwork->cols != width) )
                        pwork->create(height, width, CV_8UC1); /**/
#else
            pwork = &OCvImage;
#endif
        } else {
                pwork = &lwork;
                pwork->create(  height, //OCvImage.rows,
                                                width, //OCvImage.cols,
                                                CV_8UC1);
        }
//#ifdef LKG_85
//return  RC_OK;//LKG_85
#ifdef ORG_THRD
        threshold(      OCvImage, *pwork,
                                (opMode == OM_CALIBRATE_SQUARE_BAR)
                                                ? SE_THRESH[nthBox]// 200: //221023 for Box-3 SE 100 :
                                                : 50, //square-bar brighter, rail, especially its side, is darker M#
                                255,
                                THRESH_BINARY);
#endif
        int rc = SimpleBinaryThrshold(*pwork,
                                                 (opMode == OM_CALIBRATE_SQUARE_BAR)
                                                 ? SE_THRESH[nthBox]// 200: //221023 for Box-3 SE 100 :
                                       #ifdef _CALIB_AT_NEIHU
                                                  : 40
                                       #else
                                                 : 50 //square-bar brighter, rail, especially its side, is darker M#
                                       #endif
                                            );
        if (rc != RC_OK)
                return rc;
                            #ifdef _DEBUG_FILE
                                            imwrite(nsCameras::DEBUG_PATH + "$FM 4B_" + cameraID + ".bmp",  *pwork);
                            #endif
//#endif
//#ifdef LKG_87
//#ifdef LKG_87
#ifndef _CALIB_AT_NEIHU
        for (uint i = 0; i < 3; i++)
                dilate(*pwork, *pwork, Mat());
#endif
                #ifdef _DEBUG_FILE
                      imwrite(nsCameras::DEBUG_PATH + "$FM-3_" + cameraID + ".bmp",  *pwork );
                #endif
//return  RC_OK;//LKG_87
        //for each row, define a vector of a pair of mid x & width of a "pulse"
        //constexpr uint MAX_ROWS = 10240;
        vector< pair<double, int> > vpMidline[height];
        for (int nthRow = 0; nthRow < height; nthRow++)
                vpMidline[nthRow].clear();

//#ifdef LKG_89
        for (int nthRow = 0; nthRow < height; nthRow++)
        {
#ifdef _DEBUG_FILE
                  //  if (nthRow == 150 )
                   //         imwrite( nsCameras::DEBUG_PATH + "$FM 4_" + cameraID + ".bmp",  work );
#endif
              for (int nthCol = 0, start = -1; nthCol < OCvImage.cols; nthCol++)
                    if (pwork->at<uchar>(nthRow, nthCol) == 255)
                    {
                            if (start < 0)
                                start = nthCol;
                    } else if (start > 0)
                    {
                            uint last = nthCol - start + 1;
                            //pair<double, int>p = {0.2, 3};
                            if (last > 10)  //laser line noise  M# 50
                                    vpMidline[nthRow].push_back( make_pair((nthCol + start) / 2 ,  last) );

                            start = -1;
                    }
        }
//#ifdef LKG_93
        //Mat imgMidline = Mat::zeros(OCvImage.rows, OCvImage.cols, CV_8UC1);
        OCvImage.setTo(0);
        for (int nthRow = 0; nthRow < OCvImage.rows; nthRow++)
                if (!vpMidline[nthRow].empty())
                    if (vpMidline[nthRow].size() == 1)
                    {
                            auto p = vpMidline[nthRow].back(); // .pop_back();
                            OCvImage.at<uchar>(nthRow, get<0>(p)) = 255;
                    } else {
                            int i, sumW, sumXxW, n = vpMidline[nthRow].size();
                            for (i = sumXxW = sumW = 0; i < n; i++)
                            {
                                    auto p = vpMidline[nthRow].back(); //
                                    sumXxW += get<0>(p) * get<1>(p),  sumW += get<1>(p);
                                    vpMidline[nthRow].pop_back();
                            }
                            OCvImage.at<uchar>(nthRow, round(sumXxW / sumW) ) = 255;
                    }
                        #ifdef _DEBUG_FILE
                                if (!OCvImage.empty())
                                        imwrite(nsCameras::DEBUG_PATH + "$FM 5_" + cameraID + ".bmp",  OCvImage );
                        #endif
        for (int nthRow = 0; nthRow < height; nthRow++)
                vpMidline[nthRow].clear();
//#endif

        if (!extWork)
                lwork.release(),   lwork.deallocate();//221024
        return RC_OK;
}

void nsImageProcess::GeneratePhysicalCorners(vector<Point3f>& phyCorners, const SKBGrids& KBGrids)
{
        //constexpr double GRID_DIM_X = 6.22 / 7; //220707  5.48 / 7; //0.8; //1; //0.5;
        //constexpr double GRID_DIM_Y = 4.44 / 5;
        for(uint y{0}; y < KBGrids.vertTotalGrdNum/*gridRowNum CB_INT_CNR_ROW*/; y++)
                for(uint x = 0; x < KBGrids.horzTotalGrdNum /*gridColNum CB_INT_CNR_COL*/ ; x++)
                        phyCorners.push_back( Point3f(x * KBGrids.mmOneGridWidth, //.mmGridWidth, //GRID_DIM_X
                                                                                    y * KBGrids.mmOneGridHeight, //.mmGridHeight, // GRID_DIM_Y,
                                                                                    0) );
}

int nsImageProcess::CalibrateOneCamera(
        const SKBGrids& KBGrids,  //shared across cameras
        const uint whichCamera, //0 or 1
        const string photoDir,
        const uint photoNum,  //say 5, 0L.bmp ~ 4L.bmp;   or 8 0R.bmp ~ 7R.bmp
        const bool rotated90,
        Size& unifiedImageSize,
        vector<vector<cv::Point3f>>& aryPhyCorners,
        vector<vector<cv::Point2f>>& aryImgCorners,
        Mat &intrnscParam,  //(3, 3, CV_32FC1);
        Mat &distortCoeffs,
        Rect &validPxlROI,
        double &reprojError
        )
{
        if (photoDir.empty())
            return RC_FAIL_TO_LOAD;

        const Size CB_INT_CNRS(KBGrids.horzTotalGrdNum,// .gridColNum,
                                                            KBGrids.vertTotalGrdNum // .gridRowNum
                                                        );

        vector<Point3f> phyCorners;            // Defining the world coordinates for 3D points

        GeneratePhysicalCorners(phyCorners, KBGrids);            // Creating vector to store vectors of 3D points for each checkerboard image

        const string side = GetCameraChar((nsCameras::CameraEnum) whichCamera); // (!whichCamera) ? "L": "R" ;
        //Size imgSize;
        for(uint i{1}; i <= photoNum ; i++)
        {
                const String filename = to_string(i) + side + ".bmp";
                Mat photoImage = imread(photoDir + "/" + filename , IMREAD_GRAYSCALE);

                /*if (rotated90)
                        switch(whichCamera)
                        {
                        case nsCameras::Camera_Left_0:
                                                                rotate(photoImage , photoImage , ROTATE_90_CLOCKWISE);
                                                                break;
                        case nsCameras::Camera_Right_1:
                                                                rotate(photoImage , photoImage , ROTATE_90_COUNTERCLOCKWISE);
                                                                break;
                        }
                            imwrite(DEBUG_PATH + "$RT_" + filename,  photoImage );
            */

                duration<double, milli> d(0);

                vector<Point2f> imgCorners;  imgCorners.clear();
                try {
                    auto t1 = high_resolution_clock::now();
                    cout << "Finding corners of " << filename << " ... " ;
                    //photoImage.resize(2748, 3840);
                    //resize(photoImage, photoImage, Size(3640, 2720));
                    const double shrink = 0.85;
                    resize(photoImage, photoImage, Size(), shrink, shrink);
                    auto found = findChessboardCorners( photoImage, CB_INT_CNRS, imgCorners,
                                                        CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK
                                                        // |  cv::CALIB_CB_FILTER_QUADS  like forever!
                                                        //| cv::CALIB_CB_NORMALIZE_IMAGE   like forever!
                                                        //CALIB_CB_NORMALIZE_IMAGE   //CV_CALIB_CB_FAST_CHECK
                                               );
                    d = high_resolution_clock::now() - t1;
                    uint seconds = round(d.count() / 1000);
                    if (found)
                    {
                            cout << " found with " <<  seconds << "s\n";
                            TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001);

                            cornerSubPix(photoImage, imgCorners, Size(11, 11), Size(-1,-1), criteria);

                            bool found = false;
                            drawChessboardCorners(photoImage, CB_INT_CNRS, imgCorners, found);

                            if (aryPhyCorners.size() < photoNum) //if (whichCamera == nsCameras::Camera_Left_0 )
                                    aryPhyCorners.push_back(phyCorners);
                            aryImgCorners.push_back(imgCorners);

                            if (!unifiedImageSize.width)
                                    unifiedImageSize = photoImage.size();
                            else if (unifiedImageSize != photoImage.size())
                                    qDebug() << "DIFFERENT sizes!\n" ;
                    } else {
                            cout << " NOT found after " << seconds << "s, abort!\n";
                            return RC_FAIL_TO_FIND;
                    }
                } catch (const GenericException& e) {
                        cerr << "An exception 5 occurred." << endl << e.GetDescription() << endl;
                        return RC_FAIL_TO_FIND;
                }
        }

        //Mat intrinsic = Mat(3, 3, CV_32FC1);
        //Mat distCoeffs;
        vector<Mat> vcR, vcT;
        //intrnscParam.ptr<float>(0)[0] = 1,  intrnscParam.ptr<float>(1)[1] = 1;
        reprojError = calibrateCamera (aryPhyCorners, aryImgCorners, unifiedImageSize,
                                            intrnscParam, distortCoeffs, vcR, vcT,
                                            CALIB_FIX_PRINCIPAL_POINT
                                            | CALIB_RATIONAL_MODEL  //CALIB_USE_INTRINSIC_GUESS  //| CALIB_FIX_K4|cv::CALIB_FIX_K5
                                    );
                cout << "\n\ncalibrateCamera e=" << reprojError << endl;

                //cout << "cameraMatrix : " << intrnscParam << endl;
                //cout << "distCoeffs : " << distortCoeffs << endl;
                cout << "vcR size=" << vcR.size() << endl;
                cout << "vcR=" << vcR.at(0) << endl;
                cout << "vcT size=" << vcT.size() << endl;
                cout << "vcT=" << vcT.at(0) << endl;
                //cout << "Rotation vector : " << vcR << endl;
                //cout << "Translation vector : " << vcT << endl;
        intrnscParam = getOptimalNewCameraMatrix(
                                        intrnscParam, distortCoeffs, unifiedImageSize, 1, unifiedImageSize, &validPxlROI);
                //cout << "optimized cameraMatrix : " << intrnscParam << endl;
                //cout << "distCoeffs : " << distortCoeffs << endl;
                //cout << "validPxlROI=" << validPxlROI << endl;

        return RC_OK;
}
//presume single segment each row
static double locateWhiteMidOfARow(const Mat& img, const uint theRow, const Rect& roi)
{
if ((int) theRow < img.rows)
    for (int nthCol = 0; nthCol < roi.width; nthCol++)
            if (roi.x +  nthCol < img.cols)
                    if (img.at<uchar>(theRow, roi.x +  nthCol) == 255) //M# 221013 > 0)  S3DRP
                    {
//qDebug() << "S3DRP: "  << theRow << "," << roi.x +  nthCol << "=" << img.at<uchar>(theRow, roi.x +  nthCol);
                            uint startX =  roi.x +  nthCol;
                            for (++nthCol; nthCol <  roi.width; nthCol++)
                                    if (img.at<uchar>(theRow, roi.x +  nthCol) == 0)
                                            return (startX +  roi.x +  nthCol) / 2;
                    }
    return -1;
}

static double interpolate(const Mat& img, const uint nthRow, const Rect& roi, const Point2i& last,
                      Point2i& next)
{
    if ((int) nthRow <= last.y)
            return -1;

    if (next.y == last.y)
            return (next.x + last.x) / 2;

    if (next.y < last.y)
            //find next
            for (int nextRow = nthRow + 1; nextRow < roi.y + roi.height; nextRow++)
            {
                    double mid = locateWhiteMidOfARow(img,  nextRow,  roi);
                    if (mid > 0)
                    {
                          next = Point2i(mid, nextRow);
                          break;
                    }
            }

    if (next.y <= last.y)
            return -1;

    return (last.x * (nthRow - last.y) + next.x * (next.y - nthRow)) / (next.y - last.y);
}

//*
int nsImageProcess::GetStereo3DFromRectifiedPair (
    //EOperateMode opMode,
    const Mat &rectifiedLeftImage,   //full if OM_CALIBRATE_SQUARE_BAR; roi if OM_FETCH_RAIL_CROWN:
    Rect *pLeftROI,           //roi w/h if OM_CALIBRATE_SQUARE_BAR; full w/h if if OM_FETCH_RAIL_CROWN
    const Mat &rectifiedRightImage,   Rect *pRightROI,
    int pxlCameraOffset,  //Tx, pixels
    int pxlRectifiedFocus,  //Tx, pixels
    const double mmRectifiedFocus, //mm
    const Point2d principalPoint,
    vector<Point3d>& coords3D,  //in Camera's view
    Point &leftROIStart, Point &rightROIStart
    )
{
    //bool isROIImage = false;
    //Point ulcLeft = {0, 0}, ulcRight = {0, 0};
    Rect leftROI, rightROI;
    if (pLeftROI)
            leftROI= *pLeftROI;
    else
            leftROI= Rect(0, 0, rectifiedLeftImage.cols, rectifiedLeftImage.rows);
    if (pRightROI)
            rightROI= *pRightROI;
    else
            rightROI= Rect(0, 0, rectifiedRightImage.cols, rectifiedRightImage.rows);

    if (pxlCameraOffset < 0)  pxlCameraOffset = -pxlCameraOffset;
    if (pxlRectifiedFocus < 0)  pxlRectifiedFocus = -pxlRectifiedFocus;
    const double FO = mmRectifiedFocus * pxlCameraOffset;
    coords3D.clear();
    //align roi.y
    if (leftROI.y < rightROI.y)
    {
            leftROI.height -= rightROI.y - leftROI.y;
            leftROI.y = rightROI.y;
    } else if (leftROI.y > rightROI.y)
    {
            rightROI.height -= leftROI.y - rightROI.y;
            rightROI.y = leftROI.y;
    }
    //align height
    if (leftROI.height  < rightROI.height )
            rightROI.height  = leftROI.height;
    else if (rightROI.height  < leftROI.height )
            leftROI.height  = rightROI.height;

                //imwrite( nsCameras::DEBUG_PATH + "$S3DRP_rectifiedLeftImage.bmp",  rectifiedLeftImage );

    Point2i leftLast(0), leftNext(0), rightLast(0), rightNext(0);
    bool firstFound = false;
    for (int nthRow = leftROI.y; nthRow < leftROI.y + leftROI.height; nthRow++)
    {
            double dLeft = locateWhiteMidOfARow(rectifiedLeftImage,  nthRow,  leftROI);
            if (dLeft > 0)
                    leftLast = Point2i(dLeft, nthRow);
            else if (!firstFound )
                        continue;

            double dRight = locateWhiteMidOfARow(rectifiedRightImage,  nthRow,  rightROI);
            if (dRight > 0)
                    rightLast = Point2i(dRight, nthRow);
            else if (!firstFound )
                        continue;

            if ((dLeft > 0) && (dRight < 0)) {
                    dRight = interpolate(rectifiedRightImage,  nthRow,  rightROI, rightLast, rightNext);
                    if (dRight < 0)
                            break;  //no next
            } else if ((dLeft < 0) && (dRight > 0)) {
                    dLeft = interpolate(rectifiedLeftImage,  nthRow,  leftROI, leftLast, leftNext);
                    if (dLeft < 0)
                            break;  //no next
            }
            if ((dLeft < 0) || (dRight < 0))
                    continue;
            else if (!firstFound )
                        firstFound = true;

            dLeft += leftROIStart.x,  dRight += rightROIStart.x;  //ROI_IMAGE
            int Tx_d = (dLeft - principalPoint.x) - (dRight - principalPoint.x);

            double Zc = FO /*mmRectifiedFocus * pxlCameraOffset*/
                                    / Tx_d ;

            double Xc = Zc * (dLeft - principalPoint.x)  / pxlRectifiedFocus;
            double Yc = Zc * (nthRow - principalPoint.y) / pxlRectifiedFocus;

            coords3D.push_back(Point3d(Xc, Yc, Zc));
    }

    cout << "PC# = " << coords3D.size() << endl;
    //cout << coords3D << endl;
    return (coords3D.size() > 0) ?  RC_OK : RC_FAIL_TO_FIND;
}

/**/

int getAggregateCoord(
    const vector<Point3d>& fusedCoords3D,
    const uint start,
    const uint aggNum,
    Point3d &aggCrd)
{
    const uint coordSize = fusedCoords3D.size();
    Point3d sum(0, 0, 0);
    uint n;
    for (n = 0; n < aggNum; n++)
            if ((start + n)  < coordSize)
                sum += fusedCoords3D[start + n];  //presumed each fusedCoords3D[start + i].z <= lastZc
    if (!n)  return RC_FAIL_TO_FIND;

    aggCrd = Point3d(sum.x / n, sum.y / n, sum.z / n);
    return RC_OK;
}

//fusedCoords3D ordered, row-majered, top-to-down,
int nsImageProcess::DetermineXAxisOfCrossSquareBar(
    const vector<Point3d>& fusedCoords3D,
    Point3d& GaugeAxisX_0, Point3d& GaugeAxisX_1,
    Point3d& GaugeAxisY
    )
{
    constexpr uint AGGREGRATE_NUM = 5;
    const uint coordSize = fusedCoords3D.size();
    if (coordSize  < AGGREGRATE_NUM * 2)
            return RC_FAIL_TO_FIND;

    //should be descending
    uint i, n;
    int start = -1;
    double curZc, nextZc, lastZc =  fusedCoords3D[0].z;
    for (n = 0, i = 1; i < coordSize; lastZc =  nextZc, i++)
    {
            int d = (nextZc = fusedCoords3D[i].z) - lastZc;
            if ((d > -1) && (d <= 0))   //M#
            {
                    if (++n > 3)
                        break;
                    if (start < 0) start = i;
            }  else
                    start = -1;
    }
    /*
            if ((nextZc = fusedCoords3D[i].z) > lastZc)
                    lastZc =  nextZc, start = i;
            else if (nextZc < lastZc)
                    break;
     * Point3d sum(0, 0, 0);
    for (n = 0; n < AGGREGRATE_NUM; n++)
            if ((start + n)  < coordSize)
                sum += fusedCoords3D[start + n];  //presumed each fusedCoords3D[start + i].z <= lastZc
     GaugeAxisX_0 = Point3d(sum.x / n, sum.y / n, sum.z / n);
     */
    getAggregateCoord(fusedCoords3D, start, AGGREGRATE_NUM, GaugeAxisX_0);

    start += AGGREGRATE_NUM;
     lastZc =  fusedCoords3D[start++].z;
     //curZc =  fusedCoords3D[start++].z;  //presumed curZc  <= lastZc
     uint equalCnt = 0;
     for (n = start; n < coordSize; n++)
             if ((nextZc = fusedCoords3D[n].z) == lastZc)
                    equalCnt++;
            else  if (nextZc < lastZc) {
                     if ((lastZc - nextZc) > 3.0)  //noise before
                            getAggregateCoord(fusedCoords3D,
                                                start + n, AGGREGRATE_NUM, GaugeAxisX_0);
                     lastZc = nextZc, equalCnt = 0;
             } else if (nextZc > lastZc)
                    if ((n + AGGREGRATE_NUM * 4) < coordSize)  //2->4 220918  M#  why 4 230129
                            if ((lastZc < fusedCoords3D[n + AGGREGRATE_NUM * 4].z)) //noise
                                    break;    //D#
     if (n >= coordSize)
                return RC_FAIL_TO_FIND;
     uint aggregatedNum = max(equalCnt, AGGREGRATE_NUM);
     Point3d sum(0, 0, 0); //sum = Point3d(0, 0, 0);
     start = n - 1 + aggregatedNum / 2;
     //start = n - aggregatedNum / 2;  //230129
     for (uint i = 0; i < aggregatedNum; i++)
             sum += fusedCoords3D[start - i];  //presumed each fusedCoords3D[start + i].z <= lastZc
     sum += fusedCoords3D[n - 1], aggregatedNum++;
     GaugeAxisX_1 = Point3d(sum.x / aggregatedNum,
                                sum.y / aggregatedNum, sum.z / aggregatedNum);

     //double axisXLen = norm(GaugeAxisX_1 - GaugeAxisX_0);
     //cout << "AxisX Len=" << axisXLen << endl;  //#DEBUGDEBUG

     start += aggregatedNum;
     lastZc =  fusedCoords3D[start++].z;
     for (n = start; n < coordSize; n++)
             if (((nextZc = fusedCoords3D[n].z) - lastZc) > 2.0)  //M#  4->2 for Box4 SE
                    break;
            else
                    lastZc =  nextZc;

     GaugeAxisY = fusedCoords3D[n - 1]; //maybe coordSize-1
     //cout << "AxisY Len=" << norm(GaugeAxisX_1 - GaugeAxisY) << endl << endl;  //#DEBUGDEBUG

     return RC_OK;
}

int nsImageProcess::MapRail3DCoordsTo2DCS(
        vector<Point3d>& fusedCoords3D,
        const Point3d& GaugeAxisX_0, const Point3d& GaugeAxisX_1, const Point3d& GaugeAxisY,
        vector<Point2d>& mapped2CS_XY2D //with its X poining towards the camera, Y pointing to ceiling
    )
{
#ifdef _DEBUG_SE
    qDebug() << "FC3D size:" << fusedCoords3D.size();
#endif
    Point2d xy;
    mapped2CS_XY2D.clear();
    for (auto p3d : fusedCoords3D) //i = 1; i < fusedCoords3D.size(); i++)
            if (nsUtility::ProjectPointToAxisX(p3d, GaugeAxisX_0, GaugeAxisX_1, GaugeAxisY, xy))
                    mapped2CS_XY2D.push_back(xy);

    return (mapped2CS_XY2D.size() > 0) ? RC_OK : RC_FAIL_TO_FIND;
}

int nsImageProcess::GetRailHeadVH(
    const uint nthBox,
     const EOperateMode opMode,
    const vector<Point2d>& contour,
    //const uint RESOLUTION_RATIO = 10,  //1 for 1mm per x-uint, 10 for 0.1mm
    const double VHVerticalOffset,
    Point2d &V, Point2d &H
    )
{
    constexpr int SMOOTH_WND = 5;
    constexpr int SPAN_WND = SMOOTH_WND * 2;
    const int N = contour.size();
    double maxYVal = -DBL_MAX; //DBL_MIN;
    int maxYIdx = -1;
    int nthIdx = -1;
    switch(opMode)
    {
    case OM_CALIBRATE_SQUARE_BAR:
                                    //vector<double> offsetY(contour.size());
                                    for (nthIdx = SPAN_WND; nthIdx < N - SPAN_WND; nthIdx++)
                                            if (contour[nthIdx].x >= 0)
                                                    break;
                                    for (nthIdx += SPAN_WND ; nthIdx < N - SPAN_WND; nthIdx++)
                                    {
                                            double offsetY = (   contour[nthIdx + SPAN_WND].y
                                                                                + contour[nthIdx - SPAN_WND].y )
                                                                                - 2 * contour[nthIdx].y;
                                            offsetY = abs(offsetY);
                                            //if (!nthBox)
                                                    offsetY += contour[nthIdx + SPAN_WND].x - contour[nthIdx - SPAN_WND].x;  //230129  BOX-     1
                                            if (offsetY > maxYVal)
                                                maxYVal = offsetY, maxYIdx = nthIdx;
                                    }
                                    break;
    case OM_FETCH_RAIL_CROWN:
                                    for (int i = 5; i < N; i++)  //M# 230317 skip noise Box4
                                            if (contour[i].y > maxYVal)
                                                    maxYVal = contour[i].y, maxYIdx = i;
                                    break;
    }
    double sumx = 0;
    double sumy = 0;
    for (int i = max(0, maxYIdx - SMOOTH_WND); i <= min(N-1, maxYIdx + SMOOTH_WND); i++)
            sumx += contour[i].x, sumy += contour[i].y;

    V.x = sumx /  (SMOOTH_WND + SMOOTH_WND + 1);
    V.y = sumy /  (SMOOTH_WND + SMOOTH_WND + 1);

    const double predictedHy = V.y - VHVerticalOffset;
    double d, closestVal = DBL_MAX;
    int closestIdx = -1;
    for (int i = maxYIdx; i < contour.size(); i++)
            if ( (d = abs(contour[i].y - predictedHy)) < closestVal )
                    closestVal  = d, closestIdx = i;

    sumx = sumy = 0;
    for (int i = max(0, closestIdx - SMOOTH_WND); i <= min(N-1, closestIdx + SMOOTH_WND); i++)
            sumx += contour[i].x, sumy += contour[i].y;

    H.x = sumx /  (SMOOTH_WND + SMOOTH_WND + 1);
    H.y = sumy /  (SMOOTH_WND + SMOOTH_WND + 1);

    return RC_OK;
}


double nsUtility::AngleBtnVectors(const Vec3d& v1, const Vec3d& v2)
{
    return acos( v1.dot(v2) / (norm(v1) * norm(v2)) );
}

bool nsUtility::ProjectPointToAxisX(const Point3d& xyz, const Point3d& AxisX_0, const Point3d& AxisX_1, const Point3d& AxisY_1, Point2d& xy)
{
        Vec3d AB = AxisX_1 - AxisX_0;
        auto AB_2 = AB.dot(AB);
        if (AB_2 == 0)  return false;
        //Vec3d uAB = AB / AB.dot(AB);  //norm(AB); //
        Vec3d AP = xyz - AxisX_0;
        auto dpb = AP.dot(AB) / AB_2;
        Point3d C = Vec3d(AxisX_0) +  dpb * AB;
        xy.x = norm(C - AxisX_0);          if (dpb < 0)  xy.x = -xy.x;

        Vec3d DB = AxisX_1 - AxisY_1;
        auto pddb = AP.dot(DB);// / DB.dot(DB);
        xy.y = norm(xyz - C);
        //double dbp = AngleBtnVectors(AP, AB);
        if (pddb < 0) xy.y = -xy.y;
        return true;
}

int nsUtility::GetCoordinatesRange(const vector<Point2d>& coords, Range& xr, Range& yr, const uint RESOLUTION_RATIO)
{
    double minX = DBL_MAX;
    double maxX = -DBL_MAX; //DBL_MIN;
    double minY = DBL_MAX;
    double maxY = -DBL_MAX; //DBL_MIN;
    for (auto p : coords)
    {
            double x = p.x * RESOLUTION_RATIO;
            double y = p.y * RESOLUTION_RATIO;
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
    }
    xr.start = round(minX),  xr.end = round(maxX);
    yr.start = round(minY),  yr.end = round(maxY);
    return RC_OK;
}

int nsUtility::SaveMatBinary(Mat& mat, String filename)
{
    ofstream OF( filename, ios::out | ios::binary);
    //cout << "OF:" << mat.rows << ", " << mat.cols << ", " << mat.type() << endl;
    int n = mat.rows;        OF.write((char *) &n, sizeof(int));
    n = mat.cols;               OF.write((char *) &n, sizeof(int));
    n = mat.type();           OF.write((char *) &n, sizeof(int));
    if (!OF.write((char *) mat.data, mat.elemSize()* mat.total()))
            cout << "OF:" << OF.good() << endl;
    OF.close();
    return RC_OK;
}

int nsUtility::LoadMatBinary(String filename, Mat& mat)
{
    ifstream IF( filename, ios::in | ios::binary );
    int r, c, t;
    IF.read((char *) &r, sizeof(int));
    IF.read((char *) &c, sizeof(int));
    IF.read((char *) &t, sizeof(int));
    if ((r <= 0) || (c <= 0) || (t < 0))
        return RC_FAIL_TO_LOAD;
    //if (mat.empty())
            mat.create(r, c, t);
            IF.read((char *) mat.data, mat.elemSize() * mat.total());
    //Mat m(r, c, t);
    //IF.read((char *) m.data, m.elemSize()* m.total());
    IF.close();
    //m.copyTo(mat);
    return RC_OK; //mat;
}

String nsUtility::GetFileStamp(String path, uint count, uint nthCamera)
{
    std::ostringstream oss("");
    try {
            auto t = time(nullptr);
            auto tm = *localtime(&t);
            //oss << "./data/";
            //oss << "./" << subdir << "/";
            oss << path;  //tailed by  /
            oss << put_time(&tm, "%Y_%m_%d_%H_%M_%S");
            oss << "_" << count;
            oss << "_" << nsCameras::GetCameraChar(nthCamera);
            /*if (cameraID == 0)
                oss << "_LEFT";
            else
                oss << "_RIGHT";*/
            oss << ".bmp";
    } catch (int thrown) {
            //execCode = thrown;
    }
    return oss.str();
}

String nsUtility::GetDirStamp(String path)
{
    std::ostringstream oss("");
    try {
            auto t = time(nullptr);
            auto tm = *localtime(&t);
            //oss << "./data/";
            //oss << "./" << subdir << "/";
            oss << path;  //tailed by  /
            oss << put_time(&tm, "%Y_%m_%d_%H_%M");
    } catch (int thrown) {
            //execCode = thrown;
    }
    return oss.str();
}

void nsUtility::DumpHexBytes(const char *data, const int size)
{
    //cout << size << " bytes: \n";
    for (int i = 0; i < size; i++)
            cout << hex << setfill('0') << setw(2)
                         << "0x" << (int) data[i] << " ";
    cout << endl << endl;
}

using namespace nsImageProcess;
int nsUtility::TrimStereoMap(  Mat &reverseMapTableX, Mat &reverseMapTableY, const  Rect &targetROI,
                                            Rect &ReverseMapTOI, Mat &ReverseMapMask)
{
    if (reverseMapTableX.empty() || reverseMapTableY.empty())
            return RC_INVALID_DATA;
    if ((reverseMapTableX.size() != reverseMapTableY.size()) || (reverseMapTableX.elemSize() != reverseMapTableY.elemSize()))
            return RC_INVALID_DATA;
    if ( !reverseMapTableX.isContinuous() )  reverseMapTableX = reverseMapTableX.clone();
    if ( !reverseMapTableY.isContinuous() )  reverseMapTableY = reverseMapTableY.clone();
    int rc = RC_FAIL_TO_MAP;

    ReverseMapMask = Mat::zeros(reverseMapTableX.size(), CV_8UC1);

    int left, top, right, bottom;
    left = top = numeric_limits<int>::max();
    right = bottom = numeric_limits<int>::min();

    //= Rect(CUR_LASER_ROI[nthBox][Camera_Left_0] );
    RECTIFY_MAP_TYPE *pReverseMapTableX = (RECTIFY_MAP_TYPE *) reverseMapTableX.data;
    RECTIFY_MAP_TYPE *pReverseMapTableY = (RECTIFY_MAP_TYPE *) reverseMapTableY.data;
    BYTE *pReverseMapMask = (BYTE *) ReverseMapMask.data;
    for(int y = 0; y < reverseMapTableX.rows; y++)
             for(int x = 0; x < reverseMapTableX.cols; x++)
             {
                        int idx = y * reverseMapTableX.cols + x;
                        RECTIFY_MAP_TYPE coordXd = pReverseMapTableX[idx];
                        int coordXi = round(coordXd);
                        if ((coordXi < targetROI.x) || (coordXi >= targetROI.x + targetROI.width))
                                continue;

                        RECTIFY_MAP_TYPE coordYd = pReverseMapTableY[idx];
                        int coordYi = round(coordYd);
                        if ((coordYi < targetROI.y) || (coordYi >= targetROI.y + targetROI.height))  //good
                                continue;

                        pReverseMapMask[idx] = 255;
                        left = min(left, x), top = min(top, y);
                        right = max(right, x), bottom = max(bottom, y);
              }
    int width = right - left + 1;
    int height = bottom - top + 1;
    if ((width <= 0) || (height <= 0))
            return RC_FAIL_TO_FIND;

    ReverseMapTOI.x = left, ReverseMapTOI.y = top;
    ReverseMapTOI.width = width, ReverseMapTOI.height = height;

    return RC_OK;
}

int nsUtility::ReverseRoundMap(   Mat &orgImage,  //const Size mappedImageSize,
                                    Mat &reverseMapTableX, Mat &reverseMapTableY,
                                    Rect *pReverseMapTOI, Mat *pReverseMapMask,
                                            Mat &mappedImage )
{
    if (orgImage.empty() || reverseMapTableX.empty() || reverseMapTableY.empty())
            return RC_INVALID_DATA;
    if ((reverseMapTableX.size() != reverseMapTableY.size()) || (reverseMapTableX.elemSize() != reverseMapTableY.elemSize()))
            return RC_INVALID_DATA;
    if ( !orgImage.isContinuous() )  orgImage = orgImage.clone();
    if ( !reverseMapTableX.isContinuous() )  reverseMapTableX = reverseMapTableX.clone();
    if ( !reverseMapTableY.isContinuous() )  reverseMapTableY = reverseMapTableY.clone();
    int rc = RC_FAIL_TO_MAP;
    Size mappedImageSize = reverseMapTableX.size();
    if (!mappedImage.empty())
            if ((mappedImage.size() != mappedImageSize) || (mappedImage.type() != orgImage.type()))
                        mappedImage.release();
    if (mappedImage.empty())
            mappedImage = Mat::zeros(mappedImageSize.height, mappedImageSize.width, orgImage.type());
    else
            mappedImage.setTo(0);

    //reverseMapTableX.elemSize()
    Rect ReverseMapTOI;
    if (pReverseMapTOI)
            ReverseMapTOI = *pReverseMapTOI;
    else
            pReverseMapMask = nullptr, ReverseMapTOI = Rect(Point(0, 0), mappedImageSize); //reverseMapTableX.cols, reverseMapTableX.rows);

    RECTIFY_MAP_TYPE *pReverseMapTableX = (RECTIFY_MAP_TYPE *) reverseMapTableX.data;
    RECTIFY_MAP_TYPE *pReverseMapTableY = (RECTIFY_MAP_TYPE *) reverseMapTableY.data;
    BYTE *pOrgImage = orgImage.data;
    BYTE *pRMapMask = (pReverseMapMask) ? pReverseMapMask->data : nullptr;
    BYTE *pMappedImage = mappedImage.data;
    for(int y = ReverseMapTOI.y; y < ReverseMapTOI.y + ReverseMapTOI.height /*reverseMapTableX.rows*/; y++)
             for(int x = ReverseMapTOI.x; x < ReverseMapTOI.x + ReverseMapTOI.width /*reverseMapTableX.cols*/; x++)
             {
                        int idx = y * reverseMapTableX.cols + x;
                        if (!pReverseMapMask || (pRMapMask[idx]))
                        {
                                RECTIFY_MAP_TYPE coordXd = pReverseMapTableX[idx];
                                int coordXi = round(coordXd);
                                if ((coordXi < 0) || (coordXi >= orgImage.cols))
                                        continue;
                                RECTIFY_MAP_TYPE coordYd = pReverseMapTableY[idx];
                                int coordYi = round(coordYd);
                                if ((coordYi < 0) || (coordYi >= orgImage.rows))
                                        continue;

                                BYTE val = pOrgImage[coordYi * orgImage.cols + coordXi];
                                if (val > 0)
                                        pMappedImage[y * mappedImage.cols + x] = val;
                        }
              }
    return RC_OK;
}

bool nsUtility::RotateBaroundA(cv::Point3d &A, cv::Point3d &B,  double angleDegree)
{
    double a = angleDegree * M_PI / 180;  //-5 +5  -0.5 -0.2(1, 52)  -0.1 (44, -20)  -0.3(15, 54)  -0.25
    double dx = B.x - A.x;
    double dy = B.y - A.y;
    B.x = dx * cos(a) - dy * sin(a) + A.x;
    B.y = dx * sin(a) + dy * cos(a) + A.y;
    return true;
}


int nsGalvanometer::SendAnglesToGalvanometer()
{


}



int CaptureWith1stCamera()
{
    try
    {
        // Create an instant camera object with the camera device found first.
        CInstantCamera camera( CTlFactory::GetInstance().CreateFirstDevice() );

        // Print the model name of the camera.
        cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;

        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        camera.MaxNumBuffer = 5;

        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        static const uint32_t c_countOfImagesToGrab = 100;
        camera.StartGrabbing( c_countOfImagesToGrab );

        // This smart pointer will receive the grab result data.
        CGrabResultPtr ptrGrabResult;

        // Camera.StopGrabbing() is called automatically by the RetrieveResult() method
        // when c_countOfImagesToGrab images have been retrieved.
        while (camera.IsGrabbing())
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException );

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                // Access the image data.
                cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
                cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
                const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
                cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << endl << endl;

    #ifdef PYLON_WIN_BUILD
                // Display the grabbed image.
                Pylon::DisplayImage( 1, ptrGrabResult );
    #endif
            }
            else
            {
                cout << "Error: " << std::hex << ptrGrabResult->GetErrorCode() << std::dec << " " << ptrGrabResult->GetErrorDescription() << endl;
            }
        }
    }
    catch (const GenericException& e)
    {
        // Error handling.
        cerr << "An exception 6 occurred." << endl
                << e.GetDescription() << endl;
    }
    return 0;
}

//=========================================================

/*
QDir.exists
bool nsUtility::DoesDirExist(String dir)
{
    struct stat info;
    if (!stat(dir.c_str(), &info))
            if (info.st_mode & S_IFDIR)
                return true;
    return false;
}
*/

#ifdef REF
void aa{
{
const int INTER_REMAP_COEF_BITS=15;
const int INTER_REMAP_COEF_SCALE=1 << INTER_REMAP_COEF_BITS;

static uchar NNDeltaTab_i[INTER_TAB_SIZE2][2];

static float BilinearTab_f[INTER_TAB_SIZE2][2][2];
static short BilinearTab_i[INTER_TAB_SIZE2][2][2];

#if CV_SIMD128
static short BilinearTab_iC4_buf[INTER_TAB_SIZE2+2][2][8];
static short (*BilinearTab_iC4)[2][8] = (short (*)[2][8])alignPtr(BilinearTab_iC4_buf, 16);
#endif

static float BicubicTab_f[INTER_TAB_SIZE2][4][4];
static short BicubicTab_i[INTER_TAB_SIZE2][4][4];

static float Lanczos4Tab_f[INTER_TAB_SIZE2][8][8];
static short Lanczos4Tab_i[INTER_TAB_SIZE2][8][8];

static inline void interpolateLinear( float x, float* coeffs ) {   coeffs[0] = 1.f - x,  coeffs[1] = x; }

static inline void interpolateCubic( float x, float* coeffs )
{
const float A = -0.75f;
coeffs[0] = ((A*(x + 1) - 5*A) * (x + 1) + 8*A) * (x + 1) - 4*A;
coeffs[1] = ((A + 2 ) * x - (A + 3 )) * x * x + 1;
coeffs[2] = ((A + 2) * (1 - x) - (A + 3)) * (1 - x) * (1 - x) + 1;
coeffs[3] = 1.f - coeffs[0] - coeffs[1] - coeffs[2];
}

static inline void interpolateLanczos4( float x, float* coeffs )
{
static const double s45 = 0.70710678118654752440084436210485;
static const double cs[][2]=
{{1, 0}, {-s45, -s45}, {0, 1}, {s45, -s45}, {-1, 0}, {s45, s45}, {0, -1}, {-s45, s45}};

if( x < FLT_EPSILON )
{
        for( int i = 0; i < 8; i++ )
            coeffs[i] = 0;
        coeffs[3] = 1;
        return;
}

float sum = 0;
double y0=-(x+3)*CV_PI*0.25, s0 = std::sin(y0), c0= std::cos(y0);
for(int i = 0; i < 8; i++ )
{
        double y = -(x+3-i)*CV_PI*0.25;
        coeffs[i] = (float)((cs[i][0]*s0 + cs[i][1]*c0)/(y*y));
        sum += coeffs[i];
}

sum = 1.f/sum;
for(int i = 0; i < 8; i++ )
        coeffs[i] *= sum;
}

static void initInterTab1D(int method, float* tab, int tabsz)
{
float scale = 1.f/tabsz;
if( method == INTER_LINEAR )
{
    for( int i = 0; i < tabsz; i++, tab += 2 )
        interpolateLinear( i*scale, tab );
}
else if( method == INTER_CUBIC )
{
    for( int i = 0; i < tabsz; i++, tab += 4 )
        interpolateCubic( i*scale, tab );
}
else if( method == INTER_LANCZOS4 )
{
    for( int i = 0; i < tabsz; i++, tab += 8 )
        interpolateLanczos4( i*scale, tab );
}
else
    CV_Error( CV_StsBadArg, "Unknown interpolation method" );
}


static const void* initInterTab2D( int method, bool fixpt )
{
static bool inittab[INTER_MAX+1] = {false};
float* tab = 0;
short* itab = 0;
int ksize = 0;
if( method == INTER_LINEAR )
    tab = BilinearTab_f[0][0], itab = BilinearTab_i[0][0], ksize=2;
else if( method == INTER_CUBIC )
    tab = BicubicTab_f[0][0], itab = BicubicTab_i[0][0], ksize=4;
else if( method == INTER_LANCZOS4 )
    tab = Lanczos4Tab_f[0][0], itab = Lanczos4Tab_i[0][0], ksize=8;
else
    CV_Error( CV_StsBadArg, "Unknown/unsupported interpolation type" );

if( !inittab[method] )
{
    AutoBuffer<float> _tab(8*INTER_TAB_SIZE);
    int i, j, k1, k2;
    initInterTab1D(method, _tab.data(), INTER_TAB_SIZE);
    for( i = 0; i < INTER_TAB_SIZE; i++ )
        for( j = 0; j < INTER_TAB_SIZE; j++, tab += ksize*ksize, itab += ksize*ksize )
        {
            int isum = 0;
            NNDeltaTab_i[i*INTER_TAB_SIZE+j][0] = j < INTER_TAB_SIZE/2;
            NNDeltaTab_i[i*INTER_TAB_SIZE+j][1] = i < INTER_TAB_SIZE/2;

            for( k1 = 0; k1 < ksize; k1++ )
            {
                float vy = _tab[i*ksize + k1];
                for( k2 = 0; k2 < ksize; k2++ )
                {
                    float v = vy*_tab[j*ksize + k2];
                    tab[k1*ksize + k2] = v;
                    isum += itab[k1*ksize + k2] = saturate_cast<short>(v*INTER_REMAP_COEF_SCALE);
                }
            }

            if( isum != INTER_REMAP_COEF_SCALE )
            {
                int diff = isum - INTER_REMAP_COEF_SCALE;
                int ksize2 = ksize/2, Mk1=ksize2, Mk2=ksize2, mk1=ksize2, mk2=ksize2;
                for( k1 = ksize2; k1 < ksize2+2; k1++ )
                    for( k2 = ksize2; k2 < ksize2+2; k2++ )
                    {
                        if( itab[k1*ksize+k2] < itab[mk1*ksize+mk2] )
                            mk1 = k1, mk2 = k2;
                        else if( itab[k1*ksize+k2] > itab[Mk1*ksize+Mk2] )
                            Mk1 = k1, Mk2 = k2;
                    }
                if( diff < 0 )
                    itab[Mk1*ksize + Mk2] = (short)(itab[Mk1*ksize + Mk2] - diff);
                else
                    itab[mk1*ksize + mk2] = (short)(itab[mk1*ksize + mk2] - diff);
            }
        }
    tab -= INTER_TAB_SIZE2*ksize*ksize;
    itab -= INTER_TAB_SIZE2*ksize*ksize;
#if CV_SIMD128
    if( method == INTER_LINEAR )
    {
        for( i = 0; i < INTER_TAB_SIZE2; i++ )
            for( j = 0; j < 4; j++ )
            {
                BilinearTab_iC4[i][0][j*2] = BilinearTab_i[i][0][0];
                BilinearTab_iC4[i][0][j*2+1] = BilinearTab_i[i][0][1];
                BilinearTab_iC4[i][1][j*2] = BilinearTab_i[i][1][0];
                BilinearTab_iC4[i][1][j*2+1] = BilinearTab_i[i][1][1];
            }
    }
#endif
    inittab[method] = true;
}
return fixpt ? (const void*)itab : (const void*)tab;
}

#ifndef __MINGW32__
static bool initAllInterTab2D()
{
return  initInterTab2D( INTER_LINEAR, false ) &&
        initInterTab2D( INTER_LINEAR, true ) &&
        initInterTab2D( INTER_CUBIC, false ) &&
        initInterTab2D( INTER_CUBIC, true ) &&
        initInterTab2D( INTER_LANCZOS4, false ) &&
        initInterTab2D( INTER_LANCZOS4, true );
}

static volatile bool doInitAllInterTab2D = initAllInterTab2D();
#endif

template<typename ST, typename DT> struct Cast
{
typedef ST type1;
typedef DT rtype;

DT operator()(ST val) const { return saturate_cast<DT>(val); }
};

template<typename ST, typename DT, int bits> struct FixedPtCast
{
typedef ST type1;
typedef DT rtype;
enum { SHIFT = bits, DELTA = 1 << (bits-1) };

DT operator()(ST val) const { return saturate_cast<DT>((val + DELTA)>>SHIFT); }
};

static inline int clip(int x, int a, int b) {  return x >= a ? (x < b ? x : b-1) : a;  }


template<typename T>
static void remapNearest( const Mat& _src, Mat& _dst, const Mat& _xy,
                      int borderType, const Scalar& _borderValue )
{
Size ssize = _src.size(), dsize = _dst.size();
const int channelNum = _src.channels();
const T* S0 = _src.ptr<T>();
size_t sstep = _src.step / sizeof(S0[0]);
T cval[CV_CN_MAX];

for(int k = 0; k < channelNum; k++ )
    cval[k] = saturate_cast<T>(_borderValue[k & 3]);

unsigned width1 = ssize.width, height1 = ssize.height;

if( _dst.isContinuous() && _xy.isContinuous() )
{
    dsize.width *= dsize.height;
    dsize.height = 1;
}

for(int dy = 0; dy < dsize.height; dy++ )
{
    T* D = _dst.ptr<T>(dy);
    const short* XY = _xy.ptr<short>(dy);

    if( channelNum == 1 )
            for(int dx = 0; dx < dsize.width; dx++ )
            {
                    int sx = XY[dx * 2], sy = XY[dx * 2 + 1];
                    if( (unsigned) sx < width1 && (unsigned) sy < height1 )
                            D[dx] = S0[sy * sstep + sx];
                    else
                            if( borderType == BORDER_REPLICATE )
                            {
                                sx = clip(sx, 0, ssize.width);
                                sy = clip(sy, 0, ssize.height);
                                D[dx] = S0[sy*sstep + sx];
                            }
                            else if( borderType == BORDER_CONSTANT )
                                D[dx] = cval[0];
                            else if( borderType != BORDER_TRANSPARENT )
                            {
                                sx = borderInterpolate(sx, ssize.width, borderType);
                                sy = borderInterpolate(sy, ssize.height, borderType);
                                D[dx] = S0[sy*sstep + sx];
                            }
            }
    else
        for(int dx = 0; dx < dsize.width; dx++, D += channelNum )
        {
                int sx = XY[dx*2], sy = XY[dx*2+1];
                const T *S;
                if( (unsigned)sx < width1 && (unsigned)sy < height1 )
                {
                    if( channelNum == 3 )
                    {
                            S = S0 + sy*sstep + sx*3;
                            D[0] = S[0], D[1] = S[1], D[2] = S[2];
                    }
                    else if( channelNum == 4 )
                    {
                            S = S0 + sy*sstep + sx*4;
                            D[0] = S[0], D[1] = S[1], D[2] = S[2], D[3] = S[3];
                    }
                    else
                    {
                            S = S0 + sy*sstep + sx * channelNum;
                            for (int k = 0; k < channelNum; k++ )
                                    D[k] = S[k];
                    }
                }
                else if( borderType != BORDER_TRANSPARENT )
                {
                    if( borderType == BORDER_REPLICATE )
                    {
                        sx = clip(sx, 0, ssize.width);
                        sy = clip(sy, 0, ssize.height);
                        S = S0 + sy*sstep + sx*channelNum;
                    }
                    else if( borderType == BORDER_CONSTANT )
                        S = &cval[0];
                    else
                    {
                        sx = borderInterpolate(sx, ssize.width, borderType);
                        sy = borderInterpolate(sy, ssize.height, borderType);
                        S = S0 + sy*sstep + sx*channelNum;
                    }
                    for(int k = 0; k < channelNum; k++ )
                        D[k] = S[k];
                }
        }
}
}

template<class CastOp, class VecOp, typename AT>
static void remapBilinear( const Mat& _src, Mat& _dst, const Mat& _xy,
                       const Mat& _fxy, const void* _wtab,
                       int borderType, const Scalar& _borderValue )
{
typedef typename CastOp::rtype T;
typedef typename CastOp::type1 WT;
Size ssize = _src.size(), dsize = _dst.size();
const int channelNum = _src.channels();
const AT* wtab = (const AT*)_wtab;
const T* S0 = _src.ptr<T>();
size_t sstep = _src.step/sizeof(S0[0]);
T cval[CV_CN_MAX];
CastOp castOp;
VecOp vecOp;

for(int k = 0; k < channelNum; k++ )
    cval[k] = saturate_cast<T>(_borderValue[k & 3]);

unsigned width1 = std::max(ssize.width-1, 0), height1 = std::max(ssize.height-1, 0);
CV_Assert( !ssize.empty() );
#if CV_SIMD128
if( _src.type() == CV_8UC3 )
    width1 = std::max(ssize.width-2, 0);
#endif

for(int dy = 0; dy < dsize.height; dy++ )
{
    T* D = _dst.ptr<T>(dy);
    const short* XY = _xy.ptr<short>(dy);
    const ushort* FXY = _fxy.ptr<ushort>(dy);
    int X0 = 0;
    bool prevInlier = false;

    for(int dx = 0; dx <= dsize.width; dx++ )
    {
        bool curInlier = dx < dsize.width ?
            (unsigned)XY[dx*2] < width1 &&
            (unsigned)XY[dx*2+1] < height1 : !prevInlier;
        if( curInlier == prevInlier )
            continue;

        int X1 = dx;
        dx = X0;
        X0 = X1;
        prevInlier = curInlier;

        if( !curInlier )
        {
            int len = vecOp( _src, D, XY + dx*2, FXY + dx, wtab, X1 - dx );
            D += len*channelNum;
            dx += len;

            if( channelNum == 1 )
            {
                for( ; dx < X1; dx++, D++ )
                {
                    int sx = XY[dx*2], sy = XY[dx*2+1];
                    const AT* w = wtab + FXY[dx]*4;
                    const T* S = S0 + sy*sstep + sx;
                    *D = castOp(WT(S[0]*w[0] + S[1]*w[1] + S[sstep]*w[2] + S[sstep+1]*w[3]));
                }
            }
            else if( channelNum == 2 )
                for( ; dx < X1; dx++, D += 2 )
                {
                    int sx = XY[dx*2], sy = XY[dx*2+1];
                    const AT* w = wtab + FXY[dx]*4;
                    const T* S = S0 + sy*sstep + sx*2;
                    WT t0 = S[0]*w[0] + S[2]*w[1] + S[sstep]*w[2] + S[sstep+2]*w[3];
                    WT t1 = S[1]*w[0] + S[3]*w[1] + S[sstep+1]*w[2] + S[sstep+3]*w[3];
                    D[0] = castOp(t0); D[1] = castOp(t1);
                }
            else if( channelNum == 3 )
                for( ; dx < X1; dx++, D += 3 )
                {
                    int sx = XY[dx*2], sy = XY[dx*2+1];
                    const AT* w = wtab + FXY[dx]*4;
                    const T* S = S0 + sy*sstep + sx*3;
                    WT t0 = S[0]*w[0] + S[3]*w[1] + S[sstep]*w[2] + S[sstep+3]*w[3];
                    WT t1 = S[1]*w[0] + S[4]*w[1] + S[sstep+1]*w[2] + S[sstep+4]*w[3];
                    WT t2 = S[2]*w[0] + S[5]*w[1] + S[sstep+2]*w[2] + S[sstep+5]*w[3];
                    D[0] = castOp(t0); D[1] = castOp(t1); D[2] = castOp(t2);
                }
            else if( channelNum == 4 )
                for( ; dx < X1; dx++, D += 4 )
                {
                    int sx = XY[dx*2], sy = XY[dx*2+1];
                    const AT* w = wtab + FXY[dx]*4;
                    const T* S = S0 + sy*sstep + sx*4;
                    WT t0 = S[0]*w[0] + S[4]*w[1] + S[sstep]*w[2] + S[sstep+4]*w[3];
                    WT t1 = S[1]*w[0] + S[5]*w[1] + S[sstep+1]*w[2] + S[sstep+5]*w[3];
                    D[0] = castOp(t0); D[1] = castOp(t1);
                    t0 = S[2]*w[0] + S[6]*w[1] + S[sstep+2]*w[2] + S[sstep+6]*w[3];
                    t1 = S[3]*w[0] + S[7]*w[1] + S[sstep+3]*w[2] + S[sstep+7]*w[3];
                    D[2] = castOp(t0); D[3] = castOp(t1);
                }
            else
                for( ; dx < X1; dx++, D += channelNum )
                {
                    int sx = XY[dx*2], sy = XY[dx*2+1];
                    const AT* w = wtab + FXY[dx]*4;
                    const T* S = S0 + sy*sstep + sx*channelNum;
                    for(int k = 0; k < channelNum; k++ )
                    {
                        WT t0 = S[k]*w[0] + S[k+channelNum]*w[1] + S[sstep+k]*w[2] + S[sstep+k+channelNum]*w[3];
                        D[k] = castOp(t0);
                    }
                }
        }
        else
        {
            if( borderType == BORDER_TRANSPARENT && channelNum != 3 )
            {
                D += (X1 - dx)*channelNum;
                dx = X1;
                continue;
            }

            if( channelNum == 1 )
                for( ; dx < X1; dx++, D++ )
                {
                    int sx = XY[dx*2], sy = XY[dx*2+1];
                    if( borderType == BORDER_CONSTANT &&
                        (sx >= ssize.width || sx+1 < 0 ||
                         sy >= ssize.height || sy+1 < 0) )
                    {
                        D[0] = cval[0];
                    }
                    else
                    {
                        int sx0, sx1, sy0, sy1;
                        T v0, v1, v2, v3;
                        const AT* w = wtab + FXY[dx]*4;
                        if( borderType == BORDER_REPLICATE )
                        {
                            sx0 = clip(sx, 0, ssize.width);
                            sx1 = clip(sx+1, 0, ssize.width);
                            sy0 = clip(sy, 0, ssize.height);
                            sy1 = clip(sy+1, 0, ssize.height);
                            v0 = S0[sy0*sstep + sx0];
                            v1 = S0[sy0*sstep + sx1];
                            v2 = S0[sy1*sstep + sx0];
                            v3 = S0[sy1*sstep + sx1];
                        }
                        else
                        {
                            sx0 = borderInterpolate(sx, ssize.width, borderType);
                            sx1 = borderInterpolate(sx+1, ssize.width, borderType);
                            sy0 = borderInterpolate(sy, ssize.height, borderType);
                            sy1 = borderInterpolate(sy+1, ssize.height, borderType);
                            v0 = sx0 >= 0 && sy0 >= 0 ? S0[sy0*sstep + sx0] : cval[0];
                            v1 = sx1 >= 0 && sy0 >= 0 ? S0[sy0*sstep + sx1] : cval[0];
                            v2 = sx0 >= 0 && sy1 >= 0 ? S0[sy1*sstep + sx0] : cval[0];
                            v3 = sx1 >= 0 && sy1 >= 0 ? S0[sy1*sstep + sx1] : cval[0];
                        }
                        D[0] = castOp(WT(v0*w[0] + v1*w[1] + v2*w[2] + v3*w[3]));
                    }
                }
            else
                for( ; dx < X1; dx++, D += channelNum )
                {
                    int sx = XY[dx*2], sy = XY[dx*2+1];
                    if( borderType == BORDER_CONSTANT &&
                        (sx >= ssize.width || sx+1 < 0 ||
                         sy >= ssize.height || sy+1 < 0) )
                    {
                        for(int k = 0; k < channelNum; k++ )
                            D[k] = cval[k];
                    }
                    else
                    {
                        int sx0, sx1, sy0, sy1;
                        const T *v0, *v1, *v2, *v3;
                        const AT* w = wtab + FXY[dx]*4;
                        if( borderType == BORDER_REPLICATE )
                        {
                            sx0 = clip(sx, 0, ssize.width);
                            sx1 = clip(sx+1, 0, ssize.width);
                            sy0 = clip(sy, 0, ssize.height);
                            sy1 = clip(sy+1, 0, ssize.height);
                            v0 = S0 + sy0*sstep + sx0*channelNum;
                            v1 = S0 + sy0*sstep + sx1*channelNum;
                            v2 = S0 + sy1*sstep + sx0*channelNum;
                            v3 = S0 + sy1*sstep + sx1*channelNum;
                        }
                        else if( borderType == BORDER_TRANSPARENT &&
                            ((unsigned)sx >= (unsigned)(ssize.width-1) ||
                            (unsigned)sy >= (unsigned)(ssize.height-1)))
                            continue;
                        else
                        {
                            sx0 = borderInterpolate(sx, ssize.width, borderType);
                            sx1 = borderInterpolate(sx+1, ssize.width, borderType);
                            sy0 = borderInterpolate(sy, ssize.height, borderType);
                            sy1 = borderInterpolate(sy+1, ssize.height, borderType);
                            v0 = sx0 >= 0 && sy0 >= 0 ? S0 + sy0*sstep + sx0*channelNum : &cval[0];
                            v1 = sx1 >= 0 && sy0 >= 0 ? S0 + sy0*sstep + sx1*channelNum : &cval[0];
                            v2 = sx0 >= 0 && sy1 >= 0 ? S0 + sy1*sstep + sx0*channelNum : &cval[0];
                            v3 = sx1 >= 0 && sy1 >= 0 ? S0 + sy1*sstep + sx1*channelNum : &cval[0];
                        }
                        for(int k = 0; k < channelNum; k++ )
                            D[k] = castOp(WT(v0[k]*w[0] + v1[k]*w[1] + v2[k]*w[2] + v3[k]*w[3]));
                    }
                }
        }
    }
}
}


template<class CastOp, typename AT, int ONE>
static void remapBicubic( const Mat& _src, Mat& _dst, const Mat& _xy,
                      const Mat& _fxy, const void* _wtab,
                      int borderType, const Scalar& _borderValue )
{
typedef typename CastOp::rtype T;
typedef typename CastOp::type1 WT;
Size ssize = _src.size(), dsize = _dst.size();
const int channelNum = _src.channels();
const AT* wtab = (const AT*)_wtab;
const T* S0 = _src.ptr<T>();
size_t sstep = _src.step/sizeof(S0[0]);
T cval[CV_CN_MAX];
CastOp castOp;

for(int k = 0; k < channelNum; k++ )
    cval[k] = saturate_cast<T>(_borderValue[k & 3]);

int borderType1 = borderType != BORDER_TRANSPARENT ? borderType : BORDER_REFLECT_101;

unsigned width1 = std::max(ssize.width-3, 0), height1 = std::max(ssize.height-3, 0);

if( _dst.isContinuous() && _xy.isContinuous() && _fxy.isContinuous() )
{
    dsize.width *= dsize.height;
    dsize.height = 1;
}

for(int dy = 0; dy < dsize.height; dy++ )
{
    T* D = _dst.ptr<T>(dy);
    const short* XY = _xy.ptr<short>(dy);
    const ushort* FXY = _fxy.ptr<ushort>(dy);

    for(int dx = 0; dx < dsize.width; dx++, D += channelNum )
    {
        int sx = XY[dx*2]-1, sy = XY[dx*2+1]-1;
        const AT* w = wtab + FXY[dx]*16;
        if( (unsigned)sx < width1 && (unsigned)sy < height1 )
        {
            const T* S = S0 + sy*sstep + sx*channelNum;
            for(int k = 0; k < channelNum; k++ )
            {
                WT sum = S[0]*w[0] + S[channelNum]*w[1] + S[channelNum*2]*w[2] + S[channelNum*3]*w[3];
                S += sstep;
                sum += S[0]*w[4] + S[channelNum]*w[5] + S[channelNum*2]*w[6] + S[channelNum*3]*w[7];
                S += sstep;
                sum += S[0]*w[8] + S[channelNum]*w[9] + S[channelNum*2]*w[10] + S[channelNum*3]*w[11];
                S += sstep;
                sum += S[0]*w[12] + S[channelNum]*w[13] + S[channelNum*2]*w[14] + S[channelNum*3]*w[15];
                S += 1 - sstep*3;
                D[k] = castOp(sum);
            }
        }
        else
        {
            int x[4], y[4];
            if( borderType == BORDER_TRANSPARENT &&
                ((unsigned)(sx+1) >= (unsigned)ssize.width ||
                (unsigned)(sy+1) >= (unsigned)ssize.height) )
                continue;

            if( borderType1 == BORDER_CONSTANT &&
                (sx >= ssize.width || sx+4 <= 0 ||
                sy >= ssize.height || sy+4 <= 0))
            {
                for(int k = 0; k < channelNum; k++ )
                    D[k] = cval[k];
                continue;
            }

            for(int i = 0; i < 4; i++ )
            {
                x[i] = borderInterpolate(sx + i, ssize.width, borderType1)*channelNum;
                y[i] = borderInterpolate(sy + i, ssize.height, borderType1);
            }

            for(int k = 0; k < channelNum; k++, S0++, w -= 16 )
            {
                WT cv = cval[k], sum = cv*ONE;
                for(int i = 0; i < 4; i++, w += 4 )
                {
                    int yi = y[i];
                    const T* S = S0 + yi*sstep;
                    if( yi < 0 )
                        continue;
                    if( x[0] >= 0 )
                        sum += (S[x[0]] - cv)*w[0];
                    if( x[1] >= 0 )
                        sum += (S[x[1]] - cv)*w[1];
                    if( x[2] >= 0 )
                        sum += (S[x[2]] - cv)*w[2];
                    if( x[3] >= 0 )
                        sum += (S[x[3]] - cv)*w[3];
                }
                D[k] = castOp(sum);
            }
            S0 -= channelNum;
        }
    }
}
}


template<class CastOp, typename AT, int ONE>
static void remapLanczos4( const Mat& _src, Mat& _dst, const Mat& _xy,
                       const Mat& _fxy, const void* _wtab,
                       int borderType, const Scalar& _borderValue )
{
typedef typename CastOp::rtype T;
typedef typename CastOp::type1 WT;
Size ssize = _src.size(), dsize = _dst.size();
const int channelNum = _src.channels();
const AT* wtab = (const AT*)_wtab;
const T* S0 = _src.ptr<T>();
size_t sstep = _src.step/sizeof(S0[0]);
T cval[CV_CN_MAX];
CastOp castOp;

for(int k = 0; k < channelNum; k++ )
    cval[k] = saturate_cast<T>(_borderValue[k & 3]);

int borderType1 = borderType != BORDER_TRANSPARENT ? borderType : BORDER_REFLECT_101;

unsigned width1 = std::max(ssize.width-7, 0), height1 = std::max(ssize.height-7, 0);

if( _dst.isContinuous() && _xy.isContinuous() && _fxy.isContinuous() )
{
    dsize.width *= dsize.height;
    dsize.height = 1;
}

for(int dy = 0; dy < dsize.height; dy++ )
{
    T* D = _dst.ptr<T>(dy);
    const short* XY = _xy.ptr<short>(dy);
    const ushort* FXY = _fxy.ptr<ushort>(dy);

    for(int dx = 0; dx < dsize.width; dx++, D += channelNum )
    {
        int sx = XY[dx*2] - 3,  sy = XY[dx*2+1] - 3;
        const AT* w = wtab + FXY[dx]*64;
        const T* S = S0 + sy * sstep + sx * channelNum;
        if ( (unsigned) sx < width1 && (unsigned) sy < height1 )
                for (int k = 0; k < channelNum; k++ )
                {
                    WT sum = 0;
                    for( int r = 0; r < 8; r++, S += sstep, w += 8 )
                            sum += S[0] * w[0] + S[channelNum] * w[1] + S[channelNum*2] * w[2] + S[channelNum*3] * w[3] +
                                            S[channelNum*4] * w[4] + S[channelNum*5] * w[5] + S[channelNum*6] * w[6] + S[channelNum*7] * w[7];
                    w -= 64;
                    S -= sstep*8 - 1;
                    D[k] = castOp(sum);
                }
        else
        {
            int x[8], y[8];
            if( borderType == BORDER_TRANSPARENT &&
                ((unsigned)(sx+3) >= (unsigned)ssize.width ||
                (unsigned)(sy+3) >= (unsigned)ssize.height) )
                continue;

            if( borderType1 == BORDER_CONSTANT &&
                (sx >= ssize.width || sx+8 <= 0 ||
                sy >= ssize.height || sy+8 <= 0))
            {
                for(int k = 0; k < channelNum; k++ )
                    D[k] = cval[k];
                continue;
            }

            for(int i = 0; i < 8; i++ )
            {
                x[i] = borderInterpolate(sx + i, ssize.width, borderType1)*channelNum;
                y[i] = borderInterpolate(sy + i, ssize.height, borderType1);
            }

            for(int k = 0; k < channelNum; k++, S0++, w -= 64 )
            {
                WT cv = cval[k], sum = cv*ONE;
                for(int i = 0; i < 8; i++, w += 8 )
                {
                    int yi = y[i];
                    const T* S1 = S0 + yi*sstep;
                    if( yi < 0 )
                        continue;
                    if( x[0] >= 0 )
                        sum += (S1[x[0]] - cv)*w[0];
                    if( x[1] >= 0 )
                        sum += (S1[x[1]] - cv)*w[1];
                    if( x[2] >= 0 )
                        sum += (S1[x[2]] - cv)*w[2];
                    if( x[3] >= 0 )
                        sum += (S1[x[3]] - cv)*w[3];
                    if( x[4] >= 0 )
                        sum += (S1[x[4]] - cv)*w[4];
                    if( x[5] >= 0 )
                        sum += (S1[x[5]] - cv)*w[5];
                    if( x[6] >= 0 )
                        sum += (S1[x[6]] - cv)*w[6];
                    if( x[7] >= 0 )
                        sum += (S1[x[7]] - cv)*w[7];
                }
                D[k] = castOp(sum);
            }
            S0 -= channelNum;
        }
    }
}
}


typedef void (*RemapNNFunc)(const Mat& _src, Mat& _dst,
                        const Mat& _xy,
                        int borderType, const Scalar& _borderValue );

typedef void (*RemapFunc)(const Mat& _src, Mat& _dst,
                      const Mat& _xy, const Mat& _fxy, const void* _wtab,
                      int borderType, const Scalar& _borderValue);


class RemapInvoker : public ParallelLoopBody
{
public:
RemapInvoker(
            const Mat& _src,
            Mat& _dst,
            const Mat *_m1, const Mat *_m2,
            int _borderType, const Scalar &_borderValue,
            int _planar_input,
            RemapNNFunc _nnfunc,
            RemapFunc _ifunc,
            const void *_ctab) :
    ParallelLoopBody(),
    src(&_src), dst(&_dst),
    m1(_m1), m2(_m2),
    borderType(_borderType), borderValue(_borderValue),
    planar_input(_planar_input),
    nnfunc(_nnfunc),  ifunc(_ifunc),  ctab(_ctab) { }

virtual void operator() (const Range& range) const CV_OVERRIDE
{
    int x, y, x1, y1;
    const int buf_size = 1 << 14;
    int brows0 = std::min(128, dst->rows), map_depth = m1->depth();
    int bcols0 = std::min(buf_size/brows0, dst->cols);
    brows0 = std::min(buf_size/bcols0, dst->rows);

    Mat _bufxy(brows0, bcols0, CV_16SC2), _bufa;
    if( !nnfunc )
            _bufa.create(brows0, bcols0, CV_16UC1);

    for( y = range.start; y < range.end; y += brows0 )
            for( x = 0; x < dst->cols; x += bcols0 )
            {
                int brows = std::min(brows0, range.end - y);
                int bcols = std::min(bcols0, dst->cols - x);
                Mat dpart(*dst, Rect(x, y, bcols, brows));
                Mat bufxy(_bufxy, Rect(0, 0, bcols, brows));

                if( nnfunc )
                {
                    if( m1->type() == CV_16SC2 && m2->empty() ) // the data is already in the right format
                            bufxy = (*m1)(Rect(x, y, bcols, brows));
                    else if( map_depth != CV_32F )
                    {
                        for( y1 = 0; y1 < brows; y1++ )
                        {
                            short* XY = bufxy.ptr<short>(y1);
                            const short* sXY = m1->ptr<short>(y+y1) + x*2;
                            const ushort* sA = m2->ptr<ushort>(y+y1) + x;

                            for( x1 = 0; x1 < bcols; x1++ )
                            {
                                int a = sA[x1] & (INTER_TAB_SIZE2-1);
                                XY[x1*2] = sXY[x1*2] + NNDeltaTab_i[a][0];
                                XY[x1*2+1] = sXY[x1*2+1] + NNDeltaTab_i[a][1];
                            }
                        }
                    } else
                        if( !planar_input )
                                (*m1)(Rect(x, y, bcols, brows)).convertTo(bufxy, bufxy.depth());
                        else
                            for( y1 = 0; y1 < brows; y1++ )
                            {
                                short* XY = bufxy.ptr<short>(y1);
                                const float* sX = m1->ptr<float>(y+y1) + x;
                                const float* sY = m2->ptr<float>(y+y1) + x;
                                x1 = 0;

                                #if CV_SIMD128
                                {
                                    int span = v_float32x4::nlanes;
                                    for( ; x1 <= bcols - span * 2; x1 += span * 2 )
                                    {
                                        v_int32x4 ix0 = v_round(v_load(sX + x1));
                                        v_int32x4 iy0 = v_round(v_load(sY + x1));
                                        v_int32x4 ix1 = v_round(v_load(sX + x1 + span));
                                        v_int32x4 iy1 = v_round(v_load(sY + x1 + span));

                                        v_int16x8 dx, dy;
                                        dx = v_pack(ix0, ix1);
                                        dy = v_pack(iy0, iy1);
                                        v_store_interleave(XY + x1 * 2, dx, dy);
                                    }
                                }
                                #endif
                                for( ; x1 < bcols; x1++ )
                                {
                                    XY[x1*2] = saturate_cast<short>(sX[x1]);
                                    XY[x1*2+1] = saturate_cast<short>(sY[x1]);
                                }
                            }

                    nnfunc( *src, dpart, bufxy, borderType, borderValue );

                    continue;
                } //if( nnfunc )

                Mat bufa(_bufa, Rect(0, 0, bcols, brows));
                for( y1 = 0; y1 < brows; y1++ )
                {
                    short* XY = bufxy.ptr<short>(y1);
                    ushort* A = bufa.ptr<ushort>(y1);

                    if( m1->type() == CV_16SC2 && (m2->type() == CV_16UC1 || m2->type() == CV_16SC1) )
                    {
                        bufxy = (*m1)(Rect(x, y, bcols, brows));

                        const ushort* sA = m2->ptr<ushort>(y+y1) + x;
                        x1 = 0;

                        #if CV_SIMD128
                        {
                            v_uint16x8 v_scale = v_setall_u16(INTER_TAB_SIZE2 - 1);
                            int span = v_uint16x8::nlanes;
                            for( ; x1 <= bcols - span; x1 += span )
                                v_store((unsigned short*)(A + x1), v_load(sA + x1) & v_scale);
                        }
                        #endif

                        for( ; x1 < bcols; x1++ )
                                A[x1] = (ushort)(sA[x1] & (INTER_TAB_SIZE2 - 1));

                    } else if( planar_input ) {
                        const float* sX = m1->ptr<float>(y+y1) + x;
                        const float* sY = m2->ptr<float>(y+y1) + x;

                        x1 = 0;
                        #if CV_SIMD128
                        {
                            v_float32x4 v_scale = v_setall_f32((float)INTER_TAB_SIZE);
                            v_int32x4 v_scale2 = v_setall_s32(INTER_TAB_SIZE - 1);
                            int span = v_float32x4::nlanes;
                            for( ; x1 <= bcols - span * 2; x1 += span * 2 )
                            {
                                v_int32x4 v_sx0 = v_round(v_scale * v_load(sX + x1));
                                v_int32x4 v_sy0 = v_round(v_scale * v_load(sY + x1));
                                v_int32x4 v_sx1 = v_round(v_scale * v_load(sX + x1 + span));
                                v_int32x4 v_sy1 = v_round(v_scale * v_load(sY + x1 + span));
                                v_uint16x8 v_sx8 = v_reinterpret_as_u16(v_pack(v_sx0 & v_scale2, v_sx1 & v_scale2));
                                v_uint16x8 v_sy8 = v_reinterpret_as_u16(v_pack(v_sy0 & v_scale2, v_sy1 & v_scale2));
                                v_uint16x8 v_v = v_shl<INTER_BITS>(v_sy8) | (v_sx8);
                                v_store(A + x1, v_v);

                                v_int16x8 v_d0 = v_pack(v_shr<INTER_BITS>(v_sx0), v_shr<INTER_BITS>(v_sx1));
                                v_int16x8 v_d1 = v_pack(v_shr<INTER_BITS>(v_sy0), v_shr<INTER_BITS>(v_sy1));
                                v_store_interleave(XY + (x1 << 1), v_d0, v_d1);
                            }
                        }
                        #endif
                        for( ; x1 < bcols; x1++ )
                        {
                            int sx = cvRound(sX[x1]*INTER_TAB_SIZE);
                            int sy = cvRound(sY[x1]*INTER_TAB_SIZE);
                            int v = (sy & (INTER_TAB_SIZE-1))*INTER_TAB_SIZE + (sx & (INTER_TAB_SIZE-1));
                            XY[x1*2] = saturate_cast<short>(sx >> INTER_BITS);
                            XY[x1*2+1] = saturate_cast<short>(sy >> INTER_BITS);
                            A[x1] = (ushort)v;
                        }
                    } else {
                        const float* sXY = m1->ptr<float>(y+y1) + x*2;
                        x1 = 0;

                        #if CV_SIMD128
                        {
                            v_float32x4 v_scale = v_setall_f32((float)INTER_TAB_SIZE);
                            v_int32x4 v_scale2 = v_setall_s32(INTER_TAB_SIZE - 1), v_scale3 = v_setall_s32(INTER_TAB_SIZE);
                            int span = v_float32x4::nlanes;
                            for( ; x1 <= bcols - span * 2; x1 += span * 2 )
                            {
                                v_float32x4 v_fx, v_fy;
                                v_load_deinterleave(sXY + (x1 << 1), v_fx, v_fy);
                                v_int32x4 v_sx0 = v_round(v_fx * v_scale);
                                v_int32x4 v_sy0 = v_round(v_fy * v_scale);
                                v_load_deinterleave(sXY + ((x1 + span) << 1), v_fx, v_fy);
                                v_int32x4 v_sx1 = v_round(v_fx * v_scale);
                                v_int32x4 v_sy1 = v_round(v_fy * v_scale);
                                v_int32x4 v_v0 = v_muladd(v_scale3, (v_sy0 & v_scale2), (v_sx0 & v_scale2));
                                v_int32x4 v_v1 = v_muladd(v_scale3, (v_sy1 & v_scale2), (v_sx1 & v_scale2));
                                v_uint16x8 v_v8 = v_reinterpret_as_u16(v_pack(v_v0, v_v1));
                                v_store(A + x1, v_v8);
                                v_int16x8 v_dx = v_pack(v_shr<INTER_BITS>(v_sx0), v_shr<INTER_BITS>(v_sx1));
                                v_int16x8 v_dy = v_pack(v_shr<INTER_BITS>(v_sy0), v_shr<INTER_BITS>(v_sy1));
                                v_store_interleave(XY + (x1 << 1), v_dx, v_dy);
                            }
                        }
                        #endif

                        for( ; x1 < bcols; x1++ )
                        {
                            int sx = cvRound(sXY[x1*2]*INTER_TAB_SIZE);
                            int sy = cvRound(sXY[x1*2+1]*INTER_TAB_SIZE);
                            int v = (sy & (INTER_TAB_SIZE-1))*INTER_TAB_SIZE + (sx & (INTER_TAB_SIZE-1));
                            XY[x1*2] = saturate_cast<short>(sx >> INTER_BITS);
                            XY[x1*2+1] = saturate_cast<short>(sy >> INTER_BITS);
                            A[x1] = (ushort)v;
                        }
                    }
                }

                ifunc(*src, dpart, bufxy, bufa, ctab, borderType, borderValue);
            }
}

private:
const Mat* src;
Mat* dst;
const Mat *m1, *m2;
int borderType;
Scalar borderValue;
int planar_input;
RemapNNFunc nnfunc;
RemapFunc ifunc;
const void *ctab;
};


void remap( InputArray _src, OutputArray _dst,
            InputArray _map1, InputArray _map2,
            int interpolation, int borderType, const Scalar& borderValue )
{
CV_INSTRUMENT_REGION();

static RemapNNFunc nn_tab[] =
{
    remapNearest<uchar>, remapNearest<schar>, remapNearest<ushort>, remapNearest<short>,
    remapNearest<int>, remapNearest<float>, remapNearest<double>, 0
};

static RemapFunc linear_tab[] =
{
    remapBilinear< FixedPtCast<int, uchar, INTER_REMAP_COEF_BITS>, RemapVec_8u, short>, 0,
    remapBilinear<Cast<float, ushort>, RemapNoVec, float>,
    remapBilinear<Cast<float, short>, RemapNoVec, float>, 0,
    remapBilinear<Cast<float, float>, RemapNoVec, float>,
    remapBilinear<Cast<double, double>, RemapNoVec, float>, 0
};

static RemapFunc cubic_tab[] =
{
    remapBicubic<FixedPtCast<int, uchar, INTER_REMAP_COEF_BITS>, short, INTER_REMAP_COEF_SCALE>, 0,
    remapBicubic<Cast<float, ushort>, float, 1>,
    remapBicubic<Cast<float, short>, float, 1>, 0,
    remapBicubic<Cast<float, float>, float, 1>,
    remapBicubic<Cast<double, double>, float, 1>, 0
};

static RemapFunc lanczos4_tab[] =
{
    remapLanczos4<FixedPtCast<int, uchar, INTER_REMAP_COEF_BITS>, short, INTER_REMAP_COEF_SCALE>, 0,
    remapLanczos4<Cast<float, ushort>, float, 1>,
    remapLanczos4<Cast<float, short>, float, 1>, 0,
    remapLanczos4<Cast<float, float>, float, 1>,
    remapLanczos4<Cast<double, double>, float, 1>, 0
};

CV_Assert( !_map1.empty() );
CV_Assert( _map2.empty() || (_map2.size() == _map1.size()));

CV_OCL_RUN(_src.dims() <= 2 && _dst.isUMat(),
           ocl_remap(_src, _dst, _map1, _map2, interpolation, borderType, borderValue))

Mat src = _src.getMat(), map1 = _map1.getMat(), map2 = _map2.getMat();
_dst.create( map1.size(), src.type() );
Mat dst = _dst.getMat();


CV_OVX_RUN(
    src.type() == CV_8UC1 && dst.type() == CV_8UC1 &&
    !ovx::skipSmallImages<VX_KERNEL_REMAP>(src.cols, src.rows) &&
    (borderType& ~BORDER_ISOLATED) == BORDER_CONSTANT &&
    ((map1.type() == CV_32FC2 && map2.empty() && map1.size == dst.size) ||
     (map1.type() == CV_32FC1 && map2.type() == CV_32FC1 && map1.size == dst.size && map2.size == dst.size) ||
     (map1.empty() && map2.type() == CV_32FC2 && map2.size == dst.size)) &&
    ((borderType & BORDER_ISOLATED) != 0 || !src.isSubmatrix()),
    openvx_remap(src, dst, map1, map2, interpolation, borderValue));

CV_Assert( dst.cols < SHRT_MAX && dst.rows < SHRT_MAX && src.cols < SHRT_MAX && src.rows < SHRT_MAX );

if( dst.data == src.data )
    src = src.clone();

if( interpolation == INTER_AREA )
    interpolation = INTER_LINEAR;

int type = src.type(), depth = CV_MAT_DEPTH(type);

#if defined HAVE_IPP && !IPP_DISABLE_REMAP
CV_IPP_CHECK()
{
    if ((interpolation == INTER_LINEAR || interpolation == INTER_CUBIC || interpolation == INTER_NEAREST) &&
            map1.type() == CV_32FC1 && map2.type() == CV_32FC1 &&
            (borderType == BORDER_CONSTANT || borderType == BORDER_TRANSPARENT))
    {
        int ippInterpolation =
            interpolation == INTER_NEAREST ? IPPI_INTER_NN :
            interpolation == INTER_LINEAR ? IPPI_INTER_LINEAR : IPPI_INTER_CUBIC;

        ippiRemap ippFunc =
            type == CV_8UC1 ? (ippiRemap)ippiRemap_8u_C1R :
            type == CV_8UC3 ? (ippiRemap)ippiRemap_8u_C3R :
            type == CV_8UC4 ? (ippiRemap)ippiRemap_8u_C4R :
            type == CV_16UC1 ? (ippiRemap)ippiRemap_16u_C1R :
            type == CV_16UC3 ? (ippiRemap)ippiRemap_16u_C3R :
            type == CV_16UC4 ? (ippiRemap)ippiRemap_16u_C4R :
            type == CV_32FC1 ? (ippiRemap)ippiRemap_32f_C1R :
            type == CV_32FC3 ? (ippiRemap)ippiRemap_32f_C3R :
            type == CV_32FC4 ? (ippiRemap)ippiRemap_32f_C4R : 0;

        if (ippFunc)
        {
            bool ok;
            IPPRemapInvoker invoker(src, dst, map1, map2, ippFunc, ippInterpolation,
                                    borderType, borderValue, &ok);
            Range range(0, dst.rows);
            parallel_for_(range, invoker, dst.total() / (double)(1 << 16));

            if (ok)
            {
                CV_IMPL_ADD(CV_IMPL_IPP|CV_IMPL_MT);
                return;
            }
            setIppErrorStatus();
        }
    }
}
#endif

RemapNNFunc nnfunc = 0;
RemapFunc ifunc = 0;
const void* ctab = 0;
bool fixpt = depth == CV_8U;
bool planar_input = false;

if( interpolation == INTER_NEAREST )
{
    nnfunc = nn_tab[depth];
    CV_Assert( nnfunc != 0 );
} else {
    if( interpolation == INTER_LINEAR )
            ifunc = linear_tab[depth];
    else if( interpolation == INTER_CUBIC )
{
        ifunc = cubic_tab[depth];
            CV_Assert( _src.channels() <= 4 );
        }
    else if( interpolation == INTER_LANCZOS4 ){
        ifunc = lanczos4_tab[depth];
        CV_Assert( _src.channels() <= 4 );
    } else
        CV_Error( CV_StsBadArg, "Unknown interpolation method" );
    CV_Assert( ifunc != 0 );
    ctab = initInterTab2D( interpolation, fixpt );
}

const Mat *m1 = &map1, *m2 = &map2;

if( (map1.type() == CV_16SC2 && (map2.type() == CV_16UC1 || map2.type() == CV_16SC1 || map2.empty())) ||
    (map2.type() == CV_16SC2 && (map1.type() == CV_16UC1 || map1.type() == CV_16SC1 || map1.empty())) )
{
    if( map1.type() != CV_16SC2 )
        std::swap(m1, m2);
}
else
{
    CV_Assert( ((map1.type() == CV_32FC2 || map1.type() == CV_16SC2) && map2.empty()) ||
        (map1.type() == CV_32FC1 && map2.type() == CV_32FC1) );
    planar_input = map1.channels() == 1;
}

RemapInvoker invoker(src, dst, m1, m2,
                     borderType, borderValue, planar_input, nnfunc, ifunc,
                     ctab);
parallel_for_(Range(0, dst.rows), invoker, dst.total()/(double)(1<<16));
}

}
#endif
