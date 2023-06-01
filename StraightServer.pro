QT       += core network
QT       += gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11
#CONFIG  += MISSION_BS    #for BS British Steel
CONFIG  += MISSION_BS    #for XS Xiang Steel

TARGET = StraightServer

TEMPLATE = app

#DESTDIR = \"C:\Users\user\Documents\ComputerVision\Working_Code\StraightServer\"

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
#DEFINES += PYLON_WIN_BUILD$$

DEFINES +=  _SHOW_TIBBO_MSG
DEFINES += _BOX_PLAY_CLIENT
DEFINES +=  _APPLY_FIXED_DIR  #if _CALIB_AT_NEIHU BOX_ID MUST be Box-5


#DEFINES +=  _DEBUG_OUTLIER

MISSION_BS {
    DEFINES += _MISSION_BS
    DEFINES += _CAMERA_MV
    DEFINES +=  _CAMERA_GLOBAL
    DEFINES += DIRECT_QUEUE  #callback as MV ,
    #DEFINES +=  _SAVE_TO_MONITOR  #Auto SE
    DEFINES +=  _DEBUG_SE  #Auto SE

#2/4 fine, 3 exposes less, update 5
#1 still most potentially problematic?!
}

MISSION_XS {
    DEFINES += _MISSION_XS
    #DEFINES += _CAMERA_MV
    DEFINES += _CAMERA_BASLER
    #DEFINES +=  _CAMERA_ROLLING
    DEFINES +=  _CAMERA_GLOBAL
    #DEFINES += DIRECT_QUEUE  #callback as MV ,
    #DEFINES +=  _DEBUG_FILE
    #DEFINES +=  _DEBUG_ONE_CAMERA
}

#DEFINES += _CALIB_AT_NEIHU  #if so, CalibrateDir = "/home/user/Code/SG/SG_Box/execute/"

#DEFINES +=  _SIMULATE_ALL_MPC_SEVERS
#DEFINES +=  _RALPH_SIMULATE

#DEFINES +=  _DEBUG_MUTEX
#DEFINES +=  _DEBUG_SHOT_TmSmp
#DEFINES +=  _DISPLAY_LOADED
#DEFINES +=  _BOX_CLIENT_OBSOLETE

DEFINES += ROI_IMAGE
DEFINES += LCK_GRD
DEFINES += LCK_GRD_Q
DEFINES += CB_NEED_MTX
#DEFINES += HWT_CP_THD  # timer enough?!  _SMLT_TIBBO
#DEFINES += SMLT_HWT

#_WINDOWS
# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


HEADERS += \
    cameras.h \
    exposures.h \
    mainwindow.h

FORMS += \
    exposures.ui \
    mainwindow.ui

SOURCES += \
    cameras.cpp \
    exposures.cpp \
    main.cpp \
    mainwindow.cpp


win32{
        DEFINES +=  OS_WINDOWS
        #        DEFINES += _WINDOWS CPU32 _NOT_VC
        INCLUDEPATH +=  "C:/Program Files/Basler\pylon 6/Development/include"
        INCLUDEPATH +=  "C:/Program Files (x86)/MindVision/Demo/VC++/Include"
        #INCLUDEPATH +=  \"C:\Program Files\Basler\pylon 6\Development\include\"
        #INCLUDEPATH += \"$(PYLON_DEV_DIR)\include\"

        LIBS += -L"C:/Program Files/Basler/pylon 6/Development/lib/x64"
        LIBS += -L"C:\Program Files (x86)\MindVision\SDK\X64"
        #LIBS += $(PYLON_DEV_DIR)\lib\x64

        LIBS += -lMVCAMSDK_X64     #MV
        #DLLDESTDIR +=  "C:/Users/jeffrey/RunStraightServer"
        DLLDESTDIR +=  C:\Users\jeffrey\RunStraightServer\
} else {
        DEFINES +=  OS_LINUX
        INCLUDEPATH +=  "/opt/local/include"
        INCLUDEPATH +=  "/usr/include"
        INCLUDEPATH +=  "/usr/local/include"

        LIBS += -L"/usr/lib"
        LIBS += -L"/usr/local/lib"
#        DEFINES += _LINUX CPU64
        INCLUDEPATH +=  "/opt/pylon/include"
        LIBS += -L"/opt/pylon/lib"
        LIBS += -lpylonbase
        LIBS += -lGenApi_gcc_v3_1_Basler_pylon
        LIBS += -lGCBase_gcc_v3_1_Basler_pylon
        LIBS += -lpylonutility

        LIBS += -lMVSDK

        QMAKE_LFLAGS += -Wl,--rpath=\\\$\$ORIGIN/lib/Qt6/lib
        QMAKE_LFLAGS += "-Wl,--rpath=\'\$$ORIGIN/lib\'"
        QMAKE_LFLAGS += "-Wl,--rpath=\'\$$ORIGIN/lib/Qt6/lib\'"
        QMAKE_LFLAGS += "-Wl,--rpath=\'\$$ORIGIN/lib/Qt6/plugin\'"
        #QMAKE_LFLAGS_PLUGIN += "-Wl,--rpath=\'\$$ORIGIN/lib/Qt6/plugins\'"

       #QMAKE_RPATHDIR = ./lib/Qt6/lib

}


CONFIG(debug, debug|release) {
        CONFIG += console
        #C:\OpenCV_4.5.4_x64_vc2017\install\x64\vc15\lib
        win32{
             INCLUDEPATH += C:\OpenCV\4.5.4_x64_vc2017_Debug\install\include
             DEPENDPATH += C:\OpenCV\4.5.4_x64_vc2017_Debug\install\include
             LIBS += -LC:\OpenCV\4.5.4_x64_vc2017_Debug\install\x64\vc15\bin
             LIBS += -LC:\OpenCV\4.5.4_x64_vc2017_Debug\install\x64\vc15\lib
             LIBS += -lopencv_calib3d454d
             LIBS += -lopencv_core454d
             LIBS += -lopencv_features2d454d
             LIBS += -lopencv_highgui454d
             LIBS += -lopencv_imgcodecs454d
             LIBS += -lopencv_imgproc454d
             LIBS += -lopencv_photo454d
             LIBS += -lopencv_video454d
             LIBS += -lopencv_videoio454d
        } else {
            INCLUDEPATH += "/usr/local/include/opencv4"
            DEPENDPATH += "/usr/local/include/opencv4"
            LIBS += -lopencv_calib3d
            LIBS += -lopencv_core
            LIBS += -lopencv_features2d
            LIBS += -lopencv_highgui
            LIBS += -lopencv_imgcodecs
            LIBS += -lopencv_imgproc
            LIBS += -lopencv_photo
            LIBS += -lopencv_video
            LIBS += -lopencv_videoio
        }
        #win32 {  LIBS += -LDsp32b32c.dll }
         #LIBS += -DC:\OpenCV\4.5.4_x64_vc2017_Debug\bin\Debug

        #C:\OpenCV_4.5.4_x64_vc2017\install\include
             #DEFINES +=  _DEBUG
             #DEFINES +=  _DEBUG_FILE

        #DEFINES += SG_BOX_RD    #o.w. CalibrateDir would use WorkDirBox1_5
        #DEFINES += _TEST_LOCALHOST   #mainly for QHostAddress::LocalHost
}  #CONFIG(debug, debug|release) {


CONFIG(release, debug|release) {
        win32{
             INCLUDEPATH += C:\OpenCV\4.5.4_x64_vc2017_Release\install\include
             DEPENDPATH += C:\OpenCV\4.5.4_x64_vc2017_Release\install\include
             LIBS += -LC:\OpenCV\4.5.4_x64_vc2017_Release\install\x64\vc15\lib
             LIBS += -L"C:\OpenCV\4.5.4_x64_vc2017_Release\install\x64\vc15\lib"

             LIBS += -lopencv_calib3d454
             LIBS += -lopencv_core454
             LIBS += -lopencv_features2d454
             LIBS += -lopencv_highgui454
             LIBS += -lopencv_imgcodecs454
             LIBS += -lopencv_imgproc454
             LIBS += -lopencv_photo454
             LIBS += -lopencv_video454
             LIBS += -lopencv_videoio454
        } else {
            INCLUDEPATH += "/usr/local/include/opencv4"
            DEPENDPATH += "/usr/local/include/opencv4"
            LIBS += -lopencv_calib3d
            LIBS += -lopencv_core
            LIBS += -lopencv_features2d
            LIBS += -lopencv_highgui
            LIBS += -lopencv_imgcodecs
            LIBS += -lopencv_imgproc
            LIBS += -lopencv_photo
            LIBS += -lopencv_video
            LIBS += -lopencv_videoio
        }

} #CONFIG(release, debug|release) {



#INCLUDEPATH +=  "C:\Program Files\Basler\pylon 6\Development\include"
#INCLUDEPATH +=  \"C:\Program Files\Basler\pylon 6\Development\include\\"

#DEPENDPATH +=  "C:\Program Files\Basler\pylon 6\Development\lib\Win32"

#    -lGenApi_MD_VC120_v3_0_Basler_pylon_v6_0 \
#    -lGCBase_MD_VC120_v3_0_Basler_pylon_v5_0 \
#    -lPylonBase_MD_VC120_v6_0

#/Program Files/Basler/pylon 6/Development/include
#\"C:\Program Files\Basler\pylon 6\Development\include\\"

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
