#ifndef EXPOSURES_H
#define EXPOSURES_H

#include "cameras.h"
#include <string>
using namespace std;
using namespace nsCameras;

#include <QDialog>
#include <QLineEdit>
#include <QDir>

namespace Ui {
class Exposures;
}

class Exposures : public QDialog
{
    Q_OBJECT

public:
    explicit Exposures(QWidget *parent = nullptr);
    ~Exposures();

public slots:
    void on_edLeftCameraExpo_returnPressed();
    void on_edRightCameraExpo_returnPressed();
    void on_kbHWTrigger_clicked(bool checked);
    void on_kbSavePhotos_clicked(bool checked);
    void on_kbLoadPhotos_clicked(bool checked);

private slots:
    //void on_buttonBox_accepted();

    void on_btExit_clicked();

    //void on_lineEdit_returnPressed();


    void on_btExit_pressed();

    //void on_cbMode_activated(const QString &arg1);

    void on_cbMode_currentIndexChanged(int index);

    void on_kbRotate90_stateChanged(int arg1);

    void on_sbKBHorzGrids_textChanged(const QString &arg1);

    void on_sbKBVertGrids_textChanged(const QString &arg1);

    //void on_edLeftCameraExpo_2_returnPressed();

    //void on_edSquareBarLength_returnPressed();

    void on_edHorzGridRange_returnPressed();

    void on_edVertGridRange_returnPressed();

    void OnKBHorzGridsSBChanged();
    void OnKBVertGridsSBChanged();

    void on_kbHWTrigger_stateChanged(int arg1);

    void on_edVHVertOffset_returnPressed();

    void on_edSquareBarWidth_returnPressed();

    void on_sbBoxID_textChanged(const QString &arg1);

    void on_kbSavePhotos_stateChanged(int arg1);

    void on_edLeftCameraExpo_cursorPositionChanged(int arg1, int arg2);

    void on_kbToggleTibboStartStop_stateChanged(int arg1);

    void on_kbLoadPhotos_stateChanged(int arg1);

    void on_sbBoxID_valueChanged(int arg1);

    void on_kbHWTrigger_clicked();


    void on_kbRotate90_clicked(bool checked);



    void on_edSquareBarLength_returnPressed();

    void on_edFromAngle_returnPressed();

    void on_edToAngle_returnPressed();

    void on_sbGvMSteps_valueChanged(int arg1);

private:
//    Ui::Exposures *ui;

public:
    Ui::Exposures *ui;
    void *pMainForm = nullptr;
    bool areCameras90DgrRotated=true;  //accross modes
    SKBGrids KBGrids ={4, 3, 6.22, 4.44};
    //constexpr double GRID_DIM_X = 6.22 / 7 = 0.88857
    //constexpr double GRID_DIM_Y = 4.44 / 5 = 0.88

    //BS
#ifdef _MISSION_BS
    double SBWidth = 40.0;
    double SBLength = 3000.0;
#endif
    double VHVertOffset = 10.0;
    double VFineTune = 1.0;
    double HFineTune = 1.0;
    double SERFineTune = 0.0;
    double SEVOffset = 0.0;
    double SEHOffset = 0.0;

    //XS
#ifdef _MISSION_XS
    double SBWidth = 12.0;
    double SBLength = 500.0;
#endif
    double GvMSwingFrom4BCC = +30.0;
    double GvMSwingTo4BCC = +20.0;
    double GvMSwingSteps4BCC = 10;
    double GvMSwingFromPlate = +30.0;
    double GvMSwingToPlate = -30.0;
    double GvMSwingStepsPlate = 100;

    //RectifyRatioX==5
    double FOCUS_TUNE[C_TotalBoxes] = {  //M#
                                         0.44794,  //-9354(4200)  +4040(4600)   +0022(448)  +0005(44795)  +0002#
                                         0.38755, //0986(3901) +0599(3891) +0173(388)  -0988(385)  -0098(3873) +0018(3876)  -0001#
                                         0.38310, //+1254(3863)  -1995(3780)  -1212(3800)  +0745(3850)  +0157(3835) -0039(3830) +0001#
                                         0.39413, //+1435(39790) +1420(39786)  +0712(396)  +0332(395) +27(3942) -38(39403)   -7(39411) -3(39412) 0#
                                                    //0.39786, //-2966(390)  -1080(395)  +1558(402)  +0767(3999)  -0025(3978) -0002#
                                         0.39304 //-1924(388)  -0397(392) +0061(3932) +0023(3931)  +0003(39305)  0#
 /*  DEBUG
                                              0.44662, //0.39108,  +1136(450)  +0128(447)  -004(4465)  -0006(4466)
                                              0.39215, //0.3901, +109(395)  +0707(394) -0058(392) +0248(3928) +0019(3922) +0011(39218)  -0012(39212) 0#
                                              0.39284, //0.38638,+044(394)  +006(393) -0093(3926)  -0044(39283)  +0021(3929)  +0014(39288)  -0002(39284)
                                              0.39689, //0.38009, +019(3974)  +004(3970) + -0093(3968) --0005(39688) -0001(39689)
                                              0.39216, //0.38112 +1086(395)  +070(394)  -0062(3920)  +0091(3924) +0038(39226)  +0030(39224)  +0015(3922)  -0005(39215)
    /**/
                                                        };

    double ReprojErrTrn=5.0;
    double BarSectEstTrn=2;
    int LFBeforeASOR=-1;  //Load from file or not before Automatic Save One Round
    int FSBeforeASOR=-1;  //File-saving or not before Automatic Save One Round
    int HWTBeforeACSE=-1;  //HW Trigger or not before Automatic Calibrate SE
    String SOR_Dir="";

    void SetupMode(int newMode) ;
    void SetupBoxID(int newID);
    void setupCameraExposure(int whichCamera, double expoTime);
    void RefreshParameterGUIs();
    int OnCameraExpoReturnPressed(int whichCamera, QLineEdit* expoEdit);
    int OnRotate90CheckBoxChanged(bool newState);
    void SetInitializedGUIToVariables(bool);
    bool willLoadFromFiles();
    bool IsHardwareTriggered();
    bool IsSavingCapturedPhoto();
};

#endif // EXPOSURES_H
