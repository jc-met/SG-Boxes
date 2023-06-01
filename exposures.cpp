#include "exposures.h"
#include "ui_exposures.h"

#include "mainwindow.h"

Exposures::Exposures(QWidget *parent) :
    QDialog(parent), ui(new Ui::Exposures)
{
        ui->setupUi(this);
         setWindowFlag(Qt::WindowStaysOnTopHint);
         setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);

         ui->cbMode->setEnabled(false);

         ui->sbBoxID->setMaximum(C_TotalBoxes);

#ifndef _SMLT_TIBBO
         ui->kbToggleTibboStartStop->setVisible(false);
#endif
#ifdef _DEBUG
         //ui->kbLoadPhotos->setCheckState(Qt::Checked);
#endif

#ifdef _MISSION_BS
         ui->gbSquareBar->setTitle("Square Bar");
         ui->lbSEDim->setText("Cross width");
#endif

#ifdef _MISSION_XS
         ui->gbSquareBar->setTitle("Square Rods");
         ui->lbSEDim->setText("4BCC Dim");
#endif

}

Exposures::~Exposures()
{
        delete ui;
}

void Exposures::SetupMode(int newMode)
{
        ui->cbMode->setCurrentIndex(newMode); //  currentIndex()

        on_cbMode_currentIndexChanged( ui->cbMode->currentIndex() );

}

void Exposures::SetupBoxID(int newID)
{
        ui->sbBoxID->setValue(newID);
        //on_cbMode_currentIndexChanged( ui->cbMode->currentIndex() );
}

void Exposures::on_sbBoxID_textChanged(const QString &arg1)
{
}

void Exposures::on_sbBoxID_valueChanged(int arg1)
{
        MainWindow  *fmMain = (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        if (fmMain)
        {
                if (fmMain->opMode != OM_IDLE)
                {
                        QMessageBox::warning(this, "When to change ID",  "Box ID can be changed only in Idle mode.");
                        ui->sbBoxID->setValue(fmMain->BOX_ID);
                        return;
                }
                uint newID = arg1; //ui->sbBoxID->value();
                if (newID == fmMain->BOX_ID )
                        return;

                fmMain->mvCallbackContext.Box_ID = fmMain->BOX_ID = newID;
                fmMain->BOX_IP = "192.168.50.2" + to_string(fmMain->BOX_ID);
                fmMain->SaveSystemParameters();

#ifndef _BOX_PLAY_CLIENT
                fmMain->SvrSktForMPCStartListening(fmMain->BOX_ID-1);
                fmMain->SvrSktForMBDStartListening(fmMain->BOX_ID-1);
#endif
                fmMain->CloseTibboSocket();
                fmMain->CloseMBDSocket();
                fmMain->VoidLoadedRctfMaps();;

                cout << "Switch to run for SG-Box-" << fmMain->BOX_ID << "\n\n" ;
                fmMain->setWindowTitle( fmMain->BasicMainWndTitle + QString::fromStdString(to_string(newID)) );
                //fmMain->LoadSystemParameters();
                fmMain->TuneRotatedPhotoDim();

                RefreshParameterGUIs(); //220928
        }
}

void Exposures::setupCameraExposure(int whichCamera, double usExpoTime)
{
        //double ms = usExpoTime / 1000;
        std::ostringstream oss;  // Create an output string stream
        oss << fixed << setprecision(1) << usExpoTime / 1000 ;  //Add double to stream

        //uint ms = round(usExpoTime / 1000);
        QString str = QString::fromStdString(oss.str()); // to_string(ms));
        switch (whichCamera)
        {
        case Camera_Left_0:  ui->edLeftCameraExpo->setText(str);            break;
        case Camera_Right_1 :  ui->edRightCameraExpo->setText(str);     break;
        }
}

int Exposures::OnCameraExpoReturnPressed(int whichCamera, QLineEdit* expoEdit)
{
        double res, ms = expoEdit->text().toDouble();
        if (ms > MAX_EXPOSURE_ms)
                ms = MAX_EXPOSURE_ms;

        if (TuneCameraExposure(whichCamera, ms * 1000, &res) == RC_OK)
        {
                setupCameraExposure(whichCamera, res);
                MainWindow  *fmMain = (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
                if (fmMain)
                {
                        switch (whichCamera)
                        {
                        case Camera_Left_0:     fmMain->CameraExposures[ui->cbMode->currentIndex()].first = ms;   break;
                        case Camera_Right_1 :  fmMain->CameraExposures[ui->cbMode->currentIndex()].second = ms;   break;
                        }
                        fmMain->SaveSystemParameters();
                }
                return RC_OK;
        } else
                return RC_FAIL_TO_READ;

}

void Exposures::on_edLeftCameraExpo_returnPressed()
{
        OnCameraExpoReturnPressed(Camera_Left_0, ui->edLeftCameraExpo);
}

void Exposures::on_edRightCameraExpo_returnPressed()
{
        OnCameraExpoReturnPressed(Camera_Right_1, ui->edRightCameraExpo);
}



void Exposures::on_btExit_clicked()
{
        close();
}

void Exposures::RefreshParameterGUIs() // RefreshCameraExposure()
{
        MainWindow  *fmMain = (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        int mi = ui->cbMode->currentIndex();

        double usExpoTime;
        for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
                if (GetCameraExposure(nthCamera, usExpoTime) == RC_OK)
                           setupCameraExposure(nthCamera, usExpoTime / 1000);

        if (fmMain)
        {
//*
                //TuneCameraExposure(nthCamera, fmMain->CameraExposures[ui->cbMode->currentIndex()].second * 1000);
                    //220907
                usExpoTime = fmMain->CameraExposures[mi].first * 1000;
                                    //cout << "el=" << d << endl;
                if (TuneCameraExposure(Camera_Left_0, usExpoTime) == RC_OK)
                        setupCameraExposure(Camera_Left_0, usExpoTime);

                usExpoTime =fmMain->CameraExposures[mi].second * 1000;
                if (TuneCameraExposure(Camera_Right_1, usExpoTime) == RC_OK)
                        setupCameraExposure(Camera_Right_1, usExpoTime);
/**/
                ui->kbHWTrigger->setChecked( fmMain->TriggerModes[mi] == TriggerMode_Hardware );
                on_kbHWTrigger_clicked(ui->kbHWTrigger->checkState()); //230503

                ui->kbSavePhotos->setChecked( fmMain->SaveCaptured[mi]);

                ui->kbLoadPhotos->setChecked( fmMain->LoadCaptured[mi]);

                switch(mi) //fmMain->opMode)
                {
                case OM_CALIBRATE_SQUARE_BAR:
                                    ui->edFromAngle->setText(QString::number(GvMSwingFrom4BCC));
                                    ui->edToAngle->setText(QString::number(GvMSwingTo4BCC));
                                    ui->sbGvMSteps->setValue(GvMSwingSteps4BCC);
                                    break;
                case OM_FETCH_RAIL_CROWN:
                                    ui->edFromAngle->setText(QString::number(GvMSwingFromPlate));
                                    ui->edToAngle->setText(QString::number(GvMSwingToPlate));
                                    ui->sbGvMSteps->setValue(GvMSwingStepsPlate);
                                    break;
                case OM_IDLE:
                case OM_CALIBRATE_CHECKERBOARD:
                                break;
                }

        }
        ui->kbRotate90->setChecked(areCameras90DgrRotated);
        ui->sbKBHorzGrids->setValue(KBGrids.horzBlackGrdNum /*.gridColNum*/ );
        ui->sbKBVertGrids->setValue(KBGrids.vertBlackGrdNum /* .gridRowNum*/ );
        ui->edHorzGridRange->setText(QString::number(KBGrids.mmGridArrayWidth /* .mmGridWidth*/));
        ui->edVertGridRange->setText(QString::number(KBGrids.mmGridArrayHeight /* .mmGridHeight*/));
        ui->edSquareBarWidth->setText(QString::number(SBWidth));
        ui->edSquareBarLength->setText(QString::number(SBLength / 1000));
        ui->edVHVertOffset->setText(QString::number(VHVertOffset));


}

void Exposures::on_cbMode_currentIndexChanged(int index)
{
        static uint baseh = 0;
        if (!baseh)  baseh =  ui->gbCameras->height();
        uint nh;
        ui->gbCkeckerBoard->setVisible(false);
        ui->gbSquareBar->setVisible(false);
        ui->gbRailHead->setVisible(false);
        switch(index)
        {
        case  OM_IDLE:
                    nh = round(baseh * 9.5);
                    ui->kbHWTrigger->setEnabled(true);
                    break;
        case OM_CALIBRATE_CHECKERBOARD:
                    ui->gbCkeckerBoard->setVisible(true);
                    ui->kbHWTrigger->setEnabled(false);
                    nh = round(baseh * 12.5);
                    break;
        case OM_CALIBRATE_SQUARE_BAR:
                    ui->gbSquareBar->setVisible(true);
                    ui->kbHWTrigger->setEnabled(true);  //SE use trigger
                    nh = round(baseh * 12);
                    break;
        case OM_FETCH_RAIL_CROWN:
                    ui->gbRailHead->setVisible(true);
                    ui->kbHWTrigger->setEnabled(true);
                    nh = round(baseh * 12);
                    break;
        }
        this->setMinimumSize(QSize(width(), nh));
        this->setMaximumSize(QSize(width(), nh));

        RefreshParameterGUIs(); //230319
        //on_kbLoadPhotos_clicked(willLoadFromFiles()); //230319

        MainWindow  *fmMain = (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        if (fmMain)
        {
                fmMain->mvCallbackContext.cameraExposures[Camera_Left_0] = fmMain->CameraExposures[index].first;
                fmMain->mvCallbackContext.cameraExposures[Camera_Right_1] = fmMain->CameraExposures[index].second;
        }
}

void Exposures::on_kbRotate90_stateChanged(int arg1)
{
//        OnRotate90CheckBoxChanged(arg1 == Qt::Checked);
}

void Exposures::on_kbRotate90_clicked(bool checked)
{
        OnRotate90CheckBoxChanged(checked);
}

int Exposures::OnRotate90CheckBoxChanged(bool newState)
{
        areCameras90DgrRotated = newState; //arg1 == Qt::Checked;
        if (areCameras90DgrRotated)
                cout << "The photos will be rotated by 90 degree.\n\n";
        else
                    cout << "The photos will not be rotated.\n\n";

        uint swapi = ui->sbKBHorzGrids->value();
        ui->sbKBHorzGrids->setValue(ui->sbKBVertGrids->value());
        ui->sbKBVertGrids->setValue(swapi );
        OnKBHorzGridsSBChanged(),     OnKBVertGridsSBChanged();

        QString swaps = ui->edHorzGridRange->text();
        ui->edHorzGridRange->setText(ui->edVertGridRange->text());
        ui->edVertGridRange->setText(swaps );
        on_edHorzGridRange_returnPressed(), on_edVertGridRange_returnPressed();

        MainWindow  *fmMain = (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        if (fmMain)
                fmMain->TuneRotatedPhotoDim();
        return RC_OK;
}

void Exposures::OnKBHorzGridsSBChanged()
{
        KBGrids.horzBlackGrdNum = ui->sbKBHorzGrids->value();
        KBGrids.horzTotalGrdNum /* .gridColNum*/ = KBGrids.horzBlackGrdNum * 2 - 2; ///*CB_INT_CNR_COL
}
void Exposures::on_sbKBHorzGrids_textChanged(const QString &arg1)
{
        OnKBHorzGridsSBChanged();         //KBGrids.gridColNum = ui->sbKBHorzGrids->value() * 2 - 2; ///*CB_INT_CNR_COL
        //CB_INT_CNR_COL = ui->sbKBHorzGrids->value() * 2 - 2;
        //if (KBGrids.gridColNum && KBGrids.gridRowNum) CB_INT_CNRS = Size(KBGrids.gridColNum, KBGrids.gridRowNum);
}

void Exposures::OnKBVertGridsSBChanged()
{
        KBGrids.vertBlackGrdNum =  ui->sbKBVertGrids->value() ;
        KBGrids.vertTotalGrdNum /*.gridRowNum*/ = KBGrids.vertBlackGrdNum * 2 - 2;  //CB_INT_CNR_ROW
}

void Exposures::on_sbKBVertGrids_textChanged(const QString &arg1)
{
        OnKBVertGridsSBChanged();
}

void Exposures::on_edHorzGridRange_returnPressed()
{
        KBGrids.mmGridArrayWidth = ui->edHorzGridRange->text().toDouble();
        KBGrids.mmOneGridWidth = KBGrids.mmGridArrayWidth / (KBGrids.horzTotalGrdNum/* gridColNum */ + 1); //GRID_DIM_X
        MainWindow  *fmMain = (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        fmMain->SaveSystemParameters();
}

void Exposures::on_edVertGridRange_returnPressed()
{
        KBGrids.mmGridArrayHeight = ui->edVertGridRange->text().toDouble();
        KBGrids.mmOneGridHeight = KBGrids.mmGridArrayHeight / (KBGrids.vertTotalGrdNum /*.gridRowNum*/ + 1); //GRID_DIM_Y
        MainWindow  *fmMain = (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        fmMain->SaveSystemParameters();
}

void Exposures::SetInitializedGUIToVariables(bool setRotationLast)  //RefreshParameterGUIs
{
        if (!setRotationLast)
            on_kbRotate90_stateChanged(ui->kbRotate90->checkState());

        OnKBHorzGridsSBChanged(); //KBGrids.gridColNum = ui->sbKBHorzGrids->value() * 2 - 2; ///*CB_INT_CNR_COL
        OnKBVertGridsSBChanged();  //KBGrids.gridRowNum = ui->sbKBVertGrids->value() * 2 - 2;  //CB_INT_CNR_ROW
        if (!KBGrids.horzTotalGrdNum /*.gridColNum*/ || !KBGrids.vertTotalGrdNum /*.gridRowNum*/ )
                return;

        //CB_INT_CNRS = Size(KBGrids.gridColNum, KBGrids.gridRowNum);

        on_edHorzGridRange_returnPressed(); //KBGrids.mmGridWidth = (ui->edHorzGridRange->text()).toDouble() / KBGrids.gridColNum; //GRID_DIM_X
        on_edVertGridRange_returnPressed(); //KBGrids.mmGridHeight = (ui->edVertGridRange->text()).toDouble() / KBGrids.gridRowNum; //GRID_DIM_Y

        if (setRotationLast)
            on_kbRotate90_stateChanged(ui->kbRotate90->checkState());

        on_edSquareBarWidth_returnPressed();
        on_edVHVertOffset_returnPressed();
}

void Exposures::on_edSquareBarWidth_returnPressed()
{
        SBWidth = (ui->edSquareBarWidth->text()).toDouble() ;
        MainWindow  *fmMain = (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        fmMain->SaveSystemParameters();
}

void Exposures::on_edSquareBarLength_returnPressed()
{
        SBLength = (ui->edSquareBarLength->text()).toDouble() * 1000;
        MainWindow  *fmMain = (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        fmMain->SaveSystemParameters();
}

void Exposures::on_edVHVertOffset_returnPressed()
{
        VHVertOffset = (ui->edVHVertOffset->text()).toDouble() ;
        MainWindow  *fmMain = (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        fmMain->SaveSystemParameters();
}

bool Exposures::IsHardwareTriggered()
{
        return ui->kbHWTrigger->checkState() == Qt::Checked;
}

bool Exposures::willLoadFromFiles()
{
        bool result = ui->kbLoadPhotos->checkState() == Qt::Checked;
        return result;
}

bool Exposures::IsSavingCapturedPhoto()
{
        return ui->kbSavePhotos->checkState() == Qt::Checked;
}

void Exposures::on_kbHWTrigger_clicked(bool checked)
{
        const bool isAlreadyOn = checked && (ui->kbHWTrigger->checkState() == Qt::Checked);
        const bool isAlreadyOff = !checked && (ui->kbHWTrigger->checkState() == Qt::Unchecked);
        /*if (    (checked && (ui->kbHWTrigger->checkState() == Qt::Checked))
             || (!checked && (ui->kbHWTrigger->checkState() == Qt::Unchecked)) )
                return; */

        if  (checked)
        {
                if (!isAlreadyOn)
                    cout << "The Tibbo-H/W Triggering is just turned ON.\n\n";
                ui->kbLoadPhotos->setCheckState(Qt::Unchecked);
        } else
                if (!isAlreadyOff)
                    cout << "The H/W Triggering is turned OFF.\n\n";

        MainWindow  *fmMain =  (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        switch(ui->cbMode->currentIndex())
        {
        case  OM_IDLE:
                /*if  (checked)
                {
                        if (!IsSavingCapturedPhoto())
                        {
                            cout << "In Idle Mode, H/W Triggering serves only for photo-saving purposes. Turn on Save Photos option first.\n\n";
                            ui->kbHWTrigger->setCheckState(Qt::Unchecked);
                            return;
                        }
                }*/
        case OM_CALIBRATE_SQUARE_BAR:
                    if (fmMain)
                            if (fmMain->IsAutoSEing())
                                for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
                                        SetCameraTriggerMode(nthCamera, TriggerMode_Software);
                        break;  //230528
        case OM_FETCH_RAIL_CROWN:
                if  (checked)
                {
                        for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
                                SetCameraTriggerMode(nthCamera, TriggerMode_Hardware);
                        if (!isAlreadyOn)
                                qDebug() << "During live triggering, photos are too fast to be displayed. \n";

                        if (fmMain)
                        {
                                //@@@HWT
    #if (!HWT_CP_THD) //230510   || (DIRECT_QUEUE)   IS134
                                fmMain->StartHWLiveShootTimer(5) ;// so slow for single thread?! 0);
    #endif
                                fmMain->eTriggerMode = TriggerMode_Hardware;
                        }
                } else {
                                //qDebug() << "Turn off HWT. \n";
                        for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
                                SetCameraTriggerMode(nthCamera, TriggerMode_Software );

                        if (fmMain)
                        {
                                fmMain->StopHWLiveShootTimer();
                                fmMain->eTriggerMode = TriggerMode_Software;
                        }
                }
                    break;
        //case OM_CALIBRATE_CHECKERBOARD:   disalbed
        }
        if (fmMain)
        {
                fmMain->TriggerModes[ui->cbMode->currentIndex()] =
                                (checked) ? TriggerMode_Hardware : TriggerMode_Software;
                fmMain->SaveSystemParameters();
        }
}


void Exposures::on_kbSavePhotos_clicked(bool checked)
{
        MainWindow  *fmMain =  (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        if (fmMain)
        {
                if  (checked)
                        if  (ui->kbLoadPhotos->checkState() == Qt::Checked)
                        {
                                if (!fmMain->NO_GUI_MODE && !fmMain->IsAutoSEing())
                                        QMessageBox::warning(this, "Aborted", "No point to save loaded files.");
                                ui->kbSavePhotos->setCheckState(Qt::Unchecked);
                                return;
                        }

                fmMain->SaveCaptured[ui->cbMode->currentIndex()] = checked ;
                fmMain->SaveSystemParameters();

                if (checked)
                {
                        cout << "The photo-saving mode is turned ON.\n\n";

                        //CAPTURE_PATH = fmMain->WorkingDir + CAPTURE_SUBDIR + "/";
                        /*moved QString strDir = QString::fromStdString(CAPTURE_PATH); //"./" + CAPTURE_SUBDIR);
                        if (!QDir(strDir).exists())
                                QDir().mkdir(strDir); */

                        on_kbHWTrigger_clicked(ui->kbHWTrigger->checkState()); //221009
                } else
                        cout << "The photo-saving mode is turned OFF.\n\n";
        }
}

void Exposures::on_kbToggleTibboStartStop_stateChanged(int arg1)
{
        MainWindow  *fmMain = (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        if (!fmMain)  return;

        if  (ui->kbToggleTibboStartStop->checkState() == Qt::Checked)
                fmMain->tibboStartReceived(0);
        else
                fmMain->tibboStopReceived(0);
}


void Exposures::on_kbLoadPhotos_clicked(bool checked)
{
        MainWindow  *fmMain =  (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        if  (checked)
        {
                if  (ui->kbHWTrigger->checkState() == Qt::Checked)
                {
                        if (fmMain)
                                if (!fmMain->NO_GUI_MODE)
                                        QMessageBox::information(this, "Aborted", "Cannot load from files during hardware-triggered.");
                        ui->kbLoadPhotos->setCheckState(Qt::Unchecked);
                        return;
                }
                if (fmMain)
                        fmMain->eTriggerMode = TriggerMode_Files;

                if  (ui->kbSavePhotos->checkState() == Qt::Checked)
                        ui->kbSavePhotos->setCheckState(Qt::Unchecked);

                qDebug() << "Load photos from files.\n\n";
        } else {
                if (fmMain)
                        if  (ui->kbHWTrigger->checkState() == Qt::Checked)
                                fmMain->eTriggerMode = TriggerMode_Hardware;
                        else
                                fmMain->eTriggerMode = TriggerMode_Software;
                qDebug() << "Capture photos from cameras.\n\n";
        }
        if (fmMain)
        {
                fmMain->LoadCaptured[ui->cbMode->currentIndex()] = checked ;
                fmMain->SaveSystemParameters();
        }
    }


//===================================================

void Exposures::on_btExit_pressed() { }
void Exposures::on_kbHWTrigger_clicked() {}
void Exposures::on_edLeftCameraExpo_cursorPositionChanged(int arg1, int arg2) {}


void Exposures::on_kbLoadPhotos_stateChanged(int arg1)
{
    return;
        /*MainWindow  *fmMain =  (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        if  (ui->kbLoadPhotos->checkState() == Qt::Checked)
        {
                if  (ui->kbHWTrigger->checkState() == Qt::Checked)
                {
                        QMessageBox::information(this, "Aborted", "Cannot load from files during hardware-triggered.");
                        ui->kbLoadPhotos->setCheckState(Qt::Unchecked);  XX
                        return;
                }
                if (fmMain)
                        fmMain->eTriggerMode = TriggerMode_Files;

X                if  (ui->kbSavePhotos->checkState() == Qt::Checked)
                        ui->kbSavePhotos->setCheckState(Qt::Unchecked);
        } else
                if  (ui->kbHWTrigger->checkState() == Qt::Checked)
                        fmMain->eTriggerMode = TriggerMode_Hardware;
                else
                        fmMain->eTriggerMode = TriggerMode_Software;
*/
}

void Exposures::on_kbSavePhotos_stateChanged(int arg1)
{
        exit;
/*
    if  (ui->kbSavePhotos->checkState() == Qt::Checked)
            if  (ui->kbLoadPhotos->checkState() == Qt::Checked)
            {
XX                    QMessageBox::warning(this, "Aborted", "No point to save loaded files.");
                    ui->kbSavePhotos->setCheckState(Qt::Unchecked);
                    return;
            }
    MainWindow  *fmMain =  (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
    if (fmMain)
    {
            fmMain->SaveCaptured[ui->cbMode->currentIndex()] = (arg1 == Qt::Checked) ;
            fmMain->SaveSystemParameters();
            if (arg1 == Qt::Checked)
            {
         X           CAPTURE_PATH = fmMain->WorkingDir + CAPTURE_SUBDIR + "/";
         X          QString strDir = QString::fromStdString(CAPTURE_PATH); //"./" + CAPTURE_SUBDIR);
                    if (!QDir(strDir).exists())
                            QDir().mkdir(strDir);

                    on_kbHWTrigger_clicked(ui->kbHWTrigger->checkState()); //221009
            }
    }
    */
}

void Exposures::on_kbHWTrigger_stateChanged(int arg1)
{
    return;
    /*
        if  (ui->kbHWTrigger->checkState() == Qt::Checked)
                ui->kbLoadPhotos->setCheckState(Qt::Unchecked);  XX
        MainWindow  *fmMain =  (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        switch(ui->cbMode->currentIndex())
        {
        case  OM_IDLE:
        case OM_FETCH_RAIL_CROWN:
                if  (ui->kbHWTrigger->checkState() == Qt::Checked)
                {
                        for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
                                SetCameraTriggerMode(nthCamera, TriggerMode_Hardware);
                                qDebug() << "Set HWT & start timer, too fast to display. \n";

                        if (fmMain)
                        {
                                //@@@HWT
XX #if (!HWT_CP_THD) || (DIRECT_QUEUE)
                                fmMain->StartHWLiveShootTimer(5) ;// so slow for single thread?! 0);XX
#endif
                                fmMain->eTriggerMode = TriggerMode_Hardware;
                        }
                } else {
                                qDebug() << "Turn off HWT. \n";
                        for (int nthCamera = Camera_Left_0; nthCamera <= Camera_Right_1; nthCamera++)
                                SetCameraTriggerMode(nthCamera, TriggerMode_Software );

                        if (fmMain)
                        {
                                fmMain->StopHWLiveShootTimer();
                                fmMain->eTriggerMode = TriggerMode_Software;
                        }
                }
                    break;
        //case OM_CALIBRATE_CHECKERBOARD:   disalbed
        case OM_CALIBRATE_SQUARE_BAR:   //single trigger
                    break;
        }
        if (fmMain)
        {
XX                fmMain->TriggerModes[ui->cbMode->currentIndex()] =
                            (arg1 == Qt::Checked) ? TriggerMode_Hardware : TriggerMode_Software;
                fmMain->SaveSystemParameters();
        }
        */
}






void Exposures::on_edFromAngle_returnPressed()
{
        MainWindow  *fmMain = (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        if (fmMain)
        {
                switch(fmMain->opMode)
                {
                case OM_CALIBRATE_SQUARE_BAR:
                                    GvMSwingFrom4BCC = (ui->edFromAngle->text()).toDouble() ;
                                    break;
                case OM_FETCH_RAIL_CROWN:
                                    GvMSwingFromPlate = (ui->edFromAngle->text()).toDouble() ;
                                    break;
                case OM_IDLE:
                case OM_CALIBRATE_CHECKERBOARD:
                                break;
                }
                fmMain->SaveSystemParameters();
        }
}


void Exposures::on_edToAngle_returnPressed()
{
        MainWindow  *fmMain = (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        if (fmMain)
        {
                switch(fmMain->opMode)
                {
                case OM_CALIBRATE_SQUARE_BAR:
                                    GvMSwingTo4BCC = (ui->edToAngle->text()).toDouble() ;
                                    break;
                case OM_FETCH_RAIL_CROWN:
                                    GvMSwingToPlate = (ui->edToAngle->text()).toDouble() ;
                                    break;
                case OM_IDLE:
                case OM_CALIBRATE_CHECKERBOARD:
                                break;
                }
                fmMain->SaveSystemParameters();
        }
}


void Exposures::on_sbGvMSteps_valueChanged(int arg1)
{
        MainWindow  *fmMain = (MainWindow  *) pMainForm; //dynamic_cast<MainWindow  *>(this->parent());
        if (fmMain)
        {
                switch(fmMain->opMode)
                {
                case OM_CALIBRATE_SQUARE_BAR:
                                    GvMSwingSteps4BCC = (ui->edFromAngle->text()).toDouble() ;
                                    break;
                case OM_FETCH_RAIL_CROWN:
                                    GvMSwingStepsPlate = (ui->edFromAngle->text()).toDouble() ;
                                    break;
                case OM_IDLE:
                case OM_CALIBRATE_CHECKERBOARD:
                                break;
                }
                fmMain->SaveSystemParameters();
        }

}

