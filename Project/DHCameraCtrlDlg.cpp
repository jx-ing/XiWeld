// VisionShow.cpp : implementation file
//

#include "stdafx.h"
#include ".\Project\XiGrooveRobot.h"
#include ".\Project\DHCameraCtrlDlg.h"
#include ".\Project\RobotCtrlDlg.h"
#include "XiImageShow.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CVisionShow dialog


CDHCameraCtrlDlg::CDHCameraCtrlDlg(std::vector < CUnit*> vpUnit, CWnd* pParent)
	: CDialog(CDHCameraCtrlDlg::IDD, NULL), m_vpUnit(vpUnit)
{
	m_nAcquisitionMode = 1;
	m_bChangeExposureTime = false;
	m_bOpenLaser = false;
	m_bOpenStrobe = false;
	m_nLaserImageNo = 0;
	m_bStartAcquisitionMode = false;
	m_bShowImage = false;
	m_bIsShowImageClosed = true;
	for (int i = 0; i < m_vpUnit.size(); i++)
	{
		m_vpRobotDriver.push_back(m_vpUnit[i]->GetRobotCtrl());
	}


	////初始化相机
	//m_pXiGrayscaleCV = NULL;
	//m_pXiGrayscaleCV = new XiCV_Image_Processing(GrayscaleImageWidth, GrayscaleImageHeight);
	//m_pXiLaserCV = NULL;
	//m_pXiLaserCV = new XiCV_Image_Processing(LaserImageWidth, LaserImageHeight);
	//m_pcvLaserImage = NULL;
	//m_pcvLaserImage = cvCreateImage(cvSize(LaserImageWidth, LaserImageHeight), IPL_DEPTH_8U, 1);
	//m_pcvGrayscaleImage = NULL;
	//m_pcvGrayscaleImage = cvCreateImage(cvSize(GrayscaleImageWidth, GrayscaleImageHeight), IPL_DEPTH_8U, 1);



}

CDHCameraCtrlDlg::~CDHCameraCtrlDlg()
{


	//DELETE_POINTER(m_pXiLaserCV)
	//DELETE_POINTER(m_pXiGrayscaleCV)
	//if (m_pcvLaserImage != NULL)
	//{
	//	cvReleaseImage(&m_pcvLaserImage);
	//}
	//if (m_pcvGrayscaleImage != NULL)
	//{
	//	cvReleaseImage(&m_pcvGrayscaleImage);
	//}

}

void CDHCameraCtrlDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CVisionShow)
	DDX_Text(pDX, IDC_EDIT_EXPOSURE_TIME, m_lExposureTime);
	DDX_Text(pDX, IDC_EDIT_GAIN_LEVEL, m_dGainLevel);
	DDX_Radio(pDX, IDC_RADIO_CONTINUOUS, m_nAcquisitionMode);
	//}}AFX_DATA_MAP
	DDX_Control(pDX, IDC_COMBO_CTRL_UNIT, m_cChoiceCtrlUnit);
	DDX_Control(pDX, IDC_COMBO_CAMERA, m_cChoiceCamera);
}


BEGIN_MESSAGE_MAP(CDHCameraCtrlDlg, CDialog)
	//{{AFX_MSG_MAP(CVisionShow)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_BUTTON_OPEN_CAMERA, OnBtnOpenCamera)
	ON_BN_CLICKED(IDC_BUTTON_CLOSE_CAMERA, OnBtnCloseCamera)
	ON_BN_CLICKED(IDOK, &CDHCameraCtrlDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_BUTTON_ADJUST_AXIS_POS, &CDHCameraCtrlDlg::OnBnClickedButtonAdjustAxisPos)
	ON_BN_CLICKED(IDC_RADIO_CONTINUOUS, &CDHCameraCtrlDlg::OnBnClickedRadioContinuous)
	ON_BN_CLICKED(IDC_RADIO_TRIGGER, &CDHCameraCtrlDlg::OnBnClickedRadioTrigger)
	ON_EN_CHANGE(IDC_EDIT_EXPOSURE_TIME, &CDHCameraCtrlDlg::OnEnChangeEditExposureTime)
	ON_EN_CHANGE(IDC_EDIT_GAIN_LEVEL, &CDHCameraCtrlDlg::OnEnChangeEditGainLevel)
	ON_BN_CLICKED(IDC_BUTTON_SAVE_IMAGE, &CDHCameraCtrlDlg::OnBnClickedButtonSaveImage)	
	ON_BN_CLICKED(IDC_BUTTON_TRIGGER, &CDHCameraCtrlDlg::OnBnClickedButtonTrigger)
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_CHECK_LASER, &CDHCameraCtrlDlg::OnBnClickedCheckLaser)
	ON_BN_CLICKED(IDC_CHECK_STROBE, &CDHCameraCtrlDlg::OnBnClickedCheckStrobe)
	ON_WM_MOUSEMOVE()
	ON_CBN_SELCHANGE(IDC_COMBO_CTRL_UNIT, &CDHCameraCtrlDlg::OnCbnSelchangeComboCtrlUnit)
	ON_CBN_SELCHANGE(IDC_COMBO_CAMERA, &CDHCameraCtrlDlg::OnCbnSelchangeComboCamera)
END_MESSAGE_MAP()


/////////////////////////////////////////////////////////////////////////////
// CVisionShow message handlers

BOOL CDHCameraCtrlDlg::OnInitDialog() 
{
	CDialog::OnInitDialog();
	
	// TODO: Add extra initialization here
	//GetDlgItem(IDC_BUTTON_SAVE_IMAGE)->EnableWindow(FALSE);
	//GetDlgItem(IDC_BUTTON_CLOSE_CAMERA)->EnableWindow(FALSE);

	GetDlgItem(IDC_STATIC_IMAGE_FRAME)->GetWindowRect(&m_rectItm);
	ScreenToClient(&m_rectItm);

	

	for (size_t i = 0; i < m_vpUnit.size(); i++)
	{
		if (!m_vpUnit[i]->m_vtCameraPara.empty())
		{
			//已修改
			CString str1 = m_vpUnit[i]->m_tContralUnit.strChineseName.GetBuffer();
			str1 = XUI::Languge::GetInstance().translate(str1.GetBuffer());

			m_cChoiceCtrlUnit.AddString(GetStr("%d:%s", i, m_vpUnit[i]->m_tContralUnit.strChineseName.GetBuffer()));
		}
	}
	m_cChoiceCtrlUnit.SetCurSel(0);
	for (size_t i = 0; i < m_vpUnit[0]->m_vtCameraPara.size(); i++)
	{
		//已修改
		CString str = m_vpUnit[0]->m_vtCameraPara[i].strCameraName.GetBuffer();
		str = XUI::Languge::GetInstance().translate(str.GetBuffer());

		m_cChoiceCamera.AddString(GetStr("%s", m_vpUnit[0]->m_vtCameraPara[i].strCameraName.GetBuffer()));
	}
	m_cChoiceCamera.SetCurSel(0);

	switch (m_vpUnit[0]->m_vtCameraPara[0].eCameraType)
	{
	case E_DH_CAMERA:
		CalImgRect(m_rectShowWholeImage, m_vpUnit[0]->m_vtCameraPara[0].tDHCameraDriverPara.nRoiWidth,
			m_vpUnit[0]->m_vtCameraPara[0].tDHCameraDriverPara.nRoiHeight, IDC_STATIC_IMAGE_FRAME, this);
		m_lExposureTime = m_vpUnit[0]->m_vtCameraPara[0].tDHCameraDriverPara.dExposureTime;
		m_dGainLevel = m_vpUnit[0]->m_vtCameraPara[0].tDHCameraDriverPara.dGainLevel;
		break;
	case E_KINECT_CAMERA:
		CalImgRect(m_rectShowWholeImage, m_vpUnit[0]->m_vtCameraPara[0].tPANOCameraDriverPara.nDepthWidth,
			m_vpUnit[0]->m_vtCameraPara[0].tPANOCameraDriverPara.nDepthHeight, IDC_STATIC_IMAGE_FRAME, this);
		m_lExposureTime = 0;
		m_dGainLevel = 0;
		break;
	case E_MECHEYE_CAMERA:
		m_lExposureTime = 0;
		m_dGainLevel = 0;
		break;
	default:
		m_lExposureTime = 0;
		m_dGainLevel = 0;
		break;
	}

	OpenCameraChangeBox(FALSE);
	UpdateData(FALSE);
	//已修改
	XUI::Languge::GetInstance().translateDialog(this);
	return TRUE;  // return TRUE unless you set the focus to a control
	              // EXCEPTION: OCX Property Pages should return FALSE
}

bool CDHCameraCtrlDlg::ShowImage()
{
	//连续采图
	if (m_bStartAcquisitionMode)
	{
		int nUnitNo = m_cChoiceCtrlUnit.GetCurSel();
		int nCameraNo = m_cChoiceCamera.GetCurSel();
		int nCurrentSameTypeNo = m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].nSameTypeNo;
		cv::Mat mData;
		IplImage* limage = NULL;
		switch (m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].eCameraType)
		{
		case E_DH_CAMERA:
			m_vpUnit[m_cChoiceCtrlUnit.GetCurSel()]->m_vpImageCapture[nCurrentSameTypeNo]->CaptureImage(limage, 1);
			DrawImage(limage, this, IDC_STATIC_IMAGE_FRAME, &m_rectShowWholeImage, true);
			break;
		case E_KINECT_CAMERA:
			if (m_vpUnit[m_cChoiceCtrlUnit.GetCurSel()]->m_vpKinectControl[nCurrentSameTypeNo]->IsAvailable())
			{
				if (m_vpUnit[m_cChoiceCtrlUnit.GetCurSel()]->m_vpKinectControl[nCurrentSameTypeNo]->GetDepthImage())
				{
					return false;
				}
				limage = &IplImage(*(m_vpUnit[m_cChoiceCtrlUnit.GetCurSel()]->m_vpKinectControl[nCurrentSameTypeNo]->m_DepthImage));
				DrawImage(limage, this, IDC_STATIC_IMAGE_FRAME, &m_rectShowWholeImage, true);
			}
			break;
		case E_MECHEYE_CAMERA:
#if ENABLE_MECH_EYE
			if (0 == m_vpUnit[m_cChoiceCtrlUnit.GetCurSel()]->m_vpMechMindDevice[nCurrentSameTypeNo]->CapturePointXYZ(mData))
			{
				limage = &IplImage(mData);
				DrawImage(limage, this, IDC_STATIC_IMAGE_FRAME, &m_rectShowWholeImage, true);
				mData.release();
			}
#endif
			break;
		default:
			break;
		}
		XI_ReleaseImage(&limage);
	}
	return true;
}

void CDHCameraCtrlDlg::SetMostControlState(BOOL bState)
{

	GetDlgItem(IDC_RADIO_CONTINUOUS)->EnableWindow(bState);
	GetDlgItem(IDC_RADIO_TRIGGER)->EnableWindow(bState);

	GetDlgItem(IDC_EDIT_EXPOSURE_TIME)->EnableWindow(bState);
	GetDlgItem(IDC_EDIT_GAIN_LEVEL)->EnableWindow(bState);


	GetDlgItem(IDC_BUTTON_OPEN_CAMERA)->EnableWindow(bState);
	GetDlgItem(IDC_BUTTON_TRIGGER)->EnableWindow(bState);
	GetDlgItem(IDC_BUTTON_SAVE_IMAGE)->EnableWindow(bState);
	GetDlgItem(IDC_BUTTON_CLOSE_CAMERA)->EnableWindow(bState);
	GetDlgItem(IDC_BUTTON_ADJUST_AXIS_POS)->EnableWindow(bState);
	GetDlgItem(IDOK)->EnableWindow(bState);
}

void CDHCameraCtrlDlg::CloseCamera()
{
	m_bShowImage = false;
	m_bStartAcquisitionMode = false;
	while (!m_bIsShowImageClosed)
	{
		Sleep(10);
		DoEvent();
	}
}

UINT CDHCameraCtrlDlg::ThreadShowImage(void *pParam)
{
	CDHCameraCtrlDlg *pcMyObj = (CDHCameraCtrlDlg *)pParam;

	if (!(pcMyObj->m_bIsShowImageClosed))
	{
		return 0;
	}

	pcMyObj->m_bIsShowImageClosed = false;
	while (pcMyObj->m_bShowImage)
	{
		if (!(pcMyObj->ShowImage()))
		{		
			pcMyObj->m_bShowImage = false;
		}
	}	
	pcMyObj->GetDlgItem(IDC_BUTTON_OPEN_CAMERA)->EnableWindow(TRUE);
	pcMyObj->RedrawWindow(&(pcMyObj->m_rectItm));
	pcMyObj->m_bIsShowImageClosed = true;
	return 0;
}

void CDHCameraCtrlDlg::ChangeExposureTime()
{
	UpdateData(TRUE);
	long lMinExposureTime = 2;
	long lMaxExposureTime = 1000000;

	if (m_lExposureTime < lMinExposureTime)
	{
		m_lExposureTime = lMinExposureTime;
	}
	else if (m_lExposureTime > lMaxExposureTime)
	{
		m_lExposureTime = lMaxExposureTime;
	}
	UpdateData(FALSE);

	int nUnitNo = m_cChoiceCtrlUnit.GetCurSel();
	int nCameraNo = m_cChoiceCamera.GetCurSel();
	switch (m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].eCameraType)
	{
	case E_DH_CAMERA:
		m_vpUnit[nUnitNo]->m_vpImageCapture[nCameraNo]->SetExposure((double)m_lExposureTime);
		break;
	case E_KINECT_CAMERA:
		break;
	case E_MECHEYE_CAMERA:
		break;
	default:
		break;
	}
}

void CDHCameraCtrlDlg::OnBtnOpenCamera() 
{
	//开相机
	int nUnitNo = m_cChoiceCtrlUnit.GetCurSel();
	int nCameraNo = m_cChoiceCamera.GetCurSel();
	//int nCurrentSameTypeNo = m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].nSameTypeNo;
	m_vpUnit[nUnitNo]->SwitchDHCamera(nCameraNo, true, true, E_ACQUISITION_MODE_SOURCE_SOFTWARE, E_CALL_BACK_MODE_OFF);
	m_vpUnit[nUnitNo]->m_vpImageCapture[nCameraNo]->StartAcquisition();

	//防止误操作
	OpenCameraChangeBox(TRUE);

	//打开相机并采图
	UpdateData(TRUE);
	if (m_nAcquisitionMode == 0)
	{
		m_bStartAcquisitionMode = true;
	}
	m_bShowImage = true;
	AfxBeginThread(ThreadShowImage, this);

}

void CDHCameraCtrlDlg::OnBtnCloseCamera() 
{
	UpdateData(TRUE);
	CloseCamera();
	OpenCameraChangeBox(FALSE);

	int nUnitNo = m_cChoiceCtrlUnit.GetCurSel();
	int nCameraNo = m_cChoiceCamera.GetCurSel();
	//int nCurrentSameTypeNo = m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].nSameTypeNo;
	m_vpUnit[nUnitNo]->SwitchDHCamera(nCameraNo, false, true, E_ACQUISITION_MODE_SOURCE_SOFTWARE, E_CALL_BACK_MODE_OFF);
}

void CDHCameraCtrlDlg::OnBnClickedOk()
{
	//关闭显示
	if (m_bIsShowImageClosed == false)
	{
		CloseCamera();
	}

	//还原增益和曝光时间
	for (size_t nUnitNo = 0; nUnitNo < m_vpUnit.size(); nUnitNo++)
	{
		if (!m_vpUnit[nUnitNo]->m_vtCameraPara.empty())
		{
			for (size_t nCameraNo = 0; nCameraNo < m_vpUnit[nUnitNo]->m_vtCameraPara.size(); nCameraNo++)
			{
				switch (m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].eCameraType)
				{
				case E_DH_CAMERA:
					m_vpUnit[nUnitNo]->m_vpImageCapture[nCameraNo]->SetExposure(-1);
					m_vpUnit[nUnitNo]->m_vpImageCapture[nCameraNo]->SetGain(-1);
					break;
				case E_KINECT_CAMERA:
					break;
				case E_MECHEYE_CAMERA:
					break;
				default:
					break;
				}
			}
		}
	}

	CDialog::OnOK();
}

void CDHCameraCtrlDlg::OnBnClickedButtonAdjustAxisPos()
{
	UpdateData(TRUE);
	CRobotCtrlDlg *pRobotCtrl = NULL;
	pRobotCtrl = new CRobotCtrlDlg(m_vpRobotDriver);
	pRobotCtrl->DoModal();
	DELETE_POINTER(pRobotCtrl);
}

void CDHCameraCtrlDlg::OnBnClickedRadioContinuous()
{
	UpdateData(TRUE);
	m_bStartAcquisitionMode = true;
}


void CDHCameraCtrlDlg::OnBnClickedRadioTrigger()
{
	UpdateData(TRUE);
	m_bStartAcquisitionMode = false;
}


void CDHCameraCtrlDlg::OnEnChangeEditExposureTime()
{
	m_bChangeExposureTime = true;
}


void CDHCameraCtrlDlg::OnEnChangeEditGainLevel()
{
	UpdateData(TRUE);
	double dMaxGainLevel;
	//if (m_bCameraType == FALSE)
	//{
	//	dMaxGainLevel = 17.0;
	//} 
	//else
	{
		dMaxGainLevel = 50;
	}

	m_dGainLevel = DecimalRound(m_dGainLevel, 1);
	if (m_dGainLevel < 0)
	{
		m_dGainLevel = 0;
	} 
	else if(m_dGainLevel > dMaxGainLevel)
	{
		m_dGainLevel = dMaxGainLevel;
	}

	UpdateData(FALSE);

	int nUnitNo = m_cChoiceCtrlUnit.GetCurSel();
	int nCameraNo = m_cChoiceCamera.GetCurSel();
	switch (m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].eCameraType)
	{
	case E_DH_CAMERA:
		m_vpUnit[nUnitNo]->m_vpImageCapture[nCameraNo]->SetGain(m_dGainLevel);
		break;
	case E_KINECT_CAMERA:
		break;
	case E_MECHEYE_CAMERA:
		break;
	default:
		break;
	}
}


void CDHCameraCtrlDlg::OnBnClickedButtonSaveImage()
{
	UpdateData(TRUE);
	int nUnitNo = m_cChoiceCtrlUnit.GetCurSel();
	int nCameraNo = m_cChoiceCamera.GetCurSel();

	cv::Mat mData;
	IplImage* limage = NULL;
	CString strLaserImagePath = LASER_IMAGES + m_vpUnit[nUnitNo]->m_tContralUnit.strUnitName + "\\";
	CheckFolder(strLaserImagePath);
	T_ROBOT_COORS tcurcoord = m_vpUnit[nUnitNo]->m_pYasakawaRobotDriver->GetCurrentPos();
	int nCurrentSameTypeNo = m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].nSameTypeNo;

	//已修改
	if (IDOK != XUI::MesBox::PopOkCancel("向Z负方向连续采图31张，点击确定只采集一张图"))
	//if (IDCANCEL == XiMessageBox("点击取消向Z正方向连续采图31张，点击确定只采集一张图。"))
	{
		int config[7] = { 0,0,0,0,0,0,0 };
		for (size_t i = 0; i < 31; i++)
		{
			m_vpUnit[nUnitNo]->m_pYasakawaRobotDriver->MoveByJob(tcurcoord, T_ROBOT_MOVE_SPEED{ 3000,100,100 }, -1, "MOVL", config);
			m_vpUnit[nUnitNo]->RobotCheckDone(1400);
			m_vpUnit[nUnitNo]->m_vpImageCapture[nCameraNo]->CaptureImage(limage, 1);
			SaveImage(limage, GetStr("%s%d.jpg", strLaserImagePath, i));
			XI_ReleaseImage(&limage);
			tcurcoord.dZ -= 2;
		}
		//已修改
		XUI::MesBox::PopOkCancel("采集完成");

		return;
	}

	switch (m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].eCameraType)
	{
	case E_DH_CAMERA:
		if (m_nLaserImageNo == 0)
		{
			DelFiles(strLaserImagePath);
		}
		m_vpUnit[nUnitNo]->m_vpImageCapture[nCurrentSameTypeNo]->CaptureImage(limage, 1);
		SaveImage(limage, GetStr("%sImage_%d.jpg", strLaserImagePath, m_nLaserImageNo));
		DrawImage(limage, this, IDC_STATIC_IMAGE_FRAME, &m_rectShowWholeImage, true);
		//cvReleaseImage(&limage);
		m_nLaserImageNo++;
		break;
	case E_KINECT_CAMERA:
		if (m_vpUnit[m_cChoiceCtrlUnit.GetCurSel()]->m_vpKinectControl[nCurrentSameTypeNo]->IsAvailable())
		{
			if (m_vpUnit[m_cChoiceCtrlUnit.GetCurSel()]->m_vpKinectControl[nCurrentSameTypeNo]->GetDepthImage())
			{
				return;
			}
			limage = &IplImage(*(m_vpUnit[m_cChoiceCtrlUnit.GetCurSel()]->m_vpKinectControl[nCurrentSameTypeNo]->m_DepthImage));
			DrawImage(limage, this, IDC_STATIC_IMAGE_FRAME, &m_rectShowWholeImage, true);
			//cvReleaseImage(&limage);
			return;
		}
		break;
	case E_MECHEYE_CAMERA:
#if ENABLE_MECH_EYE
		if (0 == m_vpUnit[m_cChoiceCtrlUnit.GetCurSel()]->m_vpMechMindDevice[nCurrentSameTypeNo]->CapturePointXYZ(mData))
		{
			//				limage = cvCreateImage(cvSize(1920, 1200), IPL_DEPTH_32F, 3);
			//				cv::imwrite("222222222.png", mData);
			limage = &IplImage(mData);
			//				SaveImage(limage, "1111111111111111.png");
			CalImgRect(m_rectShowWholeImage, limage->width, limage->height, IDC_STATIC_IMAGE_FRAME, this);
			//				XiImageShow cImgShow(limage->width, limage->height);
			//				cImgShow.ShowImage(limage, this, IDC_STATIC_IMAGE_FRAME, &m_rectShowWholeImage);
			DrawImage(limage, this, IDC_STATIC_IMAGE_FRAME, &m_rectShowWholeImage, true);
			//				mData.release();
		}
#endif
		break;
	default:
		break;
	}

	XI_ReleaseImage(&limage);
}


void CDHCameraCtrlDlg::OnBnClickedButtonTrigger()
{
	UpdateData(TRUE);
	if (m_nAcquisitionMode == 1)
	{
		int nUnitNo = m_cChoiceCtrlUnit.GetCurSel();
		int nCameraNo = m_cChoiceCamera.GetCurSel();
		int nCurrentSameTypeNo = m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].nSameTypeNo;
		cv::Mat mData;
		IplImage *limage = NULL;
		switch (m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].eCameraType)
		{
		case E_DH_CAMERA:
			m_vpUnit[m_cChoiceCtrlUnit.GetCurSel()]->m_vpImageCapture[nCurrentSameTypeNo]->CaptureImage(limage, 1);
			DrawImage(limage, this, IDC_STATIC_IMAGE_FRAME, &m_rectShowWholeImage, true);
			break;
		case E_KINECT_CAMERA:
			if (m_vpUnit[m_cChoiceCtrlUnit.GetCurSel()]->m_vpKinectControl[nCurrentSameTypeNo]->IsAvailable())
			{
				if (m_vpUnit[m_cChoiceCtrlUnit.GetCurSel()]->m_vpKinectControl[nCurrentSameTypeNo]->GetDepthImage())
				{
					return ;
				}
				limage = &IplImage(*(m_vpUnit[m_cChoiceCtrlUnit.GetCurSel()]->m_vpKinectControl[nCurrentSameTypeNo]->m_DepthImage));
				DrawImage(limage, this, IDC_STATIC_IMAGE_FRAME, &m_rectShowWholeImage, true);
				return ;
			}
			break;
		case E_MECHEYE_CAMERA:
#if ENABLE_MECH_EYE
			if (0 == m_vpUnit[m_cChoiceCtrlUnit.GetCurSel()]->m_vpMechMindDevice[nCurrentSameTypeNo]->CapturePointXYZ(mData))
			{
//				limage = cvCreateImage(cvSize(1920, 1200), IPL_DEPTH_32F, 3);
//				cv::imwrite("222222222.png", mData);
				limage = &IplImage(mData);
//				SaveImage(limage, "1111111111111111.png");
				CalImgRect(m_rectShowWholeImage, limage->width,limage->height, IDC_STATIC_IMAGE_FRAME, this);
//				XiImageShow cImgShow(limage->width, limage->height);
//				cImgShow.ShowImage(limage, this, IDC_STATIC_IMAGE_FRAME, &m_rectShowWholeImage);
				DrawImage(limage, this, IDC_STATIC_IMAGE_FRAME, &m_rectShowWholeImage, true);
//				mData.release();
			}
#endif
			break;
		default:
			break;
		}
		XI_ReleaseImage(&limage);
	}
}


void CDHCameraCtrlDlg::OnBnClickedCheckLaser()
{
	int nUnitNo = m_cChoiceCtrlUnit.GetCurSel();
	int nCamNo = m_cChoiceCamera.GetCurSel(); 
	CString sLaserIOName;
	sLaserIOName = m_vpUnit[nUnitNo]->GetCameraParam(nCamNo).tCameraIO.vsLaserIOName[0];
	//switch (nCamNo)
	//{
	//case 0: sLaserIOName = "TrackLaser"; break;
	//case 1: sLaserIOName = "MeasureLaser"; break;
	//default: sLaserIOName.Format("LineScanLaser%d", nCamNo - 1); break;
	//}

	m_vpUnit[nUnitNo]->SwitchIO(sLaserIOName, !m_bOpenLaser);
	m_bOpenLaser = !m_bOpenLaser;
}


void CDHCameraCtrlDlg::OnBnClickedCheckStrobe()
{
	AfxMessageBox("未添加");
	if (m_bOpenStrobe)
	{
//		m_pIOControl->CloseSupLEDCtrlSignalPower(m_nCurRobotNo);
		//m_vvpDHCamera[m_nCurRobotNo][1]->CloseStrobeForGray(m_vvtCameraPara[m_nCurRobotNo][1].tDHCameraDriverPara.nStrobeLine1);
		//m_vvpDHCamera[m_nCurRobotNo][1]->CloseStrobeForGray(m_vvtCameraPara[m_nCurRobotNo][1].tDHCameraDriverPara.nStrobeLine2);
		m_bOpenStrobe = false;
	} 
	else
	{
//		m_pIOControl->OpenSupLEDCtrlSignalPower(m_nCurRobotNo);
		//m_vvpDHCamera[m_nCurRobotNo][1]->OpenStrobeForGray(m_vvtCameraPara[m_nCurRobotNo][1].tDHCameraDriverPara.nStrobeLine1);
		//m_vvpDHCamera[m_nCurRobotNo][1]->OpenStrobeForGray(m_vvtCameraPara[m_nCurRobotNo][1].tDHCameraDriverPara.nStrobeLine2);
		m_bOpenStrobe = true;
	}
}


void CDHCameraCtrlDlg::OnMouseMove(UINT nFlags, CPoint point)
{
	if (m_bChangeExposureTime)
	{
		m_bChangeExposureTime = false;
		ChangeExposureTime();
	}

	CDialog::OnMouseMove(nFlags, point);
}


void CDHCameraCtrlDlg::OnCbnSelchangeComboCtrlUnit()
{
	// TODO: 在此添加控件通知处理程序代码
	if (!GetDlgItem(IDC_BUTTON_OPEN_CAMERA)->IsWindowEnabled())
	{
		UpdateData(FALSE);
	}
	int nUnitNo = m_cChoiceCtrlUnit.GetCurSel();
	m_cChoiceCamera.ResetContent();
	for (size_t i = 0; i < m_vpUnit[nUnitNo]->m_vtCameraPara.size(); i++)
	{
		//已修改
		m_vpUnit[nUnitNo]->m_vtCameraPara[i].strCameraName = XUI::Languge::GetInstance().translate(m_vpUnit[nUnitNo]->m_vtCameraPara[i].strCameraName.GetBuffer());
		
		m_cChoiceCamera.AddString(GetStr("%s", m_vpUnit[nUnitNo]->m_vtCameraPara[i].strCameraName.GetBuffer()));
	}
	m_cChoiceCamera.SetCurSel(0);
	int nCameraNo = m_cChoiceCamera.GetCurSel();

	switch (m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].eCameraType)
	{
	case E_DH_CAMERA:
		CalImgRect(m_rectShowWholeImage, m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].tDHCameraDriverPara.nRoiWidth,
			m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].tDHCameraDriverPara.nRoiHeight, IDC_STATIC_IMAGE_FRAME, this);
		m_lExposureTime = m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].tDHCameraDriverPara.dExposureTime;
		m_dGainLevel = m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].tDHCameraDriverPara.dGainLevel;
		break;
	case E_KINECT_CAMERA:
		CalImgRect(m_rectShowWholeImage, m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].tPANOCameraDriverPara.nDepthWidth,
			m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].tPANOCameraDriverPara.nDepthHeight, IDC_STATIC_IMAGE_FRAME, this);
		m_lExposureTime = 0;
		m_dGainLevel = 0;
		break;
	case E_MECHEYE_CAMERA:
		m_lExposureTime = 0;
		m_dGainLevel = 0;
		break;
	default:
		m_lExposureTime = 0;
		m_dGainLevel = 0;
		break;
	}
	UpdateData(FALSE);
}

void CDHCameraCtrlDlg::OpenCameraChangeBox(BOOL bCameraState)
{
	GetDlgItem(IDC_BUTTON_OPEN_CAMERA)->EnableWindow(!bCameraState);
	GetDlgItem(IDC_COMBO_CTRL_UNIT)->EnableWindow(!bCameraState);
	GetDlgItem(IDC_COMBO_CAMERA)->EnableWindow(!bCameraState);
	GetDlgItem(IDC_BUTTON_TRIGGER)->EnableWindow(bCameraState);
	GetDlgItem(IDC_BUTTON_SAVE_IMAGE)->EnableWindow(bCameraState);
	GetDlgItem(IDC_BUTTON_CLOSE_CAMERA)->EnableWindow(bCameraState);
}


void CDHCameraCtrlDlg::OnCbnSelchangeComboCamera()
{
	int nUnitNo = m_cChoiceCtrlUnit.GetCurSel();
	int nCameraNo = m_cChoiceCamera.GetCurSel();
	switch (m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].eCameraType)
	{
	case E_DH_CAMERA:
		CalImgRect(m_rectShowWholeImage, m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].tDHCameraDriverPara.nRoiWidth,
			m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].tDHCameraDriverPara.nRoiHeight, IDC_STATIC_IMAGE_FRAME, this);
		m_lExposureTime = m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].tDHCameraDriverPara.dExposureTime;
		m_dGainLevel = m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].tDHCameraDriverPara.dGainLevel;
		break;
	case E_KINECT_CAMERA:
		CalImgRect(m_rectShowWholeImage, m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].tPANOCameraDriverPara.nDepthWidth,
			m_vpUnit[nUnitNo]->m_vtCameraPara[nCameraNo].tPANOCameraDriverPara.nDepthHeight, IDC_STATIC_IMAGE_FRAME, this);
		m_lExposureTime = 0;
		m_dGainLevel = 0;
		break;
	case E_MECHEYE_CAMERA:
		m_lExposureTime = 0;
		m_dGainLevel = 0;
		break;
	default:
		m_lExposureTime = 0;
		m_dGainLevel = 0;
		break;
	}
	UpdateData(FALSE);
}
