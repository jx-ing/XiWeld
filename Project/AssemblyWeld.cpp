// InciseHead.cpp : implementation file

#include "stdafx.h"
#include "AssemblyWeld.h"
#include "LineScanForTeach.h"
#include "WeldDataJson.h"
#include ".\LocalFiles\ExLib\Vision\include\GFPGJointLib.h"

#ifdef _DEBUG
#define new DEBUG_NEW
//#undef THIS_FILE
//static char THIS_FILE[] = __FILE__;
#endif
// 
std::vector <T_PART_GRAPH> m_vtAdjustDxfPart; 
std::vector <T_PART_GRAPH> m_vtAdjustIdentifyPart; 
std::vector <T_PART_GRAPH> m_vtAdjustTemplatePart;
std::vector <T_PART_GRAPH> m_AdjustGraphData;
cv::Vec2d m_vecAdjustCameraFocals[7];

T_POINT_2D_DOUBLE m_tTranslate;
double  m_dRotate;

/////////////////////////////////////////////////////////////////////////////
// CAssemblyWeld dialog
CAssemblyWeld::CAssemblyWeld(CServoMotorDriver* pCtrlCardDriver/* = NULL*/, COPCClientCtrl* pOPCClientCtrl/* = NULL*/)
	: CDialog(CAssemblyWeld::IDD, NULL)
	, m_pServoMotorDriver(pCtrlCardDriver)
	, m_pOPCClientCtrl(pOPCClientCtrl)
	, m_bStartArcOrNot(FALSE)
	, m_bNaturalPop(FALSE)
	, m_bProcessPop(FALSE)
	, m_bTeachPop(FALSE)
	, m_pWeldAfterMeasure(NULL)
	, m_bAutoWeldWorking(false)
	, m_bThreadShowMoveStateEnd(false)
	, m_bIfEmg(false)
	, m_bOpenRightRobotRightCam(FALSE)
	, m_bOpenRightRobotLeftCam(FALSE)
	, m_bOpenLeftRobotRightCam(FALSE)
	, m_bOpenLeftRobotLeftCam(FALSE)
	, m_nUseModel(FALSE)
{
	if (TRUE != InitAllUnit())
	{
		XUI::MesBox::PopError("初始化控制单元失败");
	}
	WriteLog("初始化控制单元完毕！");

	//初始化画框控件ID
	m_vnDrawComponentID.push_back(IDC_STATIC_DRAW_PART_SINGLE);
	m_vnDrawComponentID.push_back(IDC_STATIC_DRAW_PART_SINGLE2);
	m_vnDrawComponentID.push_back(IDC_STATIC_DRAW_PART_SINGLE3);
	m_vnDrawComponentID.push_back(IDC_STATIC_DRAW_PART_SINGLE4);

	TrackInspectToolsLoad(".\\Local_Files\\ExtLib\\Vision\\ModelPath", 3, 33);
	
	//焊接信息初始化
	//m_StatisticalData = CStatisticalData::getInstance();
	m_vpUnit[0]->SwitchIO("CoolGas", true);
}

CAssemblyWeld::~CAssemblyWeld()
{	
	std::vector<CRobotDriverAdaptor*>::iterator Iter;
	for (Iter = m_vpRobotDriver.begin(); Iter != m_vpRobotDriver.end(); Iter++)
	{
		CRobotDriverAdaptor* temp = *Iter;
		DELETE_POINTER(temp);
	}
	m_vpRobotDriver.clear();

	for (auto& _pointer : m_vpScanInit) {
		delete _pointer;
		_pointer = NULL;
	}

	int nIdx = 0;

	for (int i = 0; i < m_vpShowLaserImgBuff.size(); i++)
	{
		if (NULL != m_vpShowLaserImgBuff[i])
		{
			cvReleaseImage(&m_vpShowLaserImgBuff[i]);
		}
	}
	m_pServoMotorDriver = NULL;
	m_pOPCClientCtrl = NULL;
}

BEGIN_MESSAGE_MAP(CAssemblyWeld, CDialog)
    //{{AFX_MSG_MAP(CAssemblyWeld)
    ON_WM_PAINT()
    //ON_WM_TIMER()
    ON_WM_CTLCOLOR()
    ON_BN_CLICKED(IDC_BUTTON_PAUSE_CONTINUE, OnBtnPauseContinue)
    ON_BN_CLICKED(IDC_BUTTON_CLOSE, OnBtnClose)
    ON_BN_CLICKED(IDC_BUTTON_SIMULATE_INCISE, OnBnClickedButtonSimulateIncise)
    ON_BN_CLICKED(IDC_BUTTON_SYSTEM_PARA, OnBtnSystemPara)
    ON_BN_CLICKED(IDC_BUTTON_SPARE4, OnBnClickedButtonSpare4)
    ON_BN_CLICKED(IDC_BUTTON_VISION_SHOW, OnBtnVisionShow)
    ON_WM_QUERYDRAGICON()
    ON_WM_SHOWWINDOW()
    ON_WM_GETMINMAXINFO()
    ON_WM_SYSCOMMAND()
    //}}AFX_MSG_MAP
    ON_BN_CLICKED(IDC_BUTTON_LOAD_TRACK, &CAssemblyWeld::OnBtnLoadTrack)
	ON_BN_CLICKED(IDC_BUTTON_PANO_RECOGNITION, &CAssemblyWeld::OnBnClickedButtonPanoRecognition)
	ON_BN_CLICKED(IDC_BUTTON_ROBOT_CTRL, &CAssemblyWeld::OnBnClickedButtonRobotCtrl)
	ON_BN_CLICKED(IDC_BUTTON_ANTIFEEDING, &CAssemblyWeld::OnBnClickedButtonAntifeeding)
	ON_BN_CLICKED(IDC_BUTTON_START, &CAssemblyWeld::OnBnClickedButtonStart)
	ON_WM_LBUTTONDOWN()
	ON_BN_CLICKED(IDC_BUTTON_PLASMA_COM, &CAssemblyWeld::OnBnClickedButtonPlasmaCom)
	ON_BN_CLICKED(IDC_BUTTON_ADJUST_RECOG, &CAssemblyWeld::OnBnClickedButtonAdjustRecog)
	ON_BN_CLICKED(IDC_BUTTON_BACK_HOME, &CAssemblyWeld::OnBnClickedButtonBackHome)
	ON_BN_CLICKED(IDC_RADIO_NOARC, &CAssemblyWeld::OnBnClickedRadioNoarc)
	ON_BN_CLICKED(IDC_RADIO_ARC, &CAssemblyWeld::OnBnClickedRadioArc)
	ON_WM_MOUSEWHEEL()
	ON_WM_RBUTTONDOWN()
	ON_WM_RBUTTONUP()
	ON_BN_CLICKED(IDC_BUTTON_COMMONLY_USED_IO, &CAssemblyWeld::OnBnClickedButtonCommonlyUsedIO)
	ON_BN_CLICKED(IDC_BUTTON_SPARE3, &CAssemblyWeld::OnBnClickedButtonSpare3)
	ON_BN_CLICKED(IDC_BUTTON_SPARE1, &CAssemblyWeld::OnBnClickedButtonSpare1)
	ON_BN_CLICKED(IDC_CHECK_NATURAL_POP, &CAssemblyWeld::OnBnClickedCheckNaturalPop)
	ON_BN_CLICKED(IDC_CHECK_PROCESS_POP, &CAssemblyWeld::OnBnClickedCheckProcessPop)
	ON_BN_CLICKED(IDC_BUTTON_ADVANCED_CONTINUE, &CAssemblyWeld::OnBnClickedButtonAdvancedContinue)
	ON_BN_CLICKED(IDC_BUTTON_TABLE_PARA, &CAssemblyWeld::OnBnClickedButtonTablePara)
	ON_BN_CLICKED(IDC_CHECK_GRAY, &CAssemblyWeld::OnBnClickedCheckGray)
	ON_BN_CLICKED(IDC_CHECK_TEACH_POP, &CAssemblyWeld::OnBnClickedCheckTeachPop)
	ON_BN_CLICKED(IDC_BUTTON_SYSTEM_PARA2, &CAssemblyWeld::OnBnClickedButtonSystemPara2)
	ON_CBN_SELCHANGE(IDC_COMBO_TABLE_GROUP2, &CAssemblyWeld::OnCbnSelchangeComboTableGroup2)
	ON_BN_CLICKED(IDC_BUTTON_TABLE_PARA2, &CAssemblyWeld::OnBnClickedButtonTablePara2)
	ON_BN_CLICKED(IDC_BUTTON_PANO_RECOGNITION2, &CAssemblyWeld::OnBnClickedButtonPanoRecognition2)
	ON_BN_CLICKED(IDC_BUTTON_TABLE_PARA3, &CAssemblyWeld::OnBnClickedButtonTablePara3)
	ON_BN_CLICKED(IDC_BUTTON_TABLE_PARA4, &CAssemblyWeld::OnBnClickedButtonTablePara4)
	ON_BN_CLICKED(IDC_BUTTON_LOAD_TRACK2, &CAssemblyWeld::OnBnClickedButtonLoadTrack2)
	ON_CBN_SELCHANGE(IDC_WORKPIECE_TYPE, &CAssemblyWeld::OnCbnSelchangeWorkpieceType)
	//ON_BN_CLICKED(IDC_CHECK_GRAY3, &CAssemblyWeld::OnBnClickedCheckGray3)
	ON_BN_CLICKED(IDC_CHECK_GRAY2, &CAssemblyWeld::OnBnClickedCheckGray2)
	ON_BN_CLICKED(IDC_CHECK3, &CAssemblyWeld::OnBnClickedCheck3)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_BUTTON2, &CAssemblyWeld::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &CAssemblyWeld::OnBnClickedButton3)
	ON_WM_PAINT()
	ON_WM_DRAWITEM()
	ON_CBN_SELCHANGE(IDC_WORKPIECE_TYPE2, &CAssemblyWeld::OnCbnSelchangeWorkpieceType2)
	ON_BN_CLICKED(IDC_CHECK_MODEL, &CAssemblyWeld::OnBnClickedCheckModel)
END_MESSAGE_MAP()

// <start> ******************************* 界面相关函数 ******************************* <start> //
BOOL CAssemblyWeld::OnInitDialog()
{
	CDialog::OnInitDialog();

	if (!g_bLocalDebugMark)
	{
		CFullScreen cFullScreen(this);
		cFullScreen.Init();
	}

	m_bkBrush.CreateSolidBrush(RGB(15, 60, 170));   //(150, 255, 212)  (173, 215, 255)  (15, 60, 170)
	m_Brush.CreateSolidBrush(RGB(173, 215, 255));
	m_cFontCorporation.CreateFont(20, 0, 0, 0, FW_NORMAL, TRUE, TRUE, 0, ANSI_CHARSET, OUT_DEFAULT_PRECIS,
		CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY, DEFAULT_PITCH & FF_SWISS, "宋体");

	m_static.SubclassDlgItem(IDC_STATIC_OPERATE_HINT, this);
	m_cFont.CreatePointFont(200, _T("宋体"));
	m_static.SetFont(&m_cFont);

	VerticalCenter(m_cCoorX);
	VerticalCenter(m_cCoorY);
	VerticalCenter(m_cCoorZ);
	VerticalCenter(m_cCoorRX);
	VerticalCenter(m_cCoorRY);
	VerticalCenter(m_cCoorRZ);	

	initWorkpieceType();

	LoadOptionalFunctionPara();
	LoadDebugPara();
	InitVariable();
	LoadTableGroupScan();
	LoadPartType();
	MainFromArrangement();
	AfxBeginThread(ThreadShowMoveState, this);

	for (int nUnitNum = 0; nUnitNum < m_vpUnit.size(); nUnitNum++) {
		m_vpScanInit[nUnitNum]->m_pShowImage = m_vpShowLaserImgBuff[nUnitNum];
	}

	SetTimer(2, 200, NULL); // 显示图片
	//已修改
	XUI::Languge::GetInstance().translateDialog(this);
	WriteLog("初始化系统完毕！");
	return TRUE;
}

VOID CAssemblyWeld::MainFromArrangement()
{

}

BOOL CAssemblyWeld::PreTranslateMessage(MSG* pMsg)
{
	// TODO: 在此添加专用代码和/或调用基类
	//鼠标停留 提示信息
	//m_tooltip.RelayEvent(pMsg);

	int nUnitNo = 0;
	int nDriverNo = 0;
	if (m_vpUnit[nUnitNo]->GetRobotCtrl()->m_pvpMotorDriver->size() <= 0)
	{
		return CDialog::PreTranslateMessage(pMsg);
	}
	CUnitDriver* pUnitDriver = m_vpUnit[nUnitNo]->GetRobotCtrl()->m_pvpMotorDriver->at(nDriverNo);
	bool bServoRdy = false;
	pUnitDriver->GetSevonRdy(bServoRdy);
	double dMaxSpeed = pUnitDriver->GetMaxSpeed(nDriverNo);

	bool bWorking = (true == m_bAutoWeldWorking) || ((NULL != m_pWeldAfterMeasure) && (true == m_pWeldAfterMeasure->IsWorking()));
	if (true == bWorking)
	{
		//WriteLog("工作中禁止使用按键移动外部轴!");
		if (pMsg->message == WM_KEYDOWN || pMsg->message == WM_KEYUP)
		{
			return TRUE;
		}
		return CDialog::PreTranslateMessage(pMsg);
	}

	if (pMsg->message == WM_KEYDOWN)
	{
		if ((pMsg->wParam == VK_RETURN) || (pMsg->wParam == VK_ESCAPE))
		{
			return TRUE;
		}
		else if (pMsg->wParam == VK_SPACE)
		{
			//WriteLog("触发空格暂停");
			//OnBtnStop();
			//CloseCameraLaserAssembly();
			return TRUE;
		}
		else if (pMsg->wParam == VK_LEFT)
		{
			if (!bServoRdy)
			{
				//已修改
				SetHintInfo(XUI::Languge::GetInstance().translate("外部轴未就绪!"));//"回安全位置失败！"
				//SetHintInfo("外部轴未就绪!");
				return TRUE;
			}
			pUnitDriver->ContiMove(1, /*3000.0*/dMaxSpeed / 60.0, 0.8);
			//m_cMoveCtrl.ContiMoveDis(AXIS_X, 0, 0, m_cMoveCtrl.m_dNormalVelX, 0.5, 0.5);
			return TRUE;
		}
		else if (pMsg->wParam == VK_RIGHT)
		{
			if (!bServoRdy)
			{
				//已修改
				SetHintInfo(XUI::Languge::GetInstance().translate("外部轴未就绪!"));//"回安全位置失败！"
				//SetHintInfo("外部轴未就绪!");
				return TRUE;
			}
			pUnitDriver->ContiMove(0, /*3000.0*/dMaxSpeed / 60.0, 0.8);
			//m_cMoveCtrl.ContiMoveDis(AXIS_X, 1, 0, m_cMoveCtrl.m_dNormalVelX, 0.5, 0.5);
			return TRUE;
		}
		//else if (pMsg->wParam == VK_UP)
		//{
		//	if (!bServoRdy)
		//	{
		//		SetHintInfo("外部轴未就绪!");
		//		return TRUE;
		//	}
		//	m_cMoveCtrl.ContiMoveDis(AXIS_Y, 1, 0, m_cMoveCtrl.m_dNormalVelX, 0.5, 0.5);
		//	return TRUE;
		//}
		//else if (pMsg->wParam == VK_DOWN)
		//{
		//	if (!bServoRdy)
		//	{
		//		SetHintInfo("外部轴未就绪!");
		//		return TRUE;
		//	}
		//	m_cMoveCtrl.ContiMoveDis(AXIS_Y, 0, 0, m_cMoveCtrl.m_dNormalVelX, 0.5, 0.5);
		//	return TRUE;
		//}
		return TRUE;
	}
	if (pMsg->message == WM_KEYUP)
	{
		if (pMsg->wParam == VK_LEFT || pMsg->wParam == VK_RIGHT)
		{

			pUnitDriver->DecelStop();
			//m_cMoveCtrl.DecelStop(AXIS_X, 0.5, 0);
			return TRUE;
		}
		//else if (pMsg->wParam == VK_UP || pMsg->wParam == VK_DOWN)
		//{
		//	m_cMoveCtrl.DecelStop(AXIS_Y, 0.5, 0);
		//	return TRUE;
		//}
		//return TRUE;
	}
	return CDialog::PreTranslateMessage(pMsg);
}

void CAssemblyWeld::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
		//if (nIDEvent == 1)
	//{
	//	if (false == m_cMoveCtrl.ReadInbitDis(0) && false == m_bIfEmg)
	//	{
	//		Sleep(100);
	//		if (false == m_cMoveCtrl.ReadInbitDis(0))
	//		{
	//			m_bIfEmg = true;
	//			m_vpRobotDriver[0]->ServoOff();
	//			//m_vpRobotDriver[1]->ServoOff();
	//			WriteLog("触发急停");
	//		}
	//	}
	//	else if (TRUE == m_cMoveCtrl.ReadInbitDis(0) && true == m_bIfEmg)
	//	{
	//		m_bIfEmg = false;
	//		m_vpRobotDriver[0]->HoldOff();
	//		//m_vpRobotDriver[1]->HoldOff();
	//	}
	//}
	if (nIDEvent == 2) // 显示图像
	{
		ShowTeachImage();
	}
	if (nIDEvent == 99)
	{
		CString sHintInfo = "";
		for (int i = 0; i < m_vpUnit.size(); i++)
		{
			sHintInfo.Format("%s%s:%s            ", sHintInfo, m_vpUnit[i]->m_tContralUnit.strUnitName, m_vpUnit[i]->m_sHintInfo);
		}
		SetHintInfo(sHintInfo);
	}
	CDialog::OnTimer(nIDEvent);
}

HBRUSH CAssemblyWeld::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor)
{
	HBRUSH hbr = CDialog::OnCtlColor(pDC, pWnd, nCtlColor);

	if (pWnd->GetDlgCtrlID() == IDC_RADIO_ARC || pWnd->GetDlgCtrlID() == IDC_RADIO_NOARC
		|| pWnd->GetDlgCtrlID() == IDC_CHECK_GRAY4 || pWnd->GetDlgCtrlID() == IDC_CHECK_GRAY
		|| pWnd->GetDlgCtrlID() == IDC_CHECK_NATURAL_POP || pWnd->GetDlgCtrlID() == IDC_CHECK_TEACH_POP
		|| pWnd->GetDlgCtrlID() == IDC_CHECK_GRAY2 || pWnd->GetDlgCtrlID() == IDC_CHECK3
		|| pWnd->GetDlgCtrlID() == IDC_CHECK_PROCESS_POP)
	{
		pDC->SetBkMode(TRANSPARENT);
		pDC->SetTextColor(RGB(0, 0, 0)); //字体颜色
		return   m_Brush;  
	}

	if ((pWnd->GetDlgCtrlID() == IDC_STATIC_CORPORATION)
		|| (pWnd->GetDlgCtrlID() == IDC_STATIC_SOFTWARE_NAME))
	{
		pDC->SetBkMode(TRANSPARENT);
		pDC->SelectObject(&m_cFontCorporation);
		pDC->SetTextColor(RGB(155, 0, 155));
		return   m_bkBrush;
	}

	if (pWnd->GetDlgCtrlID() == IDC_STATIC_1007 || pWnd->GetDlgCtrlID() == IDC_STATIC_1009
		|| pWnd->GetDlgCtrlID() == IDC_STATIC_1010 || pWnd->GetDlgCtrlID() == IDC_STATIC_1011
		|| pWnd->GetDlgCtrlID() == IDC_STATIC_1012 || pWnd->GetDlgCtrlID() == IDC_STATIC_1013
		|| pWnd->GetDlgCtrlID() == IDC_STATIC_1014 || pWnd->GetDlgCtrlID() == IDC_STATIC_1015
		|| pWnd->GetDlgCtrlID() == IDC_STATIC_1016 || pWnd->GetDlgCtrlID() == IDC_STATIC_1017
		|| pWnd->GetDlgCtrlID() == IDC_STATIC_1018 || pWnd->GetDlgCtrlID() == IDC_STATIC_1019
		|| pWnd->GetDlgCtrlID() == IDC_STATIC_1020 || pWnd->GetDlgCtrlID() == IDC_STATIC_1021
		|| pWnd->GetDlgCtrlID() == IDC_STATIC_1022 || pWnd->GetDlgCtrlID() == IDC_STATIC_1023
		|| pWnd->GetDlgCtrlID() == IDC_STATIC_1008 || pWnd->GetDlgCtrlID() == IDC_STATIC_EX_AXIS_Y
		|| pWnd->GetDlgCtrlID() == IDC_STATIC_1024 /*|| pWnd->GetDlgCtrlID() == IDC_STATIC_OPERATE_HINT*/
		|| pWnd->GetDlgCtrlID() == IDC_STATIC_TYPE_CHOOSE || pWnd->GetDlgCtrlID() == IDC_STATIC_LABEL_SHOWIMGNUM
		|| pWnd->GetDlgCtrlID() == IDC_STATIC_LABEL_SHOWIMGNUM2 || pWnd->GetDlgCtrlID() == IDC_STATIC_LABEL_SHOWIMGNUM3
		|| pWnd->GetDlgCtrlID() == IDC_STATIC_LABEL_SHOWIMGNUM4)  //静态文本
	{
		//15, 60, 170
		pDC->SetTextColor(RGB(255, 255, 255));
		pDC->SetBkMode(TRANSPARENT);    //设置控件透明
		return   (HBRUSH)::GetStockObject(NULL_BRUSH);    //记住一定要有这句
	}

	if (pWnd->GetDlgCtrlID() == IDC_EDIT_COOR_X || pWnd->GetDlgCtrlID() == IDC_EDIT_COOR_Y
		|| pWnd->GetDlgCtrlID() == IDC_EDIT_COOR_Z || pWnd->GetDlgCtrlID() == IDC_EDIT_COOR_RX
		|| pWnd->GetDlgCtrlID() == IDC_EDIT_COOR_RZ || pWnd->GetDlgCtrlID() == IDC_EDIT_COOR_RY
		|| pWnd->GetDlgCtrlID() == IDC_EDIT_EXTERNAL_X || pWnd->GetDlgCtrlID() == IDC_EDIT_EXTERNAL_Y
		|| pWnd->GetDlgCtrlID() == IDC_EDIT_EXTERNAL_Z)
	{
		pDC->SetTextColor(RGB(255, 255, 255));
		pDC->SetBkMode(TRANSPARENT);    //设置控件透明
		return   m_bkBrush;
	}

	if (pWnd->GetDlgCtrlID() == IDC_COMBO_TABLE_GROUP2 || pWnd->GetDlgCtrlID() == IDC_WORKPIECE_TYPE)
	{
		pDC->SetTextColor(RGB(255, 255, 255));
		pDC->SetBkColor(RGB(0, 0, 0));    
		pDC->SetBkMode(TRANSPARENT);    //设置控件透明
		return   m_bkBrush;
	}

	return  m_Brush;
}

void CAssemblyWeld::DoDataExchange(CDataExchange* pDX)
{
	// TODO: 在此添加专用代码和/或调用基类
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CAssemblyWeld)
	DDX_Control(pDX, IDC_EDIT_COOR_X, m_cCoorX);
	DDX_Control(pDX, IDC_EDIT_COOR_Y, m_cCoorY);
	DDX_Control(pDX, IDC_EDIT_COOR_Z, m_cCoorZ);
	DDX_Control(pDX, IDC_EDIT_COOR_RX, m_cCoorRX);
	DDX_Control(pDX, IDC_EDIT_COOR_RY, m_cCoorRY);
	DDX_Control(pDX, IDC_EDIT_COOR_RZ, m_cCoorRZ);
	DDX_Check(pDX, IDC_CHECK_TEACH_POP, m_bTeachPop);
	DDX_Check(pDX, IDC_CHECK_NATURAL_POP, m_bNaturalPop);
	DDX_Check(pDX, IDC_CHECK_PROCESS_POP, m_bProcessPop);
	DDX_Check(pDX, IDC_CHECK3, m_bCleanGunEnable);
	DDX_Check(pDX, IDC_CHECK_GRAY, m_bMeausreThickEnable);
	DDX_Check(pDX, IDC_CHECK_GRAY2, m_bNeedWrap);
	DDX_Control(pDX, IDC_COMBO_TABLE_GROUP2, m_comboTableGroupleft);
	DDX_Control(pDX, IDC_WORKPIECE_TYPE, m_ctlWorkpieceType);
	DDX_Control(pDX, IDC_WORKPIECE_TYPE2, m_comboWorkpieceType);
	//}}AFX_DATA_MAP
	DDX_Check(pDX, IDC_CHECK_MODEL, m_nUseModel);
}

void CAssemblyWeld::SetHintInfo(CString sHintInfo)
{
	WriteLog(sHintInfo);
	((CButton*)GetDlgItem(IDC_STATIC_OPERATE_HINT))->SetWindowTextA(sHintInfo);
}

void CAssemblyWeld::InitMoveState()
{
	CRect rect;
	GetDlgItem(IDC_STATIC_AXIS_STATE)->GetWindowRect(&rect);
	ScreenToClient(&rect);
	rect.left += 10;	
}


void CAssemblyWeld::OnBtnPauseContinue()
{
	WriteLog("单击：测试按钮");
	int nCameraNo = 0;
	if (IDOK == XiMessageBox("校验跟踪相机？"))
		nCameraNo = 0;
	else if (IDOK == XiMessageBox("校验测量相机？"))
		nCameraNo = 1;
	else
		return;

	testCompenWithPara(m_vpUnit[0], nCameraNo);
	return;

//	int nRobotNo = 0;
//	int nCameraNo = 1;
//	T_ANGLE_PULSE tCurPulse(-90000, 0, 0, 0, 0, 0, 0, 0, 0);
//	T_ANGLE_THETA tCurTheta(0, 0.0, 0.0, 0.0, 0, 0.0);
//	vector<T_ROBOT_COORS> vtCurRobotCoors;
//	T_ROBOT_COORS tTool;
//	CUnit* pUnit = m_vpUnit[nRobotNo];
//	CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
////	T_ROBOT_COORS;
//
//
//
//	//bool rst = pUnit->CheckCO2IO("ProtectGasPressure");
//
//	CreateObject(m_tChoseWorkPieceType, pUnit, &m_pWeldAfterMeasure);
//
//	//pRobotDriver->RobotInverseKinematics()
//
//
//	if (1)
//	{
//		int* pImageNum;
//		vector<int> vtImageNum;
//		Local_Welding_Info tLocalWeldingTrackInfo;
//		vector<CvPoint3D64f> vtPointCloud;
//		CvPoint3D64f* pPointCloud = NULL;
//		int PointCloudSize = 0;
//		CvPoint3D64f* tRefPointCloud = NULL;
//		int RefPointCloudSize = 0;
//		T_ROBOT_COORS coor;
//		std::vector< T_ROBOT_COORS>vtCoord;
//		m_pWeldAfterMeasure->m_sPointCloudFileName = "LocalFiles\\OutputFiles\\RobotA\\Recognition\\0_ScanWeldLinePointCloud.txt";
//
//		//WeldLineInfo tWeldInfo = m_pWeldAfterMeasure->m_vvtWeldLineInfoGroup[0][0];
//		//LineOrCircularArcWeldingLine tWeldLine = tWeldInfo.tWeldLine;
//		//int nWeldIndex = tWeldInfo.tAtrribute.nWeldSeamIdx;
//		CString RefPointCloudFileName;
//		RefPointCloudFileName.Format("%s00_PointCloudRecoTrack%d.txt", OUTPUT_PATH + "RobotA" + RECOGNITION_FOLDER, 4);
//	//加载点云数据
//			//vector<CvPoint3D64f> vtPointCloud;
//			m_pWeldAfterMeasure->LoadContourData(pRobotDriver, m_pWeldAfterMeasure->m_sPointCloudFileName, vtPointCloud);
//
//			pPointCloud = (CvPoint3D64f*)vtPointCloud.data();
//			PointCloudSize = vtPointCloud.size();
//
//			// 加载全景相机提取的对应点云
//			vector<CvPoint3D64f> vtRefPointCloud;
//			//		CvPoint3D64f tmp3DPoint;
//
//			vector<CvPoint3D64f> vtTmpPointCloud;
//			int TmpPointCloudSize = 0;
//			CvPoint3D64f* tTmpPointCloud = NULL;
//			tTmpPointCloud = (CvPoint3D64f*)vtTmpPointCloud.data();
//
//			FILE* pf1 = fopen(RefPointCloudFileName, "r");
//			int nNo = 0;
//			CvPoint3D64f tTempPtn;
//			double x;
//			int pcImageNum = 0;
//
//			while (EOF != fscanf(pf1, "%d%lf%lf%lf%lf%lf%lf", &nNo, &tTempPtn.x, &tTempPtn.y, &tTempPtn.z, &x, &x, &x))
//			{
//				vtRefPointCloud.push_back(tTempPtn);
//			}
//			fclose(pf1);
//
//			tRefPointCloud = (CvPoint3D64f*)vtRefPointCloud.data();
//			RefPointCloudSize = vtRefPointCloud.size();
//			pImageNum = (int*)m_pWeldAfterMeasure->m_vtImageNum.data();
//
//			// 韦富进输出焊缝轮廓接口
//			SetPara("LocalFiles\\ExLib\\Vision\\ConfigFiles", "Rebuild_Local_Point_Cloud", "debug_mode", "false");
//			SetPara("LocalFiles\\ExLib\\Vision\\ConfigFiles", "Rebuild_Local_Point_Cloud", "debug_path", "D:\\XiRobotSW\\LocalFiles\\ExLib\\Vision\\ConfigFiles\\test");
//			SetPara("LocalFiles\\ExLib\\Vision\\ConfigFiles", "Rebuild_Local_Point_Cloud", "thread_nums", "4");
//			SetPara("LocalFiles\\ExLib\\Vision\\ConfigFiles", "Get_Local_Welding_Info", "debug_mode", "true");
//			SetPara("LocalFiles\\ExLib\\Vision\\ConfigFiles", "Get_Local_Welding_Info", "debug_path", "D:\\XiRobotSW\\LocalFiles\\ExLib\\Vision\\ConfigFiles\\test");
//			//bool bRst1 = Rebuild_Local_Point_Cloud(pImageNum, (Three_DPoint*)pPointCloud, PointCloudSize, (Three_DPoint*&)tTmpPointCloud, TmpPointCloudSize, "LocalFiles\\ExLib\\Vision\\ConfigFiles");
//			//bool bRst2 = Get_Local_Welding_Info((Three_DPoint*)tTmpPointCloud, TmpPointCloudSize, (Three_DPoint*)tRefPointCloud, RefPointCloudSize, tLocalWeldingTrackInfo, "LocalFiles\\ExLib\\Vision\\ConfigFiles");
//			bool bRst2 = Get_Local_Welding_Info((Three_DPoint*)pPointCloud, PointCloudSize, (Three_DPoint*)tRefPointCloud, RefPointCloudSize, tLocalWeldingTrackInfo, "LocalFiles\\ExLib\\Vision\\ConfigFiles");
//
//			FILE* pf = fopen("LocalFiles\\OutputFiles\\0_ScanWeldLine.txt", "w");
//			for (unsigned int i = 0; i < tLocalWeldingTrackInfo.points_size; i++)
//			{
//				fprintf(pf, "%d %lf %lf %lf\n", i, tLocalWeldingTrackInfo.points[i].x, tLocalWeldingTrackInfo.points[i].y, tLocalWeldingTrackInfo.points[i].z);
//				coor.dX = tLocalWeldingTrackInfo.points[i].x;
//				coor.dY = tLocalWeldingTrackInfo.points[i].y;
//				coor.dZ = tLocalWeldingTrackInfo.points[i].z;
//
//				vtCoord.push_back(coor);
//			}
//			fclose(pf);
//		
//
//	}
//
//	// 加载焊接轨迹
//	E_WELD_SEAM_TYPE eWeldSeamType;
//	double dExAxlePos;
//	std::vector<T_ROBOT_COORS> vtRealWeldCoord;
//	std::vector<int> vnPtnType;
//	if (false == m_pWeldAfterMeasure->LoadRealWeldTrack(0, 0, eWeldSeamType, dExAxlePos, vtRealWeldCoord, vnPtnType)) {
//		return;
//	}
//
//	WeldTrackLineFilter(vtRealWeldCoord, 150, 5);	
//
//	return;
	//SaveGrooveData(0, 2, pRobotDriver->m_strRobotName, false);
	//return;

	//TestGroovePointCloudProcess();
	//TestImageProcess(nRobotNo, nCameraNo);
	//TestLockProcess(nRobotNo, nCameraNo);

	//AfxBeginThread(ThreadTest, this);
	//AfxBeginThread(ThreadTest, this);
	//AfxBeginThread(ThreadTest, this);
	//AfxBeginThread(ThreadTest, this);
	
//	TestLaserCenterPtnImageProcess(nRobotNo, nCameraNo);
	//TestGrooveImageProcess(nRobotNo, nCameraNo);
	//TestEndpointAndStandBoardLine(nRobotNo, nCameraNo);
	//TestFindEndPoint();
	//TestContiSwaySpot();
	//if (IDOK == XiMessageBox("测试图片转化为点云"))
	//{
	//	TestScanEndptnImageProcess();
	//}
	//else if (IDOK == XiMessageBox("测试获取点云端点"))
	//{
	//	TestGetEndPtnUsePointCloud_F();
	//}

	//// 带有安川外部轴机器人BP变量发送和Job:CONTIMOVANY-BP运动测试
	//T_ROBOT_COORS tCoord = pRobotDriver->GetCurrentPos();// (900.0, 1100.0, 1900.0, 0.0, 0.0, 66.0, 0.0, 100.0, 200.0);
	//T_ANGLE_PULSE tPulse = pRobotDriver->GetCurrentPulse();// (0, 0, 0, 0, 0, 0, 0, 0, 0);
	//T_ROBOT_MOVE_INFO tRobotMoveInfo;
	//vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo(0);
	//tCoord.dX += 100.0;
	//tCoord.dY += 100.0;
	//tCoord.dZ -= 100.0;
	//tCoord.dBY += 100.0;
	//tCoord.dBZ -= 100.0;
	//tRobotMoveInfo = pRobotDriver->PVarToRobotMoveInfo(0, tCoord, pRobotDriver->m_tPulseLowSpeed, MOVL);
	//vtRobotMoveInfo.push_back(tRobotMoveInfo);
	//tRobotMoveInfo = pRobotDriver->PVarToRobotMoveInfo(0, tPulse, pRobotDriver->m_tPulseLowSpeed, MOVJ);
	//vtRobotMoveInfo.push_back(tRobotMoveInfo);
	//pRobotDriver->SetMoveValue(vtRobotMoveInfo, false, true);
	//pRobotDriver->CallJob("CONTIMOVANY-BP");

	XiMessageBoxOk("Done");
	return;
}

void CAssemblyWeld::OnBnClickedButtonSimulateIncise()
{
	WriteLog("单击：手动存图");
    SaveErrorData("手动存图");
	return;
 }

void CAssemblyWeld::OnBtnClose() 
{
	WriteLog("单击：退出");
	if (true == m_bAutoWeldWorking)
	{
		XiMessageBoxOk("正在工作中，不能执行线扫!");
		return;
	}

	//m_cIOControl->CloseLeftBurial();
	KillTimer(1);
	KillTimer(2);
	SaveLastCtrlState();
    CleanUp();

	if (!g_bLocalDebugMark)
	{
		for (int i = 0; i < m_vpRobotDriver.size(); i++)
		{
			m_vpUnit[i]->SwitchIO("CoolGas", false);
			m_vpRobotDriver[i]->HoldOff();
			m_vpRobotDriver[i]->ServoOff();
		}
	}

	//重置闪退状态
	COPini opini;
	opini.SetFileName(OPTIONAL_FUNCTION);
	opini.SetSectionName("Flashback");
	opini.WriteString("Flashback", false);

    CDialog::OnOK();
}

void CAssemblyWeld::CleanUp()
{	
	//退出界面显示刷新子线程
	m_bQuit = true;
	m_bIfWindowOn = FALSE;	
	m_bThreadShowMoveStateEnd = false;
}

void CAssemblyWeld::OnBtnSystemPara() 
{
	WriteLog("单击：工艺参数");
	int nRobotNo = 0;
	CUnit* pUnit = m_vpUnit[nRobotNo];
	CWeldParamProcess cWeldParamProcess(m_vpUnit);
    cWeldParamProcess.DoModal();
    return;
}

void CAssemblyWeld::OnBnClickedButtonSpare4()
{
	WriteLog("单击：执行线扫");
	if (true == m_bAutoWeldWorking)
	{
		XiMessageBoxOk("正在工作中，不能执行线扫!");
		return;
	}
	if (true == m_vtRobotThread[m_vpRobotDriver[0]->m_nRobotNo]->bRobotThreadStatus)
	{
		XiMessageBox("禁止重复开启线扫！");
		return;
	}
	AfxBeginThread(ThreadScanLine, m_vtRobotThread[m_vpRobotDriver[0]->m_nRobotNo]);
	return;
}

void CAssemblyWeld::OnBtnVisionShow() 
{
	WriteLog("单击：视觉窗口");
	CDHCameraCtrlDlg cDHCameraCtrlDlg(m_vpUnit, this);
	cDHCameraCtrlDlg.DoModal();
}

void CAssemblyWeld::OnBtnLoadTrack()
{
	WriteLog("单击：识别参数");
	int nRobotNo = 0;
	CUnit* pUnit = m_vpUnit[nRobotNo];
	WAM::WeldAfterMeasure::InputParaWindow(pUnit->m_tContralUnit.strUnitName, m_tChoseWorkPieceType);
	return;
}

UINT CAssemblyWeld::ThreadScanLine(void* pParam)
{
	T_ROBOT_THREAD* tThreadParam = ((T_ROBOT_THREAD*)pParam);
	if (true == tThreadParam->bRobotThreadStatus)
	{
		XUI::MesBox::PopOkCancel("{0} 机械臂线扫线程已在运行", tThreadParam->pRobotCtrl->m_strRobotName);
		return 0;
	}
	long long lTimeS = XI_clock();
	tThreadParam->bRobotThreadStatus = true;
	try
	{
		// 执行线扫
		tThreadParam->cIncisePlanePart->ScanLine(tThreadParam->pRobotCtrl->m_nRobotNo);
	}
	catch (...)
	{
		//已修改
		XUI::MesBox::PopInfo("{0}线扫出现异常", tThreadParam->pRobotCtrl->m_strRobotName.GetBuffer());
		//XiMessageBox("%s 线扫出现异常", tThreadParam->pRobotCtrl->m_strRobotName);
	}

	tThreadParam->bRobotThreadStatus = false;
	//m_lWorkTime += (XI_clock() - lTimeS); // 工作时间中的线扫时间
	//m_lScanTime += (XI_clock() - lTimeS); // 线扫时间
	//m_dTotalScanLen += (tThreadParam->pRobotCtrl->m_dScanLength / 1000.0); // 单位m
	//CStatisticalData* pStatisticalData = CStatisticalData::getInstance();
	//pStatisticalData->UpdateScanData((tThreadParam->pRobotCtrl->m_dScanLength / 1000.0), m_lScanTime);
	//pStatisticalData->UpdateWorkTime((XI_clock() - lTimeS));

	return 0;
}

bool CAssemblyWeld::ScanLine(int nRobotNo)
{
	CUnit* pUnit = m_vpUnit[nRobotNo];
	T_CAMREA_PARAM tCamParam = pUnit->GetCameraParam(pUnit->m_nLineScanCameraNo);
	if (E_VZENSE_CAMERA == tCamParam.eCameraType)
	{
		return ScanLineForVzenseCam(nRobotNo);
	}
	CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
	CHECK_BOOL_RETURN(pUnit->CheckIsReadyRun());
	int nPtnNum = 0;
	CvPoint3D64f* ptPointCloud = NULL;
	
	if (-1 != m_vpUnit[0]->m_nMeasureAxisNo_up)
	{
		BackHome();
	}

	if (1)
	{
		int nCurUseTable = 0;
		COPini opini2;
		opini2.SetFileName(DATA_PATH + m_vpUnit[0]->m_tContralUnit.strUnitName + LINE_SCAN_PARAM);
		opini2.SetSectionName("CurUseTableNo");
		opini2.ReadString("CurUseTableNo", &nCurUseTable);

		//线扫
		CString sFolderName = ".\\LineScan\\GantryLeft";
		if (nCurUseTable != 0)
			sFolderName = ".\\LineScan\\GantryRight";
		//线扫
		std::vector<CvPoint3D64f> vtPointCloud;
		CLineScanForGantry cLineScanForGantry(&m_bNaturalPop);
		if (!cLineScanForGantry.LineScan(pUnit, pUnit->m_vpImageCapture,"Table",sFolderName)
			|| !cLineScanForGantry.ReadPointCloud(vtPointCloud))
		{
			cLineScanForGantry.ShowErrorInfo();
			return false;
		}
		// 获取点云数据
		nPtnNum = vtPointCloud.size();
		ptPointCloud = vtPointCloud.data();
	}
	else
	{
		int i = 0;
		for (i = 0; i < LINESCAN_CAMERA_NUM; i++)
		{
			pUnit->SwitchDHCamera(pUnit->m_nLineScanCameraNo + i, true, true, E_ACQUISITION_MODE_CONTINUE, E_CALL_BACK_MODE_SAVE_IMAGE);
		}
		pUnit->SwitchIO("LineScanLaser", true); //开线扫激光
		// 执行线扫动作
		CHECK_BOOL_RETURN(m_vpLaserLineScreen[nRobotNo]->Start());
		// 关线扫相机及激光
		for (i = 0; i < LINESCAN_CAMERA_NUM; i++)
		{
			pUnit->SwitchDHCamera(pUnit->m_nLineScanCameraNo + i, false);
		}
		pUnit->SwitchIO("LineScanLaser", false); //关线扫激光
		// 获取点云数据
		nPtnNum = 0;
		ptPointCloud = m_vpLaserLineScreen[nRobotNo]->GetPointCloudData(nPtnNum);
	}

	if (nPtnNum == 0 || NULL == ptPointCloud)
	{
		XiMessageBox("获取点云数据失败");
		return false;
	}
	WriteLog("已获取大车点云数据");

	//点云处理
	CHECK_BOOL_RETURN(CreateObject(m_tChoseWorkPieceType, pUnit, &m_pWeldAfterMeasure));
	m_pWeldAfterMeasure->SetRecoParam();

	// 回安全位置
	//OnBnClickedButtonBackHome();
	vector<CvPoint3D64f> vtPointCloud;
	CvPoint3D64f* pPointCloud = NULL;
	int PointCloudSize = 0;
	vector<CvPoint3D64f> vtPointCloud2;
	vector<CvPoint3D64f> vtPointCloud3;
	vector<CvPoint3D32f> vtPointCloud2_32;
	vector<CvPoint3D32f> vtPointCloud3_32;
	CvPoint3D32f* pPointCloud2 = NULL;
	int PointCloudSize2 = 0;
	CvPoint3D32f* pPointCloud3 = NULL;
	int PointCloudSize3 = 0;
	//CString PointCloudFileName_2 = "LineScan\\Gantry\\PointCloud\\Camera2_Thread0.txt";
	//CString PointCloudFileName_3 = "LineScan\\Gantry\\PointCloud\\Camera3_Thread0.txt";
	int nCurUseTable = 0;
	COPini opini2;
	opini2.SetFileName(DATA_PATH + m_vpUnit[0]->m_tContralUnit.strUnitName + LINE_SCAN_PARAM);
	opini2.SetSectionName("CurUseTableNo");
	opini2.ReadString("CurUseTableNo", &nCurUseTable);

	//线扫
	CString PointCloudFileName = "LineScan\\GantryLeft\\PointCloud\\Scan5D.txt";
	if (nCurUseTable != 0)
		PointCloudFileName = "LineScan\\GantryRight\\PointCloud\\Scan5D.txt";
	m_pWeldAfterMeasure->LoadContourData(pRobotDriver, PointCloudFileName, vtPointCloud);
	pPointCloud = vtPointCloud.data();
	PointCloudSize = vtPointCloud.size();
	//CStatisticalData* pStatisticalData = CStatisticalData::getInstance();
	//if (false /*== g_bRemoveCloud*/)//开启去点云背景功能
	//{
	//	char ID2[] = "BackGround";
	//	//char ID3[] = "BackGround1";
	//	long long l1 = XI_clock();
	//	//m_pWeldAfterMeasure->LoadContourData(pRobotDriver, PointCloudFileName_2, vtPointCloud2);
	//	m_pWeldAfterMeasure->LoadContourData(pRobotDriver, PointCloudFileName, vtPointCloud2);
	//	long long l2 = XI_clock();
	//	WriteLog("加载线扫点云耗时：%dms", l2 - l1);
	//	for (int p2 = 0; p2 < vtPointCloud2.size(); p2++)
	//	{
	//		CvPoint3D32f tmp;
	//		tmp = cvPoint3D32f(vtPointCloud2[p2].x, vtPointCloud2[p2].y, vtPointCloud2[p2].z);
	//		vtPointCloud2_32.push_back(tmp);
	//	}
	//	/*for (int p3 = 0; p3 < vtPointCloud3.size(); p3++)
	//	{
	//		CvPoint3D32f tmp;
	//		tmp = cvPoint3D32f(vtPointCloud3[p3].x, vtPointCloud3[p3].y, vtPointCloud3[p3].z);
	//		vtPointCloud3_32.push_back(tmp);
	//	}*/
	//	pPointCloud2 = (CvPoint3D32f*)vtPointCloud2_32.data();
	//	PointCloudSize2 = vtPointCloud2_32.size();
	//	//pPointCloud3 = (CvPoint3D32f*)vtPointCloud3_32.data();
	//	//PointCloudSize3 = vtPointCloud3_32.size();
	//	//if ((TRUE == m_bNaturalPop) && IDOK == XiMessageBox("是否扫描空工作台去除多于点云（执行一次即可）"))
	//	//{
	//	   // CString directory = _T("Local_Files\\ExtLib\\Vision\\BackgroundCloud");
	//	   // m_pWeldAfterMeasure->DelPngFile(directory);//每次去背景前先删除已经有的背景文件
	//	   // BackgroundCloudFileSave(ID2, pPointCloud2, PointCloudSize2, 1.0F, true);
	//	   // BackgroundCloudFileSave(ID3, pPointCloud3, PointCloudSize3, 1.0F, true);
	//	   // XiMessageBox("去点云背景处理成功");
	//	   // return;
	//	//}
	//	l1 = XI_clock();
	//	//PointCloudSize2 = BackgroundCloudRemove(ID2, pPointCloud2, PointCloudSize2, 10.0F, 5.0F) - 1;
	//	l2 = XI_clock();
	//	l1 = XI_clock();
	//	//PointCloudSize3 = BackgroundCloudRemove(ID3, pPointCloud3, PointCloudSize3, 10.0F, 5.0F) - 1;
	//	l2 = XI_clock();
	//	WriteLog("去除工作台点云耗时：%dms", l2 - l1);

	//	PointCloudSize = PointCloudSize2 + PointCloudSize3;
	//	for (int pn = 0; pn < PointCloudSize2; pn++)
	//	{
	//		CvPoint3D64f t2 = cvPoint3D64f((double)pPointCloud2[pn].x, (double)pPointCloud2[pn].y, (double)pPointCloud2[pn].z);
	//		vtPointCloud.push_back(t2);
	//	}
	//	for (int pn3 = 0; pn3 < PointCloudSize3; pn3++)
	//	{
	//		CvPoint3D64f t3 = cvPoint3D64f((double)pPointCloud3[pn3].x, (double)pPointCloud3[pn3].y, (double)pPointCloud3[pn3].z);
	//		vtPointCloud.push_back(t3);
	//	}

	//	CString FileName = DATA_PATH + pUnit->m_tContralUnit.strUnitName + REMOVE_CLOUD_PATH;
	//	l1 = XI_clock();
	//	pPointCloud = m_pWeldAfterMeasure->SaveRemoveCloud(pRobotDriver, FileName, vtPointCloud, PointCloudSize);
	//	l2 = XI_clock();
	//	WriteLog("保存去除工作台点云耗时：%dms", l2 - l1);
	//}
	if (false /*== g_bRemoveCloud*/)//开启去点云背景功能
	{
		char ID[] = "BackGround";
		long long l1 = XI_clock();
		m_pWeldAfterMeasure->LoadContourData(pRobotDriver, PointCloudFileName, vtPointCloud2);
		long long l2 = XI_clock();
		WriteLog("加载线扫点云耗时：%dms", l2 - l1);
		for (int p2 = 0; p2 < vtPointCloud2.size(); p2++)
		{
			CvPoint3D32f tmp;
			tmp = cvPoint3D32f(vtPointCloud2[p2].x, vtPointCloud2[p2].y, vtPointCloud2[p2].z);
			vtPointCloud2_32.push_back(tmp);
		}
		pPointCloud2 = (CvPoint3D32f*)vtPointCloud2_32.data();
		PointCloudSize2 = vtPointCloud2_32.size();
		if ((TRUE == m_bNaturalPop) && IDOK == XiMessageBox("是否扫描空工作台去除多于点云（执行一次即可）"))
		{
			CString directory = _T("Local_Files\\ExtLib\\Vision\\BackgroundCloud");
			m_pWeldAfterMeasure->DelPngFile(directory);//每次去背景前先删除已经有的背景文件
			BackgroundCloudFileSave(ID, pPointCloud2, PointCloudSize2, 1.0F, true);
			XiMessageBox("去点云背景处理成功");
			return true;
		}
		l1 = XI_clock();
		PointCloudSize2 = BackgroundCloudRemove(ID, pPointCloud2, PointCloudSize2, 10.0F, 5.0F) - 1;
		l2 = XI_clock();
		WriteLog("去除工作台点云耗时：%dms", l2 - l1);
		vtPointCloud.clear();
		for (int pn = 0; pn < PointCloudSize2; pn++)
		{
			CvPoint3D64f t2 = cvPoint3D64f((double)pPointCloud2[pn].x, (double)pPointCloud2[pn].y, (double)pPointCloud2[pn].z);
			vtPointCloud.push_back(t2);
		}

		CString FileName = OUTPUT_PATH + pUnit->m_tContralUnit.strUnitName + REMOVE_CLOUD_PATH;
		pPointCloud = m_pWeldAfterMeasure->SaveRemoveCloud(pRobotDriver, FileName, vtPointCloud, PointCloudSize);
	}
	if (false == m_pWeldAfterMeasure->PointCloudProcess(TRUE == m_nUseModel, pPointCloud, PointCloudSize))
	{
		DELETE_POINTER(m_pWeldAfterMeasure);
		XiMessageBox("识别处理失败!");
		return false;
	}
	m_vpLaserLineScreen[nRobotNo]->ReleasePointCloud();

	// 显示处理结果交互软件
	RunInteractiveWindow(pUnit->m_tContralUnit.strUnitName);

	//自動分組
	AutomaticGrouping(pUnit->m_tContralUnit.strUnitName);

	//////焊缝接口更改
	//SmallPartsConvertor convertor;
	//convertor.inputWelds(".\\GraphData\\PointCloudIdentifyReaultAfter.txt");
	//convertor.compute(m_nRobotHangPos);
	////初始化关联文件
	//if (E_DIAPHRAGM == pRobotDriver->m_tChoseWorkPieceType) // 隔板类操作
	//{
	//	((CDiaphragmWeld*)m_pWeldAfterMeasure)->InitWorkPieceReleveInfo(pRobotDriver);
	//}
	DELETE_POINTER(m_pWeldAfterMeasure);
	XiMessageBox("接口更新完成");
	return true;
}

bool CAssemblyWeld::ScanLineForVzenseCam(int nRobotNo)
{
	CUnit* pUnit = m_vpUnit[nRobotNo];
	CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
	T_CAMREA_PARAM tCamParam = pUnit->GetCameraParam(pUnit->m_nLineScanCameraNo);
	std::vector<CvPoint3D64f> vtPointCloud(0);
	
	if (false == GetVzensePointCloud(nRobotNo, tCamParam, vtPointCloud))
	{
		XiMessageBox("采集点云数据失败");
		return false;
	}
	
	int nPtnNum = vtPointCloud.size();
	CvPoint3D64f* ptPointCloud = vtPointCloud.data();
	if (nPtnNum == 0 || NULL == ptPointCloud)
	{
		XiMessageBox("获取点云数据失败");
		return false;
	}
	WriteLog("已获取大车点云数据");

	//点云处理
	CHECK_BOOL_RETURN(CreateObject(m_tChoseWorkPieceType, pUnit, &m_pWeldAfterMeasure));
	m_pWeldAfterMeasure->SetRecoParam();

	// 回安全位置
	//OnBnClickedButtonBackHome();

	if (false == m_pWeldAfterMeasure->PointCloudProcess(TRUE == m_nUseModel, ptPointCloud, nPtnNum))
	{
		DELETE_POINTER(m_pWeldAfterMeasure);
		XiMessageBox("识别处理失败!");
		return false;
	}
	m_vpLaserLineScreen[nRobotNo]->ReleasePointCloud();

	// 显示处理结果交互软件
	RunInteractiveWindow(pUnit->m_tContralUnit.strUnitName);

	//自動分組
	AutomaticGrouping(pUnit->m_tContralUnit.strUnitName);

	//////焊缝接口更改
	//SmallPartsConvertor convertor;
	//convertor.inputWelds(".\\GraphData\\PointCloudIdentifyReaultAfter.txt");
	//convertor.compute(m_nRobotHangPos);
	////初始化关联文件
	//if (E_DIAPHRAGM == pRobotDriver->m_tChoseWorkPieceType) // 隔板类操作
	//{
	//	((CDiaphragmWeld*)m_pWeldAfterMeasure)->InitWorkPieceReleveInfo(pRobotDriver);
	//}
	DELETE_POINTER(m_pWeldAfterMeasure);
	XiMessageBox("接口更新完成");
	return true;
}

bool CAssemblyWeld::GetVzensePointCloud(int nRobotNo, T_CAMREA_PARAM tCamParam, std::vector<CvPoint3D64f>& vtPointCloud)
{
	vtPointCloud.clear();
	CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];

	//CString strPointCloudFile = OUTPUT_PATH + pRobotDriver->m_strRobotName + "\\Recognition\\PointCloudVzenseCam.txt";
	//CString strPointCloudFileNew = OUTPUT_PATH + pRobotDriver->m_strRobotName + "\\Recognition\\PointCloud.txt";
	CString strPointCloudFile = "LocalFiles\\OutputFiles\\" + pRobotDriver->m_strRobotName + "\\Recognition\\PointCloudVzenseCam.txt";
	CString strPointCloudFileNew = "LocalFiles\\OutputFiles\\" + pRobotDriver->m_strRobotName + "\\Recognition\\PointCloud_0.txt";
	DelFiles("LocalFiles\\OutputFiles\\" + pRobotDriver->m_strRobotName + "\\Recognition", "PointCloudVzenseCam.txt");

	std::string sPointCloudFile = strPointCloudFile;
	std::string sPointCloudFileNew = strPointCloudFileNew;

	// 维感3D相机采集点云
	WeiGanCapture Vzense;
	pRobotDriver->m_cLog->Write("维感相机采集点云：FlyingPixelFilterSet");
	Vzense.FlyingPixelFilterSet(Vzense.status, 0, true, 40); // 飞点消除滤波 40
	CHECK_BOOL_RETURN((VzReturnStatus::VzRetOK == Vzense.status));

	pRobotDriver->m_cLog->Write("维感相机采集点云：TOFExposureSet");
	Vzense.TOFExposureSet(Vzense.status, 0, VzExposureControlMode_Manual, 1000); // 曝光 1000
	CHECK_BOOL_RETURN((VzReturnStatus::VzRetOK == Vzense.status));

	pRobotDriver->m_cLog->Write("维感相机采集点云：ConfidenceFilterSet");
	Vzense.ConfidenceFilterSet(Vzense.status, 0, false);
	CHECK_BOOL_RETURN((VzReturnStatus::VzRetOK == Vzense.status));

	pRobotDriver->m_cLog->Write("维感相机采集点云：CloudPoint");
	Vzense.CloudPoint(Vzense.status, 0, sPointCloudFile);
	Sleep(2000);
	CHECK_BOOL_RETURN((VzReturnStatus::VzRetOK == Vzense.status));

	pRobotDriver->m_cLog->Write("维感相机采集点云：CameraClose");
	Vzense.CameraClose(Vzense.status, 0);
	CHECK_BOOL_RETURN((VzReturnStatus::VzRetOK == Vzense.status));

	pRobotDriver->m_cLog->Write("维感相机采集点云：CameraApiShutdown");
	Vzense.CameraApiShutdown(Vzense.status);
	CHECK_BOOL_RETURN((VzReturnStatus::VzRetOK == Vzense.status));

	// 坐标相机坐标系到机器人世界坐标系转换  并按新格式保存
	FILE *pf = fopen(sPointCloudFile.c_str(), "r");
	FILE *pfNew = fopen(sPointCloudFileNew.c_str(), "w");
	if (NULL == pf) return false;

	CvPoint3D64f tPtn;
	int nPtnNo = 0;
	while (EOF != fscanf(pf, "%lf%lf%lf", &tPtn.x, &tPtn.y, &tPtn.z))
	{
		if (fabs(tPtn.x) < 0.0001 && fabs(tPtn.y) < 0.0001 && fabs(tPtn.z) < 0.0001)
		{
			continue; // 去除无效数据
		}
		tPtn = CoordTrans::TransPoint(tCamParam.tCameraTransfomPara.dMatrix, tPtn);
		fprintf(pfNew, "%d%11.3lf%11.3lf%11.3lf\n", nPtnNo++, tPtn.x, tPtn.y, tPtn.z);
		vtPointCloud.push_back(tPtn);
	}
	fclose(pf);
	fclose(pfNew);
	return true;
}

bool CAssemblyWeld::CreateObject(E_WORKPIECE_TYPE ePartType, CUnit* pUnit, WAM::WeldAfterMeasure** pWeldAfterMeasure)
{
	DELETE_POINTER(*pWeldAfterMeasure);
	switch (ePartType)
	{
	case E_DIAPHRAGM: // 隔板
		*pWeldAfterMeasure = new CDiaphragmWeld(pUnit, ePartType);
		break;
	case E_LINELLAE: // 线条 
		*pWeldAfterMeasure = new GenericWeld(pUnit, ePartType);
		break;
	case SMALL_PIECE: // 小散件(先测后焊)
		*pWeldAfterMeasure = new GenericWeld(pUnit, ePartType);
		break;
	case STIFFEN_PLATE: // 加劲板(吊车梁)
		*pWeldAfterMeasure = new GenericWeld(pUnit, ePartType);
		break;
	case E_PURLIN_HANGER: // 檩托
		//*pWeldAfterMeasure = new PurlinHanger(pRobotDriver, ePartType);
		break;
	case E_END_PLATE: // 端板 屋面梁
		*pWeldAfterMeasure = new GenericWeld(pUnit, ePartType);
		break;
	case E_CORBEL: // 牛腿
		*pWeldAfterMeasure = new GenericWeld(pUnit, ePartType);
		break;
	case E_CAKE: // 佳木斯电厂
		*pWeldAfterMeasure = new GenericWeld(pUnit, ePartType);
		break;
	case E_BEVEL: // 坡口
		*pWeldAfterMeasure = new class::GrooveWeld(pUnit, ePartType);
		break;	
	case E_SINGLE_BOARD: // 单板单筋
			*pWeldAfterMeasure = new class::SingleBoard(pUnit, ePartType);
			break;
	case E_SCAN_WELD_LINE: // 全扫描测量
		*pWeldAfterMeasure = new class::ScanWeldLine(pUnit, ePartType);
		break;
	default:
		XiMessageBox("工件类型错误！"); return false;
		return false;
	}

	return true;
}
bool CAssemblyWeld::WeldAfterMeasureMultiMachine()
{
	if (g_bAutoGroupingMark)
	{
		// 加载自动分组数据
		CString strFileAuto = OUTPUT_PATH + m_vpUnit[0]->m_tContralUnit.strUnitName + "\\" + RECOGNITION_FOLDER + "planned_welds.txt";
		LoadGroupingResult(strFileAuto);
	}
	else
	{
		// 手动分组数据
		LoadCloudProcessResultMultiMachine(m_vpUnit[0]->m_tContralUnit.strUnitName);
	}
	// 判断任务数量
	vector<CWinThread* > vpcThread;//线程状态
	vector<int> vnWorkRobot;// 工作单元	
	int i = 0, n = 0;
	for (i = 0; i < m_vvvtWeldSeamInfo.size(); i++)
	{
		vpcThread.clear();
		vnWorkRobot.clear();
		for (n = 0; n < m_vvvtWeldSeamInfo[i].size(); n++)
		{
			m_vpUnit[n]->m_bSingleRobotWork = TRUE;
			if (m_vvvtWeldSeamInfo[i].at(n).size() > 0)
			{
				vnWorkRobot.push_back(n);
			}
		}
		if (vnWorkRobot.size() > 1)// 多机工作
		{
			for (n = 0; n < vnWorkRobot.size(); n++)
			{
				m_vpUnit[vnWorkRobot[n]]->m_bSingleRobotWork = false;
			}
		}
		vpcThread.resize(vnWorkRobot.size());
		//int nWorkNo = 0;
		//for (; nWorkNo < vnWorkRobot.size(); nWorkNo++)
		int nWorkNo = vnWorkRobot.size() - 1;
		for (; nWorkNo >= 0; nWorkNo--)
		{
			T_ROBOT_THREAD* tRobot = new T_ROBOT_THREAD();
			tRobot->nGroupNo = i;
			tRobot->cIncisePlanePart = this;
			tRobot->bRobotThreadStatus = false;
			tRobot->pRobotCtrl = m_vpUnit[vnWorkRobot.at(nWorkNo)]->GetRobotCtrl();
			vpcThread[nWorkNo] = AfxBeginThread(ThreadWeldAfterMeasureMultiMachine, tRobot);
			Sleep(50);
		}

		for (nWorkNo = 0; nWorkNo < vnWorkRobot.size(); nWorkNo++)
		{
			WaitForSingleObject(vpcThread[nWorkNo]->m_hThread, INFINITE);
		}
	}
	XiMessageBox("焊接完成");
	return true;
}
UINT CAssemblyWeld::ThreadWeldAfterMeasureMultiMachine(void* pParam)
{
	T_ROBOT_THREAD* pObj = (T_ROBOT_THREAD*)pParam;
	//DELETE_POINTER(pParam);
	long long lTimeS = XI_clock();
	int nCurGroupNo = pObj->nGroupNo;
	pObj->bRobotThreadStatus = true;
	bool bRst = pObj->cIncisePlanePart->WorkWeldAfterMeasure(pObj->pRobotCtrl->m_nRobotNo, nCurGroupNo);
	m_lWorkTime += (XI_clock() - lTimeS); // 工作时间中的测量焊接流程时间
	pObj->bRobotThreadStatus = false;
	return 0;
}

UINT CAssemblyWeld::ThreadWeldAfterMeasure(void *pParam)
{
	CAssemblyWeld *pObj = (CAssemblyWeld*)pParam;
	if (true == pObj->m_bAutoWeldWorking)
	{
		XiMessageBoxOk("正在工作中，不能重复开启!");
		return -1;
	}
	long long lTimeS = XI_clock();
	pObj->m_bAutoWeldWorking = true;
	int nCurGroupNo = 0;
	bool bRst = false;
	if (54 == pObj->m_vpRobotDriver[0]->m_nExternalAxleType) // 国焊坡口焊接调度流程
	{
		bRst = pObj->WeldAfterMeasureMultiMachine_G();
	}
	else
	{
		bRst = pObj->WeldAfterMeasureMultiMachine()/*WorkWeldAfterMeasure(0,nCurGroupNo)*/;
	}
	pObj->m_bAutoWeldWorking = false;
	//m_lWorkTime += (XI_clock() - lTimeS); 
	// 工作时间中的测量焊接流程时间
	//CStatisticalData* pStatisticalData = CStatisticalData::getInstance();
	//pStatisticalData->UpdateWorkTime((XI_clock() - lTimeS));
	return 0;
}

UINT CAssemblyWeld::ThreadCheckWeldIO(void* pParam)
{
	CAssemblyWeld* pObj = (CAssemblyWeld*)pParam;
	pObj->CheckWeldIO(pObj);
	return 0;
}

int CAssemblyWeld::CheckWeldIO(void* pParam)
{
	XUI::MesBox::PopError("功能未实现");
	return 0;
	CAssemblyWeld* pObj = (CAssemblyWeld*)pParam;
	while (true == pObj->m_bAutoWeldWorking)
	{
		if (false == pObj->m_vpUnit[0]->CheckCO2IO("ProtectGasPressure"))
		{
			pObj->m_vpUnit[0]->UnitEmgStop();
			pObj->m_bAutoWeldWorking = false;
			XiMessageBoxOk("保护气异常，停止焊接！");
			//return 0;
		}
	}
	return 0;
}

bool CAssemblyWeld::WorkWeldAfterMeasure(int nRobotNo, int& nCurGroupNo)
{
	long long dCalceStartime;
	dCalceStartime = XI_clock();
	CString sHintInfo;
	CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
	CUnit* pUnit = m_vpUnit[nRobotNo];
	// 创建并初始化加劲板焊接实例
	WAM::WeldAfterMeasure* pWeldAfterMeasure = NULL;// = m_pWeldAfterMeasure; // 为了不修以下变量名
	CHECK_BOOL_RETURN(CreateObject(m_tChoseWorkPieceType, m_vpUnit[nRobotNo], &pWeldAfterMeasure));
	pWeldAfterMeasure->SetRecoParam();
	pWeldAfterMeasure->SetHardware(m_vpScanInit[nRobotNo]);
	pWeldAfterMeasure->SetWeldParam(&m_bStartArcOrNot, &m_bNaturalPop, &m_bTeachPop, &m_bNeedWrap);
	pWeldAfterMeasure->m_pColorImg = &m_vpShowLaserImgBuff[nRobotNo];
	pWeldAfterMeasure->m_bIsLocalDebug = GetLocalDebugMark() == TRUE;
	pWeldAfterMeasure->LoadPauseInfo();
	pWeldAfterMeasure->m_ptUnit->m_bWeldAfterMeasureMoveExAxis = (E_SCAN_WELD_LINE == m_tChoseWorkPieceType) ? true : false;
	// 点云处理
	//已修改
	SetHintInfo(XUI::Languge::GetInstance().translate("点云处理中……"));
	//SetHintInfo("点云处理中……");
	pWeldAfterMeasure->m_vtWeldSeamData = m_vvvtWeldSeamData.at(nCurGroupNo).at(nRobotNo);
	pWeldAfterMeasure->m_vtWeldSeamInfo = m_vvvtWeldSeamInfo.at(nCurGroupNo).at(nRobotNo);
	// 转换矩阵位置添加 // !!!!单机多级需要优化 单机龙门
	if (g_bGantryEnableMark) // 龙门使用
	{
		pWeldAfterMeasure->WeldSeamTransCoor_Gantry2Robot(pWeldAfterMeasure->m_vtWeldSeamData, pWeldAfterMeasure->m_vtWeldSeamInfo);
	}
	// 焊缝排序分组
	//已修改
	SetHintInfo(XUI::Languge::GetInstance().translate("焊缝信息排序分组……"));//"焊缝信息排序分组……"
	//SetHintInfo("焊缝信息排序分组……");
	int nWeldGropuNum = 0;
	CHECK_BOOL_RETURN(pWeldAfterMeasure->WeldSeamGrouping(nWeldGropuNum));

	// 跟踪临时添加：跟踪从测量使用跟踪相机 记录原测量相机号
	//pUnit->LoadCameraParam(pUnit->m_tContralUnit.strUnitName, pUnit->m_vtCameraPara);
	CString strFileName = DATA_PATH + pUnit->m_tContralUnit.strUnitName + CAMERA_PARAM_INI;
	COPini opini;
	opini.SetFileName(false, strFileName);
	opini.SetSectionName("Base");
	opini.ReadAddString("MeasureCameraNo", &pUnit->m_nMeasureCameraNo, -1);
	opini.ReadAddString("TrackCameraNo", &pUnit->m_nTrackCameraNo, -1);
	opini.ReadAddString("LineScanCameraNo", &pUnit->m_nLineScanCameraNo, -1);
	int nRealMeasureCamNo = pUnit->m_nMeasureCameraNo; 
	bool bChange = false;
	bool bWeld = false;
	// 循环测量和焊接每个焊缝组
	for (int nGroupNo = pWeldAfterMeasure->m_nPauseGroupNo; nGroupNo < nWeldGropuNum; nGroupNo++)
	{
		if (pWeldAfterMeasure->m_vvtWeldLineInfoGroup[nGroupNo][0].tAtrribute.nStartWrapType==10)
		{
			m_tChoseWorkPieceType = SMALL_PIECE;
			CHECK_BOOL_RETURN(CreateObject(m_tChoseWorkPieceType, m_vpUnit[nRobotNo], &pWeldAfterMeasure));
			bChange = true;

			pWeldAfterMeasure->SetRecoParam();
			pWeldAfterMeasure->SetHardware(m_vpScanInit[nRobotNo]);
			pWeldAfterMeasure->SetWeldParam(&m_bStartArcOrNot, &m_bNaturalPop, &m_bTeachPop, &m_bNeedWrap);
			pWeldAfterMeasure->m_pColorImg = &m_vpShowLaserImgBuff[nRobotNo];
			pWeldAfterMeasure->m_bIsLocalDebug = GetLocalDebugMark() == TRUE;
			pWeldAfterMeasure->LoadPauseInfo();
			pWeldAfterMeasure->m_ptUnit->m_bWeldAfterMeasureMoveExAxis = (E_SCAN_WELD_LINE == m_tChoseWorkPieceType) ? true : false;
			// 点云处理
			SetHintInfo("点云处理中……");
			pWeldAfterMeasure->m_vtWeldSeamData = m_vvvtWeldSeamData.at(nCurGroupNo).at(nRobotNo);
			pWeldAfterMeasure->m_vtWeldSeamInfo = m_vvvtWeldSeamInfo.at(nCurGroupNo).at(nRobotNo);
			// 转换矩阵位置添加 // !!!!单机多级需要优化 单机龙门
			if (g_bGantryEnableMark) // 龙门使用
			{
				pWeldAfterMeasure->WeldSeamTransCoor_Gantry2Robot(pWeldAfterMeasure->m_vtWeldSeamData, pWeldAfterMeasure->m_vtWeldSeamInfo);
			}
			// 焊缝排序分组
			SetHintInfo("焊缝信息排序分组……");
			nWeldGropuNum = 0;
			CHECK_BOOL_RETURN(pWeldAfterMeasure->WeldSeamGrouping(nWeldGropuNum));
		}
		else
		{
			//m_tChoseWorkPieceType = E_SCAN_WELD_LINE;
			CHECK_BOOL_RETURN(CreateObject(m_tChoseWorkPieceType, m_vpUnit[nRobotNo], &pWeldAfterMeasure));
			bChange = true;

			pWeldAfterMeasure->SetRecoParam();
			pWeldAfterMeasure->SetHardware(m_vpScanInit[nRobotNo]);
			pWeldAfterMeasure->SetWeldParam(&m_bStartArcOrNot, &m_bNaturalPop, &m_bTeachPop, &m_bNeedWrap);
			pWeldAfterMeasure->m_pColorImg = &m_vpShowLaserImgBuff[nRobotNo];
			pWeldAfterMeasure->m_bIsLocalDebug = GetLocalDebugMark() == TRUE;
			pWeldAfterMeasure->LoadPauseInfo();
			pWeldAfterMeasure->m_ptUnit->m_bWeldAfterMeasureMoveExAxis = (E_SCAN_WELD_LINE == m_tChoseWorkPieceType) ? true : false;
			// 点云处理
			SetHintInfo("点云处理中……");
			pWeldAfterMeasure->m_vtWeldSeamData = m_vvvtWeldSeamData.at(nCurGroupNo).at(nRobotNo);
			pWeldAfterMeasure->m_vtWeldSeamInfo = m_vvvtWeldSeamInfo.at(nCurGroupNo).at(nRobotNo);
			// 转换矩阵位置添加 // !!!!单机多级需要优化 单机龙门
			if (g_bGantryEnableMark) // 龙门使用
			{
				pWeldAfterMeasure->WeldSeamTransCoor_Gantry2Robot(pWeldAfterMeasure->m_vtWeldSeamData, pWeldAfterMeasure->m_vtWeldSeamInfo);
			}
			// 焊缝排序分组
			SetHintInfo("焊缝信息排序分组……");
			nWeldGropuNum = 0;
			CHECK_BOOL_RETURN(pWeldAfterMeasure->WeldSeamGrouping(nWeldGropuNum));
		}
		pWeldAfterMeasure->m_nGroupNo = nGroupNo;
		CString sInfo;
		sInfo.Format("焊接第%d组焊缝？共%d组", nGroupNo + 1, nWeldGropuNum);
		int nRst = 0;

		CString cStrTitle = XUI::Languge::GetInstance().translate(
			"{0}组号输(1-{1})关闭进入下一组无效退出",
			XUI::Languge::GetInstance().translate(pRobotDriver->m_strRobotName),
			nWeldGropuNum);
		std::vector<CString> vsInputName(1, "焊接组号:");
		vsInputName.push_back("焊接板厚:");
		std::vector<double> vnInputData(1, nGroupNo + 1);
		double dThick = 10.0;
		CString str;
		str.Format("%s%s\\SystemParam.ini", DATA_PATH, pRobotDriver->m_strRobotName);
		COPini opini;
		opini.SetFileName(str);
		opini.SetSectionName("BOARD_THICK");
		opini.ReadString("BoardThick", &dThick);
		vnInputData.push_back(dThick);

		// 修改识别参数窗口
		ParamInput cParamDlg(cStrTitle, vsInputName, &vnInputData);
		if (TRUE == m_bNaturalPop)
		{
			nRst = cParamDlg.DoModal();
			if ((1 == nRst) && ((int)vnInputData[0] - 1 >= 0) && ((int)vnInputData[0] - 1 < nWeldGropuNum))
			{	
				nGroupNo = (int)vnInputData[0] - 1;
				m_vpScanInit[nRobotNo]->m_pTraceModel->dBoardThink = vnInputData[1];
				opini.WriteString("BoardThick", m_vpScanInit[nRobotNo]->m_pTraceModel->dBoardThink);
			}
			else if (((int)vnInputData[0] < 1) || (vnInputData[0] > nWeldGropuNum))
			{
				break;
			}
			else
			{
				//已修改
				//sHintInfo.Format("跳过第%d组焊缝焊接", nGroupNo + 1);
				sHintInfo = XUI::Languge::GetInstance().translate("跳过第{0}组焊缝焊接", nGroupNo + 1);
				continue;
			}
		}
		//已修改
		//sHintInfo.Format("计算第%d组焊缝测量轨迹……", nGroupNo + 1);
		sHintInfo = XUI::Languge::GetInstance().translate("计算第{0}组焊缝测量轨迹……", nGroupNo + 1);
		SetHintInfo(sHintInfo);
		vector<T_ROBOT_COORS> vtMeasureCoord;
		vector<T_ANGLE_PULSE> vtMeasurePulse;
		vector<int> vnMeasureType;
		double dExAxlePos = 0.0;
		double dExAxlePos_y = 0.0;

		// 跟踪临时添加：跟踪从测量使用跟踪相机 记录原测量相机号
 		if (4 == pWeldAfterMeasure->m_vvtWeldLineInfoGroup[nGroupNo][0].tAtrribute.nStartWrapType)
		{
			pUnit->m_nMeasureCameraNo = pUnit->m_nTrackCameraNo;
		}
		else
		{
			pUnit->m_nMeasureCameraNo = nRealMeasureCamNo;
		}

		// 测量数据测量运动
		//已修改
		//sHintInfo.Format("XiRobot %s 第%d组焊缝测量轨迹计算中…… 进度：%d组/共%d组", pRobotDriver->m_strRobotName, nGroupNo + 1, nGroupNo + 1, nWeldGropuNum);
		sHintInfo = XUI::Languge::GetInstance().translate("XiRobot{0}第{1}组焊缝测量轨迹计算中……进度:{2}组/共{3}组", pRobotDriver->m_strRobotName.GetBuffer(), nGroupNo + 1, nGroupNo + 1, nWeldGropuNum);
		SetHintInfo(sHintInfo);
		pWeldAfterMeasure->m_vvtWeldSeamGroupAdjust = pWeldAfterMeasure->m_vvtCowWeldSeamGroupAdjust;
		// 机器人基座中心到大车中心距离
		double dSafeHeight = 0.0;
		CHECK_BOOL_RETURN(pWeldAfterMeasure->CalcMeasureTrack(nGroupNo, vtMeasureCoord, vtMeasurePulse, vnMeasureType, dExAxlePos, dSafeHeight));
		if (g_bGantryEnableMark) // 龙门使用
		{
			// 龙门设备移动龙门轴，移动轨道坐标Y和机械臂Z1
			//=======================计算大车位置=======================
			CvPoint3D64f CarPoint;
			CarPoint.x = pUnit->m_dExAxisXPos;	//轨道方向和机器人的x方向平行
			CarPoint.y = 0;
			CarPoint.z = 0;
			CvPoint3D64f RealCarPoint = pUnit->TransCoor_Robot2Gantry(CarPoint);
			double dTrackPos = RealCarPoint.y;	//轨道方向是世界坐标系的y方向
			dTrackPos += pUnit->m_dGantryRobotOriginDis;
			if (!pUnit->m_bSingleRobotWork)// 多机
			{
				dTrackPos = pWeldAfterMeasure->m_vvtWeldLineInfoGroup[nGroupNo].at(0).tAtrribute.dGroupTrackPos +
					pUnit->m_dRobotBaseToGantryCtrDis;
			}
			//continue;
			//=======================================================
			m_vpUnit[0]->GetRobotCtrl()->m_pvpMotorDriver->at(0)->PosMove(dTrackPos, 1, 5000.0 / 60, 0.3, 0.3);
			m_vpUnit[0]->GetRobotCtrl()->m_pvpMotorDriver->at(0)->CheckAxisDone();
		}
		
		////// 测量轨迹碰撞检测
		////sHintInfo.Format("第%d组 测量碰撞检测中……", nGroupNo + 1);
		////SetHintInfo(sHintInfo);
		////pWeldAfterMeasure->GenerateFilePLYPlane(dExAxlePos);
		////pWeldAfterMeasure->SetCheckCollidePlane();
		////bool bIsCollide = pWeldAfterMeasure->CheckIsCollide(vtMeasureCoord, dExAxlePos, false);
		////sHintInfo.Format("第%d组 测量碰撞检测结束 [%s]", nGroupNo + 1, bIsCollide ? "有干涉" : "无干涉");
		////SetHintInfo(sHintInfo);
		//continue; // 临时调试使用 跳过测量过轨迹以后的流程

		long long dCalceEndtime;
		dCalceEndtime = XI_clock();
		double dCalceTime = 0.0;
		dCalceTime = (double)(dCalceEndtime - dCalceStartime) / CLOCKS_PER_SEC;

		long long dTeachStartime;
		dTeachStartime = XI_clock();
		// 外部轴运动 执行测量运动 外部轴 相机 图像处理对象
		//已修改
		//sHintInfo.Format("XiRobot %s 第%d组焊缝测量中…… 进度：%d组/共%d组", pRobotDriver->m_strRobotName, nGroupNo + 1, nGroupNo + 1, nWeldGropuNum);
		sHintInfo = XUI::Languge::GetInstance().translate("XiRobot{0}第{1}组焊缝测量中……进度：{2}组/共{3}组", pRobotDriver->m_strRobotName.GetBuffer(), nGroupNo + 1, nGroupNo + 1, nWeldGropuNum);
		SetHintInfo(sHintInfo);

		//jwq 临时，判断是否是闭合圆弧
		double dDis = TwoPointDis(pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0].StartPoint.x,
			pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0].StartPoint.y,
			pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0].StartPoint.z,
			pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0].EndPoint.x,
			pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0].EndPoint.y,
			pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0].EndPoint.z);
		XiAlgorithm alg;
		double dDirAngleS = alg.CalcArcAngle(pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0].StartNormalVector.x, pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0].StartNormalVector.y);
		double dDirAngleE = alg.CalcArcAngle(pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0].EndNormalVector.x, pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0].EndNormalVector.y);
		double dMidDirAngle = (dDirAngleS + dDirAngleE) / 2.0;
		dMidDirAngle = fmod(dMidDirAngle, 360.0);
		dMidDirAngle = dMidDirAngle < 0.0 ? dMidDirAngle + 360.0 : dMidDirAngle;
		double dMachineOffsetDir = dMidDirAngle < 180.0 ? -1.0 : 1.0;
		double dStartY = pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0].StartPoint.y;
		double dEndY = pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0].EndPoint.y;
		double dWeldMinY = dStartY < dEndY ? dStartY : dEndY;
		double dMachinePosForArc = dWeldMinY + (100.0 * dMachineOffsetDir);

		// 测试参数
		pWeldAfterMeasure->m_bWorkpieceShape = WeldingLineIsArc(pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0]); // 工件形状
		pWeldAfterMeasure->m_dOverflowHoleStart = pWeldAfterMeasure->m_vvtWeldLineInfoGroup[nGroupNo][0].tAtrribute.dStartHoleSize;// 过水孔
		pWeldAfterMeasure->m_dOverflowHoleEnd = pWeldAfterMeasure->m_vvtWeldLineInfoGroup[nGroupNo][0].tAtrribute.dEndHoleSize;// 过水孔
		pWeldAfterMeasure->m_pScanInit->m_bWorkpieceShape = pWeldAfterMeasure->m_bWorkpieceShape;

		if (!GetLocalDebugMark())
		{
			//已修改
			if ((!pWeldAfterMeasure->m_ptUnit->m_bBreakPointContinue) && (!m_bNaturalPop ||
				IDOK == XUI::MesBox::PopOkCancel("开始示教!!!可跳过但必须确保有测量数据!!!")/*XiMessageBox("开始示教？\n!!!可跳过但必须确保有测量数据!!!")*/))
			//if ((!pWeldAfterMeasure->m_ptUnit->m_bBreakPointContinue) && (!m_bNaturalPop || 
			//	IDOK == XiMessageBox("开始示教？\n!!!可跳过但必须确保有测量数据!!!")))
			{
				long long lTimeS = XI_clock();
				//打开镜片防护
				pUnit->SwitchIO("LensProtection", true);
				//if (!pUnit->SwitchIO("LensProtection", true)) {
				//	XiMessageBox("镜片防护打开失败!!!\n 即将退出焊接程序");
				//	return false;
				//}
				//示教测量运动

				if (false && pWeldAfterMeasure->m_bWorkpieceShape
					&& dDis > 1.0)
				{
					// 单机圆弧测量先动外部轴到合适位置
					if (0 != pUnit->MoveExAxisFun(dMachinePosForArc, 9000, pUnit->m_nMeasureAxisNo))return false;
					pUnit->WorldCheckRobotDone();
					double dCurExPos = pUnit->GetExPositionDis(pUnit->m_nMeasureAxisNo);
					if (fabs(dMachinePosForArc - dCurExPos) > 5.0) // 与目标位置相差超过阈值 判断为运动失败
					{
						XiMessageBox("ScanWeldTrack:外部轴未运动到指定位置");
						return false;
					}

					//jwq临时，暂时只用一条焊缝
					if (!ScanWeldTrack(pWeldAfterMeasure, pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0], nRobotNo, nGroupNo, dSafeHeight))
					{
						return false;
					}
				}
				else
				{
					if (!pWeldAfterMeasure->DoTeach(nGroupNo, vtMeasurePulse, vnMeasureType, dExAxlePos))
					{
						if (E_STAND_SEAM == pWeldAfterMeasure->GetWeldSeamType(pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0]))
						{
							//已修改
							XUI::MesBox::PopInfo("{0}立峰“测量”失败，继续其他焊缝焊接", pRobotDriver->m_strRobotName.GetBuffer());
							//XiMessageBox("%s 立峰“测量”失败，继续其他焊缝焊接", pRobotDriver->m_strRobotName);
							continue;
						}
						return false;
					}
				}

				//continue;
				//关闭镜片防护
				pUnit->SwitchIO("LensProtection", false);
				//if (!pUnit->SwitchIO("LensProtection", false)) {
				//	XiMessageBox("镜片防护关闭失败!!!\n 即将退出焊接程序");
				//	return false;
				//}
			}
			else if (!pWeldAfterMeasure->m_bWorkpieceShape)
			{
				pWeldAfterMeasure->SetTeachData(vtMeasurePulse, vnMeasureType, dExAxlePos, dExAxlePos_y); // 调试时需要设置 实际运行时DoTeach内部设置
				CHECK_BOOL_RETURN(pWeldAfterMeasure->LoadTeachResult(nGroupNo));
			}

		}
		else
		{
			//pWeldAfterMeasure->DoTeach(nGroupNo, vtMeasurePulse, vnMeasureType, dExAxlePos);
			CHECK_BOOL_RETURN(pWeldAfterMeasure->GeneralTeachResult(nGroupNo, vtMeasureCoord, vtMeasurePulse, vnMeasureType, dExAxlePos));
			//continue;
			pWeldAfterMeasure->SetTeachData(vtMeasurePulse, vnMeasureType, dExAxlePos, dExAxlePos_y); // 调试时需要设置 实际运行时DoTeach内部设置
			CHECK_BOOL_RETURN(pWeldAfterMeasure->LoadTeachResult(nGroupNo));
		}
		//continue;

		// 计算焊道
		//已修改
		//sHintInfo.Format("XiRobot %s 第%d组焊缝焊接轨迹计算中…… 进度：%d组/共%d组", pRobotDriver->m_strRobotName, nGroupNo + 1, nGroupNo + 1, nWeldGropuNum);
		sHintInfo = XUI::Languge::GetInstance().translate("XiRobot{0}第{1}组焊缝焊接轨迹计算中…… 进度：{2}组/共{3}组", pRobotDriver->m_strRobotName.GetBuffer(), nGroupNo + 1, nGroupNo + 1, nWeldGropuNum);
		SetHintInfo(sHintInfo);
		//continue; // 临时调试使用 跳过测量过轨迹以后的流程
		if (/*(!pWeldAfterMeasure->m_bWorkpieceShape || dDis <= 1.0) && */!pWeldAfterMeasure->CalcWeldTrack(nGroupNo))
		{
			if (E_STAND_SEAM == pWeldAfterMeasure->GetWeldSeamType(pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0]))
			{
				//已修改
				XUI::MesBox::PopInfo("{0}立峰“计算”失败，继续其他焊缝焊接", pRobotDriver->m_strRobotName.GetBuffer());
				//XiMessageBox("%s 立峰“计算”失败，继续其他焊缝焊接", pRobotDriver->m_strRobotName);
				continue;
			}
			return false;
		}

		long long dTeachEndtime;
		dTeachEndtime = XI_clock();
		double dTeachTime = 0.0;
		dTeachTime = (double)(dTeachEndtime - dTeachStartime) / CLOCKS_PER_SEC;

		// 时间统计
		time_t curtime;
		time(&curtime);
		tm *nowtime = localtime(&curtime);
		//CString strRobot1 = pRobotDriver->m_strRobotName;
		CString strPath1;
		strPath1.Format(".\\SteelStructure\\焊接前时间\\%d%d%d.txt", 1900 + nowtime->tm_year, 1 + nowtime->tm_mon, nowtime->tm_mday);
		//string strPathAdr = strPath1;

		std::ofstream outline;
		outline.open(string(strPath1.GetBuffer()), ios::app);

		outline << setiosflags(ios::fixed) << setprecision(4) << "点云处理时间:" << dCalceTime << "测量时间:" << dTeachTime << endl;
		outline.close();

		//continue; // 不焊接调试

		// 执行焊接
		//已修改
		//sHintInfo.Format("XiRobot %s 第%d组焊缝焊接中…… 进度：%d组/共%d组", pRobotDriver->m_strRobotName, nGroupNo + 1, nGroupNo + 1, nWeldGropuNum);
		sHintInfo = XUI::Languge::GetInstance().translate("XiRobot{0}第{1}组焊缝焊接中…… 进度：{2}组/共{3}组", pRobotDriver->m_strRobotName.GetBuffer(), nGroupNo + 1, nGroupNo + 1, nWeldGropuNum);
		SetHintInfo(sHintInfo);
		//2024/02/29 新加焊接调度（跟踪\普通焊接 切换调度）
		if (!WeldSchedule(pWeldAfterMeasure, nGroupNo))
		{
			if (E_STAND_SEAM == pWeldAfterMeasure->GetWeldSeamType(pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0]))
			{
				//已修改
				XUI::MesBox::PopInfo("{0}立缝“焊接”失败，继续其他焊缝焊接", pRobotDriver->m_strRobotName.GetBuffer());
				//XiMessageBox("%s 立缝“焊接”失败，继续其他焊缝焊接", pRobotDriver->m_strRobotName);
				continue;
			}
			return false;
		}
		//更新数据
		//CStatisticalData* pStatisticalData = CStatisticalData::getInstance();
		//pStatisticalData->UpDateStatisticsData();
		// 抬枪
		if (!GetLocalDebugMark())
		{
			//m_nStiffenPlatenum++;
			T_ANGLE_PULSE tBackPulse = pRobotDriver->m_tHomePulse;
			if (/*nGroupNo != (nWeldGropuNum - 1)*/FALSE) // 非最后一组焊缝 不完全收枪
			{
				// 绝对安全抬枪
				//T_ANGLE_PULSE tCurPulse = pRobotDriver->GetCurrentPulse();
				//tBackPulse.nSPulse = tCurPulse.nSPulse;
				// 
				// 抬枪到工件最高点上方
				XI_POINT backPoint;
				T_ROBOT_COORS tBackCoor;
				T_ANGLE_PULSE tCurPulse = pRobotDriver->GetCurrentPulse();
				tBackPulse.nSPulse = tCurPulse.nSPulse;
				pRobotDriver->RobotKinematics(tBackPulse, pRobotDriver->m_tTools.tGunTool, tBackCoor);
				backPoint = pWeldAfterMeasure->GetPieceHeight(nGroupNo, 100.0); // 焊缝删减后变少，存在高度计算错误问题，有碰抢风险
				double dis = pUnit->GetPositionDis();
#ifdef SINGLE_ROBOT
				tBackCoor.dX = backPoint.x;
				tBackCoor.dY = backPoint.y - dis;
				tBackCoor.dZ = backPoint.z;
#else
				tBackCoor.dX = backPoint.x - dis;
				tBackCoor.dY = backPoint.y;
				tBackCoor.dZ = backPoint.z;
#endif // SINGLE_ROBOT
				pRobotDriver->RobotInverseKinematics(tBackCoor, pRobotDriver->m_tHomePulse, pRobotDriver->m_tTools.tGunTool, tBackPulse);
			}
			T_ROBOT_MOVE_INFO tRobotMoveInfo;
			vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo(0);
			T_ANGLE_PULSE tCurPulse = pRobotDriver->GetCurrentPulse();
			tBackPulse.nSPulse = tCurPulse.nSPulse;
			tRobotMoveInfo = pRobotDriver->PVarToRobotMoveInfo(1, tBackPulse, pRobotDriver->m_tBackHomeSpeed, MOVJ);
			vtRobotMoveInfo.push_back(tRobotMoveInfo);
			pRobotDriver->SetMoveValue(vtRobotMoveInfo);
			pRobotDriver->CallJob("CONTIMOVANY");
			//pRobotDriver->MoveByJob(tBackPulse, pRobotDriver->m_tPulseHighSpeed, pRobotDriver->m_nExternalAxleType, "MOVJ");
			pUnit->RobotCheckDone();
		}
		if (m_dCurWeldLenBeforeCleanGun > pWeldAfterMeasure-> m_dCleanGunDis) // 大于3米进行一次清枪
			{
				CleanGun(pRobotDriver);
				m_dCurWeldLenBeforeCleanGun = 0.0;
			}
	}
	vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfoNew(0);
	vtRobotMoveInfoNew.push_back(pRobotDriver->PVarToRobotMoveInfo(1, pRobotDriver->m_tHomePulse, pRobotDriver->m_tBackHomeSpeed, MOVJ));
	pRobotDriver->SetMoveValue(vtRobotMoveInfoNew);
	pRobotDriver->CallJob("CONTIMOVANY");
	pUnit->RobotCheckDone();
	
	//if (m_vpRobotDriver.size() > 1)  // 非单机 焊接后横移轴回零
	//{
	//	double dExSafePos = pRobotDriver->m_tHomePulse.lBYPulse * pRobotDriver->m_tExternalAxle[pUnit->m_nMeasureAxisNo - 1].dPulse;
	//	pUnit->MoveExAxisFun(dExSafePos, 6000, pUnit->m_nMeasureAxisNo);
	//	pUnit->RobotCheckDone();
	//}
	//已修改
	//SetHintInfo("XiRobot 焊接作业结束");
	SetHintInfo(XUI::Languge::GetInstance().translate("XiRobot焊接作业结束"));//"XiRobot 焊接作业结束"
	return true;
}

bool CAssemblyWeld::ScanWeldTrack(WAM::WeldAfterMeasure* pWeldAfterMeasure, LineOrCircularArcWeldingLine SeamData, int nRobotNo, int nGroupNo, double dSafeHeight)
{
	////计算扫描轨迹
	//std::vector<T_ROBOT_COORS> vtMeasureCoord;
	//vector<T_ANGLE_PULSE> vtMeasurePulse;
	//int nStartPointCount = 0;
	//int nEndPointCount = 0;
	//if (!pWeldAfterMeasure->CalcArcMoveTrackBySeamData(nRobotNo, SeamData, vtMeasureCoord, vtMeasurePulse, 5.0, nStartPointCount, nEndPointCount)
	//	|| vtMeasurePulse.size() < 3
	//	|| vtMeasureCoord.size() != vtMeasurePulse.size())
	//{
	//	XiMessageBox("计算扫描轨迹失败！");
	//	return false;
	//}

	////插入一个下枪安全位置
	//T_ROBOT_COORS tAimGunSafePos = vtMeasureCoord[0];
	//tAimGunSafePos.dZ = dSafeHeight;
	//vtMeasureCoord.insert(vtMeasureCoord.begin(), tAimGunSafePos);
	//T_ANGLE_PULSE tAimGunSafePulse;
	//if (!m_vpRobotDriver[nRobotNo]->RobotInverseKinematics(tAimGunSafePos, vtMeasurePulse[0], m_vpRobotDriver[nRobotNo]->m_tTools.tGunTool, tAimGunSafePulse))
	//{
	//	XiMessageBox("无有效安全位置坐标！");
	//	return false;
	//}
	//vtMeasurePulse.insert(vtMeasurePulse.begin(), tAimGunSafePulse);

	////插入一个抬枪安全位置
	//tAimGunSafePos = vtMeasureCoord.back();
	//tAimGunSafePos.dZ = dSafeHeight;
	//vtMeasureCoord.push_back(tAimGunSafePos);
	//if (!m_vpRobotDriver[nRobotNo]->RobotInverseKinematics(tAimGunSafePos, vtMeasurePulse.back(), m_vpRobotDriver[nRobotNo]->m_tTools.tGunTool, tAimGunSafePulse))
	//{
	//	XiMessageBox("无有效安全位置坐标！");
	//	return false;
	//}
	//vtMeasurePulse.push_back(tAimGunSafePulse);

	//std::vector<CvPoint3D64f> vtPointCloud(0);

	////开始扫描
	//BOOL nNaturalPop = false;
	//if (g_bGantryEnableMark)
	//{
	//	CLineScanForTeach cLineScanForTeach(&nNaturalPop);
	//	if (!cLineScanForTeach.LineScan(m_vnDrawComponentID[nRobotNo], this, m_vpUnit[nRobotNo], vtMeasurePulse, _T("Table"), ".\\" + m_vpRobotDriver[nRobotNo]->m_strRobotName))
	//	{
	//		cLineScanForTeach.ShowErrorInfo();
	//		return false;
	//	}
	//	if (!cLineScanForTeach.ReadPointCloud(vtPointCloud))
	//	{
	//		cLineScanForTeach.ShowErrorInfo();
	//		return false;
	//	}
	//}
	//else
	//{
	//	CLineScanForPanasonicTeach cLineScanForTeach(&nNaturalPop);
	//	if (!cLineScanForTeach.LineScan(m_vpShowLaserImgBuff[nRobotNo], m_vpUnit[nRobotNo], vtMeasurePulse, _T("Table"), ".\\" + m_vpRobotDriver[nRobotNo]->m_strRobotName))
	//	{
	//		cLineScanForTeach.ShowErrorInfo();
	//		return false;
	//	}
	//	if (!cLineScanForTeach.ReadPointCloud(vtPointCloud))
	//	{
	//		cLineScanForTeach.ShowErrorInfo();
	//		return false;
	//	}
	//}
	//Three_DPoint* input_point_cloud = (Three_DPoint*)vtPointCloud.data();
	//int input_point_cloud_size = vtPointCloud.size();

	//// 全部点云提取弧线
	//int* cloud_type = new int;
	//int* nSampPointSize = new int;
	//*cloud_type = 0;
	//*nSampPointSize = 0;
	//CvPoint3D64f tStartAngle = TranslationPVec(SeamData.StartNormalVector, -90.0);
	//CvPoint3D64f tEndAngle = TranslationPVec(SeamData.EndNormalVector, 90.0);
	//tStartAngle.z = 0;
	//tEndAngle.z = 0;
	//double dCurCarPos = 0.0; pWeldAfterMeasure->m_ptUnit->GetExPositionDis(pWeldAfterMeasure->m_ptUnit->m_nLinedScanAxisNo);

	//Three_DPoint reference_vector_1 = { SeamData.StartPoint.x + dCurCarPos, SeamData.StartPoint.y, SeamData.StartPoint.z };
	//Three_DPoint reference_normal_1 = { SeamData.StartNormalVector.x, SeamData.StartNormalVector.y, 0 };
	//Three_DPoint reference_vector_2 = { SeamData.EndPoint.x + dCurCarPos, SeamData.EndPoint.y, SeamData.EndPoint.z };
	//Three_DPoint reference_normal_2 = { SeamData.EndNormalVector.x, SeamData.EndNormalVector.y, 0 };
	//Three_DPoint reference_dir = { tStartAngle.x,tStartAngle.y,tStartAngle.z };
	//// 输出点云接口入参
	//CString sFilerefer;
	//sFilerefer.Format("%s%d_%d_GetLocalWelding2Refer.txt", pWeldAfterMeasure->m_sDataSavePath, nGroupNo, 0);
	//FILE* fp = fopen(sFilerefer.GetBuffer(0), "w");
	//fprintf(fp, "%11.3lf %11.3lf %11.3lf\n", reference_vector_1.x, reference_vector_1.y, reference_vector_1.z);
	//fprintf(fp, "%11.3lf %11.3lf %11.3lf\n", reference_normal_1.x, reference_normal_1.y, reference_normal_1.z);
	//fprintf(fp, "%11.3lf %11.3lf %11.3lf\n", reference_vector_2.x, reference_vector_2.y, reference_vector_2.z);
	//fprintf(fp, "%11.3lf %11.3lf %11.3lf\n", reference_normal_2.x, reference_normal_2.y, reference_normal_2.z);
	//fprintf(fp, "%11.3lf %11.3lf %11.3lf\n", reference_dir.x, reference_dir.y, reference_dir.z);
	//fclose(fp);
	////Three_DPoint* CoutCloud = GetLocalWelding2(input_point_cloud, input_point_cloud_size - 1000, nSampPointSize, false,
	////	reference_vector_1,
	////	reference_normal_1,
	////	reference_vector_2,
	////	reference_normal_2,
	////	reference_dir,
	////	2.0,
	////	cloud_type);
	////// -----------

	//if (*cloud_type != 2 || *nSampPointSize < 2)
	//{
	//	XiMessageBox("提取圆弧数据数据");
	//	return false;
	//}

	//// 根据点云输出数据平滑处理生成最终焊接轨迹
	//vector<XI_POINT> vtWeldTrack;
	//vector<T_ROBOT_COORS> vtWeldTrackRobot;
	//vtWeldTrack.clear();
	//CString sFile;
	//sFile.Format("%s%d_%d_FilterBeforpoints.txt", pWeldAfterMeasure->m_sDataSavePath, nGroupNo, 0);
	//for (size_t i = 0; i < *nSampPointSize; i++)
	//{
	//	XI_POINT tp = { CoutCloud[i].x,CoutCloud[i].y,CoutCloud[i].z };
	//	vtWeldTrack.push_back(tp);
	//}
	//SavePointsData(vtWeldTrack, sFile);
	//// Y样条滤波
	//pWeldAfterMeasure->SplineFiltering(vtMeasureCoord.back().dBY, vtWeldTrack, vtWeldTrackRobot);
	//// 计算最终轨迹
	//pWeldAfterMeasure->CalcRealWeldTrack(pWeldAfterMeasure->m_vvtWeldLineInfoGroup[nGroupNo][0], vtWeldTrackRobot);

	//// 保存 序号 直角坐标 外部轴坐标 焊接类型
	//CString sFileName;
	//sFileName.Format("%s%d_%d_RealWeldCoord.txt", pWeldAfterMeasure->m_sDataSavePath, nGroupNo, 2);
	//FILE* pf = fopen(sFileName, "w");
	//for (int nPtnIdx = 0; nPtnIdx < vtWeldTrackRobot.size(); nPtnIdx++)
	//{
	//	T_ROBOT_COORS tCoord = vtWeldTrackRobot[nPtnIdx];
	//	fprintf(pf, "%d %11.3lf %11.3lf %11.3lf %11.3lf %11.3lf %11.3lf %11.3lf %4d %4d\n", nPtnIdx,
	//		tCoord.dX, tCoord.dY, tCoord.dZ, tCoord.dRX, tCoord.dRY, tCoord.dRZ, vtWeldTrackRobot[nPtnIdx].dBY, E_FLAT_SEAM, E_WELD_TRACK);
	//}
	//fclose(pf);

	//for (size_t i = 0; i < 2; i++)
	//{
	//	int nWeldInfoNum = 0;
	//	CvPoint3D32f tRefPtn[2] = { 0.0 };
	//	tRefPtn[0] = Xi_P2P<XI_POINT, CvPoint3D32f>(vtWeldTrack[0]);
	//	double dDirAngle = pWeldAfterMeasure->m_ptUnit->GetRobotCtrl()->RzToDirAngle(vtMeasureCoord[0].dRZ);
	//	if (i > 0)
	//	{
	//		tRefPtn[0] = Xi_P2P<XI_POINT, CvPoint3D32f>(vtWeldTrack.back());
	//		dDirAngle = pWeldAfterMeasure->m_ptUnit->GetRobotCtrl()->RzToDirAngle(vtMeasureCoord.back().dRZ);
	//	}
	//	tRefPtn[1] = tRefPtn[0];
	//	tRefPtn[1].z -= 50.0;

	//	CvPoint3D32f tCameraNorm = cvPoint3D32f(-CosD(dDirAngle), -SinD(dDirAngle), pWeldAfterMeasure->m_ptUnit->GetRobotCtrl()->m_nRobotInstallDir > 0.0 ? -1.0 : 1.0);
	//	CvPoint3D32f tPlaneHNorm = cvPoint3D32f(0.0, 0.0, pWeldAfterMeasure->m_ptUnit->GetRobotCtrl()->m_nRobotInstallDir > 0.0 ? 1.0 : -1.0);
	//	RiserEndPointInfo tWeldInfo[3] = { 0 };
	//	CString referenceInfo;
	//	referenceInfo.Format(".\\RobotA\\LineScan\\PointCloud\\%d_Inputvertical.txt", i);
	//	FILE* pfPointCloudInput = fopen(referenceInfo.GetBuffer(0), "w");
	//	// 保存每次输入信息及处理结果
	//	fprintf(pfPointCloudInput, "tCameraNorm:%11.3lf%11.3lf%11.3lf tPlaneHNorm:%11.3lf%11.3lf%11.3lf tRefPtn[0]:%11.3lf%11.3lf%11.3lf tRefPtn[1]:%11.3lf%11.3lf%11.3lf\n",
	//		tCameraNorm.x, tCameraNorm.y, tCameraNorm.z, tPlaneHNorm.x, tPlaneHNorm.y, tPlaneHNorm.z,
	//		tRefPtn[0].x, tRefPtn[0].y, tRefPtn[0].z, tRefPtn[1].x, tRefPtn[1].y, tRefPtn[1].z);
	//	fclose(pfPointCloudInput);

	//	if (i > 0)
	//	{
	//		//nWeldInfoNum = GetRiserEndPoint((CvPoint3D64f*)iRight_point_cloud, iRight_point_cloud_size, tWeldInfo, tCameraNorm, tPlaneHNorm, tRefPtn, 2, "GetRiserEndPoint_Vertical", false);
	//		nWeldInfoNum = GetRiserEndPoint((CvPoint3D64f*)input_point_cloud, input_point_cloud_size - 1000, tWeldInfo, tCameraNorm, tPlaneHNorm, tRefPtn, 2, "GetRiserEndPoint_Vertical");
	//	}
	//	else 
	//	{
	//		//nWeldInfoNum = GetRiserEndPoint((CvPoint3D64f*)iLeft_point_cloud, iLeft_point_cloud_size, tWeldInfo, tCameraNorm, tPlaneHNorm, tRefPtn, 2, "GetRiserEndPoint_Vertical", false);
	//		nWeldInfoNum = GetRiserEndPoint((CvPoint3D64f*)input_point_cloud, input_point_cloud_size - 1000, tWeldInfo, tCameraNorm, tPlaneHNorm, tRefPtn, 2, "GetRiserEndPoint_Vertical");

	//	}
	//	if (nWeldInfoNum == 0)
	//	{
	//		XiMessageBox("提取圆弧立缝数据失败！");
	//		return false;
	//	}

	//	//jwq临时，求出理想起点坐标
	//	double dDis = TwoPointDis(tWeldInfo[0].staPnt.x, tWeldInfo[0].staPnt.y, tWeldInfo[0].staPnt.z, tWeldInfo[0].endPnt.x, tWeldInfo[0].endPnt.y, tWeldInfo[0].endPnt.z);
	//	double dDisX = (tWeldInfo[0].endPnt.x - tWeldInfo[0].staPnt.x) / dDis;
	//	double dDisY = (tWeldInfo[0].endPnt.y - tWeldInfo[0].staPnt.y) / dDis;
	//	double dDisZ = (tWeldInfo[0].endPnt.z - tWeldInfo[0].staPnt.z) / dDis;
	//	T_LINE_PARA tPointDir;
	//	tPointDir.dDirX = dDisX;
	//	tPointDir.dDirY = dDisY;
	//	tPointDir.dDirZ = dDisZ;
	//	tPointDir.dPointX = tWeldInfo[0].staPnt.x;
	//	tPointDir.dPointY = tWeldInfo[0].staPnt.y;
	//	tPointDir.dPointZ = tWeldInfo[0].staPnt.z;
	//	T_ALGORITHM_POINT tpoint;
	//	XiBase::CXiOpenFile file;
	//	if (i == 0)
	//	{
	//		file.XI_fopen("ceshi_dVerdir0.txt", "w");
	//		tpoint = { vtWeldTrack[0].x, vtWeldTrack[0].y, vtWeldTrack[0].z };
	//	}
	//	else
	//	{
	//		file.XI_fopen("ceshi_dVerdir1.txt", "w");
	//		tpoint = { vtWeldTrack.back().x, vtWeldTrack.back().y, vtWeldTrack.back().z };
	//	}
	//	T_ALGORITHM_POINT tpointProjection;
	//	if (!PointtoLineProjection(tPointDir, tpoint, tpointProjection))
	//	{
	//		return false;
	//	}

	//	for (size_t j = 0; j < 20; j++)
	//	{
	//		file.XI_fprintf("%d %11.3lf %11.3lf %11.3lf\n", j,
	//			tWeldInfo->normal.x * j + tpointProjection.dCoorX,
	//			tWeldInfo->normal.y * j + tpointProjection.dCoorY,
	//			tWeldInfo->normal.z * j + tpointProjection.dCoorZ);
	//	}
	//	file.XI_fclose();

	//	//jwq临时，改为固定长度
	//	double dRealDis = 17.0;
	//	int nNo = (int)(dRealDis / 2.0 + 0.5);
	//	double dStepDis = dRealDis / (double)nNo;
	//	double dStepDisX = dDisX * dStepDis;
	//	double dStepDisY = dDisY * dStepDis;
	//	double dStepDisZ = dDisZ * dStepDis;
	//	XI_POINT tp;

	//	vector<XI_POINT> vtVertical;
	//	for (int j = 0; j <= nNo; j++)
	//	{
	//		tp.x = tpointProjection.dCoorX + dStepDisX * j + 5.0 * dDisX;
	//		tp.y = tpointProjection.dCoorY + dStepDisY * j + 5.0 * dDisY;
	//		tp.z = tpointProjection.dCoorZ + dStepDisZ * j + 5.0 * dDisZ;
	//		vtVertical.push_back(tp);
	//	}
	//	if (vtVertical.size() < 21)
	//	{
	//		for (size_t j = vtVertical.size(); j <= 21; j++)
	//		{
	//			vtVertical.push_back(vtVertical.back());
	//		}
	//	}
	//	CString vertical;
	//	vertical.Format(".\\RobotA\\LineScan\\PointCloud\\vertical_%d.txt", i);
	//	SavePointsData(vtVertical, vertical);
	//	// 生成立峰焊接轨迹
	//	double dVerdir = atan2(tWeldInfo->normal.y, tWeldInfo->normal.x) * 180.0 / PI;
	//	
	//	//double dVerdir = atan2(SeamData.StartNormalVector.y, SeamData.StartNormalVector.x) * 180.0 / PI;
	//	//dVerdir -= 45.0;
	//	//if (i > 0)
	//	//{
	//	//	dVerdir = atan2(SeamData.EndNormalVector.y, SeamData.EndNormalVector.x) * 180.0 / PI;
	//	//	dVerdir += 45.0;
	//	//}
	//	sFileName.Format("%s%d_%d_RealWeldCoord.txt", pWeldAfterMeasure->m_sDataSavePath, nGroupNo, i);
	//	pf = fopen(sFileName, "w");
	//	T_ROBOT_COORS tCoord = vtWeldTrackRobot.back();
	//	double dStartAngle = -55.0;
	//	double dEndAngle = -55.0;
	//	double dRYAngleStep = (dEndAngle - dStartAngle) / (nNo * 1.0);
	//	for (size_t j = 0; j < vtVertical.size(); j++)
	//	{
	//		T_ROBOT_COORS tRobotCoord = tCoord;
	//		tRobotCoord.dX = vtVertical[j].x;
	//		tRobotCoord.dY = vtVertical[j].y - tCoord.dBY;
	//		tRobotCoord.dZ = vtVertical[j].z;
	//		tRobotCoord.dRZ = pWeldAfterMeasure->m_ptUnit->GetRobotCtrl()->DirAngleToRz(dVerdir);
	//		tRobotCoord.dBY = tCoord.dBY;
	//		if (j < nNo)
	//		{
	//			tRobotCoord.dRY = dRYAngleStep * j + dStartAngle;
	//		}
	//		else
	//		{
	//			tRobotCoord.dRY = dEndAngle;
	//		}

	//		fprintf(pf, "%zd %11.3lf %11.3lf %11.3lf %11.3lf %11.3lf %11.3lf %11.3lf %4d %4d\n", j,
	//			tRobotCoord.dX, tRobotCoord.dY, tRobotCoord.dZ, tRobotCoord.dRX, tRobotCoord.dRY, tRobotCoord.dRZ, tRobotCoord.dBY, E_STAND_SEAM, E_WELD_TRACK);
	//	}
	//	fclose(pf);
	//}
	return true;
}

void CAssemblyWeld::GetCameraTool(int nRobotNo, int nCameraNo, T_ROBOT_COORS& tCameraTool)
{
	CUnit* pUnit = m_vpUnit[nRobotNo];
	tCameraTool = pUnit->GetCameraTool(nCameraNo);
	WriteLog("%d号机械臂激光相机工具：%lf,%lf,%lf,%lf,%lf,%lf", 
		nRobotNo, tCameraTool.dX, tCameraTool.dY, tCameraTool.dZ, tCameraTool.dRX, tCameraTool.dRY, tCameraTool.dRZ);
	m_vpRobotDriver[nRobotNo]->SetPosVar(99, tCameraTool);
}

void CAssemblyWeld::GetRecogCameraTool(int nRobotNo, T_ROBOT_COORS& tRecogCameraTool)
{

}

void CAssemblyWeld::ShowMoveState(CUnit* pUnit)
{
	CRobotDriverAdaptor* pRobotDriver = pUnit->GetRobotCtrl();
	//显示机械臂当前位置
	T_ROBOT_COORS tCurCoord = pRobotDriver->GetCurrentPos();

    CString strCoorX;
    strCoorX.Format("%.3f", tCurCoord.dX);
	SetDlgItemData(IDC_EDIT_COOR_X, strCoorX, this, m_bIfWindowOn);

    CString strCoorY;
    strCoorY.Format("%.3f", tCurCoord.dY);
	SetDlgItemData(IDC_EDIT_COOR_Y, strCoorY, this, m_bIfWindowOn);

    CString strCoorZ;
    strCoorZ.Format("%.3f", tCurCoord.dZ);
	SetDlgItemData(IDC_EDIT_COOR_Z, strCoorZ, this, m_bIfWindowOn);

    CString strCoorRX;
    strCoorRX.Format("%.4f", tCurCoord.dRX);
	SetDlgItemData(IDC_EDIT_COOR_RX, strCoorRX, this, m_bIfWindowOn);

    CString strCoorRY;
    strCoorRY.Format("%.4f", tCurCoord.dRY);
	SetDlgItemData(IDC_EDIT_COOR_RY, strCoorRY, this, m_bIfWindowOn);

    CString strCoorRZ;
    strCoorRZ.Format("%.4f", tCurCoord.dRZ);
	SetDlgItemData(IDC_EDIT_COOR_RZ, strCoorRZ, this, m_bIfWindowOn);

	CString strAxis_X;
	strAxis_X.Format("%.4f", tCurCoord.dBX * pRobotDriver->GetPanasonicExDir(E_EX_X));
	SetDlgItemData(IDC_EDIT_EXTERNAL_X, strAxis_X, this, m_bIfWindowOn);

	CString strAxis_Y;
	strAxis_Y.Format("%.4f", tCurCoord.dBY * pRobotDriver->GetPanasonicExDir(E_EX_Y));
	SetDlgItemData(IDC_EDIT_EXTERNAL_Y, strAxis_Y, this, m_bIfWindowOn);

	CString strAxis_Z;
	strAxis_Z.Format("%.4f", tCurCoord.dBZ * pRobotDriver->GetPanasonicExDir(E_EX_Z));
	SetDlgItemData(IDC_EDIT_EXTERNAL_Z, strAxis_Z, this, m_bIfWindowOn);

//	// 焊丝检测
// 	if (m_cIOControl->ReadInbitNEW(202) == TRUE&&HSJC == false)
// 	{
// 		XiMessageBox("焊丝已经用完");
// 		OnBtnStop(pCuttingRobot);
// 		HSJC = true;
// 		return;
// 	}
}

void CAssemblyWeld::SaveLastCtrlState()
{
	unsigned long long ullCtrlState = 0;
	for (int i = 0; i < m_vnCtrlID.size(); i++)
	{
		if (TRUE == GetDlgItem(m_vnCtrlID[i])->IsWindowEnabled())
		{
			unsigned long long ullCurCtrlState = 1;
			ullCurCtrlState = ullCurCtrlState << (63 - i);
			ullCtrlState += ullCurCtrlState;
		}
	}
	CString strCtrlState;
	if (!ULongLongToCString(strCtrlState, ullCtrlState))
	{
		return;
	}
	COPini opini;
	opini.SetFileName(OPTIONAL_FUNCTION);
	opini.SetSectionName("LastState");
	opini.WriteString("LastState", strCtrlState);
}

bool CAssemblyWeld::LoadLastCtrlState()
{
	unsigned long long ullCtrlState = 0;
	CString strCtrlState;
	COPini opini;
	opini.SetFileName(OPTIONAL_FUNCTION);
	opini.SetSectionName("LastState");
	opini.ReadString("LastState", strCtrlState);
	if (!CStringToULongLong(ullCtrlState, strCtrlState))
	{
		XUI::MesBox::PopError("Error:{0}文件中出现错误的LastState,这是不被允许的行为，请勿擅自更改LastState！",
			(const char*)OPTIONAL_FUNCTION);
		return false;
	}
	//SetCtrlState(ullCtrlState);
	return true;
}

void CAssemblyWeld::LoadTableGroupScan()
{
	int nRobotNo = 0;
	CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];

	int nTotalTableNumber = 0;
	int nCurTableNo = 0;
	CString strKey;
	CString sFileName = DATA_PATH + pRobotDriver->m_strRobotName + LINE_SCAN_PARAM;
	//加载数据
	COPini opini;
	opini.SetFileName(sFileName);
	opini.SetSectionName("CurUseTableNo");
	opini.ReadString("CurUseTableNo", &nCurTableNo);

	strKey.Format("Table%d", nCurTableNo);
	while (opini.CheckExists(sFileName, strKey, "startpulse.nS"))
	{
		nTotalTableNumber++;
		strKey.Format("Table%d", nTotalTableNumber);
	}
	if (nTotalTableNumber < 1)
	{
		XiMessageBox("料台信息错误，请检查");
	}
	
	//刷新界面
	m_comboTableGroupleft.ResetContent();
	for (int i = 0; i < nTotalTableNumber; i++)
	{
		CString str;
		str.Format("%d号料台", i);
		//已修改
		str = XUI::Languge::GetInstance().translate(str.GetBuffer());
		m_comboTableGroupleft.AddString(str);
	}

	m_comboTableGroupleft.SetCurSel(nCurTableNo);
	LoadRobotandCar(pRobotDriver, nCurTableNo);
}

void CAssemblyWeld::LoadPartType()
{
	m_ctlWorkpieceType.ResetContent();
	int nSetNo = 0;
	int nCurChooseType = 0;
	int nTotalTypeNum = 0;
	CString str;
	CString sTypeName;
	int nTypeNo = 0;
	COPini opini;
	opini.SetFileName(SYSTEM_PARA_INI);
	opini.SetSectionName("TotalPartType");
	opini.ReadString("CurChooseType", &nCurChooseType);
	opini.ReadString("TotalNumber", &nTotalTypeNum);
	for (int i = 0; i < nTotalTypeNum; i++)
	{
		str.Format("Name%d", i);
		opini.ReadString(str, sTypeName);
		//sTypeName = Utf8ToGBK(sTypeName);
		str.Format("Number%d", i);
		opini.ReadString(str, &nTypeNo);
		m_nsPartType[nTypeNo] = sTypeName;
		//已修改
		sTypeName = XUI::Languge::GetInstance().translate(sTypeName.GetBuffer());
		m_ctlWorkpieceType.AddString(sTypeName);
		if (nTypeNo == nCurChooseType)
		{
			nSetNo = i;
		}
	}
	if (m_nsPartType.size() != nTotalTypeNum)
	{
		XiMessageBox("工件类型加载失败！");
	}

	m_ctlWorkpieceType.SetCurSel(nSetNo);
	m_tChoseWorkPieceType = (E_WORKPIECE_TYPE)nCurChooseType;
	//已修改
	XUI::Languge::GetInstance().translateDialog(this);
}

void CAssemblyWeld::LoadDebugPara()
{
	COPini opini;
	opini.SetFileName(DEBUG_INI);
	opini.SetSectionName("Debug");
	opini.ReadString("ShowAllButton", &m_bShowAllButton);
}

void CAssemblyWeld::LoadRobotandCar(CRobotDriverAdaptor* pRobotCtrl, int tablenum)
{
	
	//初始化大车极限距离，机械臂极限距离，大车最小速度，大车加减速时间	 
//	double dXScanSize;
//	double dZScanSize;
//	double RXScanSize;
//	double RYScanSize;
//	double RZScanSize;
//	CString table;
//	table.Format("Table%d", tablenum);
//	COPini opini;
//	opini.SetFileName(DATA_PATH+m_vpRobotDriver[0]->m_strRobotName + LINE_SCAN_PARAM);
//	opini.SetSectionName(table);
//	opini.ReadString("YMaxCar", &pRobotCtrl->m_tRobotLimitation.drYMaxCar);
//	opini.ReadString("YMinCar", &pRobotCtrl->m_tRobotLimitation.drYMinCar);
//	opini.ReadString("YMaxRobot", &pRobotCtrl->m_tRobotLimitation.drYMaxRobot);
//	opini.ReadString("YMinRobot", &pRobotCtrl->m_tRobotLimitation.drYMinRobot);
//	opini.ReadString("XMax", &pRobotCtrl->m_tRobotLimitation.drXMax);
//	opini.ReadString("XMin", &pRobotCtrl->m_tRobotLimitation.drXMin);
//	opini.ReadString("ZMax", &pRobotCtrl->m_tRobotLimitation.drZMax);
//	opini.ReadString("ZMin", &pRobotCtrl->m_tRobotLimitation.drZMin);
//	opini.ReadString("dMinVel", &pRobotCtrl->m_tRobotLimitation.drdMinVel);
//	opini.ReadString("dAcc", &pRobotCtrl->m_tRobotLimitation.drdAcc);
//	opini.ReadString("dDec", &pRobotCtrl->m_tRobotLimitation.drdDec);
//	opini.ReadString("ScanSpeed", &pRobotCtrl->m_tRobotLimitation.drScanSpeed);
//	opini.ReadString("RunSpeed", &pRobotCtrl->m_tRobotLimitation.drRunSpeed);
//	opini.ReadString("TableZ", &pRobotCtrl->m_tRobotLimitation.drTableZ);
//	opini.ReadString("TableY", &pRobotCtrl->m_tRobotLimitation.drTableY);
//
//	opini.ReadString("startpulse.nS", &pRobotCtrl->t_StartPlus.nSPulse);
//	opini.ReadString("startpulse.nL", &pRobotCtrl->t_StartPlus.nLPulse);
//	opini.ReadString("startpulse.nU", &pRobotCtrl->t_StartPlus.nUPulse);
//	opini.ReadString("startpulse.nR", &pRobotCtrl->t_StartPlus.nRPulse);
//	opini.ReadString("startpulse.nB", &pRobotCtrl->t_StartPlus.nBPulse);
//	opini.ReadString("startpulse.nT", &pRobotCtrl->t_StartPlus.nTPulse);
//	opini.ReadString("ScanStartCarLoction", &pRobotCtrl->ScanStartCarLoction);
//	opini.ReadString("Scanlength", &pRobotCtrl->m_dScanLength);
//	opini.ReadString("XScanSize", &dXScanSize);
//	opini.ReadString("ZScanSize", &dZScanSize);
//	opini.ReadString("RXScanSize", &RXScanSize);
//	opini.ReadString("RYScanSize", &RYScanSize);
//	opini.ReadString("RZScanSize", &RZScanSize);
//
//	pRobotCtrl->RobotKinematics(pRobotCtrl->t_StartPlus, pRobotCtrl->m_tTools.tGunTool, pRobotCtrl->startWorldCoors);
//	// 线扫位置和姿态 补偿 临时
//	if (1 == pRobotCtrl->m_nRobotInstallDir)
//	{
//#ifdef SINGLE_ROBOT
//		pRobotCtrl->startWorldCoors.dX += dXScanSize;
//		pRobotCtrl->startWorldCoors.dZ += dZScanSize;
//		pRobotCtrl->startWorldCoors.dRX += RXScanSize;
//		pRobotCtrl->startWorldCoors.dRY += RYScanSize;
//		pRobotCtrl->startWorldCoors.dRZ += RZScanSize;
//#else
//		pRobotCtrl->startWorldCoors.dY += dXScanSize;
//		pRobotCtrl->startWorldCoors.dZ += dZScanSize;
//		pRobotCtrl->startWorldCoors.dRX += RXScanSize;
//		pRobotCtrl->startWorldCoors.dRY += RYScanSize;
//		pRobotCtrl->startWorldCoors.dRZ += RZScanSize;
//#endif // SINGLE_ROBOT
//	}
//
//	bool bRst = pRobotCtrl->RobotInverseKinematics(pRobotCtrl->startWorldCoors, pRobotCtrl->t_StartPlus, pRobotCtrl->m_tTools.tGunTool, pRobotCtrl->t_StartPlus);
//	if (false == bRst)
//	{
//		XiMessageBox("线扫初始扫描坐标修改失败");
//	}
//
//	pRobotCtrl->startWorldCoors.dY += pRobotCtrl->ScanStartCarLoction;
//	pRobotCtrl->endWorldCoors = pRobotCtrl->startWorldCoors;
//	pRobotCtrl->endWorldCoors.dY += pRobotCtrl->m_dScanLength;
//
//
//	//加载线扫点云处理范围
//	opini.ReadString("Range_XMax", &(m_vpLaserLineScreen[0]->m_dRange_XMax));
//	opini.ReadString("Range_XMin", &(m_vpLaserLineScreen[0]->m_dRange_XMin));
//	opini.ReadString("Range_YMax", &(m_vpLaserLineScreen[0]->m_dRange_YMax));
//	opini.ReadString("Range_YMin", &(m_vpLaserLineScreen[0]->m_dRange_YMin));
//	opini.ReadString("Range_ZMax", &(m_vpLaserLineScreen[0]->m_dRange_ZMax));
//	opini.ReadString("Range_ZMin", &(m_vpLaserLineScreen[0]->m_dRange_ZMin));

	//m_cLeftCleanGunIOQQ = 209;
	//m_cLeftCleanGunIOJS = 210;
	//m_cLeftCleanGunIOJJ = 200;
}

BOOL CAssemblyWeld::InitAllUnit()
{
	BOOL bRtn = TRUE;
	COPini opini;
	bRtn = bRtn && opini.SetFileName(CONTRAL_UNIT_INFO_INI);
	bRtn = bRtn && opini.SetSectionName("UnitNum");
	int nNum = 0;
	bRtn = bRtn && opini.ReadString("UnitNum", &nNum);
	for (size_t i = 0; i < nNum; i++)
	{
		T_CONTRAL_UNIT tContralUnitInfo;
		tContralUnitInfo.nUnitNo = i;
		bRtn = bRtn && opini.SetSectionName("UnitName");
		bRtn = bRtn && opini.ReadString(GetStr("Unit%d", i), tContralUnitInfo.strUnitName);
		bRtn = bRtn && opini.SetSectionName("ChineseName");
		bRtn = bRtn && opini.ReadString(GetStr("Unit%d", i), tContralUnitInfo.strChineseName);
		bRtn = bRtn && opini.SetSectionName("ContralType");
		bRtn = bRtn && opini.ReadString(GetStr("Unit%d", i), tContralUnitInfo.strUnitType);
		bRtn = bRtn && opini.SetSectionName("UnitType");
		bRtn = bRtn && opini.ReadString(GetStr("Unit%d", i), &tContralUnitInfo.nContralUnitType);
		CString strFilePath = DATA_PATH + tContralUnitInfo.strUnitName;
		CheckFolder(strFilePath);
		m_vtContralUnitInfo.push_back(tContralUnitInfo);

		switch (tContralUnitInfo.nContralUnitType)
		{
		case 1:
		{
			break;
		}
		case 2:
		{
			break;
		}
		case 3:
		{
			//m_pGeneralControl = new CGeneralControl(tContralUnitInfo, m_pCtrlCardDriver);
			break;
		}
		case 4:
		{
			//CCompositionTechniqueRobot* pCompositionTechniqueRobot;
			//pCompositionTechniqueRobot = new CCompositionTechniqueRobot(tContralUnitInfo, m_pCtrlCardDriver);
			//pCompositionTechniqueRobot->m_nSortingRobotNo = m_vpCompositionTechniqueRobot.size();
			//m_vpCompositionTechniqueRobot.push_back(pCompositionTechniqueRobot);
			break;
		}
		case 5:
		{
			//CChamferRobot* pChamferRobot;
			//pChamferRobot = new CChamferRobot(tContralUnitInfo, m_pCtrlCardDriver);
			//pChamferRobot->m_nChamferRobotNo = m_vpChamferRobot.size();
			//m_vpChamferRobot.push_back(pChamferRobot);
			break;
		}
		case 6:
		{
			CUnit* pUnit;
			pUnit = new CUnit(tContralUnitInfo, m_pServoMotorDriver);
			m_vpUnit.push_back(pUnit);
			break;
		}
		default:
			break;
		}
	}

	m_vpRobotDriver.clear();
	m_vtRobotThread.clear();
	m_vpLaserLineScreen.clear();
	m_vpShowLaserImgBuff.clear();
	for (int nUnitNo = 0; nUnitNo < m_vpUnit.size(); nUnitNo++)
	{
		CUnit* pUnit = m_vpUnit[nUnitNo];
		pUnit->GetRobotCtrl()->m_nRobotNo = nUnitNo;
		m_vpRobotDriver.push_back(pUnit->GetRobotCtrl());
		T_ROBOT_THREAD* tRobotTheard = new T_ROBOT_THREAD;
		tRobotTheard->pRobotCtrl = pUnit->GetRobotCtrl();
		tRobotTheard->cIncisePlanePart = this;
		tRobotTheard->bRobotThreadStatus = false;
		m_vtRobotThread.push_back(tRobotTheard);
		if (-1 != pUnit->m_nLineScanCameraNo)
		{
			CLaserLineScreen* pLaserLineScreen = new CLaserLineScreen(pUnit, &m_bNaturalPop);
			m_vpLaserLineScreen.push_back(pLaserLineScreen);
		}				
		IplImage* pImg = cvCreateImage(cvSize(
			pUnit->GetCameraParam(1).tDHCameraDriverPara.nRoiWidth,
			pUnit->GetCameraParam(1).tDHCameraDriverPara.nRoiHeight), IPL_DEPTH_8U, 3);
		m_vpShowLaserImgBuff.push_back(pImg);
		CScanInitModule *m_pScanInit = new CScanInitModule(m_vpRobotDriver[nUnitNo],m_vpUnit[nUnitNo]);
		m_vpScanInit.push_back(m_pScanInit);
		if (!GetLocalDebugMark())
		{
		m_vpRobotDriver[nUnitNo]->ServoOn();
		m_vpRobotDriver[nUnitNo]->HoldOff();
		}
	}	

	//if (!GetLocalDebugMark())
	//{
	//	WORD wState = OFF;
	//	m_pGeneralControl->GetAllEmgStopState(wState);
	//	if (wState == ON)
	//	{
	//		m_pGeneralControl->SetAbsEncoderCoor();
	//	}
	//	else
	//	{
	//		MESSAGE_BOX("设备已急停，当前绝对位置设置失败");
	//	}
	//}
	//GetRobotTransMatrix(m_dAToBTransMatrix, m_dBToATransMatrix);

	return bRtn;
}

void CAssemblyWeld::OnBnClickedButtonPanoRecognition()
{	
	WriteLog("单击：模板匹配识别");
	if (true == m_bAutoWeldWorking)
	{
		XiMessageBoxOk("正在工作中!");
		return;
	}
	//AfxBeginThread(ThreadGrooveWeld, this);
	int nRobotNo = 0;
	CUnit* pUnit = m_vpUnit[nRobotNo];
	CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
	CHECK_BOOL(CreateObject(m_tChoseWorkPieceType, pUnit, &m_pWeldAfterMeasure));
	if (1 == XiMessageBox("确定:创建模板信息  取消:焊缝模板匹配"))
	{
		m_pWeldAfterMeasure->GenerateTemplate();
	}
	else
	{
		m_pWeldAfterMeasure->TemplateMatch();
	}

	return;
}

void CAssemblyWeld::OnBnClickedButtonRobotCtrl()
{
	WriteLog("单击：机器人控制");
	CRobotCtrlDlg *pRobotCtrl = NULL;
	pRobotCtrl = new CRobotCtrlDlg(m_vpRobotDriver);
	pRobotCtrl->DoModal();
	DELETE_POINTER(pRobotCtrl);
}

void CAssemblyWeld::OnBnClickedButtonAntifeeding()
{
    CRobotDriverAdaptor *pRobotCtrl;
    if (XUI::MesBox::PopOkCancel("左机器人运行？"))
    {
        pRobotCtrl = m_vpRobotDriver[0];
    }
    else if (XUI::MesBox::PopOkCancel("右机器人运行？"))
    {
        pRobotCtrl = m_vpRobotDriver[1];
    }
    else
    {
		XUI::MesBox::PopInfo("未选择机器人");
        return;
    }
    T_ROBOT_COORS tRobotCurCoord;
    static int i = 0;
    CString strName;
    strName.Format(".\\MeasureData\\%s\\RealTeachPoint.txt", pRobotCtrl->m_strRobotName);
    FILE *CoutTeachPoint = fopen(strName.GetBuffer(0), "a+");
    tRobotCurCoord.dX = pRobotCtrl->GetCurrentPos(ROBOT_AXIS_X);
    tRobotCurCoord.dY = pRobotCtrl->GetCurrentPos(ROBOT_AXIS_Y);
    tRobotCurCoord.dZ = pRobotCtrl->GetCurrentPos(ROBOT_AXIS_Z);
    fprintf(CoutTeachPoint, "%d %lf %lf %lf\n", i, tRobotCurCoord.dX, tRobotCurCoord.dY, tRobotCurCoord.dZ);
    fclose(CoutTeachPoint);
    i++;
	XUI::MesBox::PopInfo("第：{0} 次采集位置完成", i);
    return;
}

void CAssemblyWeld::OnBnClickedButtonStart()
{
	WriteLog("单击：自动焊接"); 
	//OutputDebugString("Hello!!!");
	if (true == m_bAutoWeldWorking)
	{
		XiMessageBoxOk("正在工作中，不能重复开启!");
		return;
	}
	if (!RunPara::GetInstance().load())
	{
		XUI::MesBox::PopError("读取运行参数失败！");
		return;
	}
	WinExec("del_pic.bat", SW_SHOW);
	for (int i = 0; i < m_vpUnit.size(); i++)
	{
		CHECK_BOOL(m_vpUnit[i]->CheckIsReadyRun());
		//break;
	}
	E_WORKPIECE_TYPE tPartType = m_tChoseWorkPieceType; 

	for (int i = 0; i < m_vpUnit.size(); i++)
	{
	 m_vpUnit[i]->m_bBreakPointContinue = false;
	 m_vpUnit[i]->GetRobotCtrl()->m_eThreadStatus = INCISEHEAD_THREAD_STATUS_START;
	}
	AfxBeginThread(ThreadWeldAfterMeasure, this);

	AfxBeginThread(ThreadCheckWeldIO, this);
	return;
}

bool CAssemblyWeld::LoadGroupingResult(CString sFileName/* = ""*/)
{
	CString sOutFileName = "GraphData\\PointCloudIdentifyReaultAfter.txt";
	if ("" != sFileName)
	{
		sOutFileName = sFileName;
	}
	FILE* pf = fopen(sOutFileName.GetBuffer(), "r");
	if (NULL == pf)
	{
		XUI::MesBox::PopOkCancel("加载点云处理结果文件 {0} 打开失败", sOutFileName);
		return false;
	}

	std::vector<LineOrCircularArcWeldingLine> vtWeldSeamData; // 点云处理得到的焊缝信息
	std::vector<WeldLineInfo> vtWeldSeamInfo; // 包括识别结果及焊脚等无法识别的属性信息
	m_vvvtWeldSeamData.clear();
	m_vvvtWeldSeamInfo.clear();
	m_vvvtWeldSeamData.resize(m_vpRobotDriver.size());
	m_vvvtWeldSeamInfo.resize(m_vpRobotDriver.size());
	//m_vtWeldSeamData.clear();
	//m_vtWeldSeamInfo.clear();

	std::vector < std::vector<LineOrCircularArcWeldingLine>> vvtWeldSeamData1; // 多机点云处理得到的焊缝信息
	std::vector < std::vector<WeldLineInfo>> vvtWeldSeamInfo1; // 多机包括识别结果及焊脚等无法识别的属性信息

	int nIdx = 0;
	bool m = 0;
	LineOrCircularArcWeldingLine tWeld;
	WeldLineInfo tWeldInfo;
	while (EOF != fscanf(pf, "%d%d%d%lf %d %lf%lf%lf %lf%lf%lf %lf%lf%lf %lf%lf%lf %lf%lf%lf %d%d %d%d%d %lf%lf%lf%d%lf%d%d%lf",
		&nIdx, &tWeld.IsArc, (int*)(void*)&tWeld.isClockwise, &tWeld.ZSide, (int*)(void*)&tWeld.isLeft,
		&tWeld.CenterPoint.x, &tWeld.CenterPoint.y, &tWeld.CenterPoint.z,
		&tWeld.StartPoint.x, &tWeld.StartPoint.y, &tWeld.StartPoint.z,
		&tWeld.EndPoint.x, &tWeld.EndPoint.y, &tWeld.EndPoint.z,
		&tWeld.StartNormalVector.x, &tWeld.StartNormalVector.y, &tWeld.StartNormalVector.z,
		&tWeld.EndNormalVector.x, &tWeld.EndNormalVector.y, &tWeld.EndNormalVector.z,
		&tWeld.StartPointType, &tWeld.EndPointType,
		&tWeldInfo.tAtrribute.nIsDoubleWelding, &tWeldInfo.tAtrribute.nStartWrapType, &tWeldInfo.tAtrribute.nEndWrapType,
		&tWeldInfo.tAtrribute.nWeldAngleSize, &tWeldInfo.tAtrribute.dStartHoleSize, &tWeldInfo.tAtrribute.dEndHoleSize,
		&tWeldInfo.tAtrribute.nRobotSelete, &tWeldInfo.tAtrribute.dGroupTrackPos, &tWeldInfo.tAtrribute.nGroupNo,
		&tWeldInfo.tAtrribute.nBigGroupNo,&tWeldInfo.tAtrribute.dThoeryLength))
	{
		int ng = 0;
		tWeldInfo.tWeldLine = tWeld;
		vtWeldSeamData.clear();
		vtWeldSeamInfo.clear();
		for (ng = 0; ng < vvtWeldSeamData1.size(); ng++)
		{
			//if (fabs(tWeldInfo.tAtrribute.dGroupTrackPos - vvtWeldSeamInfo1.at(ng).at(0).tAtrribute.dGroupTrackPos) < 1.0)
			if (tWeldInfo.tAtrribute.nBigGroupNo == vvtWeldSeamInfo1.at(ng).at(0).tAtrribute.nBigGroupNo)
			{
				vvtWeldSeamData1.at(ng).push_back(tWeld);
				vvtWeldSeamInfo1.at(ng).push_back(tWeldInfo);
				break;
			}
		}
		if (vvtWeldSeamData1.size() < 1 || vvtWeldSeamData1.size() == ng)
		{
			vtWeldSeamData.push_back(tWeld);
			vtWeldSeamInfo.push_back(tWeldInfo);
			vvtWeldSeamData1.push_back(vtWeldSeamData);
			vvtWeldSeamInfo1.push_back(vtWeldSeamInfo);
			vtWeldSeamData.clear();
			vtWeldSeamInfo.clear();
		}
	}
	fclose(pf);
	for (int nWeldSeamNo = 0; nWeldSeamNo < vvtWeldSeamData1.size(); nWeldSeamNo++)
	{
		for (int i = 0; i < vvtWeldSeamData1.at(nWeldSeamNo).size(); i++)
		{
			//vvtWeldSeamInfo1[nWeldSeamNo].at(i).tAtrribute.dGroupTrackPos -= m_vpUnit[0]->m_dRobotBaseToGantryCtrDis;
		}
	}
	m_vvvtWeldSeamData.clear();
	m_vvvtWeldSeamInfo.clear();
	m_vvvtWeldSeamData.resize(vvtWeldSeamData1.size());
	m_vvvtWeldSeamInfo.resize(vvtWeldSeamData1.size());
	for (int nGpNo = 0; nGpNo < vvtWeldSeamData1.size(); nGpNo++)
	{
		m_vvvtWeldSeamData.at(nGpNo).resize(m_vpRobotDriver.size());
		m_vvvtWeldSeamInfo.at(nGpNo).resize(m_vpRobotDriver.size());

		for (int i = 0; i < vvtWeldSeamInfo1.at(nGpNo).size(); i++)
		{
			for (int nRb = 0; nRb < m_vpRobotDriver.size(); nRb++)
			{
				if (nRb == vvtWeldSeamInfo1[nGpNo].at(i).tAtrribute.nRobotSelete-1)
				{
					int nRobotNo = nRb;
					/*if (1 == nRb)
					{
						nRobotNo = 2;
					}
					else if (2 == nRb)
					{
						nRobotNo = 3;
					}
					else if (3 == nRb)
					{
						nRobotNo = 1;
					}*/
					m_vvvtWeldSeamData.at(nGpNo).at(nRobotNo).push_back(vvtWeldSeamInfo1[nGpNo].at(i).tWeldLine);
					m_vvvtWeldSeamInfo.at(nGpNo).at(nRobotNo).push_back(vvtWeldSeamInfo1[nGpNo].at(i));
				}
			}
		}
	}
	// 输出分组后数据便于查看
	for (size_t i = 0; i < m_vvvtWeldSeamInfo.size(); i++)
	{
		for (int nRb = 0; nRb < m_vpRobotDriver.size(); nRb++)
		{
			CString strFile1 = OUTPUT_PATH + m_vpUnit[0]->m_tContralUnit.strUnitName + "\\" + RECOGNITION_FOLDER;
			CString strFile;

			strFile.Format("%s%dRobot%dGroup.txt", strFile1, nRb + 1, i);
			FILE* pFile = fopen(strFile.GetBuffer(0), "w");
			for (size_t j = 0; j < m_vvvtWeldSeamInfo.at(i)[nRb].size(); j++)
			{
				LineOrCircularArcWeldingLine tWeld = m_vvvtWeldSeamInfo.at(i)[nRb][j].tWeldLine;
				WeldSeamAtrribute tAtrribute = m_vvvtWeldSeamInfo.at(i)[nRb][j].tAtrribute;

				fprintf(pFile, "%11.3lf%11.3lf%11.3lf\n", tWeld.StartPoint.x, tWeld.StartPoint.y, tWeld.StartPoint.z);
				fprintf(pFile, "%11.3lf%11.3lf%11.3lf\n", tWeld.EndPoint.x, tWeld.EndPoint.y, tWeld.EndPoint.z);

				/*fprintf(pFile, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d%4d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d%4d%4d%4d%4d%11.3lf%11.3lf%4d%11.3lf\n",
					i, tWeld.StartPoint.x, tWeld.StartPoint.y, tWeld.StartPoint.z,
					tWeld.EndPoint.x, tWeld.EndPoint.y, tWeld.EndPoint.z,
					tWeld.ZSide, tWeld.isLeft, tWeld.isClockwise, tWeld.IsArc, tWeld.CenterPoint.x, tWeld.CenterPoint.y, tWeld.CenterPoint.z,
					tWeld.StartNormalVector.x, tWeld.StartNormalVector.y, tWeld.StartNormalVector.z,
					tWeld.EndNormalVector.x, tWeld.EndNormalVector.y, tWeld.EndNormalVector.z,
					tWeld.StartPointType, tWeld.EndPointType,
					tAtrribute.nIsDoubleWelding, tAtrribute.nStartWrapType, tAtrribute.nEndWrapType,
					tAtrribute.nWeldAngleSize, tAtrribute.dStartHoleSize, tAtrribute.dEndHoleSize, tAtrribute.nRobotSelete, tAtrribute.dGroupTrackPos
				);*/
			}
			fclose(pFile);
		}
	}

	return true;
}
bool CAssemblyWeld::LoadCloudProcessResultMultiMachine(CString sFileName/* = ""*/)
{
	CString sOutFileName = OUTPUT_PATH + sFileName + "\\" + IDENTIFY_RESULT_SAVE;
	//CString sOutFileName = "GraphData\\PointCloudIdentifyReaultAfter.txt";
	/*if ("" != sFileName)
	{
		sOutFileName = sFileName;
	}*/
	FILE* pf = fopen(sOutFileName.GetBuffer(), "r");
	if (NULL == pf)
	{
		XUI::MesBox::PopOkCancel("加载点云处理结果文件 {0} 打开失败", sOutFileName);
		return false;
	}

	std::vector<LineOrCircularArcWeldingLine> vtWeldSeamData; // 点云处理得到的焊缝信息
	std::vector<WeldLineInfo> vtWeldSeamInfo; // 包括识别结果及焊脚等无法识别的属性信息
	m_vvvtWeldSeamData.clear();
	m_vvvtWeldSeamInfo.clear();
	m_vvvtWeldSeamData.resize(m_vpRobotDriver.size());
	m_vvvtWeldSeamInfo.resize(m_vpRobotDriver.size());
	//m_vtWeldSeamData.clear();
	//m_vtWeldSeamInfo.clear();

	std::vector < std::vector<LineOrCircularArcWeldingLine>> vvtWeldSeamData1; // 多机点云处理得到的焊缝信息
	std::vector < std::vector<WeldLineInfo>> vvtWeldSeamInfo1; // 多机包括识别结果及焊脚等无法识别的属性信息

	int nIdx = 0;
	bool m = 0;
	double xj;
	LineOrCircularArcWeldingLine tWeld;
	WeldLineInfo tWeldInfo;
	while (EOF != fscanf(pf, "%d%d%d%lf %d %lf%lf%lf %lf%lf%lf %lf%lf%lf %lf%lf%lf %lf%lf%lf %d%d %d%d%d %lf%lf%lf%d %lf%lf%lf%lf",
		&tWeldInfo.tAtrribute.nWeldSeamIdx, &tWeld.IsArc, (int*)(void*)&tWeld.isClockwise, &tWeld.ZSide, (int*)(void*)&tWeld.isLeft,
		&tWeld.CenterPoint.x, &tWeld.CenterPoint.y, &tWeld.CenterPoint.z,
		&tWeld.StartPoint.x, &tWeld.StartPoint.y, &tWeld.StartPoint.z,
		&tWeld.EndPoint.x, &tWeld.EndPoint.y, &tWeld.EndPoint.z,
		&tWeld.StartNormalVector.x, &tWeld.StartNormalVector.y, &tWeld.StartNormalVector.z,
		&tWeld.EndNormalVector.x, &tWeld.EndNormalVector.y, &tWeld.EndNormalVector.z,
		&tWeld.StartPointType, &tWeld.EndPointType,
		&tWeldInfo.tAtrribute.nIsDoubleWelding, &tWeldInfo.tAtrribute.nStartWrapType, &tWeldInfo.tAtrribute.nEndWrapType,
		(double*)(void*)&tWeldInfo.tAtrribute.nWeldAngleSize, &tWeldInfo.tAtrribute.dStartHoleSize, &tWeldInfo.tAtrribute.dEndHoleSize,
		&tWeldInfo.tAtrribute.nRobotSelete, &tWeldInfo.tAtrribute.dThoeryLength, &xj, &xj, &xj))
	{
		//tWeldInfo.tAtrribute.nGroupNo = tWeldInfo.tAtrribute.nEndWrapType; // ?????
		tWeldInfo.tAtrribute.nGroupNo = 0; // ?????
		int ng = 0;
		tWeldInfo.tWeldLine = tWeld;
		vtWeldSeamData.clear();
		vtWeldSeamInfo.clear();
		for (ng = 0; ng < vvtWeldSeamData1.size(); ng++)
		{
			if (tWeldInfo.tAtrribute.nGroupNo == vvtWeldSeamInfo1.at(ng).at(0).tAtrribute.nGroupNo)
			{
				vvtWeldSeamData1.at(ng).push_back(tWeld);
				vvtWeldSeamInfo1.at(ng).push_back(tWeldInfo);
				break;
			}
		}
		if (vvtWeldSeamData1.size() < 1 || vvtWeldSeamData1.size() == ng)
		{
			vtWeldSeamData.push_back(tWeld);
			vtWeldSeamInfo.push_back(tWeldInfo);
			vvtWeldSeamData1.push_back(vtWeldSeamData);
			vvtWeldSeamInfo1.push_back(vtWeldSeamInfo);
			vtWeldSeamData.clear();
			vtWeldSeamInfo.clear();
		}

		for (int nRoobtNo = 0; nRoobtNo < m_vpRobotDriver.size(); nRoobtNo++)
		{		
			if (nRoobtNo == tWeldInfo.tAtrribute.nRobotSelete)
			{
				vtWeldSeamData.clear();
				vtWeldSeamInfo.clear();
				tWeldInfo.tWeldLine = tWeld;
				int nGpNo = 0;
				for (nGpNo = 0; nGpNo < m_vvvtWeldSeamData.at(nRoobtNo).size(); nGpNo++)
				{
					if (m_vvvtWeldSeamInfo.at(nRoobtNo).at(nGpNo).at(0).tAtrribute.nGroupNo == tWeldInfo.tAtrribute.nGroupNo)
					{
						m_vvvtWeldSeamData.at(nRoobtNo).at(nGpNo).push_back(tWeld);
						m_vvvtWeldSeamInfo.at(nRoobtNo).at(nGpNo).push_back(tWeldInfo);
						break;
					}
				}
				if (m_vvvtWeldSeamInfo.at(nRoobtNo).size() < 1 || m_vvvtWeldSeamInfo.at(nRoobtNo).size() == nGpNo)
				{
					vtWeldSeamData.push_back(tWeld);
					vtWeldSeamInfo.push_back(tWeldInfo);
					m_vvvtWeldSeamData.at(nRoobtNo).push_back(vtWeldSeamData);
					m_vvvtWeldSeamInfo.at(nRoobtNo).push_back(vtWeldSeamInfo);
					break;
				}								
			}
		}
	}
	fclose(pf);

	double dPosY = 0;
	for (int nWeldSeamNo = 0; nWeldSeamNo < vvtWeldSeamData1.size(); nWeldSeamNo++)
	{
		for (int i = 0; i < vvtWeldSeamData1.at(nWeldSeamNo).size(); i++)
		{
			dPosY += vvtWeldSeamData1[nWeldSeamNo].at(i).StartPoint.y;
			dPosY += vvtWeldSeamData1[nWeldSeamNo].at(i).EndPoint.y;

			vvtWeldSeamData1[nWeldSeamNo].at(i).ZSide += 10.0;
		}
		dPosY /= vvtWeldSeamData1.at(nWeldSeamNo).size() * 2;

		//dPosY -= m_vpRobotDriver[0]->m_dRobotBaseCtrToGantryCtrDis;

		for (int i = 0; i < vvtWeldSeamData1.at(nWeldSeamNo).size(); i++)
		{
			vvtWeldSeamInfo1[nWeldSeamNo].at(i).tAtrribute.dGroupTrackPos = dPosY;
		}
	}
	m_vvvtWeldSeamData.clear();
	m_vvvtWeldSeamInfo.clear();
	m_vvvtWeldSeamData.resize(vvtWeldSeamData1.size());
	m_vvvtWeldSeamInfo.resize(vvtWeldSeamData1.size());
	for (int nGpNo = 0; nGpNo < vvtWeldSeamData1.size(); nGpNo++)
	{
		m_vvvtWeldSeamData.at(nGpNo).resize(m_vpRobotDriver.size());
		m_vvvtWeldSeamInfo.at(nGpNo).resize(m_vpRobotDriver.size());

		for (int i = 0; i < vvtWeldSeamInfo1.at(nGpNo).size(); i++)
		{
			for (int nRb = 0; nRb < m_vpRobotDriver.size(); nRb++)
			{
				if (nRb == vvtWeldSeamInfo1[nGpNo].at(i).tAtrribute.nRobotSelete)
				{
					m_vvvtWeldSeamData.at(nGpNo).at(nRb).push_back(vvtWeldSeamInfo1[nGpNo].at(i).tWeldLine);
					m_vvvtWeldSeamInfo.at(nGpNo).at(nRb).push_back(vvtWeldSeamInfo1[nGpNo].at(i));
				}
			}			
		}
	}
	return true;
}
  void CAssemblyWeld::ShowTeachImage()
  {
	  int nBuffSize = m_vpShowLaserImgBuff.size();
	  for (int nDrawBoardNum = 0; nDrawBoardNum < m_vnDrawComponentID.size(); nDrawBoardNum++) {
		  if (nDrawBoardNum < nBuffSize && NULL != m_vpShowLaserImgBuff[nDrawBoardNum])
		  {
			  if (0 == nDrawBoardNum && m_vpShowLaserImgBuff[nDrawBoardNum]->imageData[0] >= 0)
			  {
				  DrawImage(m_vpShowLaserImgBuff[nDrawBoardNum], this, IDC_STATIC_DRAW_PART_GLOBAL);
			  }
			  else if (m_vpShowLaserImgBuff[nDrawBoardNum]->imageData[0] >= 0)
			  {
				  DrawImage(m_vpShowLaserImgBuff[nDrawBoardNum], this, m_vnDrawComponentID[nDrawBoardNum]);
			  }
		  }
	  }
  }

  void CAssemblyWeld::ShowTeachImage(IplImage *pImage)
  {
	  CRect tRect;
	  GetDlgItem(IDC_STATIC_DRAW_PART_GLOBAL)->GetClientRect(&tRect);
	  tRect.left += 5;
	  tRect.right -= 5;
	  tRect.top += 5;
	  tRect.bottom -= 5;
	  if (NULL != pImage)
	  {
		  DrawImage(pImage, this, IDC_STATIC_DRAW_PART_GLOBAL);
	  }
  }

  void CAssemblyWeld::SetShowImg(IplImage* pImage)
  {
	  if (m_vpShowLaserImgBuff.size() <= 0)
		  return;
	  auto pTemp = pImage;
	  pImage = m_vpShowLaserImgBuff[0];
	  m_vpShowLaserImgBuff[0] = pTemp;
	 // cvCopy(pImage, m_vpShowLaserImgBuff[0]);
  }

 void CAssemblyWeld::OnBnClickedButtonPlasmaCom()
 {	
    /* int m_nLaserColor = 0;
     int m_nRobotNumber = 0;
     E_CAM_ID eCamId;
     CRobotDriverAdaptor *pRobotCtrl;
     
     if (IDOK == XiMessageBoxGroup(1, "左机器人运行？"))
     {
         pRobotCtrl = m_vpRobotDriver[0];;
         if (IDOK == XiMessageBox("左左相机测量"))
         {
             eCamId = E_LEFT_ROBOT_LEFT_CAM_H;

         }
         else if (IDOK == XiMessageBox("左右相机测量"))
         {
             eCamId = E_LEFT_ROBOT_RIGHT_CAM_H;
         }
         else
         {
             XiMessageBox("请选择相机");
             return;
         }
     }
     else if (IDOK == XiMessageBoxGroup(1, "右机器人运行？"))
     {
         pRobotCtrl = m_vpRobotDriver[1];;
         if (IDOK == XiMessageBox("右左相机测量"))
         {
             eCamId = E_RIGHT_ROBOT_LEFT_CAM_H;

         }
         else if (IDOK == XiMessageBox("右右相机测量"))
         {
             eCamId = E_RIGHT_ROBOT_RIGHT_CAM_H;
         }
         else
         {
             XiMessageBox("请选择相机");
             return;
         }
     }
     else
     {
         XiMessageBoxGroup(1, "未选择机器人");
         return;
     }
     testCompenWithPara(pRobotCtrl, eCamId);*/
     return;
 }

 void CAssemblyWeld::TestGroovePointCloudProcess()
 {
	 
 }

 void CAssemblyWeld::TestGetEndPtnUsePointCloudWRZ()
 {
	 int nImageNoS = 0;
	 int nImageNoE = 0;
	 bool bIsUpRight = false;
	 CString sPointCloudFile;
	 CString sRecoResultFile;
	 CString sSaveResultFile;
	 CString sAllResultFile;
	 CString sPartCloudFile;
	 sPointCloudFile = OpenFileDlg(this, _T("*"), "./");
	 //sPointCloudFile = UnicodeToUtf8(sPointCloudFile.GetBuffer());


	 int nIdx = sPointCloudFile.ReverseFind('.');
	 CString s1 = sPointCloudFile.Left(nIdx);

	 sRecoResultFile.Format("%s_ImgNoS%d-ImgNoE%d_Rst.txt", /*UnicodeToUtf8*/(s1.GetBuffer()), nImageNoS, nImageNoE);
	 sSaveResultFile.Format("%s_ImgNoS%d-ImgNoE%d_Save.txt", /*UnicodeToUtf8*/(s1.GetBuffer()), nImageNoS, nImageNoE);
	 sAllResultFile.Format("%s_ImgNoS%d-ImgNoE%d_AllResult.txt", /*UnicodeToUtf8*/(s1.GetBuffer()), nImageNoS, nImageNoE);
	 sPartCloudFile.Format("%s_ImgNoS%d-ImgNoE%d_Part.txt", /*UnicodeToUtf8*/(s1.GetBuffer()), nImageNoS, nImageNoE);


	 CvPoint3D32f tRefPtn[2] = { 0.0 };
	 CvPoint3D32f tPlaneHNorm;
	 CvPoint3D32f tCameraNorm;
	 RiserEndPointInfo tWeldInfo[3] = { 0 };
	 std::vector<Three_DPoint> vtPtns(0);
	 int  nWeldInfoNum = 0;
	 int nPtnNum;

	 // 读取输入参数
	 int nImageNo = 0;
	 int nPtnNo = 0;
	 Three_DPoint tPtn;
	 FILE* pf = fopen(sPointCloudFile.GetBuffer(), "r");

// nImageNoS:0 nImageNoE : 105 
 //tCameraNorm : 0.941      0.339      1.000 
 //tPlaneHNorm : 0.000      0.000 - 1.000 
	 //tRefPtn[0] : -658.400   3590.340   1936.170 
	 //tRefPtn[1] : -658.180   3666.050   1936.180
	 
	 fscanf(pf, "nImageNoS:%d nImageNoE:%d tCameraNorm:%f%f%f tPlaneHNorm:%f%f%f tRefPtn[0]:%f%f%f tRefPtn[1]:%f%f%f\n", &nImageNo, &nPtnNo,
		 &tCameraNorm.x, &tCameraNorm.y, &tCameraNorm.z,
		 &tPlaneHNorm.x, &tPlaneHNorm.y, &tPlaneHNorm.z,
		 &tRefPtn[0].x, &tRefPtn[0].y, &tRefPtn[0].z,
		 &tRefPtn[1].x, &tRefPtn[1].y, &tRefPtn[1].z);

	 /*fscanf(pf, "%d%d %f%f%f %f%f%f %f%f%f %f%f%f\n", &nImageNo, &nPtnNo,
		 &tCameraNorm.x, &tCameraNorm.y, &tCameraNorm.z,
		 &tPlaneHNorm.x, &tPlaneHNorm.y, &tPlaneHNorm.z,
		 &tRefPtn[0].x, &tRefPtn[0].y, &tRefPtn[0].z,
		 &tRefPtn[1].x, &tRefPtn[1].y, &tRefPtn[1].z);*/
	


	 while (EOF != fscanf(pf, "%d%lf%lf%lf\n", &nPtnNo, &tPtn.x, &tPtn.y, &tPtn.z))
	 {
		 vtPtns.push_back(tPtn);
	 }
	 fclose(pf);
	 nPtnNum = vtPtns.size();

	 FILE* pf2 = fopen(sPartCloudFile.GetBuffer(), "w");
	 for (int i = 0; i < nPtnNum; i++)
	 {
		 fprintf(pf2, "%d%11.3lf%11.3lf%11.3lf\n", i, vtPtns[i].x, vtPtns[i].y, vtPtns[i].z);
	 }
	 fclose(pf2);


	 try
	 {
		 nWeldInfoNum = GetRiserEndPoint((CvPoint3D64f*)vtPtns.data(), nPtnNum, tWeldInfo, tCameraNorm, tPlaneHNorm, tRefPtn, 2, "GetRiserEndPoint");
	 }
	 catch (...)
	 {
		 XUI::MesBox::PopOkCancel("点云处理异常! 图号{0}-{1}", nImageNoS, nImageNoE);
		 return;
	 }

	 XUI::MesBox::PopInfo("处理结果数量：{0}", nWeldInfoNum);
	 if (nWeldInfoNum<=0)
	 {
		 return;
	 }

	 CvPoint3D32f tPtnS = tWeldInfo[0].staPnt; // 只影响处理结果的点顺序
	 CvPoint3D32f tPtnE = tWeldInfo[0].endPnt; // 只影响处理结果的点顺序
	 FILE* pfOut = fopen(sRecoResultFile.GetBuffer(), "w");
	 fprintf(pfOut, "%d %4d%4d%11.3lf%4d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d 0 0 0 %4d%11.3lf%11.3lf 0\n",
		 0, 0, 0, 0.0, true,
		 0.0, 0.0, 0.0,
		 tPtnS.x, tPtnS.y, tPtnS.z,
		 tPtnE.x, tPtnE.y, tPtnE.z,
		 0.0, 0.0, 0.0,
		 0.0, 0.0, 0.0,
		 0, 0,
		 0, 0.0, 0.0);
	 fclose(pfOut);

	/* pf2 = fopen(sAllResultFile.GetBuffer(), "w");
	 for (int i = 0; i < nResultPtnNum; i++)
	 {
		 fprintf(pf2, "%d%11.3lf%11.3lf%11.3lf\n", i, ptResult[i].x, ptResult[i].y, ptResult[i].z);
	 }
	 fclose(pf2);*/

	// RunInteractiveWindow(UnicodeToUtf8(sPartCloudFile.GetBuffer()), UnicodeToUtf8(sSaveResultFile.GetBuffer()), UnicodeToUtf8(sRecoResultFile.GetBuffer()));
	 RunInteractiveWindow(sPartCloudFile.GetBuffer(), sSaveResultFile.GetBuffer(), sRecoResultFile.GetBuffer());
	 return;
 }


 void CAssemblyWeld::TestGetEndPtnUsePointCloud()
 {
	 int nImageNoS = 0;
	 int nImageNoE = 0;
	 bool bIsUpRight = false;
	 CString sPointCloudFile;
	 CString sRecoResultFile;
	 CString sSaveResultFile;
	 CString sAllResultFile;
	 CString sPartCloudFile;
	 sPointCloudFile = OpenFileDlg(this, _T("*"), "./");
	 //sPointCloudFile = UnicodeToUtf8(sPointCloudFile.GetBuffer());

	 // 弹窗标题
	 CString cStrTitle = "输入开始和结束图号";
	 // 参数的中文名称
	 std::vector<CString> vsInputName;
	 vsInputName.push_back("开始图号");
	 vsInputName.push_back("结束图号");
	 vsInputName.push_back("倒挂0正座1");
	 std::vector<double> vnInputData;
	 vnInputData.push_back(nImageNoS);
	 vnInputData.push_back(nImageNoE);
	 vnInputData.push_back((int)bIsUpRight);
	 ParamInput cParamDlg(cStrTitle, vsInputName, &vnInputData);
	 cParamDlg.DoModal();
	 nImageNoS = vnInputData.at(0);
	 nImageNoE = vnInputData.at(1);
	 bIsUpRight = vnInputData.at(2) > 0;

	 int nIdx = sPointCloudFile.ReverseFind('.');
	 CString s1 = sPointCloudFile.Left(nIdx);
	 sRecoResultFile.Format("%s_ImgNoS%d-ImgNoE%d_Rst.txt", /*UnicodeToUtf8*/(s1.GetBuffer()), nImageNoS, nImageNoE);
	 sSaveResultFile.Format("%s_ImgNoS%d-ImgNoE%d_Save.txt", /*UnicodeToUtf8*/(s1.GetBuffer()), nImageNoS, nImageNoE);
	 sAllResultFile.Format("%s_ImgNoS%d-ImgNoE%d_AllResult.txt", /*UnicodeToUtf8*/(s1.GetBuffer()), nImageNoS, nImageNoE);
	 sPartCloudFile.Format("%s_ImgNoS%d-ImgNoE%d_Part.txt", /*UnicodeToUtf8*/(s1.GetBuffer()), nImageNoS, nImageNoE);

	 std::map<int, vector<Three_DPoint>> mntPoints;
	 mntPoints.clear();

	 // 读取输入参数
	 int nImageNo = 0;
	 int nPtnNo = 0;
	 Three_DPoint tPtn;
	 FILE* pf = fopen(sPointCloudFile.GetBuffer(), "r");
	 while (EOF != fscanf(pf, "图号:%d 点号%d %lf%lf%lf\n", &nImageNo, &nPtnNo, &tPtn.x, &tPtn.y, &tPtn.z))
	 {
		 std::map<int, vector<Three_DPoint>>::iterator iter = mntPoints.find(nImageNo);
		 if (iter != mntPoints.end())
		 {
			 mntPoints[nImageNo].push_back(tPtn);
		 }
		 else
		 {
			 std::vector<Three_DPoint> vtPtn(1, tPtn);
			 mntPoints[nImageNo] = vtPtn;
		 }
	 }
	 fclose(pf);

	 std::vector<Three_DPoint> vtPtns(0);
	 for (int i = nImageNoS; i <= nImageNoE; i++)
	 {
		 vtPtns.insert(vtPtns.end(), mntPoints[i].begin(), mntPoints[i].end());
	 }
	 Three_DPoint* ptResult = NULL;
	 Three_DPoint* pThreeDPoint = vtPtns.data();
	 int nPtnNum = vtPtns.size();
	 int nResultPtnNum = 0;
	 Three_DPoint tScanVector = { 1.0, 0.0, 0.0 }; // 只影响处理结果的点顺序

	 FILE* pf2 = fopen(sPartCloudFile.GetBuffer(), "w");
	 for (int i = 0; i < nPtnNum; i++)
	 {
		 fprintf(pf2, "%d%11.3lf%11.3lf%11.3lf\n", i, vtPtns[i].x, vtPtns[i].y, vtPtns[i].z);
	 }
	 fclose(pf2);

	 try
	 {
		 //ptResult = GetLocalFreeEnd(pThreeDPoint, nPtnNum, &nResultPtnNum, bIsUpRight, tScanVector, 2.0);
	 }
	 catch (...)
	 {
		 XUI::MesBox::PopOkCancel("点云处理异常! 图号{0}-{1}", nImageNoS, nImageNoE);
		 return;
	 }

	 XUI::MesBox::PopInfo("处理结果数量：{0}", nResultPtnNum);

	 Three_DPoint tPtnS = ptResult[0]; // 只影响处理结果的点顺序
	 Three_DPoint tPtnE = ptResult[nResultPtnNum - 1]; // 只影响处理结果的点顺序
	 FILE* pfOut = fopen(sRecoResultFile.GetBuffer(), "w");
	 fprintf(pfOut, "%d %4d%4d%11.3lf%4d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d 0 0 0 %4d%11.3lf%11.3lf 0\n",
		 0, 0, 0, 0.0, true,
		 0.0, 0.0, 0.0,
		 tPtnS.x, tPtnS.y, tPtnS.z,
		 tPtnE.x, tPtnE.y, tPtnE.z,
		 0.0, 0.0, 0.0,
		 0.0, 0.0, 0.0,
		 0, 0,
		 0, 0.0, 0.0);
	 fclose(pfOut);

	 pf2 = fopen(sAllResultFile.GetBuffer(), "w");
	 for (int i = 0; i < nResultPtnNum; i++)
	 {
		 fprintf(pf2, "%d%11.3lf%11.3lf%11.3lf\n", i, ptResult[i].x, ptResult[i].y, ptResult[i].z);
	 }
	 fclose(pf2);

	 RunInteractiveWindow(UnicodeToUtf8(sPartCloudFile.GetBuffer()), UnicodeToUtf8(sSaveResultFile.GetBuffer()), UnicodeToUtf8(sRecoResultFile.GetBuffer()));
	 return;
 }

 void CAssemblyWeld::TestLockProcess(int nRobotNo/* = 0*/, int nCamerNo/* = 0*/)
 {
	 CUnit* pUnit = m_vpUnit[nRobotNo];
	 CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
	 CString sParamFileName = pUnit->GetTrackTipDataFileName(nCamerNo);
	 CString saveDirectory = _T(OUTPUT_PATH + pUnit->m_tContralUnit.strUnitName + SEARCH_PRO_PATH);
	 CheckFolder(saveDirectory);
	 DelFiles(saveDirectory);

	 CString sDefaultPath = "./";
	 CString sFilePath;
	 sFilePath = OpenFileDlg(this, _T("*"), sDefaultPath);
	 int n = sFilePath.ReverseFind('\\');
	 CString sPrefix = sFilePath.Left(n + 1);

	 T_CAMREA_PARAM tCameraParam = pUnit->GetCameraParam(nCamerNo);
	 CvSize tImageSize = cvSize(tCameraParam.tDHCameraDriverPara.nMaxWidth, tCameraParam.tDHCameraDriverPara.nMaxHeight);
	 GROUP_STAND_DIP_NS::CImageProcess ImageProcess(tImageSize.width, tImageSize.height, 1, sParamFileName.GetBuffer());
	 IplImage* pImage = cvLoadImage(sFilePath, 0);
	 IplImage* pColorImg = cvCreateImage(cvSize(pImage->width, pImage->height), IPL_DEPTH_8U, 3);

	// T_TEACH_RESULT tTeachResult;
	// long long nptime = XI_clock();
	// findConner.FindCrossPntAndTwoLines2(pImage, 1,
	//	 tTeachResult.tKeyPtn2D, tTeachResult.vtLeftPtns2D, tTeachResult.vtRightPtns2D, 150, 180, 0, 2000, 1, 0, 2);
	//XiMessageBoxOk("提取激光点FindCrossPntAndTwoLines2用时%d", XI_clock() - nptime);
	//return;

	/* if (IDOK == XiMessageBox("左右翻转图片？"))
	 {
		 cvFlip(pImage, pImage, 1);
	 }*/
	 bool bFil = false;
	 if (IDOK == XiMessageBox("上下翻转图片？"))
	 {
		 cvFlip(pImage, pImage, 0);
		 bFil = true;
	 }

	 // 调用 处理角点和两条激光线上的点接口处理获取锁定需要的数据
	 CvPoint cpMidKeyPoint;
	 CvPoint cpBesideKeyPoint;
	 vector<CvPoint> vtLeftPtns;
	 vector<CvPoint> vtRightPtns;
	 //FlipImage(pImage, pUnit->GetCameraParam(pUnit->m_nMeasureCameraNo).eFlipMode);
	 ImageProcess.GetBesideKeyPoints(pImage, cpMidKeyPoint, cpBesideKeyPoint, vtLeftPtns, vtRightPtns, false,
		 500, 500,
		 300, 300,
		 20, 20, bFil);
	 CvPoint LeftPtn = vtLeftPtns[vtLeftPtns.size() / 2];
	 CvPoint RightPtn = vtRightPtns[vtRightPtns.size() / 2];

	 CvPoint tKeyPoint;
	 XiLineParamNode tFrontLine, tBackLine;
	 ImageProcess.InitIfStartOrEndPntParam(GROUP_STAND_DIP_NS::E_PIECE_START); Sleep(50);

	 // 替代原锁定函数 GetGroupStandKeyPoint
	 tKeyPoint = ImageProcess.HandlockKeyPoint(cpMidKeyPoint, LeftPtn, RightPtn, tFrontLine, tBackLine);
	 //tKeyPoint = pImageProcess->GetGroupStandKeyPoint(pImage, tFrontLine, tBackLine, true, false, false, true);

	 CvPoint p1 = cvPoint(0, tFrontLine.b);
	 CvPoint p2 = cvPoint(tImageSize.width, tImageSize.width * tFrontLine.k + tFrontLine.b);
	 CvPoint p3 = cvPoint(0, tBackLine.b);
	 CvPoint p4 = cvPoint(tImageSize.width, tImageSize.width * tBackLine.k + tBackLine.b);

	 cvCvtColor(pImage, pColorImg, CV_GRAY2RGB);
	 cvCircle(pColorImg, tKeyPoint, 10, CV_RGB(255, 0, 0), 3);
	 cvLine(pColorImg, p1, p2, CV_RGB(0, 255, 0), 2);
	 cvLine(pColorImg, p3, p4, CV_RGB(0, 0, 255), 2);

	 ShowTeachImage(pColorImg);
	 SaveImage(pColorImg, saveDirectory + "\\Lock.jpg");
	 cvReleaseImage(&pImage);

	 double dCompensateValue = 0.0;
	 for (int i = 2; i < 1000; i += 2)
	 {
		 sFilePath.Format("%s%d.jpg", sPrefix, i);
		 if (!CheckFileExists(sFilePath)) break;
		 IplImage* pImage = cvLoadImage(sFilePath, 0);
		 cvCvtColor(pImage, pColorImg, CV_GRAY2RGB);
		 /*bool bHasKeyPtn = ImageProcess.IfPieceStartOrEndPoint(
			 pImage,500, dCompensateValue, tKeyPoint, GROUP_STAND_DIP_NS::E_PIECE_START);*/

		 bool bProcRst = WeightedPointCluster(pImage, &tFrontLine, &tBackLine, &tKeyPoint, false, "WeightedPointCluster");
		 CvPoint* points = new CvPoint[10000];
		 //int nLength = FindLaserMidPiontEE(*pImageBuff, points, 3, 30, 90, 5, 11, 0, 3, 20000, 3, 1, 2, false);
		 int nLength = FindLaserMidPiontEEEEEInTrack_(pImage, tFrontLine, tKeyPoint, points, 100, 0, 3, 30, 8, 30, 10, 0.5, 10000, 3, 11, 0);
		 nLength += FindLaserMidPiontEEEEEInTrack_(pImage, tBackLine, tKeyPoint, points + nLength, 50, 0, 3, 90, 11, 90, 10, 0.5, 10000 - nLength, 3, 11, 1);


		 cvCircle(pColorImg, tKeyPoint, 10, CV_RGB(255, 0, 0), 3);
		 ShowTeachImage(pColorImg);
		 //sFilePath.Format("%s\\Pro_ % d_ % s.jpg", saveDirectory, i, bHasKeyPtn ? "Y" : "N");
		 //SaveImage(pColorImg, sFilePath);
		 cvReleaseImage(&pImage);
		 DoEvent();
	 }
	 cvReleaseImage(&pColorImg);
	 return;
 }

 void CAssemblyWeld::TestGrooveImageProcess(int nRobotNo/* = 0*/, int nCamerNo/* = 0*/)
 {
	 CUnit* pUnit = m_vpUnit[nRobotNo];
	 CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
	 CString sParamFileName = pUnit->GetTrackTipDataFileName(nCamerNo);
	 CString saveDirectory = _T(OUTPUT_PATH + pUnit->m_tContralUnit.strUnitName + SEARCH_PRO_PATH);
	 CheckFolder(saveDirectory);
	 DelFiles(saveDirectory);

	 CString sDefaultPath = "./";
	 CString sFilePath;
	 sFilePath = OpenFileDlg(this, _T("*"), sDefaultPath);
	 int n = sFilePath.ReverseFind('\\');
	 CString sPrefix = sFilePath.Left(n + 1);

	 CString sNo = sFilePath.Right(sFilePath.GetLength() - n - 1);
	 sNo = sNo.Left(sNo.Find('.'));
	 int nImageNo = atoi(sNo.GetBuffer());//1;
	 char c = sNo[0];
	 bool bIsNumber = sNo[0] > 0 && isdigit(sNo[0] % 255);

	 bool bContinueProc = false;
	 if (1 == XiMessageBox("连续处理?"))
	 {
		 bContinueProc = true;
	 }

	 double dCompensateValue = 0.0;
	 for (int i = nImageNo; i < 1000; i += 8)
	 {
		 if (bContinueProc)
		 {
			sFilePath.Format("%s%d.jpg", sPrefix, i);
		 }
		 if (!CheckFileExists(sFilePath)) break;
		 IplImage* pImage = cvLoadImage(sFilePath, 0); //CV_LOAD_IMAGE_GRAYSCALE;
		 long lTime = GetTickCount();
		 std::vector<CvPoint> pnts, HPnts, GPnts, BPnts;
		 IplImage *dst = cvCreateImage(cvSize(pImage->width, pImage->height), pImage->depth, 3);
		 GrooveEndPntsInfo* Info = new GrooveEndPntsInfo;
		 int nRst = FindGrooveEndPnts(pImage, 1, Info, 190, 1, 1, 0, "Local_Files\\ExtLib\\Vision\\ConfigFiles\\GroovePnts.ini");
		 int nLength = 0;

		 cvCvtColor(pImage, m_vpShowLaserImgBuff[0], CV_GRAY2RGB);
		 for (int i = 0; i < 4; i++)
		 {
			 cvCircle(m_vpShowLaserImgBuff[0], Info->outputPnts[i], 10, CV_RGB(255, 0, 255), 3);
		 }
		 for (int i = 0; i < Info->HPntsNum; i++)
		 {
			 //tPoints[nLength++] = Info->HorizontalLinePnts[i];
			 cvCircle(m_vpShowLaserImgBuff[0], Info->HorizontalLinePnts[i], 3, CV_RGB(255, 0, 0), 1);
		 }
		 for (int i = 0; i < Info->GPntsNum; i++)
		 {
			 //tPoints[nLength++] = Info->GrooveLinePnts[i];
			 cvCircle(m_vpShowLaserImgBuff[0], Info->GrooveLinePnts[i], 3, CV_RGB(0, 255, 0), 1);
		 }
		 for (int i = 0; i < Info->BPntsNum; i++)
		 {
			 //tPoints[nLength++] = Info->BottomLinePnts[i];
			 cvCircle(m_vpShowLaserImgBuff[0], Info->BottomLinePnts[i], 3, CV_RGB(0, 0, 255), 1);
		 }
		 pRobotDriver->m_cLog->Write("[点云搜端点]:处理图号%d ResultNum: %d %d %d", i, Info->HPntsNum, Info->GPntsNum, Info->BPntsNum);
		 delete Info;
		 long lProcessTime = GetTickCount() - lTime;
		 WriteLog("图片%d 耗时%dms", i, GetTickCount() - lTime);
		 sFilePath.Format("%s\\Pro_%d_%s.jpg", saveDirectory, i, 1 == nRst ? "Y" : "N");
		 ShowTeachImage(m_vpShowLaserImgBuff[0]);
		 SaveImage(m_vpShowLaserImgBuff[0], sFilePath);
		 cvReleaseImage(&pImage);
		 DoEvent();
		 if (!bContinueProc)
		 {
			 break;
		 }
	 }
	 return;
 }
 
void CAssemblyWeld::TestLaserCenterPtnImageProcess(int nRobotNo/* = 0*/, int nCamerNo/* = 0*/)
 {
	 int nProcMethod = 1;
	 CUnit* pUnit = m_vpUnit[nRobotNo];
	 CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
	 CString sParamFileName = pUnit->GetTrackTipDataFileName(nCamerNo);
	 CString saveDirectory = _T(OUTPUT_PATH + pUnit->m_tContralUnit.strUnitName + SEARCH_PRO_PATH);
	 CheckFolder(saveDirectory);
	 DelFiles(saveDirectory);
	 IplImage* pShowImage = NULL;
	 T_ROBOT_COORS tCapCoord;
	 T_ANGLE_PULSE tCapPulse;
	 pRobotDriver->RobotKinematics(tCapPulse, pRobotDriver->m_tTools.tGunTool, tCapCoord);

	 CString sDefaultPath = "./";
	 CString sFilePath;
	 sFilePath = OpenFileDlg(this, _T("*"), sDefaultPath);
	 int n = sFilePath.ReverseFind('\\');
	 CString sPrefix = sFilePath.Left(n + 1);

	 CString sNo = sFilePath.Right(sFilePath.GetLength() - n - 1);
	 sNo = sNo.Left(sNo.Find('.'));
	 int nImageNo = atoi(sNo.GetBuffer());//1;
	 char c = sNo[0];
	 bool bIsNumber = sNo[0] > 0 && isdigit(sNo[0] % 255);

	 bool bContinueProc = false;
	 if (1 == XiMessageBox("连续处理?"))
	 {
		 bContinueProc = true;
	 }

	 CString sPointCloud = saveDirectory + "\\TestGeneralPointCloud.txt";
	 FILE* pf = fopen(sPointCloud.GetBuffer(), "w");
	 double dCompensateValue = 0.0;
	 for (int i = nImageNo; i < 1000; i += 2)
	 {
		 if (bContinueProc)
		 {
			 sFilePath.Format("%s%d.jpg", sPrefix, i);
		 }
		 if (!CheckFileExists(sFilePath)) break;
		 IplImage* pImage = cvLoadImage(sFilePath, 0); //CV_LOAD_IMAGE_GRAYSCALE;
		 long lTime = GetTickCount();
		 std::vector<CvPoint> vtPoints;

		 if (1 == nProcMethod)
		 {
			 // 激光中心点提取接口1
			 GetLaserPointFromImageX(pImage, LINE_SCAN_BINARY_THRESHOLD,
				 LINE_SCAN_STEP_2D_POINT, LINE_SCAN_LASER_WIDTH, vtPoints);
		 }
		 else if (2 == nProcMethod)
		 {
			 // 激光中心点提取接口2
			 CvPoint* ptPtns = new CvPoint[10000];
			 CvPoint tROILeftTop = cvPoint(0, 0);
			 CvPoint tROIRightBottom = cvPoint(pImage->width, pImage->height);
			 CvPoint tTopLeftCoord = cvPoint(0, 800);
			 int nPtnNum = FindLaserMidPiontS(ptPtns, pImage, tROILeftTop, tROIRightBottom, tTopLeftCoord,
				 30, 0.05,
				 2,
				 6, 1, 1, false);
			 for (int i = 0; i < nPtnNum; i++)
			 {
				 ptPtns[i].x -= tTopLeftCoord.x;
				 ptPtns[i].y -= tTopLeftCoord.y;
				 vtPoints.push_back(ptPtns[i]);
			 }
			 delete[] ptPtns;
		 }

		 if (NULL == pShowImage)
		 {
			 pShowImage = cvCreateImage(cvSize(pImage->width, pImage->height), pImage->depth, 3);
		 }
		 cvCvtColor(pImage, pShowImage, CV_GRAY2RGB);

		 tCapCoord.dBX += 2.0;
		 pRobotDriver->RobotInverseKinematics(tCapCoord, tCapPulse, pRobotDriver->m_tTools.tGunTool, tCapPulse);
		 std::vector<T_ROBOT_COORS> vtResult = pUnit->GetMeasureInWorldLeaser(
			 pUnit->m_nLineScanCameraNo + nCamerNo, pUnit->P2P(vtPoints),
			 tCapCoord, tCapPulse);
		 for (int i = 0; i < vtPoints.size(); i++)
		 {
			 cvCircle(pShowImage, vtPoints[i], 3, CV_RGB(255, 0, 0), 1);
		 }
		 for (int i = 0; i < vtResult.size(); i++)
		 {
			 fprintf(pf, "%d%11.3lf%11.3lf%11.3lf\n", i, vtResult[i].dX + vtResult[i].dBX, vtResult[i].dY + vtResult[i].dBY, vtResult[i].dZ + vtResult[i].dBZ);
		 }
		 pRobotDriver->m_cLog->Write("[点云搜端点]:处理图号%d ResultNum: %d", i, vtPoints.size());
		 long lProcessTime = GetTickCount() - lTime;
		 WriteLog("图片%d 耗时%dms", i, GetTickCount() - lTime);
		 sFilePath.Format("%s\\Pro_%d.jpg", saveDirectory, i);
		 ShowTeachImage(pShowImage);
		 SaveImage(pShowImage, sFilePath);
		 cvReleaseImage(&pImage);
		 DoEvent();
		 if (!bContinueProc)
		 {
			 break;
		 }
	 }
	 fclose(pf);
	 if (NULL != pShowImage)
	 {
		 cvReleaseImage(&pShowImage);
	 }
	 return;
 }
 void CAssemblyWeld::TestImageProcess(int nRobotNo, int nCamerNo)
 {
	 CUnit* pUnit = m_vpUnit[nRobotNo];
	 CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
	 CString sParamFileName = pUnit->GetTrackTipDataFileName(nCamerNo);
	 CString saveDirectory = _T(OUTPUT_PATH + pUnit->m_tContralUnit.strUnitName + TEACH_PRO_PATH);
	 CheckFolder(saveDirectory);
	 DelFiles(saveDirectory);

	 CString sDefaultPath = "./";
	 CString sFileName;
	 sFileName = OpenFileDlg(this, _T("*"), sDefaultPath);
	 if (sFileName == "") return;
	 int n = sFileName.ReverseFind('\\');
	 CString sFilePath = sFileName.Left(n + 1); 
	 CString sNo = sFileName.Right(sFileName.GetLength() - n - 1);
	 sNo = sNo.Left(sNo.Find('.'));
	 int nImageNo = atoi(sNo.GetBuffer());//1;
	 char c = sNo[0];
	 bool bIsNumber = sNo[0] > 0 && isdigit(sNo[0] % 255);
	 bool bPop = false;
	 if (bIsNumber)
	 {
		bPop = 1 == XiMessageBox("弹窗确认？");
	 }
	 int nLinePtnNum = 5;

	 bool bIsFlip = false;
	 int nRefLineNo = 0;
	 if (1 == XiMessageBox("处理翻转图像？"))
	 {
		 bIsFlip = true;
		 nRefLineNo = 0;
	 }

	 do
	 {
		 SetHintInfo(sFileName);
		 IplImage* pImage = cvLoadImage(sFileName, 0);
		 IplImage* pColorImg = cvCreateImage(cvSize(pImage->width, pImage->height), IPL_DEPTH_8U, 3);
		 GROUP_STAND_DIP_NS::CImageProcess ImageProcess(pImage->width, pImage->height, 1, sParamFileName.GetBuffer());
		 ImageProcess.m_nShowImage = false;
		 CvPoint cpMidKeyPoint;
		 CvPoint cpBesideKeyPoint;
		 vector<CvPoint> vtLeftPtns;
		 vector<CvPoint> vtRightPtns;
		 FlipImage(pImage, pUnit->GetCameraParam(pUnit->m_nMeasureCameraNo).eFlipMode);
		 // 统一处理接口 （测试前要保证，要求角点在上，两个激光线在下(测立板上断面需要上下翻转)）
		 long long lTimeS = XI_clock();
		 ImageProcess.GetBesideKeyPoints(pImage, cpMidKeyPoint, cpBesideKeyPoint, vtLeftPtns, vtRightPtns, false,
			 500, 500,
			 300, 300,
			 20, 20, bIsFlip, nRefLineNo);
		 long long lTimeE = XI_clock();
		 WriteLog("图像%d 处理耗时%dms", nImageNo, lTimeE - lTimeS);

		 cvCvtColor(pImage, pColorImg, CV_GRAY2RGB);
		 cvCircle(pColorImg, cpMidKeyPoint, 10, CV_RGB(255, 0, 0), 3);
		 cvCircle(pColorImg, cpBesideKeyPoint, 10, CV_RGB(255, 0, 0), 3);
  		 for (int i = 0; i < vtLeftPtns.size(); )
		 {
			 cvCircle(pColorImg, vtLeftPtns[i], 8, CV_RGB(0, 255, 0), 3);
			 i += vtLeftPtns.size() / nLinePtnNum;
		 }
		 for (int i = 0; i < vtRightPtns.size(); )
		 {
			 cvCircle(pColorImg, vtRightPtns[i], 8, CV_RGB(0, 0, 255), 3);
			 i += vtRightPtns.size() / nLinePtnNum;
		 }
		 ShowTeachImage(pColorImg);

		 int nPopRst = 0;
		 if (bPop)
		 {
			 nPopRst = AfxMessageBox("第%d张图处理正常?", MB_YESNOCANCEL);
		 }

		 if (bPop && IDCANCEL == nPopRst)
		 {
			 cvReleaseImage(&pImage);
			 cvReleaseImage(&pColorImg);
			 break;
		 }
		 if (bPop && (IDNO == nPopRst))
		 {
			 sFileName.Format(_T("%s\\Err_Src_%d.jpg"), saveDirectory, nImageNo);
			 cvSaveImage(sFileName, pImage);
			 sFileName.Format(_T("%s\\Err_%d.jpg"), saveDirectory, nImageNo);
			 cvSaveImage(sFileName, pColorImg);
		 }

		 if(!bPop)
		 {
			 sFileName.Format(_T("%s\\Test_Pro_%d.jpg"), saveDirectory, nImageNo);
			 cvSaveImage(sFileName, pColorImg);
		 }

		 cvReleaseImage(&pImage);
		 cvReleaseImage(&pColorImg);
		 sFileName.Format("%s%d.jpg", sFilePath, ++nImageNo);//图片路径
	 } while (bIsNumber && CheckFileExists(sFileName, false));
	 SetHintInfo("处理测试结束");
	 return;
 }

 void CAssemblyWeld::TestEndpointAndStandBoardLine(int nRobotNo, int nCamerNo)
 {
	 CUnit* pUnit = m_vpUnit[nRobotNo];
	 CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
	 CString sParamFileName = pUnit->GetTrackTipDataFileName(nCamerNo);
	 CString saveDirectory = _T(OUTPUT_PATH + pUnit->m_tContralUnit.strUnitName + TEACH_PRO_PATH);
	 CheckFolder(saveDirectory);
	 DelFiles(saveDirectory);

	 CString sDefaultPath = "./";
	 CString sFilePath;
	 sFilePath = OpenFileDlg(this, _T("*"), sDefaultPath);
	 if (sFilePath == "") return;
	 int nLinePtnNum = 5;
	 IplImage* pImage = cvLoadImage(sFilePath, 0);
	 IplImage* pColorImg = cvCreateImage(cvSize(pImage->width, pImage->height), IPL_DEPTH_8U, 3);
	 GROUP_STAND_DIP_NS::CImageProcess ImageProcess(pImage->width, pImage->height, 1, sParamFileName);
	 CvPoint cpMidKeyPoint;
	 CvPoint cpBesideKeyPoint;
	 vector<CvPoint> vtLeftPtns;
	 vector<CvPoint> vtRightPtns;

	 if (IDOK == XiMessageBox("左右翻转图片？"))
	 {
		 cvFlip(pImage, pImage, 1);
	 }
	 bool bFil = false;
	 if (IDOK == XiMessageBox("上下翻转图片？"))
	 {
		 cvFlip(pImage, pImage, 0);
		 bFil = true;
	 }

	 int nFirstRefLineNo = 1;  // 默认使用右侧参考线1 
	 int nSecondRefLineNo = 0; // 使用默认参考线处理失败后使用左侧参考线0
	 ImageProcess.m_nShowImage = false;
	 // 统一处理接口 （测试前要保证，要求角点在上，两个激光线在下(测立板上断面需要上下翻转)）
	 long long tTime = XI_clock();
	 ImageProcess.GetBesideKeyPoints(pImage, cpMidKeyPoint, cpBesideKeyPoint, vtLeftPtns, vtRightPtns, false,
		 500, 500,
		 300, 300,
		 20, 20, bFil);
	 WriteLog("GetBesideKeyPoints处理时间：%d", XI_clock() - tTime);
	 XUI::MesBox::PopOkCancel("GetBesideKeyPoints处理时间：{0}", XI_clock() - tTime);
	 //pImgProcess->GetBesideKeyPoints(pImg, tKeyPoint, tBesideKeyPoint, vLeftPtn, vRightPtn, false,
		// 500, 500,
		// 300, 300,
		// 20, 20);

	 cvCvtColor(pImage, pColorImg, CV_GRAY2RGB);
	 cvCircle(pColorImg, cpMidKeyPoint, 10, CV_RGB(255, 0, 0), 3);
	 cvCircle(pColorImg, cpBesideKeyPoint, 10, CV_RGB(255, 0, 0), 3);
	 for (int i = 0; i < vtLeftPtns.size(); )
	 {
		 cvCircle(pColorImg, vtLeftPtns[i], 8, CV_RGB(0, 255, 0), 3);
		 i += vtLeftPtns.size() / nLinePtnNum;
	 }
	 for (int i = 0; i < vtRightPtns.size(); )
	 {
		 cvCircle(pColorImg, vtRightPtns[i], 8, CV_RGB(0, 0, 255), 3);
		 i += vtRightPtns.size() / nLinePtnNum;
	 }
	 ShowTeachImage(pColorImg);

	 SaveImage(pColorImg, saveDirectory + "\\EndpointAngStandBoardLine.jpg");
	 cvReleaseImage(&pImage);
	 cvReleaseImage(&pColorImg);
	 return;
 }

 void CAssemblyWeld::TestFindEndPoint()
 {
	 //CString sDefaultPath = "./";
	 //CString sFilePath;
	 //sFilePath = OpenFileDlg(this, _T("*"), sDefaultPath);

	 //IplImage* pImage = cvLoadImage(sFilePath, 0);
	 //IplImage* pColorImg = cvCreateImage(cvSize(pImage->width, pImage->height), IPL_DEPTH_8U, 3);

	 //CFindCornner cFindCornner(pImage->width, pImage->height, "Data\\FindCornnerParam.ini");
	 //std::vector<CvPoint> vtOutPutPtns;
	 //vtOutPutPtns.clear();
	 //cFindCornner.FindLaserEndpoint(pImage, vtOutPutPtns, 1);

	 
	 //cvCvtColor(pImage, pColorImg, CV_GRAY2RGB);

	 //for (int i = 0; i < vtOutPutPtns.size(); i++)
	 //{
		// cvCircle(pColorImg, vtOutPutPtns[i], 8, CV_RGB(255, 0, 0), 3);
	 //}

	 //ShowTeachImage(pColorImg);
	 //SaveImage(pColorImg, "A_New.jpg");
	 //cvReleaseImage(&pImage);
	 //cvReleaseImage(&pColorImg);

	 //if (2 != vtOutPutPtns.size())
	 //{
		// XiMessageBox("测厚找点函数处理失败！点数 %d ", vtOutPutPtns.size());
	 //}
	 //return;
 }

 void CAssemblyWeld::GenetateCirclePath(XI_POINT tStartPtn, double dRadius,
	 double dFirstPtnDirAngle, double dPtnUnit, double dTotalAngle, std::vector<XI_POINT>& vtCirclePtns)
 {
	 vtCirclePtns.clear();
	 if (dTotalAngle > 360.0 || dTotalAngle <= 0.0)
	 {
		 dTotalAngle = 360.0; // 非正常输入 输出整个圆轨迹
	 }
	 XI_POINT tCircleCenterPtn;
	 tCircleCenterPtn.x = tStartPtn.x + (dRadius * CosD(dFirstPtnDirAngle + 180.0));
	 tCircleCenterPtn.y = tStartPtn.y + (dRadius * SinD(dFirstPtnDirAngle + 180.0));
	 tCircleCenterPtn.z = tStartPtn.z;
	 double dEndPtnDirAngle = dFirstPtnDirAngle + dTotalAngle;
	 double dAngleInterval = 360.0 * dPtnUnit / (2.0 * 3.1415926 * dRadius);
	 XI_POINT tPtn;
	 FILE* pf = fopen("WeldData\\GenetateCirclePart.txt", "w");
	 for (double dDirAngle = dFirstPtnDirAngle; dDirAngle < dEndPtnDirAngle; )
	 {
		 tPtn.x = tCircleCenterPtn.x + (dRadius * CosD(dDirAngle));
		 tPtn.y = tCircleCenterPtn.y + (dRadius * SinD(dDirAngle));
		 tPtn.z = tCircleCenterPtn.z;
		 vtCirclePtns.push_back(tPtn);
		 WriteLog("GenetateCirclePath_DirAngle:%lf:%11.3lf%11.3lf%11.3lf", dDirAngle, tPtn.x, tPtn.y, tPtn.z);
		 fprintf(pf, "%11.3lf%11.3lf%11.3lf%11.3lf\n", tPtn.x, tPtn.y, tPtn.z, dDirAngle);
		 dDirAngle += dAngleInterval;
	 }
	 fclose(pf);
	 return;
 }

 void CAssemblyWeld::testCompenWithPara(CRobotDriverAdaptor *pRobotCtrl, E_CAM_ID camId)
 {
#if 0
     // TODO: 在此添加控件通知处理程序代码
	 int nRobotNo = pRobotCtrl->m_nRobotNo;
     E_CAM_ID eCamId = camId;
     T_ABS_POS_IN_BASE tPointAbsCoordInBase;
     std::vector<CvPoint> vtImagePoint;
     vtImagePoint.clear();
     CvPoint tImagePoint;
     int nImageNo;

     CString strName;
     strName.Format(".\\MeasureData\\%s\\", pRobotCtrl->m_strRobotName);

     CString strImagePoint;
     strImagePoint.Format("%s%d_ImagePoint.txt", strName, eCamId);
     CString strMeasurePluase;
     strMeasurePluase.Format("%s%d_MeasurePluase.txt", strName, eCamId);
     CString strMeasurePos;
     strMeasurePos.Format("%s%d_MeasurePos.txt", strName, eCamId);
     CString strRealPos;
     strRealPos.Format("%sRealTeachPoint.txt", strName);


     FILE *pfImageResult = fopen(strImagePoint.GetBuffer(0)/*".\\MeasureData\\ImagePoint.txt"*/, "r");
     while (fscanf(pfImageResult, "%d%d%d", &nImageNo, &tImagePoint.x, &tImagePoint.y)>0)
     {
         vtImagePoint.push_back(tImagePoint);
     }
     fclose(pfImageResult);

     int nPointNum = vtImagePoint.size();
     XiMessageBox("nPointNum:%d", nPointNum);

     std::vector<T_ROBOT_COORS> vtRobotMeasureCoors;
     vtRobotMeasureCoors.clear();

     T_ROBOT_COORS tRobotCoor;

     tRobotCoor.dX = 0.0;
     tRobotCoor.dY = 0.0;
     tRobotCoor.dZ = 0.0;
     tRobotCoor.dRX = 0.0;
     tRobotCoor.dRY = 0.0;
     tRobotCoor.dRZ = 0.0;

     for (int nPointNo = 0; nPointNo < nPointNum; nPointNo++)
     {
         vtRobotMeasureCoors.push_back(tRobotCoor);
     }

     T_ROBOT_COORS tFirstRobotCoor, tSecondRobotCoor, tThirdRobotCoor, tFourthRobotCoor;
     T_ROBOT_COORS tRealRobotCoor;
     tRealRobotCoor.dX = 0.0;
     tRealRobotCoor.dY = 0.0;
     tRealRobotCoor.dZ = 0.0;
     tRealRobotCoor.dRX = 0.0;
     tRealRobotCoor.dRY = 0.0;
     tRealRobotCoor.dRZ = 0.0;
     int nRealPointNo = 0;
     int nRealPointType = 0;

     std::vector<T_ROBOT_COORS> vtRobotBaseCoors;
     vtRobotBaseCoors.clear();

     FILE* pfRealTeach = fopen(strRealPos.GetBuffer(0)/*".\\MeasureData\\TeachPoint.txt"*/, "r");
     while (fscanf(pfRealTeach, "%d%lf%lf%lf", &nRealPointNo, &tRealRobotCoor.dX, &tRealRobotCoor.dY, &tRealRobotCoor.dZ) > 0)
     {
         vtRobotBaseCoors.push_back(tRealRobotCoor);
     }
     fclose(pfRealTeach);

     std::vector<T_ANGLE_PULSE> vtRobotAnglePulses;
     vtRobotAnglePulses.clear();
     T_ANGLE_PULSE tAnglePulse;
     int nPointNo = 0;
     int nPointType = 1;

     std::vector<T_ABS_POS_IN_BASE> vtAbsPosInBase;
     vtAbsPosInBase.clear();

     FILE* pf = fopen(strMeasurePluase.GetBuffer(0)/*".\\MeasureData\\MeasurePosPluse.txt"*/, "r");
     while (fscanf(pf, "%d%ld%ld%ld%ld%ld%ld", &nPointNo, &tAnglePulse.nSPulse, &tAnglePulse.nLPulse, &tAnglePulse.nUPulse, &tAnglePulse.nRPulse, &tAnglePulse.nBPulse, &tAnglePulse.nTPulse) > 0)
     {
         vtRobotAnglePulses.push_back(tAnglePulse);
     }
     fclose(pf);

     double dXAdjustSum = 4.0;
     double dYAdjustSum = 4.0;
     double dZAdjustSum = 4.0;

     double dXAdjustUnit = 0.2;
     double dYAdjustUnit = 0.2;
     double dZAdjustUnit = 0.2;

     double dXAdjust = 0.0;
     double dYAdjust = 0.0;
     double dZAdjust = 0.0;

     double dMinDis = 999999.0;
     double dMinXAdjust = 0.0;
     double dMinYAdjust = 0.0;
     double dMinZAdjust = 0.0;
     double dDis = 0.0;

     CAbsPositionCalcAdaptor cAbsPosAdaptor(pRobotCtrl->m_strRobotName);

     for (int nXNo = 0; nXNo < int((2 * dXAdjustSum) / dXAdjustUnit); nXNo++)
     {
         for (int nYNo = 0; nYNo < int((2 * dYAdjustSum) / dYAdjustUnit); nYNo++)
         {
             for (int nZNo = 0; nZNo < int((2 * dZAdjustSum) / dZAdjustUnit); nZNo++)
             {
                 dXAdjust = -dXAdjustSum + dXAdjustUnit*nXNo;
                 dYAdjust = -dYAdjustSum + dYAdjustUnit*nYNo;
                 dZAdjust = -dZAdjustSum + dZAdjustUnit*nZNo;

                 vtAbsPosInBase.clear();
                 for (int nPointNo = 0; nPointNo < nPointNum; nPointNo++)
                 {
                     XI_POINT tKeyPoint3D;
                     XI_POINT tCamCenter3D;

                     XiLaserLightParamNode tLaserLightParam;

                     int nRobotId = eCamId / 4;
                     int nCamId = eCamId % 4;

                     tLaserLightParam.l_dx = cAbsPosAdaptor.m_ptCamera[nRobotId][nCamId].l_dx;
                     tLaserLightParam.l_dy = cAbsPosAdaptor.m_ptCamera[nRobotId][nCamId].l_dy;
                     tLaserLightParam.l_f = cAbsPosAdaptor.m_ptCamera[nRobotId][nCamId].l_f;
                     tLaserLightParam.l_len = cAbsPosAdaptor.m_ptCamera[nRobotId][nCamId].l_len;
                     tLaserLightParam.l_ctan = cAbsPosAdaptor.m_ptCamera[nRobotId][nCamId].l_ctan;

                     CvPoint3D64f cptKeyPnt3D = m_vpDHCameraVision[nRobotNo]->CamId2ImageProcess(eCamId)->m_pXiCvObj->KeyPoint2DTo3D(vtImagePoint[nPointNo], 1, tLaserLightParam);
                     tKeyPoint3D.x = cptKeyPnt3D.x;
                     tKeyPoint3D.y = cptKeyPnt3D.y;
                     tKeyPoint3D.z = cptKeyPnt3D.z;

                     CvPoint      tCamCentrePnt = cvPoint(m_vpDHCameraVision[nRobotNo]->CamId2ImageProcess(eCamId)->m_nWidth /2, m_vpDHCameraVision[nRobotNo]->CamId2ImageProcess(eCamId)->m_nHeight /2);
                     CvPoint3D64f cpCamCenterPnt3D = m_vpDHCameraVision[nRobotNo]->CamId2ImageProcess(eCamId)->m_pXiCvObj->KeyPoint2DTo3D(tCamCentrePnt, 1, tLaserLightParam);
                     tCamCenter3D.x = cpCamCenterPnt3D.x;
                     tCamCenter3D.y = cpCamCenterPnt3D.y;
                     tCamCenter3D.z = cpCamCenterPnt3D.z;

                     XI_POINT tCornerPoint;

                     tCornerPoint.x = (tKeyPoint3D.x - tCamCenter3D.x) * (1.0);
                     tCornerPoint.y = (tKeyPoint3D.y - tCamCenter3D.y) * (1.0);
                     tCornerPoint.z = (tKeyPoint3D.z - tCamCenter3D.z) * (1.0);

                     cAbsPosAdaptor.m_cXiRobotAbsCoorsTrans->GetCaliInfo
                         (
                             cAbsPosAdaptor.m_ptCameraReadyRobotCoors[nRobotId][nCamId],
                             cAbsPosAdaptor.m_ptCameraReadyRobotPulse[nRobotId][nCamId],
                             cAbsPosAdaptor.m_ptCameraReadyRobotTheta[nRobotId][nCamId],
                             cAbsPosAdaptor.m_ptHandEyeParam[nRobotId][nCamId].dDx + dXAdjust,
                             cAbsPosAdaptor.m_ptHandEyeParam[nRobotId][nCamId].dDy + dYAdjust,
                             cAbsPosAdaptor.m_ptHandEyeParam[nRobotId][nCamId].dDz + dZAdjust,
                             cAbsPosAdaptor.m_ptHandEyeParam[nRobotId][nCamId].dCenterDisUToBase
                             );

                     tPointAbsCoordInBase = cAbsPosAdaptor.m_cXiRobotAbsCoorsTrans->MeasureWeldLinePointInBase(tCornerPoint, vtRobotMeasureCoors[nPointNo], vtRobotAnglePulses[nPointNo], eCamId, pRobotCtrl->m_eManipulatorType);

                     vtAbsPosInBase.push_back(tPointAbsCoordInBase);
                 }

                 dDis = 0.0;
                 for (int nPointNo = 0; nPointNo < nPointNum; nPointNo++)
                 {
                     dDis += sqrt(SQUARE(vtRobotBaseCoors[nPointNo].dX - vtAbsPosInBase[nPointNo].tWeldLinePos.x) +
                         SQUARE(vtRobotBaseCoors[nPointNo].dY - vtAbsPosInBase[nPointNo].tWeldLinePos.y) +
                         SQUARE(vtRobotBaseCoors[nPointNo].dZ - vtAbsPosInBase[nPointNo].tWeldLinePos.z));
                 }

                 if (dDis < dMinDis)
                 {
                     dMinDis = dDis;
                     dMinXAdjust = dXAdjust;
                     dMinYAdjust = dYAdjust;
                     dMinZAdjust = dZAdjust;
                 }
             }
         }
     }

     XiMessageBox("%11.3lf%11.3lf%11.3lf%11.3lf", dMinDis, dMinXAdjust, dMinYAdjust, dMinZAdjust);
     WriteLog("手眼校验数据：%11.3lf%11.3lf%11.3lf%11.3lf", dMinDis, dMinXAdjust, dMinYAdjust, dMinZAdjust);

     vtAbsPosInBase.clear();
     for (nPointNo = 0; nPointNo < nPointNum; nPointNo++)
     {
         XI_POINT tKeyPoint3D;
         XI_POINT tCamCenter3D;

         XiLaserLightParamNode tLaserLightParam;

         int nRobotId = eCamId / 4;
         int nCamId = eCamId % 4;

         tLaserLightParam.l_dx = cAbsPosAdaptor.m_ptCamera[nRobotId][nCamId].l_dx;
         tLaserLightParam.l_dy = cAbsPosAdaptor.m_ptCamera[nRobotId][nCamId].l_dy;
         tLaserLightParam.l_f = cAbsPosAdaptor.m_ptCamera[nRobotId][nCamId].l_f;
         tLaserLightParam.l_len = cAbsPosAdaptor.m_ptCamera[nRobotId][nCamId].l_len;
         tLaserLightParam.l_ctan = cAbsPosAdaptor.m_ptCamera[nRobotId][nCamId].l_ctan;

         CvPoint3D64f cptKeyPnt3D = m_vpDHCameraVision[nRobotNo]->CamId2ImageProcess(eCamId)->m_pXiCvObj->KeyPoint2DTo3D(vtImagePoint[nPointNo], 1, tLaserLightParam);
         tKeyPoint3D.x = cptKeyPnt3D.x;
         tKeyPoint3D.y = cptKeyPnt3D.y;
         tKeyPoint3D.z = cptKeyPnt3D.z;

         CvPoint      tCamCentrePnt = cvPoint(m_vpDHCameraVision[nRobotNo]->CamId2ImageProcess(eCamId)->m_nWidth / 2, m_vpDHCameraVision[nRobotNo]->CamId2ImageProcess(eCamId)->m_nHeight / 2);
         CvPoint3D64f cpCamCenterPnt3D = m_vpDHCameraVision[nRobotNo]->CamId2ImageProcess(eCamId)->m_pXiCvObj->KeyPoint2DTo3D(tCamCentrePnt, 1, tLaserLightParam);
         tCamCenter3D.x = cpCamCenterPnt3D.x;
         tCamCenter3D.y = cpCamCenterPnt3D.y;
         tCamCenter3D.z = cpCamCenterPnt3D.z;

         XI_POINT tCornerPoint;

         tCornerPoint.x = (tKeyPoint3D.x - tCamCenter3D.x) * (1.0);
         tCornerPoint.y = (tKeyPoint3D.y - tCamCenter3D.y) * (1.0);
         tCornerPoint.z = (tKeyPoint3D.z - tCamCenter3D.z) * (1.0);

         cAbsPosAdaptor.m_cXiRobotAbsCoorsTrans->GetCaliInfo
             (
                 cAbsPosAdaptor.m_ptCameraReadyRobotCoors[nRobotId][nCamId],
                 cAbsPosAdaptor.m_ptCameraReadyRobotPulse[nRobotId][nCamId],
                 cAbsPosAdaptor.m_ptCameraReadyRobotTheta[nRobotId][nCamId],
                 cAbsPosAdaptor.m_ptHandEyeParam[nRobotId][nCamId].dDx + dMinXAdjust,
                 cAbsPosAdaptor.m_ptHandEyeParam[nRobotId][nCamId].dDy + dMinYAdjust,
                 cAbsPosAdaptor.m_ptHandEyeParam[nRobotId][nCamId].dDz + dMinZAdjust,
                 cAbsPosAdaptor.m_ptHandEyeParam[nRobotId][nCamId].dCenterDisUToBase
                 );

         tPointAbsCoordInBase = cAbsPosAdaptor.m_cXiRobotAbsCoorsTrans->MeasureWeldLinePointInBase(tCornerPoint, vtRobotMeasureCoors[nPointNo], vtRobotAnglePulses[nPointNo], eCamId, pRobotCtrl->m_eManipulatorType);

         vtAbsPosInBase.push_back(tPointAbsCoordInBase);
     }

     dDis = 0.0;
     std::vector<double> vdDis;
     vdDis.clear();
     CString strResult;
     strResult.Format("%s_result.txt", strName);
     FILE *Result = fopen(strResult.GetBuffer(0), "w");
     for (nPointNo = 0; nPointNo < nPointNum; nPointNo++)
     {
         dDis = sqrt(SQUARE(vtRobotBaseCoors[nPointNo].dX - vtAbsPosInBase[nPointNo].tWeldLinePos.x) +
             SQUARE(vtRobotBaseCoors[nPointNo].dY - vtAbsPosInBase[nPointNo].tWeldLinePos.y) +
             SQUARE(vtRobotBaseCoors[nPointNo].dZ - vtAbsPosInBase[nPointNo].tWeldLinePos.z));
         vdDis.push_back(dDis);

         fprintf(Result, "%4d%11.3lf%11.3lf%11.3lf\n", nPointNo, vtAbsPosInBase[nPointNo].tWeldLinePos.x, vtAbsPosInBase[nPointNo].tWeldLinePos.y, vtAbsPosInBase[nPointNo].tWeldLinePos.z);
     }
     fclose(Result);
#endif
 }

 void CAssemblyWeld::TestContiSwaySpot()
 {
	 //// 连续摆动点焊(可不摆动) 运动顺序 位置1 -》位置2 -》位置3 -》抬高位置1 -》抬高位置2 -》抬高位置3 ……
	 //// I003：Job内开头修改：起弧开关(默认0不起弧) 1:起弧   0:不起弧
	 //// I004：移动速度(单位:6mm/min)
	 //// I005：电流 (A)
	 //// I006：电压 (V)
	 //// I007：点焊时间(单位：0.01s) 
	 //// I008：相邻摆动循环抬高距离(单位：0.1mm)
	 //// P110：焊接起点 
	 //// P111：焊接终点 
	 //// P112：位置1偏移量 
	 //// P113：位置2偏移量 
	 //// P114：位置3偏移量   
	 ////	三个偏移位置说明：X:干伸长  Y:左右摆幅 Z:相对于中间点的抬高距离 Rz:左右摆动Rz变化值
	 //// 	X:干伸长度(mm)：增加向焊缝外移动 减小向焊缝里移动
	 ////		Y:摆动幅度(mm)：在焊丝角度看 负数:向左摆动 正数:向右摆动
	 ////		Z:抬高距离(mm)：一个摆动循环内 各位置相对与基准位置的高度偏移
	 ////		Rz：摆动同时Rz变化(°)：建议 向左摆动(Y值正数)Rz为正 向右摆动(Y值负数)Rz为负
	 //
	 //int nRobotNo = 0;
	 //CUnit* pUnit = m_vpUnit[nRobotNo];
	 //CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];

	 //double dLoopUpDis = 0.0;
	 //T_ROBOT_COORS tStartCoord; // P110
	 //T_ROBOT_COORS tEndCoord;	// P111
	 //T_ROBOT_COORS tOffset1;	// P112	 
	 //T_ROBOT_COORS tOffset2;	// P113
	 //T_ROBOT_COORS tOffset3;	// P114

	 //int nI008 = pRobotDriver->GetIntVar(8);
	 //dLoopUpDis = (double)nI008 / 10.0;
	 //double adRobotPos[5][6] = { 0 };
	 //UINT *pToolNo = new UINT;
	 //UINT *pUserNo = new UINT;
	 //UINT *pPosture = new UINT;
	 //pRobotDriver->GetMultiPosVar(5, 110, adRobotPos, pToolNo, pUserNo, pPosture);
	 //tStartCoord.dX = adRobotPos[0][0] ; // / 1000.0;
	 //tStartCoord.dY = adRobotPos[0][1] ; // / 1000.0;
	 //tStartCoord.dZ = adRobotPos[0][2] ; // / 1000.0;
	 //tStartCoord.dRX = adRobotPos[0][3]; //  / 1000.0;
	 //tStartCoord.dRY = adRobotPos[0][4]; //  / 1000.0;
	 //tStartCoord.dRZ = adRobotPos[0][5]; //  / 1000.0;
	 //tEndCoord.dX = adRobotPos[1][0]; /// 1000.0;
	 //tEndCoord.dY = adRobotPos[1][1]; /// 1000.0;
	 //tEndCoord.dZ = adRobotPos[1][2]; /// 1000.0;
	 //tEndCoord.dRX = adRobotPos[1][3]; // / 1000.0;
	 //tEndCoord.dRY = adRobotPos[1][4]; // / 1000.0;
	 //tEndCoord.dRZ = adRobotPos[1][5]; // / 1000.0;
	 //tOffset1.dX = adRobotPos[2][0]; /// 1000.0;
	 //tOffset1.dY = adRobotPos[2][1]; /// 1000.0;
	 //tOffset1.dZ = adRobotPos[2][2]; /// 1000.0;
	 //tOffset1.dRX = adRobotPos[2][3]; // / 1000.0;
	 //tOffset1.dRY = adRobotPos[2][4]; // / 1000.0;
	 //tOffset1.dRZ = adRobotPos[2][5]; // / 1000.0;
	 //tOffset2.dX = adRobotPos[3][0]; /// 1000.0;
	 //tOffset2.dY = adRobotPos[3][1]; /// 1000.0;
	 //tOffset2.dZ = adRobotPos[3][2]; /// 1000.0;
	 //tOffset2.dRX = adRobotPos[3][3]; // / 1000.0;
	 //tOffset2.dRY = adRobotPos[3][4]; // / 1000.0;
	 //tOffset2.dRZ = adRobotPos[3][5]; // / 1000.0;
	 //tOffset3.dX = adRobotPos[4][0]; /// 1000.0;
	 //tOffset3.dY = adRobotPos[4][1]; /// 1000.0;
	 //tOffset3.dZ = adRobotPos[4][2]; /// 1000.0;
	 //tOffset3.dRX = adRobotPos[4][3]; // / 1000.0;
	 //tOffset3.dRY = adRobotPos[4][4]; // / 1000.0;
	 //tOffset3.dRZ = adRobotPos[4][5]; // / 1000.0;

	 //double dNorAngle = pRobotDriver->RzToDirAngle(tStartCoord.dRZ);
	 //double dWeldSeamLen = TwoPointDis(tStartCoord.dX, tStartCoord.dY, tStartCoord.dZ, tEndCoord.dX, tEndCoord.dY, tEndCoord.dZ);
	 //double dDisX = tEndCoord.dX - tStartCoord.dX;
	 //double dDisY = tEndCoord.dY - tStartCoord.dY;
	 //double dDisZ = tEndCoord.dZ - tStartCoord.dZ;
	 //int nPtnNum = dWeldSeamLen / dLoopUpDis;
	 //dLoopUpDis = dWeldSeamLen / (double)nPtnNum;
	 //T_ROBOT_COORS tCoord;
	 //T_ROBOT_COORS tTemp;
	 //vector<T_ROBOT_COORS> vtWeldCoord(0);
	 //vector<T_ANGLE_PULSE> vtWeldPulse(0);
	 //for (int i = 0; i <= nPtnNum; i++)
	 //{
		// // Base Coord
		// tCoord = tStartCoord;
		// tCoord.dX += ((double)i * dDisX / (double)nPtnNum);
		// tCoord.dY += ((double)i * dDisY / (double)nPtnNum);
		// tCoord.dZ += ((double)i * dDisZ / (double)nPtnNum)/* + (double)i * dLoopUpDis*/;

		// // Offset1
		// tTemp = tCoord;
		// tTemp.dX += tOffset1.dX * CosD(dNorAngle);
		// tTemp.dY += tOffset1.dX * SinD(dNorAngle);
		// tTemp.dX += tOffset1.dY * CosD(dNorAngle - 90.0);
		// tTemp.dY += tOffset1.dY * SinD(dNorAngle - 90.0);
		// tTemp.dZ += tOffset1.dZ;
		// tTemp.dRZ += tOffset1.dRZ;
		// vtWeldCoord.push_back(tTemp);
		// 
		// // Offset2
		// tTemp = tCoord;
		// tTemp.dX += tOffset2.dX * CosD(dNorAngle);
		// tTemp.dY += tOffset2.dX * SinD(dNorAngle);
		// tTemp.dX += tOffset2.dY * CosD(dNorAngle - 90.0);
		// tTemp.dY += tOffset2.dY * SinD(dNorAngle - 90.0);
		// tTemp.dZ += tOffset2.dZ;
		// tTemp.dRZ += tOffset2.dRZ;
		// vtWeldCoord.push_back(tTemp);
		// 
		// // Offset3
		// tTemp = tCoord;
		// tTemp.dX += tOffset3.dX * CosD(dNorAngle);
		// tTemp.dY += tOffset3.dX * SinD(dNorAngle);
		// tTemp.dX += tOffset3.dY * CosD(dNorAngle - 90.0);
		// tTemp.dY += tOffset3.dY * SinD(dNorAngle - 90.0);
		// tTemp.dZ += tOffset3.dZ;
		// tTemp.dRZ += tOffset3.dRZ;
		// vtWeldCoord.push_back(tTemp);
	 //}
	 //CHECK_BOOL(CreateObject(m_tChoseWorkPieceType, pUnit, &m_pWeldAfterMeasure));

	 //m_pWeldAfterMeasure->CalcContinuePulseForWeld(vtWeldCoord, vtWeldPulse, true);

	 //m_pWeldAfterMeasure->GenerateJobSpotWeld(vtWeldPulse, MOVL, "CONTI_SWAY_SPOT");

 }

 bool CAssemblyWeld::WeldTrackLineFilter(std::vector<T_ROBOT_COORS>& vtCoord, int nSingleTimePtnNum, int nSampleInterval)
 {
	 XiAlgorithm alg;
	 T_ALGORITHM_POINT tAlgPtn;
	 std::vector<T_ROBOT_COORS> vtCoordNew(0);
	 T_ROBOT_COORS tCoord;
	 int nPtnNum = vtCoord.size();
	 if (nPtnNum < nSingleTimePtnNum)
	 {
		 XiMessageBoxOk("滤波输入点数小于单次滤波点数，滤波失败");
		 return false;
	 }
	 // 每次取dSingleTimePtnNum个点 采样间隔nSampleInterval  执行滤波 
	 for (int i = nSingleTimePtnNum; i < nPtnNum; i += nSingleTimePtnNum)
	 {
		 int nIdxS = i - nSingleTimePtnNum;
		 int nIdxE = i;
		 if (nIdxE + nSingleTimePtnNum > nPtnNum)
		 {
			 nIdxE = nPtnNum;
		 }

		 std::vector<T_ALGORITHM_POINT> vtAlgPtns(0);
		 std::vector<T_ALGORITHM_POINT> vtOutPutAlgPtns(0);
		 for (int i = nIdxS; i < nIdxE; i += nSampleInterval)
		 {
			 tAlgPtn.dCoorX = vtCoord[i].dX;
			 tAlgPtn.dCoorY = vtCoord[i].dBY;
			 tAlgPtn.dCoorZ = vtCoord[i].dZ;
			 vtAlgPtns.push_back(tAlgPtn);
		 }
		 if (nIdxE == nPtnNum)
		 {
			 tAlgPtn.dCoorX = vtCoord.back().dX;
			 tAlgPtn.dCoorY = vtCoord.back().dBY;
			 tAlgPtn.dCoorZ = vtCoord.back().dZ;
			 vtAlgPtns.push_back(tAlgPtn);
		 }

		 alg.PointSmooth(vtAlgPtns, E_LINE, 0.8, vtOutPutAlgPtns);
		 tCoord = vtCoord.front();
		 for (int i = 0; i < vtOutPutAlgPtns.size(); i++)
		 {
			 tCoord.dX  = vtOutPutAlgPtns[i].dCoorX;
			 tCoord.dBY = vtOutPutAlgPtns[i].dCoorY;
			 tCoord.dZ  = vtOutPutAlgPtns[i].dCoorZ;
			 vtCoordNew.push_back(tCoord);
		 }
	 }
	 FILE* pf = fopen("A_Test.txt", "w");
	 vtCoord.clear();
	 vtCoord.push_back(vtCoordNew.front());
	 for (int nPtnNo = 0; nPtnNo + 1 < vtCoordNew.size(); nPtnNo++)
	 {

		 T_ROBOT_COORS tCoor1 = vtCoordNew[nPtnNo];
		 T_ROBOT_COORS tCoor2 = vtCoordNew[nPtnNo + 1];
		 double dDisX = tCoor2.dX - tCoor1.dX;
		 double dDisY = tCoor2.dBY - tCoor1.dBY;
		 double dDisZ = tCoor2.dZ - tCoor1.dZ;
		 for (int i = 1; i <= nSampleInterval; i++)
		 {
			 T_ROBOT_COORS tCoor = tCoor1;
			 tCoor.dX += (dDisX * (double)i / (double)nSampleInterval);
			 tCoor.dBY += (dDisY * (double)i / (double)nSampleInterval);
			 tCoor.dZ += (dDisZ * (double)i / (double)nSampleInterval);
			 fprintf(pf, "%d %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf\n", nPtnNo,
				 tCoor.dX, tCoor.dY, tCoor.dZ, tCoor.dRX, tCoor.dRY, tCoor.dRZ, tCoor.dBX, tCoor.dBY, tCoor.dBZ);
			 vtCoord.push_back(tCoor);
		 }
	 }
	 fclose(pf);
	 return true;
 }
 

 void CAssemblyWeld::TestGetEndPtnUsePointCloud_F()
{
	CString sPointCloudFile;
	CString sRecoResultFile;
	CString sSaveResultFile;
	sPointCloudFile = OpenFileDlg(this, _T("*"), "./");
	//sPointCloudFile = UnicodeToUtf8(sPointCloudFile.GetBuffer());
	int nIdx = sPointCloudFile.ReverseFind('.');
	CString s1 = sPointCloudFile.Left(nIdx);
	sRecoResultFile.Format("%s_Rst.txt", /*UnicodeToUtf8*/(s1.GetBuffer()));
	sSaveResultFile.Format("%s_Save.txt", /*UnicodeToUtf8*/(s1.GetBuffer()));

	RiserEndPointInfo tWeldInfo[3] = { 0 };
	CvPoint3D32f tRefPtn[2] = { 6.5, -1436, -467, 6.5, -1436, -376 };
	CvPoint3D32f tCameraNorm = cvPoint3D32f(-1.0, -1.0, -1.0);
	CvPoint3D32f tPlaneHNorm = cvPoint3D32f(0.0, 0.0, 1.0);

	// 弹窗标题
	CString cStrTitle = "输入参数";
	// 参数的中文名称
	std::vector<CString> vsInputName;
	vsInputName.push_back("法向X");
	vsInputName.push_back("法向Y");
	vsInputName.push_back("法向Z");	
	vsInputName.push_back("1正座-1倒挂");	
	std::vector<double> vnInputData;
	vnInputData.push_back(tCameraNorm.x);
	vnInputData.push_back(tCameraNorm.y);
	vnInputData.push_back(tCameraNorm.z);
	vnInputData.push_back(m_vpRobotDriver[0]->m_nRobotInstallDir);
	ParamInput cParamDlg(cStrTitle, vsInputName, &vnInputData);
	cParamDlg.DoModal();
	tCameraNorm.x = vnInputData.at(0);
	tCameraNorm.y = vnInputData.at(1);
	tCameraNorm.z = vnInputData.at(2);
	tPlaneHNorm.z = vnInputData.at(3);

	vsInputName.clear();
	vsInputName.push_back("参考点1_X");
	vsInputName.push_back("参考点1_Y");
	vsInputName.push_back("参考点1_Z");
	vsInputName.push_back("参考点2_X");
	vsInputName.push_back("参考点2_Y");
	vsInputName.push_back("参考点2_Z");

	vnInputData.clear();
	vnInputData.push_back(tRefPtn[0].x);
	vnInputData.push_back(tRefPtn[0].y);
	vnInputData.push_back(tRefPtn[0].z);
	vnInputData.push_back(tRefPtn[1].x);
	vnInputData.push_back(tRefPtn[1].y);
	vnInputData.push_back(tRefPtn[1].z);

	ParamInput cParamDlg2(cStrTitle, vsInputName, &vnInputData);
	cParamDlg2.DoModal();
	tRefPtn[0].x = vnInputData.at(0);
	tRefPtn[0].y = vnInputData.at(1);
	tRefPtn[0].z = vnInputData.at(2);
	tRefPtn[1].x = vnInputData.at(3);
	tRefPtn[1].y = vnInputData.at(4);
	tRefPtn[1].z = vnInputData.at(5);
	
	CvPoint3D64f tmpCloudPt;
	int x;
	vector<CvPoint3D64f> vtCloudPt;
	FILE* pf = fopen(sPointCloudFile, "r");
	while (~fscanf(pf, "%d%lf%lf%lf", &x, &tmpCloudPt.x, &tmpCloudPt.y, &tmpCloudPt.z))
	{
		vtCloudPt.push_back(tmpCloudPt);
	}

	int nWeldInfoNum = GetRiserEndPoint(vtCloudPt.data(), vtCloudPt.size(), tWeldInfo, tCameraNorm, tPlaneHNorm, tRefPtn, 2, "GetRiserEndPoint_Stand");
	fclose(pf);

	pf = fopen(sRecoResultFile.GetBuffer(), "w");
	for (int i = 0; i < nWeldInfoNum; i++)
	{
		fprintf(pf, "%d %4d%4d%11.3lf%4d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%4d 0 0 0 %4d%11.3lf%11.3lf 0\n",
			i, tWeldInfo[i].GrooveID, 0, tWeldInfo[i].GrooveWidth, -1 == tWeldInfo[i].GrooveType ? true : false,
			0.0, 0.0, 0.0,
			tWeldInfo[i].staPnt.x, tWeldInfo[i].staPnt.y, tWeldInfo[i].staPnt.z,
			tWeldInfo[i].endPnt.x, tWeldInfo[i].endPnt.y, tWeldInfo[i].endPnt.z,
			tWeldInfo[i].normal.x, tWeldInfo[i].normal.y, tWeldInfo[i].normal.z,
			tWeldInfo[i].normal.x, tWeldInfo[i].normal.y, tWeldInfo[i].normal.z,
			tWeldInfo[i].staPntWeldType, tWeldInfo[i].endPntWeldType, 
			0, tWeldInfo[i].staPntRA_Radius, tWeldInfo[i].endPntRA_Radius);
	}
	fclose(pf);

	RunInteractiveWindow(UnicodeToUtf8(sPointCloudFile.GetBuffer()), UnicodeToUtf8(sSaveResultFile.GetBuffer()), UnicodeToUtf8(sRecoResultFile.GetBuffer()));
	return;
}

void CAssemblyWeld::TestGetPicturePtnToPointCloud()
{
	//CString sDefaultPath = "./";
	//sFileName = OpenFileDlg(this, _T("*"), sDefaultPath);
	//CvPoint *Cpoints = new CvPoint[20000], *pointPtr = Cpoints;
	//int size = FindLaserMidPiontEE(pImageBuff, Cpoints, 3, 30, 30, 5, 11, 0, 20000, 3, 3, 2, false);
	
//	CString sFileName;
//	int PictureNum = 19; double Ynum = 0; int n = -1;
//	CRobotDriverAdaptor* m_pRobotDriver = m_vpRobotDriver[0];
//	E_CAM_ID eCamId = E_LEFT_ROBOT_LEFT_CAM_H;
//	T_ABS_POS_IN_BASE tPointAbsCoordInBase;
//	sFileName.Format(".\\WeldData\\vtPoints.txt");
//	FILE* pf = fopen(sFileName.GetBuffer(0), "w");
//	for (PictureNum; PictureNum <= 185; PictureNum++)
//	{
//		sFileName.Format(".\\LeftGun_LeftCam\\Src\\End\\ForwardScan\\%d.jpg", PictureNum);
//		IplImage* pImage = cvLoadImage(sFileName, 0);
//		CvPoint* Cpoints = new CvPoint[20000], * pointPtr = Cpoints;
//		unsigned short winsize = 3;
//		uchar binthreshold = 100;
//		uchar refbright = 100;
//		unsigned short combintimes = 4;
//		unsigned short conbinsize = 7;
//		float GaussCoeff = 3.0F;
//		float compensate = 10;
//		unsigned short maxpnts = 20000;
//		float accX = 5;
//		float accY = 3;
//		int mode = 2;
//		bool debugswitch = false;
//		int size = FindLaserMidPiontEE(pImage, Cpoints, winsize, binthreshold, refbright, combintimes, conbinsize, GaussCoeff, compensate, maxpnts, accX, accY, mode, debugswitch);
//		for (int i = 0; i < size; i++)
//		{
//			T_ROBOT_COORS tRobotCoors;
//			tRobotCoors.dX = 0; tRobotCoors.dY = 0; tRobotCoors.dZ = 1800;
//			tRobotCoors.dRX = 0; tRobotCoors.dRY = -45; tRobotCoors.dRZ = 0;
//			T_ANGLE_PULSE tRobotPulse;
//			m_vpRobotDriver[0]->RobotInverseKinematics(tRobotCoors, m_vpRobotDriver[0]->m_tHomePulse, m_vpRobotDriver[0]->m_tTools.tGunTool, tRobotPulse);
//			m_vpDHCameraVision[0]->GetMeasurePosInBaseNew(m_vpRobotDriver[0]->m_strRobotName, Cpoints[i], eCamId, tPointAbsCoordInBase,
//				tRobotCoors, tRobotPulse, m_vpRobotDriver[0]->m_eManipulatorType);
//			tPointAbsCoordInBase.tWeldLinePos.y += Ynum;
//			fprintf(pf, "%d %11.3lf%11.3lf%11.3lf\n", n += 1, tPointAbsCoordInBase.tWeldLinePos.x, tPointAbsCoordInBase.tWeldLinePos.y, tPointAbsCoordInBase.tWeldLinePos.z);
//		}
//		Ynum += 0.7;
//	}
//	fclose(pf);
	return;
}

void CAssemblyWeld::TestScanEndptnImageProcess()
{
//	CString sDefaultPath = "./";
//	CString sFileName;
//	double dImageIntervalDis = 0.7;
//	double dRobotInstallMode = -1.0; // 正装：1.0   倒装：-1.0
//	CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[0];
//	E_CAM_ID eCamId = E_LEFT_ROBOT_LEFT_CAM_H;
//	T_ABS_POS_IN_BASE tPointAbsCoordInBase;
//	sFileName = OpenFileDlg(this, _T("*"), sDefaultPath);
//	CString sRootPath = sFileName.Left(sFileName.ReverseFind('\\'));
//
//	// 开始图片索引 选择文件
//	CString sIdxS = sFileName.Right(sFileName.GetLength() - 1 - sFileName.ReverseFind('('));
//	sIdxS = sIdxS.Left(sIdxS.Find(')'));
//	int nIdxS = atoi(sIdxS.GetBuffer());
//	int nIdxE = 1000;
//	
//	// 结束图片索引 输入
//	CString cStrTitle = "结束图片索引";
//	// 参数的中文名称
//	std::vector<CString> vsInputName(1, "结束索引");
//	// 实际参数数值
//	std::vector<double> vnInputData(1, (double)nIdxE);
//	// 修改识别参数窗口
//	ParamInput cParamDlg(cStrTitle, vsInputName, &vnInputData);
//	int nRst = cParamDlg.DoModal();
//	nIdxE = vnInputData[0];
//
//	// 创建一个处理图文件夹
//	CString sProImagePath = sRootPath.Left(sRootPath.ReverseFind('\\'));
//	CString sOrgFolder = sRootPath.Right(sRootPath.GetLength() - 1 - sRootPath.ReverseFind('\\'));
//	sProImagePath.Format("%s\\%sPro", sProImagePath, sOrgFolder);
//	CheckFolder(sProImagePath);
//
//	CString sPointCloudFileName;
//	sPointCloudFileName.Format("%s\\_vtPoints.txt", sProImagePath.GetBuffer());
//	FILE* pf = fopen(sPointCloudFileName.GetBuffer(0), "w");
//
//
//	sFileName.Format("%s\\1 (%d).jpg", sRootPath.GetBuffer(), nIdxS);
//	SetHintInfo(sFileName);
//	IplImage* pImage = cvLoadImage(sFileName, 0);
//	if (NULL == pImage) return;
//	IplImage* pColorImg = cvCreateImage(cvSize(pImage->width, pImage->height), IPL_DEPTH_8U, 3);
//	cvReleaseImage(&pImage);
//
//	for (int PictureNum = nIdxS; PictureNum <= nIdxE; PictureNum++)
//	{
//		sFileName.Format("%s\\1 (%d).jpg", sRootPath.GetBuffer(), PictureNum);
//		SetHintInfo(sFileName);
//		IplImage* pImage = cvLoadImage(sFileName, 0);
//		if (NULL == pImage) break;
//		CvPoint* Cpoints = new CvPoint[20000];
//		unsigned short winsize = 3;
//		uchar binthreshold = 40;
//		uchar refbright = 40;
//		unsigned short combintimes = 4;
//		unsigned short conbinsize = 15;
//		float GaussCoeff = 3.0F;
//		float compensate = -10;
//		unsigned short maxpnts = 20000;
//		float accX = 3;
//		float accY = 1;
//		int mode = 2;
//		bool debugswitch = false;
//		int size = FindLaserMidPiontEE(pImage, Cpoints, winsize, binthreshold, refbright, combintimes, conbinsize, GaussCoeff, compensate, maxpnts, accX, accY, mode, debugswitch);
//
//		cvCvtColor(pImage, pColorImg, CV_GRAY2RGB);
//		for (int i = 0; i < size; i++)
//		{
//			cvCircle(pColorImg, Cpoints[i], 5, CV_RGB(255, 0, 0), 1);
//		}
//		sFileName.Format("%s\\Pro_%d.jpg", sProImagePath, PictureNum);
//		ShowTeachImage(pColorImg);
//		SaveImage(pColorImg, sFileName);
//		cvReleaseImage(&pImage);
//
//		for (int i = 0; i < size; i += 1/*LINE_SCAN_STEP_2D_POINT*/)
//		{
//			T_ROBOT_COORS tRobotCoors;
//			tRobotCoors.dX = 0; tRobotCoors.dY = 0; tRobotCoors.dZ = 1800;
//			tRobotCoors.dRX = 0; tRobotCoors.dRY = 45* dRobotInstallMode; tRobotCoors.dRZ = 180;
//			T_ANGLE_PULSE tRobotPulse;
//			pRobotDriver->RobotInverseKinematics(tRobotCoors, pRobotDriver->m_tHomePulse, pRobotDriver->m_tTools.tGunTool, tRobotPulse);
//			m_vpDHCameraVision[0]->GetMeasurePosInBaseNew(pRobotDriver->m_strRobotName, Cpoints[i], eCamId, tPointAbsCoordInBase,
//				tRobotCoors, tRobotPulse, pRobotDriver->m_eManipulatorType);
//			tPointAbsCoordInBase.tWeldLinePos.y += (dImageIntervalDis * (PictureNum - nIdxS));
//			fprintf(pf, "%d %11.3lf%11.3lf%11.3lf\n", PictureNum, tPointAbsCoordInBase.tWeldLinePos.x, tPointAbsCoordInBase.tWeldLinePos.y, tPointAbsCoordInBase.tWeldLinePos.z);
//		}
//	}
//	cvReleaseImage(&pColorImg);
//	fclose(pf);
//	XiMessageBoxOk("done");
	return;
}

bool CAssemblyWeld::BackHome_G()
{
	std::vector<CWinThread*> vpThread(0);
	if (1 == XiMessageBox("机器人1回安全位置？"))
	{
		T_ROBOT_THREAD* tRobot = new T_ROBOT_THREAD();
		tRobot->cIncisePlanePart = this;
		tRobot->pRobotCtrl = m_vpUnit[0]->GetRobotCtrl();
		CWinThread* pThread = AfxBeginThread(ThreadBackHome, tRobot);
		pThread->m_bAutoDelete = false;
		vpThread.push_back(pThread);
	}
	if (1 == XiMessageBox("机器人2回安全位置？"))
	{
		T_ROBOT_THREAD* tRobot = new T_ROBOT_THREAD();
		tRobot->cIncisePlanePart = this;
		tRobot->pRobotCtrl = m_vpUnit[1]->GetRobotCtrl();
		CWinThread* pThread = AfxBeginThread(ThreadBackHome, tRobot);
		pThread->m_bAutoDelete = false;
		vpThread.push_back(pThread);
	}

	bool bSuccess = WaitAndCheckAllThreadExit(vpThread);
	if (!bSuccess)
	{
		//已修改
		//SetHintInfo("回安全位置失败！");
		SetHintInfo(XUI::Languge::GetInstance().translate("回安全位置失败!"));//"回安全位置失败！"
	}
	else
	{
		//已修改
		//SetHintInfo("回安全位置完成！");
		SetHintInfo(XUI::Languge::GetInstance().translate("回安全位置完成!"));//"回安全位置失败！"
	};
	return bSuccess;
}

UINT CAssemblyWeld::ThreadBackHome(void* pParam)
{
	
	T_ROBOT_THREAD* pObj = (T_ROBOT_THREAD*)pParam;
	return pObj->cIncisePlanePart->FuncBackHome(pObj->pRobotCtrl->m_nRobotNo);
}

int CAssemblyWeld::FuncBackHome(int nRobotNo)
{
	CUnit* pUnit = m_vpUnit[nRobotNo];
	CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
	bool bSuccess = m_vpUnit[nRobotNo]->RobotBackHome(pRobotDriver->m_tBackHomeSpeed);
	return bSuccess ? 0 : -1;
}

UINT CAssemblyWeld::ThreadGrooveTeachWeld(void *pParam)
{
	CAssemblyWeld* pObj = (CAssemblyWeld*)pParam;
	if (true == pObj->m_bGrooveTeachWeldRunning)
	{
		XiMessageBoxOk("坡口焊接线程运行中，禁止重复开启!");
		return -1;
	}
	pObj->m_bGrooveTeachWeldRunning = true;
	int nRst = pObj->FuncGrooceTeachWeld();
	pObj->m_bGrooveTeachWeldRunning = false;
	return nRst;
}

int CAssemblyWeld::FuncGrooceTeachWeld()
{
	int nRobotNo = 0;
	int dirRobot = -1;
	WriteLog("单击：坡口焊接");
	vector<T_WAVE_PARA> vtTWavePara;
	vector<T_INFOR_WAVE_RAND> vtGrooveRand;
	//获取起终点数据
	/*T_ROBOT_COORS tRobotStartCoord = { 0,0,0,180,45,0,0,0,0 };
	T_ROBOT_COORS tRobotEndCoord = { 100,-100,0,0,0,0,0,0,0 };*/
	T_ROBOT_COORS tRobotStartCoord = { 100,100,0,180,45,0,0,0,0 };
	T_ROBOT_COORS tRobotEndCoord = { 100,100,-100,0,0,0,0,0,0 };
	GetTeachPos(tRobotStartCoord, tRobotEndCoord, nRobotNo);
	if (0 != GetGroovePara(tRobotStartCoord, tRobotEndCoord, vtTWavePara))
	{
		XiMessageBox("获取摆弧参数失败");
	}
	// TODO: 在此添加控件通知处理程序代码
	// 
	T_GROOVE_INFOR tGrooveInfor;
	//视觉处理函数，最后参数还未整理，目前只可以用来测试视觉功能
	//if (MB_OK == XiMessageBox("是否进行扫描"))
	//{
	   // ScanGrooveInfo(tRobotStartCoord, tRobotEndCoord, nRobotNo, tGrooveInfor);
	//}


	//自动排道
	tGrooveInfor.weldAngle = tRobotStartCoord.dRZ;
	tGrooveInfor.dPlateThickness = 20.0;
	tGrooveInfor.dStartLowerFace = 5.0;
	tGrooveInfor.dStartUpperFace = 25.0;
	tGrooveInfor.dEndLowerFace = 12.0;
	tGrooveInfor.dEndUpperFace = 20.0;
	tGrooveInfor.tStartPoint = tRobotStartCoord;
	tGrooveInfor.tEndPoint = tRobotEndCoord;
	if (0 != GrooveAutoRandNew(tGrooveInfor, vtTWavePara, vtGrooveRand, dirRobot))
	{
		return -1;
	}
	vector<T_ROBOT_COORS> vtGrooveWavePath;
	vector<double> weldSpeedRate;
	vector<vector<T_ROBOT_COORS>> vvtGrooveWavePath;
	vector<vector<double>> vWeldSpeedRate;
	//计算摆弧轨迹
	for (size_t i = 0; i < vtGrooveRand.size(); i++)
	{
		vtGrooveWavePath.clear();
		weldSpeedRate.clear();
		CalWavePath(tGrooveInfor, vtGrooveRand[i], vtTWavePara[i], vtGrooveWavePath, weldSpeedRate);
		for (int nPtnNo = 0; nPtnNo < vtGrooveWavePath.size(); nPtnNo++)
		{
			vtGrooveWavePath[nPtnNo].dRX = tRobotStartCoord.dRX;
			vtGrooveWavePath[nPtnNo].dRY = tRobotStartCoord.dRY;
			vtGrooveWavePath[nPtnNo].dRZ = tRobotStartCoord.dRZ;
		}

		vvtGrooveWavePath.push_back(vtGrooveWavePath);
		vWeldSpeedRate.push_back(weldSpeedRate);

		CString sFileName;
		sFileName.Format("LocalFiles\\OutputFiles\\RobotA\\Recognition\\%d_%d_RealWeldCoord.txt", 0, i);
		FILE* pf = fopen(sFileName, "w");
		for (int nPtnIdx = 0; nPtnIdx < vtGrooveWavePath.size(); nPtnIdx++)
		{
			T_ROBOT_COORS& tCoord = vtGrooveWavePath[nPtnIdx];
			fprintf(pf, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%10d%11.3lf\n", nPtnIdx,
				tCoord.dX, tCoord.dY, tCoord.dZ, tCoord.dRX, tCoord.dRY, tCoord.dRZ, tCoord.dBX, tCoord.dBY, tCoord.dBZ,
				0.0, E_PLAT_GROOVE, E_WELD_TRACK, weldSpeedRate[nPtnIdx]);
		}
		fclose(pf);

	}
	//焊接
	if (1 == GrooveWeld(vvtGrooveWavePath, vtTWavePara, vWeldSpeedRate, nRobotNo))
	{
		MessageBox("焊接成功");
		return 0;
	}
	else
	{
		MessageBox("焊接失败");
		return -1;
	}
}

int CAssemblyWeld::GetGroovePara(const T_ROBOT_COORS& tRobotStartCoord, const T_ROBOT_COORS& tRobotEndCoord, vector<T_WAVE_PARA>& vtTWavePara)
{
	bool isVerticalWeld = false;//是否立焊
	double disEndToStart = sqrt(pow((tRobotEndCoord.dX - tRobotStartCoord.dX), 2) + pow((tRobotEndCoord.dY - tRobotStartCoord.dY), 2) + pow((tRobotEndCoord.dZ - tRobotStartCoord.dZ), 2));
	double vecX = (tRobotEndCoord.dX - tRobotStartCoord.dX) / disEndToStart;
	double vecY = (tRobotEndCoord.dY - tRobotStartCoord.dY) / disEndToStart;
	double vecZ = (tRobotEndCoord.dZ - tRobotStartCoord.dZ) / disEndToStart;
	if (abs(vecZ) > 0.5)
	{
		isVerticalWeld = true;
	}
	if (isVerticalWeld)
	{
		GetVerGroovePara(vtTWavePara);
	}
	else
	{
		GetFlatGroovePara(vtTWavePara);
	}
	return 0;
}

int CAssemblyWeld::GetFlatGroovePara(vector<T_WAVE_PARA>& vtTWavePara)
{
	vtTWavePara.clear();
	ChangeGroovePara  GroovePara;
	vtTWavePara = GroovePara.vtFlatWavePara;
	for (int i = 0; i < vtTWavePara.size(); i++)
	{
		vtTWavePara[i].bStandWeld = false;
	}
	return 0;
}

int CAssemblyWeld::GetVerGroovePara(vector<T_WAVE_PARA>& vtTWavePara)
{
	vtTWavePara.clear();
	ChangeGroovePara  GroovePara;
	vtTWavePara = GroovePara.vtVerWavePara;
	for (int i = 0; i < vtTWavePara.size(); i++)
	{
		vtTWavePara[i].bStandWeld = true;
	}
	return 0;
}


int CAssemblyWeld::GetTeachPos(T_ROBOT_COORS& tRobotStartCoord, T_ROBOT_COORS& tRobotEndCoord, int nRobotNo)
{
	//获取示教的点位
	double pTeach[2][6] = { 0 };
	UINT* pToolNo = new UINT;
	UINT* pUserNo = new UINT;
	UINT* pPosture = new UINT;
	m_vpRobotDriver[nRobotNo]->GetMultiPosVar(2, 111, pTeach, pToolNo, pUserNo, pPosture);

	//确定起点位置
	tRobotStartCoord.dX = pTeach[0][0];
	tRobotStartCoord.dY = pTeach[0][1];
	tRobotStartCoord.dZ = pTeach[0][2];
	tRobotStartCoord.dRX = pTeach[0][3];
	tRobotStartCoord.dRY = pTeach[0][4];
	tRobotStartCoord.dRZ = pTeach[0][5];

	//确定终点位置
	tRobotEndCoord.dX = pTeach[1][0];
	tRobotEndCoord.dY = pTeach[1][1];
	tRobotEndCoord.dZ = pTeach[1][2];
	tRobotEndCoord.dRX = pTeach[1][3];
	tRobotEndCoord.dRY = pTeach[1][4];
	tRobotEndCoord.dRZ = pTeach[1][5];
	return 0;
}

int CAssemblyWeld::CalWavePath(T_GROOVE_INFOR tGrooveInfor, T_INFOR_WAVE_RAND tGrooveRand, T_WAVE_PARA tWavePara, vector<T_ROBOT_COORS>& vtGrooveWavePath, vector<double>& weldSpeedRate)
{
	T_ROBOT_COORS tRobotStartCoord = tGrooveRand.tStartPoint;
	T_ROBOT_COORS tRobotEndCoord = tGrooveRand.tEndPoint;
	bool isVerticalWeld = false;//是否立焊
	vtGrooveWavePath.clear();
	double disEndToStart = sqrt(pow((tRobotEndCoord.dX - tRobotStartCoord.dX), 2) + pow((tRobotEndCoord.dY - tRobotStartCoord.dY), 2) + pow((tRobotEndCoord.dZ - tRobotStartCoord.dZ), 2));
	double vecX = (tRobotEndCoord.dX - tRobotStartCoord.dX) / disEndToStart;
	double vecY = (tRobotEndCoord.dY - tRobotStartCoord.dY) / disEndToStart;
	double vecZ = (tRobotEndCoord.dZ - tRobotStartCoord.dZ) / disEndToStart;
	if (abs(vecZ) > 0.5)
	{
		isVerticalWeld = true;
	}
	T_ROBOT_COORS tmpCoor = tRobotStartCoord;
	//计算起点至终点距离以及方向向量
	// 
	//确定姿态以及插入起点以及起点速率
	if (!isVerticalWeld)
	{
		tmpCoor.dRZ = atan2(vecY, vecX) * 180 / PI;
		/*tmpCoor.dRY = 0;*/
		tmpCoor.dRX = 0 - tGrooveRand.dAngleOffset;
		/*if (tGrooveRand.dAngleOffset > 0)
		{
			tmpCoor.dRX -= tGrooveRand.dAngleOffset;
		}
		else if (tGrooveRand.dAngleOffset < 0)
		{
			tmpCoor.dRX = -180 - tGrooveRand.dAngleOffset;
		}*/
	}
	else
	{
		tmpCoor.dRX = 0.0;
		tmpCoor.dRZ += tGrooveRand.dAngleOffset;
		if (tmpCoor.dRZ > 180)
		{
			tmpCoor.dRZ = tmpCoor.dRZ - 360;
		}
		if (tmpCoor.dRZ < -180)
		{
			tmpCoor.dRZ = tmpCoor.dRZ + 360;
		}
	}
	vtGrooveWavePath.push_back(tmpCoor);
	weldSpeedRate.push_back(1);

	double vecOnlyX;
	double vecOnlyY;
	if (isVerticalWeld)
	{
		//计算摆弧点偏移向量,焊枪角度为焊道方向
		double weldAngle = PI * tGrooveInfor.weldAngle / 180;
		vecOnlyX = cos(weldAngle);
		vecOnlyY = sin(weldAngle);
	}
	else
	{
		////计算摆弧偏移方向向量旋转前（不考虑Z方向）
		double disXY = sqrt(pow((tRobotEndCoord.dX - tRobotStartCoord.dX), 2) + pow((tRobotEndCoord.dY - tRobotStartCoord.dY), 2));
		vecOnlyX = (tRobotEndCoord.dX - tRobotStartCoord.dX) / disXY;
		vecOnlyY = (tRobotEndCoord.dY - tRobotStartCoord.dY) / disXY;
	}

	//计算摆弧偏移方向向量逆时针旋转
	double vecXAntiClockWise = vecOnlyX * cos(PI / 2) - vecOnlyY * sin(PI / 2);
	double vecYAntiClockWise = vecOnlyX * sin(PI / 2) - vecOnlyY * cos(PI / 2);
	//计算摆弧偏移方向向量顺时针旋转
	double vecXClockWise = vecOnlyX * cos(-PI / 2) - vecOnlyY * sin(-PI / 2);
	double vecYClockWise = vecOnlyX * sin(-PI / 2) - vecOnlyY * cos(-PI / 2);
	//
	double disWaveEndtoStart = tWavePara.dEndWave - tWavePara.dStartWave;//起点终点摆幅差
	/*double waveAngle = tWavePara.dWaveAngle * PI / 180;*/
	double disWave = tWavePara.dStartWave; //每段摆幅度
	/*double disWaveRunVec = disWave * cos(waveAngle) / sin(waveAngle);*/
	double sunDisRunVec = tWavePara.dWaveDistance;//记录沿焊缝方向距离
	int totalWave = disEndToStart / tWavePara.dWaveDistance;//初步计算摆幅总段数
	double surplusWave = disEndToStart - totalWave * tWavePara.dWaveDistance;//确定剩余摆弧余量
	//确定摆弧总段数
	if (surplusWave > (tWavePara.dWaveDistance / 2))
	{
		totalWave++;
	}
	if (!isVerticalWeld || (isVerticalWeld && 0 == tWavePara.waveType))
	{
		//确定摆幅固定增加变量
		double waveChangeValue = disWaveEndtoStart / totalWave;
		//插入摆弧的第一个摆幅点
		tmpCoor.dX = tRobotStartCoord.dX + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecX;
		tmpCoor.dY = tRobotStartCoord.dY + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecY;
		tmpCoor.dZ = tRobotStartCoord.dZ + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecZ;
		tmpCoor.dX = tmpCoor.dX + disWave * vecXAntiClockWise;
		tmpCoor.dY = tmpCoor.dY + disWave * vecYAntiClockWise;
		vtGrooveWavePath.push_back(tmpCoor);
		weldSpeedRate.push_back(1);

		for (int waveNum = 1; waveNum < totalWave; waveNum++)
		{
			sunDisRunVec += tWavePara.dWaveDistance;
			disWave += waveChangeValue;
			if (tWavePara.dEndWave == tWavePara.dStartWave)
			{
				weldSpeedRate.push_back(1.0);
			}
			else
			{
				weldSpeedRate.push_back(1 - (disWave - tWavePara.dStartWave) * (1 - tWavePara.dEndSpeedRate) / (tWavePara.dEndWave - tWavePara.dStartWave));
			}
			if (1 == waveNum % 2)
			{
				tmpCoor.dX = tRobotStartCoord.dX + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecX;
				tmpCoor.dY = tRobotStartCoord.dY + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecY;
				tmpCoor.dZ = tRobotStartCoord.dZ + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecZ;
				tmpCoor.dX = tmpCoor.dX + disWave * vecXClockWise;
				tmpCoor.dY = tmpCoor.dY + disWave * vecYClockWise;
				vtGrooveWavePath.push_back(tmpCoor);
			}
			else
			{
				tmpCoor.dX = tRobotStartCoord.dX + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecX;
				tmpCoor.dY = tRobotStartCoord.dY + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecY;
				tmpCoor.dZ = tRobotStartCoord.dZ + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecZ;
				tmpCoor.dX = tmpCoor.dX + disWave * vecXAntiClockWise;
				tmpCoor.dY = tmpCoor.dY + disWave * vecYAntiClockWise;
				vtGrooveWavePath.push_back(tmpCoor);
			}
		}
		tmpCoor.dX = tRobotEndCoord.dX;
		tmpCoor.dY = tRobotEndCoord.dY;
		tmpCoor.dZ = tRobotEndCoord.dZ;
		vtGrooveWavePath.push_back(tmpCoor);
		weldSpeedRate.push_back(tWavePara.dEndSpeedRate);
		return 0;
	}
	else if (1 == tWavePara.waveType)
	{
		//确定摆幅固定增加变量
		double waveChangeValue = disWaveEndtoStart / (totalWave - 1);
		for (int waveNum = 0; waveNum < totalWave; waveNum++)
		{
			if (0 == waveNum % 2)
			{
				tmpCoor.dX = tRobotStartCoord.dX + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecX;
				tmpCoor.dY = tRobotStartCoord.dY + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecY;
				tmpCoor.dZ = tRobotStartCoord.dZ + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecZ;
				tmpCoor.dX = tmpCoor.dX + vecOnlyX * tWavePara.dWaveHeight;
				tmpCoor.dY = tmpCoor.dY + vecOnlyY * tWavePara.dWaveHeight;
				tmpCoor.dX = tmpCoor.dX + disWave * vecXClockWise;
				tmpCoor.dY = tmpCoor.dY + disWave * vecYClockWise;
				vtGrooveWavePath.push_back(tmpCoor);
				weldSpeedRate.push_back(1 - (disWave - tWavePara.dStartWave) * (1 - tWavePara.dEndSpeedRate) / (tWavePara.dEndWave - tWavePara.dStartWave));
			}
			else
			{
				tmpCoor.dX = tRobotStartCoord.dX + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecX;
				tmpCoor.dY = tRobotStartCoord.dY + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecY;
				tmpCoor.dZ = tRobotStartCoord.dZ + (sunDisRunVec - tWavePara.dWaveDistance / 2) * vecZ;
				tmpCoor.dX = tmpCoor.dX + vecOnlyX * tWavePara.dWaveHeight;
				tmpCoor.dY = tmpCoor.dY + vecOnlyY * tWavePara.dWaveHeight;
				tmpCoor.dX = tmpCoor.dX + disWave * vecXAntiClockWise;
				tmpCoor.dY = tmpCoor.dY + disWave * vecYAntiClockWise;
				vtGrooveWavePath.push_back(tmpCoor);
				weldSpeedRate.push_back(1 - (disWave - tWavePara.dStartWave) * (1 - tWavePara.dEndSpeedRate) / (tWavePara.dEndWave - tWavePara.dStartWave));
				if (sunDisRunVec < disEndToStart)
				{
					tmpCoor.dX = tRobotStartCoord.dX + sunDisRunVec * vecX;
					tmpCoor.dY = tRobotStartCoord.dY + sunDisRunVec * vecY;
					tmpCoor.dZ = tRobotStartCoord.dZ + sunDisRunVec * vecZ;
					vtGrooveWavePath.push_back(tmpCoor);
					weldSpeedRate.push_back(1 - (disWave - tWavePara.dStartWave) * (1 - tWavePara.dEndSpeedRate) / (tWavePara.dEndWave - tWavePara.dStartWave));
				}
			}
			sunDisRunVec += tWavePara.dWaveDistance;
			disWave += waveChangeValue;
		}
		tmpCoor.dX = tRobotEndCoord.dX;
		tmpCoor.dY = tRobotEndCoord.dY;
		tmpCoor.dZ = tRobotEndCoord.dZ;
		vtGrooveWavePath.push_back(tmpCoor);
		weldSpeedRate.push_back(1 - (disWave - tWavePara.dStartWave) * (1 - tWavePara.dEndSpeedRate) / (tWavePara.dEndWave - tWavePara.dStartWave));
		return 0;
	}
	else
	{
		XiMessageBox("暂无此摆动");
		return -1;
	}
}

int CAssemblyWeld::GrooveWeld(vector<vector<T_ROBOT_COORS>> vvtGrooveWavePath, vector<T_WAVE_PARA> vtTWavePara, vector<vector<double>> vWeldSpeedRate, int nRobotNo)
{
	CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
	CUnit* pUnit = m_vpUnit[nRobotNo];
	// 创建并初始化加劲板焊接实例
	WAM::WeldAfterMeasure* pWeldAfterMeasure = NULL;// = m_pWeldAfterMeasure; // 为了不修以下变量名
	CHECK_BOOL_RETURN(CreateObject(SMALL_PIECE/*pRobotDriver->m_tChoseWorkPieceType*/, m_vpUnit[nRobotNo], &pWeldAfterMeasure));
	pWeldAfterMeasure->SetRecoParam();
	pWeldAfterMeasure->SetHardware(m_vpScanInit[nRobotNo]);
	pWeldAfterMeasure->SetWeldParam(&m_bStartArcOrNot, &m_bNaturalPop, &m_bTeachPop, &m_bNeedWrap);
	pWeldAfterMeasure->m_pColorImg = &m_vpShowLaserImgBuff[nRobotNo];
	pWeldAfterMeasure->m_bIsLocalDebug = GetLocalDebugMark() == TRUE;
	pWeldAfterMeasure->LoadPauseInfo();
	CHECK_BOOL_RETURN(pWeldAfterMeasure->FreeWaveGrooveWeld(vvtGrooveWavePath, vtTWavePara, vWeldSpeedRate));
	return 1;
}

int CAssemblyWeld::GrooveAutoRandNew(T_GROOVE_INFOR tGrooveInfor, vector<T_WAVE_PARA> vtTWavePara, vector<T_INFOR_WAVE_RAND>& vtGrooveRand, int nRobotDir)
{
	T_INFOR_WAVE_RAND tmpPointOffset;
	int noCount = 0;//按照道数进行偏移
	double dArcRandOffset = 20.0;//多层多道每次起收弧位置偏移
	double dArcThickness = 4.0;//每层焊接厚度
	double dUpperFace = tGrooveInfor.dStartUpperFace > tGrooveInfor.dEndUpperFace ? tGrooveInfor.dStartUpperFace : tGrooveInfor.dEndUpperFace;
	double dLowerFace = tGrooveInfor.dStartLowerFace > tGrooveInfor.dEndLowerFace ? tGrooveInfor.dStartLowerFace : tGrooveInfor.dEndLowerFace;
	double disFace = dUpperFace - dLowerFace;
	int numLayel = tGrooveInfor.dPlateThickness / dArcThickness;
	double dRemain = tGrooveInfor.dPlateThickness - numLayel * dArcThickness;
	if (dArcThickness / 2 < dRemain)
	{
		numLayel++;
	}
	double disRateFace = disFace / numLayel;
	bool isVerticalWeld = false;//是否立焊
	T_ROBOT_COORS tRobotStartCoord = tGrooveInfor.tStartPoint;
	T_ROBOT_COORS tRobotEndCoord = tGrooveInfor.tEndPoint;
	double disEndToStart = sqrt(pow((tRobotEndCoord.dX - tRobotStartCoord.dX), 2) + pow((tRobotEndCoord.dY - tRobotStartCoord.dY), 2) + pow((tRobotEndCoord.dZ - tRobotStartCoord.dZ), 2));
	if (120 > disEndToStart)
	{
		XiMessageBox("焊缝距离过短!");
		return -1;
	}
	double vecX = (tRobotEndCoord.dX - tRobotStartCoord.dX) / disEndToStart;
	double vecY = (tRobotEndCoord.dY - tRobotStartCoord.dY) / disEndToStart;
	double vecZ = (tRobotEndCoord.dZ - tRobotStartCoord.dZ) / disEndToStart;
	double dHorOffsetX;
	double dHorOffsetY;
	double vecOnlyX;
	double vecOnlyY;
	double vecOnlyZ;
	if (abs(vecZ) > 0.5)
	{
		isVerticalWeld = true;
		double weldAngle = PI * tRobotStartCoord.dRZ / 180;
		dHorOffsetX = sin(weldAngle) * nRobotDir;
		dHorOffsetY = -cos(weldAngle) * nRobotDir;
		vecOnlyX = cos(weldAngle) * vecZ * nRobotDir;
		vecOnlyY = sin(weldAngle) * vecZ * nRobotDir;
		vecOnlyZ = sqrt(pow(vecX, 2) + pow(vecY, 2)) * nRobotDir;
	}
	else
	{
		dHorOffsetX = -vecY * nRobotDir;
		dHorOffsetY = vecX * nRobotDir;
		vecOnlyZ = sqrt(pow(vecX, 2) + pow(vecY, 2)) * nRobotDir;
	}

	tmpPointOffset.tStartPoint = tRobotStartCoord;
	tmpPointOffset.tEndPoint = tRobotEndCoord;
	if (!isVerticalWeld)
	{
		for (size_t i = 0; i < numLayel; i++)
		{
			if (i < 2 || (dLowerFace < 16 && dLowerFace > 0))
			{
				if (0 == i)
				{
					/*tmpPointOffset.tStartPoint = tRobotStartCoord;*/
					tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX;
					tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY;
					tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ /*+ dArcThickness * i * vecOnlyZ*/;

					/*tmpPointOffset.tEndPoint = tRobotEndCoord;*/
					tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX;
					tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY;
					tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ /*+ dArcThickness * i * vecOnlyZ*/;

					tmpPointOffset.dAngleOffset = 0;
					vtGrooveRand.push_back(tmpPointOffset);
					noCount++;
				}
				else
				{
					/*tmpPointOffset.tStartPoint = tRobotStartCoord;*/
					tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + dArcRandOffset * vecX;
					tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + dArcRandOffset * vecY;
					tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ /*+ dArcThickness * i * vecOnlyZ*/ + dArcRandOffset * vecZ;

					/*tmpPointOffset.tEndPoint = tRobotEndCoord;*/
					tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX - dArcRandOffset * vecX;
					tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY - dArcRandOffset * vecY;
					tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ /*+ dArcThickness * i * vecOnlyZ*/ - dArcRandOffset * vecZ;

					tmpPointOffset.dAngleOffset = 0;
					vtGrooveRand.push_back(tmpPointOffset);
					noCount++;
				}
			}
			else if (i > 1 && (dLowerFace >= 16 && dLowerFace < 24))
			{
				/*offset = dLowerFace / 4;*/
				/*tmpPointOffset.tStartPoint = tRobotStartCoord;*/
				tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX /*+ offset * dHorOffsetX*/ + dArcRandOffset * vecX;
				tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY /*+ offset * dHorOffsetY*/ + dArcRandOffset * vecY;
				tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ /*+ dArcThickness * i * vecOnlyZ*/ + dArcRandOffset * vecZ;

				/*tmpPointOffset.tEndPoint = tRobotEndCoord;*/
				tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX /*+ offset * dHorOffsetX*/ - dArcRandOffset * vecX;
				tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY /*+ offset * dHorOffsetY*/ - dArcRandOffset * vecY;
				tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ /*+ dArcThickness * i * vecOnlyZ*/ - dArcRandOffset * vecZ;

				tmpPointOffset.dAngleOffset = 0.0;
				vtGrooveRand.push_back(tmpPointOffset);
				noCount++;

				/*tmpPointOffset.tStartPoint = tRobotStartCoord;*/
				tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX/* - offset * dHorOffsetX + 10 * i * vecX*/;
				tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY /*- offset * dHorOffsetY + 10 * i * vecY*/;
				tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ /*+ dArcThickness * i * vecOnlyZ + 10 * i * vecZ*/;

				/*tmpPointOffset.tEndPoint = tRobotEndCoord;*/
				tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX /*- offset * dHorOffsetX - 10 * i * vecX*/;
				tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY/* - offset * dHorOffsetY - 10 * i * vecY*/;
				tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ /*+ dArcThickness * i * vecOnlyZ - 10 * i * vecZ*/;

				tmpPointOffset.dAngleOffset = 0.0;
				vtGrooveRand.push_back(tmpPointOffset);
				noCount++;
			}
			else if (i > 1 && dLowerFace >= 24)
			{
				/*offset = dLowerFace * 2 / 3;*/
				/*tmpPointOffset.tStartPoint = tRobotStartCoord;*/
				tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX /*+ offset * dHorOffsetX*/ + dArcRandOffset * vecX;
				tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY /*+ offset * dHorOffsetY*/ + dArcRandOffset * vecY;
				tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ /*+ dArcThickness * i * vecOnlyZ*/ + dArcRandOffset * vecZ;

				/*tmpPointOffset.tEndPoint = tRobotEndCoord;*/
				tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX /*+ offset * dHorOffsetX*/ - dArcRandOffset * vecX;
				tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY /*+ offset * dHorOffsetY*/ - dArcRandOffset * vecY;
				tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ /*+ dArcThickness * i * vecOnlyZ*/ - dArcRandOffset * vecZ;

				tmpPointOffset.dAngleOffset = 0.0;
				vtGrooveRand.push_back(tmpPointOffset);
				noCount++;

				/*tmpPointOffset.tStartPoint = tRobotStartCoord;*/
				tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX /*+ 10 * i * vecX*/;
				tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY /*+ 10 * i * vecY*/;
				tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ /*+ dArcThickness * i * vecOnlyZ + 10 * i * vecZ*/;

				/*tmpPointOffset.tEndPoint = tRobotEndCoord;*/
				tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX /*- 10 * i * vecX*/;
				tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY /*- 10 * i * vecY*/;
				tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ /*+ dArcThickness * i * vecOnlyZ - 10 * i * vecZ*/;

				tmpPointOffset.dAngleOffset = 0;
				vtGrooveRand.push_back(tmpPointOffset);
				noCount++;

				/*tmpPointOffset.tStartPoint = tRobotStartCoord;*/
				tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX /*- offset * dHorOffsetX + 10 * i * vecX*/;
				tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY /*- offset * dHorOffsetY + 10 * i * vecY*/;
				tmpPointOffset.tStartPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ /*+ dArcThickness * i * vecOnlyZ + 10 * i * vecZ*/;

				/*tmpPointOffset.tEndPoint = tRobotEndCoord;*/
				tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX/* - offset * dHorOffsetX - 10 * i * vecX*/;
				tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY /*- offset * dHorOffsetY - 10 * i * vecY*/;
				tmpPointOffset.tEndPoint.dZ += vtTWavePara[noCount].dNormalOffset * vecOnlyZ /*+ dArcThickness * i * vecOnlyZ - 10 * i * vecZ*/;

				tmpPointOffset.dAngleOffset = 0.0;
				vtGrooveRand.push_back(tmpPointOffset);
				noCount++;
			}
			else
			{
				XiMessageBox("底面焊缝过宽！");
				return -1;
			}
			dLowerFace += disRateFace;
		}
	}
	else
	{
		for (size_t i = 0; i < numLayel; i++)
		{
			if (i < numLayel - 1)
			{
				if (0 == i)
				{
					/*tmpPointOffset.tStartPoint = tRobotStartCoord;*/
					tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX /*+ dArcThickness * i * vecOnlyX*/ + vtTWavePara[noCount].dNormalOffset * vecOnlyX;
					tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY /*+ dArcThickness * i * vecOnlyY*/ + vtTWavePara[noCount].dNormalOffset * vecOnlyY;
					tmpPointOffset.tStartPoint.dZ += /*dArcThickness * i * vecOnlyZ +*/ vtTWavePara[noCount].dNormalOffset * vecOnlyZ;

					/*tmpPointOffset.tEndPoint = tRobotEndCoord;*/
					tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX /*+ dArcThickness * i * vecOnlyX*/ + vtTWavePara[noCount].dNormalOffset * vecOnlyX;
					tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY /*+ dArcThickness * i * vecOnlyY*/ + vtTWavePara[noCount].dNormalOffset * vecOnlyY;
					tmpPointOffset.tEndPoint.dZ += /*dArcThickness * i * vecOnlyZ +*/ vtTWavePara[noCount].dNormalOffset * vecOnlyZ;

					tmpPointOffset.dAngleOffset = 0;
					vtGrooveRand.push_back(tmpPointOffset);
					noCount++;
				}
				else
				{
					/*tmpPointOffset.tStartPoint = tRobotStartCoord;*/
					tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX /*+ dArcThickness * i * vecOnlyX*/ + vtTWavePara[noCount].dNormalOffset * vecOnlyX + 0.0 * vecX;
					tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY /*+ dArcThickness * i * vecOnlyY*/ + vtTWavePara[noCount].dNormalOffset * vecOnlyY + 0.0 * vecY;
					tmpPointOffset.tStartPoint.dZ += /*dArcThickness * i * vecOnlyZ +*/ vtTWavePara[noCount].dNormalOffset * vecOnlyZ + 0.0 * vecZ;

					/*tmpPointOffset.tEndPoint = tRobotEndCoord;*/
					tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX /*+ dArcThickness * i * vecOnlyX*/ + vtTWavePara[noCount].dNormalOffset * vecOnlyX - dArcRandOffset * vecX;
					tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY /*+ dArcThickness * i * vecOnlyY*/ + vtTWavePara[noCount].dNormalOffset * vecOnlyY - dArcRandOffset * vecY;
					tmpPointOffset.tEndPoint.dZ += /*dArcThickness * i * vecOnlyZ +*/ vtTWavePara[noCount].dNormalOffset * vecOnlyZ - dArcRandOffset * vecZ;

					tmpPointOffset.dAngleOffset = 0;
					vtGrooveRand.push_back(tmpPointOffset);
					noCount++;
				}
			}
			else if (numLayel - 1 == i)
			{
				/*offset = dLowerFace / 4;*/
				/*tmpPointOffset.tStartPoint = tRobotStartCoord;*/
				tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX/* + dArcThickness * i * vecOnlyX*/ + vtTWavePara[noCount].dNormalOffset * vecOnlyX /*+ offset * dHorOffsetX*/ + 0.0 * vecX;
				tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY/* + dArcThickness * i * vecOnlyY*/ + vtTWavePara[noCount].dNormalOffset * vecOnlyY /*+ offset * dHorOffsetY*/ + 0.0 * vecY;
				tmpPointOffset.tStartPoint.dZ +=/* dArcThickness * i * vecOnlyZ +*/ vtTWavePara[noCount].dNormalOffset * vecOnlyZ + 0.0 * vecZ;

				/*tmpPointOffset.tEndPoint = tRobotEndCoord;*/
				tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX/* + dArcThickness * i * vecOnlyX*/ + vtTWavePara[noCount].dNormalOffset * vecOnlyX /*+ offset * dHorOffsetX*/ - dArcRandOffset * vecX;
				tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY /*+ dArcThickness * i * vecOnlyY*/ + vtTWavePara[noCount].dNormalOffset * vecOnlyY /*+ offset * dHorOffsetY*/ - dArcRandOffset * vecY;
				tmpPointOffset.tEndPoint.dZ +=/* dArcThickness * i * vecOnlyZ +*/ vtTWavePara[noCount].dNormalOffset * vecOnlyZ - dArcRandOffset * vecZ;

				tmpPointOffset.dAngleOffset = 0.0;
				vtGrooveRand.push_back(tmpPointOffset);
				noCount++;

				/*tmpPointOffset.tStartPoint = tRobotStartCoord;*/
				tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX/* + dArcThickness * i * vecOnlyX*/ + vtTWavePara[noCount].dNormalOffset * vecOnlyX /*- offset * dHorOffsetX + 10 * i * vecX*/;
				tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY /*+ dArcThickness * i * vecOnlyY*/ + vtTWavePara[noCount].dNormalOffset * vecOnlyY /*- offset * dHorOffsetY + 10 * i * vecY*/;
				tmpPointOffset.tStartPoint.dZ += /*dArcThickness * i * vecOnlyZ +*/ vtTWavePara[noCount].dNormalOffset * vecOnlyZ/* + 10 * i * vecZ*/;
				/*tmpPointOffset.tEndPoint = tRobotEndCoord;*/
				tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX /*+ dArcThickness * i * vecOnlyX */ + vtTWavePara[noCount].dNormalOffset * vecOnlyX /*- offset * dHorOffsetX - 10 * i * vecX*/;
				tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY /*+ dArcThickness * i * vecOnlyY*/ + vtTWavePara[noCount].dNormalOffset * vecOnlyY /*- offset * dHorOffsetY - 10 * i * vecY*/;
				tmpPointOffset.tEndPoint.dZ += /*dArcThickness * i * vecOnlyZ +*/ vtTWavePara[noCount].dNormalOffset * vecOnlyZ/* - 10 * i * vecZ*/;

				tmpPointOffset.dAngleOffset = 0.0;
				vtGrooveRand.push_back(tmpPointOffset);
				noCount++;
			}
			/*else if (dLowerFace > 24)
			{
				offset = dLowerFace * 2 / 3;
				tmpPointOffset.tStartPoint = tRobotStartCoord;
				tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + dArcThickness * i * vecOnlyX + vtTWavePara[noCount].dNormalOffset * vecOnlyX + offset * dHorOffsetX + 10 * i * vecX;
				tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + dArcThickness * i * vecOnlyY + vtTWavePara[noCount].dNormalOffset * vecOnlyY + offset * dHorOffsetY + 10 * i * vecY;
				tmpPointOffset.tStartPoint.dZ += dArcThickness * i * vecOnlyZ + vtTWavePara[noCount].dNormalOffset * vecOnlyZ + 10 * i * vecZ;

				tmpPointOffset.tEndPoint = tRobotEndCoord;
				tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + dArcThickness * i * vecOnlyX + vtTWavePara[noCount].dNormalOffset * vecOnlyX - offset * dHorOffsetX - 10 * i * vecX;
				tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + dArcThickness * i * vecOnlyY + vtTWavePara[noCount].dNormalOffset * vecOnlyY - offset * dHorOffsetY - 10 * i * vecY;
				tmpPointOffset.tEndPoint.dZ += dArcThickness * i * vecOnlyZ + vtTWavePara[noCount].dNormalOffset * vecOnlyZ - 10 * i * vecZ;

				tmpPointOffset.dAngleOffset = 5.0;
				vtGrooveRand.push_back(tmpPointOffset);
				noCount++;

				tmpPointOffset.tStartPoint = tRobotStartCoord;
				tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + dArcThickness * i * vecOnlyX + vtTWavePara[noCount].dNormalOffset * vecOnlyX + 10 * i * vecX;
				tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + dArcThickness * i * vecOnlyY + vtTWavePara[noCount].dNormalOffset * vecOnlyY + 10 * i * vecY;
				tmpPointOffset.tStartPoint.dZ += dArcThickness * i * vecOnlyZ + vtTWavePara[noCount].dNormalOffset * vecOnlyZ + 10 * i * vecZ;

				tmpPointOffset.tEndPoint = tRobotEndCoord;
				tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + dArcThickness * i * vecOnlyX + vtTWavePara[noCount].dNormalOffset * vecOnlyX - 10 * i * vecX;
				tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + dArcThickness * i * vecOnlyY + vtTWavePara[noCount].dNormalOffset * vecOnlyY - 10 * i * vecY;
				tmpPointOffset.tEndPoint.dZ += dArcThickness * i * vecOnlyZ + vtTWavePara[noCount].dNormalOffset * vecOnlyZ - 10 * i * vecZ;
				tmpPointOffset.dAngleOffset = 0;
				vtGrooveRand.push_back(tmpPointOffset);
				noCount++;

				tmpPointOffset.tStartPoint = tRobotStartCoord;
				tmpPointOffset.tStartPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + dArcThickness * i * vecOnlyX + vtTWavePara[noCount].dNormalOffset * vecOnlyX - offset * dHorOffsetX + 10 * i * vecX;
				tmpPointOffset.tStartPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + dArcThickness * i * vecOnlyY + vtTWavePara[noCount].dNormalOffset * vecOnlyY - offset * dHorOffsetY + 10 * i * vecY;
				tmpPointOffset.tStartPoint.dZ += dArcThickness * i * vecOnlyZ + vtTWavePara[noCount].dNormalOffset * vecOnlyZ + 10 * i * vecZ;

				tmpPointOffset.tEndPoint = tRobotEndCoord;
				tmpPointOffset.tEndPoint.dX += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetX + dArcThickness * i * vecOnlyX + vtTWavePara[noCount].dNormalOffset * vecOnlyX - offset * dHorOffsetX - 10 * i * vecX;
				tmpPointOffset.tEndPoint.dY += vtTWavePara[noCount].dHorizontalOffset * dHorOffsetY + dArcThickness * i * vecOnlyY + vtTWavePara[noCount].dNormalOffset * vecOnlyY - offset * dHorOffsetY - 10 * i * vecY;
				tmpPointOffset.tEndPoint.dZ += dArcThickness * i * vecOnlyZ + vtTWavePara[noCount].dNormalOffset * vecOnlyZ - 10 * i * vecZ;
				tmpPointOffset.dAngleOffset = -5.0;
				vtGrooveRand.push_back(tmpPointOffset);
				noCount++;
			}*/
			else
			{
				XiMessageBox("底面焊缝过宽！");
				return -1;
			}
			dLowerFace += disRateFace;
		}
	}
	return 0;
}

bool CAssemblyWeld::JudgeGrooveStandWeld(WeldLineInfo tWeldLineInfo)
{
	bool bIsGrooveStandWeld = false;//是否立焊
	CvPoint3D64f tStartPtn = tWeldLineInfo.tWeldLine.StartPoint;
	CvPoint3D64f tEndPtn = tWeldLineInfo.tWeldLine.EndPoint;
	double disEndToStart = sqrt(pow((tEndPtn.x - tStartPtn.x), 2) + pow((tEndPtn.y - tStartPtn.y), 2) + pow((tEndPtn.z - tStartPtn.z), 2));
	double vecX = (tEndPtn.x - tStartPtn.x) / disEndToStart;
	double vecY = (tEndPtn.y - tStartPtn.y) / disEndToStart;
	double vecZ = (tEndPtn.z - tStartPtn.z) / disEndToStart;
	if (abs(vecZ) > 0.5) bIsGrooveStandWeld = true;
	return bIsGrooveStandWeld;
}

 UINT CAssemblyWeld::ThreadShowMoveState( void *pParam )
 {
	 CAssemblyWeld *pcMyObj=(CAssemblyWeld *)pParam;
	 pcMyObj->m_bThreadShowMoveStateEnd = true;
	 while(!(pcMyObj->m_bQuit))
	 {    
		 if (pcMyObj->m_bThreadShowMoveStateEnd == false)
		 {
			 break;
		 }
         pcMyObj->ShowMoveState(pcMyObj->m_vpUnit[0]);
		 DoEvent();
		 Sleep(50);
	 }
	 pcMyObj->m_bThreadShowMoveStateEnd = false;
	 return 0;
 }

 void CAssemblyWeld::OnBnClickedButtonAdjustRecog()
 {
	 WriteLog("单击：手动清枪");
	 if (true == m_bAutoWeldWorking)
	 {
		 XiMessageBoxOk("正在工作中，不能执行清枪!");
		 return;
	 }
	 int nRobotNo = 0;
	 CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];

	 if ((TRUE == m_bNaturalPop) && (1 == XiMessageBox("第一台是否清枪")))
	 {
		 CleanGun(m_vpRobotDriver[0]);
	 }
	 
	 //CleanGunH(m_vpRobotDriver[1]);
	
	 return;
 }

 void CAssemblyWeld::BackHome()
 {
	 if (54 == m_vpRobotDriver[0]->m_nExternalAxleType) // 国焊临时
	 {
		 BackHome_G();
		 return;
	 }

	 //int nRobotNo = 1 == XiMessageBox("确定机器人1,取消机器人2") ? 0 : 1;
	 int nRobotNo = 0;
	 CUnit* pUnit = m_vpUnit[nRobotNo];
	 CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
	 CHECK_BOOL(pUnit->CheckIsReadyRun());

	 // 正座机器人 or 倒挂机器人 ？
	 double dRobotInstallMode = pRobotDriver->m_nRobotInstallDir; // 正装：1.0   倒装：-1.0
	 double dMinUpMoveDis = 100.0;
	 double dBackHeightThreshold = 400.0;
	 T_ANGLE_PULSE tCurPulse;
	 T_ROBOT_COORS tCurCoord;
	 T_ROBOT_COORS tTransCoord;
	 T_ROBOT_COORS tTempCoord;
	 T_ROBOT_COORS tHomeCoord;
	 T_ROBOT_MOVE_INFO tRobotMoveInfo;
	 vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo;
	 vtRobotMoveInfo.clear();

	 tCurCoord = pRobotDriver->GetCurrentPos();
	 tCurPulse = pRobotDriver->GetCurrentPulse();
	 pRobotDriver->RobotKinematics(pRobotDriver->m_tHomePulse, pRobotDriver->m_tTools.tGunTool, tHomeCoord);
	 pRobotDriver->RobotKinematics(tCurPulse, pRobotDriver->m_tTools.tGunTool, tTransCoord);

	 //// 读取直角坐标和读取关节坐标转换出来结果不同时，不能运动
	 //if (false == pRobotDriver->CompareCoords(tTransCoord, tCurCoord))
	 //{
		// WriteLog("MoveToSafeHeight:读取坐标和计算坐标误差过大");
		// XiMessageBoxOk("读取坐标和计算坐标不同，无法自动回安全位置！");
		// return;
	 //}

	 double dMinBackGunHeight = tHomeCoord.dZ - (dBackHeightThreshold * dRobotInstallMode);//最小，最大高度安全位置加上阈值
	 // 高度小于dMinBackGunHeight时 执行操作：1、RY恢复标准45  2、最少抬高100mm（MOVL距离太小速度非常快）
	 tTempCoord = tCurCoord;
	 tTempCoord.dRX = tHomeCoord.dRX;
	 tTempCoord.dRY = tHomeCoord.dRY;
	 if ((dRobotInstallMode > 0.0 && tCurCoord.dZ < dMinBackGunHeight) ||
		 (dRobotInstallMode < 0.0 && tCurCoord.dZ > dMinBackGunHeight))//判断，当前位置与最大最小高度比较
	 {
		 if (fabs(tCurCoord.dZ - dMinBackGunHeight) < dMinUpMoveDis)
		 {
			 tTempCoord.dZ += (dMinUpMoveDis * dRobotInstallMode);
		 }
		 else
		 {
			 tTempCoord.dZ = dMinBackGunHeight;
		 }
		 tRobotMoveInfo = pRobotDriver->PVarToRobotMoveInfo(0, tTempCoord, pRobotDriver->m_tCoordHighSpeed, MOVL);
		 vtRobotMoveInfo.push_back(tRobotMoveInfo);
	 }
	 T_ANGLE_PULSE tPulse = pRobotDriver->m_tHomePulse;
	 tPulse.nSPulse = pRobotDriver->GetCurrentPulse(SAxis);
	 tRobotMoveInfo = pRobotDriver->PVarToRobotMoveInfo(1, tPulse, pRobotDriver->m_tBackHomeSpeed, MOVJ);
	 tRobotMoveInfo = pRobotDriver->PVarToRobotMoveInfo(2, pRobotDriver->m_tHomePulse, pRobotDriver->m_tBackHomeSpeed, MOVJ);
	 vtRobotMoveInfo.push_back(tRobotMoveInfo);
	 pRobotDriver->SetMoveValue(vtRobotMoveInfo);
	 pRobotDriver->CallJob("CONTIMOVANY");
	 pUnit->RobotCheckDone();

	 if (false == pRobotDriver->ComparePulse(pRobotDriver->m_tHomePulse))
	 {
		 //已修改
		 //SetHintInfo("回安全位置失败！");
		 SetHintInfo(XUI::Languge::GetInstance().translate("回安全位置失败!"));//"回安全位置失败！"
		 return;
	 }
	 //已修改
	 //SetHintInfo("回安全位置完成！");
	 SetHintInfo(XUI::Languge::GetInstance().translate("回安全位置完成!"));//"回安全位置失败！"

	 //先移动悬臂轴-y方向
	 if (-1 != m_vpUnit[nRobotNo]->m_nMeasureAxisNo_up)
	 {
		 if (0 != m_vpUnit[nRobotNo]->MoveExAxisFun(0, 9000, 2))return;
		 m_vpUnit[nRobotNo]->WorldCheckRobotDone();
		 //需要更改获取轴数据，临时更改
		 double dCurExPos = m_vpUnit[nRobotNo]->GetExPositionDis(2);
		 if (fabs(0 - dCurExPos) > 5.0)
		 {
			 XiMessageBox("自动示教：外部轴未运动到指定位置");
			 return;
		 }
	 }
	 //if ((1 == pRobotDriver->m_nRobotInstallDir) && (IDOK == XiMessageBox("是否回下料安全位置？")))
	 //{
		// T_ANGLE_PULSE pSafePoint;
		// double dAxisUnitT = pRobotDriver->m_tAxisUnit.dSPulse; // T轴脉冲当量
		// pSafePoint.nSPulse = pRobotDriver->m_tHomePulse.nSPulse + (90.0 / dAxisUnitT);
		// pSafePoint.nLPulse = pRobotDriver->m_tHomePulse.nLPulse;
		// pSafePoint.nUPulse = pRobotDriver->m_tHomePulse.nUPulse;
		// pSafePoint.nRPulse = pRobotDriver->m_tHomePulse.nRPulse;
		// pSafePoint.nBPulse = pRobotDriver->m_tHomePulse.nBPulse;
		// pSafePoint.nTPulse = pRobotDriver->m_tHomePulse.nTPulse;
		// pRobotDriver->MoveByJob(pSafePoint, pRobotDriver->m_tBackHomeSpeed, pRobotDriver->m_nExternalAxleType, "MOVJ");
		// pRobotDriver->CheckRobotDone();
		// SetHintInfo("回下料安全位置完成！");
	 //}

	 return;
 }
  
 void CAssemblyWeld::OnBnClickedButtonBackHome()
 {
	 WriteLog("单击：回安全位置");
	 if (true == m_bAutoWeldWorking)
	 {
		 XiMessageBoxOk("正在工作中，不能回安全位置!");
		 return;
	 }
	 BackHome();
	 return;
 }
 
 void CAssemblyWeld::LoadOptionalFunctionPara()
 {
	 bool bPause = false;
	 bool bFlashback = false;

	 COPini opini;
	 opini.SetFileName(OPTIONAL_FUNCTION);

	 opini.SetSectionName("START_ARC_OR_NOT");
	 opini.ReadString("StartArc", &m_bStartArcOrNot);
	 ((CButton*)GetDlgItem(IDC_RADIO_ARC))->SetCheck((int)m_bStartArcOrNot);
	 ((CButton*)GetDlgItem(IDC_RADIO_NOARC))->SetCheck((int)(!m_bStartArcOrNot));

	 //opini.SetSectionName("PAUSE_OR_CONTINUE");
	 //opini.ReadString("Pause ", &bPause);
	 //CString sBtnText = bPause ? "暂停" : "继续";
	 //GetDlgItem(IDC_BUTTON_PAUSE_CONTINUE)->SetWindowText(sBtnText);
	 //GetDlgItem(IDC_BUTTON_PAUSE_CONTINUE)->EnableWindow(!bPause);
	 //GetDlgItem(IDC_BUTTON_ADVANCED_CONTINUE)->EnableWindow(!bPause);

	 opini.SetFileName(OPTIONAL_FUNCTION);
	 opini.SetSectionName("Popup");
	 opini.ReadString("TeachPopup", &m_bTeachPop);
	 ((CButton*)GetDlgItem(IDC_CHECK_TEACH_POP))->SetCheck((int)m_bTeachPop);

	 opini.ReadString("NaturalPopup", &m_bNaturalPop);
	 ((CButton*)GetDlgItem(IDC_CHECK_NATURAL_POP))->SetCheck((int)m_bNaturalPop);

	 opini.ReadString("ProcessPopup", &m_bProcessPop);
	 ((CButton*)GetDlgItem(IDC_CHECK_PROCESS_POP))->SetCheck((int)m_bProcessPop);

	 opini.ReadString("CleanGunPopup", &m_bCleanGunEnable);
	 ((CButton*)GetDlgItem(IDC_CHECK3))->SetCheck((int)m_bCleanGunEnable);

	 opini.ReadString("EnableMeasureThick", &m_bMeausreThickEnable);
	 ((CButton*)GetDlgItem(IDC_CHECK_GRAY))->SetCheck((int)m_bMeausreThickEnable);

	 opini.ReadString("NeedWrap", &m_bNeedWrap);
	 ((CButton*)GetDlgItem(IDC_CHECK_GRAY2))->SetCheck((int)m_bNeedWrap);

	 //opini.SetSectionName("Flashback");
	 //opini.ReadString("Flashback", &bFlashback);
	 //if (bFlashback && !g_bLocalDebugMark)
	 //{
		// SaveErrorData("闪退");
	 //}
	 //opini.WriteString("Flashback", true);
 }

 void CAssemblyWeld::OnBnClickedRadioArc()
 {
	 // TODO: 在此添加控件通知处理程序代码
	 WriteLog("单击：起弧");
	 m_bStartArcOrNot = TRUE;
	 COPini cIni;
	 cIni.SetFileName(OPTIONAL_FUNCTION);
	 cIni.SetSectionName("START_ARC_OR_NOT");
	 cIni.WriteString("StartArc", m_bStartArcOrNot);
 }

 void CAssemblyWeld::OnBnClickedRadioNoarc()
 {
	 // TODO: 在此添加控件通知处理程序代码
	 WriteLog("单击：空走");
	 m_bStartArcOrNot = FALSE;
	 COPini cIni;
	 cIni.SetFileName(OPTIONAL_FUNCTION);
	 cIni.SetSectionName("START_ARC_OR_NOT");
	 cIni.WriteString("StartArc", m_bStartArcOrNot);
 }

 void CAssemblyWeld::SaveErrorData(CString strName)
 {
	 //获取限制日期
	 long lMaxDate = 100;
	 COPini opini;
	 opini.SetFileName(DEBUG_INI);
	 opini.SetSectionName("Debug");
	 opini.ReadString("MaxDateOfSaveErrorData", &lMaxDate);

	 //获取时间
	 SYSTEMTIME tTime;
	 GetLocalTime(&tTime);
	 CString strDate;
	 strDate.Format("\\%.2d-%.2d", tTime.wMonth, tTime.wDay);
	 CString strTime;
	 strTime.Format("-%.2d-%.2d-%.2d", tTime.wHour, tTime.wMinute, tTime.wSecond);

	 //获取路径
	 CheckFolder(ERROR_DATA_PATH);
	 CString strRootDir = ERROR_DATA_PATH;
	 CString strDir;
	 strDir = strRootDir + strDate;
	 strDir += strTime;

	 //计算截止日期
	 SYSTEMTIME tDeadline;
	 CalDeadLine(tDeadline, tTime, -lMaxDate);

	 //清除截止日期前的数据
	 CFileFind finder;
	 BOOL bWorking = finder.FindFile(strRootDir + "\\*.*");
	 while (bWorking)
	 {
		 bWorking = finder.FindNextFile();
		 if (finder.IsDirectory() && !finder.IsDots())
		 {
			 //获取文件上次修改日期
			 ULARGE_INTEGER fTime1;
			 finder.GetCreationTime((FILETIME*)&fTime1);

			 //转化日期
			 ULARGE_INTEGER fDeadline;
			 SystemTimeToFileTime(&tDeadline, (FILETIME*)&fDeadline);

			 //清理旧日期文件
			 if (fTime1.QuadPart < fDeadline.QuadPart)
			 {
				 DelFiles(finder.GetFilePath());
				 RemoveDirectory(finder.GetFilePath());
			 }
		 }
	 }

	 //复制数据
	 strDir += strName;
	 CopyFolder(".\\ConfigFiles", strDir);
	 CopyFolder(".\\LocalFiles\\OutputFiles", strDir);
	 CopyFolder(".\\Monitor", strDir);
	 //CString strLogName = GetLogName();
	 //CopyFile((LPCTSTR)"./WidgetWeldExtraction.ini", (LPCTSTR)(strDir + "\\WidgetWeldExtraction.ini"), FALSE);
	 //CopyFile((LPCTSTR)strLogName, (LPCTSTR)(strDir + "\\Log.txt"), FALSE);
 }

 BOOL CAssemblyWeld::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
 {
	 CRect tRect;
	 GetDlgItem(IDC_STATIC_DRAW_PART_GLOBAL)->GetClientRect(&tRect);
	 GetDlgItem(IDC_STATIC_DRAW_PART_GLOBAL)->ClientToScreen(&tRect);
	 if (pt.x >= tRect.left && 
		 pt.x <= tRect.right && 
		 pt.y >= tRect.top && 
		 pt.y <= tRect.bottom)
	 {
		 //XiMessageBox("控件内滚轮转动");
	 }
	 return CDialog::OnMouseWheel(nFlags, zDelta, pt);
 }

 void CAssemblyWeld::OnRButtonDown(UINT nFlags, CPoint point)
 {
	 CRect tRect;
	 GetDlgItem(IDC_STATIC_DRAW_PART_GLOBAL)->GetClientRect(&tRect);
	 GetDlgItem(IDC_STATIC_DRAW_PART_GLOBAL)->ClientToScreen(&tRect);
	 if (point.x >= tRect.left &&
		 point.x <= tRect.right &&
		 point.y >= tRect.top &&
		 point.y <= tRect.bottom)
	 {
		 //XiMessageBox("控件内按下鼠标右键");
	 }
	 CDialog::OnRButtonDown(nFlags, point);
 }

 void CAssemblyWeld::OnRButtonUp(UINT nFlags, CPoint point)
 {
	 CRect tRect;
	 GetDlgItem(IDC_STATIC_DRAW_PART_GLOBAL)->GetClientRect(&tRect);
	 GetDlgItem(IDC_STATIC_DRAW_PART_GLOBAL)->ClientToScreen(&tRect);
	 if (point.x >= tRect.left &&
		 point.x <= tRect.right &&
		 point.y >= tRect.top &&
		 point.y <= tRect.bottom)
	 {
		 //XiMessageBox("控件内释放鼠标右键");
	 }
	 CDialog::OnRButtonUp(nFlags, point);
 }
 
 void CAssemblyWeld::OnBnClickedButtonCommonlyUsedIO()
 {
	 WriteLog("单击：常用IO");
	 CIOCtrlDlg cIOCtrlDlg(&m_vpUnit);
	 cIOCtrlDlg.DoModal();
 }

 void CAssemblyWeld::OnBnClickedButtonSpare3()
 {
	 WriteLog("单击：备用按钮");
//	 E_CAM_ID eCamId;
//	 CRobotDriverAdaptor *pRobotCtrl;
//	 if (IDOK == XiMessageBoxGroup(1, "左机器人运行？"))
//	 {
//		 pRobotCtrl = m_vtRobotThread[0]->pRobotCtrl;
//		 if (IDOK == XiMessageBox("左左相机测量"))
//		 {
//			 eCamId = E_LEFT_ROBOT_LEFT_CAM_H;
//
//		 }
//		 else if (IDOK == XiMessageBox("左右相机测量"))
//		 {
//			 eCamId = E_LEFT_ROBOT_RIGHT_CAM_H;
//		 }
//		 else
//		 {
//			 XiMessageBox("请选择相机");
//			 return;
//		 }
//	 }
//	 else if (IDOK == XiMessageBoxGroup(1, "右机器人运行？"))
//	 {
//		 pRobotCtrl = m_vtRobotThread[1]->pRobotCtrl;
//		 if (IDOK == XiMessageBox("右左相机测量"))
//		 {
//			 eCamId = E_RIGHT_ROBOT_LEFT_CAM_H;
//
//		 }
//		 else if (IDOK == XiMessageBox("右右相机测量"))
//		 {
//			 eCamId = E_RIGHT_ROBOT_RIGHT_CAM_H;
//		 }
//		 else
//		 {
//			 XiMessageBox("请选择相机");
//			 return;
//		 }
//	 }
//	 else
//	 {
//		 XiMessageBoxGroup(1, "未选择机器人");
//		 return;
//	 }
//	 int nRobotNo = pRobotCtrl->m_nRobotNo;
//	 pRobotCtrl->SwitchIO("TrackLaser", true);
//     //m_cIOControl->OpenLeftRedALaser(); Sleep(50);
//	 FILE *CoutMeasPoint;
//	 FILE *CoutTeachPoint;
//	 FILE *CoutTeachPointRobot;
//	 CString strPath ;
//	 strPath.Format(".\\MeasureData\\%s\\%d", pRobotCtrl->m_strRobotName, eCamId);
//	 CString strImagePoint = "_ImagePoint.txt";
//	 CString strMeasurePluase = "_MeasurePluase.txt";
//	 CString strMeasurePos = "_MeasurePos.txt";
//	 CString strSaveName1 = strPath+ strImagePoint;
//	 CString strSaveName2 = strPath + strMeasurePluase;
//	 CString strSaveName3 = strPath + strMeasurePos;
//
//	 CoutMeasPoint = fopen(strSaveName1.GetBuffer(0), "a+");
//	 CoutTeachPoint = fopen(strSaveName2.GetBuffer(0), "a+");
//	 CoutTeachPointRobot = fopen(strSaveName3.GetBuffer(0), "a+");
//
//	 static int i = 0;
//	 CString str;
//	 CvPoint cpMidKeyPoint = {0};
//	 CvPoint cpBesideKeyPoint = {0};
//	 std::vector<CvPoint> vcpLeftOutPoints;
//	 std::vector<CvPoint> vcpRightOutPoints;
//	 cvCopyImage(m_vpDHCameraVision[nRobotNo]->CamId2CamDrv(eCamId)->LaserImageCapture(false), m_vpDHCameraVision[nRobotNo]->CamId2ImageBuff(eCamId));
//	 Sleep(10);
//	 m_vpDHCameraVision[nRobotNo]->CamId2ImageProcess(eCamId)->GetBesideKeyPoints(m_vpDHCameraVision[nRobotNo]->CamId2ImageBuff(eCamId), cpMidKeyPoint, cpBesideKeyPoint,
//		 vcpLeftOutPoints, vcpRightOutPoints);
//	 WriteLog("二维点：X；%d Y:%d", cpMidKeyPoint.x, cpMidKeyPoint.y);
//	 cvCircle(m_vpDHCameraVision[nRobotNo]->CamId2ImageBuff(eCamId), cpMidKeyPoint, 3, CV_RGB(125, 0, 0), 3);
//	 m_vpDHCameraVision[nRobotNo]->CamId2ImageProcess(eCamId)->m_pXiCvObj->xiSaveImage(m_vpDHCameraVision[nRobotNo]->CamId2ImageBuff(eCamId), ".\\LeftGun_LeftCam\\", NULL, i, 0);
//
//	 if (IDOK == XiMessageBox("确认图像找点正确"))
//	 {
//		 fprintf(CoutMeasPoint, "%d	%d	%d\n", i, cpMidKeyPoint.x, cpMidKeyPoint.y);
//		 T_ROBOT_COORS tRobotCurCoord = pRobotCtrl->GetCurrentPos();
//		 T_ANGLE_PULSE tRobotCurPulses = pRobotCtrl->GetCurrentPulse();
//
//		 fprintf(CoutTeachPointRobot, "%d %lf %lf %lf %lf %lf %lf\n", i, tRobotCurCoord.dX, tRobotCurCoord.dY, tRobotCurCoord.dZ, tRobotCurCoord.dRX, tRobotCurCoord.dRY, tRobotCurCoord.dRZ);
//		 fprintf(CoutTeachPoint, "%d %ld %ld %ld %ld %ld %ld\n", i, tRobotCurPulses.nSPulse, tRobotCurPulses.nLPulse, tRobotCurPulses.nUPulse, tRobotCurPulses.nRPulse, tRobotCurPulses.nBPulse, tRobotCurPulses.nTPulse);
//		 i++;
//	 }
//	 fclose(CoutMeasPoint);
//	 fclose(CoutTeachPoint);
//	 fclose(CoutTeachPointRobot);
	 return;
 }

 void CAssemblyWeld::OnBnClickedButtonSpare1()
 {
	 WriteLog("单击：包角参数");
	 int nRobotNo = 0;
	 CUnit* pUnit = m_vpUnit[nRobotNo];
	 CWrapAngleParam cWrapAngleParam(pUnit->m_tContralUnit.strUnitName);
	 cWrapAngleParam.DoModal();
	 return;
 }

 void CAssemblyWeld::OnBnClickedCheckNaturalPop()
 {
	 UpdateData(TRUE);
	 if (m_bNaturalPop)
	 {
		 WriteLog("单击：打开提示性弹窗");
	 } 
	 else
	 {
		 WriteLog("单击：关闭提示性弹窗");
	 }
	 COPini opini;
	 opini.SetFileName(OPTIONAL_FUNCTION);
	 opini.SetSectionName("Popup");
	 opini.WriteString("NaturalPopup", m_bNaturalPop);
	 GetNaturalPop().setPopup(TRUE == m_bNaturalPop);
 }

 void CAssemblyWeld::OnBnClickedCheckProcessPop()
 {
	 UpdateData(TRUE);
	 if (m_bProcessPop)
	 {
		 WriteLog("单击：打开工艺弹窗");
	 }
	 else
	 {
		 WriteLog("单击：关闭工艺弹窗");
	 }
	 COPini opini;
	 opini.SetFileName(OPTIONAL_FUNCTION);
	 opini.SetSectionName("Popup");
	 opini.WriteString("ProcessPopup", m_bNaturalPop);
 }

 void CAssemblyWeld::OnBnClickedButtonAdvancedContinue()
 {
	 WriteLog("单击：点云处理");
	 long long lTimeS = XI_clock();
	 // 执行点云处理
	 int nRobotNo = 0;
	 vector<CvPoint3D64f> vtPointCloud2;
	 vector<CvPoint3D32f> vtPointCloud2_32;
	 vector<CvPoint3D64f> vtPointCloud3;
	 vector<CvPoint3D32f> vtPointCloud3_32;
	 CUnit* pUnit = m_vpUnit[nRobotNo];
	 CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
	 CvPoint3D32f* pPointCloud2 = NULL;
	 int PointCloudSize2 = 0;
	 CvPoint3D32f* pPointCloud3 = NULL;
	 int PointCloudSize3 = 0;

	 vector<CvPoint3D64f> vtPointCloud;
	 CvPoint3D64f* pPointCloud = NULL;
	 int PointCloudSize = 0;
	 
	 CHECK_BOOL(CreateObject(m_tChoseWorkPieceType, pUnit, &m_pWeldAfterMeasure));
	 m_pWeldAfterMeasure->SetRecoParam();
	 //CStatisticalData* pStatisticalData = CStatisticalData::getInstance();

	 //CString PointCloudFileName_2 = "LineScan\\Gantry\\PointCloud\\Camera2_Thread0.txt";
	 //CString PointCloudFileName_3 = "LineScan\\Gantry\\PointCloud\\Camera3_Thread0.txt";
	 ////CStatisticalData* pStatisticalData = CStatisticalData::getInstance();
	 //if (false /*== g_bRemoveCloud*/)//开启去点云背景功能
	 //{
		// char ID2[] = "BackGround";
		// char ID3[] = "BackGround1";
		// long long l1 = XI_clock();
		// m_pWeldAfterMeasure->LoadContourData(pRobotDriver, PointCloudFileName_2, vtPointCloud2);
		// m_pWeldAfterMeasure->LoadContourData(pRobotDriver, PointCloudFileName_3, vtPointCloud3);
		// long long l2 = XI_clock();
		// WriteLog("加载线扫点云耗时：%dms", l2 - l1);
		// for (int p2 = 0; p2 < vtPointCloud2.size(); p2++)
		// {
		//	 CvPoint3D32f tmp;
		//	 tmp = cvPoint3D32f(vtPointCloud2[p2].x, vtPointCloud2[p2].y, vtPointCloud2[p2].z);
		//	 vtPointCloud2_32.push_back(tmp);
		// }
		// for (int p3 = 0; p3 < vtPointCloud3.size(); p3++)
		// {
		//	 CvPoint3D32f tmp;
		//	 tmp = cvPoint3D32f(vtPointCloud3[p3].x, vtPointCloud3[p3].y, vtPointCloud3[p3].z);
		//	 vtPointCloud3_32.push_back(tmp);
		// }
		// pPointCloud2 = (CvPoint3D32f*)vtPointCloud2_32.data();
		// PointCloudSize2 = vtPointCloud2_32.size();
		// pPointCloud3 = (CvPoint3D32f*)vtPointCloud3_32.data();
		// PointCloudSize3 = vtPointCloud3_32.size();
		// //if ((TRUE == m_bNaturalPop) && IDOK == XiMessageBox("是否扫描空工作台去除多于点云（执行一次即可）"))
		// //{
		//	// CString directory = _T("Local_Files\\ExtLib\\Vision\\BackgroundCloud");
		//	// m_pWeldAfterMeasure->DelPngFile(directory);//每次去背景前先删除已经有的背景文件
		//	// BackgroundCloudFileSave(ID2, pPointCloud2, PointCloudSize2, 1.0F, true);
		//	// BackgroundCloudFileSave(ID3, pPointCloud3, PointCloudSize3, 1.0F, true);
		//	// XiMessageBox("去点云背景处理成功");
		//	// return;
		// //}
		// l1 = XI_clock();
		////PointCloudSize2 = BackgroundCloudRemove(ID2, pPointCloud2, PointCloudSize2, 10.0F, 5.0F) - 1;
		// l2 = XI_clock();
		// l1 = XI_clock();
		////PointCloudSize3 = BackgroundCloudRemove(ID3, pPointCloud3, PointCloudSize3, 10.0F, 5.0F) - 1;
		// l2 = XI_clock();
		// WriteLog("去除工作台点云耗时：%dms", l2 - l1);

		// PointCloudSize = PointCloudSize2 + PointCloudSize3;
		// for (int pn = 0; pn < PointCloudSize2; pn++)
		// {
		//	 CvPoint3D64f t2 = cvPoint3D64f((double)pPointCloud2[pn].x, (double)pPointCloud2[pn].y, (double)pPointCloud2[pn].z);
		//	 vtPointCloud.push_back(t2);
		// }
		// for (int pn3 = 0; pn3 < PointCloudSize3; pn3++)
		// {
		//	 CvPoint3D64f t3 = cvPoint3D64f((double)pPointCloud3[pn3].x, (double)pPointCloud3[pn3].y, (double)pPointCloud3[pn3].z);
		//	 vtPointCloud.push_back(t3);
		// }

		// CString FileName = OUTPUT_PATH + pUnit->m_tContralUnit.strUnitName + REMOVE_CLOUD_PATH;
		// l1 = XI_clock();
		// pPointCloud = m_pWeldAfterMeasure->SaveRemoveCloud(pRobotDriver, FileName, vtPointCloud, PointCloudSize);
		// l2 = XI_clock();
		// WriteLog("保存去除工作台点云耗时：%dms", l2 - l1);
	 //}
	 int nCurUseTable = 0;
	 COPini opini2;
	 opini2.SetFileName(DATA_PATH + m_vpUnit[0]->m_tContralUnit.strUnitName + LINE_SCAN_PARAM);
	 opini2.SetSectionName("CurUseTableNo");
	 opini2.ReadString("CurUseTableNo", &nCurUseTable);

	 //线扫
	 CString PointCloudFileName = "LineScan\\GantryLeft\\PointCloud\\Scan5D.txt";
	 if (nCurUseTable != 0)
		 PointCloudFileName = "LineScan\\GantryRight\\PointCloud\\Scan5D.txt";
	 //CString PointCloudFileName = "LineScan\\Gantry\\PointCloud\\Scan5D.txt";

	 //CStatisticalData* pStatisticalData = CStatisticalData::getInstance();
	 if (false /*== g_bRemoveCloud*/)//开启去点云背景功能
	 {
		 char ID[] = "BackGround";
		 long long l1 = XI_clock();
		 m_pWeldAfterMeasure->LoadContourData(pRobotDriver, PointCloudFileName, vtPointCloud2);
		 long long l2 = XI_clock();
		 WriteLog("加载线扫点云耗时：%dms", l2 - l1);
		 for (int p2 = 0; p2 < vtPointCloud2.size(); p2++)
		 {
			 CvPoint3D32f tmp;
			 tmp = cvPoint3D32f(vtPointCloud2[p2].x, vtPointCloud2[p2].y, vtPointCloud2[p2].z);
			 vtPointCloud2_32.push_back(tmp);
		 }
		 pPointCloud2 = (CvPoint3D32f*)vtPointCloud2_32.data();
		 PointCloudSize2 = vtPointCloud2_32.size();
		 if ((TRUE == m_bNaturalPop) && IDOK == XiMessageBox("是否扫描空工作台去除多于点云（执行一次即可）"))
		 {
			 CString directory = _T("Local_Files\\ExtLib\\Vision\\BackgroundCloud");
			 m_pWeldAfterMeasure->DelPngFile(directory);//每次去背景前先删除已经有的背景文件
			 BackgroundCloudFileSave(ID, pPointCloud2, PointCloudSize2, 1.0F, true);
			 XiMessageBox("去点云背景处理成功");
			 return;
		 }
		 l1 = XI_clock();
		 PointCloudSize2 = BackgroundCloudRemove(ID, pPointCloud2, PointCloudSize2, 10.0F, 5.0F) - 1;
		 l2 = XI_clock();
		 WriteLog("去除工作台点云耗时：%dms", l2 - l1);
		 vtPointCloud.clear();
		 for (int pn = 0; pn < PointCloudSize2; pn++)
		 {
			 CvPoint3D64f t2 = cvPoint3D64f((double)pPointCloud2[pn].x, (double)pPointCloud2[pn].y, (double)pPointCloud2[pn].z);
			 vtPointCloud.push_back(t2);
		 }

		 CString FileName = OUTPUT_PATH + pUnit->m_tContralUnit.strUnitName + REMOVE_CLOUD_PATH;
		 pPointCloud = m_pWeldAfterMeasure->SaveRemoveCloud(pRobotDriver, FileName, vtPointCloud, PointCloudSize);
	 }
	 if (false == m_pWeldAfterMeasure->PointCloudProcess(TRUE == m_nUseModel, pPointCloud, PointCloudSize))
	 {
		 XiMessageBox("识别处理失败!");
		// m_lWorkTime += (XI_clock() - lTimeS); // 工作时间中的点云处理按钮时间

		 //pStatisticalData->UpdateWorkTime((XI_clock() - lTimeS));
		 return;	
	 }

	 // 显示交互界面
	 RunInteractiveWindow(pUnit->m_tContralUnit.strUnitName);
     // 自动分组
	 AutomaticGrouping(pUnit->m_tContralUnit.strUnitName);
	 XiMessageBox("接口更新完成");
	 //m_lWorkTime += (XI_clock() - lTimeS); // 工作时间中的点云处理按钮时间
	 //pStatisticalData->UpdateWorkTime((XI_clock() - lTimeS));
	 return;
 }

 void CAssemblyWeld::OnBnClickedButtonTablePara()
 {
	 /*
     端点续焊功能：
     1、检查当前状态
     2、获取当前焊接序号
     3、获取焊接状态
     4、获取焊接工件类型
     5、获取停止时焊接位置
     6、获取焊接特性，（已知终点，未知终点，先测后焊，是否跟踪，隔板类都是已知起始终止点只需记录当前焊接长度、当前焊接份数、如果是跟踪需要进行锁定
     不跟踪则不需要）
     */
     //获取工件类型
	 WriteLog("单击：断点续焊");
	 for (int i = 0; i < m_vpUnit.size(); i++)
	 {
		 CHECK_BOOL(m_vpUnit[i]->CheckIsReadyRun());
		 m_vpUnit[i] -> m_bBreakPointContinue = true;
		 m_vpUnit[i]->GetRobotCtrl()->m_eThreadStatus = INCISEHEAD_THREAD_STATUS_START;
	 }
	 AfxBeginThread(ThreadWeldAfterMeasure, this);
	 return;
 }

 void CAssemblyWeld::OnBnClickedCheck3()
 {
	 UpdateData(TRUE);
	 if (m_bCleanGunEnable)
	 {
		 WriteLog("单击：开启清枪");
	 }
	 else
	 {
		 WriteLog("单击：关闭清枪");
	 }
	 COPini opini;
	 opini.SetFileName(OPTIONAL_FUNCTION);
	 opini.SetSectionName("Popup");
	 opini.WriteString("CleanGunPopup", m_bCleanGunEnable);
 }

 void CAssemblyWeld::OnBnClickedCheckGray()
 {
	 UpdateData(TRUE);
	 if (m_bMeausreThickEnable)
	 {
		 WriteLog("单击：开启测厚");
	 }
	 else
	 {
		 WriteLog("单击：关闭测厚");
	 }
	 COPini opini;
	 opini.SetFileName(OPTIONAL_FUNCTION);
	 opini.SetSectionName("Popup");
	 opini.WriteString("EnableMeasureThick", m_bMeausreThickEnable);
 }

 void CAssemblyWeld::OnBnClickedCheckGray2()
 {
	 // TODO: 在此添加控件通知处理程序代码
	 UpdateData(TRUE);
	 if (m_bNeedWrap)
	 {
		 WriteLog("单击：包角");
	 }
	 else
	 {
		 WriteLog("单击：不包角");
	 }
	 COPini opini;
	 opini.SetFileName(OPTIONAL_FUNCTION);
	 opini.SetSectionName("Popup");
	 opini.WriteString("NeedWrap", m_bNeedWrap);
 }

 void CAssemblyWeld::OnBnClickedCheckGray3()
 {
	 // TODO: 在此添加控件通知处理程序代码
	 // 备用 备用
	 //UpdateData(TRUE);
	 //if (true)
	 //{
		// WriteLog("单击：立焊向上");
	 //}
	 //else
	 //{
		// WriteLog("单击：立焊向下");
	 //}
	 //COPini opini;
	 //opini.SetFileName(OPTIONAL_FUNCTION);
	 //opini.SetSectionName("Popup");
 }

 void CAssemblyWeld::OnBnClickedCheckTeachPop()
 {
	 UpdateData(TRUE);	 
	 if (m_bTeachPop)
	 {
		 WriteLog("单击：打开示教弹窗");
	 } 
	 else
	 {
		 WriteLog("单击：关闭示教弹窗");
	 }
	 COPini opini;
	 opini.SetFileName(OPTIONAL_FUNCTION);
	 opini.SetSectionName("Popup");
	 opini.WriteString("TeachPopup", m_bTeachPop);
 }

 void CAssemblyWeld::OnBnClickedButtonSystemPara2()
 {
	 WriteLog("单击：交互界面");
	 int nRobotNo = 0;
	 CUnit* pUnit = m_vpUnit[nRobotNo];
	 CRobotDriverAdaptor *pRobotDriver = m_vpRobotDriver[nRobotNo];
	 CHECK_BOOL(CreateObject(m_tChoseWorkPieceType, pUnit, &m_pWeldAfterMeasure));
	 // 显示处理结果 交互界面
	 //m_pWeldAfterMeasure->LoadCloudProcessResultNew(OUTPUT_PATH + pUnit->m_tContralUnit.strUnitName + "\\" + POINT_CLOUD_IDENTIFY_RESULT);
	 //((GenericWeld*)m_pWeldAfterMeasure)->Segmentation();
	 RunInteractiveWindow(pUnit->m_tContralUnit.strUnitName);
	 m_pWeldAfterMeasure->SetFilePath();
	 //m_pWeldAfterMeasure->SplitCloudProcessResult(pUnit->m_tContralUnit.strUnitName);
	 // 自动分组
	 if (54 != m_vpRobotDriver[0]->m_nExternalAxleType)
	 {
		 //AutomaticGrouping(pUnit->m_tContralUnit.strUnitName);
	 }
	 
	 //////焊缝接口更改
	 //CHECK_BOOL(CreateObject(m_tChoseWorkPieceType, pUnit, &m_pWeldAfterMeasure));
	 //m_pWeldAfterMeasure->LoadCloudProcessResultNew("GraphData\\PointCloudIdentifyReaultAfter.txt");
	 //SmallPartsConvertor convertor;
	 //convertor.inputWelds(".\\GraphData\\PointCloudIdentifyReaultAfterOld.txt");
	 //convertor.compute(m_nRobotHangPos/*HANGING_POS*/, SORT_LEFT_TO_RIGHT);
	 //CDiaphragmWeld cDiaphragmWeld(pRobotDriver, E_DIAPHRAGM, m_pScanInitLeft);
	 //cDiaphragmWeld.InitWorkPieceReleveInfo(m_vpRobotDriver[0]);
	 XiMessageBox("接口更新完成");
	 return;
 }

 void CAssemblyWeld::OnCbnSelchangeComboTableGroup2()
 {
	 //提示
	 int nWorkTableNo = -1;
	 if (nWorkTableNo != m_comboTableGroupleft.GetCurSel())
	 {
		 int wait = nWorkTableNo;
		 nWorkTableNo = m_comboTableGroupleft.GetCurSel();

		 //CString str1 = "您正在尝试更改料台组，这将导致下次运行时程序会选择与当前不同的料台。\n";
		 //CString str2 = "请您确保当前上料台上已无可切割工件，且另一组料台上无障碍物，再点击确定！\n";
		 //CString str3 = "设备将先开到安全位置，再弹窗提示，请您耐心等候，不要尝试其它操作！\n您也可以点击取消以还原旧选项！";
		 //if (!XUI::MesBox::PopOkCancel(str1 + str2 + str3))
		 //{
			// nWorkTableNo = wait;
			// m_comboTableGroupleft.SetCurSel(nWorkTableNo);
			// return;
		 //}
	 }
	 else
	 {
		 return;
	 }

	 COPini opini;
	 opini.SetFileName(DATA_PATH + m_vpRobotDriver[0]->m_strRobotName + LINE_SCAN_PARAM);
	 opini.SetSectionName("CurUseTableNo");
	 opini.WriteString("CurUseTableNo", nWorkTableNo);
	 LoadRobotandCar(m_vpRobotDriver[0], nWorkTableNo);
	 XiMessageBox("更改工作台完毕！");
 }

 void CAssemblyWeld::OnBnClickedButtonTablePara2()
 {
	 WriteLog("单击：精度校验");
	 int nRobotNo = -1;
	 for (int i = 0; i < m_vpRobotDriver.size(); i++)
	 {
		 if (1 == XUI::MesBox::PopOkCancel("{0} 验证？", m_vpRobotDriver[i]->m_strCustomName))
		 {
			 nRobotNo = m_vpRobotDriver[i]->m_nRobotNo;
			 break;
		 }
	 }
	 if (nRobotNo < 0) return;

	 if (1 == XiMessageBox("坐标转换验证？"))
	 {
		 // 坐标转换验证
		 CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
		 T_ROBOT_COORS tRobotCoors;
		 T_ROBOT_COORS tReadRobotCoors = pRobotDriver->GetCurrentPos();
		 T_ANGLE_PULSE tRobotPulse = pRobotDriver->GetCurrentPulse();
		 pRobotDriver->RobotKinematics(tRobotPulse, pRobotDriver->m_tTools.tGunTool, tRobotCoors);
		 pRobotDriver->SetPosVar(99, tRobotCoors);
		 return;
	 }
	 else if (1 == XiMessageBox("计算相机工具？"))
	 {
		 // 计算相机工具
		 int nCameraNo = 1 == XiMessageBox("确定相机0  取消相机1") ? 0 : 1;
		 T_ROBOT_COORS tCamTool;
		 GetCameraTool(nRobotNo, nCameraNo, tCamTool);
		 return;
	 }
	 else if (1 == XiMessageBox("枪尖坐标 转 相机工具？"))
	 {
		 // 枪尖坐标 转 相机工具 走到 验证
		 CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
		 T_ROBOT_COORS tGunCoord;
		 T_ROBOT_COORS tCamCoord;
		 tGunCoord = pRobotDriver->GetCurrentPos();
		 pRobotDriver->MoveToolByWeldGun(tGunCoord, pRobotDriver->m_tTools.tGunTool, tGunCoord, pRobotDriver->m_tTools.tCameraTool, tCamCoord);
		 pRobotDriver->SetPosVar(99, tCamCoord);
		 return;
	 }
	 else if (1 == XiMessageBox("相机手眼精度验证？"))
	 {
		 // 相机手眼精度验证
		 CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
		 CUnit* pUnit = m_vpUnit[nRobotNo];
		 int nCameraNo = 0;
		 CvPoint cpInterPnt2D;
		 COPini opini;
		 opini.SetFileName(DEBUG_INI);
		 opini.SetSectionName("CheckHandEye");
		 opini.ReadString("CamType", &nCameraNo);
		 opini.ReadString("PtnX2D", &cpInterPnt2D.x);
		 opini.ReadString("PtnY2D", &cpInterPnt2D.y);

		 T_ROBOT_COORS tRobotCurCoord = pRobotDriver->GetCurrentPos();
		 T_ANGLE_PULSE tRobotCurPulses = pRobotDriver->GetCurrentPulse();
		 
		 T_ROBOT_COORS tWeldCoord;
		 if (1 < nCameraNo) // 
		 {
			 CvPoint2D64f dPoint = { double(cpInterPnt2D.x),double(cpInterPnt2D.y) };
			 //tWeldCoord = pUnit->GetMeasureInWorldLeaser(nCamNo, dPoint);
			 tWeldCoord = pUnit->GetMeasureInWorldLeaser(nCameraNo, cpInterPnt2D, tRobotCurCoord, tRobotCurPulses);
		 }
		 else
		 {
			 tWeldCoord = pUnit->TranImageToBase(nCameraNo, cpInterPnt2D, tRobotCurCoord, tRobotCurPulses);
		 }
		 
		 pRobotDriver->SetPosVar(99, tWeldCoord);
		 //已修改
		 XUI::MesBox::PopInfo("相机中心：{0:.4f} {1:.4f} {2:.4f} Cur: {3:.4f} {4:.4f} {5:.4f} ",
			 tWeldCoord.dX, tWeldCoord.dY, tWeldCoord.dZ, tRobotCurCoord.dX, tRobotCurCoord.dY, tRobotCurCoord.dZ);
		// XiMessageBox("相机中心：%f %f %f Cur: %f %f %f ", 
			// tWeldCoord.dX, tWeldCoord.dY, tWeldCoord.dZ, tRobotCurCoord.dX, tRobotCurCoord.dY, tRobotCurCoord.dZ);
	 }
	 else if (1 == XiMessageBox("龙门坐标转换关系验证"))
	 {
		 //转换test
		 CvPoint3D64f GantryPoint = { 4084.295,0,1859.581 };/*{ 4084.295,-278.772,1859.581 };2721.228*/
		 CvPoint3D64f RobotPoint = { 281.778,-5401.415,1859.222 };
		 CvPoint3D64f reRobotPoint = m_vpUnit[1]->TransCoor_Gantry2RobotNew(GantryPoint);
		 CvPoint3D64f reGantryPoint = m_vpUnit[1]->TransCoor_Robot2Gantry(RobotPoint);
		 //XiMessageBox("");
	 }
	 //else if (1 == XiMessageBox("是否计算清枪器角度"))
	 //{
		// XiAlgorithm alg;
		// XI_POINT p1, p2;
		// COPini opini;
		// opini.SetFileName("Data\\Debug.ini");
		// opini.SetSectionName("Cleaner");
		// opini.ReadString("p1.x", &p1.x);
		// opini.ReadString("p1.y", &p1.y);
		// opini.ReadString("p1.z", &p1.z);
		// opini.ReadString("p2.x", &p2.x);
		// opini.ReadString("p2.y", &p2.y);
		// opini.ReadString("p2.z", &p2.z);
		// double detX = p1.x - p2.x;
		// double detY = p1.y - p2.y;
		// double angle = alg.CalcArcAngle(detX, detY);
		// XiMessageBox("%.3lf", angle);
	 //}
	 return;
 }

 void CAssemblyWeld::OnBnClickedButtonPanoRecognition2()
 {
	 int nRobotNo = 0;
	 CUnit* pUnit = m_vpUnit[nRobotNo];
	 ChangeScanLineParam cInterface(pUnit);
	 cInterface.DoModal();
	 int nTableNo = m_comboTableGroupleft.GetCurSel();
	 LoadRobotandCar(m_vpRobotDriver[0], nTableNo);
	 return;
 }

 void CAssemblyWeld::OnBnClickedButtonTablePara3()
 {
	 // TODO: 在此添加控件通知处理程序代码
	 int nRobotNo = 0;
	 if (IDOK == XiMessageBox("1号机械臂参数调整"))
	 {
		 nRobotNo = 0;
	 }
	 else if (IDOK == XiMessageBox("2号机械臂参数调整"))
	 {
		 nRobotNo = 1;
	 }
	 else if (IDOK == XiMessageBox("3号机械臂参数调整"))
	 {
		 nRobotNo = 2;
	 }
	 else if (IDOK == XiMessageBox("4号机械臂参数调整"))
	 {
		 nRobotNo = 3;
	 }
	 CUnit* pUnit = m_vpUnit[nRobotNo];
	 double dFlatWeldHorComp;
	 double dFlatWeldHeightComp;
	 std::vector<CString> vStrName(0);
	 std::vector<double> vnInputData(0);
	 COPini opini;
	 //已修改
	 CString str1 = "平焊补偿: 水平方向:外加內减 高度方向:上加下减";
	 str1 = XUI::Languge::GetInstance().translate(str1.GetBuffer());
	 opini.SetFileName(DATA_PATH + pUnit->m_tContralUnit.strUnitName + WELD_PARAM_FILE);
	 if (TRUE == XiMessageBox("是否修改平焊  0° 90° 180° 270° 方向偏移量？"))
	 {
		 vStrName.push_back("平焊0°水平:");
		 vStrName.push_back("平焊0°垂直:");
		 vStrName.push_back("平焊90°水平:");
		 vStrName.push_back("平焊90°垂直:");
		 vStrName.push_back("平焊180°水平:");
		 vStrName.push_back("平焊180°垂直:");
		 vStrName.push_back("平焊270°水平:");
		 vStrName.push_back("平焊270°垂直:");
		 vStrName.push_back("跟踪 水平:");
		 vStrName.push_back("跟踪 垂直:");
		 opini.SetSectionName("WeldCompVal0");
		 opini.ReadString("FlatWeldHorComp", &dFlatWeldHorComp);
		 opini.ReadString("FlatWeldHeightComp", &dFlatWeldHeightComp);
		 vnInputData.push_back(dFlatWeldHorComp);
		 vnInputData.push_back(dFlatWeldHeightComp);
		 opini.SetSectionName("WeldCompVal90");
		 opini.ReadString("FlatWeldHorComp", &dFlatWeldHorComp);
		 opini.ReadString("FlatWeldHeightComp", &dFlatWeldHeightComp);
		 vnInputData.push_back(dFlatWeldHorComp);
		 vnInputData.push_back(dFlatWeldHeightComp);
		 opini.SetSectionName("WeldCompVal180");
		 opini.ReadString("FlatWeldHorComp", &dFlatWeldHorComp);
		 opini.ReadString("FlatWeldHeightComp", &dFlatWeldHeightComp);
		 vnInputData.push_back(dFlatWeldHorComp);
		 vnInputData.push_back(dFlatWeldHeightComp);
		 opini.SetSectionName("WeldCompVal270");
		 opini.ReadString("FlatWeldHorComp", &dFlatWeldHorComp);
		 opini.ReadString("FlatWeldHeightComp", &dFlatWeldHeightComp);
		 vnInputData.push_back(dFlatWeldHorComp);
		 vnInputData.push_back(dFlatWeldHeightComp);
		 opini.SetSectionName("WeldCompVal405");
		 opini.ReadString("FlatWeldHorComp", &dFlatWeldHorComp);
		 opini.ReadString("FlatWeldHeightComp", &dFlatWeldHeightComp);
		 vnInputData.push_back(dFlatWeldHorComp);
		 vnInputData.push_back(dFlatWeldHeightComp);
		 ParamInput cParamDlg("平焊补偿: 水平方向:外加內减 高度方向:上加下减", vStrName, &vnInputData);
		 cParamDlg.DoModal();
		 opini.SetSectionName("WeldCompVal0");
		 opini.WriteString("FlatWeldHorComp", vnInputData[0]);
		 opini.WriteString("FlatWeldHeightComp", vnInputData[1]);
		 opini.SetSectionName("WeldCompVal90");
		 opini.WriteString("FlatWeldHorComp", vnInputData[2]);
		 opini.WriteString("FlatWeldHeightComp", vnInputData[3]);
		 opini.SetSectionName("WeldCompVal180");
		 opini.WriteString("FlatWeldHorComp", vnInputData[4]);
		 opini.WriteString("FlatWeldHeightComp", vnInputData[5]);
		 opini.SetSectionName("WeldCompVal270");
		 opini.WriteString("FlatWeldHorComp", vnInputData[6]);
		 opini.WriteString("FlatWeldHeightComp", vnInputData[7]);
		 opini.SetSectionName("WeldCompVal405");
		 opini.WriteString("FlatWeldHorComp", vnInputData[8]);
		 opini.WriteString("FlatWeldHeightComp", vnInputData[9]);
	 }
	 else if (TRUE == XiMessageBox("是否修改平焊  45° 135° 225° 315° 方向偏移量？"))
	 {
		 vStrName.push_back("平焊45°水平:");
		 vStrName.push_back("平焊45°垂直:");
		 vStrName.push_back("平焊135°水平:");
		 vStrName.push_back("平焊135°垂直:");
		 vStrName.push_back("平焊225°水平:");
		 vStrName.push_back("平焊225°垂直:");
		 vStrName.push_back("平焊315°水平:");
		 vStrName.push_back("平焊315°垂直:");
		 opini.SetSectionName("WeldCompVal45");
		 opini.ReadString("FlatWeldHorComp", &dFlatWeldHorComp);
		 opini.ReadString("FlatWeldHeightComp", &dFlatWeldHeightComp);
		 vnInputData.push_back(dFlatWeldHorComp);
		 vnInputData.push_back(dFlatWeldHeightComp);
		 opini.SetSectionName("WeldCompVal135");
		 opini.ReadString("FlatWeldHorComp", &dFlatWeldHorComp);
		 opini.ReadString("FlatWeldHeightComp", &dFlatWeldHeightComp);
		 vnInputData.push_back(dFlatWeldHorComp);
		 vnInputData.push_back(dFlatWeldHeightComp);
		 opini.SetSectionName("WeldCompVal225");
		 opini.ReadString("FlatWeldHorComp", &dFlatWeldHorComp);
		 opini.ReadString("FlatWeldHeightComp", &dFlatWeldHeightComp);
		 vnInputData.push_back(dFlatWeldHorComp);
		 vnInputData.push_back(dFlatWeldHeightComp);
		 opini.SetSectionName("WeldCompVal315");
		 opini.ReadString("FlatWeldHorComp", &dFlatWeldHorComp);
		 opini.ReadString("FlatWeldHeightComp", &dFlatWeldHeightComp);
		 vnInputData.push_back(dFlatWeldHorComp);
		 vnInputData.push_back(dFlatWeldHeightComp);
		 ParamInput cParamDlg("平焊补偿: 水平方向:外加內减 高度方向:上加下减", vStrName, &vnInputData);
		 cParamDlg.DoModal();
		 opini.SetSectionName("WeldCompVal45");
		 opini.WriteString("FlatWeldHorComp", vnInputData[0]);
		 opini.WriteString("FlatWeldHeightComp", vnInputData[1]);
		 opini.SetSectionName("WeldCompVal135");
		 opini.WriteString("FlatWeldHorComp", vnInputData[2]);
		 opini.WriteString("FlatWeldHeightComp", vnInputData[3]);
		 opini.SetSectionName("WeldCompVal225");
		 opini.WriteString("FlatWeldHorComp", vnInputData[4]);
		 opini.WriteString("FlatWeldHeightComp", vnInputData[5]);
		 opini.SetSectionName("WeldCompVal315");
		 opini.WriteString("FlatWeldHorComp", vnInputData[6]);
		 opini.WriteString("FlatWeldHeightComp", vnInputData[7]);
	 }
	 return;
 }

 void CAssemblyWeld::OnBnClickedButtonTablePara4()
 {
	 // TODO: 在此添加控件通知处理程序代码
	 int nRobotNo = 0;
	 if (IDOK == XiMessageBox("1号机械臂参数调整"))
	 {
		 nRobotNo = 0;
	 }
	 else if (IDOK == XiMessageBox("2号机械臂参数调整"))
	 {
		 nRobotNo = 1;
	 }
	 else if (IDOK == XiMessageBox("3号机械臂参数调整"))
	 {
		 nRobotNo = 2;
	 }
	 else if (IDOK == XiMessageBox("4号机械臂参数调整"))
	 {
		 nRobotNo = 3;
	 }
	 CUnit* pUnit = m_vpUnit[nRobotNo];
	 double dStandWeldLenComp;
	 double dStandWeldVerComp;
	 std::vector<CString> vStrName(0);
	 std::vector<double> vnInputData(0);
	 COPini opini;
	 //已修改
	 CString str1 = "立焊补偿:焊丝方向:长加短减 焊丝垂直方向:逆加顺减";
	 str1 = XUI::Languge::GetInstance().translate(str1.GetBuffer());
	 opini.SetFileName(DATA_PATH + pUnit->m_tContralUnit.strUnitName + WELD_PARAM_FILE);
	 if (TRUE == XiMessageBox("是否修改立焊  45° 135° 225° 315° 方向偏移量？"))
	 {
		 vStrName.push_back("立焊45°水平:");
		 vStrName.push_back("立焊45°垂直:");
		 vStrName.push_back("立焊135°水平:");
		 vStrName.push_back("立焊135°垂直:");
		 vStrName.push_back("立焊225°水平:");
		 vStrName.push_back("立焊225°垂直:");
		 vStrName.push_back("立焊315°水平:");
		 vStrName.push_back("立焊315°垂直:");
		 opini.SetSectionName("StandWeldComp45");
		 opini.ReadString("StandWeldLenComp", &dStandWeldLenComp);
		 opini.ReadString("StandWeldVerComp", &dStandWeldVerComp);
		 vnInputData.push_back(dStandWeldLenComp);
		 vnInputData.push_back(dStandWeldVerComp);
		 opini.SetSectionName("StandWeldComp135");
		 opini.ReadString("StandWeldLenComp", &dStandWeldLenComp);
		 opini.ReadString("StandWeldVerComp", &dStandWeldVerComp);
		 vnInputData.push_back(dStandWeldLenComp);
		 vnInputData.push_back(dStandWeldVerComp);
		 opini.SetSectionName("StandWeldComp225");
		 opini.ReadString("StandWeldLenComp", &dStandWeldLenComp);
		 opini.ReadString("StandWeldVerComp", &dStandWeldVerComp);
		 vnInputData.push_back(dStandWeldLenComp);
		 vnInputData.push_back(dStandWeldVerComp);
		 opini.SetSectionName("StandWeldComp315");
		 opini.ReadString("StandWeldLenComp", &dStandWeldLenComp);
		 opini.ReadString("StandWeldVerComp", &dStandWeldVerComp);
		 vnInputData.push_back(dStandWeldLenComp);
		 vnInputData.push_back(dStandWeldVerComp);
		 ParamInput cParamDlg("立焊补偿:焊丝方向:长加短减 焊丝垂直方向:逆加顺减", vStrName, &vnInputData);
		 cParamDlg.DoModal();
		 opini.SetSectionName("StandWeldComp45");
		 opini.WriteString("StandWeldLenComp", vnInputData[0]);
		 opini.WriteString("StandWeldVerComp", vnInputData[1]);
		 opini.SetSectionName("StandWeldComp135");
		 opini.WriteString("StandWeldLenComp", vnInputData[2]);
		 opini.WriteString("StandWeldVerComp", vnInputData[3]);
		 opini.SetSectionName("StandWeldComp225");
		 opini.WriteString("StandWeldLenComp", vnInputData[4]);
		 opini.WriteString("StandWeldVerComp", vnInputData[5]);
		 opini.SetSectionName("StandWeldComp315");
		 opini.WriteString("StandWeldLenComp", vnInputData[6]);
		 opini.WriteString("StandWeldVerComp", vnInputData[7]);
	 }
	 else if (TRUE == XiMessageBox("是否修改立焊  0° 90° 180° 270° 方向偏移量？"))
	 {
		 vStrName.push_back("立焊0°水平:");
		 vStrName.push_back("立焊0°垂直:");
		 vStrName.push_back("立焊90°水平:");
		 vStrName.push_back("立焊90°垂直:");
		 vStrName.push_back("立焊180°水平:");
		 vStrName.push_back("立焊180°垂直:");
		 vStrName.push_back("立焊270°水平:");
		 vStrName.push_back("立焊270°垂直:");
		 opini.SetSectionName("StandWeldComp0");
		 opini.ReadString("StandWeldLenComp", &dStandWeldLenComp);
		 opini.ReadString("StandWeldVerComp", &dStandWeldVerComp);
		 vnInputData.push_back(dStandWeldLenComp);
		 vnInputData.push_back(dStandWeldVerComp);
		 opini.SetSectionName("StandWeldComp90");
		 opini.ReadString("StandWeldLenComp", &dStandWeldLenComp);
		 opini.ReadString("StandWeldVerComp", &dStandWeldVerComp);
		 vnInputData.push_back(dStandWeldLenComp);
		 vnInputData.push_back(dStandWeldVerComp);
		 opini.SetSectionName("StandWeldComp180");
		 opini.ReadString("StandWeldLenComp", &dStandWeldLenComp);
		 opini.ReadString("StandWeldVerComp", &dStandWeldVerComp);
		 vnInputData.push_back(dStandWeldLenComp);
		 vnInputData.push_back(dStandWeldVerComp);
		 opini.SetSectionName("StandWeldComp270");
		 opini.ReadString("StandWeldLenComp", &dStandWeldLenComp);
		 opini.ReadString("StandWeldVerComp", &dStandWeldVerComp);
		 vnInputData.push_back(dStandWeldLenComp);
		 vnInputData.push_back(dStandWeldVerComp);
		 ParamInput cParamDlg("立焊补偿:焊丝方向:长加短减 焊丝垂直方向:逆加顺减", vStrName, &vnInputData);
		 cParamDlg.DoModal();
		 opini.SetSectionName("StandWeldComp0");
		 opini.WriteString("StandWeldLenComp", vnInputData[0]);
		 opini.WriteString("StandWeldVerComp", vnInputData[1]);
		 opini.SetSectionName("StandWeldComp90");
		 opini.WriteString("StandWeldLenComp", vnInputData[2]);
		 opini.WriteString("StandWeldVerComp", vnInputData[3]);
		 opini.SetSectionName("StandWeldComp180");
		 opini.WriteString("StandWeldLenComp", vnInputData[4]);
		 opini.WriteString("StandWeldVerComp", vnInputData[5]);
		 opini.SetSectionName("StandWeldComp270");
		 opini.WriteString("StandWeldLenComp", vnInputData[6]);
		 opini.WriteString("StandWeldVerComp", vnInputData[7]);
	 }
	 return;
 }

 void CAssemblyWeld::OnBnClickedButtonLoadTrack2()
 {
	 //// TODO: 在此添加控件通知处理程序代码
	 //COPini opini;
	 //opini.SetFileName(SYSTEM_PARA_INI);
	 //opini.SetSectionName("TodaysPartNum");
	 //double dRatio = 1.0;
	 //if (opini.CheckExists(SYSTEM_PARA_INI, "TodaysPartNum", "Ratio"))
	 //{
		// opini.ReadString("Ratio", &dRatio);
	 //}

	 //XiMessageBox("联机时间：%s\n断机时间：%s\n总联机时间：%.3lf小时\n总断机时间：%.3lf小时\n\n工作时间：%.3lf小时(%.3lf分钟)\
		// \n待机时间： % .3lf小时(% .3lf分钟)\n平焊总用时：%.3lf小时(%.3lf分钟)\n立焊总用时：%.3lf小时(%.3lf分钟)\n扫描总用时：%.3lf小时(%.3lf分钟)\
  //       \n扫描焊接总用时：%.3lf小时(%.3lf分钟)\n\n平焊总长度：%.3lf米\n立焊总长度：%.3lf米\n焊缝总长度：%.3lf米\n",
		// m_sFirstOpenTime, m_sLastCloseTime, (double)m_lTotalOpenTime / (3600.0 * 1000), (double)m_lTotalColseTime / (3600.0 * 1000), 
		// (double)m_lWorkTime / (3600.0 * 1000.0), (double)m_lWorkTime / 1000.0 / 60.0, (double)m_lStandbyTime / (3600.0 * 1000.0), (double)m_lStandbyTime / 1000.0 / 60.0,
		// (double)m_lFlatWeldTime / (3600.0 * 1000.0), (double)m_lFlatWeldTime / 1000.0 / 60.0, (double)m_lStandWeldTime / (3600.0 * 1000.0), (double)m_lStandWeldTime / 1000.0 / 60.0,
		// (double)m_lScanTime / (3600.0 * 1000.0), (double)m_lScanTime / 1000.0 / 60.0, (double)m_lScanWeldTime / (3600.0 * 1000.0), (double)m_lScanWeldTime / 1000.0 / 60.0,
		// m_dFlatWeldLen* dRatio, m_dStandWeldLen* dRatio, m_dTotalWeldLen* dRatio, m_dTotalScanLen* dRatio + m_dTotalWeldLen * dRatio);
	 //CStatisticalData* pStatisticalData = CStatisticalData::getInstance();
	 //pStatisticalData->UpDateStatisticsData();
	 //pStatisticalData->SendWeldData();
	 return;
 }

 void CAssemblyWeld::OnCbnSelchangeWorkpieceType()
 {
	 E_WORKPIECE_TYPE eOldType = m_tChoseWorkPieceType;
	 int nIndex = m_ctlWorkpieceType.GetCurSel();
	 CString str;
	 m_ctlWorkpieceType.GetLBText(nIndex, str);

	 std::map<int, CString>::iterator iter = m_nsPartType.begin();
	 for (; iter != m_nsPartType.end(); iter++)
	 {
		 //已修改
		 iter->second = XUI::Languge::GetInstance().translate(iter->second.GetBuffer());
		 if (iter->second == str)
		 {
			 break;
		 }
	 }
	 if (iter == m_nsPartType.end())
	 {
		 XiMessageBox("未选择正确工件");
		 m_tChoseWorkPieceType = E_LINELLAE;
	 }
	 m_tChoseWorkPieceType = (E_WORKPIECE_TYPE)iter->first;

	 CString s;
	 s.Format("%d", (int)m_tChoseWorkPieceType);
	 COPini::WriteString(SYSTEM_PARA_INI, "TotalPartType", "CurChooseType", s);
	 //已修改
	 XUI::Languge::GetInstance().translateDialog(this);
 }

 BOOL CAssemblyWeld::InitVariable()
 {
	 for (int i = 0; i < m_vpRobotDriver.size(); i++)
	 {
		 m_vpRobotDriver[i]->m_eThreadStatus = INCISEHEAD_THREAD_STATUS_START;
	 }

	 WriteLog("CAssemblyWeld: 初始化变量完毕");
	 return TRUE;
 }

 bool CAssemblyWeld::CleanGunH(CRobotDriverAdaptor* pRobotCtrl)
 {
	 //位置过低时先抬枪再回到安全位置
	 CUnit* pUnit = m_vpUnit[pRobotCtrl->m_nRobotNo];
	 BackHome();
	 CHECK_BOOL_RETURN(pUnit->CheckIsReadyRun());
	 pRobotCtrl->CallJob("CLEAR");
	 pUnit->RobotCheckDone();
	 BackHome();
	 return true;
	 //CUnit* pUnit = m_vpUnit[pRobotCtrl->m_nRobotNo];
	 //CHECK_BOOL_RETURN(pUnit->CheckIsReadyRun());
	 //double NowZ = pRobotCtrl->GetCurrentPos(ROBOT_AXIS_Z);
	 ///*double dLimitationZ = 1920;
	 //if (NowZ > dLimitationZ - 300)
	 //{
		// pRobotCtrl->PosMove(ROBOT_AXIS_Z, dLimitationZ - 310.0, 500, COORD_ABS);
		// pUnit->RobotCheckDone();
		// NowZ = pRobotCtrl->GetCurrentPos(ROBOT_AXIS_Z);
		// if (NowZ != dLimitationZ - 310.0)
		// {
		//	 if (TRUE == m_bNaturalPop)
		//	 {
		//		 XiMessageBox("当前位置过低，自动抬枪失败，请手动抬枪");
		//	 }
		//	 return FALSE;
		// }
	 //}*/
	 ////OnBnClickedButtonBackHome();
	 //pUnit->SwitchIO("CleanGun", false); //关闭清枪
	 //Sleep(50);
	 //pUnit->SwitchIO("CutSilk", false); //松开剪丝(加紧)
	 //Sleep(50);
	 //pUnit->SwitchIO("FuelInjection", false); //关闭喷油
	 //Sleep(50);

	 //T_ROBOT_COORS tCurrentRobotCoors;
	 //T_ROBOT_COORS tQQCoors;// 清枪点
	 //T_ROBOT_COORS tJSCoors;// 剪丝点

	 //if (0 == pRobotCtrl->m_nRobotNo)
	 //{
		// tQQCoors.dX = -44.500;
		// tQQCoors.dY = 463.552;
		// tQQCoors.dZ = -811.254;
		// tQQCoors.dRX = -89.8377;
		// tQQCoors.dRY = 55.2306;
		// tQQCoors.dRZ = -179.45;

		// tJSCoors.dX = 55.548;
		// tJSCoors.dY = 482.000;
		// tJSCoors.dZ = -829.174;
		// tJSCoors.dRX = -89.8414;
		// tJSCoors.dRY = 55.2301;
		// tJSCoors.dRZ = -177.45;
	 //}
	 //else if (1 == pRobotCtrl->m_nRobotNo)
	 //{
		// tQQCoors.dX = -52.031;
		// tQQCoors.dY = 462.357;
		// tQQCoors.dZ = -813.541;
		// tQQCoors.dRX = -89.8569;
		// tQQCoors.dRY = 55.2306;
		// tQQCoors.dRZ = -179.500;

		// tJSCoors.dX = 48.107;
		// tJSCoors.dY = 488.200;
		// tJSCoors.dZ = -832.8954;
		// tJSCoors.dRX = -89.8412;
		// tJSCoors.dRY = 55.2301;
		// tJSCoors.dRZ = -179.500;
	 //}
	 //else
	 //{
		// XiMessageBox("机器人选择失败，停止清枪");
		// return false;
	 //}
	 ////T_ROBOT_COORS tPYCoors;// 喷油点
	 ////tPYCoors.dX = -1216.419;
	 ////tPYCoors.dY = 677.719;
	 ////tPYCoors.dZ = 1014.599;
	 ////tPYCoors.dRX = 0;
	 ////tPYCoors.dRY = 30;
	 ////tPYCoors.dRZ = 180;
	 //T_ANGLE_PULSE tResultPulse;
	 //T_ROBOT_MOVE_SPEED tPulseMove;
	 //T_ROBOT_MOVE_SPEED tPulseMove2;
	 //T_ROBOT_MOVE_SPEED tPulseMove3;
	 //tPulseMove.dSpeed = 1000;
	 //tPulseMove.dACC = 20;
	 //tPulseMove.dDEC = 20;
	 //tPulseMove2.dSpeed = 1000;
	 //tPulseMove2.dACC = 20;
	 //tPulseMove2.dDEC = 20;
	 //tPulseMove3.dSpeed = 1000;
	 //tPulseMove3.dACC = 20;
	 //tPulseMove3.dDEC = 20;
	 //T_ROBOT_MOVE_INFO tRobotMoveInfo;
	 //vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo;
	 //vtRobotMoveInfo.clear();

	 //COPini opini;
	 //opini.SetFileName(DATA_PATH + pUnit->m_tContralUnit.strUnitName + LINE_SCAN_PARAM);
	 //opini.SetSectionName("Table0");

	 //T_ANGLE_PULSE tCleanInitPlus;//清枪标志位(关节)
	 //T_ROBOT_COORS tCleanInitCoord; //清枪标志位(直角) 
	 //opini.ReadString("CleanInitPlus.nSPulse", &tCleanInitPlus.nSPulse);
	 //opini.ReadString("CleanInitPlus.nLPulse", &tCleanInitPlus.nLPulse);
	 //opini.ReadString("CleanInitPlus.nUPulse", &tCleanInitPlus.nUPulse);
	 //opini.ReadString("CleanInitPlus.nRPulse", &tCleanInitPlus.nRPulse);
	 //opini.ReadString("CleanInitPlus.nBPulse", &tCleanInitPlus.nBPulse);
	 //opini.ReadString("CleanInitPlus.nTPulse", &tCleanInitPlus.nTPulse);
	 //pRobotCtrl->RobotKinematics(tCleanInitPlus, pRobotCtrl->m_tTools.tGunTool, tCurrentRobotCoors);
	 ////pRobotCtrl->RobotKinematics(pRobotCtrl->CleanInitPlus, pRobotCtrl->m_tFirstTool, tCurrentRobotCoors);

	 //if (tCurrentRobotCoors.dX == 0 && tCurrentRobotCoors.dY == 0 && tCurrentRobotCoors.dZ == 0)
	 //{
		// XiMessageBox("清枪原点位置出错");
		// return FALSE;
	 //}
	 //if (tCurrentRobotCoors.dX < -5000)
	 //{
		// XiMessageBox("清枪原点位置出错");
		// return FALSE;
	 //}

	 ////tCurrentRobotCoors.dZ -= 150;

	 //T_ROBOT_COORS oldtCurrentRobotCoors = tCurrentRobotCoors;
	 //bool Transresult = pRobotCtrl->RobotInverseKinematics(tCurrentRobotCoors, tCleanInitPlus, pRobotCtrl->m_tFirstTool, tResultPulse);
	 //if (Transresult == FALSE)
	 //{
		// XiMessageBox("机器人无法到达清枪位置");
		// return FALSE;
	 //}

	 //long oldadplus[6];
	 //oldadplus[0] = pRobotCtrl->GetCurrentPulse(SAxis);
	 //oldadplus[1] = pRobotCtrl->GetCurrentPulse(LAxis);
	 //oldadplus[2] = pRobotCtrl->GetCurrentPulse(UAxis);
	 //oldadplus[3] = pRobotCtrl->GetCurrentPulse(RAxis);
	 //oldadplus[4] = pRobotCtrl->GetCurrentPulse(BAxis);
	 //oldadplus[5] = pRobotCtrl->GetCurrentPulse(TAxis);

	 ///*if (pRobotCtrl->m_nRobotNo == 0)
	 //{
		// tCurrentRobotCoors.dX += -45;
		// tCurrentRobotCoors.dY += 33.1;
	 //}
	 //else
	 //{
		// tCurrentRobotCoors.dX += 39.81;
		// tCurrentRobotCoors.dY += 30;
	 //}*/

	 //bool Transresult2 = pRobotCtrl->RobotInverseKinematics(tCurrentRobotCoors, tCleanInitPlus, pRobotCtrl->m_tFirstTool, tResultPulse);
	 //if (Transresult2 == FALSE)
	 //{
		// XiMessageBox("机器人无法到达清枪位置");
		// return FALSE;
	 //}
	 //tRobotMoveInfo = pRobotCtrl->PVarToRobotMoveInfo(0, tResultPulse, tPulseMove2, MOVJ);
	 //vtRobotMoveInfo.push_back(tRobotMoveInfo);

	 //// 计算清枪过渡点
	 //T_ROBOT_COORS tCalQQCoors;// 计算清枪中间点
	 //tCalQQCoors = tQQCoors;
	 //tCalQQCoors.dY += 150;
	 //tCalQQCoors.dX -= 5;
	 ////tCalQQCoors.dZ -= 150;
	 //tRobotMoveInfo = pRobotCtrl->PVarToRobotMoveInfo(1, tCalQQCoors, tPulseMove2, MOVL);
	 //vtRobotMoveInfo.push_back(tRobotMoveInfo);

	 //tCalQQCoors.dY -= 150;
	 ////tCalQQCoors.dX -= 3;
	 //tRobotMoveInfo = pRobotCtrl->PVarToRobotMoveInfo(2, tCalQQCoors, tPulseMove, MOVL);
	 //vtRobotMoveInfo.push_back(tRobotMoveInfo);
	 //tRobotMoveInfo = pRobotCtrl->PVarToRobotMoveInfo(3, tQQCoors, tPulseMove, MOVL);
	 //vtRobotMoveInfo.push_back(tRobotMoveInfo);
	 //pRobotCtrl->SetMoveValue(vtRobotMoveInfo);
	 //pRobotCtrl->CallJob("CONTIMOVANY");
	 //pUnit->RobotCheckDone();
	 //vtRobotMoveInfo.clear();
	 ////XiMessageBox("检查是否在清枪点斜上方（调试完弹框删除）");
	 //pUnit->SwitchIO("CleanGun", true); //开启清枪
	 //Sleep(2000);
	 //pUnit->SwitchIO("CleanGun", false); //关闭清枪
	 //Sleep(1000);

	 //pUnit->SwitchIO("FuelInjection", true); //开启喷油
	 //Sleep(2000);
	 //pUnit->SwitchIO("FuelInjection", false); //关闭喷油
	 //Sleep(100);
	 //// 清完回清枪中间点
	 //tCalQQCoors.dY += 150;
	 //tRobotMoveInfo = pRobotCtrl->PVarToRobotMoveInfo(0, tCalQQCoors, tPulseMove2, MOVL);
	 //vtRobotMoveInfo.push_back(tRobotMoveInfo);


	 //T_ROBOT_COORS tCalJSCoors;// 计算剪丝中间点
	 //tCalJSCoors = tJSCoors;
	 //tCalJSCoors.dY += 150;
	 ////tCalJSCoors.dZ -= 150;
	 ////tCalJSCoors.dZ += 30;
	 //tRobotMoveInfo = pRobotCtrl->PVarToRobotMoveInfo(1, tCalJSCoors, tPulseMove, MOVL);
	 //vtRobotMoveInfo.push_back(tRobotMoveInfo);


	 //// 剪丝偏30 从侧面进去
	 //tJSCoors.dZ += 30;
	 //tRobotMoveInfo = pRobotCtrl->PVarToRobotMoveInfo(2, tJSCoors, tPulseMove, MOVL);
	 //vtRobotMoveInfo.push_back(tRobotMoveInfo);
	 //pRobotCtrl->SetMoveValue(vtRobotMoveInfo);
	 //pRobotCtrl->CallJob("CONTIMOVANY");
	 //pUnit->RobotCheckDone();
	 //vtRobotMoveInfo.clear();
	 //pRobotCtrl->CallJob("SS");
	 //Sleep(500);

	 //// 剪丝点
	 //tJSCoors.dZ -= 30;
	 //tRobotMoveInfo = pRobotCtrl->PVarToRobotMoveInfo(0, tJSCoors, tPulseMove2, MOVL);
	 //vtRobotMoveInfo.push_back(tRobotMoveInfo);
	 //pRobotCtrl->SetMoveValue(vtRobotMoveInfo);
	 //pRobotCtrl->CallJob("CONTIMOVANY");
	 //pUnit->RobotCheckDone();
	 //vtRobotMoveInfo.clear();

	 //
	 //pUnit->SwitchIO("CutSilk", true); //剪丝(夹紧)
	 //Sleep(500);
	 //pUnit->SwitchIO("CutSilk", false); //松开剪丝
	 //Sleep(100);

	 ///*tCalJSCoors.dY -= 30*/;
	 //tRobotMoveInfo = pRobotCtrl->PVarToRobotMoveInfo(0, tCalJSCoors, tPulseMove2, MOVL);
	 //vtRobotMoveInfo.push_back(tRobotMoveInfo);
	 //pRobotCtrl->SetMoveValue(vtRobotMoveInfo);
	 //pRobotCtrl->CallJob("CONTIMOVANY");
	 //pUnit->RobotCheckDone();
	 //vtRobotMoveInfo.clear();

	 //// 回最开始位置
	 //pRobotCtrl->MoveToAbsPluse(oldadplus, 1000);
	 //pUnit->RobotCheckDone();

	 ////------------------------------

	 //return true;
 }


 //bool CAssemblyWeld::CleanGunH(CRobotDriverAdaptor* pRobotCtrl)
 //{
	// //位置过低时先抬枪再回到安全位置
	// CUnit* pUnit = m_vpUnit[pRobotCtrl->m_nRobotNo];
	// CHECK_BOOL_RETURN(pUnit->CheckIsReadyRun());
	// double NowZ = pRobotCtrl->GetCurrentPos(ROBOT_AXIS_Z);
	// OnBnClickedButtonBackHome();
	// //if (NowZ < pRobotCtrl->m_tRobotLimitation.drTableZ + 550)
	// //{
	//	// pRobotCtrl->PosMove(ROBOT_AXIS_Z, pRobotCtrl->m_tRobotLimitation.drTableZ + 550.0, 500, COORD_ABS);
	//	// pRobotCtrl->CheckRobotDone();
	//	// NowZ = pRobotCtrl->GetCurrentPos(ROBOT_AXIS_Z);
	//	// if (NowZ != pRobotCtrl->m_tRobotLimitation.drTableZ + 550.0)
	//	// {
	//	//	 if (TRUE == m_bNaturalPop)
	//	//	 {
	//	//		 XiMessageBox("当前位置过低，自动抬枪失败，请手动抬枪");
	//	//	 }
	//	//	 return FALSE;
	//	// }
	// //}

	// XiAlgorithm alg;
	// vector<T_ROBOT_COORS> vtRobotCoor;
	// MP_USR_VAR_INFO tUsrVarInfo;
	// vector<MP_USR_VAR_INFO> vtUsrVarInfo;
	// COPini opini;
	// opini.SetFileName("./data/RobotAndCar.ini");
	// opini.SetSectionName("ClearGunParam");

	// T_ANGLE_PULSE tCleanInitPlus;//清枪标志位(关节)
	// T_ROBOT_COORS tCleanInitCoord; //清枪标志位(直角) 
	// opini.ReadString("CleanInitPlus.nSPulse", &tCleanInitPlus.nSPulse);
	// opini.ReadString("CleanInitPlus.nLPulse", &tCleanInitPlus.nLPulse);
	// opini.ReadString("CleanInitPlus.nUPulse", &tCleanInitPlus.nUPulse);
	// opini.ReadString("CleanInitPlus.nRPulse", &tCleanInitPlus.nRPulse);
	// opini.ReadString("CleanInitPlus.nBPulse", &tCleanInitPlus.nBPulse);
	// opini.ReadString("CleanInitPlus.nTPulse", &tCleanInitPlus.nTPulse);
	// pRobotCtrl->RobotKinematics(tCleanInitPlus, pRobotCtrl->m_tTools.tGunTool, tCleanInitCoord);

	// T_ROBOT_COORS tClearShifting;   //清枪偏移距离
	// T_ROBOT_COORS tCutSilkShifting; //剪丝偏移距离
	// opini.ReadString("ClearShifting.dX", &tClearShifting.dX);
	// opini.ReadString("ClearShifting.dY", &tClearShifting.dY);
	// opini.ReadString("ClearShifting.dZ", &tClearShifting.dZ);
	// opini.ReadString("CutSilkShifting.dX", &tCutSilkShifting.dX);
	// opini.ReadString("CutSilkShifting.dY", &tCutSilkShifting.dY);
	// opini.ReadString("CutSilkShifting.dZ", &tCutSilkShifting.dZ);

	// double realCleanerAngle;		//清枪器实际安装角度
	// opini.ReadString("RealCleanerAngle", &realCleanerAngle);

	// //清枪点
	// T_ROBOT_COORS tClearGunPoint(tCleanInitCoord);
	// tClearGunPoint.dX = tClearGunPoint.dX + tClearShifting.dX;
	// tClearGunPoint.dY = tClearGunPoint.dY + tClearShifting.dY * SinD(realCleanerAngle) * pRobotCtrl->m_nRobotInstallDir;
	// tClearGunPoint.dZ = tClearGunPoint.dZ + tClearShifting.dZ * pRobotCtrl->m_nRobotInstallDir;

	// //剪丝点
	// T_ROBOT_COORS tCutSilkPoint(tCleanInitCoord);
	// tCutSilkPoint.dX = tCutSilkPoint.dX + tCutSilkShifting.dX;
	// tCutSilkPoint.dY = tCutSilkPoint.dY + tCutSilkShifting.dY * SinD(realCleanerAngle) * pRobotCtrl->m_nRobotInstallDir;
	// tCutSilkPoint.dZ = tCutSilkPoint.dZ + tCutSilkShifting.dZ * pRobotCtrl->m_nRobotInstallDir;

	// //清枪过渡点(下)
	// T_ROBOT_COORS tClearGunExcessivePointBefor(tClearGunPoint);
	// tClearGunExcessivePointBefor.dX -= 4.0;

	// //清枪过渡点(上)
	// T_ROBOT_COORS tClearGunExcessivePoint(tClearGunExcessivePointBefor);
	// tClearGunExcessivePoint.dZ += 100;

	// //剪丝过渡点
	// T_ROBOT_COORS tCutSilkExcssivePoint(tCutSilkPoint);
	// tCutSilkExcssivePoint.dZ += 100;

	// //清枪过渡点(上)
	// vtRobotCoor.push_back(tClearGunExcessivePoint);
	// //清枪过渡点(下)
	// vtRobotCoor.push_back(tClearGunExcessivePointBefor);
	// //清枪点
	// vtRobotCoor.push_back(tClearGunPoint);
	// //剪丝过渡点
	// vtRobotCoor.push_back(tCutSilkExcssivePoint);
	// //剪丝点
	// vtRobotCoor.push_back(tCutSilkPoint);
	// //安全位置
	// tUsrVarInfo = pRobotCtrl->PrepareValData(0, pRobotCtrl->m_tHomePulse);
	// vtUsrVarInfo.push_back(tUsrVarInfo);
	// for (int i = 0; i < 5; i++)
	// {
	//	 tUsrVarInfo = pRobotCtrl->PrepareValData(i + 1, vtRobotCoor[i]);
	//	 vtUsrVarInfo.push_back(tUsrVarInfo);
	// }
	// pRobotCtrl->SetMultiVar_H(vtUsrVarInfo);

	// pRobotCtrl->CallJob("CLEARGUN");

	// pUnit->RobotCheckDone();
	// CHECK_PULSE_RETURN_BOOL(pRobotCtrl, pRobotCtrl->m_tHomePulse);

	// return true;
 //}

 bool CAssemblyWeld::CleanGun(CRobotDriverAdaptor* pRobotCtrl)
 {
	 int config[7] = { 0,0,0,0,0,0,0 };
	 // XiMessageBox("检查机器人是否在安全位置（调试完弹框删除）");
	 CUnit* pUnit = m_vpUnit[pRobotCtrl->m_nRobotNo];
	 COPini opini;
	 opini.SetFileName(DATA_PATH + pUnit->m_tContralUnit.strUnitName + Clean_Gun);
	 T_ANGLE_PULSE tCleanGunPlus;//清枪标志位(关节)
	 T_ROBOT_COORS tCleanGunCoord; //清枪标志位(直角) 
	 T_ROBOT_COORS tCutSilkCoord;//剪丝
	 T_ROBOT_COORS tSprayOilCoord;//喷油
	 T_ANGLE_PULSE Pulse = pRobotCtrl->GetCurrentPulse();
	 T_ROBOT_COORS Pos = pRobotCtrl->GetCurrentPos();
	 tCleanGunCoord = Pos;
	 tCutSilkCoord = Pos;
	 tSprayOilCoord = Pos;
	 opini.SetSectionName("CleanGun");
	 opini.ReadString("CleanGun.X", &tCleanGunCoord.dX);
	 opini.ReadString("CleanGun.Y", &tCleanGunCoord.dY);
	 opini.ReadString("CleanGun.Z", &tCleanGunCoord.dZ);
	 opini.ReadString("CleanGun.RX", &tCleanGunCoord.dRX);
	 opini.ReadString("CleanGun.RY", &tCleanGunCoord.dRY);
	 opini.ReadString("CleanGun.RZ", &tCleanGunCoord.dRZ);
	 T_ROBOT_COORS tCleanGunGDCoord(tCleanGunCoord);//清枪过渡点
	 tCleanGunGDCoord.dZ += 120 * pUnit->GetRobotCtrl()->m_nRobotInstallDir;
	 opini.SetSectionName("CutSilk");
	 opini.ReadString("CutSilk.X", &tCutSilkCoord.dX);
	 opini.ReadString("CutSilk.Y", &tCutSilkCoord.dY);
	 opini.ReadString("CutSilk.Z", &tCutSilkCoord.dZ);
	 opini.ReadString("CutSilk.RX", &tCutSilkCoord.dRX);
	 opini.ReadString("CutSilk.RY", &tCutSilkCoord.dRY);
	 opini.ReadString("CutSilk.RZ", &tCutSilkCoord.dRZ);
	 T_ROBOT_COORS tCutSilkGDCoord(tCutSilkCoord);//剪丝过渡点
	 tCutSilkGDCoord.dX += 30 * pUnit->GetRobotCtrl()->m_nRobotInstallDir;
	 tCutSilkGDCoord.dZ += 120 * pUnit->GetRobotCtrl()->m_nRobotInstallDir;
	 opini.SetSectionName("SprayOil");
	 opini.ReadString("SprayOil.X", &tSprayOilCoord.dX);
	 opini.ReadString("SprayOil.Y", &tSprayOilCoord.dY);
	 opini.ReadString("SprayOil.Z", &tSprayOilCoord.dZ);
	 opini.ReadString("SprayOil.RX", &tSprayOilCoord.dRX);
	 opini.ReadString("SprayOil.RY", &tSprayOilCoord.dRY);
	 opini.ReadString("SprayOil.RZ", &tSprayOilCoord.dRZ);
	 T_ROBOT_COORS tSprayOilGDCoord(tSprayOilCoord);//喷油过渡点
	 tSprayOilGDCoord.dZ += 120 * pUnit->GetRobotCtrl()->m_nRobotInstallDir;
	 opini.SetSectionName("ClearGunParam");
	 opini.ReadString("CleanInitPlus.nSPulse", &tCleanGunPlus.nSPulse);
	 opini.ReadString("CleanInitPlus.nLPulse", &tCleanGunPlus.nLPulse);
	 opini.ReadString("CleanInitPlus.nUPulse", &tCleanGunPlus.nUPulse);
	 opini.ReadString("CleanInitPlus.nRPulse", &tCleanGunPlus.nRPulse);
	 opini.ReadString("CleanInitPlus.nBPulse", &tCleanGunPlus.nBPulse);
	 opini.ReadString("CleanInitPlus.nTPulse", &tCleanGunPlus.nTPulse);
	 T_ROBOT_MOVE_SPEED tPulseMove;
	 tPulseMove.dSpeed = 1500;
	 tPulseMove.dACC = 100;
	 tPulseMove.dDEC = 100;
	 T_ROBOT_COORS tNowCoor = pRobotCtrl->GetCurrentPos();
	 T_ROBOT_COORS tHomeCoor;
	 pRobotCtrl->RobotKinematics(pRobotCtrl->m_tHomePulse, pRobotCtrl->m_tFirstTool, tHomeCoor);
	 if (tNowCoor.dZ < tHomeCoor.dZ)
	 {
		 tNowCoor.dZ += 150;
		 pRobotCtrl->MoveByJob(tNowCoor, tPulseMove, pRobotCtrl->m_nExternalAxleType, "MOVL", config);
		 pUnit->RobotCheckDone(200);
	 }
	 pRobotCtrl->MoveByJob(pRobotCtrl->m_tHomePulse, pRobotCtrl->m_tBackHomeSpeed, pRobotCtrl->m_nExternalAxleType, "MOVJ");
	 pUnit->RobotCheckDone(200);

	 pRobotCtrl->RobotInverseKinematics(tCleanGunGDCoord, pRobotCtrl->m_tHomePulse, pRobotCtrl->m_tFirstTool, tCleanGunPlus);

	 //===========================================================================================================	
	 tCleanGunPlus.lBXPulse = Pulse.lBXPulse;
	 tCleanGunPlus.lBYPulse = Pulse.lBYPulse;
	 tCleanGunPlus.lBZPulse = Pulse.lBZPulse;
	 pRobotCtrl->MoveByJob(tCleanGunPlus, tPulseMove, pRobotCtrl->m_nExternalAxleType, "MOVJ"); //回清枪过渡点
	 pUnit->RobotCheckDone(200);
	 pRobotCtrl->MoveByJob(tCleanGunCoord, tPulseMove, pRobotCtrl->m_nExternalAxleType, "MOVL", config); //回清枪点
	 pUnit->RobotCheckDone(200);
	 pUnit->SwitchIO("CleanGun", true); 
	 Sleep(2000);
	 pUnit->SwitchIO("CleanGun", false);
	 Sleep(200);
	 pRobotCtrl->MoveByJob(tCleanGunGDCoord, tPulseMove, pRobotCtrl->m_nExternalAxleType, "MOVL", config); //回清枪过渡点
	 pUnit->RobotCheckDone(200);

	 pRobotCtrl->MoveByJob(tCutSilkGDCoord, tPulseMove, pRobotCtrl->m_nExternalAxleType, "MOVL", config); //回剪丝过渡点
	 pUnit->RobotCheckDone(200);
	 pRobotCtrl->MoveByJob(tCutSilkCoord, tPulseMove, pRobotCtrl->m_nExternalAxleType, "MOVL", config); //回剪丝点
	 pUnit->RobotCheckDone(200);
	 pRobotCtrl->CallJob("CS");
	 pUnit->RobotCheckDone(200);
	 pUnit->SwitchIO("CutSilk", true); //剪丝(松开)
	 Sleep(1000);
	 pUnit->SwitchIO("CutSilk", false); //松开剪丝(加紧)
	 Sleep(200);
	 pRobotCtrl->MoveByJob(tCutSilkGDCoord, tPulseMove, pRobotCtrl->m_nExternalAxleType, "MOVL", config); //回剪丝过渡点
	 pUnit->RobotCheckDone(200);
	 pRobotCtrl->MoveByJob(tSprayOilGDCoord, tPulseMove, pRobotCtrl->m_nExternalAxleType, "MOVL", config); //回喷油过渡点
	 pUnit->RobotCheckDone(200);
	 pRobotCtrl->MoveByJob(tSprayOilCoord, tPulseMove, pRobotCtrl->m_nExternalAxleType, "MOVL", config); //回喷油点
	 pUnit->RobotCheckDone(200);
	 pUnit->SwitchIO("FuelInjection", true);
	 Sleep(2000);
	 pUnit->SwitchIO("FuelInjection", false); 
	 Sleep(200);
	 pRobotCtrl->MoveByJob(tSprayOilGDCoord, tPulseMove, pRobotCtrl->m_nExternalAxleType, "MOVL", config);  //回喷油过渡点
	 pUnit->RobotCheckDone(200);
	 pRobotCtrl->MoveByJob(pRobotCtrl->m_tHomePulse, pRobotCtrl->m_tBackHomeSpeed, pRobotCtrl->m_nExternalAxleType, "MOVJ");//回安全位置
	 pUnit->RobotCheckDone(200);

	 return 1;
 }

 vector<string> CAssemblyWeld::split(const string& str, const string& delim) {
	 vector<string> res;
	 if ("" == str) return res;
	 //先将要切割的字符串从string类型转换为char*类型  
	 char* strs = new char[str.length() + 1]; //不要忘了  
	 strcpy(strs, str.c_str());

	 char* d = new char[delim.length() + 1];
	 strcpy(d, delim.c_str());

	 char* p = strtok(strs, d);
	 while (p) {
		 string s = p; //分割得到的字符串转换为string类型  
		 res.push_back(s); //存入结果数组  
		 p = strtok(NULL, d);
	 }

	 return res;
 }
 /**
  * @brief 枚举对应组号的焊缝，根据对应焊缝的bWeldMode判断是否需要跟踪焊接，不需要则使用理论直接焊接，需要则走跟踪焊接流程
  * @param pWeldData 存储焊接数据的结构体(类)
  * @param nGroupNo 焊接组号
  * @return 对应组是否焊接成功
 */
 bool CAssemblyWeld::WeldSchedule(WeldAfterMeasure* pWeldData, int nGroupNo)
 {
	 //******************************初始化跟踪对象
	 CDiaphragmWeld tTraceWeldObj(pWeldData->m_ptUnit, E_EMPTY_ERROR);
	 tTraceWeldObj.SetHardware(pWeldData->m_pScanInit);
	 tTraceWeldObj.SetWeldParam(pWeldData->m_pIsArcOn, pWeldData->m_pIsNaturalPop, pWeldData->m_pIsTeachPop, pWeldData->m_bNeedWrap);
	 tTraceWeldObj.m_pColorImg = pWeldData->m_pColorImg;
	 tTraceWeldObj.m_bIsLocalDebug = pWeldData->m_bIsLocalDebug;
	 //初始化工艺参数
	 tTraceWeldObj.InitWeldStartVal(pWeldData->m_pRobotDriver);
	 pWeldData->m_pScanInit->m_pTraceModel->m_eStartWrapAngleType;
	 int nSingleBoard = pWeldData->m_vvtWeldLineInfoGroup[nGroupNo][0].tAtrribute.nStartWrapType;// 4为不搜起终的一次包角		
	 // 跳枪补丁
	 if (pWeldData->m_pScanInit->m_pTraceModel->m_vtRealEndpointCoor.size() > 0)
	 {
		 pWeldData->m_pScanInit->m_pTraceModel->m_eStartWrapAngleType = E_WRAPANGLE_JUMP;
	 }
	 
	 //													copy来的代码↓
	 //tTraceWeldObj.m_pRobotDriver->m_tRobotLimitation = m_vpLaserLineScreen[tTraceWeldObj.m_pRobotDriver->m_nRobotNo]->m_tRobotLimitation;
	 tTraceWeldObj.m_dSafePosRunSpeed = 3000;				//安全位置移动速度，快速
	 tTraceWeldObj.m_dFastApproachToWorkSpeed = 2000;		//快速靠近工件， 中速
	 tTraceWeldObj.m_dSafeApproachToWorkSpeed = 1000;		//安全移动到工件速度，慢速
	 //******************************初始化跟踪对象

	 // 清除用于生成完整焊接Job 的缓冲区
	 pWeldData->CleaeJobBuffer(); 

	 //jwq 临时，判断是否是闭合圆弧
	 double dDis = TwoPointDis(pWeldData->m_vvtWeldSeamGroupAdjust[nGroupNo][0].StartPoint.x,
		 pWeldData->m_vvtWeldSeamGroupAdjust[nGroupNo][0].StartPoint.y,
		 pWeldData->m_vvtWeldSeamGroupAdjust[nGroupNo][0].StartPoint.z,
		 pWeldData->m_vvtWeldSeamGroupAdjust[nGroupNo][0].EndPoint.x,
		 pWeldData->m_vvtWeldSeamGroupAdjust[nGroupNo][0].EndPoint.y,
		 pWeldData->m_vvtWeldSeamGroupAdjust[nGroupNo][0].EndPoint.z);

	 // jwq黄埔测试，临时使用
	 if (false && pWeldData->m_bWorkpieceShape && dDis > 1.0)
	 {
		 if (1 == pWeldData->m_vvtWeldSeamGroupAdjust[nGroupNo].size())
		 {
			 for (size_t i = 0; i < 2; i++)
			 {
				 pWeldData->m_vvtWeldSeamGroupAdjust[nGroupNo].push_back(pWeldData->m_vvtWeldSeamGroupAdjust[nGroupNo].back());
				 pWeldData->m_vvtWeldLineInfoGroup[nGroupNo].push_back(pWeldData->m_vvtWeldLineInfoGroup[nGroupNo].back());
			 }
			 
		 }
	 }
	 //对应组焊缝信息	（下面流程会将该组的立焊（如果存在）提取出来先焊接）
	 vector<LineOrCircularArcWeldingLine>& vtWeldSeam = pWeldData->m_vvtWeldSeamGroupAdjust[nGroupNo];
	 //vtWeldSeam中存放立焊的编号的集合
	 vector<int> vnStandWeldNoIdx(0);
	 //vtWeldSeam中存放平焊的编号的集合
	 vector<int> vnFlatWeldNoIdx(0);
	 //焊缝编号集合
	 vector<int> vnWeldOrder(0);
	 //记录焊接的平焊序号 1,2,3,4....↑
	 int nFlatWeldNo = 0;
	 //记录焊接的立焊序号 1,2,3,4....↑
	 int nStandWeldNo = 0;
	 
	 //********************************************************* 修改焊接流程为先焊接nGroupNo组焊缝的立焊
	 for (int i = 0; i < vtWeldSeam.size(); i++)
	 {
		 //取焊缝类型
		 E_WELD_SEAM_TYPE eWeldSeamType = pWeldData->GetWeldSeamType(vtWeldSeam[i]);
		 /*
			立焊在vtWeldSeam中的编号放入vnStandWeldNoIdx，
			平焊在vtWeldSeam中的编号放入vnFlatWeldNoIdx
		 */
		 if (E_FLAT_SEAM == eWeldSeamType)
		 {
			 vnFlatWeldNoIdx.push_back(i);
		 }
		 else if (E_STAND_SEAM == eWeldSeamType)
		 {
			 vnStandWeldNoIdx.push_back(i);
		 }
	 }
	 //将所有立焊编号放入vnWeldOrder中
	 vnWeldOrder.insert(vnWeldOrder.end(), vnStandWeldNoIdx.begin(), vnStandWeldNoIdx.end());
	 //将所有平焊编号放入vnWeldOrder中（在容器的末尾插入意味着平焊编号存储在容器中立焊编号之后）
	 vnWeldOrder.insert(vnWeldOrder.end(), vnFlatWeldNoIdx.begin(), vnFlatWeldNoIdx.end());
	 //********************************************************* 修改焊接流程为先焊接nGroupNo组焊缝的立焊

	 //遍历该组的所有焊缝（一组焊缝不会超过8个焊道）
	 for (int i = pWeldData->m_nPauseWeldNo; i < 8; i++) {
		 if (i >= vnWeldOrder.size()) {
			 break;
		 }
		 E_WELD_SEAM_TYPE eWeldSeamType;					//焊缝类型
		 vector<T_ROBOT_COORS> vtRealWeldCoord;				//焊接轨迹坐标
		 vector<T_ROBOT_COORS> vtAdjustRealWeldCoord;		//焊接轨迹坐标（添加过补偿量后）
		 vector<int> vnPtnType;								//焊接轨迹点类型
		 double dExAxlePos;									//焊接时外部轴位置（外部轴不动的情况下的位置） 
		 int nWeldNo;										//对应焊缝在vtWeldSeam中的编号
		 int nWeldAngleSize;								//焊脚大小
		 double dDirAngle;									//方向角（取焊接轨迹中间段坐标点的Rz计算得出 360 - RZ）
		 int n1;											//辅助计算方向角的变量（商）
		 int n2;											//辅助计算方向角的变量（余数）
		 int nDirAngle;										//方向角（整数）
		 CString sWeldName;									//焊缝焊接提示字符串
		 vector<T_WELD_PARA> vtWeldPara;					//工艺参数

		 // 演示只取一组内第一条焊缝，此次分组包角焊缝分为一组，最多两条焊缝，其余焊缝单独作为一组
		 if (i > 0 && pWeldData->DetermineWarpMode(nGroupNo) > 0)
		 {
			 break;
		 }
		 // ----------------------------

		 // 加载焊接轨迹
		 if (false == pWeldData->LoadRealWeldTrack(nGroupNo, i, eWeldSeamType, dExAxlePos, vtRealWeldCoord, vnPtnType)) {
			 break;
		 }
		 pWeldData->m_dTeachExAxlePos = dExAxlePos;
		 nWeldNo = vnWeldOrder[i];
		 nWeldAngleSize = pWeldData->m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nWeldAngleSize;
		 dDirAngle = pWeldData->RzToDirAngle(vtRealWeldCoord[vtRealWeldCoord.size() / 2].dRZ);
		 n1 = dDirAngle / 45.0;
		 n2 = (fmod(dDirAngle, 45.0) / 45.0 > 0.5) ? 45 : 0;
		 nDirAngle = (n1 * 45 + n2) % 360;
		 eWeldSeamType == E_FLAT_SEAM ? nFlatWeldNo++ : nStandWeldNo++;
		 sWeldName.Format("%s%s%d",
			 pWeldData->m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.bWeldMode ? "跟踪" : "普通",
			 eWeldSeamType == E_FLAT_SEAM ? "平焊" : "立焊",
			 eWeldSeamType == E_FLAT_SEAM ? nFlatWeldNo : nStandWeldNo);

		 //已修改
		 CString str1 = pWeldData->m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.bWeldMode ? "跟踪" : "普通";
		 CString str2 = eWeldSeamType == E_FLAT_SEAM ? "平焊" : "立焊";
		 str1 = XUI::Languge::GetInstance().translate(str1.GetBuffer());
		 str2 = XUI::Languge::GetInstance().translate(str2.GetBuffer());
		 sWeldName = XUI::Languge::GetInstance().translate("{0}{1}{2}", str1.GetBuffer(), str2.GetBuffer(), eWeldSeamType == E_FLAT_SEAM ? nFlatWeldNo : nStandWeldNo);
		
		 WriteLog("【交互参数】组号：%d,焊缝号：%d %s%s 焊脚:%d 起点过焊孔:%.3lf 终点过焊孔:%.3lf", nGroupNo, nWeldNo,
			 pWeldData->m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.bWeldMode ? "跟踪" : "普通",
			 E_FLAT_SEAM == eWeldSeamType ? "平焊" : "立焊",
			 pWeldData->m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.nWeldAngleSize,
			 pWeldData->m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.dStartHoleSize,
			 pWeldData->m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.dEndHoleSize);

		 //匹配对应焊脚工艺
		 if (false == pWeldData->GetWeldParam(eWeldSeamType, nWeldAngleSize, vtWeldPara))
		 {
			 XiMessageBox("加载焊接工艺参数失败！");
			 return false;
		 }
		 //判断该焊缝是否为需要跟踪焊接
		 if (pWeldData->m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.bWeldMode)
		 {
			 //*********跟踪焊接（暂不使用vtAdjustRealWeldCoord，而是使用vtRealWeldCoord 作为焊接轨迹）*********//
			 //*********跟踪焊接（默认单道焊接）*********//
			 tTraceWeldObj.m_vtTrackTheoryTrack = pWeldData->m_vtTrackTheoryTrack;
			 vector<T_ANGLE_PULSE> vtRealWeldPulse;
			 //vector<int> vnDataPointType(vtRealWeldCoord.size(),E_WELD_TRACK);
			 vector<int> vnDataPointType(vnPtnType);
			 //相机模式：软触发
			 E_DHGIGE_ACQUISITION_MODE eCameraMode = E_ACQUISITION_MODE_SOURCE_SOFTWARE;
			 //相机模式：回调关闭
			 E_DHGIGE_CALL_BACK eCallBack = E_CALL_BACK_MODE_OFF;
			 //******* 添加收下枪坐标
			 //合成世界坐标 （以下#ifdef SINGLE_ROBOT写法 源自WeldAfterMeasure->DoWelding中使用外部轴的方式）
#ifdef SINGLE_ROBOT
			 double dEx = vtRealWeldCoord[0].dY;
#else
			 double dEx = vtRealWeldCoord[0].dX;
#endif
			 for (int nIdx = 0; nIdx < vtRealWeldCoord.size(); nIdx++) {
#ifdef SINGLE_ROBOT
				 vtRealWeldCoord[nIdx].dY += dExAxlePos;
				 vtRealWeldCoord[nIdx].dBY = 0;

#else
				 vtRealWeldCoord[nIdx].dX += dExAxlePos;
				 vtRealWeldCoord[nIdx].dBX = 0;
#endif
			 }
			 // 焊接长度
			 tTraceWeldObj.m_pTraceModel->m_dWeldLen = pWeldData->m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.dThoeryLength;
			 // 结尾是否干涉,暂定45mm变化45度，步长3mm每步
			 if (pWeldData->m_vvtWeldLineInfoGroup[nGroupNo][nWeldNo].tAtrribute.bEndFixScan)
			 {
				 tTraceWeldObj.m_pTraceModel->m_nCloseTrackingPos = 15;
			 }
			 //  最高点赋值
			 tTraceWeldObj.m_dWorkPieceHighTop = pWeldData->CalcBoardMaxHeight(nGroupNo);
			 // 姿态分配，拆分机器人和外轴坐标
			 vtRealWeldCoord = tTraceWeldObj.GetRealWeldData(tTraceWeldObj.m_pRobotDriver, vtRealWeldCoord);
			 if (nSingleBoard != 4)
			 {
				 // 添加收下枪安全位置
				 tTraceWeldObj.AddSafeDownGunPos(vtRealWeldCoord, vnDataPointType, pWeldData->m_dGunDownBackSafeDis, tTraceWeldObj.m_dWorkPieceHighTop);
				 // 计算下枪关节坐标及可以连续运动的焊接轨迹关节坐标轨迹
#ifdef SINGLE_ROBOT
				 double dRealRobotExPos = vtRealWeldCoord[1].dY, dRealWeldExPos;
#else
				 double dRealRobotExPos = vtRealWeldCoord[1].dX, dRealWeldExPos;
#endif
				 double dExAxleChangeDisThreshold = 1000;
				 double dMinExAxleChangeDis = -dExAxleChangeDisThreshold;
				 double dMaxExAxleChangeDis = dExAxleChangeDisThreshold;

				 if (!pWeldData->CalcContinuePulseForWeld(vtRealWeldCoord, vtRealWeldPulse, TRUE))
				 {
					 bool bCalcRst = false;
					 bool bAddorSubtract = false;//true加false减
					 int nIndex = 1;
					 //double dExAxleChangeDis = dMinExAxleChangeDis;
					 double dExAxleChangeDis = 0;
					 while (dMinExAxleChangeDis <= dExAxleChangeDis && dExAxleChangeDis <= dMaxExAxleChangeDis) {
						 dRealWeldExPos = dRealRobotExPos;
						 vector<T_ROBOT_COORS> vtTempWeldCoord(vtRealWeldCoord); // 焊接轨迹 + 下枪收枪坐标
						 dRealWeldExPos += dExAxleChangeDis;

						 double dMaxExAxlePos = (double)pWeldData->m_ptUnit->GetRobotCtrl()->m_tExternalAxle[pWeldData->m_ptUnit->m_nTrackAxisNo - 1].lMaxPulseNum * pWeldData->m_ptUnit->GetRobotCtrl()->m_tExternalAxle[pWeldData->m_ptUnit->m_nTrackAxisNo - 1].dPulse;
						 double dMinExAxlePos = (double)pWeldData->m_ptUnit->GetRobotCtrl()->m_tExternalAxle[pWeldData->m_ptUnit->m_nTrackAxisNo - 1].lMinPulseNum * pWeldData->m_ptUnit->GetRobotCtrl()->m_tExternalAxle[pWeldData->m_ptUnit->m_nTrackAxisNo - 1].dPulse;

						 for (int nPtnNo = 0; nPtnNo < vtTempWeldCoord.size(); nPtnNo++)
						 {
#ifdef SINGLE_ROBOT
							 vtTempWeldCoord[nPtnNo].dY = dRealWeldExPos;
							 vtTempWeldCoord[nPtnNo].dBY -= dExAxleChangeDis;

							 if (vtTempWeldCoord[nPtnNo].dBY > dMaxExAxlePos || vtTempWeldCoord[nPtnNo].dBY < dMinExAxlePos)
							 {
								 WriteLog("外部轴%d 目标位置%.3lf 超出设定极限%.3lf - %.3lf!", pWeldData->m_ptUnit->m_nTrackAxisNo, vtTempWeldCoord[nPtnNo].dBY, dMinExAxlePos, dMaxExAxlePos);
								 continue;
							 }
#else
							 vtTempWeldCoord[nPtnNo].dX = dRealWeldExPos;
							 vtTempWeldCoord[nPtnNo].dBX -= dExAxleChangeDis;

							 if (vtTempWeldCoord[nPtnNo].dBX > dMaxExAxlePos || vtTempWeldCoord[nPtnNo].dBX < dMinExAxlePos)
							 {
								 WriteLog("外部轴%d 目标位置%.3lf 超出设定极限%.3lf - %.3lf!", pWeldData->m_ptUnit->m_nTrackAxisNo, vtTempWeldCoord[nPtnNo].dBY, dMinExAxlePos, dMaxExAxlePos);
								 continue;
							 }
#endif // SINGLE_ROBOT
						 }
						 bCalcRst = pWeldData->CalcContinuePulseForWeld(vtTempWeldCoord, vtRealWeldPulse, TRUE);
						 if (true == bCalcRst)
						 {
							 vtRealWeldCoord.clear();
							 vtRealWeldCoord.insert(vtRealWeldCoord.begin(), vtTempWeldCoord.begin(), vtTempWeldCoord.end());
							 //tBackCoord = vtTempWeldCoord[vtTempWeldCoord.size() - 1];
							 break;
						 }
						 dExAxleChangeDis = false == bAddorSubtract ? -50 * nIndex : 50 * nIndex;
						 nIndex = true == bAddorSubtract ? nIndex + 1 : nIndex;
						 bAddorSubtract = false == bAddorSubtract ? true : false;
					 }
					 if (false == bCalcRst)
					 {
						 XiMessageBox("多外部轴位置，焊接下枪过渡点计算失败！");
						 return false;
					 }
				 }
			 }
			 //记录结尾数据,包角时会记录多个结尾数据，不包角时改数据为空
			 tTraceWeldObj.m_pTraceModel->m_bCameraFindWeldEnd = false;
			 tTraceWeldObj.RecordEndPointCoors(tTraceWeldObj.m_pRobotDriver, vtRealWeldCoord, vnDataPointType);
			 tTraceWeldObj.m_ptUnit->SwitchDHCamera(tTraceWeldObj.m_ptUnit->m_nTrackCameraNo, true, true, eCameraMode, eCallBack); // 下枪时 开相机 激光
			 tTraceWeldObj.m_ptUnit->m_vpImageCapture[tTraceWeldObj.m_ptUnit->m_nTrackCameraNo]->StartAcquisition();
			 for (int nLayerNo = pWeldData->m_nPauseLayerNo; nLayerNo < vtWeldPara.size(); nLayerNo++)
			 {
				 T_WELD_PARA tWeldPara = vtWeldPara[nLayerNo];
				 //已修改
				 if (TRUE == m_bNaturalPop/* *pWeldData->m_pIsNaturalPop*/ &&
					 IDOK != XUI::MesBox::PopOkCancel("{0}焊脚{1}第{2}道焊接角度{3}？", sWeldName.GetBuffer(),
						 tWeldPara.nWeldAngleSize,
						 tWeldPara.nLayerNo + 1, nDirAngle))
					 /*
				 if (TRUE == *pWeldData->m_pIsNaturalPop &&
					 IDOK != XiMessageBox("%s 焊脚%d 第%d道 焊接角度%d ？",
						 sWeldName,
						 tWeldPara.nWeldAngleSize,
						 tWeldPara.nLayerNo + 1,
						 nDirAngle))*/
				 {
					 continue;
				 }
				 // 匹配焊接姿态
				 for (size_t i = 0; i < vtRealWeldCoord.size(); i++)
				 {
					 vtRealWeldCoord[i].dRY = tWeldPara.dWeldAngle * (double)tTraceWeldObj.m_nRobotInstallDir;
				 }

				 if (m_vpUnit[0]->m_bBreakPointContinue)
				 {
					 CString strCoutPath;
					 strCoutPath.Format("%s%s%s理论焊接数据.txt", OUTPUT_PATH, tTraceWeldObj.m_pRobotDriver->m_strRobotName, RECOGNITION_FOLDER);
					 //获取焊接轨迹
					 tTraceWeldObj.LoadDataRobotCoors(vtRealWeldCoord, vnDataPointType, strCoutPath.GetBuffer());
					 for (int i = 0; i < vtRealWeldCoord.size(); i++) // 理论焊接数据.txt中记录的坐标xyz是世界坐标 恢复成机器人坐标 此处xy外部轴不通用
					 {
						 double dExAxleOffset = 1 == tTraceWeldObj.m_pRobotDriver->m_nRobotInstallDir ? 0.0 : -500.0;
#ifdef SINGLE_ROBOT
						 vtRealWeldCoord[i].dY = dExAxleOffset;
#else
						 vtRealWeldCoord[i].dX = dExAxleOffset;
#endif // SINGLE_ROBOT

					 }
				 }
				 // 设置焊接工艺
				 tTraceWeldObj.SetWeldTlyParam(tWeldPara);		
				 if (nSingleBoard != 4)
				 {
					 //初始段数据处理
					 pWeldData->m_pScanInit->SetTrackProcessParam(nLayerNo);// 测试跟踪多层多道
					 if (!tTraceWeldObj.WeldProcessBefor(tTraceWeldObj.m_pRobotDriver, vtRealWeldCoord, vnDataPointType, vtRealWeldPulse, TRUE)) {
						 XiMessageBoxOk("初始段数据处理失败");
						 return false;
					 }
				 }
				 else if(nSingleBoard == 4)
				 {
					 TrackFilter_Init(tTraceWeldObj.m_ptUnit->m_nRobotSmooth, 3.0);
					 
					 tTraceWeldObj.m_pTraceModel->m_eStartWrapAngleType = E_WRAPANGLE_ONCE; //单板单筋一次包角
					 tTraceWeldObj.m_pRobotDriver->m_vtWeldLineInWorldPoints.clear();
					 tTraceWeldObj.m_pTraceModel->m_vtWeldLinePointType.clear();

					 for (int nTrackNo = 0; nTrackNo < vtRealWeldCoord.size(); nTrackNo++)
					 {
						 double dExAxle = vtRealWeldCoord[nTrackNo].dX - dEx;
						 vtRealWeldCoord[nTrackNo].dX = dEx;
						 vtRealWeldCoord[nTrackNo].dBX += dExAxle;

						 //vtRealWeldCoord[nTrackNo].dY = vtRealWeldCoord[nTrackNo].dY + vtRealWeldCoord[nTrackNo].dBY;
						 //double dExAxle = vtRealWeldCoord[nTrackNo].dY - dEx;
						 //vtRealWeldCoord[nTrackNo].dY = dEx;
						 //vtRealWeldCoord[nTrackNo].dBY = dExAxle;

						 tTraceWeldObj.m_pRobotDriver->m_vtWeldLineInWorldPoints.push_back(vtRealWeldCoord.at(nTrackNo));
						 tTraceWeldObj.m_pTraceModel->m_vtWeldLinePointType.push_back(vnDataPointType[nTrackNo]);
					 }
					 //锁斜率
					 CHECK_BOOL_RETURN(pWeldData->m_pScanInit->RealTimeTrackLockArcBefor(tTraceWeldObj.m_pRobotDriver, tTraceWeldObj.m_ptUnit->m_bBreakPointContinue));
				 }
				 //开始焊接
				 bool bDoWeldSuccess = tTraceWeldObj.DoweldRuning(tTraceWeldObj.m_pRobotDriver,
					 tTraceWeldObj.m_pRobotDriver->m_vtWeldLineInWorldPoints, tTraceWeldObj.m_pTraceModel->m_vtWeldLinePointType, true, eWeldSeamType);
				 tTraceWeldObj.SavePauseInfo(nGroupNo, i, nLayerNo); // 暂停继续使用
				 if (!bDoWeldSuccess)
				 {
					 return false;
				 }
			 }			 
			 tTraceWeldObj.m_ptUnit->SwitchDHCamera(tTraceWeldObj.m_ptUnit->m_nTrackCameraNo, false); // 关相机 激光
		 }
		 else {
			 //*********直接使用理论轨迹焊接*********//
			 // 多层多道焊接(单道参数即单道焊接)
			 for (int nLayerNo = pWeldData->m_nPauseLayerNo; nLayerNo < vtWeldPara.size(); nLayerNo++)
			 {
				 T_WELD_PARA tWeldPara = vtWeldPara[nLayerNo];
				 WriteLog("工艺参数%s：第%d道 偏移量%.3lf %.3lf",
					 tWeldPara.strWorkPeace, tWeldPara.nLayerNo, tWeldPara.CrosswiseOffset, tWeldPara.verticalOffset);
				 //已修改
				 if (TRUE == (*pWeldData->m_pIsNaturalPop) && IDOK != XUI::MesBox::PopOkCancel("{0}焊脚{1}第{2}道焊接角度{3}", sWeldName.GetBuffer(), tWeldPara.nWeldAngleSize, tWeldPara.nLayerNo + 1, nDirAngle)/*XiMessageBox("%s 焊脚%d 第%d道 焊接角度%d ？",
					 sWeldName, tWeldPara.nWeldAngleSize, tWeldPara.nLayerNo + 1, nDirAngle)*/)
					 /*
				 if (TRUE == (*pWeldData->m_pIsNaturalPop) && IDOK != XiMessageBox("%s 焊脚%d 第%d道 焊接角度%d ？",
					 sWeldName, tWeldPara.nWeldAngleSize, tWeldPara.nLayerNo + 1, nDirAngle))*/
				 {
					 continue;
				 }

				 // jwq 设置是否开启新跟踪
				 if (tTraceWeldObj.m_ptUnit->m_ScanTrackingWeldEnable && eWeldSeamType != E_STAND_SEAM)
				 {
					 pWeldData->m_ptUnit->m_cRealTimeTrack.setEnable(true);
				 }

				 // jwq 设置跟踪补偿
				 if (tTraceWeldObj.m_ptUnit->m_ScanTrackingWeldEnable && eWeldSeamType != E_STAND_SEAM)
				 {
					 double dFlatHorComp =/*-2.5*/ tTraceWeldObj.m_mdFlatHorComp[405.0];
					 double dFlatHeightComp =/*-0.7*/ tTraceWeldObj.m_mdFlatHeightComp[405.0];
					 double dGunToEyeCompenX = tTraceWeldObj.m_pTraceModel->m_tWeldParam.CrosswiseOffset + dFlatHorComp;	// 外加内减
					 double dGunToEyeCompenZ = tTraceWeldObj.m_pTraceModel->m_tWeldParam.verticalOffset + dFlatHeightComp;// 向上补加 向下补减
					 dGunToEyeCompenZ *= tTraceWeldObj.m_nRobotInstallDir;
					 pWeldData->m_ptUnit->m_cRealTimeTrack.setUnit(*(pWeldData->m_ptUnit));
					 pWeldData->m_ptUnit->m_cRealTimeTrack.setCompensation(dGunToEyeCompenX, dGunToEyeCompenZ);
				 }

				 //加载补偿量后的焊接轨迹输出至vtAdjustRealWeldCoord
				 pWeldData->TrackComp(nGroupNo, eWeldSeamType, vtRealWeldCoord, tWeldPara, vtAdjustRealWeldCoord);

				 pWeldData->SaveWeldTrack(nGroupNo, i, tWeldPara.nLayerNo, eWeldSeamType, dExAxlePos, vtAdjustRealWeldCoord, vnPtnType);
				 pWeldData->GetPauseWeldTrack(vtAdjustRealWeldCoord, vnPtnType, eWeldSeamType);

				 // jwq 开启相机
				 if (tTraceWeldObj.m_ptUnit->m_ScanTrackingWeldEnable && eWeldSeamType != E_STAND_SEAM)
				 {
					 if (pWeldData->m_ptUnit->m_cRealTimeTrack.isEnable())
					 {
						 E_DHGIGE_ACQUISITION_MODE eCameraMode = E_ACQUISITION_MODE_SOURCE_SOFTWARE;//相机模式：软触发
						 E_DHGIGE_CALL_BACK eCallBack = E_CALL_BACK_MODE_OFF;//相机模式：回调关闭
						 tTraceWeldObj.m_ptUnit->SwitchDHCamera(tTraceWeldObj.m_ptUnit->m_nTrackCameraNo, true, true, eCameraMode, eCallBack); // 下枪时 开相机 激光
						 tTraceWeldObj.m_ptUnit->m_vpImageCapture[tTraceWeldObj.m_ptUnit->m_nTrackCameraNo]->StartAcquisition();
					 }
				 }
				 bool bDoWeldSuccess = pWeldData->DoWelding(nGroupNo, nWeldNo, eWeldSeamType, vtAdjustRealWeldCoord, vnPtnType, tWeldPara);

				 // jwq 关闭相机
				 if (tTraceWeldObj.m_ptUnit->m_ScanTrackingWeldEnable && eWeldSeamType != E_STAND_SEAM)
				 {
					 if (pWeldData->m_ptUnit->m_cRealTimeTrack.isEnable())
					 {
						 tTraceWeldObj.m_ptUnit->SwitchDHCamera(tTraceWeldObj.m_ptUnit->m_nTrackCameraNo, false); // 关相机 激光
					 }
				 }
				 pWeldData->SavePauseInfo(nGroupNo, i, nLayerNo); // 暂停继续使用
				 if (false == bDoWeldSuccess)
				 {
					 return false;
				 }
			 }
		 }
	 }
	 pWeldData->GenerateJobForContinueWeld(nGroupNo, true);
	 pWeldData->CleaeJobBuffer();
	 return true;
 }
 
 bool CAssemblyWeld::WeldSchedule_G(WeldAfterMeasure* pWeldData, int nGroupNo, int nLayerNo)
 {
	 // 清除用于生成完整焊接Job 的缓冲区
	 pWeldData->CleaeJobBuffer();
	 int nFlatWeldNo = 0;
	 int nStandWeldNo = 0;
	 
	 int nWeldNo = nLayerNo;									//对应焊缝在vtWeldSeam中的编号nWeldNo = i;
	 E_WELD_SEAM_TYPE eWeldSeamType;					//焊缝类型
	 vector<T_ROBOT_COORS> vtRealWeldCoord;				//焊接轨迹坐标
	 vector<T_ROBOT_COORS> vtAdjustRealWeldCoord;		//焊接轨迹坐标（添加过补偿量后）
	 vector<int> vnPtnType;								//焊接轨迹点类型
	 double dExAxlePos;									//焊接时外部轴位置（外部轴不动的情况下的位置） 
	 int nWeldAngleSize;								//焊脚大小
	 double dDirAngle;									//方向角（取焊接轨迹中间段坐标点的Rz计算得出 360 - RZ）
	 int n1;											//辅助计算方向角的变量（商）
	 int n2;											//辅助计算方向角的变量（余数）
	 int nDirAngle;										//方向角（整数）
	 CString sWeldName;									//焊缝焊接提示字符串
	 vector<T_WELD_PARA> vtWeldPara;					//工艺参数

	 // 加载焊接轨迹
	 if (false == pWeldData->LoadRealWeldTrack(nGroupNo, nWeldNo, eWeldSeamType, dExAxlePos, vtRealWeldCoord, vnPtnType)) {
		 return false;
	 }
	 pWeldData->m_dTeachExAxlePos = dExAxlePos;
	 nWeldAngleSize = 0;
	 dDirAngle = pWeldData->RzToDirAngle(vtRealWeldCoord[vtRealWeldCoord.size() / 2].dRZ);
	 n1 = dDirAngle / 45.0;
	 n2 = (fmod(dDirAngle, 45.0) / 45.0 > 0.5) ? 45 : 0;
	 nDirAngle = (n1 * 45 + n2) % 360;
	 eWeldSeamType == E_PLAT_GROOVE ? nFlatWeldNo++ : nStandWeldNo++;
	 sWeldName.Format("%s%s%d", "普通",
		 eWeldSeamType == E_PLAT_GROOVE ? "平焊" : "立焊",
		 eWeldSeamType == E_PLAT_GROOVE ? nFlatWeldNo : nStandWeldNo);

	 WriteLog("【交互参数】组号：%d,焊缝号：%d %s%s 焊脚:%d",
		 nGroupNo, nWeldNo, "普通",
		 E_PLAT_GROOVE == eWeldSeamType ? "平焊坡口" : "立焊坡口",
		 0);

	 //匹配对应焊脚工艺
	 if (false == pWeldData->GetWeldParam(eWeldSeamType, nWeldAngleSize, vtWeldPara))
	 {
		 XiMessageBox("加载焊接工艺参数失败！");
		 return false;
	 }
	 T_WELD_PARA tWeldPara = vtWeldPara[nWeldNo];
	 vector<T_WAVE_PARA> vtTWavePara(0);
	 if (0 != GetGroovePara(vtRealWeldCoord.front(), vtRealWeldCoord.back(), vtTWavePara))
	 {
		 return false;
	 }
	 tWeldPara.tGrooveWaveParam = vtTWavePara[nWeldNo];
	 WriteLog("工艺参数%s：第%d道 偏移量%.3lf %.3lf",
		 tWeldPara.strWorkPeace, tWeldPara.nLayerNo, tWeldPara.CrosswiseOffset, tWeldPara.verticalOffset);
	 //已修改
	 if (TRUE == (*pWeldData->m_pIsNaturalPop) && IDOK != XUI::MesBox::PopOkCancel("{0}焊脚{1}第{2}道 焊接角度{3}", sWeldName.GetBuffer(), tWeldPara.nWeldAngleSize, tWeldPara.nLayerNo + 1, nDirAngle)/*XiMessageBox("%s 焊脚%d 第%d道 焊接角度%d ？",
		sWeldName, tWeldPara.nWeldAngleSize, tWeldPara.nLayerNo + 1, nDirAngle)*/)
		 /*
	 if (TRUE == (*pWeldData->m_pIsNaturalPop) && IDOK != XiMessageBox("%s 焊脚%d 第%d道 焊接角度%d ？",
		 sWeldName, tWeldPara.nWeldAngleSize, tWeldPara.nLayerNo + 1, nDirAngle))*/
	 {
		 return true;
	 }
	 //加载补偿量后的焊接轨迹输出至vtAdjustRealWeldCoord
	 pWeldData->TrackComp(nGroupNo, eWeldSeamType, vtRealWeldCoord, tWeldPara, vtAdjustRealWeldCoord);
	 
	 /*
	 if (pWeldData->m_pRobotDriver->m_nRobotNo == 0)
	 {
		 double dHeightComp = 5.0;
		 T_ROBOT_COORS p1 = vtAdjustRealWeldCoord.front();
		 T_ROBOT_COORS p2 = vtAdjustRealWeldCoord.back();

		 double dDis = TwoPointDis(
			 p1.dX + p1.dBX, p1.dY + p1.dBY, p1.dZ + p1.dBZ,
			 p2.dX + p2.dBX, p2.dY + p2.dBY, p2.dZ + p2.dBZ);
		 for (int i = 0; i < vtAdjustRealWeldCoord.size(); i++)
		 {
			 T_ROBOT_COORS p = vtAdjustRealWeldCoord[i];
			 double d = TwoPointDis(
				 p2.dX + p2.dBX, p2.dY + p2.dBY, p2.dZ + p2.dBZ,
				 p.dX + p.dBX, p.dY + p.dBY, p.dZ + p.dBZ);
			 vtAdjustRealWeldCoord[i].dZ += dHeightComp * d / dDis;
		 }
	 }

	 if (pWeldData->m_pRobotDriver->m_nRobotNo == 1)
	 {
		 double dHeightComp = 10.0;
		 T_ROBOT_COORS p1 = vtAdjustRealWeldCoord.front();
		 T_ROBOT_COORS p2 = vtAdjustRealWeldCoord.back();

		 double dDis = TwoPointDis(
			 p1.dX + p1.dBX, p1.dY + p1.dBY, p1.dZ + p1.dBZ,
			 p2.dX + p2.dBX, p2.dY + p2.dBY, p2.dZ + p2.dBZ);
		 for (int i = 0; i < vtAdjustRealWeldCoord.size(); i++)
		 {
			 T_ROBOT_COORS p = vtAdjustRealWeldCoord[i];
			 double d = TwoPointDis(
				 p2.dX + p2.dBX, p2.dY + p2.dBY, p2.dZ + p2.dBZ,
				 p.dX + p.dBX, p.dY + p.dBY, p.dZ + p.dBZ);
			 vtAdjustRealWeldCoord[i].dZ += dHeightComp * d / dDis;
		 }
	 }
	 */

	 pWeldData->SaveWeldTrack(nGroupNo, nWeldNo, tWeldPara.nLayerNo, eWeldSeamType, dExAxlePos, vtAdjustRealWeldCoord, vnPtnType);
	 pWeldData->GetPauseWeldTrack(vtAdjustRealWeldCoord, vnPtnType, eWeldSeamType);
	 vtAdjustRealWeldCoord.erase(vtAdjustRealWeldCoord.begin());
	 vnPtnType.erase(vnPtnType.begin());
	 bool bDoWeldSuccess = pWeldData->DoWelding(nGroupNo, nWeldNo, eWeldSeamType, vtAdjustRealWeldCoord, vnPtnType, tWeldPara);
	 pWeldData->SavePauseInfo(nGroupNo, nWeldNo, nLayerNo); // 暂停继续使用
	 if (false == bDoWeldSuccess)
	 {
		 return false;
	 }

	 pWeldData->GenerateJobForContinueWeld(nGroupNo, true);
	 pWeldData->CleaeJobBuffer();
	 return true;
 }

 bool CAssemblyWeld::WeldAfterMeasureMultiMachine_G()
 {
	 bool bFlatWeldEnabld = false; // 是否运行立焊
	 bool bStandWeldEnable = true; // 是否运行平焊
	 SetTimer(99, 200, NULL); 
	 if (g_bAutoGroupingMark)
	 {
		 // 加载自动分组数据
		 CString strFileAuto = OUTPUT_PATH + m_vpUnit[0]->m_tContralUnit.strUnitName + "\\" + RECOGNITION_FOLDER + "planned_welds.txt";
		 LoadGroupingResult(strFileAuto);
	 }
	 else
	 {
		 // 手动分组数据
		 LoadCloudProcessResultMultiMachine(m_vpUnit[0]->m_tContralUnit.strUnitName);
	 }

	 std::vector <std::vector<WeldLineInfo>>& vvtWeldSeamInfo = m_vvvtWeldSeamInfo[0];
	 std::vector < WeldLineInfo>& vtWeldSeamInfoRobotA = vvtWeldSeamInfo[0];
	 std::vector < WeldLineInfo>& vtWeldSeamInfoRobotB = vvtWeldSeamInfo[1];
	 int nMaxWeldSeamNum = vtWeldSeamInfoRobotA.size() > vtWeldSeamInfoRobotB.size() ? vtWeldSeamInfoRobotA.size() : vtWeldSeamInfoRobotB.size();
	 int nMaxLayerNo = 8;
	 int nWeldSeamIdx = -1;
	 int nLayerNo = -1;

	 vector<CWinThread* > vpcThread;
	 CString sHintInfo;

	 for (nLayerNo = 0; nLayerNo < nMaxLayerNo; nLayerNo++) // 所有焊缝每层每道焊接
	 {
		 CString cStrTitle;
		 cStrTitle.Format(" 焊接道数(1-%d) 关闭进入下一层 无效退出", nMaxLayerNo);
		 std::vector<CString> vsInputName(1, "焊接道数:");
		 std::vector<double> vnInputData(1, nLayerNo + 1);
		 ParamInput cParamDlg(cStrTitle, vsInputName, &vnInputData);
		 if (TRUE == m_bNaturalPop)
		 {
			 int nRst = cParamDlg.DoModal();
			 if ((1 == nRst) && ((int)vnInputData[0] - 1 >= 0) && ((int)vnInputData[0] - 1 < nMaxLayerNo))
			 {
				 nLayerNo = (int)vnInputData[0] - 1;
			 }
			 else if (((int)vnInputData[0] < 1) || (vnInputData[0] > nMaxLayerNo))
			 {
				 break;
			 }
			 else
			 {
				 sHintInfo.Format("跳过第%d层焊缝焊接", nLayerNo + 1);
				 continue;
			 }
		 }

		 // 遍历焊缝索引，从小到达依次焊接，相同索引可同时焊接
		 for (nWeldSeamIdx = 0; nWeldSeamIdx <= nMaxWeldSeamNum; nWeldSeamIdx++)
		 {
			 vpcThread.clear();
			 for (int i = 0; i < vtWeldSeamInfoRobotA.size(); i++)
			 {
				 if (nWeldSeamIdx == vtWeldSeamInfoRobotA[i].tAtrribute.nWeldSeamIdx) // A存在焊缝索引nWeldSeamIdx的焊缝
				 {
					 if (((nLayerNo >= 4) && (true == JudgeGrooveStandWeld(vtWeldSeamInfoRobotA[i]))) ||
						 ((false == bFlatWeldEnabld) && (false == JudgeGrooveStandWeld(vtWeldSeamInfoRobotA[i]))) ||
						 ((false == bStandWeldEnable) && (true == JudgeGrooveStandWeld(vtWeldSeamInfoRobotA[i])))) // 立焊四层以后自动跳过
					 {
						 break;
					 }
					 //已修改
					 if ((TRUE == m_bNaturalPop) && (1 != XUI::MesBox::PopOkCancel("RobotA 第{0}道 焊缝{1} 开始测量焊接？取消跳过！", nLayerNo + 1, nWeldSeamIdx + 1)/*XiMessageBox("RobotA 第%d道 焊缝%d 开始测量焊接？取消跳过！", nLayerNo + 1, nWeldSeamIdx + 1)*/))
					 //if ((TRUE == m_bNaturalPop) && (1 != XiMessageBox("RobotA 第%d道 焊缝%d 开始测量焊接？取消跳过！", nLayerNo + 1, nWeldSeamIdx + 1)))
					 {
						 break;
					 }					 
					 // 开启测量焊接线程A
					 WriteLog("坡口焊接：机器人A 第%d道 焊缝%d 线程开启", nLayerNo, nWeldSeamIdx);
					 T_ROBOT_THREAD* tRobot = new T_ROBOT_THREAD();
					 tRobot->nGroupNo = i;
					 tRobot->nLayerNo = nLayerNo;
					 tRobot->cIncisePlanePart = this;
					 tRobot->bRobotThreadStatus = false;
					 tRobot->pRobotCtrl = m_vpUnit[0]->GetRobotCtrl();
					 CWinThread *pThread = AfxBeginThread(ThreadGrooveWeld_G, tRobot);
					 vpcThread.push_back(pThread);
					 break;
				 }
			 }
			 for (int i = 0; i < vtWeldSeamInfoRobotB.size(); i++)
			 {
				 if (nWeldSeamIdx == vtWeldSeamInfoRobotB[i].tAtrribute.nWeldSeamIdx) // B存在焊缝索引nWeldSeamIdx的焊缝
				 {
					 if (((nLayerNo >= 4) && (true == JudgeGrooveStandWeld(vtWeldSeamInfoRobotB[i]))) ||
						 ((false == bFlatWeldEnabld) && (false == JudgeGrooveStandWeld(vtWeldSeamInfoRobotB[i]))) ||
						 ((false == bStandWeldEnable) && (true == JudgeGrooveStandWeld(vtWeldSeamInfoRobotB[i])))) // 立焊四层以后自动跳过
					 {
						 break;
					 }
					 if ((TRUE == m_bNaturalPop) && (1 != XUI::MesBox::PopOkCancel("RobotB 第{0}道 焊缝{1} 开始测量焊接？取消跳过！", nLayerNo + 1, nWeldSeamIdx + 1)))
					 {
						 break;
					 }
					 // 开启测量焊接线程B
					 WriteLog("坡口焊接：机器人B 第%d道 焊缝%d 线程开启", nLayerNo, nWeldSeamIdx);
					 T_ROBOT_THREAD* tRobot = new T_ROBOT_THREAD();
					 tRobot->nGroupNo = i;
					 tRobot->nLayerNo = nLayerNo;
					 tRobot->cIncisePlanePart = this;
					 tRobot->bRobotThreadStatus = false;
					 tRobot->pRobotCtrl = m_vpUnit[1]->GetRobotCtrl();
					 CWinThread* pThread = AfxBeginThread(ThreadGrooveWeld_G, tRobot);
					 vpcThread.push_back(pThread);
					 break;
				 }
			 }
			 // 等到线程结束
			 for (int nNo = 0; nNo < vpcThread.size(); nNo++)
			 {
				 WaitForSingleObject(vpcThread[nNo]->m_hThread, INFINITE);
			 }
		 }
	 }
	 KillTimer(99);
	 XiMessageBox("焊接完成");
	 return true;
 }

 UINT CAssemblyWeld::ThreadGrooveWeld_G(void* pParam)
 {
	 T_ROBOT_THREAD* pObj = (T_ROBOT_THREAD*)pParam;
	 //DELETE_POINTER(pParam);
	 long long lTimeS = XI_clock();
	 int nCurGroupNo = pObj->nGroupNo;
	 int nLayerNo = pObj->nLayerNo;
	 pObj->bRobotThreadStatus = true;
	 bool bRst = pObj->cIncisePlanePart->FuncGrooveWeld_G(pObj->pRobotCtrl->m_nRobotNo, nCurGroupNo, nLayerNo);
	 pObj->bRobotThreadStatus = false;
	 return 0;
 }


 bool CAssemblyWeld::FuncGrooveWeld_G(int nRobotNo, int& nGroupNo, int nLayerNo)
 {
	 CRobotDriverAdaptor* pRobotDriver = m_vpRobotDriver[nRobotNo];
	 CUnit* pUnit = m_vpUnit[nRobotNo];
	 // 创建并初始化加劲板焊接实例
	 WAM::WeldAfterMeasure* pWeldAfterMeasure = NULL;// = m_pWeldAfterMeasure; // 为了不修以下变量名
	 CHECK_BOOL_RETURN(CreateObject(m_tChoseWorkPieceType, m_vpUnit[nRobotNo], &pWeldAfterMeasure));
	 pWeldAfterMeasure->SetRecoParam();
	 pWeldAfterMeasure->SetHardware(m_vpScanInit[nRobotNo]);
	 pWeldAfterMeasure->SetWeldParam(&m_bStartArcOrNot, &m_bNaturalPop, &m_bTeachPop, &m_bNeedWrap);
	 pWeldAfterMeasure->m_pColorImg = &m_vpShowLaserImgBuff[nRobotNo];
	 pWeldAfterMeasure->m_bIsLocalDebug = GetLocalDebugMark() == TRUE;
	 pWeldAfterMeasure->LoadPauseInfo();

	 pWeldAfterMeasure->m_vtWeldSeamData = m_vvvtWeldSeamData.at(0).at(nRobotNo);
	 pWeldAfterMeasure->m_vtWeldSeamInfo = m_vvvtWeldSeamInfo.at(0).at(nRobotNo);
	 pWeldAfterMeasure->m_nGroupNo = nGroupNo;
	 pWeldAfterMeasure->m_nLayerNo = nLayerNo;

	 if (true == pUnit->m_bBreakPointContinue)
	 {
		 pWeldAfterMeasure->m_nGroupNo = pWeldAfterMeasure->m_nPauseGroupNo;
		 pWeldAfterMeasure->m_nLayerNo = pWeldAfterMeasure->m_nPauseLayerNo;
	 }

	 // 焊缝排序分组
	 pUnit->m_sHintInfo.Format("焊缝信息排序分组");
	 int nWeldGropuNum = 0;
	 CHECK_BOOL_RETURN(pWeldAfterMeasure->WeldSeamGrouping(nWeldGropuNum));

	 // 测量和焊接焊缝
	 pUnit->m_sHintInfo.Format("计算第%d组焊缝测量轨迹……", nGroupNo + 1);
	 vector<T_ROBOT_COORS> vtMeasureCoord;
	 vector<T_ANGLE_PULSE> vtMeasurePulse;
	 vector<int> vnMeasureType;
	 double dExAxlePos = 0.0;

	 // 测量数据测量运动
	 pUnit->m_sHintInfo.Format("第%d组焊缝测量轨迹计算中 进度：%d组/共%d组", nGroupNo + 1, nGroupNo + 1, nWeldGropuNum);
	 // 机器人基座中心到大车中心距离
	 double dSafeHeight = 0.0;
	 CHECK_BOOL_RETURN(pWeldAfterMeasure->CalcMeasureTrack(nGroupNo, vtMeasureCoord, vtMeasurePulse, vnMeasureType, dExAxlePos, dSafeHeight));

	 // 外部轴运动 执行测量运动 外部轴 相机 图像处理对象
	 pUnit->m_sHintInfo.Format("第%d组焊缝测量中 进度：%d组/共%d组", nGroupNo + 1, nGroupNo + 1, nWeldGropuNum);

	 int nMaxMeasureLayerNo = 1; // 调整测量开关
	 if (nLayerNo < nMaxMeasureLayerNo) // 前nMaxMeasureLayerNo测量调整
	 {
		 if (!GetLocalDebugMark())
		 {
			 //已修改
			 if ((!pWeldAfterMeasure->m_ptUnit->m_bBreakPointContinue) && (!m_bNaturalPop ||
				 IDOK == XUI::MesBox::PopOkCancel("开始示教!!!可跳过但必须确保有测量数据!!!")/*XiMessageBox("开始示教？\n!!!可跳过但必须确保有测量数据!!!")*/))
			 /*
			 if ((!pWeldAfterMeasure->m_ptUnit->m_bBreakPointContinue) && (!m_bNaturalPop ||
				 IDOK == XiMessageBox("开始示教？\n!!!可跳过但必须确保有测量数据!!!")))*/
			 {
				 long long lTimeS = XI_clock();
				 //打开镜片防护
				 pUnit->SwitchIO("LensProtection", true);

				 if (!pWeldAfterMeasure->DoTeach(nGroupNo, vtMeasurePulse, vnMeasureType, dExAxlePos, nLayerNo))
				 {
					 SaveGrooveData(nLayerNo, nGroupNo, pWeldAfterMeasure->m_pRobotDriver->m_strRobotName, false);
					 if (E_STAND_SEAM == pWeldAfterMeasure->GetWeldSeamType(pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0]))
					 {
						 //已修改
						 XUI::MesBox::PopInfo("{0}立峰“测量”失败，继续其他焊缝焊接", pRobotDriver->m_strRobotName.GetBuffer());
						 //XiMessageBox("%s 立峰“测量”失败，继续其他焊缝焊接", pRobotDriver->m_strRobotName);
					 }
					 return false;
				 }
				 SaveGrooveData(nLayerNo, nGroupNo, pWeldAfterMeasure->m_pRobotDriver->m_strRobotName, true);
			 }
			 else
			 {
				 double y;
				 pWeldAfterMeasure->SetTeachData(vtMeasurePulse, vnMeasureType, dExAxlePos, y); // 调试时需要设置 实际运行时DoTeach内部设置
				 CHECK_BOOL_RETURN(pWeldAfterMeasure->LoadTeachResult(nGroupNo, nLayerNo));
			 }

		 }
		 else
		 {
			 double y;
			 //pWeldAfterMeasure->DoTeach(nGroupNo, vtMeasurePulse, vnMeasureType, dExAxlePos);
			 CHECK_BOOL_RETURN(pWeldAfterMeasure->GeneralTeachResult(nGroupNo, vtMeasureCoord, vtMeasurePulse, vnMeasureType, dExAxlePos));
			 pWeldAfterMeasure->SetTeachData(vtMeasurePulse, vnMeasureType, dExAxlePos, y); // 调试时需要设置 实际运行时DoTeach内部设置
			 CHECK_BOOL_RETURN(pWeldAfterMeasure->LoadTeachResult(nGroupNo, nLayerNo));
		 }

		 // 计算焊道
		 pUnit->m_sHintInfo.Format("第%d组焊缝焊接轨迹计算中 进度：%d组/共%d组", nGroupNo + 1, nGroupNo + 1, nWeldGropuNum);
		 if (!pWeldAfterMeasure->CalcWeldTrack(nGroupNo))
		 {
			 if (E_STAND_SEAM == pWeldAfterMeasure->GetWeldSeamType(pWeldAfterMeasure->m_vvtWeldSeamGroup[nGroupNo][0]))
			 {
				 //已修改
				 XUI::MesBox::PopInfo("{0}立峰“计算”失败，继续其他焊缝焊接", pRobotDriver->m_strRobotName.GetBuffer());
				// XiMessageBox("%s 立峰“计算”失败，继续其他焊缝焊接", pRobotDriver->m_strRobotName);
			 }
			 return false;
		 }
	 }

	 // 执行焊接
	 pUnit->m_sHintInfo.Format("第%d组焊缝焊接中…… 进度：%d组/共%d组", nGroupNo + 1, nGroupNo + 1, nWeldGropuNum);
	 if (!WeldSchedule_G(pWeldAfterMeasure, nGroupNo, nLayerNo))
	 {
		 return false;
	 }

	 // 运动外部轴到安全位置 接头测试临时去除
	 pUnit->MoveExAxleToSafe();

	 // 抬枪
	 if (!GetLocalDebugMark())
	 {
		 T_ANGLE_PULSE tBackPulse = pRobotDriver->m_tHomePulse;
		 T_ROBOT_MOVE_INFO tRobotMoveInfo;
		 vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfo(0);
		 T_ANGLE_PULSE tCurPulse = pRobotDriver->GetCurrentPulse();
		 tBackPulse.nSPulse = tCurPulse.nSPulse;
		 tRobotMoveInfo = pRobotDriver->PVarToRobotMoveInfo(1, tBackPulse, pRobotDriver->m_tBackHomeSpeed, MOVJ);
		 vtRobotMoveInfo.push_back(tRobotMoveInfo);
		 pRobotDriver->SetMoveValue(vtRobotMoveInfo);
		 pRobotDriver->CallJob("CONTIMOVANY");
		 pUnit->RobotCheckDone();
	 }

	 vector<T_ROBOT_MOVE_INFO> vtRobotMoveInfoNew(0);
	 vtRobotMoveInfoNew.push_back(pRobotDriver->PVarToRobotMoveInfo(1, pRobotDriver->m_tHomePulse, pRobotDriver->m_tBackHomeSpeed, MOVJ));
	 pRobotDriver->SetMoveValue(vtRobotMoveInfoNew);
	 pRobotDriver->CallJob("CONTIMOVANY");
	 pUnit->RobotCheckDone();
	 //if (0== nLayerNo%2)
	 {
		 CleanGunH(pRobotDriver);
	 }


	 pUnit->m_sHintInfo.Format("焊接作业结束");
	 return true;
 }

 UINT CAssemblyWeld::ThreadTest(void* pParam)
 {
	 CAssemblyWeld* pObj = (CAssemblyWeld*)pParam;
	 pObj->TestGrooveImageProcess(0, 0);
	 return 0;
 }

 void CAssemblyWeld::testCompenWithPara(CUnit* ptUnit, int CameraNo)
 {
#if 1
	 CRobotDriverAdaptor* pRobotCtrl = ptUnit->GetRobotCtrl();

	 int nRobotNo = pRobotCtrl->m_nRobotNo;
	 T_ABS_POS_IN_BASE tPointAbsCoordInBase;
	 std::vector<CvPoint> vtImagePoint;
	 vtImagePoint.clear();
	 CvPoint tImagePoint;
	 int nImageNo;
	 CString strName;
	 strName.Format(".\\HandEyeCompen-%s\\", pRobotCtrl->m_strRobotName);
	 CheckFolder(strName);

	 CString strImagePoint;
	 strImagePoint.Format("%s%d_Image2DPoint.txt", strName, CameraNo);
	 CString strMeasurePluase;
	 strMeasurePluase.Format("%s%d_TeachPulse.txt", strName, CameraNo);
	 CString strMeasurePos;
	 strMeasurePos.Format("%s%d_TeachPos.txt", strName, CameraNo);
	 CString strRealPos;
	 strRealPos.Format("%s%d_Real3DPoint.txt", strName, CameraNo);

	 CheckFileExists(strImagePoint, true);
	 CheckFileExists(strMeasurePluase, true);
	 CheckFileExists(strMeasurePos, true);
	 CheckFileExists(strRealPos, true);

	 if (IDOK != XiMessageBox("确认数据文件都已保存？"))
	 {
		 return;
	 }

	 FILE* pfImageResult = fopen(strImagePoint.GetBuffer(0)/*".\\MeasureData\\ImagePoint.txt"*/, "r");
	 while (fscanf(pfImageResult, "%d%d%d", &nImageNo, &tImagePoint.x, &tImagePoint.y) > 0)
	 {
		 vtImagePoint.push_back(tImagePoint);
	 }
	 fclose(pfImageResult);

	 int nPointNum = vtImagePoint.size();
	 XUI::MesBox::PopInfo("nPointNum:{0}", nPointNum);

	 std::vector<T_ROBOT_COORS> vtRobotMeasureCoors;
	 vtRobotMeasureCoors.clear();

	 T_ROBOT_COORS tRobotCoor;

	 tRobotCoor.dX = 0.0;
	 tRobotCoor.dY = 0.0;
	 tRobotCoor.dZ = 0.0;
	 tRobotCoor.dRX = 0.0;
	 tRobotCoor.dRY = 0.0;
	 tRobotCoor.dRZ = 0.0;

	 FILE* pfMeasurePos = fopen(strMeasurePos.GetBuffer(0)/*".\\MeasureData\\TeachPoint.txt"*/, "r");
	 int nRealPointNo = 0;
	 while (fscanf(pfMeasurePos, "%d%lf%lf%lf%lf%lf%lf", &nRealPointNo, &tRobotCoor.dX, &tRobotCoor.dY, &tRobotCoor.dZ, &tRobotCoor.dRX, &tRobotCoor.dRY, &tRobotCoor.dRZ) > 0)
	 {
		 vtRobotMeasureCoors.push_back(tRobotCoor);
	 }
	 fclose(pfMeasurePos);

	 //for (int nPointNo = 0; nPointNo < nPointNum; nPointNo++)
	 //{
		// vtRobotMeasureCoors.push_back(tRobotCoor);
	 //}

	 T_ROBOT_COORS tFirstRobotCoor, tSecondRobotCoor, tThirdRobotCoor, tFourthRobotCoor;
	 T_ROBOT_COORS tRealRobotCoor;
	 tRealRobotCoor.dX = 0.0;
	 tRealRobotCoor.dY = 0.0;
	 tRealRobotCoor.dZ = 0.0;
	 tRealRobotCoor.dRX = 0.0;
	 tRealRobotCoor.dRY = 0.0;
	 tRealRobotCoor.dRZ = 0.0;
	 int nRealPointType = 0;

	 std::vector<T_ROBOT_COORS> vtRobotBaseCoors;
	 vtRobotBaseCoors.clear();

	 FILE* pfRealTeach = fopen(strRealPos.GetBuffer(0)/*".\\MeasureData\\TeachPoint.txt"*/, "r");
	 while (fscanf(pfRealTeach, "%d%lf%lf%lf", &nRealPointNo, &tRealRobotCoor.dX, &tRealRobotCoor.dY, &tRealRobotCoor.dZ) > 0)
	 {
		 vtRobotBaseCoors.push_back(tRealRobotCoor);
	 }
	 fclose(pfRealTeach);

	 std::vector<T_ANGLE_PULSE> vtRobotAnglePulses;
	 vtRobotAnglePulses.clear();
	 T_ANGLE_PULSE tAnglePulse;
	 int nPointNo = 0;
	 int nPointType = 1;

	 std::vector<T_ABS_POS_IN_BASE> vtAbsPosInBase;
	 vtAbsPosInBase.clear();

	 FILE* pf = fopen(strMeasurePluase.GetBuffer(0)/*".\\MeasureData\\MeasurePosPluse.txt"*/, "r");
	 while (fscanf(pf, "%d%ld%ld%ld%ld%ld%ld", &nPointNo, &tAnglePulse.nSPulse, &tAnglePulse.nLPulse, &tAnglePulse.nUPulse, &tAnglePulse.nRPulse, &tAnglePulse.nBPulse, &tAnglePulse.nTPulse) > 0)
	 {
		 vtRobotAnglePulses.push_back(tAnglePulse);
	 }
	 fclose(pf);

	 double dXAdjustSum = 4.0;
	 double dYAdjustSum = 4.0;
	 double dZAdjustSum = 4.0;

	 double dXAdjustUnit = 0.2;
	 double dYAdjustUnit = 0.2;
	 double dZAdjustUnit = 0.2;

	 double dXAdjust = 0.0;
	 double dYAdjust = 0.0;
	 double dZAdjust = 0.0;

	 double dMinDis = 999999.0;
	 double dMinXAdjust = 0.0;
	 double dMinYAdjust = 0.0;
	 double dMinZAdjust = 0.0;
	 double dDis = 0.0;

	 XiLaserLightParamNode tLaserLightParam;
	 XiCV_Image_Processing* pXcvImgProc;
	 pXcvImgProc = &XiCV_Image_Processing(ptUnit->m_vtCameraPara[CameraNo].tDHCameraDriverPara.nMaxWidth,
		 ptUnit->m_vtCameraPara[CameraNo].tDHCameraDriverPara.nMaxHeight);

	 double dX = ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dX;
	 double dY = ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dY;
	 double dZ = ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dZ;

	 for (int nXNo = 0; nXNo < int((2 * dXAdjustSum) / dXAdjustUnit); nXNo++)
	 {
		 for (int nYNo = 0; nYNo < int((2 * dYAdjustSum) / dYAdjustUnit); nYNo++)
		 {
			 for (int nZNo = 0; nZNo < int((2 * dZAdjustSum) / dZAdjustUnit); nZNo++)
			 {
				 dXAdjust = -dXAdjustSum + dXAdjustUnit * nXNo;
				 dYAdjust = -dYAdjustSum + dYAdjustUnit * nYNo;
				 dZAdjust = -dZAdjustSum + dZAdjustUnit * nZNo;

				 vtAbsPosInBase.clear();
				 for (int nPointNo = 0; nPointNo < nPointNum; nPointNo++)
				 {
					 XI_POINT tKeyPoint3D;
					 XI_POINT tCamCenter3D;


					 tLaserLightParam.l_dx = ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tCameraInnerPara.dPixelX;
					 tLaserLightParam.l_dy = ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tCameraInnerPara.dPixelY;
					 tLaserLightParam.l_f = ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tCameraInnerPara.dFocal;
					 tLaserLightParam.l_len = ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tCameraInnerPara.dBaseLineLength;
					 tLaserLightParam.l_ctan = ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tCameraInnerPara.dCtanBaseLineAngle;


					 CvPoint3D64f cptKeyPnt3D = pXcvImgProc->KeyPoint2DTo3D(vtImagePoint[nPointNo], 1, tLaserLightParam);
					 tKeyPoint3D.x = cptKeyPnt3D.x;
					 tKeyPoint3D.y = cptKeyPnt3D.y;
					 tKeyPoint3D.z = cptKeyPnt3D.z;

					 CvPoint      tCamCentrePnt = cvPoint(ptUnit->m_vtCameraPara[CameraNo].tDHCameraDriverPara.nMaxWidth / 2,
						 ptUnit->m_vtCameraPara[CameraNo].tDHCameraDriverPara.nMaxHeight / 2);
					 CvPoint3D64f cpCamCenterPnt3D = pXcvImgProc->KeyPoint2DTo3D(tCamCentrePnt, 1, tLaserLightParam);
					 tCamCenter3D.x = cpCamCenterPnt3D.x;
					 tCamCenter3D.y = cpCamCenterPnt3D.y;
					 tCamCenter3D.z = cpCamCenterPnt3D.z;

					 XI_POINT tCornerPoint;

					 tCornerPoint.x = (tKeyPoint3D.x - tCamCenter3D.x) * (1.0);
					 tCornerPoint.y = (tKeyPoint3D.y - tCamCenter3D.y) * (1.0);
					 tCornerPoint.z = (tKeyPoint3D.z - tCamCenter3D.z) * (1.0);

					 ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dX = dX + dXAdjust;
					 ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dY = dY + dYAdjust;
					 ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dZ = dZ + dZAdjust;

					 tPointAbsCoordInBase = ptUnit->GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->MeasureWeldLinePointInBase(
						 tCornerPoint, vtRobotMeasureCoors[nPointNo], vtRobotAnglePulses[nPointNo],
						 ptUnit->GetRobotCtrl()->m_eManipulatorType, ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara);

					 vtAbsPosInBase.push_back(tPointAbsCoordInBase);
				 }

				 dDis = 0.0;
				 for (int nPointNo = 0; nPointNo < nPointNum; nPointNo++)
				 {
					 dDis += sqrt(SQUARE(vtRobotBaseCoors[nPointNo].dX - vtAbsPosInBase[nPointNo].tWeldLinePos.x) +
						 SQUARE(vtRobotBaseCoors[nPointNo].dY - vtAbsPosInBase[nPointNo].tWeldLinePos.y) +
						 SQUARE(vtRobotBaseCoors[nPointNo].dZ - vtAbsPosInBase[nPointNo].tWeldLinePos.z));
				 }

				 if (dDis < dMinDis)
				 {
					 dMinDis = dDis;
					 dMinXAdjust = dXAdjust;
					 dMinYAdjust = dYAdjust;
					 dMinZAdjust = dZAdjust;
				 }
			 }
		 }
	 }

	 double dAverageError = dMinDis / double(nPointNum);
	 CString sResult;
	 sResult.Format("平均误差：%11.3lf\n X 补偿：%11.3lf\n Y 补偿：%11.3lf\n Z 补偿：%11.3lf", dAverageError, dMinXAdjust, dMinYAdjust, dMinZAdjust);
	 if (dAverageError < 0.5)
	 {
		 XiMessageBox("手眼校验结果良好！\n" + sResult);
	 }
	 else if (dAverageError < 1.0)
	 {
		 XiMessageBox("手眼校验结果一般！\n" + sResult);
	 }
	 else
	 {
		 XiMessageBox("手眼校验结果非常差，请检查数据是否正确！\n" + sResult);
	 }

	 vtAbsPosInBase.clear();
	 for (nPointNo = 0; nPointNo < nPointNum; nPointNo++)
	 {
		 XI_POINT tKeyPoint3D;
		 XI_POINT tCamCenter3D;

		 tLaserLightParam.l_dx = ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tCameraInnerPara.dPixelX;
		 tLaserLightParam.l_dy = ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tCameraInnerPara.dPixelY;
		 tLaserLightParam.l_f = ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tCameraInnerPara.dFocal;
		 tLaserLightParam.l_len = ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tCameraInnerPara.dBaseLineLength;
		 tLaserLightParam.l_ctan = ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tCameraInnerPara.dCtanBaseLineAngle;


		 CvPoint3D64f cptKeyPnt3D = pXcvImgProc->KeyPoint2DTo3D(vtImagePoint[nPointNo], 1, tLaserLightParam);
		 tKeyPoint3D.x = cptKeyPnt3D.x;
		 tKeyPoint3D.y = cptKeyPnt3D.y;
		 tKeyPoint3D.z = cptKeyPnt3D.z;

		 CvPoint      tCamCentrePnt = cvPoint(ptUnit->m_vtCameraPara[CameraNo].tDHCameraDriverPara.nMaxWidth / 2,
			 ptUnit->m_vtCameraPara[CameraNo].tDHCameraDriverPara.nMaxHeight / 2);
		 CvPoint3D64f cpCamCenterPnt3D = pXcvImgProc->KeyPoint2DTo3D(tCamCentrePnt, 1, tLaserLightParam);
		 tCamCenter3D.x = cpCamCenterPnt3D.x;
		 tCamCenter3D.y = cpCamCenterPnt3D.y;
		 tCamCenter3D.z = cpCamCenterPnt3D.z;

		 XI_POINT tCornerPoint;

		 tCornerPoint.x = (tKeyPoint3D.x - tCamCenter3D.x) * (1.0);
		 tCornerPoint.y = (tKeyPoint3D.y - tCamCenter3D.y) * (1.0);
		 tCornerPoint.z = (tKeyPoint3D.z - tCamCenter3D.z) * (1.0);

		 ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dX = dX + dMinXAdjust;
		 ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dY = dY + dMinYAdjust;
		 ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dZ = dZ + dMinZAdjust;

		 tPointAbsCoordInBase = ptUnit->GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->MeasureWeldLinePointInBase(
			 tCornerPoint, vtRobotMeasureCoors[nPointNo], vtRobotAnglePulses[nPointNo],
			 ptUnit->GetRobotCtrl()->m_eManipulatorType, ptUnit->m_vtCameraPara[CameraNo].tHandEyeCaliPara);

		 vtAbsPosInBase.push_back(tPointAbsCoordInBase);
	 }

	 dDis = 0.0;
	 std::vector<double> vdDis;
	 vdDis.clear();
	 CString strResult;
	 strResult.Format("%s_result.txt", strName);
	 FILE* Result = fopen(strResult.GetBuffer(0), "w");
	 for (nPointNo = 0; nPointNo < nPointNum; nPointNo++)
	 {
		 dDis = sqrt(SQUARE(vtRobotBaseCoors[nPointNo].dX - vtAbsPosInBase[nPointNo].tWeldLinePos.x) +
			 SQUARE(vtRobotBaseCoors[nPointNo].dY - vtAbsPosInBase[nPointNo].tWeldLinePos.y) +
			 SQUARE(vtRobotBaseCoors[nPointNo].dZ - vtAbsPosInBase[nPointNo].tWeldLinePos.z));
		 vdDis.push_back(dDis);
		 fprintf(Result, "%4d%11.3lf%11.3lf%11.3lf\n", nPointNo, vtAbsPosInBase[nPointNo].tWeldLinePos.x, vtAbsPosInBase[nPointNo].tWeldLinePos.y, vtAbsPosInBase[nPointNo].tWeldLinePos.z);
	 }
	 fclose(Result);
#endif
 }

 void CAssemblyWeld::OnBnClickedButton2()
 {
	 // TODO: 在此添加控件通知处理程序代码	
	 WriteLog("单击：坡口参数");
	 ChangeGroovePara cWeldParamProcess;
	 cWeldParamProcess.DoModal();
	 return;
 }


 void CAssemblyWeld::OnBnClickedButton3()
 {
	 // TODO: 在此添加控件通知处理程序代码
	 WriteLog("单击：坡口焊接");
	 AfxBeginThread(ThreadGrooveTeachWeld, this);
	 return;
	 /*
	 int nRobotNo = 0;
	 int dirRobot = -1;
	 vector<T_WAVE_PARA> vtTWavePara;
	 vector<T_INFOR_WAVE_RAND> vtGrooveRand;
	 //获取起终点数据
	 T_ROBOT_COORS tRobotStartCoord = { 100,100,0,180,45,0,0,0,0 };
	 T_ROBOT_COORS tRobotEndCoord = { 100,100,-100,0,0,0,0,0,0 };
	 GetTeachPos(tRobotStartCoord, tRobotEndCoord, nRobotNo);
	 if (0 != GetGroovePara(tRobotStartCoord, tRobotEndCoord, vtTWavePara))
	 {
		 XiMessageBox("获取摆弧参数失败");
	 }
	 // TODO: 在此添加控件通知处理程序代码
	 // 
	 T_GROOVE_INFOR tGrooveInfor;
	 //视觉处理函数，最后参数还未整理，目前只可以用来测试视觉功能
	 //if (MB_OK == XiMessageBox("是否进行扫描"))
	 //{
		// ScanGrooveInfo(tRobotStartCoord, tRobotEndCoord, nRobotNo, tGrooveInfor);
	 //}


	 //自动排道
	 tGrooveInfor.weldAngle = tRobotStartCoord.dRZ;
	 tGrooveInfor.dPlateThickness = 20.0;
	 tGrooveInfor.dStartLowerFace = 12.0;
	 tGrooveInfor.dStartUpperFace = 25.0;
	 tGrooveInfor.dEndLowerFace = 12.0;
	 tGrooveInfor.dEndUpperFace = 20.0;
	 tGrooveInfor.tStartPoint = tRobotStartCoord;
	 tGrooveInfor.tEndPoint = tRobotEndCoord;
	 if (0 != GrooveAutoRandNew(tGrooveInfor, vtTWavePara, vtGrooveRand, dirRobot))
	 {
		 return;
	 }
	 vector<T_ROBOT_COORS> vtGrooveWavePath;
	 vector<double> weldSpeedRate;
	 vector<vector<T_ROBOT_COORS>> vvtGrooveWavePath;
	 vector<vector<double>> vWeldSpeedRate;
	 //计算摆弧轨迹
	 for (size_t i = 0; i < vtGrooveRand.size(); i++)
	 {
		 vtGrooveWavePath.clear();
		 weldSpeedRate.clear();
		 CalWavePath(tGrooveInfor, vtGrooveRand[i], vtTWavePara[i], vtGrooveWavePath, weldSpeedRate);
		 vvtGrooveWavePath.push_back(vtGrooveWavePath);
		 vWeldSpeedRate.push_back(weldSpeedRate);

		 CString sFileName;
		 sFileName.Format("LocalFiles\\OutputFiles\\RobotA\\Recognition\\%d_%d_RealWeldCoord.txt", 0, i);
		 FILE* pf = fopen(sFileName, "w");
		 for (int nPtnIdx = 0; nPtnIdx < vtGrooveWavePath.size(); nPtnIdx++)
		 {
			 T_ROBOT_COORS& tCoord = vtGrooveWavePath[nPtnIdx];
			 fprintf(pf, "%d%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%11.3lf%4d%10d%11.3lf\n", nPtnIdx,
				 tCoord.dX, tCoord.dY, tCoord.dZ, tCoord.dRX, tCoord.dRY, tCoord.dRZ, tCoord.dBX, tCoord.dBY, tCoord.dBZ,
				 0.0, E_PLAT_GROOVE, E_WELD_TRACK, weldSpeedRate[nPtnIdx]);
		 }
		 fclose(pf);

	 }
	 //焊接
	 if (0 == GrooveWeld(vvtGrooveWavePath, vtTWavePara, vWeldSpeedRate, nRobotNo))
	 {
		 MessageBox("焊接成功");
	 }
	 else
	 {
		 MessageBox("焊接失败");
	 }*/
 }


 void CAssemblyWeld::OnPaint()
 {
	 // TODO: 在此处添加消息处理程序代码
	 if (IsIconic())
	 {
		 CPaintDC dc(this); // device context for painting

		 SendMessage(WM_ICONERASEBKGND, (WPARAM)dc.GetSafeHdc(), 0);

		 // Center icon in client rectangle
		 int cxIcon = GetSystemMetrics(SM_CXICON);
		 int cyIcon = GetSystemMetrics(SM_CYICON);
		 CRect rect;
		 GetClientRect(&rect);
		 int x = (rect.Width() - cxIcon + 1) / 2;
		 int y = (rect.Height() - cyIcon + 1) / 2;

		 // Draw the icon
		 // dc.DrawIcon(x, y, m_hIcon);
	 }
	 else
	 {
		 CPaintDC dc(this); // device context for painting
		 // 不为绘图消息调用 CDialog::OnPaint()
		 CRect rect;
		 GetClientRect(&rect);
		 CDC dcMem;
		 dcMem.CreateCompatibleDC(&dc);
		 CBitmap bmpBackground;
		 bmpBackground.LoadBitmap(IDB_BITMAP1);
		 BITMAP bitmap;
		 bmpBackground.GetBitmap(&bitmap);
		 CBitmap* pbmpOld = dcMem.SelectObject(&bmpBackground);
		 dc.SetStretchBltMode(HALFTONE);             //*添加于此
		 dc.StretchBlt(0, 0, rect.Width(), rect.Height(), &dcMem, 0, 0, bitmap.bmWidth, bitmap.bmHeight, SRCCOPY);
	 }
 }


 void CAssemblyWeld::OnDrawItem(int nIDCtl, LPDRAWITEMSTRUCT lpDrawItemStruct)
 {
	 // TODO: 在此添加消息处理程序代码和/或调用默认值

	 CDialog::OnDrawItem(nIDCtl, lpDrawItemStruct);

	 CDC dc;
	 dc.Attach(lpDrawItemStruct->hDC);//得到绘制的设备环境CDC
	 ASSERT(lpDrawItemStruct->CtlType == ODT_BUTTON);
	 CString strText;
	 ((CButton*)GetDlgItem(nIDCtl))->GetWindowText(strText);
	 SetBkMode(lpDrawItemStruct->hDC, TRANSPARENT);//透明
	 if (nIDCtl == IDC_BUTTON_SPARE4 || nIDCtl == IDC_BUTTON_SYSTEM_PARA2 || nIDCtl == IDC_BUTTON_ADVANCED_CONTINUE
		 || nIDCtl == IDC_BUTTON_START || nIDCtl == IDC_BUTTON_TABLE_PARA2 || nIDCtl == IDC_BUTTON_LOAD_TRACK
		 || nIDCtl == IDC_BUTTON_SYSTEM_PARA || nIDCtl == IDC_BUTTON_SPARE1 || nIDCtl == IDC_BUTTON_COMMONLY_USED_IO
		 || nIDCtl == IDC_BUTTON_VISION_SHOW || nIDCtl == IDC_BUTTON_ROBOT_CTRL || nIDCtl == IDC_BUTTON_TABLE_PARA
		 || nIDCtl == IDC_BUTTON_PAUSE_CONTINUE || nIDCtl == IDC_BUTTON_PANO_RECOGNITION || nIDCtl == IDC_BUTTON_ADJUST_RECOG
		 || nIDCtl == IDC_BUTTON_BACK_HOME || nIDCtl == IDC_BUTTON_PANO_RECOGNITION2 || nIDCtl == IDC_BUTTON_TABLE_PARA3
		 || nIDCtl == IDC_BUTTON_TABLE_PARA4 || nIDCtl == IDC_BUTTON_SIMULATE_INCISE || nIDCtl == IDC_BUTTON_CLOSE
		 || nIDCtl == IDC_BUTTON3 || nIDCtl == IDC_BUTTON2
		 || nIDCtl == IDC_RADIO_ARC || nIDCtl == IDC_RADIO_NOARC
		 || nIDCtl == IDC_CHECK3 || nIDCtl == IDC_CHECK_GRAY2 || nIDCtl == IDC_CHECK_NATURAL_POP)
	 {
		 if (GetDlgItem(nIDCtl)->IsWindowEnabled())  //当按钮不操作 & 按钮可用
		 {
			 CBrush brush(RGB(173, 215, 255));    //内背景画刷 30, 200, 255
			 CPen m_BoundryPen(0, 2, RGB(80, 80, 80));   //边框画笔
			 CBrush m_BackgroundBrush = RGB(140, 200, 255);    //大背景画刷 173, 215, 255   140, 200, 255
			 dc.FillRect(&(lpDrawItemStruct->rcItem), &m_BackgroundBrush);//利用画刷brush，填充大矩形框	
			 CRect rect = lpDrawItemStruct->rcItem;
			 CDC* pDC = CDC::FromHandle(lpDrawItemStruct->hDC);
			 POINT pt;
			 //画按钮的外边框，它是一个半径为10的圆角矩形
			 pt.x = 10;
			 pt.y = 10;
			 CPen* hOldPen = pDC->SelectObject(&m_BoundryPen);
			 pDC->RoundRect(&rect, pt);
			 pDC->SelectObject(hOldPen);
			 rect.DeflateRect(3, 3, 3, 3);   //缩进
			 CBrush* pOldBrush = pDC->SelectObject(&m_BackgroundBrush);
			 pDC->Rectangle(rect);    //画矩形
			 pDC->SelectObject(pOldBrush);
			 pDC->FillRect(rect, &brush);    //填充内矩形
			 //因为这里进行了重绘,所以文字也要重绘
			 DrawText(lpDrawItemStruct->hDC, strText, strText.GetLength(), &lpDrawItemStruct->rcItem, DT_SINGLELINE | DT_VCENTER | DT_CENTER);
		 }
		 if (lpDrawItemStruct->itemState & ODS_SELECTED & GetDlgItem(nIDCtl)->IsWindowEnabled())    //如果按钮可用 & 点击
		 {
			 CBrush brush(RGB(0, 160, 230));    //内背景画刷 0, 160, 230
			 CPen m_BoundryPen(0, 1, RGB(80, 80, 80));   //边框画笔
			 CBrush m_BackgroundBrush = RGB(140, 200, 255);    //大背景画刷 140, 200, 255
			 dc.FillRect(&(lpDrawItemStruct->rcItem), &m_BackgroundBrush);//利用画刷brush，填充矩形框	
			 CRect rect = lpDrawItemStruct->rcItem;
			 CDC* pDC = CDC::FromHandle(lpDrawItemStruct->hDC);
			 POINT pt;
			 //画按钮的外边框，它是一个半径为5的圆角矩形
			 pt.x = 10;
			 pt.y = 10;
			 CPen* hOldPen = pDC->SelectObject(&m_BoundryPen);
			 pDC->RoundRect(&rect, pt);
			 pDC->SelectObject(hOldPen);
			 rect.DeflateRect(4, 4, 3, 3);
			 CBrush* pOldBrush = pDC->SelectObject(&m_BackgroundBrush);
			 pDC->Rectangle(rect);
			 pDC->SelectObject(pOldBrush);
			 pDC->FillRect(rect, &brush);
			 //因为这里进行了重绘,所以文字也要重绘
			 DrawText(lpDrawItemStruct->hDC, strText, strText.GetLength(), &lpDrawItemStruct->rcItem, DT_SINGLELINE | DT_VCENTER | DT_CENTER);
		 }
		 if (!GetDlgItem(nIDCtl)->IsWindowEnabled())    //不可用，显示灰色
		 {
			 CBrush brush(RGB(190, 190, 200));    //内背景画刷
			 CPen m_BoundryPen(0, 2, RGB(80, 80, 80));   //边框画笔
			 CBrush m_BackgroundBrush = RGB(140, 200, 255);    //大背景画刷
			 dc.FillRect(&(lpDrawItemStruct->rcItem), &m_BackgroundBrush);//利用画刷brush，填充矩形框	
			 CRect rect = lpDrawItemStruct->rcItem;
			 CDC* pDC = CDC::FromHandle(lpDrawItemStruct->hDC);
			 POINT pt;
			 //画按钮的外边框，它是一个半径为5的圆角矩形
			 pt.x = 10;
			 pt.y = 10;
			 CPen* hOldPen = pDC->SelectObject(&m_BoundryPen);
			 pDC->RoundRect(&rect, pt);
			 pDC->SelectObject(hOldPen);
			 rect.DeflateRect(3, 3, 3, 3);
			 CBrush* pOldBrush = pDC->SelectObject(&m_BackgroundBrush);
			 pDC->Rectangle(rect);
			 pDC->SelectObject(pOldBrush);
			 pDC->FillRect(rect, &brush);
			 //因为这里进行了重绘,所以文字也要重绘
			 pDC->SetTextColor(RGB(255, 255, 255)); //设置文本颜色
			 DrawText(lpDrawItemStruct->hDC, strText, strText.GetLength(), &lpDrawItemStruct->rcItem, DT_SINGLELINE | DT_VCENTER | DT_CENTER);
		 }
		 dc.Detach();
	 }
 }

 void WeldAfterMeasure::DelPngFile(const CString& directory)
 {
	 // 构造文件匹配模式
	 CString pattern = directory + _T("\\*.png");

	 // 使用 FindFirstFile 和 FindNextFile 遍历文件
	 WIN32_FIND_DATA findData;
	 HANDLE hFind = FindFirstFile(pattern, &findData);

	 if (hFind != INVALID_HANDLE_VALUE) {
		 do {
			 CString filePath = directory + _T("\\") + findData.cFileName;
			 try {
				 // 删除文件
				 remove(filePath);
				 Sleep(100);
				 WriteLog("去除点云背景删除:%s", filePath);

			 }
			 catch (...) {
				 WriteLog("去除点云背景删除失败:%s", filePath);
			 }
		 } while (FindNextFile(hFind, &findData));
		 FindClose(hFind);
	 }
	 else {
		 WriteLog("去除点云背景中没有png文件不用删除");
	 }
 }

 CvPoint3D64f* WeldAfterMeasure::SaveRemoveCloud(CRobotDriverAdaptor* pRobotDriver, CString cFileName, vector<CvPoint3D64f>& vtPointCloud, int nPtnNum)
 {
	 FILE* pf = fopen(cFileName.GetBuffer(), "w");
	 //vtPointCloud.clear();
	 for (int i = 0; i < nPtnNum; i++)
	 {

		 fprintf(pf, "%d %11.3lf %11.3lf %11.3lf %11.3lf %11.3lf %11.3lf\n", i, vtPointCloud[i].x, vtPointCloud[i].y, vtPointCloud[i].z, 0.0, 0.0, 0.0);
		 vtPointCloud.push_back(vtPointCloud[i]);
	 }
	 fclose(pf);
	 return vtPointCloud.data();

 }

 void CAssemblyWeld::initWorkpieceType()
 {
	 //读取现在已有的工件类型
	 m_vsWorkpieceType = findSubfolder(".\\ConfigFiles\\WorkpieceType\\");
	 for (size_t i = 0; i < m_vsWorkpieceType.size(); i++)
	 {
		 int nStringNo = m_vsWorkpieceType[i].ReverseFind('\\') + 1;
		 m_vsWorkpieceType[i] = m_vsWorkpieceType[i].Right(m_vsWorkpieceType[i].GetLength() - nStringNo);
	 }

	 //读取上次使用的工件类型
	 CString sWorkpieceName;
	 XiBase::COPini opini;
	 opini.SetFileName(".\\ConfigFiles\\WorkpieceType\\Chioce.ini");
	 opini.SetSectionName("CurChioce");
	 opini.ReadString("WorkpieceName", sWorkpieceName);

	 //设置选项
	 int nCurNo = 0;
	 m_comboWorkpieceType.ResetContent();
	 for (size_t i = 0; i < m_vsWorkpieceType.size(); i++)
	 {
		 //已修改
		 sWorkpieceName = XUI::Languge::GetInstance().translate(sWorkpieceName);
		 m_vsWorkpieceType[i] = XUI::Languge::GetInstance().translate(m_vsWorkpieceType[i].GetBuffer());

		 m_comboWorkpieceType.AddString(m_vsWorkpieceType[i]);
		 if (m_vsWorkpieceType[i] == sWorkpieceName)
		 {
			 nCurNo = i;
		 }
	 }
	 m_comboWorkpieceType.SetCurSel(nCurNo);

	 //替换文件
	 CString sSrcFile = ".\\ConfigFiles\\WorkpieceType\\" + m_vsWorkpieceType[nCurNo] + "\\Get_Welding_Info.ini";
	 CString sDstFile = ".\\LocalFiles\\ExLib\\Vision\\ConfigFiles\\Get_Welding_Info.ini";
	 CopyFile(sSrcFile, sDstFile, FALSE);
 }

 void CAssemblyWeld::setWorkpieceType()
 {
	 //获取当前的选择
	 UpdateData(TRUE);
	 auto nNo = m_comboWorkpieceType.GetCurSel();
	 if (nNo < 0 || nNo >= m_vsWorkpieceType.size())
	 {
		 m_comboWorkpieceType.SetCurSel(0);
		 nNo = 0;
	 }

	 //保存当前使用的工件类型
	 CString sWorkpieceName;
	 XiBase::COPini opini;
	 opini.SetFileName(".\\ConfigFiles\\WorkpieceType\\Chioce.ini");
	 opini.SetSectionName("CurChioce");
	 opini.WriteString("WorkpieceName", m_vsWorkpieceType[nNo]);

	 //替换文件
	 CString sSrcFile = ".\\ConfigFiles\\WorkpieceType\\" + m_vsWorkpieceType[nNo] + "\\Get_Welding_Info.ini";
	 CString sDstFile = ".\\LocalFiles\\ExLib\\Vision\\ConfigFiles\\Get_Welding_Info.ini";
	 CopyFile(sSrcFile, sDstFile, FALSE);
	 sSrcFile = ".\\ConfigFiles\\WorkpieceType\\" + m_vsWorkpieceType[nNo] + "\\RTT.ini";
	 sDstFile = ".\\ConfigFiles\\RTT.ini";
	 CopyFile(sSrcFile, sDstFile, FALSE);
 }

 void CAssemblyWeld::OnCbnSelchangeWorkpieceType2()
 {
	 // TODO: 在此添加控件通知处理程序代码
	 setWorkpieceType();
 }

 void CAssemblyWeld::OnBnClickedCheckModel()
 {
	 UpdateData(TRUE);

	 if (m_nUseModel)
	 {
		 WriteLog("单击：使用模型");
	 }
	 else
	 {
		 WriteLog("单击：不使用模型");
	 }
 }
