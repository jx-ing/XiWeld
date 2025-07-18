// DlgTestRobotComm.cpp : implementation file
//

#include "stdafx.h"
#include ".\Project\XiGrooveRobot.h"
#include ".\Project\RobotCtrlDlg.h"
#include "RobotOptionalFunctionDlg.h"
#include ".\Apps\PLib\BasicFunc\Const.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CDlgTestRobotComm dialog

CRobotCtrlDlg::CRobotCtrlDlg(std::vector < CUnit*> vpUnit, CWnd* pParent /*=NULL*/)
	: CDialog(IDD_CTRL_ROBOT, pParent)
{
	for (size_t i = 0; i < vpUnit.size(); i++)
	{
		if (vpUnit[i]->GetRobotEnableState())
		{
			m_vpRobotDriver.push_back(vpUnit[i]->GetRobotCtrl());
		}
	}

	m_nRobotPosMoveMode = 0;
	m_nPosAxis = -1;
	m_nPulseAxis = -1;
	m_nRobotPulseMoveMode = 0;
	m_dPosMoveSpeed = 0.0;
	m_nPulseMoveSpeed = 0.0;

	Init();
}

CRobotCtrlDlg::CRobotCtrlDlg(std::vector<CRobotDriverAdaptor*> vpRobotDriver, CWnd* pParent /*=NULL*/)
	: m_vpRobotDriver(vpRobotDriver), CDialog(CRobotCtrlDlg::IDD, pParent)
{
	if (m_vpRobotDriver.size() > 0)
	{
		m_pCurRobotDriver = m_vpRobotDriver[0];
	}
	else
	{
		m_pCurRobotDriver = NULL;
	}
	m_nRobotPosMoveMode = 0;
	m_nPosAxis = -1;
	m_nPulseAxis = -1;
	m_nRobotPulseMoveMode = 0;
	m_dPosMoveSpeed = 0.0;
	m_nPulseMoveSpeed = 0.0;

	Init();
}

CRobotCtrlDlg::CRobotCtrlDlg(CRobotDriverAdaptor *pRobotDriver, CWnd* pParent /*= NULL*/)
	: m_pCurRobotDriver(pRobotDriver), CDialog(CRobotCtrlDlg::IDD, pParent)
{
	m_vpRobotDriver.push_back(m_pCurRobotDriver);
	m_nRobotPosMoveMode = 0;
	m_nPosAxis = -1;
	m_nPulseAxis = -1;
	m_nRobotPulseMoveMode = 0;
	m_dPosMoveSpeed = 0.0;
	m_nPulseMoveSpeed = 0.0;

	Init();
}

void CRobotCtrlDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CDlgTestRobotComm)
	DDX_Radio(pDX, IDC_RADIO_POSITION_MODE1, m_nRobotPosMoveMode);
	DDX_CBIndex(pDX, IDC_COMBO_AXIS1, m_nPosAxis);
	DDX_CBIndex(pDX, IDC_COMBO_AXIS2, m_nPulseAxis);
	DDX_Radio(pDX, IDC_RADIO_POSITION_MODE2, m_nRobotPulseMoveMode);
	DDX_Text(pDX, IDC_EDIT_LEFT_ROBOT_SPEED, m_dPosMoveSpeed);
	DDX_Text(pDX, IDC_EDIT_RIGHT_ROBOT_SPEED, m_nPulseMoveSpeed);
	//}}AFX_DATA_MAP
	DDX_Control(pDX, IDC_COMBO_ROBOT, m_cRobot);
}

BEGIN_MESSAGE_MAP(CRobotCtrlDlg, CDialog)
	//{{AFX_MSG_MAP(CDlgTestRobotComm)
	//ON_WM_TIMER()
	ON_WM_CLOSE()
	ON_BN_CLICKED(IDC_BUTTON_PRECISE_MOVE1, OnBtnPosMove)
	ON_BN_CLICKED(IDC_BUTTON_HOLD_ON, OnBtnHoldOn)
	ON_BN_CLICKED(IDC_BUTTON_PRECISE_MOVE2, OnBtnPulseMove)
	ON_BN_CLICKED(IDC_BUTTON_HOLD_OFF, OnBtnHoldOff)
	ON_BN_CLICKED(IDC_BUTTON_SERVO_ON_LEFT, OnBtnServoOn)
	ON_BN_CLICKED(IDC_CHECK_REVERSE2, OnCheckReverse2)
	ON_BN_CLICKED(IDC_CHECK_REVERSE1, OnCheckReverse1)
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_BUTTON_SERVO_OFF, &CRobotCtrlDlg::OnBtnServoOff)
	ON_BN_CLICKED(IDC_BUTTON_DEBUG, &CRobotCtrlDlg::OnBnClickedButtonDebug)
	ON_CBN_SELCHANGE(IDC_COMBO_ROBOT, &CRobotCtrlDlg::OnCbnSelchangeComboRobot)
	ON_BN_CLICKED(IDC_BUTTON_BACK_HOME, &CRobotCtrlDlg::OnBnClickedButtonBackHome)
	ON_WM_TIMER()
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDlgTestRobotComm message handlers

void CRobotCtrlDlg::Init()
{
	m_dPosMoveSpeed = 500.0;
	m_nPulseMoveSpeed = 1000;
}

void CRobotCtrlDlg::LoadRobotName()
{
	//刷新界面
	m_cRobot.ResetContent();
	for (int i = 0; i < m_vpRobotDriver.size(); i++)
	{
		//已修改
		m_vpRobotDriver[i]->m_strCustomName = XUI::Languge::GetInstance().translate(m_vpRobotDriver[i]->m_strCustomName.GetBuffer());
		
		m_cRobot.AddString(m_vpRobotDriver[i]->m_strCustomName);
	}
	m_cRobot.SetCurSel(0);
	OnCbnSelchangeComboRobot();
}

BOOL CRobotCtrlDlg::OnInitDialog() 
{
	CDialog::OnInitDialog();
	
	LoadRobotName();

	// TODO: Add extra initialization here
	SetTimer(1, 300, NULL);

	double dMoveDis = 1.0;
	SetDlgItemData(IDC_COMBO_DIS1, dMoveDis, this);
	SetDlgItemData(IDC_COMBO_DIS2, dMoveDis, this);
	
	//已修改
	XUI::Languge::GetInstance().translateDialog(this);
	return TRUE;  // return TRUE unless you set the focus to a control
	// EXCEPTION: OCX Property Pages should return FALSE
}


void CRobotCtrlDlg::OnClose() 
{
	KillTimer(1);
	Sleep(500);
	
	CDialog::OnClose();
}

void CRobotCtrlDlg::ShowRobotCoor()
{
	double dCoorX  = m_pCurRobotDriver->GetCurrentPos(ROBOT_AXIS_X);
	CString strCoorX;
	strCoorX.Format( "%f", dCoorX );
	GetDlgItem(IDC_EDIT_COOR_X1)->SetWindowText(strCoorX);
	
	double dCoorY = m_pCurRobotDriver->GetCurrentPos(ROBOT_AXIS_Y);
	CString strCoorY;
	strCoorY.Format( "%f", dCoorY );
	GetDlgItem(IDC_EDIT_COOR_Y1)->SetWindowText(strCoorY);
	
	double dCoorZ = m_pCurRobotDriver->GetCurrentPos(ROBOT_AXIS_Z);
	CString strCoorZ;
	strCoorZ.Format( "%f", dCoorZ );
	GetDlgItem(IDC_EDIT_COOR_Z1)->SetWindowText(strCoorZ);
	
	double dCoorRX = m_pCurRobotDriver->GetCurrentPos(ROBOT_AXIS_RX);
	CString strCoorRX;
	strCoorRX.Format( "%f", dCoorRX );
	GetDlgItem(IDC_EDIT_COOR_RX1)->SetWindowText(strCoorRX);
	
	double dCoorRY = m_pCurRobotDriver->GetCurrentPos(ROBOT_AXIS_RY);
	CString strCoorRY;
	strCoorRY.Format( "%f", dCoorRY );
	GetDlgItem(IDC_EDIT_COOR_RY1)->SetWindowText(strCoorRY);
	
	double dCoorRZ = m_pCurRobotDriver->GetCurrentPos(ROBOT_AXIS_RZ);
	CString strCoorRZ;
	strCoorRZ.Format( "%f", dCoorRZ );
	GetDlgItem(IDC_EDIT_COOR_RZ1)->SetWindowText(strCoorRZ);

	long lSPulse = m_pCurRobotDriver->GetCurrentPulse(SAxis);
	CString strCoorSAxis;
	strCoorSAxis.Format("%ld", lSPulse);
	GetDlgItem(IDC_EDIT_PULSE_S)->SetWindowText(strCoorSAxis);

	long lLPulse = m_pCurRobotDriver->GetCurrentPulse(LAxis);
	CString strCoorLAxis;
	strCoorLAxis.Format("%ld", lLPulse);
	GetDlgItem(IDC_EDIT_PULSE_L)->SetWindowText(strCoorLAxis);

	long lUPulse = m_pCurRobotDriver->GetCurrentPulse(UAxis);
	CString strCoorUAxis;
	strCoorUAxis.Format("%ld", lUPulse);
	GetDlgItem(IDC_EDIT_PULSE_U)->SetWindowText(strCoorUAxis);

	long lRPulse = m_pCurRobotDriver->GetCurrentPulse(RAxis);
	CString strCoorRAxis;
	strCoorRAxis.Format("%ld", lRPulse);
	GetDlgItem(IDC_EDIT_PULSE_R)->SetWindowText(strCoorRAxis);

	long lBPulse = m_pCurRobotDriver->GetCurrentPulse(BAxis);
	CString strCoorBAxis;
	strCoorBAxis.Format("%ld", lBPulse);
	GetDlgItem(IDC_EDIT_PULSE_B)->SetWindowText(strCoorBAxis);

	long lTPulse = m_pCurRobotDriver->GetCurrentPulse(TAxis);
	CString strCoorTAxis;
	strCoorTAxis.Format("%ld", lTPulse);
	GetDlgItem(IDC_EDIT_PULSE_T)->SetWindowText(strCoorTAxis);
}

void CRobotCtrlDlg::OnBtnPosMove() 
{
	CheckPosAxis();

	UpdateData(TRUE);

	double dMoveDis = 0;
	GetDlgItemData(IDC_COMBO_DIS1, dMoveDis, this);

	int nMoveMode = 0;
	if (0 == m_nRobotPosMoveMode)
	{
		nMoveMode = COORD_REL;
	}
	else
	{
		nMoveMode = COORD_ABS;
	}	
	m_pCurRobotDriver->PosMove(m_nPosAxis, dMoveDis, m_dPosMoveSpeed, nMoveMode);
}

void CRobotCtrlDlg::OnBtnHoldOn() 
{
	m_pCurRobotDriver->HoldOn();
}

void CRobotCtrlDlg::OnBtnPulseMove() 
{
	CheckPulseAxis();
	
	int nMoveMode = 0;
	if (0 == m_nRobotPulseMoveMode)
	{
		nMoveMode = COORD_REL;
	}
	else
	{
		nMoveMode = COORD_ABS;
	}
	
	double dPulse = 0.0;
	GetDlgItemData(IDC_COMBO_DIS2, dPulse, this);

	long lPulse = (long)dPulse;
	m_pCurRobotDriver->AxisPulseMove(m_nPulseAxis, lPulse, m_nPulseMoveSpeed, nMoveMode);
}

void CRobotCtrlDlg::OnBtnHoldOff() 
{
	m_pCurRobotDriver->HoldOff();
}

void CRobotCtrlDlg::OnBtnServoOn() 
{
	m_pCurRobotDriver->ServoOn();
}

void CRobotCtrlDlg::OnCheckReverse2() 
{
	UpdateData(TRUE);
	double dMoveDis = 0;
	GetDlgItemData(IDC_COMBO_DIS2, dMoveDis, this);
	dMoveDis *= -1;
	SetDlgItemData(IDC_COMBO_DIS2, dMoveDis, this);
}

void CRobotCtrlDlg::OnCheckReverse1() 
{
	UpdateData(TRUE);
	double dMoveDis = 0;
	GetDlgItemData(IDC_COMBO_DIS1, dMoveDis, this);
	dMoveDis *= -1;
	SetDlgItemData(IDC_COMBO_DIS1, dMoveDis, this);
}

void CRobotCtrlDlg::CheckPulseAxis()
{
	UpdateData(TRUE);

	if (-1 == m_nPulseAxis)
	{
		XiMessageBox("请选择轴号!");
		return ;
	}
}

void CRobotCtrlDlg::CheckPosAxis()
{
	UpdateData(TRUE);
	
	if (-1 == m_nPosAxis)
	{
		XiMessageBox("请选择轴号!");
		return ;
	}
}

void CRobotCtrlDlg::OnBtnServoOff()
{
	m_pCurRobotDriver->ServoOff();
}


void CRobotCtrlDlg::OnBnClickedButtonDebug()
{
	CRobotOptionalFunctionDlg cRobotOptionalFunctionDlg(m_pCurRobotDriver);
	cRobotOptionalFunctionDlg.DoModal();
}


void CRobotCtrlDlg::OnCbnSelchangeComboRobot()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	int nIndex = m_cRobot.GetCurSel();
	CString str;
	m_cRobot.GetLBText(nIndex, str);

	for (int i = 0; i < m_vpRobotDriver.size(); i++)
	{
		if (str == m_vpRobotDriver[i]->m_strCustomName)
		{
			m_pCurRobotDriver = m_vpRobotDriver[i];
			break;
		}
	}
	UpdateData(FALSE);
}


void CRobotCtrlDlg::OnBnClickedButtonBackHome()
{
	T_ROBOT_MOVE_SPEED tPulseMove;
	tPulseMove.dACC = 20;
	tPulseMove.dDEC = 80;
	tPulseMove.dSpeed = 1500;
	m_pCurRobotDriver->MoveByJob(m_pCurRobotDriver->m_tHomePulse, tPulseMove, m_pCurRobotDriver->m_nExternalAxleType, "MOVJ-TEST");
}


void CRobotCtrlDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值

	if (1 == nIDEvent)
	{
		ShowRobotCoor();
	}
	CDialog::OnTimer(nIDEvent);
}
