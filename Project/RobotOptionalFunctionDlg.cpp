// RobotOptionalFunctionDlg.cpp : 实现文件
//

#include "stdafx.h"
#include ".\Project\XiGrooveRobot.h"
#include "RobotOptionalFunctionDlg.h"
#include "afxdialogex.h"


// CRobotOptionalFunctionDlg 对话框

IMPLEMENT_DYNAMIC(CRobotOptionalFunctionDlg, CDialogEx)

CRobotOptionalFunctionDlg::CRobotOptionalFunctionDlg(CRobotDriverAdaptor *pRobotDriver, CWnd* pParent /*=NULL*/)
	: m_pYasakawaRobotDriver(pRobotDriver), CDialogEx(IDD_ADD_ROBOT, pParent)
	, m_nPVarNum(0)
	, m_dPosX(0)
	, m_dPosY(0)
	, m_dPosZ(0)
	, m_dPosRX(0)
	, m_dPosRY(0)
	, m_dPosRZ(0)
	, m_lSPulse(0)
	, m_lLPulse(0)
	, m_lUPulse(0)
	, m_lRPulse(0)
	, m_lBPulse(0)
	, m_lTPulse(0)
	, m_BType(FALSE)
{

}

CRobotOptionalFunctionDlg::~CRobotOptionalFunctionDlg()
{
}

void CRobotOptionalFunctionDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT_P_NUM, m_nPVarNum);
	DDV_MinMaxInt(pDX, m_nPVarNum, 0, 200);
	DDX_Text(pDX, IDC_EDIT_COOR_X1, m_dPosX);
	DDX_Text(pDX, IDC_EDIT_COOR_Y1, m_dPosY);
	DDX_Text(pDX, IDC_EDIT_COOR_Z1, m_dPosZ);
	DDX_Text(pDX, IDC_EDIT_COOR_RX1, m_dPosRX);
	DDX_Text(pDX, IDC_EDIT_COOR_RY1, m_dPosRY);
	DDX_Text(pDX, IDC_EDIT_COOR_RZ1, m_dPosRZ);
	DDX_Text(pDX, IDC_EDIT_PULSE_S, m_lSPulse);
	DDX_Text(pDX, IDC_EDIT_PULSE_L, m_lLPulse);
	DDX_Text(pDX, IDC_EDIT_PULSE_U, m_lUPulse);
	DDX_Text(pDX, IDC_EDIT_PULSE_R, m_lRPulse);
	DDX_Text(pDX, IDC_EDIT_PULSE_B, m_lBPulse);
	DDX_Text(pDX, IDC_EDIT_PULSE_T, m_lTPulse);
	DDX_Radio(pDX, IDC_RADIO1, m_BType);
}


BEGIN_MESSAGE_MAP(CRobotOptionalFunctionDlg, CDialogEx)
	ON_BN_CLICKED(IDC_BUTTON_SAVE_P_VAR, &CRobotOptionalFunctionDlg::OnBnClickedButtonSavePVar)
	ON_BN_CLICKED(IDC_BUTTON_CHANGE, &CRobotOptionalFunctionDlg::OnBnClickedButtonChange)
	ON_BN_CLICKED(IDC_RADIO1, &CRobotOptionalFunctionDlg::OnBnClickedRadio1)
	ON_BN_CLICKED(IDC_RADIO2, &CRobotOptionalFunctionDlg::OnBnClickedRadio2)
END_MESSAGE_MAP()


BOOL CRobotOptionalFunctionDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  在此添加额外的初始化
	SetState(TRUE, FALSE);
	//已修改
	XUI::Languge::GetInstance().translateDialog(this);
	return TRUE;  // return TRUE unless you set the focus to a control
				  // 异常: OCX 属性页应返回 FALSE
}

// CRobotOptionalFunctionDlg 消息处理程序


void CRobotOptionalFunctionDlg::OnBnClickedButtonSavePVar()
{
	UpdateData(TRUE);

	//读取
	UINT unCount = 1;
	MP_VAR_INFO tVarInfo[1];
	tVarInfo[0].usIndex = m_nPVarNum;
	tVarInfo[0].usType = MP_RESTYPE_VAR_ROBOT;
	LONG lPosVarData[10];
	m_pYasakawaRobotDriver->GetMultiPosVar(unCount, tVarInfo, lPosVarData);

	//写入
	FILE *pfSpecialPoints;
	XI_fopen_s(&pfSpecialPoints, ".\\GraphData\\PVar.txt", "a+");
	if (MP_ROBO_COORD == (lPosVarData[0] & 0x3F))//直角坐标
	{
		fprintf_s(pfSpecialPoints, "P%d:   %10.3lf   %10.3lf   %10.3lf   %10.4lf   %10.4lf   %10.4lf\n",
			m_nPVarNum, lPosVarData[2] / 1000.0, lPosVarData[3] / 1000.0, lPosVarData[4] / 1000.0, 
			lPosVarData[5] / 10000.0, lPosVarData[6] / 10000.0, lPosVarData[7] / 10000.0);
	}
	else if (MP_PULSE_COORD == (lPosVarData[0] & 0x3F))//关节坐标
	{
		fprintf_s(pfSpecialPoints, "P%d：%10ld%10ld%10ld%10ld%10ld%10ld\n", m_nPVarNum, lPosVarData[2], lPosVarData[3],
			lPosVarData[4], lPosVarData[5], lPosVarData[6], lPosVarData[7] );
	}	
	fclose(pfSpecialPoints);
}


void CRobotOptionalFunctionDlg::OnBnClickedButtonChange()
{
	UpdateData(TRUE);
	if (m_BType == FALSE)
	{
		double adCoord[6];
		adCoord[0] = m_dPosX;
		adCoord[1] = m_dPosY;
		adCoord[2] = m_dPosZ;
		adCoord[3] = m_dPosRX;
		adCoord[4] = m_dPosRY;
		adCoord[5] = m_dPosRZ;
		long alPulse[6];	
		m_pYasakawaRobotDriver->ConvCartToAxesPos(adCoord, alPulse);
		m_lSPulse = alPulse[0];
		m_lLPulse = alPulse[1];
		m_lUPulse = alPulse[2];
		m_lRPulse = alPulse[3];
		m_lBPulse = alPulse[4];
		m_lTPulse = alPulse[5];
		UpdateData(FALSE);
	}
	else
	{
		long alPulse[6];
		alPulse[0] = m_lSPulse;
		alPulse[1] = m_lLPulse;
		alPulse[2] = m_lUPulse;
		alPulse[3] = m_lRPulse;
		alPulse[4] = m_lBPulse;
		alPulse[5] = m_lTPulse;
		UINT * unFig_ctrl = 0;
		double adCoord[6];
		m_pYasakawaRobotDriver->ConvAxesToCartPos(alPulse, unFig_ctrl, adCoord);
		m_dPosX = adCoord[0];
		m_dPosY = adCoord[1];
		m_dPosZ = adCoord[2];
		m_dPosRX = adCoord[3];
		m_dPosRY = adCoord[4];
		m_dPosRZ = adCoord[5];
		UpdateData(FALSE);
	}
}


void CRobotOptionalFunctionDlg::OnBnClickedRadio1()
{
	UpdateData(TRUE);
	SetState(TRUE, FALSE);
}


void CRobotOptionalFunctionDlg::OnBnClickedRadio2()
{
	UpdateData(TRUE);
	SetState(FALSE, TRUE);
}

void CRobotOptionalFunctionDlg::SetState(bool bRightAngleState, bool bPulseState)
{
	GetDlgItem(IDC_EDIT_COOR_X1)->EnableWindow(bRightAngleState);
	GetDlgItem(IDC_EDIT_COOR_Y1)->EnableWindow(bRightAngleState);
	GetDlgItem(IDC_EDIT_COOR_Z1)->EnableWindow(bRightAngleState);
	GetDlgItem(IDC_EDIT_COOR_RX1)->EnableWindow(bRightAngleState);
	GetDlgItem(IDC_EDIT_COOR_RY1)->EnableWindow(bRightAngleState);
	GetDlgItem(IDC_EDIT_COOR_RZ1)->EnableWindow(bRightAngleState);
	GetDlgItem(IDC_EDIT_PULSE_S)->EnableWindow(bPulseState);
	GetDlgItem(IDC_EDIT_PULSE_L)->EnableWindow(bPulseState);
	GetDlgItem(IDC_EDIT_PULSE_U)->EnableWindow(bPulseState);
	GetDlgItem(IDC_EDIT_PULSE_R)->EnableWindow(bPulseState);
	GetDlgItem(IDC_EDIT_PULSE_B)->EnableWindow(bPulseState);
	GetDlgItem(IDC_EDIT_PULSE_T)->EnableWindow(bPulseState);
}
