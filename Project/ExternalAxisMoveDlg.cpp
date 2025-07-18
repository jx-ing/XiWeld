// ExternalAxisMoveDlg.cpp : 实现文件
//

#include "stdafx.h"
#include ".\Project\XiGrooveRobot.h"
#include ".\Project\ExternalAxisMoveDlg.h"
#include "afxdialogex.h"
#include "LTDMC.h"


const CString EXTERNAL_AXIS = _T("Data\\ExternalAxis.ini");
// CExternalAxisMoveDlg 对话框

IMPLEMENT_DYNAMIC(CExternalAxisMoveDlg, CDialogEx)

CExternalAxisMoveDlg::CExternalAxisMoveDlg(CServoMotorDriver *pSetServoCtrl,CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_DIALOG_EXTERNAL, pParent)
	, m_nInput1(0)
	, m_nInput2(0)
	, m_nInput3(0)
	, m_nOutput1(0)
	, m_nOutput2(0)
	, m_nOutput3(0)
	, m_nOutputMark1(FALSE)
	, m_nOutputMark2(FALSE)
	, m_nOutputMark3(FALSE)
{
	m_pcSetServoCtrl = pSetServoCtrl;
	if (m_pcSetServoCtrl == NULL)
	{
		m_pcSetServoCtrl = new CServoMotorDriver;
		m_pcSetServoCtrl->LoadCtrlCardParam(m_pcSetServoCtrl->m_vtCtrlCardInfo);

		if (m_pcSetServoCtrl->InitCtrlCard(m_pcSetServoCtrl->m_vtCtrlCardInfo) != 0)
		{
			AfxMessageBox("运动控制卡初始化失败");
		}
	}
}

CExternalAxisMoveDlg::CExternalAxisMoveDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_DIALOG_EXTERNAL, pParent)
	, m_nInput1(0)
	, m_nInput2(0)
	, m_nInput3(0)
	, m_nOutput1(0)
	, m_nOutput2(0)
	, m_nOutput3(0)
	, m_nOutputMark1(FALSE)
	, m_nOutputMark2(FALSE)
	, m_nOutputMark3(FALSE)
{
	m_pcSetServoCtrl = new CServoMotorDriver;
	m_pcSetServoCtrl->LoadCtrlCardParam(m_pcSetServoCtrl->m_vtCtrlCardInfo);

	if (m_pcSetServoCtrl->InitCtrlCard(m_pcSetServoCtrl->m_vtCtrlCardInfo) != 0)
	{
		AfxMessageBox("运动控制卡初始化失败");
	}
}

CExternalAxisMoveDlg::~CExternalAxisMoveDlg()
{
}

void CExternalAxisMoveDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO1, m_ComBox_ExtAxis);
	DDX_Control(pDX, IDC_COMBO3, m_ComBox_AxisMode);
	DDX_Control(pDX, IDC_COMBO4, m_ComBox_MoveDirection);
	DDX_Control(pDX, IDC_COMBO2, m_ComBox_StopMode);
	DDX_Text(pDX, IDC_EDIT8, m_nInput1);
	DDX_Text(pDX, IDC_EDIT11, m_nInput2);
	DDX_Text(pDX, IDC_EDIT12, m_nInput3);
	DDX_Text(pDX, IDC_EDIT13, m_nOutput1);
	DDX_Text(pDX, IDC_EDIT14, m_nOutput2);
	DDX_Text(pDX, IDC_EDIT15, m_nOutput3);
	DDX_Check(pDX, IDC_CHECK1, m_nOutputMark1);
	DDX_Check(pDX, IDC_CHECK2, m_nOutputMark2);
	DDX_Check(pDX, IDC_CHECK4, m_nOutputMark3);
	DDX_Control(pDX, IDC_COMBO5, m_cBox_ChoiceSpeed);
}


BEGIN_MESSAGE_MAP(CExternalAxisMoveDlg, CDialogEx)
	ON_WM_TIMER()
	ON_CBN_SELCHANGE(IDC_COMBO1, &CExternalAxisMoveDlg::OnCbnSelchangeCombo1)
	ON_CBN_SELCHANGE(IDC_COMBO3, &CExternalAxisMoveDlg::OnCbnSelchangeCombo3)
	ON_CBN_SELCHANGE(IDC_COMBO4, &CExternalAxisMoveDlg::OnCbnSelchangeCombo4)
	ON_CBN_SELCHANGE(IDC_COMBO2, &CExternalAxisMoveDlg::OnCbnSelchangeCombo2)
	ON_BN_CLICKED(IDC_BUTTON3, &CExternalAxisMoveDlg::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_BUTTON11, &CExternalAxisMoveDlg::OnBnClickedButton11)
	ON_BN_CLICKED(IDC_BUTTON14, &CExternalAxisMoveDlg::OnBnClickedButton14)
	ON_BN_CLICKED(IDC_BUTTON1, &CExternalAxisMoveDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON17, &CExternalAxisMoveDlg::OnBnClickedButton17)
	ON_BN_CLICKED(IDC_BUTTON13, &CExternalAxisMoveDlg::OnBnClickedButton13)
	ON_BN_CLICKED(IDOK, &CExternalAxisMoveDlg::OnBnClickedOk)
	ON_WM_CTLCOLOR()
	ON_EN_CHANGE(IDC_EDIT8, &CExternalAxisMoveDlg::OnEnChangeEdit8)
	ON_EN_CHANGE(IDC_EDIT11, &CExternalAxisMoveDlg::OnEnChangeEdit11)
	ON_EN_CHANGE(IDC_EDIT12, &CExternalAxisMoveDlg::OnEnChangeEdit12)
	ON_BN_CLICKED(IDC_CHECK1, &CExternalAxisMoveDlg::OnBnClickedCheck1)
	ON_EN_CHANGE(IDC_EDIT13, &CExternalAxisMoveDlg::OnEnChangeEdit13)
	ON_EN_CHANGE(IDC_EDIT14, &CExternalAxisMoveDlg::OnEnChangeEdit14)
	ON_EN_CHANGE(IDC_EDIT15, &CExternalAxisMoveDlg::OnEnChangeEdit15)
	ON_BN_CLICKED(IDC_CHECK2, &CExternalAxisMoveDlg::OnBnClickedCheck2)
	ON_BN_CLICKED(IDC_CHECK4, &CExternalAxisMoveDlg::OnBnClickedCheck4)
	ON_BN_CLICKED(IDC_BUTTON16, &CExternalAxisMoveDlg::OnBnClickedButton16)
	ON_BN_CLICKED(IDC_BUTTON18, &CExternalAxisMoveDlg::OnBnClickedButton18)
	ON_BN_CLICKED(IDC_BUTTON16, &CExternalAxisMoveDlg::OnBnClickedButton16)
	ON_BN_CLICKED(IDC_BUTTON12, &CExternalAxisMoveDlg::OnBnClickedButton12)
	ON_CBN_SELCHANGE(IDC_COMBO5, &CExternalAxisMoveDlg::OnCbnSelchangeCombo5)
END_MESSAGE_MAP()


// CExternalAxisMoveDlg 消息处理程序


BOOL CExternalAxisMoveDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	// TODO:  在此添加额外的初始化


	


	m_usCardNo = 0;
	//下拉框设置
	//外部轴选择
	for (int n = 0; n < m_pcSetServoCtrl->m_vtCtrlCardInfo.size(); n++)
	{
		for (int m = 0; m < m_pcSetServoCtrl->m_vtCtrlCardInfo[n].nAxisNum; m++)
		{
			m_ComBox_ExtAxis.AddString(GetStr("卡%d:%s", n, m_pcSetServoCtrl->m_vtCtrlCardInfo[n].vtAxisInfo[m].strAxisName));
			m_pcSetServoCtrl->SetSevon(m_pcSetServoCtrl->m_vtCtrlCardInfo[n].vtAxisInfo[m].nSoftAxisNo, true);
		}
	}
// 	m_ComBox_ExtAxis.AddString("轴0");
// 	m_ComBox_ExtAxis.AddString("轴1");
// 	m_ComBox_ExtAxis.AddString("轴2");
	m_ComBox_ExtAxis.SetCurSel(0);

	//设置初始速度的值
	m_cBox_ChoiceSpeed.ResetContent();
	for (int i = 0; i < 5; i++)
	{
		m_cBox_ChoiceSpeed.AddString(GetStr("%d  mm/s", naSpeedTableByRobot[i]));
	}
	m_cBox_ChoiceSpeed.SetCurSel(0);

	InitExternalPluseEquivalent();

	m_Axis = 0;
	GetDlgItem(IDC_EDIT31)->SetWindowText(GetStr("轴 %d", m_Axis));
	GetDlgItem(IDC_EDIT33)->SetWindowText(GetStr("脉冲当量:%.2lf", m_dAxisPluseEquivalent[m_Axis]));

	//轴运动模式
	m_ComBox_AxisMode.AddString("相对位置");
	m_ComBox_AxisMode.AddString("绝对位置");
	m_ComBox_AxisMode.SetCurSel(0);
	m_AxisMode = 0;
	//运动方向
	m_ComBox_MoveDirection.AddString("负方向");
	m_ComBox_MoveDirection.AddString("正方向");
	m_ComBox_MoveDirection.SetCurSel(1);
	m_MoveDirection = 1;
	//停止模式
	m_ComBox_StopMode.AddString("减速停止");
	m_ComBox_StopMode.AddString("立即停止");
	m_ComBox_StopMode.SetCurSel(0);
	m_StopMode = 0;
	
	SetTimer(1, 100, NULL);//定时器
	m_dDistancePluse[m_Axis]=0;//脉冲
	m_dVeloci[m_Axis] = 2000;//速度
	m_dOriVel[m_Axis] = 1000;//初始速度
	m_dADDTime[m_Axis] = 2;//加速时间
	m_dDesTime[m_Axis] = 2;//减速时间
	InitExternalArgument();//初始化速度信息
	GetDlgItem(IDC_EDIT1)->SetWindowText(GetStr("%0.0lf", m_dDistancePluse[m_Axis]));
	GetDlgItem(IDC_EDIT2)->SetWindowText(GetStr("%0.2lf", m_dOriVel[m_Axis]));
	GetDlgItem(IDC_EDIT4)->SetWindowText(GetStr("%0.2lf", m_dVeloci[m_Axis]));
	GetDlgItem(IDC_EDIT5)->SetWindowText(GetStr("%0.2lf", m_dADDTime[m_Axis]));
	GetDlgItem(IDC_EDIT6)->SetWindowText(GetStr("%0.2lf", m_dDesTime[m_Axis]));

	ShowTransformVelociToMM(m_Axis);
	ShowTransformDistanceToMM(m_Axis);

	//初始速度，设置为禁止使用
	
	GetDlgItem(IDC_EDIT4)->EnableWindow(FALSE);
	GetDlgItem(IDC_EDIT32)->EnableWindow(FALSE);//速度
	//已修改
	XUI::Languge::GetInstance().translateDialog(this);
	return TRUE;  // return TRUE unless you set the focus to a control
				  // 异常: OCX 属性页应返回 FALSE
}


void CExternalAxisMoveDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值

	if (nIDEvent == 1)
	{
		ShowExternalAxis0Information();
		// 	ShowExternalAxis1Information();
		// 	ShowExternalAxis2Information();
			//ShowTransformDistanceToMM(m_Axis);
			//ShowTransformVelociToMM(m_Axis);
		WORD wState;
		int nSoftCtrlCardNo = m_nInput1 / 1000;
		int nNoteNo = (m_nInput1 % 1000) / 100;
		int nBitNo = m_nInput1 % 100;
		int nRtn;
		nRtn = m_pcSetServoCtrl->ReadInbit(nSoftCtrlCardNo, nBitNo, wState, nNoteNo);
		if (nRtn != 0)
		{
			m_abError[0] = true;
		}
		else
		{
			m_abError[0] = false;
		}
		if (!wState != m_abSignal[0])
		{
			m_abSignal[0] = !m_abSignal[0];
		}
		GetDlgItem(IDC_EDIT8)->RedrawWindow();

		nSoftCtrlCardNo = m_nInput2 / 1000;
		nNoteNo = (m_nInput2 % 1000) / 100;
		nBitNo = m_nInput2 % 100;
		nRtn = m_pcSetServoCtrl->ReadInbit(nSoftCtrlCardNo, nBitNo, wState, nNoteNo);
		if (nRtn != 0)
		{
			m_abError[1] = true;
		}
		else
		{
			m_abError[1] = false;
		}
		if (!wState != m_abSignal[1])
		{
			m_abSignal[1] = !m_abSignal[1];
		}
		GetDlgItem(IDC_EDIT11)->RedrawWindow();

		nSoftCtrlCardNo = m_nInput3 / 1000;
		nNoteNo = (m_nInput3 % 1000) / 100;
		nBitNo = m_nInput3 % 100;
		nRtn = m_pcSetServoCtrl->ReadInbit(nSoftCtrlCardNo, nBitNo, wState, nNoteNo);
		if (nRtn != 0)
		{
			m_abError[2] = true;
		}
		else
		{
			m_abError[2] = false;
		}
		if (!wState != m_abSignal[2])
		{
			m_abSignal[2] = !m_abSignal[2];
		}
		GetDlgItem(IDC_EDIT12)->RedrawWindow();
	}
	



	CDialogEx::OnTimer(nIDEvent);
}

bool CExternalAxisMoveDlg::ShowExternalAxis0Information()
{
	WORD wAxis = m_Axis;
	double dPositionu;
	short iret = 0;
	CString NowValue;
	iret = dmc_get_position_unit(m_usCardNo, wAxis, &dPositionu);
	//位置unis
	
	GetDlgItem(IDC_EDIT7)->GetWindowTextA(NowValue);
	if (NowValue != GetStr("%.0lf", dPositionu))
	{
		GetDlgItem(IDC_EDIT7)->SetWindowTextA(GetStr("%.0lf", dPositionu));
	}
	
	//位置mm
	double dPosion_mm = dPositionu / m_dAxisPluseEquivalent[wAxis];
	GetDlgItem(IDC_EDIT9)->GetWindowTextA(NowValue);
	if (NowValue != GetStr("%.2lf", dPosion_mm))
	{
		GetDlgItem(IDC_EDIT9)->SetWindowTextA(GetStr("%.2lf", dPosion_mm));
	}

	

	//当前速度u/s
	double current_speed;
	dmc_read_current_speed_unit(m_usCardNo, wAxis, &current_speed);
	GetDlgItem(IDC_EDIT10)->SetWindowTextA(GetStr("%.2lf", current_speed));
	//当前速度mm/min
	double current_speed_MM = current_speed / m_dAxisPluseEquivalent[wAxis]*60;
	GetDlgItem(IDC_EDIT22)->SetWindowTextA(GetStr("%.2lf", current_speed_MM));


	DWORD status = dmc_check_done(m_usCardNo, wAxis);
	if (status == 1)
	{
		GetDlgItem(IDC_EDIT24)->SetWindowText("静止");
	}
	else
	{
		GetDlgItem(IDC_EDIT24)->SetWindowText("运动");
	}

	WORD runmode = 0;
	iret = dmc_get_axis_run_mode(m_usCardNo, wAxis, &runmode);
	switch (runmode)
	{
	case 0: {GetDlgItem(IDC_EDIT25)->SetWindowText("空闲"); break; }
	case 1: {GetDlgItem(IDC_EDIT25)->SetWindowText("定长"); break; }
	case 2: {GetDlgItem(IDC_EDIT25)->SetWindowText("定速"); break; }
	default:
		break;
	}
	if (iret==0)
	{
		return true;
	}
	return false;
}

bool CExternalAxisMoveDlg::ShowExternalAxis1Information()
{
	WORD wAxis = 1;
	double dPositionu;
	short iret = 0;
	CString NowValue;
	iret = dmc_get_position_unit(m_usCardNo, wAxis, &dPositionu);
	//位置unis
	GetDlgItem(IDC_EDIT16)->GetWindowTextA(NowValue);
	if (NowValue != GetStr("%.0lf", dPositionu))
	{
		GetDlgItem(IDC_EDIT16)->SetWindowTextA(GetStr("%.0lf", dPositionu));
	}

	//位置mm
	double dPosion_mm = dPositionu / m_dAxisPluseEquivalent[wAxis];
	GetDlgItem(IDC_EDIT17)->GetWindowTextA(NowValue);
	if (NowValue != GetStr("%.2lf", dPosion_mm))
	{
		GetDlgItem(IDC_EDIT17)->SetWindowTextA(GetStr("%.2lf", dPosion_mm));
	}


	//当前速度u/s
	double current_speed;
	dmc_read_current_speed_unit(m_usCardNo, wAxis, &current_speed);
	GetDlgItem(IDC_EDIT23)->SetWindowTextA(GetStr("%.2lf", current_speed));
	//当前速度mm/min
	double current_speed_MM = current_speed / m_dAxisPluseEquivalent[wAxis] * 60;
	GetDlgItem(IDC_EDIT34)->SetWindowTextA(GetStr("%.2lf", current_speed_MM));

	DWORD status = dmc_check_done(m_usCardNo, wAxis);
	if (status == 1)
	{
		GetDlgItem(IDC_EDIT26)->SetWindowText("静止");
	}
	else
	{
		GetDlgItem(IDC_EDIT26)->SetWindowText("运动");
	}
	WORD runmode = 0;
	iret = dmc_get_axis_run_mode(m_usCardNo, wAxis, &runmode);
	switch (runmode)
	{
	case 0: {GetDlgItem(IDC_EDIT27)->SetWindowText("空闲"); break; }
	case 1: {GetDlgItem(IDC_EDIT27)->SetWindowText("定长"); break; }
	case 2: {GetDlgItem(IDC_EDIT27)->SetWindowText("定速"); break; }
	default:
		break;
	}
	if (iret == 0)
	{
		return true;
	}
	return false;
}

bool CExternalAxisMoveDlg::ShowExternalAxis2Information()
{
	WORD wAxis = 2;
	double dPositionu;
	short iret = 0;
	CString NowValue;
	iret = dmc_get_position_unit(m_usCardNo, wAxis, &dPositionu);
	//位置unis
	GetDlgItem(IDC_EDIT20)->GetWindowTextA(NowValue);
	if (NowValue != GetStr("%.0lf", dPositionu))
	{
		GetDlgItem(IDC_EDIT20)->SetWindowTextA(GetStr("%.0lf", dPositionu));
	}

	//位置mm
	double dPosion_mm = dPositionu / m_dAxisPluseEquivalent[wAxis];
	GetDlgItem(IDC_EDIT21)->GetWindowTextA(NowValue);
	if (NowValue != GetStr("%.2lf", dPosion_mm))
	{
		GetDlgItem(IDC_EDIT21)->SetWindowTextA(GetStr("%.2lf", dPosion_mm));
	}
	

	//当前速度u/s
	double current_speed;
	dmc_read_current_speed_unit(m_usCardNo, wAxis, &current_speed);
	GetDlgItem(IDC_EDIT35)->SetWindowTextA(GetStr("%.2lf", current_speed));
	//当前速度mm/min
	double current_speed_MM = current_speed / m_dAxisPluseEquivalent[wAxis] * 60;
	GetDlgItem(IDC_EDIT36)->SetWindowTextA(GetStr("%.2lf", current_speed_MM));

	DWORD status = dmc_check_done(m_usCardNo, wAxis);
	if (status == 1)
	{
		GetDlgItem(IDC_EDIT28)->SetWindowText("静止");
	}
	else
	{
		GetDlgItem(IDC_EDIT28)->SetWindowText("运动");
	}
	WORD runmode = 0;
	iret = dmc_get_axis_run_mode(m_usCardNo, wAxis, &runmode);
	switch (runmode)
	{
	case 0: {GetDlgItem(IDC_EDIT29)->SetWindowText("空闲"); break; }
	case 1: {GetDlgItem(IDC_EDIT29)->SetWindowText("定长"); break; }
	case 2: {GetDlgItem(IDC_EDIT29)->SetWindowText("定速"); break; }
	default:
		break;
	}
	if (iret == 0)
	{
		return true;
	}
	return false;
}

bool CExternalAxisMoveDlg::ShowTransformDistanceToMM(WORD Axis)
{
	
	double Dis;
	Dis = m_dDistancePluse[Axis] / m_dAxisPluseEquivalent[Axis] ;
	GetDlgItem(IDC_EDIT30)->SetWindowText(GetStr("%.2lf", Dis));//距离
	
	return true;
}

bool CExternalAxisMoveDlg::ShowTransformVelociToMM(WORD Axis)
{
	double Vel;
	Vel = m_dVeloci[Axis] * 60;
	GetDlgItem(IDC_EDIT32)->SetWindowText(GetStr("%.2lf", Vel));//速度
	return true;
}

bool CExternalAxisMoveDlg::ShowTransformDistanceMMToU(WORD Axis)
{
	double Dis;
	CString cstr;
	GetDlgItem(IDC_EDIT30)->GetWindowTextA(cstr);//距离
	Dis = atof(cstr)*m_dAxisPluseEquivalent[Axis];
	//Dis = m_dDistancePluse[Axis] / m_dAxisPluseEquivalent[Axis];
	GetDlgItem(IDC_EDIT1)->SetWindowText(GetStr("%.0lf", Dis));//距离
	m_dDistancePluse[Axis] = Dis;
	return true;
}

bool CExternalAxisMoveDlg::ShowTransformVelociMMToU(WORD Axis)
{
	double Vel;
	CString cstr;
	GetDlgItem(IDC_EDIT32)->GetWindowTextA(cstr);//距离
	Vel = atof(cstr) / 60;
	GetDlgItem(IDC_EDIT4)->SetWindowText(GetStr("%.02lf", Vel));//速度
	m_dVeloci[Axis] = Vel;
	return true;
}

//获取脉冲当量
BOOL CExternalAxisMoveDlg::InitExternalPluseEquivalent()
{
	BOOL bRet = TRUE;
	for (int n = 0; n < m_ComBox_ExtAxis.GetCount(); n++)
	{
		m_pcSetServoCtrl->GetDataPulseEquivalent(n, m_dAxisPluseEquivalent[n]);
		if (m_dAxisPluseEquivalent[n] < 1)
		{
			m_dAxisPluseEquivalent[n] = 1 / m_dAxisPluseEquivalent[n];
		}
	}
	return bRet;
}

BOOL CExternalAxisMoveDlg::InitExternalArgument()
{
	BOOL bRet = TRUE;

	//外部轴运动参数设置 位置转化的限制参数
	T_AXIS_SPEED tMinSpeedParam;
	T_AXIS_SPEED tMidSpeedParam;
	T_AXIS_SPEED tMaxSpeedParam;
	for (int n = 0; n < m_ComBox_ExtAxis.GetCount(); n++)
	{
		m_pcSetServoCtrl->GetDataSpeedParam(n, tMinSpeedParam, tMidSpeedParam, tMaxSpeedParam);
		m_dOriVel[n] = 0;
		m_dVeloci[n] = tMidSpeedParam.dSpeed / 60.0;
		m_dADDTime[n] = tMidSpeedParam.dAcc;
		m_dDesTime[n] = tMidSpeedParam.dDec;
	}

	return bRet;
}



//当前轴变化
void CExternalAxisMoveDlg::OnCbnSelchangeCombo1()
{
	// TODO: 在此添加控件通知处理程序代码
	int Axis = m_ComBox_ExtAxis.GetCurSel();
	m_Axis = Axis;
	GetDlgItem(IDC_EDIT31)->SetWindowText(GetStr("轴 %d", m_Axis));
	GetDlgItem(IDC_EDIT33)->SetWindowText(GetStr("脉冲当量:%.2lf",m_dAxisPluseEquivalent[m_Axis]));

	GetDlgItem(IDC_EDIT1)->SetWindowText(GetStr("%0.0lf", m_dDistancePluse[m_Axis]));
	GetDlgItem(IDC_EDIT2)->SetWindowText(GetStr("%0.2lf", m_dOriVel[m_Axis]));
	GetDlgItem(IDC_EDIT4)->SetWindowText(GetStr("%0.2lf", m_dVeloci[m_Axis]));
	GetDlgItem(IDC_EDIT5)->SetWindowText(GetStr("%0.2lf", m_dADDTime[m_Axis]));
	GetDlgItem(IDC_EDIT6)->SetWindowText(GetStr("%0.2lf", m_dDesTime[m_Axis]));

	ShowTransformDistanceToMM(m_Axis);
	ShowTransformVelociToMM(m_Axis);

	//设置初始速度的值
	m_cBox_ChoiceSpeed.ResetContent();
	for (int i = 0; i < 5; i++)
	{
		m_cBox_ChoiceSpeed.AddString(GetStr("%d  mm/s", Axis <=2? naSpeedTableByRobot[i]: naSpeedTableByZ[i]));
	}
	m_cBox_ChoiceSpeed.SetCurSel(0);
	nSpeedByFree = 0;
	GetDlgItem(IDC_EDIT4)->EnableWindow(FALSE);
	GetDlgItem(IDC_EDIT32)->EnableWindow(FALSE);//速度
	

}

//运行模式
void CExternalAxisMoveDlg::OnCbnSelchangeCombo3()
{
	// TODO: 在此添加控件通知处理程序代码
	int AxisMode = m_ComBox_AxisMode.GetCurSel();
	m_AxisMode = AxisMode;
}

//运动方向
void CExternalAxisMoveDlg::OnCbnSelchangeCombo4()
{
	// TODO: 在此添加控件通知处理程序代码
	int MoveDirection = m_ComBox_MoveDirection.GetCurSel();
	m_MoveDirection = MoveDirection;
}

//停止模式
void CExternalAxisMoveDlg::OnCbnSelchangeCombo2()
{
	// TODO: 在此添加控件通知处理程序代码
	int StopMode = m_ComBox_StopMode.GetCurSel();
	m_StopMode = StopMode;
}


BOOL CExternalAxisMoveDlg::OnCommand(WPARAM wParam, LPARAM lParam)
{
	// TODO: 在此添加专用代码和/或调用基类
	UINT nID = LOWORD(wParam);
	HWND hWndCtrl = (HWND)lParam;
	int nCode = HIWORD(wParam);
	if (nCode == EN_SETFOCUS)
	{
		m_nFocusEditId = nID;
	}
	else if (nCode == EN_KILLFOCUS )
	{
	
		switch (m_nFocusEditId)
		{
		case IDC_EDIT1: {CString cstr; GetDlgItem(IDC_EDIT1)->GetWindowTextA(cstr), m_dDistancePluse[m_Axis] = atof(cstr); ShowTransformDistanceToMM(m_Axis); break; }
		case IDC_EDIT2: {CString cstr; GetDlgItem(IDC_EDIT2)->GetWindowTextA(cstr), m_dOriVel[m_Axis] = atof(cstr); break; }
		case IDC_EDIT4: {CString cstr; GetDlgItem(IDC_EDIT4)->GetWindowTextA(cstr), m_dVeloci[m_Axis] = atof(cstr); ShowTransformVelociToMM(m_Axis); break; }
		case IDC_EDIT5: {CString cstr; GetDlgItem(IDC_EDIT5)->GetWindowTextA(cstr), m_dADDTime[m_Axis] = atof(cstr); break; }
		case IDC_EDIT6: {CString cstr; GetDlgItem(IDC_EDIT6)->GetWindowTextA(cstr), m_dDesTime[m_Axis] = atof(cstr); break; }
		case IDC_EDIT30: { ShowTransformDistanceMMToU(m_Axis); break; }
		case IDC_EDIT32: { ShowTransformVelociMMToU(m_Axis); break; }
		default:
			break;
		}
	}
	else if (nCode == BN_CLICKED)
	{
		m_nFocusEditId = -1;
	}
	return CDialogEx::OnCommand(wParam, lParam);
}

//停止
void CExternalAxisMoveDlg::OnBnClickedButton3()
{
	// TODO: 在此添加控件通知处理程序代码
	if (m_StopMode == 0)
	{
		m_pcSetServoCtrl->SetDecelStopTime(m_Axis, m_dDesTime[m_Axis]);
		m_pcSetServoCtrl->DecelStop(m_Axis);
	}
	else
	{
		m_pcSetServoCtrl->EmgStopAxis(m_Axis);
	}
// 	dmc_set_dec_stop_time(m_usCardNo, m_Axis, m_dDesTime[m_Axis]);
// 	DWORD status = dmc_stop(m_usCardNo, m_Axis, m_StopMode);
// 	TRACE("dmc_stop(%d, %d, %d)=%d\n", m_usCardNo, m_Axis, m_StopMode, status);
}

//紧急停止
void CExternalAxisMoveDlg::OnBnClickedButton11()
{
	// TODO: 在此添加控件通知处理程序代码
	m_pcSetServoCtrl->EmgStop();

}

//定长运动
void CExternalAxisMoveDlg::OnBnClickedButton14()
{
	// TODO: 在此添加控件通知处理程序代码
	double dDirection;
	dDirection = m_MoveDirection == 0 ? -1.0 : 1.0;
	
	m_pcSetServoCtrl->PosMove(m_Axis,
		dDirection *( m_dDistancePluse[m_Axis] / m_dAxisPluseEquivalent[m_Axis]), m_AxisMode,
		m_dVeloci[m_Axis], m_dADDTime[m_Axis], m_dDesTime[m_Axis], 0.5);
// 	DWORD status = dmc_set_profile_unit(m_usCardNo, m_Axis, m_dOriVel[m_Axis], m_dVeloci[m_Axis], m_dADDTime[m_Axis], m_dDesTime[m_Axis],1000);
// 	TRACE("dmc_set_profile_unit(%d,%d,%.02lf,%.02lf,%.02lf,%.02lf,%d) = %d\n", m_usCardNo, m_Axis, m_dOriVel[m_Axis], m_dVeloci[m_Axis], m_dADDTime[m_Axis], m_dDesTime[m_Axis], 1000, status);
// 
// 	status = dmc_set_s_profile(m_usCardNo, m_Axis, 0, 0.5);
// 	TRACE("dmc_set_s_profile(%d,%d,%d,%.02lf) = %d\n", m_usCardNo,  m_Axis, 0, 0.5, status);
// 
// 	int nDir = 1/*(m_MoveDirection == 0 ? -1 : 1)*/;
// 
// 	double dPluse = m_dDistancePluse[m_Axis] * nDir;
// 	status = dmc_pmove_unit(m_usCardNo, m_Axis, dPluse, m_AxisMode);
// 	TRACE("dmc_pmove_unit(%d,%d,%.02lf,%d) = %d\n", m_usCardNo, m_Axis, dPluse, m_AxisMode, status);


}

//定速运动
void CExternalAxisMoveDlg::OnBnClickedButton1()
{
	// TODO: 在此添加控件通知处理程序代码
	m_pcSetServoCtrl->ContiMove(m_Axis, m_MoveDirection, m_dVeloci[m_Axis], m_dADDTime[m_Axis], 0.5);
// 	dmc_set_profile_unit(m_usCardNo, m_Axis, m_dOriVel[m_Axis], m_dVeloci[m_Axis], m_dADDTime[m_Axis], m_dDesTime[m_Axis], 1000);
// 	dmc_set_s_profile(m_usCardNo, m_Axis, 0, 0.5);
// 
// 	WORD nDir = m_MoveDirection ;
// 	dmc_vmove(m_usCardNo, m_Axis, nDir);
}

//回零运动
void CExternalAxisMoveDlg::OnBnClickedButton17()
{
	// TODO: 在此添加控件通知处理程序代码
	dmc_set_homemode(m_usCardNo, m_Axis, m_MoveDirection, 0, 0, 0);
	dmc_set_home_profile_unit(m_usCardNo, m_Axis, m_dOriVel[m_Axis], m_dVeloci[m_Axis], m_dADDTime[m_Axis], m_dDesTime[m_Axis]);
	dmc_home_move(m_usCardNo, m_Axis);
}


BOOL CExternalAxisMoveDlg::PreTranslateMessage(MSG* pMsg)
{
	// TODO: 在此添加专用代码和/或调用基类

	if (pMsg->message == WM_LBUTTONDOWN)//按下按钮
	{
		if (pMsg->hwnd == GetDlgItem(IDC_BUTTON13)->m_hWnd)//+
		{
			WORD nDir = 1;//正方向
			//m_pcSetServoCtrl->ContiMove(m_Axis, m_MoveDirection, m_dVeloci[m_Axis], m_dADDTime[m_Axis], 0.5);//随着方向改变
			m_pcSetServoCtrl->ContiMove(m_Axis, nDir, m_dVeloci[m_Axis], m_dADDTime[m_Axis], 0.5);//往正方向移动
// 			dmc_set_profile_unit(m_usCardNo, m_Axis, m_dOriVel[m_Axis], m_dVeloci[m_Axis], m_dADDTime[m_Axis], m_dDesTime[m_Axis], 1000);
// 			dmc_set_s_profile(m_usCardNo, m_Axis, 0, 0.5);
// 
// 			WORD nDir = m_MoveDirection;
// 			dmc_vmove(m_usCardNo, m_Axis, nDir);

		}
		else if (pMsg->hwnd == GetDlgItem(IDC_BUTTON12)->m_hWnd)//-
		{
			//WORD nDir = m_MoveDirection == 1 ? 0 : 1;
			//m_pcSetServoCtrl->ContiMove(m_Axis, nDir, m_dVeloci[m_Axis], m_dADDTime[m_Axis], 0.5);//随着方向改变

			WORD nDir = 0;//负方向
			m_pcSetServoCtrl->ContiMove(m_Axis, nDir, m_dVeloci[m_Axis], m_dADDTime[m_Axis], 0.5);//随着方向改变

//  			dmc_set_profile_unit(m_usCardNo, m_Axis, m_dOriVel[m_Axis], m_dVeloci[m_Axis], m_dADDTime[m_Axis], m_dDesTime[m_Axis], 1000);
//  			dmc_set_s_profile(m_usCardNo, m_Axis, 0, 0.5);
//  
//  			WORD nDir = m_MoveDirection == 1 ? 0 : 1;
//  			dmc_vmove(m_usCardNo, m_Axis, nDir);

		}

	}else if (pMsg->message == WM_LBUTTONUP)//抬起
	{
		if (pMsg->hwnd == GetDlgItem(IDC_BUTTON13)->m_hWnd)//+
		{
			m_pcSetServoCtrl->SetDecelStopTime(m_Axis, m_dDesTime[m_Axis]);
			m_pcSetServoCtrl->DecelStop(m_Axis);
// 			dmc_set_dec_stop_time(m_usCardNo, m_Axis, m_dDesTime[m_Axis]);//设置1ms减速停止时间
// 			dmc_stop(m_usCardNo, m_Axis, 0);
		}
		else if (pMsg->hwnd == GetDlgItem(IDC_BUTTON12)->m_hWnd)//-
		{
			m_pcSetServoCtrl->SetDecelStopTime(m_Axis, m_dDesTime[m_Axis]);
			m_pcSetServoCtrl->DecelStop(m_Axis);
// 			dmc_set_dec_stop_time(m_usCardNo, m_Axis, m_dDesTime[m_Axis]);
// 			dmc_stop(m_usCardNo, m_Axis, 0);

		}
		
	}
	return CDialogEx::PreTranslateMessage(pMsg);
}




void CExternalAxisMoveDlg::OnBnClickedButton13()
{
	double dSpeed = 30;
	m_pcSetServoCtrl->ContiMove(m_Axis,1, dSpeed/*m_dVeloci[m_Axis]*/,0.5);
}



void CExternalAxisMoveDlg::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	CDialogEx::OnOK();

}


HBRUSH CExternalAxisMoveDlg::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor)
{
	HBRUSH hbr = CDialogEx::OnCtlColor(pDC, pWnd, nCtlColor);
	return hbr;
	// TODO:  在此更改 DC 的任何特性
	COLORREF temp;
	switch (nCtlColor)
	{
	case CTLCOLOR_EDIT:
		if (pWnd->GetDlgCtrlID() == IDC_EDIT8)
		{
			if (m_abSignal[0])
			{
				temp = m_cColorGreen;
			}
			else
			{
				temp = m_cColorRed;
			}
			if (m_abError[0])
			{
				temp = m_cColorYellow;
			}
			pDC->SetBkColor(temp);
			return CreateSolidBrush(temp);
		}
		if (pWnd->GetDlgCtrlID() == IDC_EDIT11)
		{
			if (m_abSignal[1])
			{
				temp = m_cColorGreen;
			}
			else
			{
				temp = m_cColorRed;
			}
			if (m_abError[1])
			{
				temp = m_cColorYellow;
			}
			pDC->SetBkColor(temp);
			return CreateSolidBrush(temp);
		}
		if (pWnd->GetDlgCtrlID() == IDC_EDIT12)
		{
			if (m_abSignal[2])
			{
				temp = m_cColorGreen;
			}
			else
			{
				temp = m_cColorRed;
			}
			if (m_abError[2])
			{
				temp = m_cColorYellow;
			}
			pDC->SetBkColor(temp);
			return CreateSolidBrush(temp);
		}
	default:
		break;
	}
	
	// TODO:  如果默认的不是所需画笔，则返回另一个画笔
	return hbr;
}


void CExternalAxisMoveDlg::OnEnChangeEdit8()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	UpdateData(TRUE);
}


void CExternalAxisMoveDlg::OnEnChangeEdit11()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	UpdateData(TRUE);
}


void CExternalAxisMoveDlg::OnEnChangeEdit12()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	UpdateData(TRUE);
}

void CExternalAxisMoveDlg::OnEnChangeEdit13()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	int nTemp = m_nOutput1;
	UpdateData(TRUE);
	if (nTemp != m_nOutput1)
	{
		WORD wState;
		int nSoftCtrlCardNo = m_nOutput1 / 1000;
		int nNoteNo = (m_nOutput1 % 1000) / 100;
		int nBitNo = m_nOutput1 % 100;
		m_pcSetServoCtrl->ReadOutbit(nSoftCtrlCardNo, nBitNo, wState, nNoteNo);
		m_nOutputMark1 = !wState;
		UpdateData(FALSE);
	}
}


void CExternalAxisMoveDlg::OnEnChangeEdit14()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	int nTemp = m_nOutput2;
	UpdateData(TRUE);
	if (nTemp != m_nOutput2)
	{
		WORD wState;
		int nSoftCtrlCardNo = m_nOutput2 / 1000;
		int nNoteNo = (m_nOutput2 % 1000) / 100;
		int nBitNo = m_nOutput2 % 100;
		m_pcSetServoCtrl->ReadOutbit(nSoftCtrlCardNo, nBitNo, wState, nNoteNo);
		m_nOutputMark2 = !wState;
		UpdateData(FALSE);
	}
}


void CExternalAxisMoveDlg::OnEnChangeEdit15()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	int nTemp = m_nOutput3;
	UpdateData(TRUE);
	if (nTemp != m_nOutput3)
	{
		WORD wState;
		int nSoftCtrlCardNo = m_nOutput3 / 1000;
		int nNoteNo = (m_nOutput3 % 1000) / 100;
		int nBitNo = m_nOutput3 % 100;
		m_pcSetServoCtrl->ReadOutbit(nSoftCtrlCardNo, nBitNo, wState, nNoteNo);
		m_nOutputMark3 = !wState;
		UpdateData(FALSE);
	}
}

void CExternalAxisMoveDlg::OnBnClickedCheck1()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	int nSoftCtrlCardNo = m_nOutput1 / 1000;
	int nNoteNo = (m_nOutput1 % 1000) / 100;
	int nBitNo = m_nOutput1 % 100;
	int nRtn;
	nRtn = m_pcSetServoCtrl->WriteOutbit(nSoftCtrlCardNo, nBitNo, !m_nOutputMark1, nNoteNo);
	if (nRtn != 0)
	{
		m_nOutputMark1 = !m_nOutputMark1;
		UpdateData(FALSE);
		AfxMessageBox("操作IO失败！");
	}
}

void CExternalAxisMoveDlg::OnBnClickedCheck2()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	int nSoftCtrlCardNo = m_nOutput2 / 1000;
	int nNoteNo = (m_nOutput2 % 1000) / 100;
	int nBitNo = m_nOutput2 % 100;
	int nRtn;
	nRtn = m_pcSetServoCtrl->WriteOutbit(nSoftCtrlCardNo, nBitNo, !m_nOutputMark2, nNoteNo);
	if (nRtn != 0)
	{
		m_nOutputMark2 = !m_nOutputMark2;
		UpdateData(FALSE);
		AfxMessageBox("操作IO失败！");
	}
}


void CExternalAxisMoveDlg::OnBnClickedCheck4()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	int nSoftCtrlCardNo = m_nOutput3 / 1000;
	int nNoteNo = (m_nOutput3 % 1000) / 100;
	int nBitNo = m_nOutput3 % 100;
	int nRtn;
	nRtn = m_pcSetServoCtrl->WriteOutbit(nSoftCtrlCardNo, nBitNo, !m_nOutputMark3, nNoteNo);
	if (nRtn != 0)
	{
		m_nOutputMark3 = !m_nOutputMark3;
		UpdateData(FALSE);
		AfxMessageBox("操作IO失败！");
	}
}


void CExternalAxisMoveDlg::OnBnClickedButton16()
{
	// TODO: 在此添加控件通知处理程序代码
	double dAbsCoor = 0.0;
	m_pcSetServoCtrl->OpenAbsEncoder(m_Axis);
	m_pcSetServoCtrl->GetAbsData(m_Axis, dAbsCoor);
	m_pcSetServoCtrl->CloseAbsEncoder(m_Axis);
	m_pcSetServoCtrl->SetPosition(m_Axis, 0);
	SetDlgItemData(IDC_EDIT19, GetStr("%lf", dAbsCoor), this);
	SetDlgItemData(IDC_EDIT18, GetStr("%d", int(dAbsCoor * m_dAxisPluseEquivalent[m_Axis])), this);
}


void CExternalAxisMoveDlg::OnBnClickedButton18()
{
	// TODO: 在此添加控件通知处理程序代码
	if (IDOK != XiMessageBox("确定重置绝对零点么?"))
	{
		return;
	}
	double dAbsCoor = 0.0;
	m_pcSetServoCtrl->OpenAbsEncoder(m_Axis);
	m_pcSetServoCtrl->ClearManyLapData(m_Axis);
	m_pcSetServoCtrl->SetInitLapData(m_Axis);
	m_pcSetServoCtrl->GetAbsData(m_Axis, dAbsCoor);
	m_pcSetServoCtrl->CloseAbsEncoder(m_Axis);
	SetDlgItemData(IDC_EDIT19, GetStr("%lf", dAbsCoor), this);
	SetDlgItemData(IDC_EDIT18, GetStr("%d", int(dAbsCoor * m_dAxisPluseEquivalent[m_Axis])), this);

}


void CExternalAxisMoveDlg::OnBnClickedButton12()
{
	// TODO: 在此添加控件通知处理程序代码
	double dSpeed = 30;
	m_pcSetServoCtrl->ContiMove(m_Axis, 0, dSpeed/*m_dVeloci[m_Axis]*/, 0.5);
}


void CExternalAxisMoveDlg::OnCbnSelchangeCombo5()
{
	// TODO: 在此添加控件通知处理程序代码
	//初始速度，设置为禁止使用
	int nIndex = m_cBox_ChoiceSpeed.GetCurSel();
	if (nIndex>=0&& nIndex<=4)
	{
		m_dVeloci[m_Axis] =(float)( m_Axis <= 2 ? naSpeedTableByRobot[nIndex] : naSpeedTableByZ[nIndex]);
		GetDlgItem(IDC_EDIT4)->SetWindowText(GetStr("%.2lf", m_dVeloci[m_Axis]));
		ShowTransformVelociToMM(m_Axis);
		//GetDlgItem(IDC_EDIT32)->EnableWindow(FALSE);//速度
	}
	nSpeedByFree++;
	int nMAx = 30 + rand() % 40;
	if (nSpeedByFree > nMAx)
	{
		GetDlgItem(IDC_EDIT4)->EnableWindow(TRUE);
		GetDlgItem(IDC_EDIT32)->EnableWindow(TRUE);//速度
	}
	
}
