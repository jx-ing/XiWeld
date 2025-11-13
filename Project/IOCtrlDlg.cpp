// IOCtrlDlg.cpp : 实现文件
//
#include "stdafx.h"
#include "res\resource.h"
#include "IOCtrlDlg.h"
#include ".\Apps\PLib\BasicFunc\Const.h"
#include ".\OpenClass\FileOP\ini\opini.h"

// CIOCtrlDlg 对话框

IMPLEMENT_DYNAMIC(CIOCtrlDlg, CDialogEx)

BEGIN_MESSAGE_MAP(CIOCtrlDlg, CDialogEx)
	ON_BN_CLICKED(IDC_TRACK_LASER, &CIOCtrlDlg::OnBnClickedTrackLaser)
	ON_BN_CLICKED(IDC_LINE_SCAN_LASER, &CIOCtrlDlg::OnBnClickedLineScanLaser)
	ON_BN_CLICKED(IDC_MEASURE_LASER, &CIOCtrlDlg::OnBnClickedMeasureLaser)
	ON_BN_CLICKED(IDC_IO_CHECK, &CIOCtrlDlg::OnBnClickedIoCheck)
	ON_CBN_SELCHANGE(IDC_COMBO_ROBOT_CHOOSE, &CIOCtrlDlg::OnCbnSelchangeComboRobotChoose)
	ON_BN_CLICKED(IDC_BUTTON_IO_OPEN, &CIOCtrlDlg::OnBnClickedButtonIoOpen)
	ON_BN_CLICKED(IDC_BUTTON_IO_CLOSE, &CIOCtrlDlg::OnBnClickedButtonIoClose)
END_MESSAGE_MAP()

CIOCtrlDlg::CIOCtrlDlg(std::vector<CUnit*>* pvUnit, CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_CTRL_IO, pParent)
	, m_pvUnit(pvUnit)
{
}

CIOCtrlDlg::~CIOCtrlDlg()
{
	m_pvUnit = NULL;
}

void CIOCtrlDlg::Init()
{
	m_mtShowIOParam.clear();
	std::map<CString, T_IO_PARAM>& mtDeviceIO = (*m_pvUnit)[m_nCurUnitNo]->m_mtDeviceIO;
	std::map<CString, T_IO_PARAM>& mtRobotIO = (*m_pvUnit)[m_nCurUnitNo]->m_mtRobotIO;

	std::map<CString, T_IO_PARAM>::iterator iter = mtDeviceIO.begin();
	m_bComboBoxChooseIO.ResetContent();
	for (; iter != mtDeviceIO.end(); iter++) {
		//已修改
		iter->second.sNameCN = XUI::Languge::GetInstance().translate(iter->second.sNameCN.GetBuffer());
		
		m_bComboBoxChooseIO.AddString(iter->second.sNameCN);
		m_mtShowIOParam[iter->second.sNameCN] = iter->second;
	}
	iter = mtRobotIO.begin();
	for (; iter != mtRobotIO.end(); iter++)
	{
		//已修改
		iter->second.sNameCN = XUI::Languge::GetInstance().translate(iter->second.sNameCN.GetBuffer());
		
		m_bComboBoxChooseIO.AddString(iter->second.sNameCN);
		m_mtShowIOParam[iter->second.sNameCN] = iter->second;
	}
	m_bComboBoxChooseIO.SetCurSel(0);
}

void CIOCtrlDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO_IO_CHOOSE, m_bComboBoxChooseIO);
	DDX_Control(pDX, IDC_COMBO_ROBOT_CHOOSE, m_cComboBoxChooseRobot);
}

BOOL CIOCtrlDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	m_cComboBoxChooseRobot.ResetContent();
	for (int i = 0; i < m_pvUnit->size(); i++)
	{
		//已修改
		m_pvUnit->at(i)->m_tContralUnit.strChineseName = XUI::Languge::GetInstance().translate(m_pvUnit->at(i)->m_tContralUnit.strChineseName.GetBuffer());
		
		m_cComboBoxChooseRobot.AddString(m_pvUnit->at(i)->m_tContralUnit.strChineseName);
		m_cComboBoxChooseRobot.SetCurSel(0);
	}
	Init();
	//已修改
	XUI::Languge::GetInstance().translateDialog(this);
	return TRUE;
}

void CIOCtrlDlg::OnCbnSelchangeComboRobotChoose()
{
	// TODO: 在此添加控件通知处理程序代码
	CString str;
	m_nCurUnitNo = m_cComboBoxChooseRobot.GetCurSel();
	Init();
}

void CIOCtrlDlg::OnBnClickedTrackLaser()
{
	// TODO: 在此添加控件通知处理程序代码	
	UpdateData(true);
	bool bOff_On = BST_CHECKED == ((CButton*)GetDlgItem(IDC_TRACK_LASER))->GetCheck();
	(*m_pvUnit)[m_nCurUnitNo]->SwitchIO("TrackLaser", bOff_On);
}

void CIOCtrlDlg::OnBnClickedLineScanLaser()
{
	UpdateData(true);
	bool bOff_On = BST_CHECKED == ((CButton*)GetDlgItem(IDC_LINE_SCAN_LASER))->GetCheck(); 
	(*m_pvUnit)[m_nCurUnitNo]->SwitchIO("LineScanLaser", bOff_On);
}

void CIOCtrlDlg::OnBnClickedMeasureLaser()
{
	UpdateData(true);
	bool bOff_On = BST_CHECKED == ((CButton*)GetDlgItem(IDC_MEASURE_LASER))->GetCheck();
	(*m_pvUnit)[m_nCurUnitNo]->SwitchIO("MeasureLaser", bOff_On);
}

void CIOCtrlDlg::OnBnClickedButtonIoOpen()
{
	int nRobotNo = m_nCurUnitNo;
	CUnit* pUnit = (*m_pvUnit)[nRobotNo];
	CString str;
	int nIndex = m_bComboBoxChooseIO.GetCurSel();
	m_bComboBoxChooseIO.GetLBText(nIndex, str);
	std::map<CString, T_IO_PARAM>::iterator iter = m_mtShowIOParam.find(str);
	if (iter == m_mtShowIOParam.end())
	{
		XUI::MesBox::PopInfo("打开IO:所选IO名称[{0}]不存在", str);
		return;
	}
	T_IO_PARAM tIOParam = iter->second;

	if (tIOParam.bReadOnly) // 只读(输入) 读取状态 显示
	{
		bool bStatus = pUnit->CheckInIO(tIOParam.sNameEN);
		str.Format("输入信号：%s 状态：%s", tIOParam.sNameCN, bStatus ? "ture" : "false");
		GetDlgItem(IDC_STATIC_HINT)->SetWindowText(str);
	}
	else
	{
		bool bStatus1 = pUnit->CheckOutIO(tIOParam.sNameEN); Sleep(100);
		pUnit->SwitchIO(tIOParam.sNameEN, true); Sleep(2000);
		bool bStatus2 = pUnit->CheckOutIO(tIOParam.sNameEN);
		str.Format("输出信号：%s 状态：%s %s", tIOParam.sNameCN,
			bStatus1 ? "ture" : "false", bStatus2 ? "ture" : "false");
		GetDlgItem(IDC_STATIC_HINT)->SetWindowText(str);
	}
	return;
}

void CIOCtrlDlg::OnBnClickedButtonIoClose()
{
	int nRobotNo = m_nCurUnitNo;
	CUnit* pUnit = (*m_pvUnit)[nRobotNo];
	CString str;
	int nIndex = m_bComboBoxChooseIO.GetCurSel();
	m_bComboBoxChooseIO.GetLBText(nIndex, str);
	std::map<CString, T_IO_PARAM>::iterator iter = m_mtShowIOParam.find(str);
	if (iter == m_mtShowIOParam.end())
	{
		XUI::MesBox::PopInfo("关闭IO:所选IO名称[{0}]不存在", str);
		return;
	}
	T_IO_PARAM tIOParam = iter->second;

	if (tIOParam.bReadOnly) // 只读(输入) 读取状态 显示
	{
		bool bStatus = pUnit->CheckInIO(tIOParam.sNameEN);
		str.Format("输入信号：%s 状态：%s", tIOParam.sNameCN, bStatus ? "ture" : "false");
		GetDlgItem(IDC_STATIC_HINT)->SetWindowText(str);
	}
	else
	{
		bool bStatus1 = pUnit->CheckOutIO(tIOParam.sNameEN); Sleep(100);
		pUnit->SwitchIO(tIOParam.sNameEN, false); Sleep(2000);
		bool bStatus2 = pUnit->CheckOutIO(tIOParam.sNameEN);
		str.Format("输出信号：%s 状态：%s %s", tIOParam.sNameCN,
			bStatus1 ? "ture" : "false", bStatus2 ? "ture" : "false");
		GetDlgItem(IDC_STATIC_HINT)->SetWindowText(str);
	}
	return;
}

void CIOCtrlDlg::OnBnClickedIoCheck()
{
	int nRobotNo = m_nCurUnitNo;
	CUnit* pUnit = (*m_pvUnit)[nRobotNo];
	CString str;
	int nIndex = m_bComboBoxChooseIO.GetCurSel();
	m_bComboBoxChooseIO.GetLBText(nIndex, str);
	std::map<CString, T_IO_PARAM>::iterator iter = m_mtShowIOParam.find(str);
	if (iter == m_mtShowIOParam.end())
	{
		XUI::MesBox::PopInfo("检查IO:所选IO名称[{0}]不存在", str);
		return;
	}
	T_IO_PARAM tIOParam = iter->second;

	if (tIOParam.bReadOnly) // 只读(输入) 读取状态 显示
	{
		bool bStatus = pUnit->CheckInIO(tIOParam.sNameEN);
		str.Format("输入信号：%s 状态：%s", tIOParam.sNameCN, bStatus ? "ture" : "false");
		GetDlgItem(IDC_STATIC_HINT)->SetWindowText(str);
	}
	else
	{
		bool bStatus1 = pUnit->CheckOutIO(tIOParam.sNameEN); Sleep(100);
		pUnit->SwitchIO(tIOParam.sNameEN, true); Sleep(2000);
		bool bStatus2 = pUnit->CheckOutIO(tIOParam.sNameEN); Sleep(100);
		pUnit->SwitchIO(tIOParam.sNameEN, false); Sleep(100);
		bool bStatus3 = pUnit->CheckOutIO(tIOParam.sNameEN);
		str.Format("输出信号：%s 状态：%s %s %s", tIOParam.sNameCN,
			bStatus1 ? "ture" : "false", bStatus2 ? "ture" : "false", bStatus3 ? "ture" : "false");
		GetDlgItem(IDC_STATIC_HINT)->SetWindowText(str);
	}
	return;
}