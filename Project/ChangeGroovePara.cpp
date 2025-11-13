// D:\XiRobotSW\Project\ChangeGroovePara.cpp: 实现文件
//

#include "StdAfx.h"
#include "afxdialogex.h"
#include "ChangeGroovePara.h"


// ChangeGroovePara 对话框

IMPLEMENT_DYNAMIC(ChangeGroovePara, CDialogEx)

ChangeGroovePara::ChangeGroovePara(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_GROOVE_PARA, pParent)
	, m_StartWave(0)
	, m_EndWave(0)
	, m_WaveDistance(0)
	, m_EndSpeedRate(0)
	, m_LeftWaitTime(0)
	, m_RightWaitTime(0)
	, m_MidWaitTime(0)
	, m_VerticalAngle(0)
	, m_NormalOffset(0)
	, m_WeldNo(0)
	, m_HorizontalOffset(0)
	, m_WaveHeight(0)
	, m_WaveUpBase(0)
	, m_ArcType(0)
{
	LoadGroovePara();
	LoadAllGroovePara(vtFlatWavePara, vtVerWavePara);
}

ChangeGroovePara::~ChangeGroovePara()
{
}

BOOL ChangeGroovePara::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	InitCombo1();
	ShowGroovePara();
	//已修改
	XUI::Languge::GetInstance().translateDialog(this);
	return 0;
}

void ChangeGroovePara::InitCombo1()
{
	m_ComboWeldNo.InsertString(0, _T("1"));
	m_ComboWeldNo.InsertString(1, _T("2"));
	m_ComboWeldNo.InsertString(2, _T("3"));
	m_ComboWeldNo.InsertString(3, _T("4"));
	m_ComboWeldNo.InsertString(4, _T("5"));
	m_ComboWeldNo.InsertString(5, _T("6"));
	m_ComboWeldNo.InsertString(6, _T("7"));
	m_ComboWeldNo.InsertString(7, _T("8"));
	m_ComboWeldNo.InsertString(8, _T("9"));
	m_ComboWeldNo.InsertString(9, _T("10"));
	m_ComboWeldNo.InsertString(10, _T("11"));
	m_ComboWeldNo.InsertString(11, _T("12"));
	m_ComboWeldNo.InsertString(12, _T("13"));
	m_ComboWeldNo.InsertString(13, _T("14"));
	m_ComboArcType.InsertString(0, _T("平焊"));
	m_ComboArcType.InsertString(1, _T("立焊"));
}

void ChangeGroovePara::SetGroovePara()
{
	GetDlgItemData(IDC_EDIT1, m_StartWave, this);
	GetDlgItemData(IDC_EDIT2, m_EndWave, this);
	GetDlgItemData(IDC_EDIT4, m_WaveDistance, this);
	GetDlgItemData(IDC_EDIT5, m_EndSpeedRate, this);
	GetDlgItemData(IDC_EDIT6, m_LeftWaitTime, this);
	GetDlgItemData(IDC_EDIT7, m_RightWaitTime, this);
	GetDlgItemData(IDC_EDIT9, m_MidWaitTime, this);
	GetDlgItemData(IDC_EDIT24, m_VerticalAngle, this);
	GetDlgItemData(IDC_EDIT8, m_WaveHeight, this);
	GetDlgItemData(IDC_EDIT11, m_WaveUpBase, this);
	GetDlgItemData(IDC_EDIT37, m_NormalOffset, this);
	GetDlgItemData(IDC_EDIT38, m_HorizontalOffset, this);
	m_WaveType = m_ComboWaveType.GetCurSel();

	m_ArcType = m_ComboArcType.GetCurSel();
	m_WeldNo = m_ComboWeldNo.GetCurSel();
	CString weldName;
	weldName.Format("GroovePara%d", m_WeldNo + 1);
	COPini opini;
	if (0 == m_ArcType)
	{
		opini.SetFileName("ConfigFiles\\FlatGroovePara.ini");
	}
	else
	{
		opini.SetFileName("ConfigFiles\\VerGroovePara.ini");
	}
	/*CString str = str("ConfigFiles\\RobotA\\GroovePara%d.ini", m_WeldNo)*/
	opini.SetSectionName(weldName);
	opini.WriteString("StartWave", m_StartWave);
	opini.WriteString("EndWave", m_EndWave);
	opini.WriteString("WaveDistance", m_WaveDistance);
	opini.WriteString("EndSpeedRate", m_EndSpeedRate);
	opini.WriteString("LeftWaitTime", m_LeftWaitTime);
	opini.WriteString("RightWaitTime", m_RightWaitTime);
	opini.WriteString("MidWaitTime", m_MidWaitTime);
	opini.WriteString("VerticalAngle", m_VerticalAngle);
	opini.WriteString("WaveType", m_WaveType);
	opini.WriteString("WaveHeight", m_WaveHeight);
	opini.WriteString("WaveUpBase", m_WaveUpBase);
	opini.WriteString("NormalOffset", m_NormalOffset);
	opini.WriteString("HorizontalOffset", m_HorizontalOffset);
}

void ChangeGroovePara::LoadGroovePara()
{
	CString weldName;
	weldName.Format("GroovePara%d", m_WeldNo + 1);
	COPini opini;
	if (0 == m_ArcType)
	{
		opini.SetFileName("ConfigFiles\\FlatGroovePara.ini");
	}
	else
	{
		opini.SetFileName("ConfigFiles\\VerGroovePara.ini");
	}
	opini.SetSectionName(weldName);
	opini.ReadString("StartWave", &m_StartWave);
	opini.ReadString("EndWave", &m_EndWave);
	opini.ReadString("WaveDistance", &m_WaveDistance);
	opini.ReadString("EndSpeedRate", &m_EndSpeedRate);
	opini.ReadString("LeftWaitTime", &m_LeftWaitTime);
	opini.ReadString("RightWaitTime", &m_RightWaitTime);
	opini.ReadString("MidWaitTime", &m_MidWaitTime);
	opini.ReadString("VerticalAngle", &m_VerticalAngle);
	opini.ReadString("WaveType", &m_WaveType);
	opini.ReadString("WaveHeight", &m_WaveHeight);
	opini.ReadString("WaveUpBase", &m_WaveUpBase);
	opini.ReadString("NormalOffset", &m_NormalOffset);
	opini.ReadString("HorizontalOffset", &m_HorizontalOffset);
	return;
}

void ChangeGroovePara::ShowGroovePara()
{
	m_ComboWeldNo.SetCurSel(m_WeldNo);
	m_ComboArcType.SetCurSel(m_ArcType);
	SetDlgItemData(IDC_EDIT1, m_StartWave, this);
	SetDlgItemData(IDC_EDIT2, m_EndWave, this);
	SetDlgItemData(IDC_EDIT4, m_WaveDistance, this);
	SetDlgItemData(IDC_EDIT5, m_EndSpeedRate, this);
	SetDlgItemData(IDC_EDIT6, m_LeftWaitTime, this);
	SetDlgItemData(IDC_EDIT7, m_RightWaitTime, this);
	SetDlgItemData(IDC_EDIT9, m_MidWaitTime, this);
	SetDlgItemData(IDC_EDIT24, m_VerticalAngle, this);
	m_ComboWaveType.SetCurSel(m_WaveType);
	SetDlgItemData(IDC_EDIT8, m_WaveHeight, this);
	SetDlgItemData(IDC_EDIT11, m_WaveUpBase, this);
	SetDlgItemData(IDC_EDIT37, m_NormalOffset, this);
	SetDlgItemData(IDC_EDIT38, m_HorizontalOffset, this);
	return;
}

void ChangeGroovePara::LoadAllGroovePara(vector<T_WAVE_PARA>& vtFlatWavePara, vector<T_WAVE_PARA>& vtVerWavePara)
{
	vtFlatWavePara.clear();
	vtVerWavePara.clear();
	T_WAVE_PARA tmpPata;
	for (size_t i = 0; i < 14; i++)
	{
		CString weldName;
		weldName.Format("GroovePara%d", i + 1);
		COPini opini;
		opini.SetFileName("ConfigFiles\\FlatGroovePara.ini");
		opini.SetSectionName(weldName);
		opini.ReadString("StartWave", &tmpPata.dStartWave);
		opini.ReadString("EndWave", &tmpPata.dEndWave);
		opini.ReadString("WaveDistance", &tmpPata.dWaveDistance);
		opini.ReadString("EndSpeedRate", &tmpPata.dEndSpeedRate);
		opini.ReadString("LeftWaitTime", &tmpPata.dLeftWaitTime);
		opini.ReadString("RightWaitTime", &tmpPata.dRightWaitTime);
		opini.ReadString("MidWaitTime", &tmpPata.dMidWaitTime);
		opini.ReadString("VerticalAngle", &tmpPata.dVerticalAngle);
		opini.ReadString("WaveType", &tmpPata.waveType);
		opini.ReadString("WaveHeight", &tmpPata.dWaveHeight);
		opini.ReadString("WaveUpBase", &tmpPata.dWaveUpBase);
		opini.ReadString("NormalOffset", &tmpPata.dNormalOffset);
		opini.ReadString("HorizontalOffset", &tmpPata.dHorizontalOffset);
		vtFlatWavePara.push_back(tmpPata);
	}
	for (size_t i = 0; i < 14; i++)
	{
		CString weldName;
		weldName.Format("GroovePara%d", i + 1);
		COPini opini;
		opini.SetFileName("ConfigFiles\\VerGroovePara.ini");
		opini.SetSectionName(weldName);
		opini.ReadString("StartWave", &tmpPata.dStartWave);
		opini.ReadString("EndWave", &tmpPata.dEndWave);
		opini.ReadString("WaveDistance", &tmpPata.dWaveDistance);
		opini.ReadString("EndSpeedRate", &tmpPata.dEndSpeedRate);
		opini.ReadString("LeftWaitTime", &tmpPata.dLeftWaitTime);
		opini.ReadString("RightWaitTime", &tmpPata.dRightWaitTime);
		opini.ReadString("MidWaitTime", &tmpPata.dMidWaitTime);
		opini.ReadString("VerticalAngle", &tmpPata.dVerticalAngle);
		opini.ReadString("WaveType", &tmpPata.waveType);
		opini.ReadString("WaveHeight", &tmpPata.dWaveHeight);
		opini.ReadString("WaveUpBase", &tmpPata.dWaveUpBase);
		opini.ReadString("NormalOffset", &tmpPata.dNormalOffset);
		opini.ReadString("HorizontalOffset", &tmpPata.dHorizontalOffset);
		vtVerWavePara.push_back(tmpPata);
	}
	return;
}

void ChangeGroovePara::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO1, m_ComboWeldNo);
	DDX_Control(pDX, IDC_COMBO3, m_ComboWaveType);
	DDX_Text(pDX, IDC_EDIT1, m_StartWave);
	DDX_Text(pDX, IDC_EDIT2, m_EndWave);
	DDX_Text(pDX, IDC_EDIT4, m_WaveDistance);
	DDX_Text(pDX, IDC_EDIT5, m_EndSpeedRate);
	DDX_Text(pDX, IDC_EDIT6, m_LeftWaitTime);
	DDX_Text(pDX, IDC_EDIT7, m_RightWaitTime);
	DDX_Text(pDX, IDC_EDIT9, m_MidWaitTime);
	DDX_Text(pDX, IDC_EDIT24, m_VerticalAngle);
	DDX_Text(pDX, IDC_EDIT37, m_NormalOffset);
	DDX_Text(pDX, IDC_EDIT38, m_HorizontalOffset);
	DDX_Text(pDX, IDC_EDIT8, m_WaveHeight);
	DDX_Text(pDX, IDC_EDIT11, m_WaveUpBase);
	DDX_Control(pDX, IDC_COMBO4, m_ComboArcType);
}


BEGIN_MESSAGE_MAP(ChangeGroovePara, CDialogEx)
	ON_BN_CLICKED(IDC_BUTTON1, &ChangeGroovePara::OnBnClickedButton1)
	ON_CBN_SELCHANGE(IDC_COMBO1, &ChangeGroovePara::OnCbnSelchangeCombo1)
	ON_CBN_SELCHANGE(IDC_COMBO4, &ChangeGroovePara::OnCbnSelchangeCombo4)
END_MESSAGE_MAP()


// ChangeGroovePara 消息处理程序


void ChangeGroovePara::OnBnClickedButton1()
{
	// TODO: 在此添加控件通知处理程序代码
	SetGroovePara();
}


void ChangeGroovePara::OnCbnSelchangeCombo1()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_WeldNo = m_ComboWeldNo.GetCurSel();
	m_ArcType = m_ComboArcType.GetCurSel();
	CString weldName;
	weldName.Format("GroovePara%d", m_WeldNo + 1);
	COPini opini;
	if (0 == m_ArcType)
	{
		opini.SetFileName("ConfigFiles\\FlatGroovePara.ini");
	}
	else
	{
		opini.SetFileName("ConfigFiles\\VerGroovePara.ini");
	}
	opini.SetSectionName(weldName);
	opini.ReadString("StartWave", &m_StartWave);
	opini.ReadString("EndWave", &m_EndWave);
	opini.ReadString("WaveDistance", &m_WaveDistance);
	opini.ReadString("EndSpeedRate", &m_EndSpeedRate);
	opini.ReadString("LeftWaitTime", &m_LeftWaitTime);
	opini.ReadString("RightWaitTime", &m_RightWaitTime);
	opini.ReadString("MidWaitTime", &m_MidWaitTime);
	opini.ReadString("VerticalAngle", &m_VerticalAngle);
	opini.ReadString("WaveType", &m_WaveType);
	opini.ReadString("WaveHeight", &m_WaveHeight);
	opini.ReadString("WaveUpBase", &m_WaveUpBase);
	opini.ReadString("NormalOffset", &m_NormalOffset);
	opini.ReadString("HorizontalOffset", &m_HorizontalOffset);
	UpdateData(false);
	ShowGroovePara();
}


void ChangeGroovePara::OnCbnSelchangeCombo4()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_ComboWeldNo.SetCurSel(m_WeldNo);
	m_ArcType = m_ComboArcType.GetCurSel();
	CString weldName;
	weldName.Format("GroovePara%d", m_WeldNo + 1);
	COPini opini;
	if (0 == m_ArcType)
	{
		opini.SetFileName("ConfigFiles\\FlatGroovePara.ini");
	}
	else
	{
		opini.SetFileName("ConfigFiles\\VerGroovePara.ini");
	}
	opini.SetSectionName(weldName);
	opini.ReadString("StartWave", &m_StartWave);
	opini.ReadString("EndWave", &m_EndWave);
	opini.ReadString("WaveDistance", &m_WaveDistance);
	opini.ReadString("EndSpeedRate", &m_EndSpeedRate);
	opini.ReadString("LeftWaitTime", &m_LeftWaitTime);
	opini.ReadString("RightWaitTime", &m_RightWaitTime);
	opini.ReadString("MidWaitTime", &m_MidWaitTime);
	opini.ReadString("VerticalAngle", &m_VerticalAngle);
	opini.ReadString("WaveType", &m_WaveType);
	opini.ReadString("WaveHeight", &m_WaveHeight);
	opini.ReadString("WaveUpBase", &m_WaveUpBase);
	opini.ReadString("NormalOffset", &m_NormalOffset);
	opini.ReadString("HorizontalOffset", &m_HorizontalOffset);
	UpdateData(false);
	ShowGroovePara();
}
