// WrapAngleParam.cpp : 实现文件
//

#include "stdafx.h"
//#include "XiWeldRobot.h"
#include "res\resource.h"
#include "WrapAngleParam.h"
#include "afxdialogex.h"
#include "OpenClass\FileOP\ini\opini.h"


// CWrapAngleParam 对话框

IMPLEMENT_DYNAMIC(CWrapAngleParam, CDialog)

CWrapAngleParam::CWrapAngleParam(CString sUnitName, CWnd* pParent /*=NULL*/)
	: CDialog(IDD_DIA_WARP_PARAM, pParent)
    , m_strWrapAngleNew(_T(""))
	, m_bWrapType(FALSE)
	, m_bRobotNo(FALSE)
    , m_sUnitName(sUnitName)
{

}

CWrapAngleParam::~CWrapAngleParam()
{
}

void CWrapAngleParam::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBOX_WRAP_TYPE, m_ComboxWrapType);
	DDX_Text(pDX, IDC_EDIT_ADD_WRAP, m_strWrapAngleNew);
	DDX_Check(pDX, IDC_CHECK_WRAP_TYPE, m_bWrapType);
	DDX_Check(pDX, IDC_CHECK_WRAP_ROBOT, m_bRobotNo);
}


BEGIN_MESSAGE_MAP(CWrapAngleParam, CDialog)
    ON_BN_CLICKED(IDOK, &CWrapAngleParam::OnBnClickedOk)
    ON_CBN_SELCHANGE(IDC_COMBOX_WRAP_TYPE, &CWrapAngleParam::OnCbnSelchangeComboxWrapType)
    ON_BN_CLICKED(IDC_BUTTON_DELETE, &CWrapAngleParam::OnBnClickedButtonDelete)
    ON_BN_CLICKED(IDC_BUTTON_ADD, &CWrapAngleParam::OnBnClickedButtonAdd)
	ON_BN_CLICKED(IDC_CHECK_WRAP_TYPE, &CWrapAngleParam::OnBnClickedCheckWrapType)
	ON_BN_CLICKED(IDC_CHECK_WRAP_ROBOT, &CWrapAngleParam::OnBnClickedCheckWrapRobot)
END_MESSAGE_MAP()


// CWrapAngleParam 消息处理程序


void CWrapAngleParam::OnBnClickedOk()
{
    // TODO: 在此添加控件通知处理程序代码
    SaveWrapAngleParam(m_bRobotNo);
    RecordWeldWrapParem();
    CDialog::OnOK();
}
void CWrapAngleParam::SaveWrapAngleParam2(int nRobotNo, T_WELD_WRAP_ANGLE tWeldWrapAngle)
{
    // TODO: 在此添加控件通知处理程序代码
    WELD_WRAP_ANGLE tWrapAngelPara = tWeldWrapAngle.tWeldWrapAngle;
    COPini opini;
    opini.SetFileName(DATA_PATH + m_sUnitName + WRAP_PARAM_FILE);

    opini.SetSectionName("RobotWrapAngle");
    opini.WriteString("dWrapType", tWeldWrapAngle.bWrapType);
    opini.WriteString("strWrapAngleType", tWeldWrapAngle.strWrapName);
    opini.WriteString("dParallel", tWrapAngelPara.m_dWrapdParallel);
    opini.WriteString("dVertical", tWrapAngelPara.m_dWrapdVertical);
    opini.WriteString("dVelocity", tWrapAngelPara.m_dWrapdVelocity);
    opini.WriteString("dRX", tWrapAngelPara.m_dWrapdRX);
    opini.WriteString("dRY", tWrapAngelPara.m_dWrapdRY);
    opini.WriteString("dRZ", tWrapAngelPara.m_dWrapdRZ);
    opini.WriteString("dWaitTime", tWrapAngelPara.m_dWrapdWaitTime);

    opini.WriteString("dParallel2", tWrapAngelPara.m_dWrapdParallel2);
    opini.WriteString("dVertical2", tWrapAngelPara.m_dWrapdVertical2);
    opini.WriteString("dVelocity2", tWrapAngelPara.m_dWrapdVelocity2);
    opini.WriteString("dRX2", tWrapAngelPara.m_dWrapdRX2);
    opini.WriteString("dRY2", tWrapAngelPara.m_dWrapdRY2);
    opini.WriteString("dRZ2", tWrapAngelPara.m_dWrapdRZ2);
    opini.WriteString("dWaitTime2", tWrapAngelPara.m_dWrapdWaitTime2);

    opini.WriteString("dParallel3", tWrapAngelPara.m_dWrapdParallel3);
    opini.WriteString("dVertical3", tWrapAngelPara.m_dWrapdVertical3);
    opini.WriteString("dVelocity3", tWrapAngelPara.m_dWrapdVelocity3);
    opini.WriteString("dRX3", tWrapAngelPara.m_dWrapdRX3);
    opini.WriteString("dRY3", tWrapAngelPara.m_dWrapdRY3);
    opini.WriteString("dRZ3", tWrapAngelPara.m_dWrapdRZ3);
    opini.WriteString("dWaitTime3", tWrapAngelPara.m_dWrapdWaitTime3);

    opini.WriteString("dParallel4", tWrapAngelPara.m_dWrapdParallel4);
    opini.WriteString("dVertical4", tWrapAngelPara.m_dWrapdVertical4);
    opini.WriteString("dVelocity4", tWrapAngelPara.m_dWrapdVelocity4);
    opini.WriteString("dRX4", tWrapAngelPara.m_dWrapdRX4);
    opini.WriteString("dRY4", tWrapAngelPara.m_dWrapdRY4);
    opini.WriteString("dRZ4", tWrapAngelPara.m_dWrapdRZ4);
    opini.WriteString("dWaitTime4", tWrapAngelPara.m_dWrapdWaitTime4);

    opini.WriteString("dParalle5", tWrapAngelPara.m_dWrapdParallel5);
    opini.WriteString("dVertica5", tWrapAngelPara.m_dWrapdVertical5);
    opini.WriteString("dVelocit5", tWrapAngelPara.m_dWrapdVelocity5);
    opini.WriteString("dRX5", tWrapAngelPara.m_dWrapdRX5);
    opini.WriteString("dRY5", tWrapAngelPara.m_dWrapdRY5);
    opini.WriteString("dRZ5", tWrapAngelPara.m_dWrapdRZ5);
    opini.WriteString("dWaitTime5", tWrapAngelPara.m_dWrapdWaitTime5);

    opini.WriteString("dParallel6", tWrapAngelPara.m_dWrapdParallel6);
    opini.WriteString("dVertical6", tWrapAngelPara.m_dWrapdVertical6);
    opini.WriteString("dVelocity6", tWrapAngelPara.m_dWrapdVelocity6);
    opini.WriteString("dRX6", tWrapAngelPara.m_dWrapdRX6);
    opini.WriteString("dRY6", tWrapAngelPara.m_dWrapdRY6);
    opini.WriteString("dRZ6", tWrapAngelPara.m_dWrapdRZ6);
    opini.WriteString("dWaitTime6", tWrapAngelPara.m_dWrapdWaitTime6);

    opini.WriteString("dParallel7", tWrapAngelPara.m_dWrapdParallel7);
    opini.WriteString("dVertical7", tWrapAngelPara.m_dWrapdVertical7);
    opini.WriteString("dVelocity7", tWrapAngelPara.m_dWrapdVelocity7);
    opini.WriteString("dRX7", tWrapAngelPara.m_dWrapdRX7);
    opini.WriteString("dRY7", tWrapAngelPara.m_dWrapdRY7);
    opini.WriteString("dRZ7", tWrapAngelPara.m_dWrapdRZ7);
    opini.WriteString("dWaitTime7", tWrapAngelPara.m_dWrapdWaitTime7);

}
void CWrapAngleParam:: SaveWrapAngleParam(int nRobotNo)
{
    // TODO: 在此添加控件通知处理程序代码
    WELD_WRAP_ANGLE tWrapAngelPara = DownloadWrapAngleParam();
    COPini opini;
    opini.SetFileName(DATA_PATH + m_sUnitName + WRAP_PARAM_FILE);
	
    opini.SetSectionName("RobotWrapAngle");
	opini.WriteString("dWrapType", m_bWrapType);
    opini.WriteString("strWrapAngleType", m_strWrapAngleType);
    opini.WriteString("dParallel", tWrapAngelPara.m_dWrapdParallel);
    opini.WriteString("dVertical", tWrapAngelPara.m_dWrapdVertical);
    opini.WriteString("dVelocity", tWrapAngelPara.m_dWrapdVelocity);
    opini.WriteString("dRX", tWrapAngelPara.m_dWrapdRX);
    opini.WriteString("dRY", tWrapAngelPara.m_dWrapdRY);
    opini.WriteString("dRZ", tWrapAngelPara.m_dWrapdRZ);
    opini.WriteString("dWaitTime", tWrapAngelPara.m_dWrapdWaitTime);

    opini.WriteString("dParallel2", tWrapAngelPara.m_dWrapdParallel2);
    opini.WriteString("dVertical2", tWrapAngelPara.m_dWrapdVertical2);
    opini.WriteString("dVelocity2", tWrapAngelPara.m_dWrapdVelocity2);
    opini.WriteString("dRX2", tWrapAngelPara.m_dWrapdRX2);
    opini.WriteString("dRY2", tWrapAngelPara.m_dWrapdRY2);
    opini.WriteString("dRZ2", tWrapAngelPara.m_dWrapdRZ2);
    opini.WriteString("dWaitTime2", tWrapAngelPara.m_dWrapdWaitTime2);

    opini.WriteString("dParallel3", tWrapAngelPara.m_dWrapdParallel3);
    opini.WriteString("dVertical3", tWrapAngelPara.m_dWrapdVertical3);
    opini.WriteString("dVelocity3", tWrapAngelPara.m_dWrapdVelocity3);
    opini.WriteString("dRX3", tWrapAngelPara.m_dWrapdRX3);
    opini.WriteString("dRY3", tWrapAngelPara.m_dWrapdRY3);
    opini.WriteString("dRZ3", tWrapAngelPara.m_dWrapdRZ3);
    opini.WriteString("dWaitTime3", tWrapAngelPara.m_dWrapdWaitTime3);

    opini.WriteString("dParallel4", tWrapAngelPara.m_dWrapdParallel4);
    opini.WriteString("dVertical4", tWrapAngelPara.m_dWrapdVertical4);
    opini.WriteString("dVelocity4", tWrapAngelPara.m_dWrapdVelocity4);
    opini.WriteString("dRX4", tWrapAngelPara.m_dWrapdRX4);
    opini.WriteString("dRY4", tWrapAngelPara.m_dWrapdRY4);
    opini.WriteString("dRZ4", tWrapAngelPara.m_dWrapdRZ4);
    opini.WriteString("dWaitTime4", tWrapAngelPara.m_dWrapdWaitTime4);

    opini.WriteString("dParalle5", tWrapAngelPara.m_dWrapdParallel5);
    opini.WriteString("dVertica5", tWrapAngelPara.m_dWrapdVertical5);
    opini.WriteString("dVelocit5", tWrapAngelPara.m_dWrapdVelocity5);
    opini.WriteString("dRX5", tWrapAngelPara.m_dWrapdRX5);
    opini.WriteString("dRY5", tWrapAngelPara.m_dWrapdRY5);
    opini.WriteString("dRZ5", tWrapAngelPara.m_dWrapdRZ5);
    opini.WriteString("dWaitTime5", tWrapAngelPara.m_dWrapdWaitTime5);

    opini.WriteString("dParallel6", tWrapAngelPara.m_dWrapdParallel6);
    opini.WriteString("dVertical6", tWrapAngelPara.m_dWrapdVertical6);
    opini.WriteString("dVelocity6", tWrapAngelPara.m_dWrapdVelocity6);
    opini.WriteString("dRX6", tWrapAngelPara.m_dWrapdRX6);
    opini.WriteString("dRY6", tWrapAngelPara.m_dWrapdRY6);
    opini.WriteString("dRZ6", tWrapAngelPara.m_dWrapdRZ6);
    opini.WriteString("dWaitTime6", tWrapAngelPara.m_dWrapdWaitTime6);

    opini.WriteString("dParallel7", tWrapAngelPara.m_dWrapdParallel7);
    opini.WriteString("dVertical7", tWrapAngelPara.m_dWrapdVertical7);
    opini.WriteString("dVelocity7", tWrapAngelPara.m_dWrapdVelocity7);
    opini.WriteString("dRX7", tWrapAngelPara.m_dWrapdRX7);
    opini.WriteString("dRY7", tWrapAngelPara.m_dWrapdRY7);
    opini.WriteString("dRZ7", tWrapAngelPara.m_dWrapdRZ7);
    opini.WriteString("dWaitTime7", tWrapAngelPara.m_dWrapdWaitTime7);
}
WELD_WRAP_ANGLE CWrapAngleParam::InitWrapAngleParam(int nRobotNo)
{
    bool bIfWindowOn = true;
    WELD_WRAP_ANGLE tWrapAngelPara;
    COPini opini;
    opini.SetFileName(DATA_PATH + m_sUnitName + WRAP_PARAM_FILE);

	opini.SetSectionName("RobotWrapAngle");
	opini.ReadString("dWrapType", &m_bWrapType);
    opini.ReadString("strWrapAngleType", m_strWrapAngleType);
    opini.ReadString("dParallel", &tWrapAngelPara.m_dWrapdParallel);
    opini.ReadString("dVertical", &tWrapAngelPara.m_dWrapdVertical);
    opini.ReadString("dVelocity", &tWrapAngelPara.m_dWrapdVelocity);
    opini.ReadString("dRX", &tWrapAngelPara.m_dWrapdRX);
    opini.ReadString("dRY", &tWrapAngelPara.m_dWrapdRY);
    opini.ReadString("dRZ", &tWrapAngelPara.m_dWrapdRZ);
    opini.ReadString("dWaitTime", &tWrapAngelPara.m_dWrapdWaitTime);

    opini.ReadString("dParallel2", &tWrapAngelPara.m_dWrapdParallel2);
    opini.ReadString("dVertical2", &tWrapAngelPara.m_dWrapdVertical2);
    opini.ReadString("dVelocity2", &tWrapAngelPara.m_dWrapdVelocity2);
    opini.ReadString("dRX2", &tWrapAngelPara.m_dWrapdRX2);
    opini.ReadString("dRY2", &tWrapAngelPara.m_dWrapdRY2);
    opini.ReadString("dRZ2", &tWrapAngelPara.m_dWrapdRZ2);
    opini.ReadString("dWaitTime2", &tWrapAngelPara.m_dWrapdWaitTime2);

    opini.ReadString("dParallel3", &tWrapAngelPara.m_dWrapdParallel3);
    opini.ReadString("dVertical3", &tWrapAngelPara.m_dWrapdVertical3);
    opini.ReadString("dVelocity3", &tWrapAngelPara.m_dWrapdVelocity3);
    opini.ReadString("dRX3", &tWrapAngelPara.m_dWrapdRX3);
    opini.ReadString("dRY3", &tWrapAngelPara.m_dWrapdRY3);
    opini.ReadString("dRZ3", &tWrapAngelPara.m_dWrapdRZ3);
    opini.ReadString("dWaitTime3", &tWrapAngelPara.m_dWrapdWaitTime3);

    opini.ReadString("dParallel4", &tWrapAngelPara.m_dWrapdParallel4);
    opini.ReadString("dVertical4", &tWrapAngelPara.m_dWrapdVertical4);
    opini.ReadString("dVelocity4", &tWrapAngelPara.m_dWrapdVelocity4);
    opini.ReadString("dRX4", &tWrapAngelPara.m_dWrapdRX4);
    opini.ReadString("dRY4", &tWrapAngelPara.m_dWrapdRY4);
    opini.ReadString("dRZ4", &tWrapAngelPara.m_dWrapdRZ4);
    opini.ReadString("dWaitTime4", &tWrapAngelPara.m_dWrapdWaitTime4);

    opini.ReadString("dParalle5", &tWrapAngelPara.m_dWrapdParallel5);
    opini.ReadString("dVertica5", &tWrapAngelPara.m_dWrapdVertical5);
    opini.ReadString("dVelocit5", &tWrapAngelPara.m_dWrapdVelocity5);
    opini.ReadString("dRX5", &tWrapAngelPara.m_dWrapdRX5);
    opini.ReadString("dRY5", &tWrapAngelPara.m_dWrapdRY5);
    opini.ReadString("dRZ5", &tWrapAngelPara.m_dWrapdRZ5);
    opini.ReadString("dWaitTime5", &tWrapAngelPara.m_dWrapdWaitTime5);

    opini.ReadString("dParallel6", &tWrapAngelPara.m_dWrapdParallel6);
    opini.ReadString("dVertical6", &tWrapAngelPara.m_dWrapdVertical6);
    opini.ReadString("dVelocity6", &tWrapAngelPara.m_dWrapdVelocity6);
    opini.ReadString("dRX6", &tWrapAngelPara.m_dWrapdRX6);
    opini.ReadString("dRY6", &tWrapAngelPara.m_dWrapdRY6);
    opini.ReadString("dRZ6", &tWrapAngelPara.m_dWrapdRZ6);
    opini.ReadString("dWaitTime6", &tWrapAngelPara.m_dWrapdWaitTime6);

    opini.ReadString("dParallel7", &tWrapAngelPara.m_dWrapdParallel7);
    opini.ReadString("dVertical7", &tWrapAngelPara.m_dWrapdVertical7);
    opini.ReadString("dVelocity7", &tWrapAngelPara.m_dWrapdVelocity7);
    opini.ReadString("dRX7", &tWrapAngelPara.m_dWrapdRX7);
    opini.ReadString("dRY7", &tWrapAngelPara.m_dWrapdRY7);
    opini.ReadString("dRZ7", &tWrapAngelPara.m_dWrapdRZ7);
    opini.ReadString("dWaitTime7", &tWrapAngelPara.m_dWrapdWaitTime7);	
    return tWrapAngelPara;   
	
}
WELD_WRAP_ANGLE CWrapAngleParam::DownloadWrapAngleParam()
{
    UpdateData(true);
    WELD_WRAP_ANGLE tWrapAngelPara;
    bool bIfWindowOn = true;
    GetDlgItemData(IDC_EDIT_STEP1_PARALLEL, tWrapAngelPara.m_dWrapdParallel, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_VERTICAL, tWrapAngelPara.m_dWrapdVertical, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_VELOCITY, tWrapAngelPara.m_dWrapdVelocity, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RX, tWrapAngelPara.m_dWrapdRX, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RY, tWrapAngelPara.m_dWrapdRY, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RZ, tWrapAngelPara.m_dWrapdRZ, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_WAITTIME, tWrapAngelPara.m_dWrapdWaitTime, this, bIfWindowOn);

    GetDlgItemData(IDC_EDIT_STEP1_PARALLEL2, tWrapAngelPara.m_dWrapdParallel2, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_VERTICAL2, tWrapAngelPara.m_dWrapdVertical2, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_VELOCITY2, tWrapAngelPara.m_dWrapdVelocity2, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RX2, tWrapAngelPara.m_dWrapdRX2, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RY2, tWrapAngelPara.m_dWrapdRY2, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RZ2, tWrapAngelPara.m_dWrapdRZ2, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_WAITTIME2, tWrapAngelPara.m_dWrapdWaitTime2, this, bIfWindowOn);

    GetDlgItemData(IDC_EDIT_STEP1_PARALLEL3, tWrapAngelPara.m_dWrapdParallel3, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_VERTICAL3, tWrapAngelPara.m_dWrapdVertical3, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_VELOCITY3, tWrapAngelPara.m_dWrapdVelocity3, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RX3, tWrapAngelPara.m_dWrapdRX3, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RY3, tWrapAngelPara.m_dWrapdRY3, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RZ3, tWrapAngelPara.m_dWrapdRZ3, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_WAITTIME3, tWrapAngelPara.m_dWrapdWaitTime3, this, bIfWindowOn);

    GetDlgItemData(IDC_EDIT_STEP1_PARALLEL4, tWrapAngelPara.m_dWrapdParallel4, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_VERTICAL4, tWrapAngelPara.m_dWrapdVertical4, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_VELOCITY4, tWrapAngelPara.m_dWrapdVelocity4, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RX4, tWrapAngelPara.m_dWrapdRX4, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RY4, tWrapAngelPara.m_dWrapdRY4, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RZ4, tWrapAngelPara.m_dWrapdRZ4, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_WAITTIME4, tWrapAngelPara.m_dWrapdWaitTime4, this, bIfWindowOn);

    GetDlgItemData(IDC_EDIT_STEP1_PARALLEL5, tWrapAngelPara.m_dWrapdParallel5, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_VERTICAL5, tWrapAngelPara.m_dWrapdVertical5, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_VELOCITY5, tWrapAngelPara.m_dWrapdVelocity5, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RX5, tWrapAngelPara.m_dWrapdRX5, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RY5, tWrapAngelPara.m_dWrapdRY5, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RZ5, tWrapAngelPara.m_dWrapdRZ5, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_WAITTIME5, tWrapAngelPara.m_dWrapdWaitTime5, this, bIfWindowOn);

    GetDlgItemData(IDC_EDIT_STEP1_PARALLEL6, tWrapAngelPara.m_dWrapdParallel6, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_VERTICAL6, tWrapAngelPara.m_dWrapdVertical6, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_VELOCITY6, tWrapAngelPara.m_dWrapdVelocity6, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RX6, tWrapAngelPara.m_dWrapdRX6, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RY6, tWrapAngelPara.m_dWrapdRY6, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RZ6, tWrapAngelPara.m_dWrapdRZ6, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_WAITTIME6, tWrapAngelPara.m_dWrapdWaitTime6, this, bIfWindowOn);

    GetDlgItemData(IDC_EDIT_STEP1_PARALLEL7, tWrapAngelPara.m_dWrapdParallel7, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_VERTICAL7, tWrapAngelPara.m_dWrapdVertical7, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_VELOCITY7, tWrapAngelPara.m_dWrapdVelocity7, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RX7, tWrapAngelPara.m_dWrapdRX7, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RY7, tWrapAngelPara.m_dWrapdRY7, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_RZ7, tWrapAngelPara.m_dWrapdRZ7, this, bIfWindowOn);
    GetDlgItemData(IDC_EDIT_STEP1_WAITTIME7, tWrapAngelPara.m_dWrapdWaitTime7, this, bIfWindowOn);
    return tWrapAngelPara;
}
void CWrapAngleParam::RefreshWrapAngleParam(WELD_WRAP_ANGLE tWrapAngelPara)
{   
    bool bIfWindowOn = true;
    SetDlgItemData(IDC_EDIT_STEP1_PARALLEL, tWrapAngelPara.m_dWrapdParallel, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_VERTICAL, tWrapAngelPara.m_dWrapdVertical, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_VELOCITY, tWrapAngelPara.m_dWrapdVelocity, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RX, tWrapAngelPara.m_dWrapdRX, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RY, tWrapAngelPara.m_dWrapdRY, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RZ, tWrapAngelPara.m_dWrapdRZ, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_WAITTIME, tWrapAngelPara.m_dWrapdWaitTime, this, bIfWindowOn);

    SetDlgItemData(IDC_EDIT_STEP1_PARALLEL2, tWrapAngelPara.m_dWrapdParallel2, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_VERTICAL2, tWrapAngelPara.m_dWrapdVertical2, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_VELOCITY2, tWrapAngelPara.m_dWrapdVelocity2, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RX2, tWrapAngelPara.m_dWrapdRX2, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RY2, tWrapAngelPara.m_dWrapdRY2, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RZ2, tWrapAngelPara.m_dWrapdRZ2, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_WAITTIME2, tWrapAngelPara.m_dWrapdWaitTime2, this, bIfWindowOn);

    SetDlgItemData(IDC_EDIT_STEP1_PARALLEL3, tWrapAngelPara.m_dWrapdParallel3, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_VERTICAL3, tWrapAngelPara.m_dWrapdVertical3, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_VELOCITY3, tWrapAngelPara.m_dWrapdVelocity3, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RX3, tWrapAngelPara.m_dWrapdRX3, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RY3, tWrapAngelPara.m_dWrapdRY3, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RZ3, tWrapAngelPara.m_dWrapdRZ3, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_WAITTIME3, tWrapAngelPara.m_dWrapdWaitTime3, this, bIfWindowOn);

    SetDlgItemData(IDC_EDIT_STEP1_PARALLEL4, tWrapAngelPara.m_dWrapdParallel4, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_VERTICAL4, tWrapAngelPara.m_dWrapdVertical4, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_VELOCITY4, tWrapAngelPara.m_dWrapdVelocity4, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RX4, tWrapAngelPara.m_dWrapdRX4, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RY4, tWrapAngelPara.m_dWrapdRY4, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RZ4, tWrapAngelPara.m_dWrapdRZ4, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_WAITTIME4, tWrapAngelPara.m_dWrapdWaitTime4, this, bIfWindowOn);

    SetDlgItemData(IDC_EDIT_STEP1_PARALLEL5, tWrapAngelPara.m_dWrapdParallel5, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_VERTICAL5, tWrapAngelPara.m_dWrapdVertical5, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_VELOCITY5, tWrapAngelPara.m_dWrapdVelocity5, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RX5, tWrapAngelPara.m_dWrapdRX5, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RY5, tWrapAngelPara.m_dWrapdRY5, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RZ5, tWrapAngelPara.m_dWrapdRZ5, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_WAITTIME5, tWrapAngelPara.m_dWrapdWaitTime5, this, bIfWindowOn);

    SetDlgItemData(IDC_EDIT_STEP1_PARALLEL6, tWrapAngelPara.m_dWrapdParallel6, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_VERTICAL6, tWrapAngelPara.m_dWrapdVertical6, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_VELOCITY6, tWrapAngelPara.m_dWrapdVelocity6, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RX6, tWrapAngelPara.m_dWrapdRX6, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RY6, tWrapAngelPara.m_dWrapdRY6, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RZ6, tWrapAngelPara.m_dWrapdRZ6, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_WAITTIME6, tWrapAngelPara.m_dWrapdWaitTime6, this, bIfWindowOn);

    SetDlgItemData(IDC_EDIT_STEP1_PARALLEL7, tWrapAngelPara.m_dWrapdParallel7, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_VERTICAL7, tWrapAngelPara.m_dWrapdVertical7, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_VELOCITY7, tWrapAngelPara.m_dWrapdVelocity7, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RX7, tWrapAngelPara.m_dWrapdRX7, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RY7, tWrapAngelPara.m_dWrapdRY7, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_RZ7, tWrapAngelPara.m_dWrapdRZ7, this, bIfWindowOn);
    SetDlgItemData(IDC_EDIT_STEP1_WAITTIME7, tWrapAngelPara.m_dWrapdWaitTime7, this, bIfWindowOn);
}

void CWrapAngleParam::OnCbnSelchangeComboxWrapType()
{
    UpdateData(TRUE);
    WELD_WRAP_ANGLE tWrapAngelPara;
    int nIndex = m_ComboxWrapType.GetCurSel();
    CString str;
    m_ComboxWrapType.GetLBText(nIndex, str);
    m_strWrapAngleType = str;
    for (int i = 0; i < m_vtWrapAngel.size(); i++)
    {
        if (str == m_vtWrapAngel.at(i).strWrapName)
        {
			m_bWrapType = m_vtWrapAngel.at(i).bWrapType;
            tWrapAngelPara = m_vtWrapAngel.at(i).tWeldWrapAngle;
            break;
        }
    }
    RefreshWrapAngleParam(tWrapAngelPara);
	if (m_bWrapType)
	{
		GetDlgItem(IDC_CHECK_WRAP_TYPE)->SetWindowText("一次成型");
	}
	else
	{
		GetDlgItem(IDC_CHECK_WRAP_TYPE)->SetWindowText("多次成型");
	}
	UpdateData(FALSE);
}


BOOL CWrapAngleParam::OnInitDialog()
{
    CDialog::OnInitDialog();

    // TODO:  在此添加额外的初始化
	m_bRobotNo = 0;
    WELD_WRAP_ANGLE tWrapAngelPara = InitWrapAngleParam();
    RefreshWrapAngleParam(tWrapAngelPara);
    ReadWeldWrapParem();
    int nSetNo = 0;
    //刷新界面
    m_ComboxWrapType.ResetContent();
    for (int i = 0; i < m_vtWrapAngel.size(); i++)
    {
        m_ComboxWrapType.AddString(m_vtWrapAngel.at(i).strWrapName);
        if (m_strWrapAngleType == m_vtWrapAngel.at(i).strWrapName)
        {
            nSetNo = i;
        }
    }
    m_ComboxWrapType.SetCurSel(nSetNo);
	if (m_bWrapType)
	{
		GetDlgItem(IDC_CHECK_WRAP_TYPE)->SetWindowText("一次成型");
	}
	else
	{
		GetDlgItem(IDC_CHECK_WRAP_TYPE)->SetWindowText("多次成型");
	}
	UpdateData(FALSE);
	//已修改
	XUI::Languge::GetInstance().translateDialog(this);
    return TRUE;  // return TRUE unless you set the focus to a control
                  // 异常: OCX 属性页应返回 FALSE
}



T_WELD_WRAP_ANGLE CWrapAngleParam::AutoSelectWrapParem(int nRobotNo, double dWorkpieceThink,int nWrapType)
{
    WriteLog("所要选择的包角厚度：%lf 类型：%d 次成型", dWorkpieceThink, nWrapType);
    T_WELD_WRAP_ANGLE tWeldWrapAngle;
	if (2 == nWrapType)
	{
		nWrapType = 0;
	}
    ReadWeldWrapParem();
    CString str;
    int nSize = m_vtWrapAngel.size();
    int i = 0;
    for (i = 0; i < nSize; i++)
    {
        str = m_vtWrapAngel.at(i).strWrapName;
        int nType = m_vtWrapAngel.at(i).bWrapType;
        //摘取字符串
        int n = str.Find('-');
        str = str.Left(n);
        double dThink = atof(str);
        if (fabs(dWorkpieceThink-dThink)<0.6&& nWrapType == nType)
        {
            if (nType == 0)//多次包角
            {
                m_vtWrapAngel.at(i).tWeldWrapAngle.m_dWrapdVertical6 = dWorkpieceThink;
            }
            else
            {
                m_vtWrapAngel.at(i).tWeldWrapAngle.m_dWrapdVertical4 = dWorkpieceThink;
            }
            tWeldWrapAngle = m_vtWrapAngel.at(i);
            SaveWrapAngleParam2(nRobotNo, m_vtWrapAngel.at(i));
            WriteLog("实际选择的包角厚度：%s %lf 类型：%d 次成型", str, dThink, nType);
            break;
        }
    }

    if (i == nSize)
    {
        XiMessageBox("未匹配到合适的包角参数，检查后再次使用");
    }
    return tWeldWrapAngle;
}
void CWrapAngleParam::ReadWeldWrapParem()
{  
    m_vtWrapAngel.clear();
    WELD_WRAP_ANGLE tWrapAngelPara; 
    int nTotalNum = 0;
    FILE *ReadParam;

    CString sWrapAngleLibraryFile =  DATA_PATH + m_sUnitName + WRAP_LIBRARY_FILE;
    if (0 == CheckFileExists(sWrapAngleLibraryFile))
    {
        XUI::MesBox::PopError("{0}文件不存在,请检查", sWrapAngleLibraryFile.GetBuffer());
    }
    ReadParam = fopen(sWrapAngleLibraryFile.GetBuffer(), "r");
    fscanf(ReadParam, "TotalNum:%d", &nTotalNum);
    for (int i = 0; i < nTotalNum; i++)
    {
        char nName[100];
        int nTemp = 0;
        T_WELD_WRAP_ANGLE  tWrapAngel; 
        //fscanf(ReadParam, "%s\n", tWrapAngel.strWrapName);
        fscanf(ReadParam, "%s\n", nName);
		fscanf(ReadParam, "%d\n", &nTemp);
        tWrapAngel.bWrapType = nTemp > 0;
        fscanf(ReadParam, "%lf %lf %lf %lf %lf %lf %lf", &tWrapAngelPara.m_dWrapdParallel, &tWrapAngelPara.m_dWrapdVertical,
            &tWrapAngelPara.m_dWrapdVelocity, &tWrapAngelPara.m_dWrapdRX, &tWrapAngelPara.m_dWrapdRY, &tWrapAngelPara.m_dWrapdRZ, &tWrapAngelPara.m_dWrapdWaitTime);
        fscanf(ReadParam, "%lf %lf %lf %lf %lf %lf %lf", &tWrapAngelPara.m_dWrapdParallel2, &tWrapAngelPara.m_dWrapdVertical2,
            &tWrapAngelPara.m_dWrapdVelocity2, &tWrapAngelPara.m_dWrapdRX2, &tWrapAngelPara.m_dWrapdRY2, &tWrapAngelPara.m_dWrapdRZ2, &tWrapAngelPara.m_dWrapdWaitTime2);
        fscanf(ReadParam, "%lf %lf %lf %lf %lf %lf %lf", &tWrapAngelPara.m_dWrapdParallel3, &tWrapAngelPara.m_dWrapdVertical3,
            &tWrapAngelPara.m_dWrapdVelocity3, &tWrapAngelPara.m_dWrapdRX3, &tWrapAngelPara.m_dWrapdRY3, &tWrapAngelPara.m_dWrapdRZ3, &tWrapAngelPara.m_dWrapdWaitTime3);
        fscanf(ReadParam, "%lf %lf %lf %lf %lf %lf %lf", &tWrapAngelPara.m_dWrapdParallel4, &tWrapAngelPara.m_dWrapdVertical4,
            &tWrapAngelPara.m_dWrapdVelocity4, &tWrapAngelPara.m_dWrapdRX4, &tWrapAngelPara.m_dWrapdRY4, &tWrapAngelPara.m_dWrapdRZ4, &tWrapAngelPara.m_dWrapdWaitTime4);
        fscanf(ReadParam, "%lf %lf %lf %lf %lf %lf %lf", &tWrapAngelPara.m_dWrapdParallel5, &tWrapAngelPara.m_dWrapdVertical5,
            &tWrapAngelPara.m_dWrapdVelocity5, &tWrapAngelPara.m_dWrapdRX5, &tWrapAngelPara.m_dWrapdRY5, &tWrapAngelPara.m_dWrapdRZ5, &tWrapAngelPara.m_dWrapdWaitTime5);
        fscanf(ReadParam, "%lf %lf %lf %lf %lf %lf %lf", &tWrapAngelPara.m_dWrapdParallel6, &tWrapAngelPara.m_dWrapdVertical6,
            &tWrapAngelPara.m_dWrapdVelocity6, &tWrapAngelPara.m_dWrapdRX6, &tWrapAngelPara.m_dWrapdRY6, &tWrapAngelPara.m_dWrapdRZ6, &tWrapAngelPara.m_dWrapdWaitTime6);
        fscanf(ReadParam, "%lf %lf %lf %lf %lf %lf %lf", &tWrapAngelPara.m_dWrapdParallel7, &tWrapAngelPara.m_dWrapdVertical7,
            &tWrapAngelPara.m_dWrapdVelocity7, &tWrapAngelPara.m_dWrapdRX7, &tWrapAngelPara.m_dWrapdRY7, &tWrapAngelPara.m_dWrapdRZ7, &tWrapAngelPara.m_dWrapdWaitTime7);
        tWrapAngel.strWrapName.Format("%s", nName);
        tWrapAngel.tWeldWrapAngle = tWrapAngelPara;
        m_vtWrapAngel.push_back(tWrapAngel);
    }
    fclose(ReadParam);
}

void CWrapAngleParam::RecordWeldWrapParem()
{
    WELD_WRAP_ANGLE tWrapAngelPara;
    T_WELD_WRAP_ANGLE  tWrapAngel;
    
    int nTotalNum = m_vtWrapAngel.size();
    FILE *ReadParam;
    CString sWrapAngleLibraryFile = DATA_PATH + m_sUnitName + WRAP_LIBRARY_FILE;
    if (0 == CheckFileExists(sWrapAngleLibraryFile, true))
    {
        XUI::MesBox::PopError("{0}文件不存在，但已创建", sWrapAngleLibraryFile.GetBuffer());
    }
    ReadParam = fopen(sWrapAngleLibraryFile.GetBuffer(), "w");
    fprintf(ReadParam, "TotalNum:%d\n", nTotalNum);
    for (int i = 0; i < nTotalNum; i++)
    {
        CString strWrapName;
		bool bWrapType = m_vtWrapAngel.at(i).bWrapType;
        strWrapName = m_vtWrapAngel.at(i).strWrapName;
        tWrapAngelPara = m_vtWrapAngel.at(i).tWeldWrapAngle;
        fprintf(ReadParam, strWrapName + "\n");
		fprintf(ReadParam, "%d\n", bWrapType);
        fprintf(ReadParam, "%lf %lf %lf %lf %lf %lf %lf\n", tWrapAngelPara.m_dWrapdParallel, tWrapAngelPara.m_dWrapdVertical,
            tWrapAngelPara.m_dWrapdVelocity, tWrapAngelPara.m_dWrapdRX, tWrapAngelPara.m_dWrapdRY, tWrapAngelPara.m_dWrapdRZ, tWrapAngelPara.m_dWrapdWaitTime);
        fprintf(ReadParam, "%lf %lf %lf %lf %lf %lf %lf\n", tWrapAngelPara.m_dWrapdParallel2, tWrapAngelPara.m_dWrapdVertical2,
            tWrapAngelPara.m_dWrapdVelocity2, tWrapAngelPara.m_dWrapdRX2, tWrapAngelPara.m_dWrapdRY2, tWrapAngelPara.m_dWrapdRZ2, tWrapAngelPara.m_dWrapdWaitTime2);
        fprintf(ReadParam, "%lf %lf %lf %lf %lf %lf %lf\n", tWrapAngelPara.m_dWrapdParallel3, tWrapAngelPara.m_dWrapdVertical3,
            tWrapAngelPara.m_dWrapdVelocity3, tWrapAngelPara.m_dWrapdRX3, tWrapAngelPara.m_dWrapdRY3, tWrapAngelPara.m_dWrapdRZ3, tWrapAngelPara.m_dWrapdWaitTime3);
        fprintf(ReadParam, "%lf %lf %lf %lf %lf %lf %lf\n", tWrapAngelPara.m_dWrapdParallel4, tWrapAngelPara.m_dWrapdVertical4,
            tWrapAngelPara.m_dWrapdVelocity4, tWrapAngelPara.m_dWrapdRX4, tWrapAngelPara.m_dWrapdRY4, tWrapAngelPara.m_dWrapdRZ4, tWrapAngelPara.m_dWrapdWaitTime4);
        fprintf(ReadParam, "%lf %lf %lf %lf %lf %lf %lf\n", tWrapAngelPara.m_dWrapdParallel5, tWrapAngelPara.m_dWrapdVertical5,
            tWrapAngelPara.m_dWrapdVelocity5, tWrapAngelPara.m_dWrapdRX5, tWrapAngelPara.m_dWrapdRY5, tWrapAngelPara.m_dWrapdRZ5, tWrapAngelPara.m_dWrapdWaitTime5);
        fprintf(ReadParam, "%lf %lf %lf %lf %lf %lf %lf\n", tWrapAngelPara.m_dWrapdParallel6, tWrapAngelPara.m_dWrapdVertical6,
            tWrapAngelPara.m_dWrapdVelocity6, tWrapAngelPara.m_dWrapdRX6, tWrapAngelPara.m_dWrapdRY6, tWrapAngelPara.m_dWrapdRZ6, tWrapAngelPara.m_dWrapdWaitTime6);
        fprintf(ReadParam, "%lf %lf %lf %lf %lf %lf %lf\n", tWrapAngelPara.m_dWrapdParallel7, tWrapAngelPara.m_dWrapdVertical7,
            tWrapAngelPara.m_dWrapdVelocity7, tWrapAngelPara.m_dWrapdRX7, tWrapAngelPara.m_dWrapdRY7, tWrapAngelPara.m_dWrapdRZ7, tWrapAngelPara.m_dWrapdWaitTime7);
  
    }
    fclose(ReadParam);
}

void CWrapAngleParam::OnBnClickedButtonDelete()
{
    UpdateData(TRUE);
    CString strName;
//    WELD_WRAP_ANGLE tWrapAngelPara;
    int nIndex = m_ComboxWrapType.GetCurSel();
    CString str;
    m_ComboxWrapType.GetLBText(nIndex, str);
    strName = str;
    for (int i = 0; i < m_vtWrapAngel.size(); i++)
    {
        if (strName == m_vtWrapAngel.at(i).strWrapName)
        {
            if (XUI::MesBox::PopYesNo("是否确定删除：{0} 包角参数", (const char*)strName))
            {
                m_vtWrapAngel.erase(m_vtWrapAngel.begin() + i);
                XUI::MesBox::PopInfo("{0} 包角参数", (const char*)strName);
                m_ComboxWrapType.ResetContent();
                for (i = 0; i < m_vtWrapAngel.size(); i++)
                {
                    m_ComboxWrapType.AddString(m_vtWrapAngel.at(i).strWrapName);
                }
                m_ComboxWrapType.SetCurSel(0);
            }
            break;
        }
    }
}

void CWrapAngleParam::OnBnClickedButtonAdd()
{
	UpdateData(true);
	T_WELD_WRAP_ANGLE tWeldWrapAngle;
	tWeldWrapAngle.bWrapType = m_bWrapType == TRUE;
	tWeldWrapAngle.strWrapName = m_strWrapAngleNew;
	tWeldWrapAngle.tWeldWrapAngle = DownloadWrapAngleParam();
	int i;
	if ("" != m_strWrapAngleNew)
	{
		if (XUI::MesBox::PopOkCancel("确认添加包角：{0} 工艺", (const char*)m_strWrapAngleNew))
		{
			return;
		}
		for (i = 0; i < m_vtWrapAngel.size(); i++)
		{
			if (m_strWrapAngleNew == m_vtWrapAngel.at(i).strWrapName)
			{
				if (XUI::MesBox::PopYesNo("存在相同包角工艺：{0} 是否更改", (const char*)m_strWrapAngleNew))
				{
					m_vtWrapAngel.at(i).bWrapType = m_bWrapType == TRUE;
					m_vtWrapAngel.at(i) = tWeldWrapAngle;
				}
				break;
			}
		}
		if (i == m_vtWrapAngel.size())
		{
			if (XUI::MesBox::PopOkCancel("确定添加工艺：{0} ", (const char*)m_strWrapAngleNew))
			{
				m_vtWrapAngel.push_back(tWeldWrapAngle);
                XUI::MesBox::PopInfo("{0} 工艺添加成功", (const char*)m_strWrapAngleNew);
				m_ComboxWrapType.ResetContent();
				for (i = 0; i < m_vtWrapAngel.size(); i++)
				{
					m_ComboxWrapType.AddString(m_vtWrapAngel[i].strWrapName);
				}
				m_ComboxWrapType.SetCurSel(m_vtWrapAngel.size() - 1);
			}

		}
	}
	else
	{
		int nIndex = m_ComboxWrapType.GetCurSel();
		CString str;
		m_ComboxWrapType.GetLBText(nIndex, str);
		for (i = 0; i < m_vtWrapAngel.size(); i++)
		{
			if (str == m_vtWrapAngel.at(i).strWrapName)
			{
				if (XUI::MesBox::PopYesNo("是否更改：{0} 包角工艺 ", (const char*)str))
				{
					m_vtWrapAngel.at(i).bWrapType = m_bWrapType == TRUE;
					m_vtWrapAngel.at(i).tWeldWrapAngle = tWeldWrapAngle.tWeldWrapAngle;
				}
				break;
			}
		}
	}
}

BOOL CWrapAngleParam::PreTranslateMessage(MSG* pMsg)
{
    // TODO: 在此添加专用代码和/或调用基类

	if (pMsg->message == WM_KEYDOWN)
	{
		if ((pMsg->wParam == VK_RETURN) || (pMsg->wParam == VK_ESCAPE))
		{
			return TRUE;
		}
		else if (pMsg->wParam == VK_SPACE)//空格键暂停
		{
			WriteLog("按下空格键");

			return TRUE;
		}
		else if (pMsg->wParam == 'P'
			&& GetKeyState(VK_CONTROL) & 0x80000
			&& GetKeyState(VK_MENU) & 0x80000)
		{
			GetDlgItem(IDC_BUTTON_ADD)->SetWindowText("添加");
			((CStatic*)GetDlgItem(IDC_STATIC_ADD))->ShowWindow(TRUE);
			((CEdit*)GetDlgItem(IDC_EDIT_ADD_WRAP))->ShowWindow(TRUE);
			return TRUE;
		}
		else if
			(
				pMsg->wParam == 'D'&&
				GetKeyState(VK_CONTROL) & 0x80000 &&
				GetKeyState(VK_MENU) & 0x80000
				)
		{
			GetDlgItem(IDC_BUTTON_ADD)->SetWindowText("更改");
			((CStatic*)GetDlgItem(IDC_STATIC_ADD))->ShowWindow(FALSE);
			((CEdit*)GetDlgItem(IDC_EDIT_ADD_WRAP))->ShowWindow(FALSE);
			m_strWrapAngleNew = "";
			UpdateData(FALSE);
		}
		else if(pMsg->wParam == VK_LEFT)
		{
			XiMessageBox("左按下");
		}
		else if (pMsg->wParam == VK_RIGHT)
		{
			XiMessageBox("右按下");
		}
	}
    return CDialog::PreTranslateMessage(pMsg);
}

void CWrapAngleParam::OnBnClickedCheckWrapType()
{
	UpdateData(TRUE);
	if (m_bWrapType)
	{
		GetDlgItem(IDC_CHECK_WRAP_TYPE)->SetWindowText("一次成型");	
	}
	else
	{
		GetDlgItem(IDC_CHECK_WRAP_TYPE)->SetWindowText("多次成型");
	}
}

void CWrapAngleParam::OnBnClickedCheckWrapRobot()
{
	UpdateData(TRUE);
	if (m_bRobotNo)
	{
		GetDlgItem(IDC_CHECK_WRAP_ROBOT)->SetWindowText("右机械臂");
	}
	else
	{
		GetDlgItem(IDC_CHECK_WRAP_ROBOT)->SetWindowText("左机械臂");
	}
	WELD_WRAP_ANGLE tWrapAngelPara = InitWrapAngleParam(m_bRobotNo);
	RefreshWrapAngleParam(tWrapAngelPara);
	int nSetNo = 0;
	for (int i = 0; i < m_vtWrapAngel.size(); i++)
	{
		if (m_strWrapAngleType == m_vtWrapAngel.at(i).strWrapName)
		{
			nSetNo = i;
		}
	}
	m_ComboxWrapType.SetCurSel(nSetNo);
	if (m_bWrapType)
	{
		GetDlgItem(IDC_CHECK_WRAP_TYPE)->SetWindowText("一次成型");
	}
	else
	{
		GetDlgItem(IDC_CHECK_WRAP_TYPE)->SetWindowText("多次成型");
	}
	UpdateData(FALSE);
}
