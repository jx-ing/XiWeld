// SetServoCtrlParam.cpp : 实现文件
//

#include "stdafx.h"
#include ".\Project\XiGrooveRobot.h"
#include ".\Project\SetServoCtrlParam.h"
#include "afxdialogex.h"


// CSetServoCtrlParam 对话框

IMPLEMENT_DYNAMIC(CSetServoCtrlParam, CDialogEx)

CSetServoCtrlParam::CSetServoCtrlParam(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_PARA_SERVO_CTRL, pParent)
	, m_strLinkParam(_T(""))
	, m_nSoftCtrlCardNo(0)
	, m_nAxis(0)
	, m_nSoftAxisNo(0)
	, m_strAxisName(_T(""))
	, m_dPulse(0)
	, m_dDistance(0)
	, m_dPulseEquivalent(0)
	, m_bEnableAbsEncoder(FALSE)
	, m_nAbsEncoderPort(0)
	, m_nAbsEncodernLapPulse(0)
	, m_bEnableSoftLimit(FALSE)
	, m_dMinCoor(0)
	, m_dMaxCoor(0)
	, m_nMasterAxisNo(0)
	, m_dSpeedMid(0)
	, m_dSpeedMin(0)
	, m_dSpeedMax(0)
	, m_dAccMin(0)
	, m_dAccMid(0)
	, m_dAccMax(0)
	, m_dDecMin(0)
	, m_dDecMid(0)
	, m_dDecMax(0)
	, m_nNoteNum(0)
	, m_nPortNo(0)
	, m_nLocalIONum(0)
	, m_nSingleIONum(0)
	, m_nIOCFilter(1)
{

}

CSetServoCtrlParam::~CSetServoCtrlParam()
{
}

void CSetServoCtrlParam::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_SPIN_CARD_NO, m_cCardNo);
	DDX_Control(pDX, IDC_COMBO_CARD_TYPE, m_cCtrlCardType);
	DDX_Control(pDX, IDC_COMBO_CONNECT_MODE, m_cConnectMode);
	DDX_Control(pDX, IDC_COMBO_CARD_HARD_NO, m_cHardCtrlCardNo);
	DDX_Text(pDX, IDC_EDIT_LINK_PAEAM, m_strLinkParam);
	DDX_Text(pDX, IDC_EDIT_CARD_SOFT_NO, m_nSoftCtrlCardNo);
	DDX_Control(pDX, IDC_SPIN_AXIS_NO, m_cAxisNo);
	DDX_Radio(pDX, IDC_RADIO_AXIS_NO, m_nAxis);
	DDX_Control(pDX, IDC_COMBO_AXIS_HARD_NO, m_cHardAxisNo);
	DDX_Text(pDX, IDC_EDIT_AXIS_SOFT_NO, m_nSoftAxisNo);
	DDX_Text(pDX, IDC_EDIT_AXIS_NAME, m_strAxisName);
	DDX_Control(pDX, IDC_COMBO_AXIS_TYPE, m_cAxisType);
	DDX_Text(pDX, IDC_EDIT_PULSE_NUM, m_dPulse);
	DDX_Text(pDX, IDC_EDIT_DISTANCE, m_dDistance);
	DDX_Text(pDX, IDC_EDIT_PULSE_EQUIVALENT, m_dPulseEquivalent);
	DDX_Check(pDX, IDC_CHECK_ENABLE_ABS_ENCODER, m_bEnableAbsEncoder);
	DDX_Text(pDX, IDC_EDIT_ABS_PORT, m_nAbsEncoderPort);
	DDX_Text(pDX, IDC_EDIT_ABS_LAP_PULSE, m_nAbsEncodernLapPulse);
	DDX_Check(pDX, IDC_CHECK_ENABLE_LIMIT, m_bEnableSoftLimit);
	DDX_Text(pDX, IDC_EDIT_MIN_COOR, m_dMinCoor);
	DDX_Text(pDX, IDC_EDIT_MAX_COOR, m_dMaxCoor);
	DDX_Control(pDX, IDC_COMBO_GEAR_FOLLOW, m_cGearFollowProfile);
	DDX_Text(pDX, IDC_EDIT_MASTER_NO, m_nMasterAxisNo);
	DDX_Control(pDX, IDC_COMBO_ALM_LOGIC, m_cAlmLogic);
	DDX_Text(pDX, IDC_EDIT_SPEED_MIN, m_dSpeedMin);
	DDX_Text(pDX, IDC_EDIT_SPEED_MID, m_dSpeedMid);
	DDX_Text(pDX, IDC_EDIT_SPEED_MAX, m_dSpeedMax);
	DDX_Text(pDX, IDC_EDIT_ACC_MIN, m_dAccMin);
	DDX_Text(pDX, IDC_EDIT_ACC_MID, m_dAccMid);
	DDX_Text(pDX, IDC_EDIT_ACC_MAX, m_dAccMax);
	DDX_Text(pDX, IDC_EDIT_DEC_MIN, m_dDecMin);
	DDX_Text(pDX, IDC_EDIT_DEC_MID, m_dDecMid);
	DDX_Text(pDX, IDC_EDIT_DEC_MAX, m_dDecMax);
	DDX_Control(pDX, IDC_COMBO_PULSE_MODE, m_cPulseMode);
	DDX_Control(pDX, IDC_COMBO_IO_TYPE, m_cIOType);
	DDX_Text(pDX, IDC_EDIT_IO_NOTE, m_nNoteNum);
	DDV_MinMaxInt(pDX, m_nNoteNum, 0, 8);
	DDX_Text(pDX, IDC_EDIT_IO_PORT_NO, m_nPortNo);
	DDX_Text(pDX, IDC_EDIT_LOCAL_IO_NUM, m_nLocalIONum);
	DDX_Text(pDX, IDC_EDIT_SINGLE_IO_NUM, m_nSingleIONum);
	DDX_Text(pDX, IDC_EDIT_IOC_FILTER, m_nIOCFilter);
	DDV_MinMaxInt(pDX, m_nIOCFilter, 1, 100);
	DDX_Control(pDX, IDC_COMBO_CAN_BAUD_NO, m_cBaudNo);
	DDX_Control(pDX, IDC_COMBO_CAT_CCLE_TIME, m_cCATCycleTime);
	DDX_Control(pDX, IDC_COMBO_ABS_ENCODER_DIR, m_cAbsEncoderDir);
}


BEGIN_MESSAGE_MAP(CSetServoCtrlParam, CDialogEx)
	ON_BN_CLICKED(IDOK, &CSetServoCtrlParam::OnBnClickedOk)
	ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_CARD_NO, &CSetServoCtrlParam::OnDeltaposSpin1)
	ON_BN_CLICKED(IDC_BUTTON_ADD_CARD, &CSetServoCtrlParam::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON_DELETE_CARD, &CSetServoCtrlParam::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_BUTTON_SAVE_AXIS_CARD, &CSetServoCtrlParam::OnBnClickedButton2)
	ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_AXIS_NO, &CSetServoCtrlParam::OnDeltaposSpin2)
	ON_BN_CLICKED(IDC_BUTTON_ADD_AXIS, &CSetServoCtrlParam::OnBnClickedButton11)
	ON_BN_CLICKED(IDC_BUTTON_DELETE_AXIS, &CSetServoCtrlParam::OnBnClickedButton12)
	ON_CBN_SELCHANGE(IDC_COMBO_CARD_HARD_NO, &CSetServoCtrlParam::OnCbnSelchangeCombo3)
	ON_CBN_SELCHANGE(IDC_COMBO_CONNECT_MODE, &CSetServoCtrlParam::OnCbnSelchangeCombo2)
	ON_CBN_SELCHANGE(IDC_COMBO_CARD_TYPE, &CSetServoCtrlParam::OnCbnSelchangeCombo1)
	ON_EN_CHANGE(IDC_EDIT_LINK_PAEAM, &CSetServoCtrlParam::OnEnChangeEdit1)
	ON_BN_CLICKED(IDC_RADIO_AXIS_NO, &CSetServoCtrlParam::OnBnClickedRadioAxisNo)
	ON_BN_CLICKED(IDC_RADIO_AXIS_NO2, &CSetServoCtrlParam::OnBnClickedRadio2)
	ON_BN_CLICKED(IDC_RADIO_AXIS_NO3, &CSetServoCtrlParam::OnBnClickedRadio3)
	ON_BN_CLICKED(IDC_RADIO_AXIS_NO4, &CSetServoCtrlParam::OnBnClickedRadio4)
	ON_BN_CLICKED(IDC_RADIO_AXIS_NO5, &CSetServoCtrlParam::OnBnClickedRadio5)
	ON_BN_CLICKED(IDC_RADIO_AXIS_NO6, &CSetServoCtrlParam::OnBnClickedRadio6)
	ON_BN_CLICKED(IDC_RADIO_AXIS_NO7, &CSetServoCtrlParam::OnBnClickedRadio7)
	ON_BN_CLICKED(IDC_RADIO_AXIS_NO8, &CSetServoCtrlParam::OnBnClickedRadio8)
	ON_CBN_SELCHANGE(IDC_COMBO_AXIS_HARD_NO, &CSetServoCtrlParam::OnCbnSelchangeCombo4)
	ON_EN_CHANGE(IDC_EDIT_AXIS_NAME, &CSetServoCtrlParam::OnEnChangeEdit5)
	ON_CBN_SELCHANGE(IDC_COMBO_AXIS_TYPE, &CSetServoCtrlParam::OnCbnSelchangeCombo5)
	ON_EN_CHANGE(IDC_EDIT_PULSE_EQUIVALENT, &CSetServoCtrlParam::OnEnChangeEdit9)
	ON_BN_CLICKED(IDC_CHECK_ENABLE_ABS_ENCODER, &CSetServoCtrlParam::OnBnClickedCheck1)
	ON_EN_CHANGE(IDC_EDIT_ABS_PORT, &CSetServoCtrlParam::OnEnChangeEdit24)
	ON_EN_CHANGE(IDC_EDIT_ABS_LAP_PULSE, &CSetServoCtrlParam::OnEnChangeEdit10)
	ON_BN_CLICKED(IDC_CHECK_ENABLE_LIMIT, &CSetServoCtrlParam::OnBnClickedCheck2)
	ON_EN_CHANGE(IDC_CHECK_ENABLE_LIMIT, &CSetServoCtrlParam::OnEnChangeEdit25)
	ON_EN_CHANGE(IDC_EDIT_MAX_COOR, &CSetServoCtrlParam::OnEnChangeEdit17)
	ON_CBN_SELCHANGE(IDC_COMBO_GEAR_FOLLOW, &CSetServoCtrlParam::OnCbnSelchangeCombo6)
	ON_EN_CHANGE(IDC_EDIT_MASTER_NO, &CSetServoCtrlParam::OnEnChangeEdit19)
	ON_CBN_SELCHANGE(IDC_COMBO_ALM_LOGIC, &CSetServoCtrlParam::OnCbnSelchangeCombo7)
	ON_EN_CHANGE(IDC_EDIT_SPEED_MIN, &CSetServoCtrlParam::OnEnChangeEdit21)
	ON_EN_CHANGE(IDC_EDIT_SPEED_MID, &CSetServoCtrlParam::OnEnChangeEdit26)
	ON_EN_CHANGE(IDC_EDIT_SPEED_MAX, &CSetServoCtrlParam::OnEnChangeEdit27)
	ON_EN_CHANGE(IDC_EDIT_ACC_MIN, &CSetServoCtrlParam::OnEnChangeEdit28)
	ON_EN_CHANGE(IDC_EDIT_ACC_MID, &CSetServoCtrlParam::OnEnChangeEdit29)
	ON_EN_CHANGE(IDC_EDIT_ACC_MAX, &CSetServoCtrlParam::OnEnChangeEdit30)
	ON_EN_CHANGE(IDC_EDIT_DEC_MIN, &CSetServoCtrlParam::OnEnChangeEdit31)
	ON_EN_CHANGE(IDC_EDIT_DEC_MID, &CSetServoCtrlParam::OnEnChangeEdit32)
	ON_EN_CHANGE(IDC_EDIT_DEC_MAX, &CSetServoCtrlParam::OnEnChangeEdit33)
	ON_CBN_SELCHANGE(IDC_COMBO_PULSE_MODE, &CSetServoCtrlParam::OnCbnSelchangeCombo8)
	ON_CBN_SELCHANGE(IDC_COMBO_IO_TYPE, &CSetServoCtrlParam::OnCbnSelchangeCombo9)
	ON_EN_CHANGE(IDC_EDIT_IO_NOTE, &CSetServoCtrlParam::OnEnChangeEdit37)
	ON_EN_CHANGE(IDC_EDIT_IO_PORT_NO, &CSetServoCtrlParam::OnEnChangeEdit38)
	ON_EN_CHANGE(IDC_EDIT_LOCAL_IO_NUM, &CSetServoCtrlParam::OnEnChangeEdit39)
	ON_EN_CHANGE(IDC_EDIT_SINGLE_IO_NUM, &CSetServoCtrlParam::OnEnChangeEdit40)
	ON_CBN_SELCHANGE(IDC_COMBO_CAN_BAUD_NO, &CSetServoCtrlParam::OnCbnSelchangeCombo10)
	ON_CBN_SELCHANGE(IDC_COMBO_CAT_CCLE_TIME, &CSetServoCtrlParam::OnCbnSelchangeCombo11)
	ON_EN_CHANGE(IDC_EDIT_IOC_FILTER, &CSetServoCtrlParam::OnEnChangeEdit41)
	ON_CBN_SELCHANGE(IDC_COMBO_ABS_ENCODER_DIR, &CSetServoCtrlParam::OnCbnSelchangeComboAbsEncoderDir)
END_MESSAGE_MAP()


// CSetServoCtrlParam 消息处理程序

BOOL CSetServoCtrlParam::OnInitDialog()                      
{
	CDialogEx::OnInitDialog();

	// TODO:  在此添加额外的初始化
	InitAxisRadioShow();
	//加载轴卡参数
	m_cServoMotorDriver.LoadCtrlCardParam(m_cServoMotorDriver.m_vtCtrlCardInfo);



	UpdateCardNoShow(m_cServoMotorDriver.m_vtCtrlCardInfo.size());
	if (!m_cServoMotorDriver.m_vtCtrlCardInfo.empty())
	{
		UpdateAxisNoShow(m_cServoMotorDriver.m_vtCtrlCardInfo[0].vtAxisInfo.size());
	}

	ShowCtrlCardInfo(0);
	ShowAxisInfo(0, 0);
	m_nAxis = 0;
	UpdateData(FALSE);
	//设置数据的进制
	m_cCardNo.SetBase(10);
	m_cAxisNo.SetBase(10);
	//设置伙伴控件
	m_cCardNo.SetBuddy(GetDlgItem(IDC_STATIC_CARD_NO));
	m_cAxisNo.SetBuddy(GetDlgItem(IDC_STATIC_AXIS_NO));
	//设置默认显示
	m_cCardNo.SetPos(1);
	m_cAxisNo.SetPos(0);
	//已修改
	XUI::Languge::GetInstance().translateDialog(this);
	return TRUE;  // return TRUE unless you set the focus to a control
				  // 异常: OCX 属性页应返回 FALSE
}

void CSetServoCtrlParam::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
//	CDialogEx::OnOK();
}

void CSetServoCtrlParam::ShowCtrlCardInfo(int nSoftCtrlCard)
{
	T_CTRL_CARD_INFO tCtrlCardInfo;
	if (nSoftCtrlCard < m_cServoMotorDriver.m_vtCtrlCardInfo.size() && nSoftCtrlCard >= 0)
	{
		tCtrlCardInfo = m_cServoMotorDriver.m_vtCtrlCardInfo[nSoftCtrlCard];
	}
	m_cCtrlCardType.SetCurSel(tCtrlCardInfo.eCtrlCardType);
	m_cConnectMode.SetCurSel(tCtrlCardInfo.eConnectType);
	m_cHardCtrlCardNo.SetCurSel(tCtrlCardInfo.nHardCtrlCardNo);
	m_nSoftCtrlCardNo = tCtrlCardInfo.nSoftCtrlCardNo;
	if (tCtrlCardInfo.eConnectType == 0)
	{
		GetDlgItem(IDC_EDIT_LINK_PAEAM)->EnableWindow(FALSE);
		m_strLinkParam = "";
	}
	else
	{
		GetDlgItem(IDC_EDIT_LINK_PAEAM)->EnableWindow(TRUE);
		m_strLinkParam = tCtrlCardInfo.strNetWorkIP;
	}
	UpdateAxisNoShow(tCtrlCardInfo.vtAxisInfo.size());
	ShowAxisInfo(nSoftCtrlCard, m_cAxisNo.GetPos() - 1);
	ShowIOInfo(nSoftCtrlCard);
	UpdateData(FALSE);
}



void CSetServoCtrlParam::ShowIOInfo(int nSoftCtrlCard)
{
	T_CTRL_CARD_INFO tCtrlCardInfo;
	T_IO_CTRL_INFO tIOInfo;
	if (nSoftCtrlCard < m_cServoMotorDriver.m_vtCtrlCardInfo.size() && nSoftCtrlCard >= 0)
	{
		tIOInfo = m_cServoMotorDriver.m_vtCtrlCardInfo[nSoftCtrlCard].tIOCtrlInfo;
		tCtrlCardInfo = m_cServoMotorDriver.m_vtCtrlCardInfo[nSoftCtrlCard];
	}
	m_cIOType.SetCurSel((int)tIOInfo.eIOType);
	if (tIOInfo.eIOType == E_IO_CARD_TYPE_NONE)
	{
		EnableIO(false);
	}
	else
	{
		EnableIO(true);

		if (tCtrlCardInfo.eCtrlCardType != E_CTRL_CARD_TYPE_SMC604)
		{
			GetDlgItem(IDC_EDIT_IO_PORT_NO)->EnableWindow(FALSE);
		}
		else
		{
			GetDlgItem(IDC_EDIT_IO_PORT_NO)->EnableWindow(TRUE);
		}
		if (tCtrlCardInfo.eCtrlCardType != E_CTRL_CARD_TYPE_IOC0640 &&
			tCtrlCardInfo.eCtrlCardType != E_CTRL_CARD_TYPE_IOC1280)
		{
			if (tIOInfo.eIOType == E_IO_CARD_TYPE_IOC0640 ||
				tIOInfo.eIOType == E_IO_CARD_TYPE_IOC1280)
			{
				m_cIOType.SetCurSel(0);
				EnableIO(false);
			}
		}
		if (tCtrlCardInfo.eCtrlCardType != E_CTRL_CARD_TYPE_IOC0640 &&
			tCtrlCardInfo.eCtrlCardType != E_CTRL_CARD_TYPE_IOC1280)
		{
			GetDlgItem(IDC_EDIT_IOC_FILTER)->EnableWindow(FALSE);
		}
		if (tCtrlCardInfo.eCtrlCardType == E_CTRL_CARD_TYPE_IOC0640 ||
			tCtrlCardInfo.eCtrlCardType == E_CTRL_CARD_TYPE_IOC1280)
		{
			if (tCtrlCardInfo.eCtrlCardType == E_CTRL_CARD_TYPE_IOC0640)
			{
				tIOInfo.nLocalIONum = 32;
			}
			else
			{
				tIOInfo.nLocalIONum = 64;
			}
			GetDlgItem(IDC_EDIT_IO_NOTE)->EnableWindow(FALSE);
			GetDlgItem(IDC_EDIT_LOCAL_IO_NUM)->EnableWindow(FALSE);
			GetDlgItem(IDC_EDIT_SINGLE_IO_NUM)->EnableWindow(FALSE);
		}
		if (tIOInfo.eIOType != E_IO_CARD_TYPE_CAN)
		{
			GetDlgItem(IDC_COMBO_CAN_BAUD_NO)->EnableWindow(FALSE);
		}
		if (tIOInfo.eIOType != E_IO_CARD_TYPE_CAT)
		{
			GetDlgItem(IDC_COMBO_CAT_CCLE_TIME)->EnableWindow(FALSE);
		}
	}
	m_nNoteNum = tIOInfo.nNoteNum;
	m_nPortNo = tIOInfo.nPortNo;
	m_nLocalIONum = tIOInfo.nLocalIONum;
	m_nSingleIONum = tIOInfo.nSingleIONum;
	m_nIOCFilter = tIOInfo.nIOCFilter;
	m_cBaudNo.SetCurSel(tIOInfo.nCANBaudNo);
	m_cCATCycleTime.SetCurSel(tIOInfo.nCATCycleTime);
}

void CSetServoCtrlParam::ShowAxisInfo(int nSoftCtrlCard, int nSoftAxisNo)
{
	T_AXIS_CTRL_INFO tAxisInfo;
	if (nSoftCtrlCard < m_cServoMotorDriver.m_vtCtrlCardInfo.size() && nSoftCtrlCard >= 0)
	{
		if (nSoftAxisNo < m_cServoMotorDriver.m_vtCtrlCardInfo[nSoftCtrlCard].vtAxisInfo.size() && nSoftAxisNo >= 0)
		{
			tAxisInfo = m_cServoMotorDriver.m_vtCtrlCardInfo[nSoftCtrlCard].vtAxisInfo[nSoftAxisNo];
		}
	}
	m_cHardAxisNo.SetCurSel(tAxisInfo.nHardAxisNo);
	m_nSoftAxisNo = tAxisInfo.nSoftAxisNo;
	m_strAxisName = tAxisInfo.strAxisName;
	m_cAxisType.SetCurSel(tAxisInfo.eServoType);
	m_dPulseEquivalent = tAxisInfo.dPulseEquivalent;
	m_bEnableAbsEncoder = tAxisInfo.bEnableAbsEncoder;
	if (m_bEnableAbsEncoder == TRUE)
	{
		m_nAbsEncoderPort = tAxisInfo.nAbsEncoderPartNo;
		m_nAbsEncodernLapPulse = tAxisInfo.nLapPulse;
		if (tAxisInfo.nAbsEncoderDir == 1)
		{
			m_cAbsEncoderDir.SetCurSel(0);
		}
		else
		{
			m_cAbsEncoderDir.SetCurSel(1);
		}
		GetDlgItem(IDC_EDIT_ABS_PORT)->EnableWindow(TRUE);
		GetDlgItem(IDC_EDIT_ABS_LAP_PULSE)->EnableWindow(TRUE);
		GetDlgItem(IDC_COMBO_ABS_ENCODER_DIR)->EnableWindow(TRUE);

	}
	else
	{
		m_nAbsEncoderPort = -1;
		m_nAbsEncodernLapPulse = 0;
		m_cAbsEncoderDir.SetCurSel(0);
		GetDlgItem(IDC_EDIT_ABS_PORT)->EnableWindow(FALSE);
		GetDlgItem(IDC_EDIT_ABS_LAP_PULSE)->EnableWindow(FALSE);
		GetDlgItem(IDC_COMBO_ABS_ENCODER_DIR)->EnableWindow(FALSE);
	}
	m_bEnableSoftLimit = tAxisInfo.bEnableSoftLimit;
	if (m_bEnableSoftLimit == TRUE)
	{
		m_dMinCoor = tAxisInfo.dNegativeLimit;
		m_dMaxCoor = tAxisInfo.dPositiveLimit;
		GetDlgItem(IDC_EDIT_MIN_COOR)->EnableWindow(TRUE);
		GetDlgItem(IDC_EDIT_MAX_COOR)->EnableWindow(TRUE);
	}
	else
	{
		m_dMinCoor = 0;
		m_dMaxCoor = 0;
		GetDlgItem(IDC_EDIT_MIN_COOR)->EnableWindow(FALSE);
		GetDlgItem(IDC_EDIT_MAX_COOR)->EnableWindow(FALSE);
	}
	m_cGearFollowProfile.SetCurSel(tAxisInfo.nGearFollowProfile);
	if (tAxisInfo.nGearFollowProfile == 2)
	{
		GetDlgItem(IDC_EDIT_MASTER_NO)->EnableWindow(TRUE);
		m_nMasterAxisNo = tAxisInfo.nMasterAxisNo;
	}
	else
	{
		m_nMasterAxisNo = -1;
		GetDlgItem(IDC_EDIT_MASTER_NO)->EnableWindow(FALSE);
	}
	m_cAlmLogic.SetCurSel(tAxisInfo.bAlmLogic);
	m_cPulseMode.SetCurSel(tAxisInfo.nPulseMode);
	m_dSpeedMin = tAxisInfo.tMinSpeed.dSpeed;
	m_dAccMin = tAxisInfo.tMinSpeed.dAcc;
	m_dDecMin = tAxisInfo.tMinSpeed.dDec;

	m_dSpeedMid = tAxisInfo.tMidSpeed.dSpeed;
	m_dAccMid = tAxisInfo.tMidSpeed.dAcc;
	m_dDecMid = tAxisInfo.tMidSpeed.dDec;

	m_dSpeedMax = tAxisInfo.tMaxSpeed.dSpeed;
	m_dAccMax = tAxisInfo.tMaxSpeed.dAcc;
	m_dDecMax = tAxisInfo.tMaxSpeed.dDec;
	UpdateData(FALSE);
}

void CSetServoCtrlParam::UpdateCardNoShow(int nNum)
{
	if (m_cCardNo.GetPos() > nNum)
	{
		m_cCardNo.SetPos(nNum);
		ShowCtrlCardInfo(nNum - 1);
	}
	m_cCardNo.SetRange32(1, nNum);
	CString str;
	str.Format("/%d", nNum);
	GetDlgItem(IDC_STATIC_CARD_NUM)->SetWindowText(str);
	if (nNum == 0)
	{
		EnableCtrlCard(false);
		m_cCardNo.SetRange32(0, 0);
		GetDlgItem(IDC_STATIC_CARD_NO)->SetWindowText("0");
		ShowAxisInfo(-1, -1);
	}
}

void CSetServoCtrlParam::UpdateAxisNoShow(int nNum)
{
	if (m_cAxisNo.GetPos() > nNum)
	{
		m_nAxis = nNum - 1;
		m_cAxisNo.SetPos(nNum);
		ShowAxisInfo(m_cCardNo.GetPos()-1, nNum-1);
	}
	AxisRadioShow(nNum);
	m_cAxisNo.SetRange32(1, nNum);
	CString str;
	str.Format("/%d", nNum);
	GetDlgItem(IDC_STATIC_AXIS_NUM)->SetWindowText(str);
	if (nNum == 0)
	{
		EnableAxis(false);
		m_cAxisNo.SetRange32(0, 0);
		GetDlgItem(IDC_STATIC_AXIS_NO)->SetWindowText("0");
	}
	else
	{
		EnableAxis(true);
	}
	UpdateData(FALSE);
}

void CSetServoCtrlParam::EnableCtrlCard(bool bEnable)
{
	GetDlgItem(IDC_COMBO_CARD_TYPE)->EnableWindow(bEnable);
	GetDlgItem(IDC_COMBO_CONNECT_MODE)->EnableWindow(bEnable);
	GetDlgItem(IDC_COMBO_CARD_HARD_NO)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_LINK_PAEAM)->EnableWindow(bEnable);
	GetDlgItem(IDC_BUTTON_DELETE_CARD)->EnableWindow(bEnable);
}

void CSetServoCtrlParam::EnableIO(bool bEnable)
{
	GetDlgItem(IDC_EDIT_IO_NOTE)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_IO_PORT_NO)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_LOCAL_IO_NUM)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_SINGLE_IO_NUM)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_IOC_FILTER)->EnableWindow(bEnable);
	GetDlgItem(IDC_COMBO_CAN_BAUD_NO)->EnableWindow(bEnable);
	GetDlgItem(IDC_COMBO_CAT_CCLE_TIME)->EnableWindow(bEnable);
}

void CSetServoCtrlParam::EnableAxis(bool bEnable)
{
	GetDlgItem(IDC_COMBO_AXIS_HARD_NO)->EnableWindow(bEnable);
	GetDlgItem(IDC_COMBO_AXIS_TYPE)->EnableWindow(bEnable);
	GetDlgItem(IDC_COMBO_GEAR_FOLLOW)->EnableWindow(bEnable);
	GetDlgItem(IDC_COMBO_ALM_LOGIC)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_AXIS_NAME)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_PULSE_NUM)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_DISTANCE)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_PULSE_EQUIVALENT)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_ABS_LAP_PULSE)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_MIN_COOR)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_MAX_COOR)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_MASTER_NO)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_ABS_PORT)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_SPEED_MIN)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_SPEED_MID)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_SPEED_MAX)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_ACC_MIN)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_ACC_MID)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_ACC_MAX)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_DEC_MIN)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_DEC_MID)->EnableWindow(bEnable);
	GetDlgItem(IDC_EDIT_DEC_MAX)->EnableWindow(bEnable);
	GetDlgItem(IDC_CHECK_ENABLE_ABS_ENCODER)->EnableWindow(bEnable);
	GetDlgItem(IDC_CHECK_ENABLE_LIMIT)->EnableWindow(bEnable);
	GetDlgItem(IDC_COMBO_PULSE_MODE)->EnableWindow(bEnable);
	GetDlgItem(IDC_BUTTON_DELETE_AXIS)->EnableWindow(bEnable);
}

void CSetServoCtrlParam::OnDeltaposSpin1(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
	// TODO: 在此添加控件通知处理程序代码
	*pResult = 0;
	int nNo = m_cCardNo.GetPos(), lower, upper;
	m_cCardNo.GetRange32(lower, upper);
	nNo += pNMUpDown->iDelta;
	if (nNo <= upper && nNo >= lower)
	{
		ShowCtrlCardInfo(nNo - 1);
	}
}


void CSetServoCtrlParam::OnBnClickedButton1()
{
	// TODO: 在此添加控件通知处理程序代码

	EnableCtrlCard(true);
	T_CTRL_CARD_INFO tCtrlCardInfo;
	tCtrlCardInfo.nSoftCtrlCardNo = m_cServoMotorDriver.m_vtCtrlCardInfo.size();
	m_cServoMotorDriver.m_vtCtrlCardInfo.push_back(tCtrlCardInfo);
	UpdateCardNoShow(m_cServoMotorDriver.m_vtCtrlCardInfo.size());
	m_cCardNo.SetPos(m_cServoMotorDriver.m_vtCtrlCardInfo.size());
	ShowCtrlCardInfo(m_cServoMotorDriver.m_vtCtrlCardInfo.size() - 1);
}



void CSetServoCtrlParam::OnBnClickedButton3()
{
	// TODO: 在此添加控件通知处理程序代码
	if (m_cServoMotorDriver.m_vtCtrlCardInfo.size() <= m_cCardNo.GetPos()-1)
	{
		return;
	}
	int nPos = m_cCardNo.GetPos() - 1;
	m_cServoMotorDriver.m_vtCtrlCardInfo.erase(
		m_cServoMotorDriver.m_vtCtrlCardInfo.begin() + nPos, 
		m_cServoMotorDriver.m_vtCtrlCardInfo.begin() + nPos + 1);
	UpdateCardNoShow(m_cServoMotorDriver.m_vtCtrlCardInfo.size());
}


void CSetServoCtrlParam::OnBnClickedButton2()
{
	// TODO: 在此添加控件通知处理程序代码
	m_cServoMotorDriver.SaveCtrlCardParam(m_cServoMotorDriver.m_vtCtrlCardInfo);
	XUI::MesBox::PopInfo("保存完成！");
}


void CSetServoCtrlParam::OnDeltaposSpin2(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
	// TODO: 在此添加控件通知处理程序代码
	*pResult = 0;
	int nNo = m_cAxisNo.GetPos(), lower, upper;
	m_cAxisNo.GetRange32(lower, upper);
	nNo += pNMUpDown->iDelta;
	if (nNo <= upper && nNo >= lower)
	{
		m_nAxis = nNo - 1;
		ShowAxisInfo(m_cCardNo.GetPos()-1, nNo - 1);
		UpdateData(FALSE);
	}
}


void CSetServoCtrlParam::OnBnClickedButton11()
{
	// TODO: 在此添加控件通知处理程序代码
	if (m_cServoMotorDriver.m_vtCtrlCardInfo.size() <= m_cCardNo.GetPos()-1)
	{
		return;
	}
	if (m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo.size() >= 8)
	{
		AfxMessageBox("以达到最大轴数！");
		return;
	}
	if (m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].eCtrlCardType == E_CTRL_CARD_TYPE_IOC0640 || 
		m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].eCtrlCardType == E_CTRL_CARD_TYPE_IOC1280)
	{
		AfxMessageBox("IO卡无运动轴！");
		return;
	}
	EnableAxis(true);
	T_AXIS_CTRL_INFO tAxisInfo;
	tAxisInfo.nSoftAxisNo = m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo.size();
	tAxisInfo.nSoftCtrlCardNo = m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].nSoftCtrlCardNo;
	tAxisInfo.nHardCtrlCardNo = m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].nHardCtrlCardNo;
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo.push_back(tAxisInfo);
	m_cServoMotorDriver.SetSoftAxisNo();
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].nAxisNum = m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo.size();
	UpdateAxisNoShow(m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo.size());
	m_cAxisNo.SetPos(m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo.size());
	m_nAxis = m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo.size() - 1;
	ShowAxisInfo(m_cCardNo.GetPos() - 1, m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo.size()-1);
	UpdateData(FALSE);
}


void CSetServoCtrlParam::OnBnClickedButton12()
{
	// TODO: 在此添加控件通知处理程序代码
	if (m_cServoMotorDriver.m_vtCtrlCardInfo.size() <= m_cCardNo.GetPos() - 1)
	{
		return;
	}
	if (m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo.size() <= m_cAxisNo.GetPos()-1)
	{
		return;
	}
	int nPos = m_cAxisNo.GetPos()-1;
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo.erase(
		m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo.begin() + nPos,
		m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo.begin() + nPos + 1);
	m_cServoMotorDriver.SetSoftAxisNo();
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].nAxisNum = m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo.size();
	UpdateAxisNoShow(m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo.size());
}


void CSetServoCtrlParam::OnCbnSelchangeCombo3()
{
	// TODO: 在此添加控件通知处理程序代码
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].nHardCtrlCardNo = m_cHardCtrlCardNo.GetCurSel();
}


void CSetServoCtrlParam::OnCbnSelchangeCombo2()
{
	// TODO: 在此添加控件通知处理程序代码
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].eConnectType = (E_CONNECT_TYPE)m_cConnectMode.GetCurSel();
	if (m_cConnectMode.GetCurSel() == 0)
	{
		m_strLinkParam = "";
		GetDlgItem(IDC_EDIT_LINK_PAEAM)->EnableWindow(FALSE);
	}
	else
	{
		m_strLinkParam = m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].strNetWorkIP;
		GetDlgItem(IDC_EDIT_LINK_PAEAM)->EnableWindow(TRUE);
	}
	UpdateData(FALSE);
}


void CSetServoCtrlParam::OnCbnSelchangeCombo1()
{
	// TODO: 在此添加控件通知处理程序代码
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].eCtrlCardType = (E_CTRL_CARD_TYPE)m_cCtrlCardType.GetCurSel();
	if (m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].eCtrlCardType == E_CTRL_CARD_TYPE_IOC0640 || 
		m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].eCtrlCardType == E_CTRL_CARD_TYPE_IOC1280)
	{
		UpdateAxisNoShow(0);
		if ((E_CTRL_CARD_TYPE)m_cCtrlCardType.GetCurSel() == E_CTRL_CARD_TYPE_IOC0640)
		{
			m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].tIOCtrlInfo.eIOType = E_IO_CARD_TYPE_IOC0640;
			m_cIOType.SetCurSel((int)E_IO_CARD_TYPE_IOC0640);
		}
		if ((E_CTRL_CARD_TYPE)m_cCtrlCardType.GetCurSel() == E_CTRL_CARD_TYPE_IOC1280)
		{
			m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].tIOCtrlInfo.eIOType = E_IO_CARD_TYPE_IOC1280;
			m_cIOType.SetCurSel((int)E_IO_CARD_TYPE_IOC1280);
		}
		ShowIOInfo(m_cCardNo.GetPos() - 1);
		UpdateData(FALSE);
		GetDlgItem(IDC_COMBO_IO_TYPE)->EnableWindow(FALSE);
	}
	else
	{
		GetDlgItem(IDC_COMBO_IO_TYPE)->EnableWindow(TRUE);
	}
}


void CSetServoCtrlParam::OnEnChangeEdit1()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].strNetWorkIP = m_strLinkParam;
}

void CSetServoCtrlParam::InitAxisRadioShow()
{
	m_vnAxisID.clear();
	m_vnAxisID.push_back(IDC_RADIO_AXIS_NO);
	m_vnAxisID.push_back(IDC_RADIO_AXIS_NO2);
	m_vnAxisID.push_back(IDC_RADIO_AXIS_NO3);
	m_vnAxisID.push_back(IDC_RADIO_AXIS_NO4);
	m_vnAxisID.push_back(IDC_RADIO_AXIS_NO5);
	m_vnAxisID.push_back(IDC_RADIO_AXIS_NO6);
	m_vnAxisID.push_back(IDC_RADIO_AXIS_NO7);
	m_vnAxisID.push_back(IDC_RADIO_AXIS_NO8);
}

void CSetServoCtrlParam::AxisRadioShow(int nNum)
{
	for (int n = 0; n < m_vnAxisID.size(); n++)
	{
		if (n < nNum)
		{
			GetDlgItem(m_vnAxisID[n])->ShowWindow(TRUE);
		}
		else
		{
			GetDlgItem(m_vnAxisID[n])->ShowWindow(FALSE);
		}
	}
}


void CSetServoCtrlParam::OnBnClickedRadioAxisNo()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_cAxisNo.SetPos(m_nAxis+1);
	ShowAxisInfo(m_cCardNo.GetPos() - 1, m_nAxis);
	UpdateData(FALSE);
}


void CSetServoCtrlParam::OnBnClickedRadio2()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_cAxisNo.SetPos(m_nAxis + 1);
	ShowAxisInfo(m_cCardNo.GetPos() - 1, m_nAxis);
	UpdateData(FALSE);
}


void CSetServoCtrlParam::OnBnClickedRadio3()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_cAxisNo.SetPos(m_nAxis + 1);
	ShowAxisInfo(m_cCardNo.GetPos() - 1, m_nAxis);
	UpdateData(FALSE);
}


void CSetServoCtrlParam::OnBnClickedRadio4()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_cAxisNo.SetPos(m_nAxis + 1);
	ShowAxisInfo(m_cCardNo.GetPos() - 1, m_nAxis);
	UpdateData(FALSE);
}


void CSetServoCtrlParam::OnBnClickedRadio5()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_cAxisNo.SetPos(m_nAxis + 1);
	ShowAxisInfo(m_cCardNo.GetPos() - 1, m_nAxis);
	UpdateData(FALSE);
}


void CSetServoCtrlParam::OnBnClickedRadio6()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_cAxisNo.SetPos(m_nAxis + 1);
	ShowAxisInfo(m_cCardNo.GetPos() - 1, m_nAxis);
	UpdateData(FALSE);
}


void CSetServoCtrlParam::OnBnClickedRadio7()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_cAxisNo.SetPos(m_nAxis + 1);
	ShowAxisInfo(m_cCardNo.GetPos() - 1, m_nAxis);
	UpdateData(FALSE);
}


void CSetServoCtrlParam::OnBnClickedRadio8()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_cAxisNo.SetPos(m_nAxis + 1);
	ShowAxisInfo(m_cCardNo.GetPos() - 1, m_nAxis);
	UpdateData(FALSE);
}


void CSetServoCtrlParam::OnCbnSelchangeCombo4()
{
	// TODO: 在此添加控件通知处理程序代码
	for (int n = 0; n < m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo.size(); n++)
	{
		if (m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[n].nHardAxisNo == m_cHardAxisNo.GetCurSel())
		{
			AfxMessageBox("已占用的物理轴号！");
			m_cHardAxisNo.SetCurSel(-1);
			return;
		}
	}
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].nHardAxisNo = m_cHardAxisNo.GetCurSel();
}


void CSetServoCtrlParam::OnEnChangeEdit5()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].strAxisName = m_strAxisName;
}


void CSetServoCtrlParam::OnCbnSelchangeCombo5()
{
	// TODO: 在此添加控件通知处理程序代码
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].eServoType = (E_SERVO_TYPE)m_cAxisType.GetCurSel();
}


void CSetServoCtrlParam::OnEnChangeEdit9()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	if (!CheckStr(IDC_EDIT_PULSE_EQUIVALENT))
	{
		return;
	}
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].dPulseEquivalent = m_dPulseEquivalent;
}

bool CSetServoCtrlParam::CheckStr(int nID)
{
	CString str;
	GetDlgItemTextA(nID, str);
	int nNum = 0, nNum2 = 0;
	for (int n = 0; n < str.GetLength(); n++)
	{
		if (str.Mid(n, 1) == "。")
		{
			str.Replace("。", ".");
			SetDlgItemTextA(nID, str);
		}
		if (str.Mid(n, 1) == ".")
		{
			nNum++;
		}
		if (str.Mid(n, 1) == "-")
		{
			nNum2++;
		}
		if (((str.Mid(n, 1) < "0" || str.Mid(n, 1) > "9") && (str.Mid(n, 1) != "." && str.Mid(n, 1) != "-")) || nNum > 1 || nNum2 > 1)
		{
			UpdateData(FALSE);
			return false;
		}
	}
	if (nNum == 1 && str.Left(1) == ".")
	{
		str = "0" + str;
		SetDlgItemTextA(nID, str);
	}

	if (nNum2 == 1 && str.Left(1) != "-")
	{
		UpdateData(FALSE);
		return false;
	}
	if (str.Left(1) == "-" || str.Right(1) == "." || str == "")
	{
		return false;
	}
	return true;
}


void CSetServoCtrlParam::OnBnClickedCheck1()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	
	if (m_bEnableAbsEncoder == TRUE)
	{
		m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].bEnableAbsEncoder = true;
		m_nAbsEncoderPort = m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].nAbsEncoderPartNo;
		m_nAbsEncodernLapPulse = m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].nLapPulse;
		if (m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].nAbsEncoderDir == 1)
		{
			m_cAbsEncoderDir.SetCurSel(0);
		}
		else
		{
			m_cAbsEncoderDir.SetCurSel(1);
		}
		GetDlgItem(IDC_EDIT_ABS_PORT)->EnableWindow(TRUE);
		GetDlgItem(IDC_EDIT_ABS_LAP_PULSE)->EnableWindow(TRUE);
	}
	else
	{
		m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].bEnableAbsEncoder = false;
		m_nAbsEncoderPort = -1;
		m_nAbsEncodernLapPulse = 0;
		m_cAbsEncoderDir.SetCurSel(0);
		GetDlgItem(IDC_EDIT_ABS_PORT)->EnableWindow(FALSE);
		GetDlgItem(IDC_EDIT_ABS_LAP_PULSE)->EnableWindow(FALSE);
	}
	UpdateData(FALSE);
}


void CSetServoCtrlParam::OnEnChangeEdit24()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	if (!CheckStr(IDC_EDIT_ABS_PORT))
	{
		return;
	}
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].nAbsEncoderPartNo = m_nAbsEncoderPort;
}


void CSetServoCtrlParam::OnEnChangeEdit10()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].nLapPulse = m_nAbsEncodernLapPulse;
}


void CSetServoCtrlParam::OnBnClickedCheck2()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	if (m_bEnableSoftLimit == TRUE)
	{
		m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].bEnableSoftLimit = true;
		m_dMinCoor = m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].dNegativeLimit;
		m_dMaxCoor = m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].dPositiveLimit;
		GetDlgItem(IDC_EDIT_MIN_COOR)->EnableWindow(TRUE);
		GetDlgItem(IDC_EDIT_MAX_COOR)->EnableWindow(TRUE);
	}
	else
	{
		m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].bEnableSoftLimit = false;
		m_dMinCoor = 0;
		m_dMaxCoor = 0;
		GetDlgItem(IDC_EDIT_MIN_COOR)->EnableWindow(FALSE);
		GetDlgItem(IDC_EDIT_MAX_COOR)->EnableWindow(FALSE);
	}
	UpdateData(FALSE);
}


void CSetServoCtrlParam::OnEnChangeEdit25()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	if (!CheckStr(IDC_CHECK_ENABLE_LIMIT))
	{
		return;
	}
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].dNegativeLimit = m_dMinCoor;
}


void CSetServoCtrlParam::OnEnChangeEdit17()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	if (!CheckStr(IDC_EDIT_MAX_COOR))
	{
		return;
	}
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].dPositiveLimit = m_dMaxCoor;
}


void CSetServoCtrlParam::OnCbnSelchangeCombo6()
{
	// TODO: 在此添加控件通知处理程序代码
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].nGearFollowProfile = m_cGearFollowProfile.GetCurSel();
	if (m_cGearFollowProfile.GetCurSel() == 2)
	{
		GetDlgItem(IDC_EDIT_MASTER_NO)->EnableWindow(TRUE);
		m_nMasterAxisNo = m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].nMasterAxisNo;
	}
	else
	{
		m_nMasterAxisNo = -1;
		GetDlgItem(IDC_EDIT_MASTER_NO)->EnableWindow(FALSE);
	}
	UpdateData(FALSE);
}


void CSetServoCtrlParam::OnEnChangeEdit19()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	UpdateData(TRUE);
	if (m_nMasterAxisNo >= m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo.size() || m_nMasterAxisNo < 0)
	{
		AfxMessageBox("未找到主轴，请先添加主轴！");
		m_nMasterAxisNo = -1;
		UpdateData(FALSE);
		return;
	}
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].nMasterAxisNo = m_nMasterAxisNo;
}


void CSetServoCtrlParam::OnCbnSelchangeCombo7()
{
	// TODO: 在此添加控件通知处理程序代码
	if ( 0 == m_cAlmLogic.GetCurSel())
	{
		m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].bAlmLogic = false;
	}
	else
	{
		m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].bAlmLogic = true;
	}
}


void CSetServoCtrlParam::OnEnChangeEdit21()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	if (!CheckStr(IDC_EDIT_SPEED_MIN))
	{
		return;
	}
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].tMinSpeed.dSpeed = fabs(m_dSpeedMin);
}


void CSetServoCtrlParam::OnEnChangeEdit26()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	if (!CheckStr(IDC_EDIT_SPEED_MID))
	{
		return;
	}
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].tMidSpeed.dSpeed = fabs(m_dSpeedMid);
}


void CSetServoCtrlParam::OnEnChangeEdit27()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	if (!CheckStr(IDC_EDIT_SPEED_MAX))
	{
		return;
	}
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].tMaxSpeed.dSpeed = fabs(m_dSpeedMax);
}


void CSetServoCtrlParam::OnEnChangeEdit28()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	if (!CheckStr(IDC_EDIT_ACC_MIN))
	{
		return;
	}
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].tMinSpeed.dAcc = fabs(m_dAccMin);
}


void CSetServoCtrlParam::OnEnChangeEdit29()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	if (!CheckStr(IDC_EDIT_ACC_MID))
	{
		return;
	}
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].tMidSpeed.dAcc = fabs(m_dAccMid);
}


void CSetServoCtrlParam::OnEnChangeEdit30()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	if (!CheckStr(IDC_EDIT_ACC_MAX))
	{
		return;
	}
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].tMaxSpeed.dAcc = fabs(m_dAccMax);
}


void CSetServoCtrlParam::OnEnChangeEdit31()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	if (!CheckStr(IDC_EDIT_DEC_MIN))
	{
		return;
	}
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].tMinSpeed.dDec = fabs(m_dDecMin);
}


void CSetServoCtrlParam::OnEnChangeEdit32()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	if (!CheckStr(IDC_EDIT_DEC_MID))
	{
		return;
	}
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].tMidSpeed.dDec = fabs(m_dDecMid);
}


void CSetServoCtrlParam::OnEnChangeEdit33()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	if (!CheckStr(IDC_EDIT_DEC_MAX))
	{
		return;
	}
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].tMaxSpeed.dDec = fabs(m_dDecMax);
}


void CSetServoCtrlParam::OnCbnSelchangeCombo8()
{
	// TODO: 在此添加控件通知处理程序代码
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].nPulseMode = m_cPulseMode.GetCurSel();
}


void CSetServoCtrlParam::OnCbnSelchangeCombo9()
{
	// TODO: 在此添加控件通知处理程序代码
	if (m_cServoMotorDriver.m_vtCtrlCardInfo.size() <= m_cCardNo.GetPos() - 1)
	{
		m_cIOType.SetCurSel(0);
		return;
	}
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].tIOCtrlInfo.eIOType = (E_IO_CARD_TYPE)m_cIOType.GetCurSel();

	ShowIOInfo(m_cCardNo.GetPos() - 1);
}


void CSetServoCtrlParam::OnEnChangeEdit37()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].tIOCtrlInfo.nNoteNum = m_nNoteNum;
}


void CSetServoCtrlParam::OnEnChangeEdit38()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].tIOCtrlInfo.nPortNo = m_nPortNo;
}


void CSetServoCtrlParam::OnEnChangeEdit39()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].tIOCtrlInfo.nLocalIONum = m_nLocalIONum;
}


void CSetServoCtrlParam::OnEnChangeEdit40()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].tIOCtrlInfo.nSingleIONum = m_nSingleIONum;
}


void CSetServoCtrlParam::OnCbnSelchangeCombo10()
{
	// TODO: 在此添加控件通知处理程序代码
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].tIOCtrlInfo.nCANBaudNo = m_cBaudNo.GetCurSel();
}


void CSetServoCtrlParam::OnCbnSelchangeCombo11()
{
	// TODO: 在此添加控件通知处理程序代码
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].tIOCtrlInfo.nCATCycleTime = m_cCATCycleTime.GetCurSel();
}


void CSetServoCtrlParam::OnEnChangeEdit41()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	UpdateData(TRUE);
	m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].tIOCtrlInfo.nIOCFilter = m_nIOCFilter;
}


void CSetServoCtrlParam::OnCbnSelchangeComboAbsEncoderDir()
{
	// TODO: 在此添加控件通知处理程序代码
	if (m_cAbsEncoderDir.GetCurSel() == 0)
	{
		m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].nAbsEncoderDir = 1;
	}
	else
	{
		m_cServoMotorDriver.m_vtCtrlCardInfo[m_cCardNo.GetPos() - 1].vtAxisInfo[m_cAxisNo.GetPos() - 1].nAbsEncoderDir = -1;
	}
}
