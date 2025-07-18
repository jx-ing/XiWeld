#pragma once

#include "afxcmn.h"
#include ".\Apps\PLib\LeisaiCtrl\ServoMotorDriver.h"

// CSetServoCtrlParam 对话框

class CSetServoCtrlParam : public CDialogEx
{
	DECLARE_DYNAMIC(CSetServoCtrlParam)

public:
	CSetServoCtrlParam(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CSetServoCtrlParam();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_PARA_SERVO_CTRL };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL OnInitDialog();
	afx_msg void OnBnClickedOk();
	CSpinButtonCtrl m_cCardNo;
	CSpinButtonCtrl m_cAxisNo;

	CServoMotorDriver m_cServoMotorDriver;

	CComboBox m_cCtrlCardType;
	CComboBox m_cConnectMode;
	CComboBox m_cHardCtrlCardNo;
	CString m_strLinkParam;
	int m_nSoftCtrlCardNo;
	afx_msg void OnDeltaposSpin1(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnBnClickedButton1();

	afx_msg void OnBnClickedButton3();
	afx_msg void OnBnClickedButton2();

	//显示初始参数
	void ShowCtrlCardInfo(int nSoftCtrlCard);
	void ShowIOInfo(int nSoftCtrlCard);
	void ShowAxisInfo(int nSoftCtrlCard, int nSoftAxisNo);
	void UpdateCardNoShow(int nNum);
	void UpdateAxisNoShow(int nNum);

	//禁用控制卡控件
	void EnableCtrlCard(bool bEnable);
	//禁用IO信息控件
	void EnableIO(bool bEnable);
	//禁用轴信息控件
	void EnableAxis(bool bEnable);

	bool CheckStr(int nID);

	afx_msg void OnDeltaposSpin2(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnBnClickedButton11();
	afx_msg void OnBnClickedButton12();
	afx_msg void OnCbnSelchangeCombo3();
	afx_msg void OnCbnSelchangeCombo2();
	afx_msg void OnCbnSelchangeCombo1();
	afx_msg void OnEnChangeEdit1();
	int m_nAxis;
	std::vector<int> m_vnAxisID;
	void InitAxisRadioShow();
	void AxisRadioShow(int nNum);




	afx_msg void OnBnClickedRadioAxisNo();
	afx_msg void OnBnClickedRadio2();
	afx_msg void OnBnClickedRadio3();
	afx_msg void OnBnClickedRadio4();
	afx_msg void OnBnClickedRadio5();
	afx_msg void OnBnClickedRadio6();
	afx_msg void OnBnClickedRadio7();
	afx_msg void OnBnClickedRadio8();
	CComboBox m_cHardAxisNo;
	int m_nSoftAxisNo;
	CString m_strAxisName;
	CComboBox m_cAxisType;
	double m_dPulse;
	double m_dDistance;
	double m_dPulseEquivalent;
	BOOL m_bEnableAbsEncoder;
	int m_nAbsEncoderPort;
	int m_nAbsEncodernLapPulse;
	BOOL m_bEnableSoftLimit;
	double m_dMinCoor;
	double m_dMaxCoor;
	CComboBox m_cGearFollowProfile;
	int m_nMasterAxisNo;
	CComboBox m_cAlmLogic;
	double m_dSpeedMid;
	double m_dSpeedMin;
	double m_dSpeedMax;
	double m_dAccMin;
	double m_dAccMid;
	double m_dAccMax;
	double m_dDecMin;
	double m_dDecMid;
	double m_dDecMax;
	afx_msg void OnCbnSelchangeCombo4();
	afx_msg void OnEnChangeEdit5();
	afx_msg void OnCbnSelchangeCombo5();
	afx_msg void OnEnChangeEdit9();
	afx_msg void OnBnClickedCheck1();
	afx_msg void OnEnChangeEdit24();
	afx_msg void OnEnChangeEdit10();
	afx_msg void OnBnClickedCheck2();
	afx_msg void OnEnChangeEdit25();
	afx_msg void OnEnChangeEdit17();
	afx_msg void OnCbnSelchangeCombo6();
	afx_msg void OnEnChangeEdit19();
	afx_msg void OnCbnSelchangeCombo7();
	afx_msg void OnEnChangeEdit21();
	afx_msg void OnEnChangeEdit26();
	afx_msg void OnEnChangeEdit27();
	afx_msg void OnEnChangeEdit28();
	afx_msg void OnEnChangeEdit29();
	afx_msg void OnEnChangeEdit30();
	afx_msg void OnEnChangeEdit31();
	afx_msg void OnEnChangeEdit32();
	afx_msg void OnEnChangeEdit33();
	afx_msg void OnCbnSelchangeCombo8();
	CComboBox m_cPulseMode;
	CComboBox m_cIOType;
	int m_nNoteNum;
	int m_nPortNo;
	int m_nLocalIONum;
	int m_nSingleIONum;
	int m_nIOCFilter;
	afx_msg void OnCbnSelchangeCombo9();
	afx_msg void OnEnChangeEdit37();
	afx_msg void OnEnChangeEdit38();
	afx_msg void OnEnChangeEdit39();
	afx_msg void OnEnChangeEdit40();
	afx_msg void OnCbnSelchangeCombo10();
	afx_msg void OnCbnSelchangeCombo11();
	CComboBox m_cBaudNo;
	CComboBox m_cCATCycleTime;
	afx_msg void OnEnChangeEdit41();
	CComboBox m_cAbsEncoderDir;
	afx_msg void OnCbnSelchangeComboAbsEncoderDir();
};
