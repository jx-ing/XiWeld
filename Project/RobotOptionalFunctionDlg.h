#pragma once

#include ".\Apps\PLib\YaskawaRobot\RobotDriverAdaptor.h"

// CRobotOptionalFunctionDlg 对话框

class CRobotOptionalFunctionDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CRobotOptionalFunctionDlg)

public:
	CRobotOptionalFunctionDlg(CRobotDriverAdaptor *pRobotDriver, CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CRobotOptionalFunctionDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ADD_ROBOT };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButtonSavePVar();
	afx_msg void OnBnClickedButtonChange();
	afx_msg void OnBnClickedRadio1();
	afx_msg void OnBnClickedRadio2();
	int m_nPVarNum;
	double m_dPosX;
	double m_dPosY;
	double m_dPosZ;
	double m_dPosRX;
	double m_dPosRY;
	double m_dPosRZ;
	long m_lSPulse;
	long m_lLPulse;
	long m_lUPulse;
	long m_lRPulse;
	long m_lBPulse;
	long m_lTPulse;
	BOOL m_BType;

public:
	void SetState(bool bRightAngleState, bool bPulseState);

	CRobotDriverAdaptor *m_pYasakawaRobotDriver;
	
	virtual BOOL OnInitDialog();
};
