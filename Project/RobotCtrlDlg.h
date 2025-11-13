#if !defined(AFX_DLGTESTROBOTCOMM_H__F71A20CB_D57B_49F4_933A_706706B85BF4__INCLUDED_)
#define AFX_DLGTESTROBOTCOMM_H__F71A20CB_D57B_49F4_933A_706706B85BF4__INCLUDED_

#include ".\Apps\PLib\YaskawaRobot\RobotDriverAdaptor.h"
#include "afxwin.h"
#include ".\Apps\PLib\CtrlUnit\CUnit.h"

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// DlgTestRobotComm.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CDlgTestRobotComm dialog

class CRobotCtrlDlg : public CDialog
{
// Construction
public:
	CRobotCtrlDlg(std::vector < CUnit*> vpUnit, CWnd* pParent = NULL);
	CRobotCtrlDlg(std::vector<CRobotDriverAdaptor*> vpRobotDriver, CWnd* pParent = NULL);   // standard constructor
	CRobotCtrlDlg(CRobotDriverAdaptor *pRobotDriver, CWnd* pParent = NULL);

public:	
	std::vector<CRobotDriverAdaptor*> m_vpRobotDriver;
	CRobotDriverAdaptor *m_pCurRobotDriver;

// Dialog Data
	//{{AFX_DATA(CDlgTestRobotComm)
	enum { IDD = IDD_CTRL_ROBOT };
	int		m_nRobotPosMoveMode;
	int		m_nRobotPulseMoveMode;
	int		m_nPosAxis;
	int		m_nPulseAxis;
	double	m_dPosMoveSpeed;
	int		m_nPulseMoveSpeed;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CDlgTestRobotComm)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CDlgTestRobotComm)
	virtual BOOL OnInitDialog();
	afx_msg void OnClose();
	afx_msg void OnBtnPosMove();
	afx_msg void OnBtnPulseMove();
	afx_msg void OnBtnHoldOn();
	afx_msg void OnBtnHoldOff();
	afx_msg void OnBtnServoOn();
	afx_msg void OnBtnServoOff();
	afx_msg void OnCheckReverse1();
	afx_msg void OnCheckReverse2();	
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

private:
	void Init();
	void LoadRobotName();
	void ShowRobotCoor();
	void CheckPulseAxis();
	void CheckPosAxis();	
public:
	afx_msg void OnBnClickedButtonDebug();
	CComboBox m_cRobot;
	afx_msg void OnCbnSelchangeComboRobot();
	afx_msg void OnBnClickedButtonBackHome();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DLGTESTROBOTCOMM_H__F71A20CB_D57B_49F4_933A_706706B85BF4__INCLUDED_)
