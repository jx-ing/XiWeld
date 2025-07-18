// XiGrooveRobotMainDlg.h : header file
//
#if !defined(AFX_XIGROOVEROBOTMAINDLG_H__BDC04702_28DF_4809_8160_A4CF52EDABFA__INCLUDED_)
#define AFX_XIGROOVEROBOTMAINDLG_H__BDC04702_28DF_4809_8160_A4CF52EDABFA__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include ".\Apps\PLib\BasicFunc\Const.h"
//#include ".\Apps\Handing\CSortingRobot.h"
//#include ".\Project\CAutoCtrlDlg.h"
#include "OpenClass\COMM\OPC\OPCClientCtrl.h"
#include ".\Apps\PLib\Database\CDatabaseCtrl.h"
#include ".\Project\AssemblyWeld.h"

//#include "Test.h"
/////////////////////////////////////////////////////////////////////////////
// CXiGrooveRobotMainDlg dialog

class CXiGrooveRobotMainDlg : public CDialog
{
// Construction
public:
    CXiGrooveRobotMainDlg(CWnd* pParent = NULL);	// standard constructor
    ~CXiGrooveRobotMainDlg();
	bool m_bRgbInit;
	int m_nCameraNum;
// Dialog Data
	//{{AFX_DATA(CXiGrooveRobotMainDlg)
	enum { IDD = IDD_MAIN_XIGROOVEROBOT };
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CXiGrooveRobotMainDlg)
	public:
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation

protected:
	HICON m_hIcon;

	// Generated message map functions
	//{{AFX_MSG(CXiGrooveRobotMainDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();  
	afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
	virtual void OnCancel();
	afx_msg void OnBtnPartBevelCut();
	afx_msg void OnBnClickedCancel();
	afx_msg void OnBnClickedButtonSetPara();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
		
private:
    void SetToolTip(); 
    void ShowTime();

private:		
    CToolTipCtrl m_cButtonHint; // 鼠标停留 提示信息    
    CBrush m_cBckBrush;
    TCHAR szFilePath[MAX_PATH + 1];
	BOOL InitCtrlCard();

	CServoMotorDriver* m_pCtrlCardDriver = NULL;
	//CAutoCtrlDlg *m_pcAutoCtrlDlg = NULL;
	CAssemblyWeld* m_pcAssemblyWeld = NULL;
public:
	bool CheckFileAndPathExists();			//检查文件路径，文件缺失等问题
	void CreatFileAndPath();
	bool CheckFile(CString strFile);

	afx_msg void OnBnClickedButtonSetPara2();
	void LoadData(CString& strIP, CString& strUser, CString& strKey, CString& strDBName, int& nPort);
	afx_msg void OnTimer(UINT_PTR nIDEvent);

	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton3();
	afx_msg void OnBnClickedButton8();

	//OPC

	COPCClientCtrl *m_cOPCClientCtrl = NULL;
	afx_msg void OnCbnSelchangeCombo2();
	void LoadLanguge();
	CComboBox m_combLanguageChose;
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_XIGROOVEROBOTMAINDLG_H__BDC04702_28DF_4809_8160_A4CF52EDABFA__INCLUDED_)
