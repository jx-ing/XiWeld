#pragma once
#include "afxwin.h"
#include "Apps/PLib/CtrlUnit/CUnit.h"

class CIOCtrlDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CIOCtrlDlg)

public:
	CIOCtrlDlg(std::vector<CUnit*> *pvUnit, CWnd* pParent = NULL);
	~CIOCtrlDlg();
	void Init();

#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_CTRL_IO };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);
	DECLARE_MESSAGE_MAP()

public:
	virtual BOOL OnInitDialog();
	afx_msg void OnCbnSelchangeComboRobotChoose();
	afx_msg void OnBnClickedTrackLaser();
	afx_msg void OnBnClickedLineScanLaser();
	afx_msg void OnBnClickedMeasureLaser();
	afx_msg void OnBnClickedButtonIoOpen();
	afx_msg void OnBnClickedButtonIoClose();
	afx_msg void OnBnClickedIoCheck();

	std::vector<CUnit*> *m_pvUnit;
	std::map<CString, T_IO_PARAM> m_mtShowIOParam;
	CComboBox m_bComboBoxChooseIO;
	CComboBox m_cComboBoxChooseRobot;
	int m_nCurUnitNo = 0;
};
