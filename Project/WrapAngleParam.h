#pragma once
#include "afxwin.h"


// CWrapAngleParam 对话框

class CWrapAngleParam : public CDialog
{
	DECLARE_DYNAMIC(CWrapAngleParam)

public:
	CWrapAngleParam(CString sUnitName, CWnd * pParent = NULL);   // 标准构造函数
	virtual ~CWrapAngleParam();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_DIA_WARP_PARAM };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
    afx_msg void OnBnClickedOk();

public:
    virtual BOOL OnInitDialog();
    virtual BOOL PreTranslateMessage(MSG* pMsg);
    afx_msg void OnCbnSelchangeComboxWrapType();
    afx_msg void OnBnClickedButtonDelete();
    afx_msg void OnBnClickedButtonAdd();
	afx_msg void OnBnClickedCheckWrapType();
	afx_msg void OnBnClickedCheckWrapRobot();
    void SaveWrapAngleParam(int nRobotNo);
    void SaveWrapAngleParam2(int nRobotNo, T_WELD_WRAP_ANGLE tWeldWrapAngle);
    void RefreshWrapAngleParam(WELD_WRAP_ANGLE tWrapAngelPara);
    void ReadWeldWrapParem();
    void RecordWeldWrapParem();
    WELD_WRAP_ANGLE DownloadWrapAngleParam();
    WELD_WRAP_ANGLE InitWrapAngleParam(int nRobotNo = 0);
    T_WELD_WRAP_ANGLE  AutoSelectWrapParem(int nRobotNo,double dWorkpieceThink = 8, int nWrapType = 1);

    std::vector<T_WELD_WRAP_ANGLE> m_vtWrapAngel;
    CComboBox m_ComboxWrapType;
    CString m_strWrapAngleType;
    CString m_strWrapAngleNew;
    CString m_sUnitName;
	BOOL m_bWrapType;
	BOOL m_bRobotNo;
};
