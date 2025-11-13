#pragma once
#include "afxdialogex.h"
#include <vector>

// ParamInput 对话框

class ParamInput : public CDialog
{
	DECLARE_DYNAMIC(ParamInput)

public:
	ParamInput(CString sTitleName, std::vector<CString> vStrName,
		std::vector<double> *vnInputData, CWnd* pParent = nullptr);   // 标准构造函数
	//void GetParam(int& nRobotNo, int& nCameraNo, int& nPtnX, int& nPtnY);
	virtual ~ParamInput();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_PARAM_INPUT};
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持
	virtual BOOL OnInitDialog();
	DECLARE_MESSAGE_MAP()
public:
	CEdit m_cEdit1;
	CEdit m_cEdit2;
	CEdit m_cEdit3;
	CEdit m_cEdit4;
	CEdit m_cEdit5;
	CEdit m_cEdit6;
	CEdit m_cEdit7;
	CEdit m_cEdit8;
	CEdit m_cEdit9;
	CEdit m_cEdit10;
	CEdit m_cEdit11;
	CEdit m_cEdit12;
	CEdit m_cEdit13;
	double m_nEdit1;
	double m_nEdit2;
	double m_nEdit3;
	double m_nEdit4;
	double m_nEdit5;
	double m_nEdit6;
	double m_nEdit7;
	double m_nEdit8;
	double m_nEdit9;
	double m_nEdit10;
	double m_nEdit11;
	double m_nEdit12;
	double m_nEdit13;
	afx_msg void OnBnClickedOk();
	std::vector<int> m_vnStaticID;
	std::vector<CEdit*> m_vpEdit;
	std::vector<double*>m_vnData;

	std::vector<CString> m_vStrName;
	std::vector<double> *m_vnInputData;

	CString m_sTitleName;
};
