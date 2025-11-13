#pragma once
#include "afxdialogex.h"
#include ".\Apps\PLib\BasicFunc\Const.h"
#include ".\OpenClass\FileOP\xml\tinyxml2.h"

#if ENABLE_MY_SQL

// CSetCommonPara 对话框

class CSetCommonPara : public CDialogEx
{
	DECLARE_DYNAMIC(CSetCommonPara)

public:
	CSetCommonPara(CWnd* pParent = nullptr);   // 标准构造函数
	virtual ~CSetCommonPara();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_DIALOG_SET_COMMOM_PARA };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:

	std::vector<CString> m_vstrNodeName;
	std::map<CString, T_COMMON_TABLE> m_mtAllPara;//所有参数

	/// <summary>
	/// 数据库
	/// </summary>
	CCommonParaByDatabase m_cCommonPara;
	std::vector< T_COMMON_TABLE> m_vtAllPara;//所有参数
	std::map<CString, std::vector< T_COMMON_TABLE>> m_mvtAllPara;//所有参数
	bool GetAllPara(std::map<CString, std::vector< T_COMMON_TABLE>> &mvtAllPara);//得到所有参数

	/// <summary>
	/// 界面显示
	/// </summary>
	std::map< HTREEITEM, HTREEITEM> m_mhAfterNode;
	std::map < CString, HTREEITEM> m_mhNode;
	void ShowAllPara(std::vector<T_COMMON_TABLE> vtAllPara);
	CString GetNode(std::vector<CString> vstrNode, int n);
	//显示所有参数
	CString GetAllNodeText(HTREEITEM hHandle);//获取所又上层节点名
	void ShowCommonPara(T_COMMON_TABLE tCommomPara);//显示参数
	CString GetParaTipInfo(T_COMMON_TABLE tCommomPara);//显示提示信息

	//数据修改
	T_COMMON_TABLE* m_ptCurPara;//当前显示的参数
	void SetCommonPara(T_COMMON_TABLE *ptCommomPara);
	//设置参数
	CString GetChangeInfo(T_COMMON_TABLE tNewPara, T_COMMON_TABLE tOldPara);



private:
	CString m_strBackupVal1;
	CString m_strBackupVal2;
	CString m_strBackupVal3;
	CString m_strBackupVal4;
	CString m_strBackupVal5;
	CString m_strBackupVal6;
	CString m_strBackupVal7;
	CString m_strBackupVal8;
	CString m_strBackupVal9;
	CString m_strDescribe;
	CString m_strName;
	CString m_strStation;
	CString m_strVal1;
	CString m_strVal2;
	CString m_strVal3;
	CString m_strVal4;
	CString m_strVal5;
	CString m_strVal6;
	CString m_strVal7;
	CString m_strVal8;
	CString m_strVal9;
	CString m_strType;
	virtual BOOL OnInitDialog();
	CTreeCtrl m_cCommonParaTree;
	afx_msg void OnTvnSelchangedTreeCommonPara(NMHDR* pNMHDR, LRESULT* pResult);
	afx_msg void OnGetinfotipTreeCommonPara(NMHDR* pNMHDR, LRESULT* pResult);
	afx_msg void OnBnClickedOk();
	tinyxml2::XMLElement* SetParaNode(CString strPara, CString strParaType);
	void GetAllNode(HTREEITEM hHandle, tinyxml2::XMLElement* xmlNode);
public:
	afx_msg void OnBnClickedOk2();

	void LoadAllNode(CString strNodeName, tinyxml2::XMLElement* xmlNode);

	tinyxml2::XMLDocument m_xmlSavePara;
	tinyxml2::XMLDocument m_xmlLoadPara;
	afx_msg
		void LoadAllNode(tinyxml2::XMLElement* xmlNode);
	void OnBnClickedOk3();
};

#endif