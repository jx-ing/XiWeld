// CSetCommonPara.cpp: 实现文件
//

#include "stdafx.h"
#include ".\Project\XiGrooveRobot.h"
#include "afxdialogex.h"
#include "CSetCommonPara.h"

#if ENABLE_MY_SQL

// CSetCommonPara 对话框

IMPLEMENT_DYNAMIC(CSetCommonPara, CDialogEx)

CSetCommonPara::CSetCommonPara(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_DIALOG_SET_COMMOM_PARA, pParent)
	, m_strBackupVal1(_T(""))
	, m_strBackupVal2(_T(""))
	, m_strBackupVal3(_T(""))
	, m_strBackupVal4(_T(""))
	, m_strBackupVal5(_T(""))
	, m_strBackupVal6(_T(""))
	, m_strBackupVal7(_T(""))
	, m_strBackupVal8(_T(""))
	, m_strBackupVal9(_T(""))
	, m_strDescribe(_T(""))
	, m_strName(_T(""))
	, m_strStation(_T(""))
	, m_strVal1(_T(""))
	, m_strVal2(_T(""))
	, m_strVal3(_T(""))
	, m_strVal4(_T(""))
	, m_strVal5(_T(""))
	, m_strVal6(_T(""))
	, m_strVal7(_T(""))
	, m_strVal8(_T(""))
	, m_strVal9(_T(""))
	, m_strType(_T(""))
{

}

CSetCommonPara::~CSetCommonPara()
{
}

void CSetCommonPara::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_TREE_COMMON_PARA, m_cCommonParaTree);
	DDX_Text(pDX, IDC_EDIT_BACKUP_VAL1, m_strBackupVal1);
	DDX_Text(pDX, IDC_EDIT_BACKUP_VAL2, m_strBackupVal2);
	DDX_Text(pDX, IDC_EDIT_BACKUP_VAL3, m_strBackupVal3);
	DDX_Text(pDX, IDC_EDIT_BACKUP_VAL4, m_strBackupVal4);
	DDX_Text(pDX, IDC_EDIT_BACKUP_VAL5, m_strBackupVal5);
	DDX_Text(pDX, IDC_EDIT_BACKUP_VAL6, m_strBackupVal6);
	DDX_Text(pDX, IDC_EDIT_BACKUP_VAL7, m_strBackupVal7);
	DDX_Text(pDX, IDC_EDIT_BACKUP_VAL8, m_strBackupVal8);
	DDX_Text(pDX, IDC_EDIT_BACKUP_VAL9, m_strBackupVal9);
	DDX_Text(pDX, IDC_EDIT_DESCRIBE, m_strDescribe);
	DDX_Text(pDX, IDC_EDIT_NAME, m_strName);
	DDX_Text(pDX, IDC_EDIT_STATION, m_strStation);
	DDX_Text(pDX, IDC_EDIT_VAL1, m_strVal1);
	DDX_Text(pDX, IDC_EDIT_VAL2, m_strVal2);
	DDX_Text(pDX, IDC_EDIT_VAL3, m_strVal3);
	DDX_Text(pDX, IDC_EDIT_VAL4, m_strVal4);
	DDX_Text(pDX, IDC_EDIT_VAL5, m_strVal5);
	DDX_Text(pDX, IDC_EDIT_VAL6, m_strVal6);
	DDX_Text(pDX, IDC_EDIT_VAL7, m_strVal7);
	DDX_Text(pDX, IDC_EDIT_VAL8, m_strVal8);
	DDX_Text(pDX, IDC_EDIT_VAL9, m_strVal9);
	DDX_Text(pDX, IDC_EDIT_TYPE, m_strType);
}


BEGIN_MESSAGE_MAP(CSetCommonPara, CDialogEx)
	ON_NOTIFY(TVN_SELCHANGED, IDC_TREE_COMMON_PARA, &CSetCommonPara::OnTvnSelchangedTreeCommonPara)
	ON_NOTIFY(TVN_GETINFOTIP, IDC_TREE_COMMON_PARA, &CSetCommonPara::OnGetinfotipTreeCommonPara)
	ON_BN_CLICKED(IDOK, &CSetCommonPara::OnBnClickedOk)
	ON_BN_CLICKED(IDOK2, &CSetCommonPara::OnBnClickedOk2)
	ON_BN_CLICKED(IDOK3, &CSetCommonPara::OnBnClickedOk3)
END_MESSAGE_MAP()


// CSetCommonPara 消息处理程序


BOOL CSetCommonPara::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  在此添加额外的初始化
	GetAllPara(m_mvtAllPara);
	ShowAllPara(m_vtAllPara);

	//已修改
	XUI::Languge::GetInstance().translateDialog(this);
	return TRUE;  // return TRUE unless you set the focus to a control
	// 异常: OCX 属性页应返回 FALSE
}


void CSetCommonPara::OnTvnSelchangedTreeCommonPara(NMHDR* pNMHDR, LRESULT* pResult)
{
	LPNMTREEVIEW pNMTreeView = reinterpret_cast<LPNMTREEVIEW>(pNMHDR);
	// TODO: 在此添加控件通知处理程序代码

	HTREEITEM hHandle = m_cCommonParaTree.GetSelectedItem();
	CString strItemText = m_cCommonParaTree.GetItemText(hHandle);
	CString strNodeText = GetAllNodeText(hHandle);
	if (m_mvtAllPara.find(strNodeText) != m_mvtAllPara.end())
	{
		for (size_t i = 0; i < m_mvtAllPara[strNodeText].size(); i++)
		{
			if (m_mvtAllPara[strNodeText][i].strParaName == strItemText)
			{
				m_ptCurPara = &m_mvtAllPara[strNodeText][i];
				ShowCommonPara(m_mvtAllPara[strNodeText][i]);
				break;
			}
		}
	}
	*pResult = 0;
}

void CSetCommonPara::OnGetinfotipTreeCommonPara(NMHDR* pNMHDR, LRESULT* pResult)
{
	LPNMTVGETINFOTIP pGetInfoTip = reinterpret_cast<LPNMTVGETINFOTIP>(pNMHDR);
	// TODO: 在此添加控件通知处理程序代码
	HTREEITEM hHandle = pGetInfoTip->hItem;
	CString strItemText = m_cCommonParaTree.GetItemText(hHandle);
	CString strNodeText = GetAllNodeText(hHandle);
	if (m_mvtAllPara.find(strNodeText) != m_mvtAllPara.end())
	{
		for (size_t i = 0; i < m_mvtAllPara[strNodeText].size(); i++)
		{
			if (m_mvtAllPara[strNodeText][i].strParaName == strItemText)
			{
				strcpy(pGetInfoTip->pszText, GetParaTipInfo(m_mvtAllPara[strNodeText][i]).GetBuffer());
				break;
			}
		}
	}
	*pResult = 0;
}

bool CSetCommonPara::GetAllPara(std::map<CString, std::vector< T_COMMON_TABLE>>& mvtAllPara)
{
	mvtAllPara.clear();
	std::vector< T_COMMON_TABLE> vtAllPara;
	if (FALSE == m_cCommonPara.ReadCommonPara(vtAllPara))
		return false;
	if (vtAllPara.empty())
	{
		return false;
	}
	for (size_t i = 0; i < vtAllPara.size(); i++)
	{
		m_vtAllPara.push_back(vtAllPara[i]);
		mvtAllPara[vtAllPara[i].strParaStation].push_back(vtAllPara[i]);
	}
	return true;
}

void CSetCommonPara::ShowAllPara(std::vector<T_COMMON_TABLE> vtAllPara)
{
	m_cCommonParaTree.DeleteAllItems();
	m_mhAfterNode.clear();
	m_mhNode.clear();
	for (size_t i = 0; i < vtAllPara.size(); i++)
	{
		std::vector<CString> vstrNode = TokenizeCString(vtAllPara[i].strParaStation, STATION_LINK);
		if (vstrNode.empty())
		{
			continue;
		}
		HTREEITEM hRoot = TVI_ROOT;
		for (size_t i = 0; i < vstrNode.size(); i++)
		{
			if (vstrNode[i] == "null")
			{
				vstrNode[i] = "未定义节点";
			}
			CString strNode = GetNode(vstrNode, i);
			if (m_mhNode.find(strNode) == m_mhNode.end())
			{
				hRoot = m_cCommonParaTree.InsertItem(vstrNode[i], hRoot);
				m_mhNode[strNode] = hRoot;
			}
			hRoot = m_mhNode[strNode];
		}
		m_mhAfterNode[hRoot] = m_cCommonParaTree.InsertItem(vtAllPara[i].strParaName, hRoot, m_mhAfterNode[hRoot]);
	}
}

CString CSetCommonPara::GetNode(std::vector<CString> vstrNode, int n)
{
	if (n < 0 || n >= vstrNode.size())
	{
		return "";
	}
	CString strNode = vstrNode[0];
	for (size_t i = 1; i <= n; i++)
	{
		strNode += "-" + vstrNode[i];
	}
	return strNode;
}

CString CSetCommonPara::GetAllNodeText(HTREEITEM hHandle)
{
	CString strAllNodeText;
	while (hHandle != NULL)
	{
		hHandle = m_cCommonParaTree.GetParentItem(hHandle);
		if (hHandle == NULL)
		{
			break;
		}
		CString strNodeText = m_cCommonParaTree.GetItemText(hHandle);
		if (strNodeText == "未定义节点")
		{
			strNodeText = "null";
		}
		strAllNodeText = STATION_LINK + strNodeText + strAllNodeText;
	}
	int nLenght = strAllNodeText.GetLength();
	return strAllNodeText.Right(nLenght-1);
}

void CSetCommonPara::ShowCommonPara(T_COMMON_TABLE tCommomPara)
{
	m_strBackupVal1 = tCommomPara.astrParaBackups[0];
	m_strBackupVal2 = tCommomPara.astrParaBackups[1];
	m_strBackupVal3 = tCommomPara.astrParaBackups[2];
	m_strBackupVal4 = tCommomPara.astrParaBackups[3];
	m_strBackupVal5 = tCommomPara.astrParaBackups[4];
	m_strBackupVal6 = tCommomPara.astrParaBackups[5];
	m_strBackupVal7 = tCommomPara.astrParaBackups[6];
	m_strBackupVal8 = tCommomPara.astrParaBackups[7];
	m_strBackupVal9 = tCommomPara.astrParaBackups[8];
	m_strDescribe = tCommomPara.strParaDescribe;
	m_strName = tCommomPara.strParaName;
	m_strType = tCommomPara.strParaType;
	m_strStation = tCommomPara.strParaStation;
	m_strVal1 = tCommomPara.astrPara[0];
	m_strVal2 = tCommomPara.astrPara[1];
	m_strVal3 = tCommomPara.astrPara[2];
	m_strVal4 = tCommomPara.astrPara[3];
	m_strVal5 = tCommomPara.astrPara[4];
	m_strVal6 = tCommomPara.astrPara[5];
	m_strVal7 = tCommomPara.astrPara[6];
	m_strVal8 = tCommomPara.astrPara[7];
	m_strVal9 = tCommomPara.astrPara[8];
	UpdateData(FALSE);
}

CString CSetCommonPara::GetParaTipInfo(T_COMMON_TABLE tCommomPara)
{
	CString strParaTipInfo;
	strParaTipInfo += "工件名：" + tCommomPara.strParaName;
	strParaTipInfo += "\n说   明：" + tCommomPara.strParaDescribe;
	strParaTipInfo += "\n工件值：";
	for (size_t i = 0; i < COMMON_TABLE_DATA_SIZE; i++)
	{
		if (tCommomPara.astrPara[i] == "")
		{
			continue;
		}
		strParaTipInfo += "\n	" + tCommomPara.astrPara[i];
	}
	return strParaTipInfo;
}

void CSetCommonPara::SetCommonPara(T_COMMON_TABLE* ptCommomPara)
{
	ptCommomPara->astrParaBackups[0] = ptCommomPara->astrPara[0];
	ptCommomPara->astrParaBackups[1] = ptCommomPara->astrPara[1];
	ptCommomPara->astrParaBackups[2] = ptCommomPara->astrPara[2];
	ptCommomPara->astrParaBackups[3] = ptCommomPara->astrPara[3];
	ptCommomPara->astrParaBackups[4] = ptCommomPara->astrPara[4];
	ptCommomPara->astrParaBackups[5] = ptCommomPara->astrPara[5];
	ptCommomPara->astrParaBackups[6] = ptCommomPara->astrPara[6];
	ptCommomPara->astrParaBackups[7] = ptCommomPara->astrPara[7];
	ptCommomPara->astrParaBackups[8] = ptCommomPara->astrPara[8];
	ptCommomPara->astrPara[0] = m_strVal1;
	ptCommomPara->astrPara[1] = m_strVal2;
	ptCommomPara->astrPara[2] = m_strVal3;
	ptCommomPara->astrPara[3] = m_strVal4;
	ptCommomPara->astrPara[4] = m_strVal5;
	ptCommomPara->astrPara[5] = m_strVal6;
	ptCommomPara->astrPara[6] = m_strVal7;
	ptCommomPara->astrPara[7] = m_strVal8;
	ptCommomPara->astrPara[8] = m_strVal9;
}

CString CSetCommonPara::GetChangeInfo(T_COMMON_TABLE tNewPara, T_COMMON_TABLE tOldPara)
{
	CString strChangeInfo;
	strChangeInfo += "是否修改：";
	for (size_t i = 0; i < COMMON_TABLE_DATA_SIZE; i++)
	{
		if (tNewPara.astrPara[i] == "")
		{
			continue;
		}
		strChangeInfo += "\n" + tOldPara.astrPara[i] + "-->" + tNewPara.astrPara[i];
	}
	return strChangeInfo;
}

void CSetCommonPara::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	T_COMMON_TABLE tCommomPara = *m_ptCurPara;
	SetCommonPara(&tCommomPara);
	std::vector<CString> vstrPara = SetPara(tCommomPara);
	tCommomPara = GetPara(tCommomPara.strParaStation, tCommomPara.strParaName, tCommomPara.strParaType, vstrPara, tCommomPara.strParaDescribe);
	if (IDOK != XiMessageBox(GetChangeInfo(tCommomPara, *m_ptCurPara)))
	{
		return;
	}

	SetCommonPara(m_ptCurPara);
	m_cCommonPara.SetCommonPara(*m_ptCurPara);
	ShowCommonPara(*m_ptCurPara);
}

tinyxml2::XMLElement* CSetCommonPara::SetParaNode(CString strPara, CString strParaType)
{
	tinyxml2::XMLElement* paraNodeStation = m_xmlSavePara.NewElement(strParaType);
	tinyxml2::XMLText* xmlTextStation = m_xmlSavePara.NewText(strPara);
	paraNodeStation->InsertFirstChild(xmlTextStation);
	return paraNodeStation;
}

void CSetCommonPara::GetAllNode(HTREEITEM hHandle, tinyxml2::XMLElement* xmlNode)
{
	HTREEITEM temp = m_cCommonParaTree.GetChildItem(hHandle);
	while(true)
	{
		if (temp == NULL)
		{
			CString strItemText = m_cCommonParaTree.GetItemText(hHandle);
			CString strNodeText = GetAllNodeText(hHandle);
			if (m_mvtAllPara.find(strNodeText) != m_mvtAllPara.end())
			{
				for (size_t i = 0; i < m_mvtAllPara[strNodeText].size(); i++)
				{
					if (m_mvtAllPara[strNodeText][i].strParaName == strItemText)
					{
						xmlNode->InsertEndChild(SetParaNode(m_mvtAllPara[strNodeText][i].strParaStation, "ParaStation"));
						xmlNode->InsertEndChild(SetParaNode(m_mvtAllPara[strNodeText][i].strParaName, "ParaName"));
						xmlNode->InsertEndChild(SetParaNode(m_mvtAllPara[strNodeText][i].strParaType, "ParaType"));
						xmlNode->InsertEndChild(SetParaNode(m_mvtAllPara[strNodeText][i].strParaDescribe, "ParaDescribe"));
						tinyxml2::XMLElement* valNode = m_xmlSavePara.NewElement("Para");
						tinyxml2::XMLElement* valBackupsNode = m_xmlSavePara.NewElement("BackupsPara");
						for (size_t j = 0; j < COMMON_TABLE_DATA_SIZE; j++)
						{
							tinyxml2::XMLText* xmlTextVal = m_xmlSavePara.NewText(m_mvtAllPara[strNodeText][i].astrPara[j]);
							if (!m_mvtAllPara[strNodeText][i].astrPara[j].IsEmpty())
							{
								if (j != 0)
								{
									tinyxml2::XMLText* xmlTextLink1 = m_xmlSavePara.NewText(" ");
									valNode->InsertEndChild(xmlTextLink1);
								}
								valNode->InsertEndChild(xmlTextVal);
							}
							tinyxml2::XMLText* xmlTextBackupsVal = m_xmlSavePara.NewText(m_mvtAllPara[strNodeText][i].astrParaBackups[j]);
							if (!m_mvtAllPara[strNodeText][i].astrParaBackups[j].IsEmpty())
							{
								if (j != 0)
								{
									tinyxml2::XMLText* xmlTextLink2 = m_xmlSavePara.NewText(" ");
									valBackupsNode->InsertEndChild(xmlTextLink2);
								}
								tinyxml2::XMLText* xmlTextLink2 = m_xmlSavePara.NewText(" ");
								valBackupsNode->InsertEndChild(xmlTextBackupsVal);
								valBackupsNode->InsertEndChild(xmlTextLink2);
							}
						}
						xmlNode->InsertEndChild(valNode);
						xmlNode->InsertEndChild(valBackupsNode);
						break;
					}
				}
			}
			break;
		}
		else
		{
			CString str = m_cCommonParaTree.GetItemText(temp);
			str = Replace(str, '[', '-');
			str = Replace(str, ']', NULL);
			tinyxml2::XMLElement* rootNode = m_xmlSavePara.NewElement(str);
			xmlNode->InsertEndChild(rootNode);
			GetAllNode(temp, rootNode);
		}
		temp = m_cCommonParaTree.GetNextSiblingItem(temp);
	}
}

void CSetCommonPara::OnBnClickedOk2()
{
	// TODO: 在此添加控件通知处理程序代码
	CString strFilePath = SaveFileDlg(this, "xml", "ALL_COMMON_PARA", NULL);
	if (GetPath(strFilePath).IsEmpty())
	{
		return;
	}
	//插入声明
	tinyxml2::XMLDeclaration* declaration = m_xmlSavePara.NewDeclaration("xml version=\"1.0\" encoding=\"ANSI\"");
	m_xmlSavePara.InsertFirstChild(declaration);

	//插入ip地址
	tinyxml2::XMLElement* valNode = m_xmlSavePara.NewElement("TableName");
	tinyxml2::XMLText* xmlTextVal = m_xmlSavePara.NewText(m_cCommonPara.GetTableName());
	valNode->InsertEndChild(xmlTextVal);
	m_xmlSavePara.InsertEndChild(valNode);

	////插入根节点
	HTREEITEM temp = m_cCommonParaTree.GetFirstVisibleItem();
	while(true)
	{
		if (temp == NULL)
		{
			break;
		}
		tinyxml2::XMLElement* root = m_xmlSavePara.NewElement(m_cCommonParaTree.GetItemText(temp));
		GetAllNode(temp, root);
		m_xmlSavePara.InsertEndChild(root);
		temp = m_cCommonParaTree.GetNextVisibleItem(temp);
	}
	m_xmlSavePara.SaveFile(strFilePath);
	m_xmlSavePara.Clear();
	AfxMessageBox("参数导出完成");
}

void CSetCommonPara::LoadAllNode(tinyxml2::XMLElement* xmlNode)
{
	xmlNode = xmlNode->FirstChildElement();
	T_COMMON_TABLE tCommomPara;
	while (xmlNode)
	{
		if (!strcmp(xmlNode->Name(),"ParaStation" ))
		{
			tCommomPara.strParaStation = xmlNode->GetText();
		}
		else if (!strcmp(xmlNode->Name(), "ParaName"))
		{
			tCommomPara.strParaName = xmlNode->GetText();
		}
		else if (!strcmp(xmlNode->Name(), "ParaType"))
		{
			tCommomPara.strParaType = xmlNode->GetText();
		}
		else if (!strcmp(xmlNode->Name(), "ParaDescribe"))
		{
			tCommomPara.strParaDescribe = xmlNode->GetText();
		}
		else if (!strcmp(xmlNode->Name(), "Para"))
		{
			CString strPara = xmlNode->GetText();
			std::vector<CString> vstrPara = TokenizeCString(strPara, " "); 
			for (size_t i = 0; i < COMMON_TABLE_DATA_SIZE; i++)
			{
				if (i >= vstrPara.size())
				{
					break;
				}
				tCommomPara.astrPara[i] = vstrPara[i];
			}
		}
		else if (!strcmp(xmlNode->Name(), "BackupsPara"))
		{
			CString strPara = xmlNode->GetText();
			std::vector<CString> vstrPara = TokenizeCString(strPara, " ");
			for (size_t i = 0; i < COMMON_TABLE_DATA_SIZE; i++)
			{
				if (i >= vstrPara.size())
				{
					break;
				}
				tCommomPara.astrParaBackups[i] = strPara[i];
			}
		}
		xmlNode = xmlNode->NextSiblingElement();
	}
	if (tCommomPara.strParaStation.IsEmpty())
	{
		return;
	}
	m_vtAllPara.push_back(tCommomPara);
	m_mvtAllPara[tCommomPara.strParaStation].push_back(tCommomPara);
}

void CSetCommonPara::LoadAllNode(CString strNodeName, tinyxml2::XMLElement* xmlNode)
{
	tinyxml2::XMLElement* xmlStartNode = xmlNode;
	xmlNode = xmlNode->FirstChildElement();
	while (true)
	{
		if (xmlNode == NULL)
		{
			break;
		}
		else
		{
			xmlStartNode = xmlNode;
			if (xmlNode->FirstChildElement() != NULL)
			{
				CString str = xmlNode->FirstChildElement()->Name();
				if (xmlNode->FirstChildElement()->FirstChildElement() == NULL)
				{
					LoadAllNode(xmlNode);
					xmlNode = xmlStartNode->NextSiblingElement();
					continue;
				}
			}
			CString str = strNodeName + ";" + xmlNode->Name();
			LoadAllNode(str, xmlNode);
		}
		xmlNode = xmlStartNode->NextSiblingElement();
	}
}

void CSetCommonPara::OnBnClickedOk3()
{
	// TODO: 在此添加控件通知处理程序代码
	CString strFilePath = OpenFileDlg(this, "xml" , NULL);
	if (GetPath(strFilePath).IsEmpty())
	{
		return;
	}

	//导入xml文件
	if (m_xmlLoadPara.LoadFile(strFilePath) != tinyxml2::XML_SUCCESS)
	{
		return;
	}

	//判断头文件是否为空
	tinyxml2::XMLElement* rootNode = m_xmlLoadPara.FirstChildElement();
	if (rootNode == NULL)
	{
		return;
	}

	//清空原始数据
	m_vtAllPara.clear();
	m_mvtAllPara.clear();

	CString str = rootNode->Name();
	if (str != "TableName")
	{
		return;
	}
	str = rootNode->GetText();
	while (true)
	{
		if (rootNode == NULL)
		{
			break;
		}
		LoadAllNode(rootNode->Name(), rootNode);
		rootNode = rootNode->NextSiblingElement();
	}

	CCommonParaByDatabase cCommonPara(str);
	for (size_t i = 0; i < m_vtAllPara.size(); i++)
	{
		cCommonPara.SetCommonPara(m_vtAllPara[i]);
	}
	ShowAllPara(m_vtAllPara);
	m_xmlLoadPara.Clear();
	AfxMessageBox("参数导入完成");
}
#endif