#include "stdafx.h"
#include "Languge.h"

namespace XUI
{
	Languge Languge::m_instance;
	constexpr UINT g_nChineseLanguageID = MAKELANGID(LANG_CHINESE, SUBLANG_CHINESE_SIMPLIFIED);

	bool Languge::loadAllLanguge(CString sLangugeFile)
	{
		//打开表格
		std::map<CString, std::vector<CString>> mvsSettings;
		m_mvsLanguge.clear();
		OpenExcel cOpenExcel;
		if (!cOpenExcel.Init(XiBase::GetSystemLog())
			|| !cOpenExcel.Open(sLangugeFile)
			|| !cOpenExcel.Read("设置", mvsSettings)
			|| !cOpenExcel.Read("翻译", m_mvsLanguge)
			|| !cOpenExcel.Close())
		{
			XUI::MesBox::PopError((const char*)cOpenExcel.GetErrorString());
			return false;
		}

		loadSettings(mvsSettings);

		return true;
	}

	std::vector<CString> Languge::getLangugeList()
	{
		std::vector<CString> vsLangugeList;
		for (size_t i = 0; i < m_vsLangugeList.size(); i++)
		{
			if (m_vsLangugeList[i].bEnable)
			{
				vsLangugeList.push_back(m_vsLangugeList[i].sName);
			}
		}
		return vsLangugeList;
	}

	bool Languge::chioceLanguge(CString sLanguge)
	{
		m_nCurLangugeNo = -1;

		//中文不需要翻译
		if (sLanguge == "中文")
		{
			return true;
		}

		for (size_t i = 0; i < m_vsLangugeList.size(); i++)
		{
			if (m_vsLangugeList[i].bEnable
				&& m_vsLangugeList[i].sName == sLanguge)
			{
				m_nCurLangugeNo = i;
				return true;
			}
		}
		return false;
	}

	UINT Languge::getCurLangugeID()
	{		
		//不存在的语言序号
		if (m_nCurLangugeNo < 0
			|| m_nCurLangugeNo >= m_vsLangugeList.size())
		{
			return g_nChineseLanguageID;
		}

		return m_vsLangugeList[m_nCurLangugeNo].nLangugeID;
	}

	CString Languge::translate(CString srtString)
	{
		//不存在的语言序号，不翻译
		if (m_nCurLangugeNo < 0
			|| m_nCurLangugeNo >= m_vsLangugeList.size())
		{
			return srtString;
		}

		auto iter = m_mvsLanguge.find(srtString);
		if (iter == m_mvsLanguge.end()//没有这个字符串的翻译
			|| iter->second[m_nCurLangugeNo].IsEmpty())//这个字符串的翻译为空
		{
			return srtString;
		}

		return iter->second[m_nCurLangugeNo];
	}

	void Languge::translateDialog(CDialog* pDlg)
	{
		if (!pDlg)
		{
			return;
		}

		CString sTitle;
		pDlg->GetWindowText(sTitle);
		if (!sTitle.IsEmpty())
			pDlg->SetWindowText(translate(sTitle));

		char str[1024];
		HWND hwndChild = ::GetWindow(pDlg->m_hWnd, GW_CHILD);  //列出所有控件  
		while (hwndChild)
		{
			GetWindowText(hwndChild, str, 1024);
			CString sTranslate = translate(str);
			SetWindowText(hwndChild, sTranslate);
			CWnd* pWnd = CWnd::FromHandle(hwndChild);
			auto defaultFont = new CFont;
			//defaultFont->CreatePointFont(80,_T("Arial Narrow"));
			defaultFont->CreatePointFont(80,_T("宋体"));
			pWnd->SetFont(defaultFont);
			hwndChild = ::GetWindow(hwndChild, GW_HWNDNEXT);
		}
	
	}

	Languge::Languge()
	{
	}

	Languge::~Languge()
	{
	}

	void Languge::loadSettings(const std::map<CString, std::vector<CString>>& mvsSettings)
	{		
		m_vsLangugeList.clear();

		//找出语言数目
		auto iter = mvsSettings.find("名称翻译");
		if (iter == mvsSettings.end())
		{
			return;
		}
		size_t unCameraCount = iter->second.size();

		//转换参数
		for (size_t i = 2; i < unCameraCount; i++)
		{
			if (iter->second[i].IsEmpty())
			{
				continue;
			}
			T_SETTINGS tSettings;
			int nID0 = 0;
			int nID1 = 0;
			if (!XiVariableType::GetParaFromExcelData(mvsSettings, "名称翻译", i, tSettings.sName)
				|| !XiVariableType::GetParaFromExcelData(mvsSettings, "是否启用", i, tSettings.bEnable)
				|| !XiVariableType::GetParaFromExcelData(mvsSettings, "语言ID0", i, nID0)
				|| !XiVariableType::GetParaFromExcelData(mvsSettings, "语言ID1", i, nID1))
			{
				tSettings.bEnable = false;
			}
			tSettings.nLangugeID = MAKELANGID(nID0, nID1);
			m_vsLangugeList.push_back(tSettings);
		}
	}
}