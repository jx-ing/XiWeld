#include "stdafx.h"
#include "Languge.h"

namespace XUI
{
	Languge Languge::m_instance;
	constexpr UINT g_nChineseLanguageID = MAKELANGID(LANG_CHINESE, SUBLANG_CHINESE_SIMPLIFIED);

	bool Languge::loadAllLanguge(CString sLangugeFile)
	{
		//�򿪱��
		std::map<CString, std::vector<CString>> mvsSettings;
		m_mvsLanguge.clear();
		OpenExcel cOpenExcel;
		if (!cOpenExcel.Init(XiBase::GetSystemLog())
			|| !cOpenExcel.Open(sLangugeFile)
			|| !cOpenExcel.Read("����", mvsSettings)
			|| !cOpenExcel.Read("����", m_mvsLanguge)
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

		//���Ĳ���Ҫ����
		if (sLanguge == "����")
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
		//�����ڵ��������
		if (m_nCurLangugeNo < 0
			|| m_nCurLangugeNo >= m_vsLangugeList.size())
		{
			return g_nChineseLanguageID;
		}

		return m_vsLangugeList[m_nCurLangugeNo].nLangugeID;
	}

	CString Languge::translate(CString srtString)
	{
		//�����ڵ�������ţ�������
		if (m_nCurLangugeNo < 0
			|| m_nCurLangugeNo >= m_vsLangugeList.size())
		{
			return srtString;
		}

		auto iter = m_mvsLanguge.find(srtString);
		if (iter == m_mvsLanguge.end()//û������ַ����ķ���
			|| iter->second[m_nCurLangugeNo].IsEmpty())//����ַ����ķ���Ϊ��
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
		HWND hwndChild = ::GetWindow(pDlg->m_hWnd, GW_CHILD);  //�г����пؼ�  
		while (hwndChild)
		{
			GetWindowText(hwndChild, str, 1024);
			CString sTranslate = translate(str);
			SetWindowText(hwndChild, sTranslate);
			CWnd* pWnd = CWnd::FromHandle(hwndChild);
			auto defaultFont = new CFont;
			//defaultFont->CreatePointFont(80,_T("Arial Narrow"));
			defaultFont->CreatePointFont(80,_T("����"));
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

		//�ҳ�������Ŀ
		auto iter = mvsSettings.find("���Ʒ���");
		if (iter == mvsSettings.end())
		{
			return;
		}
		size_t unCameraCount = iter->second.size();

		//ת������
		for (size_t i = 2; i < unCameraCount; i++)
		{
			if (iter->second[i].IsEmpty())
			{
				continue;
			}
			T_SETTINGS tSettings;
			int nID0 = 0;
			int nID1 = 0;
			if (!XiVariableType::GetParaFromExcelData(mvsSettings, "���Ʒ���", i, tSettings.sName)
				|| !XiVariableType::GetParaFromExcelData(mvsSettings, "�Ƿ�����", i, tSettings.bEnable)
				|| !XiVariableType::GetParaFromExcelData(mvsSettings, "����ID0", i, nID0)
				|| !XiVariableType::GetParaFromExcelData(mvsSettings, "����ID1", i, nID1))
			{
				tSettings.bEnable = false;
			}
			tSettings.nLangugeID = MAKELANGID(nID0, nID1);
			m_vsLangugeList.push_back(tSettings);
		}
	}
}