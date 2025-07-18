#include "stdafx.h"
#include "MesBox.h"

namespace XUI
{
	MesBox::MesBox()
	{
		m_pLog = XiBase::GetSystemLog();
	}

	MesBox::~MesBox()
	{
		m_pLog = nullptr;
	}

	void MesBox::setPopup(bool bPopup)
	{
		m_bPopup = bPopup;
	}

	void MesBox::setLog(XiBase::CLog& cLog)
	{
		m_pLog = &cLog;
	}

}

namespace XUI
{
	//缓存弹窗数目上限
	constexpr auto g_nMessageBoxMaxNum = 100;

	//上一次申请的弹窗序号
	unsigned long g_ulLastMessageBoxpNo = 0;

	//弹窗锁
	CMyCriticalSection g_cMessageBoxLock;

	//弹窗信息
	T_MESSAGE_BOX g_vtMessageBox[g_nMessageBoxMaxNum] = { T_MESSAGE_BOX() };

	int GetMessageBoxNo(unsigned int nType)
	{
		CRITICAL_AUTO_LOCK(g_cMessageBoxLock);
		for (int i = 0; i < g_nMessageBoxMaxNum; i++)
		{
			if (!g_vtMessageBox[i].bPopup)
			{
				g_vtMessageBox[i].bPopup = true;
				g_vtMessageBox[i].nType = nType;
				g_vtMessageBox[i].ulNo = g_ulLastMessageBoxpNo;
				g_ulLastMessageBoxpNo++;
				return i;
			}
		}
		return -1;
	}
}

namespace XUI
{
	bool CheckIsPopup()
	{
		CRITICAL_AUTO_LOCK(g_cMessageBoxLock);
		for (int i = 0; i < MAX_MESSAGE_BOX_NUM; i++)
		{
			if (g_vtMessageBox[i].bPopup)
			{
				return true;
			}
		}
		return false;
	}

	bool IsErrorPopup()
	{
		CRITICAL_AUTO_LOCK(g_cMessageBoxLock);
		for (int i = 0; i < MAX_MESSAGE_BOX_NUM; i++)
		{
			if (g_vtMessageBox[i].bPopup
				&& ((g_vtMessageBox[i].nType & MB_ICONSTOP) == MB_ICONSTOP))
			{
				return true;
			}
		}
		return false;
	}

	std::vector<unsigned int> GetPopupType()
	{
		std::vector<unsigned int> vunPopupType;
		CRITICAL_AUTO_LOCK(g_cMessageBoxLock);
		for (int i = 0; i < MAX_MESSAGE_BOX_NUM; i++)
		{
			if (g_vtMessageBox[i].bPopup)
			{
				vunPopupType.push_back(g_vtMessageBox[i].nType);
			}
		}
		return vunPopupType;
	}
	HHOOK   g_hMsgBoxHook;

	LRESULT CALLBACK CBTHookProc(int nCode, WPARAM wParam, LPARAM lParam)
	{
		if (nullptr == g_hMsgBoxHook)
			return 0;

		if (nCode == HCBT_ACTIVATE)
		{
			SetDlgItemText((HWND)wParam, IDABORT, Languge::GetInstance().translate("中止"));
			SetDlgItemText((HWND)wParam, IDCANCEL, Languge::GetInstance().translate("取消"));
			SetDlgItemText((HWND)wParam, IDIGNORE, Languge::GetInstance().translate("忽略"));
			SetDlgItemText((HWND)wParam, IDNO, Languge::GetInstance().translate("否"));
			SetDlgItemText((HWND)wParam, IDOK, Languge::GetInstance().translate("确定"));
			SetDlgItemText((HWND)wParam, IDRETRY, Languge::GetInstance().translate("重试"));
			SetDlgItemText((HWND)wParam, IDYES, Languge::GetInstance().translate("是"));
			UnhookWindowsHookEx(g_hMsgBoxHook);
			g_hMsgBoxHook = nullptr;
			return 0;
		}

		return CallNextHookEx(g_hMsgBoxHook, nCode, wParam, lParam);
	}
	int XiMessageBoxPopup(XiBase::CLog& cLog, unsigned int nType, bool bPopup, std::string sMessage)
	{
		int nRtn = IDNO;
		if (bPopup)
		{
			//如果没有设置图标风格，就设为MB_ICONINFORMATION
			if ((nType & 0x000000F0L) == 0x00000000L)
			{
				nType = nType | MB_ICONINFORMATION;
			}

			//弹窗
			int nMessageBoxNo = GetMessageBoxNo(nType);
			CString sLog;
			if (nMessageBoxNo >= 0)
			{
				sLog.Format("XiMessageBox(%lu)(%lu): ", g_vtMessageBox[nMessageBoxNo].ulNo, nType);
			}
			else
			{
				sLog.Format("XiMessageBox(Overflow)(%lu): ", nType);
			}

			CString sSpace = "\n";
			for (int i = 0; i < sLog.GetLength(); i++)
			{
				sSpace += " ";
			}

			CString sLogMessage = sMessage.c_str();
			sLogMessage.Replace("\n", sSpace);
			cLog.Write(sLog + sLogMessage);

			//nRtn = AfxMessageBox(sRealMessage, nType);

			//nRtn = MessageBoxEx(NULL, sMessage.c_str(), "XiRobot", nType, Languge::GetInstance().getCurLangugeID());
			g_hMsgBoxHook = SetWindowsHookEx(
				WH_CBT,
				CBTHookProc,
				nullptr,
				GetCurrentThreadId());

			nRtn = AfxMessageBox(sMessage.c_str(), nType);
			switch (nRtn)
			{
			case 0:			cLog.Write(sLog + "return = 内存不足");	break;
			case IDABORT:	cLog.Write(sLog + "return = 中止");		break;
			case IDCANCEL:	cLog.Write(sLog + "return = 取消");		break;
			case IDIGNORE:	cLog.Write(sLog + "return = 忽略");		break;
			case IDNO:		cLog.Write(sLog + "return = 否");		break;
			case IDOK:		cLog.Write(sLog + "return = 确定");		break;
			case IDRETRY:	cLog.Write(sLog + "return = 重试");		break;
			case IDYES:		cLog.Write(sLog + "return = 是");		break;
			default:		cLog.Write(sLog + "return = %d", nRtn);	break;
			}

			if (nMessageBoxNo >= 0)
			{
				g_vtMessageBox[nMessageBoxNo].bPopup = false;
			}
		}
		else
		{
			cLog.Write("XiMessageBox(NoBox)(" + XiBase::AutoToCString((unsigned long)nType) + "):" + sMessage.c_str());
			nRtn = IDOK;
		}
		return nRtn;
	}

	int XiMessageBoxPopup(unsigned int nType, bool bPopup, std::string sMessage)
	{
		return XUI::XiMessageBoxPopup(*XiBase::GetSystemLog(), nType, bPopup, sMessage);
	}

}