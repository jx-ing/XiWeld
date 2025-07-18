#include "stdafx.h"
#include ".\Apps\PLib\BasicFunc\Const.h"
#include <fstream>
#include ".\OpenClass\FileOP\ini\opini.h"
#include "math.h"
#include "CommonAlgorithm.h"
#include "sys/stat.h"
#include <windows.h>
#include <psapi.h>
#include <io.h>
#include <chrono>
#include "LocalFiles\ExLib\Vision\include\PointCloudWeldingLinesExtraction.h"
#include "LocalFiles\ExLib\ALGO\include\XiAlgorithm.h"

#pragma comment(lib, "Psapi.Lib")

E_INCISEHEAD_THREAD_STATUS g_eThreadStatus = INCISEHEAD_THREAD_STATUS_START;

// 运行信息统计
int m_nWorkState; //判断白班还是夜班
CString m_sFirstOpenTime; // 首次打开时间 年/月/日 时:分:秒
CString m_sLastCloseTime; // 最后关闭时间 年/月/日 时:分:秒
long m_lTotalOpenTime;	// 总打开时间 ms
long m_lTotalColseTime;	// 总断开时间 ms

long m_lWorkTime;			// 扫描、交互、示教、焊接 等总工作时间
long m_lStandbyTime;		// 待机时间 = 总联机时间 - 工作时间
long m_lFlatWeldTime;		// 平焊时间
long m_lStandWeldTime;		// 立焊时间
long m_lScanTime;			// 扫描时间 线扫时间 + 示教时间
long m_lScanWeldTime;		// 扫描时间 + 平焊时间 + 立焊时间

double m_dFlatWeldLen;	// 平焊长度
double m_dStandWeldLen;	// 立焊长度
double m_dTotalWeldLen;	// 焊缝总长度 平焊长度 + 立焊长度
double m_dTotalScanLen;	// 总线扫扫描长度

long lCurOpenTimeStamp; // 本地打开时间戳
double m_dCurWeldLenBeforeCleanGun; // 上次清枪后的焊接总长度

BOOL LoadFileToVector( CString sFileName, std::vector <double> &vdData, int nDataNum )
{
    double dData = 0;
    vdData.clear();
	FILE *pFile;
	XI_fopen_s(&pFile, sFileName, "r");
    CHECK_FILE_POINTER_RET_PARA(pFile, FALSE);
    int nDataNo = 0;
    while(EOF != fscanf_s(pFile, "%lf", &dData))
    {
        vdData.push_back(dData);
        nDataNo++;
        if ((nDataNum != 0) && (nDataNo == nDataNum))
        {
            break;
        }
    }
    fclose(pFile);
    std::vector <double> (vdData).swap(vdData);
    
    return TRUE;
}

BOOL LoadFileToVector( CString sFileName, std::vector <long> &vlData, int nDataNum )
{
    long lData = 0;
    vlData.clear();
	FILE *pFile;
	XI_fopen_s(&pFile, sFileName, "r");
    CHECK_FILE_POINTER_RET_PARA(pFile, FALSE);
    int nDataNo = 0;
    while(EOF != fscanf_s(pFile, "%ld", &lData))
    {
        vlData.push_back(lData);
        nDataNo++;
        if ((nDataNum != 0) && (nDataNo == nDataNum))
        {
            break;
        }
    }
    fclose(pFile);
    
    std::vector <long> (vlData).swap(vlData);
    
    return TRUE;
}

double VelDisToVelPulse( WORD wAxisNum, WORD wPositionMode, long alPulse[], float fGunLength )
{
//     if (VELMODE_ABSOLUTE == wPositionMode)
//     {
//         for (WORD wIdx = 0; wIdx < wAxisNum; wIdx++)
//         {
//             alPulse[wIdx] = alPulse[wIdx] - g_drGetPosition(wIdx);
//         }
//     }

//     double dDisRotate = alPulse[AXIS_A] * m_dPulseToDisA 
//         * (g_drGetPosition(AXIS_R) * m_dPulseToDisR - fGunLength * sin(g_drGetPosition(AXIS_S) * m_dPulseToDisS));

    return 0;
}

int GetFileLineNum( CString fileName )
{
    std::ifstream file(fileName);
    if (!file.is_open())
    {
        //AfxMessageBox("文件打开失败");
		return 0;
    }

    char acLineBuf[1024];//N是定义的常数，目的是为了读取足够长的行
    int nLineNum = 0;
    while(!file.eof())
    {
        memset(acLineBuf, 0, sizeof(acLineBuf));
        file.getline(acLineBuf, sizeof(acLineBuf));
        nLineNum++;
    }

    if ((acLineBuf[0] == 0) && (acLineBuf[1] == 0) && (acLineBuf[2] == 0))
    {
        nLineNum--;
    }

    file.close();

    return nLineNum;
}

//int XiMessageBox(CLog *cLog, char *format, ... )
//{
//#ifdef PC
//    return 1;
//#endif
//
//	if (cLog == NULL)
//	{
//		cLog = ReturnCLog();
//	}
//	//已修改
//	XUI::MesBox::PopInfo(format);
//	return 1;
//
//    va_list args;
//    va_start(args, format);
//    char s[1000];
//	vsprintf_s(s, 1000, format, args);
//    va_end(args);
//
//	CString str = s;
//	str = _T("XiMessageBox:") + str;
//	cLog->Write(str);
//    return AfxMessageBox(s, MB_OKCANCEL);
//}

int XiMessageBox(const char* format)
{
#ifdef PC
	return 1;
#endif
	//已修改
	return XUI::MesBox::PopOkCancel(format);


	//va_list args;
	//va_start(args, format);
	//char s[1000];
	//vsprintf_s(s, 1000, format, args);
	//va_end(args);

	//CString str = s;
	//str = _T("XiMessageBox:") + str;
	//WriteLog(str);
	//return AfxMessageBox(s, MB_OKCANCEL);
}

//int XiMessageBox(CLog *cLog, CString str)
//{
//#ifdef PC
//	return 1;
//#endif
//	if (cLog == NULL)
//	{
//		cLog = ReturnCLog();
//	}
//	if (cLog == NULL)
//	{
//		cLog = ReturnCLog();
//	}
//	cLog->Write(_T("XiMessageBox:") + str);
//	return AfxMessageBox(str, MB_OKCANCEL);
//}

//int XiMessageBox(CString str)
//{
//#ifdef PC
//	return 1;
//#endif
//	CLog *cLog = ReturnCLog();
//
//	cLog->Write(_T("XiMessageBox:") + str);
//	return AfxMessageBox(str, MB_OKCANCEL);
//}

//int XiMessageBoxOk(CLog *cLog, char *format, ...)
//{
//
//#ifdef PC
//	return 1;
//#endif
//	if (cLog == NULL)
//	{
//		cLog = ReturnCLog();
//	}
//	va_list args;
//	va_start(args, format);
//	char s[1000];
//	vsprintf_s(s, 1000, format, args);
//	va_end(args);
//
//	CString str = s;
//	str = _T("XiMessageBox:") + str;
//	cLog->Write(str);
//	return AfxMessageBox(s);
//}

//int XiMessageBoxOk(CLog *cLog, CString str)
//{
//
//#ifdef PC
//	return 1;
//#endif
//	if (cLog == NULL)
//	{
//		cLog = ReturnCLog();
//	}
//	CString s = str;
//	str = _T("XiMessageBox:") + str;
//	cLog->Write(str);
//	return AfxMessageBox(s);
//}

int XiMessageBoxOk(const char* format)
{
#ifdef PC
	return 1;
#endif
	//已修改
	return XUI::MesBox::PopOkCancel(format);

	//va_list args;
	//va_start(args, format);
	//char s[1000];
	//vsprintf_s(s, 1000, format, args);
	//va_end(args);

	//CString str = s;
	//str = _T("XiMessageBox:") + str;
	//WriteLog(str);
	//return AfxMessageBox(s);
}

//int XiMessageBoxPopup(CLog *cLog, CString str, bool bPopup)
//{
//#ifdef PC
//	return 1;
//#endif
//
//	if (cLog == NULL)
//	{
//		cLog = ReturnCLog();
//	}
//	if (!bPopup)
//	{
//		cLog->Write(_T("XiMessageBox(NoBox):") + str);
//		return 1;
//	}
//
//	cLog->Write(_T("XiMessageBox:") + str);
//	return AfxMessageBox(str);
//}

//int XiMessageBoxGroup(bool bIfGroup, char* format, ...)
//{
//	va_list args;
//	va_start(args, format);
//	char s[1000];
//	vsprintf(s, format, args);
//	va_end(args);
//	CString str = s;
//	str = _T("XiMessageBox:") + str;
//	if (bIfGroup)
//	{
//		WriteLog(str);
//		return AfxMessageBox(s, MB_OKCANCEL);
//	}
//	else
//	{
//		WriteLog(str);
//		return IDOK;
//	}
//}
//
//int XiMessageBoxGroup(BOOL bIfGroup, char* format, ...)
//{
//	va_list args;
//	va_start(args, format);
//	char s[1000];
//	vsprintf(s, format, args);
//	va_end(args);
//	CString str = s;
//	str = _T("XiMessageBox:") + str;
//	if (bIfGroup == TRUE)
//	{
//		WriteLog(str);
//		return AfxMessageBox(s, MB_OKCANCEL);
//	}
//	else
//	{
//		WriteLog(str);
//		return IDOK;
//	}
//}

//int XiMessageBoxPopup(CString str, BOOL bPopup /*= TRUE*/)
//{
//#ifdef PC
//	return 1;
//#endif
//	if (bPopup == FALSE)
//	{
//		ReturnCLog()->Write(_T("XiMessageBox(NoBox):") + str);
//		return 1;
//	}
//
//	ReturnCLog()->Write(_T("XiMessageBox:") + str);
//	return AfxMessageBox(str);
//}

//int XiMessageBoxPopup(CLog *cLog, CString str, BOOL bPopup /*= TRUE*/)
//{
//#ifdef PC
//	return 1;
//#endif
//
//	if (cLog == NULL)
//	{
//		cLog = ReturnCLog();
//	}
//	if (bPopup == FALSE)
//	{
//		cLog->Write(_T("XiMessageBox(NoBox):") + str);
//		return 1;
//	}
//
//	cLog->Write(_T("XiMessageBox:") + str);
//	return AfxMessageBox(str);
//}

void SetDlgItemData( int nItem, double data, CDialog *pDlg, BOOL bIfWindowOn )
{
    if (FALSE == bIfWindowOn)
    {
        return ;
    }
    
    CString str;
    str.Format("%.1f", data);
    pDlg->GetDlgItem(nItem)->SetWindowText(str);
}


void SetDlgItemData(int nItem, int data, CDialog *pDlg, BOOL bIfWindowOn)
{
    if (FALSE == bIfWindowOn)
    {
        return ;
    }
    
    CString str;
    str.Format("%d", data);
    pDlg->GetDlgItem(nItem)->SetWindowText(str);
}

void SetDlgItemData( int nItem, CString data, CDialog *pDlg, BOOL bIfWindowOn )
{
    if (FALSE == bIfWindowOn)
    {
        return ;
    }
    
    pDlg->GetDlgItem(nItem)->SetWindowText(data);
}

void SetDlgItemData(int nItem, CDialog *pDlg, BOOL bIfWindowOn, char *format, ... )
{
    if (FALSE == bIfWindowOn)
    {
        return ;
    }
    va_list args;
    va_start(args, format);
    char s[1000];
	vsprintf_s(s, 1000, format, args);
    va_end(args);

	ReturnCLog()->Write(s);
    pDlg->GetDlgItem(nItem)->SetWindowText(s);
}

void SetDlgItemData(int nItem, CDialog* pDlg, char* format, ...)
{
	va_list args;
	va_start(args, format);
	char s[1000];
	vsprintf_s(s, 1000, format, args);
	va_end(args);

	ReturnCLog()->Write(s);
	pDlg->GetDlgItem(nItem)->SetWindowText(s);
}

void SetDlgItemData(int nItem, long data, CDialog *pDlg, BOOL bIfWindowOn /*= TRUE*/)
{
	if (FALSE == bIfWindowOn)
	{
		return;
	}

	CString str;
	str.Format("%ld", data);
	pDlg->GetDlgItem(nItem)->SetWindowText(str);
}

void GetButtonState(CButton *btn, int &state, CDialog *pDlg, BOOL bIfWindowOn)
{
    if (FALSE == bIfWindowOn)
    {
        return ;
    }
    
    state = btn->GetState() & 0x3;
}

void GetDlgItemData(int nItem, double &data, CDialog *pDlg, BOOL bIfWindowOn)
{
    if (FALSE == bIfWindowOn)
    {
        return ;
    }
    
    CString str;
    pDlg->GetDlgItem(nItem)->GetWindowText(str);
    data = atof(str);
}

void GetDlgItemData(int nItem, int &data, CDialog *pDlg, BOOL bIfWindowOn)
{
    if (FALSE == bIfWindowOn)
    {
        return ;
    }
    
    CString str;
    pDlg->GetDlgItem(nItem)->GetWindowText(str);
    data = atof(str);
}


void DoEvent()
{
	MSG msg;
	if (::PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))  // 从Windows消息队列中取出消息
	{
		if (msg.message == WM_QUIT) // 如果消息为退出，发送退出消息
		{
			::PostQuitMessage(-1);
		}
		if (!AfxGetApp()->PreTranslateMessage(&msg)) // 如果无法预处理消息
		{
			::TranslateMessage(&msg); // 转换消息
			::DispatchMessage(&msg);  // 发送消息
		}
	}
}

CString GetModuleFilePath()
{
	TCHAR szFilePath[MAX_PATH + 1];
	GetModuleFileName(NULL, szFilePath, MAX_PATH);
	(_tcsrchr(szFilePath, _T('\\')))[1] = 0; //删除文件名，只获得路径

	CString sFilePath = szFilePath;
	return sFilePath;
}

CString SaveFileDlg(CDialog* pFatherDlg, CString sFileType, CString sDefaultName, CString sDefaultPath, CString sTitle/* = "另存为"*/)
{
	CString sModuleFilePath = GetModuleFilePath();
	CString acFilter = sFileType + "文件(*." + sFileType + ")|*." + sFileType + "||";
	CFileDialog cFileDlg(FALSE, sFileType, sDefaultName, OFN_HIDEREADONLY | OFN_READONLY, acFilter, pFatherDlg);
	CString filePath;
	cFileDlg.m_ofn.lpstrTitle = sTitle;
	cFileDlg.m_ofn.lpstrInitialDir = sDefaultPath;
	if (IDOK == cFileDlg.DoModal())
	{
		filePath = cFileDlg.GetPathName();
	}
	SetCurrentDirectory(sModuleFilePath);
	return filePath;
}

CString OpenFileDlg(CDialog* pFatherDlg, CString sFileType,CString sDefaultPath, CString sTitle/* = "打开"*/)
{
	CString sModuleFilePath = GetModuleFilePath();
	CString acFilter = sFileType + "文件(*." + sFileType + ")|*." + sFileType + "||";
	CFileDialog cFileDlg(TRUE, sFileType, NULL, OFN_HIDEREADONLY | OFN_READONLY, acFilter, pFatherDlg);
	CString filePath; 

	cFileDlg.m_ofn.lpstrTitle = sTitle;
	cFileDlg.m_ofn.lpstrInitialDir = sDefaultPath;
	if (IDOK == cFileDlg.DoModal())
	{
		filePath = cFileDlg.GetPathName();
	}
	SetCurrentDirectory(sModuleFilePath);
	return filePath;
}

bool OpenFileDlg(CDialog *pFatherDlg, CString sFileType, CString sDefaultPath, std::vector< CString> &vstrFileList)
{
	vstrFileList.clear();
	CString acFilter = sFileType + "文件(*." + sFileType + ")|*." + sFileType + "||";
	CFileDialog cFileDlg(TRUE, NULL, NULL, OFN_ALLOWMULTISELECT | OFN_ENABLESIZING | OFN_HIDEREADONLY, acFilter, pFatherDlg);
	const int MIN_FILE_NUMBER = 10;                                    //至少允许选择10个文件
	cFileDlg.m_ofn.lpstrFile = new TCHAR[_MAX_PATH * MIN_FILE_NUMBER]; //重新定义缓冲区大小          
	memset(cFileDlg.m_ofn.lpstrFile, 0, _MAX_PATH * MIN_FILE_NUMBER);  //初始化定义的缓冲区
	cFileDlg.m_ofn.nMaxFile = _MAX_PATH * MIN_FILE_NUMBER;
	cFileDlg.GetOFN().lpstrInitialDir = sDefaultPath;
	if (IDOK == cFileDlg.DoModal())
	{
		POSITION pos = cFileDlg.GetStartPosition();
		int i = 0;
		while (NULL != pos)
		{
			CString filePath = cFileDlg.GetNextPathName(pos);
			vstrFileList.push_back(filePath);
		}
	}
	return true;
}
bool OpenFileDlg(CDialog* pFatherDlg, std::vector < CString> vsFileType, CString sDefaultPath, std::vector< CString>& vstrFileList)
{
	vstrFileList.clear();
	CString acFilter = "All Files(*.*)|*.*|";
	for (size_t i = 0; i < vsFileType.size(); i++)
	{
		acFilter = acFilter + vsFileType[i] + "文件(*." + vsFileType[i] + ")|*." + vsFileType[i] + "|";
	}
	acFilter = acFilter + "|";
	CFileDialog cFileDlg(TRUE, NULL, NULL, OFN_ALLOWMULTISELECT | OFN_ENABLESIZING | OFN_HIDEREADONLY, acFilter, pFatherDlg);
	const int MIN_FILE_NUMBER = 10;                                    //至少允许选择10个文件
	cFileDlg.m_ofn.lpstrFile = new TCHAR[_MAX_PATH * MIN_FILE_NUMBER]; //重新定义缓冲区大小          
	memset(cFileDlg.m_ofn.lpstrFile, 0, _MAX_PATH * MIN_FILE_NUMBER);  //初始化定义的缓冲区
	cFileDlg.m_ofn.nMaxFile = _MAX_PATH * MIN_FILE_NUMBER;
	cFileDlg.GetOFN().lpstrInitialDir = sDefaultPath;
	if (IDOK == cFileDlg.DoModal())
	{
		POSITION pos = cFileDlg.GetStartPosition();
		int i = 0;
		while (NULL != pos)
		{
			CString filePath = cFileDlg.GetNextPathName(pos);
			vstrFileList.push_back(filePath);
		}
	}
	if (vstrFileList.size() <= 0)
	{
		return false;
	}
	return true;
}
CString Utf8ToGBK(LPCTSTR strUtf8)
{
	int len = MultiByteToWideChar(CP_UTF8, 0, (LPCTSTR)strUtf8, -1, NULL, 0);
	wchar_t * wszGBK = new wchar_t[len];
	memset(wszGBK, 0, len);
	MultiByteToWideChar(CP_UTF8, 0, (LPCTSTR)strUtf8, -1, wszGBK, len);

	len = WideCharToMultiByte(CP_ACP, 0, wszGBK, -1, NULL, 0, NULL, NULL);
	char *szGBK = new char[len + 1];
	memset(szGBK, 0, len + 1);
	WideCharToMultiByte(CP_ACP, 0, wszGBK, -1, szGBK, len, NULL, NULL);
	
	CString strTemp(szGBK);
	delete[]szGBK;
	delete[]wszGBK;
	return strTemp;
}


void VerticalCenter(CEdit &cEdit)
{
    CRect cRect = CRect(0, 0, 0, 0);
    cEdit.GetClientRect(&cRect);
    TEXTMETRIC textMetric;
    CDC *pcDC = cEdit.GetDC();
    pcDC->GetTextMetrics(&textMetric);
    int nFontHight = textMetric.tmHeight + textMetric.tmExternalLeading;
    int nOffsetY = (cRect.Height() - nFontHight) / 2;//计算文字向下偏移量
    cRect.OffsetRect(0, nOffsetY);//设置向下偏移
    cEdit.SetRectNP(cRect);
}

bool CheckFileExists(CString sFileName, bool bCreate)
{
	struct _stat buffer;
	if (_stat(sFileName, &buffer) == 0)
	{
		return true;
	}
	else
	{
		if (bCreate)
		{
			FILE *pfFile;
			XI_fopen_s(&pfFile, sFileName, "w");
			fclose(pfFile);
		}
		return false;
	}
}

//bool CheckFileExists(std::string sFileName, bool bCreate /*= false*/)
//{
//	CString str;
//	str.Format("%s", sFileName.c_str());
//	return CheckFileExists(str, bCreate);
//}

long CheckFolder(CString strPath)
{
	if (!PathIsDirectory(strPath))
	{
		if (!(::CreateDirectory(strPath, 0)))
		{
			long nErro = ::GetLastError();
			if (nErro == 3)//路径不存在
			{
				int nPos = strPath.ReverseFind('\\');
				CString strNowPath = strPath;
				strNowPath = strNowPath.Left(nPos);
				CheckFolder(strNowPath);
				return CheckFolder(strPath);
			}
			return -nErro;
		}
	}
	return 1;
}

bool FindFile(std::vector<CString> &vstrFileList, CString strPath, CString strFileType, bool bIncludeSubfolder, bool bReturnPathOrName)
{
	vstrFileList.clear();
	CFileFind finder;
	if (strPath.Right(1) != '\\')
	{
		strPath += "\\";
	}
	strPath += "*.*";
	BOOL bWorking = finder.FindFile(strPath);
	while (bWorking)
	{
		bWorking = finder.FindNextFile();
		if (finder.IsDirectory() && !finder.IsDots() && bIncludeSubfolder)
		{
			std::vector<CString> vstrFileList2;
			FindFile(vstrFileList2, finder.GetFilePath(), strFileType, bIncludeSubfolder, bReturnPathOrName);
			for (int i = 0; i < vstrFileList2.size(); i++)
			{
				vstrFileList.push_back(vstrFileList2[i]);
			}
		}
		else// if (finder.IsNormal())
		{
			//获取名称
			CString strFileName = finder.GetFileName();
			CString strFileTitle = finder.GetFileTitle();
			strFileTitle += ".";
			strFileTitle += strFileType;

			//先转为大写字母
			strFileName.MakeUpper();
			strFileTitle.MakeUpper();

			//比对
			if (strFileName == strFileTitle)
			{
				if (bReturnPathOrName)
				{
					vstrFileList.push_back(finder.GetFilePath());
				} 
				else
				{
					vstrFileList.push_back(finder.GetFileName());
				}			
			}			
		}
	}
	if (vstrFileList.size() <= 0)
	{
		return false;
	}
	return true;
}

long CopyFolder(CString strSrcFolderName, CString strDstPath, bool bCopySubfolder)
{
	//初始化
	std::vector<_finddata_t> fileList;
	_finddata_t file;
	long long lHandle;
	CString  strSrcDirFile = strSrcFolderName + "\\*.*";

	//检查目标文件夹
	CString strDstDir;
	int nPos = strSrcFolderName.ReverseFind('\\');
	strDstDir = strSrcFolderName;
	strDstDir.Delete(0, nPos + 1);
	strDstDir = strDstPath + "\\" + strDstDir;
	long lReturn = CheckFolder(strDstDir);
	if (lReturn < 0)
	{
		return lReturn;
	}

	//遍历文件
	if ((lHandle = _findfirst(strSrcDirFile, &file)) == -1)//无文件
	{
		return -1;
	}
	else 
	{
		while (_findnext(lHandle, &file) == 0)
		{
			if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0)
				continue;
			fileList.push_back(file);
		}
	}
	_findclose(lHandle);	

	//复制文件
	for (int i = 0; i < fileList.size(); i++)
	{
		unsigned FileAttrib = fileList.at(i).attrib;
		if ((FileAttrib & _A_SUBDIR) == _A_SUBDIR && bCopySubfolder)
		{
			CString nowSrcDirPath = strSrcFolderName + "\\" + fileList.at(i).name;
			CopyFolder(nowSrcDirPath, strDstDir, bCopySubfolder);
		}
		CString nowSrcFilePath, nowDesFilePath;
		nowSrcFilePath = strSrcFolderName + "\\" + fileList.at(i).name;
		nowDesFilePath = strDstDir + "\\" + fileList.at(i).name;
		CopyFile((LPCTSTR)nowSrcFilePath, (LPCTSTR)nowDesFilePath, FALSE);		
	}
	return 0;
}

/////////////////////////////////////

  //ReNameFolder

  //参数：lpszFromPath 源文件夹路径 。lpszToPath 目的文件夹路径

  //作用：修改原文件夹的名字。

  //

  /////////////////////////////////////

BOOL ReNameFolder(LPCTSTR lpszFromPath, LPCTSTR lpszToPath)
{
	int nLengthFrm = strlen(lpszFromPath);
	char* NewPathFrm = new char[nLengthFrm + 2];
	strcpy(NewPathFrm, lpszFromPath);
	NewPathFrm[nLengthFrm] = '\0';
	NewPathFrm[nLengthFrm + 1] = '\0';
	SHFILEOPSTRUCT FileOp;
	ZeroMemory((void*)&FileOp, sizeof(SHFILEOPSTRUCT));
	FileOp.fFlags = FOF_NOCONFIRMATION;
	FileOp.hNameMappings = NULL;
	FileOp.hwnd = NULL;
	FileOp.lpszProgressTitle = NULL;
	FileOp.pFrom = NewPathFrm;
	FileOp.pTo = lpszToPath;
	FileOp.wFunc = FO_RENAME;
	return SHFileOperation(&FileOp) == 0;
}

char gcModulePath[500];
char gcModuleWholePath[500];
void GetModulePath(char *cPosfix)
{
	memset(gcModuleWholePath, 0, sizeof(gcModuleWholePath));
	memset(gcModulePath, 0, sizeof(gcModulePath));
	GetModuleFileName(NULL, gcModulePath, MAX_PATH);
	//(strrchr(gcModulePath, '\\'))[0] = 0;
	(strrchr(gcModulePath, '\\'))[1] = 0;

	strcat_s(gcModuleWholePath, sizeof(gcModuleWholePath), gcModulePath);
	strcat_s(gcModuleWholePath, sizeof(gcModuleWholePath), cPosfix);
}

void GetModulePath(const char *cPosfix)
{
	memset(gcModuleWholePath, 0, sizeof(gcModuleWholePath));
	memset(gcModulePath, 0, sizeof(gcModulePath));
	GetModuleFileName(NULL, gcModulePath, MAX_PATH);
	//(strrchr(gcModulePath, '\\'))[0] = 0;
	(strrchr(gcModulePath, '\\'))[1] = 0;

	strcat_s(gcModuleWholePath, sizeof(gcModuleWholePath), gcModulePath);
	strcat_s(gcModuleWholePath, sizeof(gcModuleWholePath), cPosfix);
}

char gcLogModulePath[500];
char gcLogModuleWholePath[500];
void GetLogModulePath(char *cPosfix)
{
	memset(gcLogModuleWholePath, 0, sizeof(gcLogModuleWholePath));
	memset(gcLogModulePath, 0, sizeof(gcLogModulePath));
	GetModuleFileName(NULL, gcLogModulePath, MAX_PATH);
	//(strrchr(gcLogModulePath, '\\'))[0] = 0;
	(strrchr(gcLogModulePath, '\\'))[1] = 0;

	strcat_s(gcLogModuleWholePath, sizeof(gcLogModuleWholePath), gcLogModulePath);
	strcat_s(gcLogModuleWholePath, sizeof(gcLogModuleWholePath), cPosfix);
}

void GetLogModulePath(const char *cPosfix)
{
	memset(gcLogModuleWholePath, 0, sizeof(gcLogModuleWholePath));
	memset(gcLogModulePath, 0, sizeof(gcLogModulePath));
	GetModuleFileName(NULL, gcLogModulePath, MAX_PATH);
	//(strrchr(gcLogModulePath, '\\'))[0] = 0;
	(strrchr(gcLogModulePath, '\\'))[1] = 0;

	strcat_s(gcLogModuleWholePath, sizeof(gcLogModuleWholePath), gcLogModulePath);
	strcat_s(gcLogModuleWholePath, sizeof(gcLogModuleWholePath), cPosfix);
}

double DecimalRound(double dNum, int dDECimalDigits)
{
	double dIntegerPart = floor(dNum);
	dNum -= dIntegerPart;
	for (int i = 0; i < dDECimalDigits; i++)
	{
		dNum *= 10;
	}
	dNum = floor(dNum + 0.5);
	for (int i = 0; i < dDECimalDigits; i++)
	{
		dNum /= 10;
	}
	return dIntegerPart + dNum;
}

void DelFiles(CString directory_path)
{
	if (directory_path.Right(1) != '\\')
	{
		directory_path += "\\";
	}
	CFileFind finder;
	CString path;
	path.Format("%s*.*", directory_path);
	BOOL bWorking = finder.FindFile(path);
	while (bWorking)
	{
		bWorking = finder.FindNextFile();
		if (finder.IsDirectory() && !finder.IsDots())
		{
			DelFiles(finder.GetFilePath());
			RemoveDirectory(finder.GetFilePath());
		}
		else
		{
			DeleteFile(finder.GetFilePath());
		}
	}
}

void DelFiles(CString directory_path, CString sFormat)
{
	if (directory_path.Right(1) != '\\')
	{
		directory_path += "\\";
	}
	CFileFind finder;
	CString path;
	path.Format("%s%s", directory_path, sFormat);
	BOOL bWorking = finder.FindFile(path);
	while (bWorking)
	{
		bWorking = finder.FindNextFile();
		if (finder.IsDirectory() && !finder.IsDots())
		{
			DelFiles(finder.GetFilePath());
			RemoveDirectory(finder.GetFilePath());
		}
		else
		{
			DeleteFile(finder.GetFilePath());
		}
	}
}

void DeleteFolder(CString sPath)
{
	CFileFind ff;
	BOOL bFound;
	bFound = ff.FindFile(sPath + "\\*.*");
	while (bFound)
	{
		bFound = ff.FindNextFile();
		CString sFilePath = ff.GetFilePath();

		if (ff.IsDirectory())
		{
			if (!ff.IsDots())
			{
				DeleteFolder(sFilePath);
			}
		}
		else
		{
			if (ff.IsReadOnly())
			{
				SetFileAttributes(sFilePath, FILE_ATTRIBUTE_NORMAL);
			}
			DeleteFile(sFilePath);
		}
	}
	ff.Close();
	//上面已经把文件夹下面的文件全部删除了，如果需要把文件夹也删除掉则加上一下代码
	//	SetFileAttributes(sPath, FILE_ATTRIBUTE_NORMAL);  //设置文件夹的属性
	//	RemoveDirectory(sPath);  //删除文件夹
}

void WriteJOB(int nLoopCount, int nStartPos, int nEndPos, E_ROBOT_CONTROL_CABINET_MODEL eCabinetModel, int nExternalAxle)
{
	int nMinStartPos;
	int nMaxEndPos;
	COPini opini;
	opini.SetFileName(JOB_INI);
	opini.SetSectionName("JOB");
	opini.ReadString("MinStartPos", &nMinStartPos);
	opini.ReadString("MaxEndPos", &nMaxEndPos);

	if (nStartPos < nMinStartPos || nEndPos > nMaxEndPos)
	{
		XiMessageBox("Warning:设定的P变量范围超出限制！");
		return;
	}

	CString strFileName;
	if (nExternalAxle > 0)
	{
		strFileName.Format("POS%d-%d-%d-BP.JBI", nLoopCount, nStartPos, nEndPos);
	}
	else
	{
		strFileName.Format("POS%d-%d-%d.JBI", nLoopCount, nStartPos, nEndPos);
	}
	
	FILE *pfJBI;
	XI_fopen_s(&pfJBI, ".\\Data\\JOB\\" + strFileName, "w");
	fprintf_s(pfJBI, "/JOB\n");
	if (nExternalAxle > 0)
	{
		fprintf_s(pfJBI, "//NAME POS%d-%d-%d-BP\n", nLoopCount, nStartPos, nEndPos);
	}
	else
	{
		fprintf_s(pfJBI, "//NAME POS%d-%d-%d\n", nLoopCount, nStartPos, nEndPos);
	}
	
	fprintf_s(pfJBI, "//POS\n");
	fprintf_s(pfJBI, "///NPOS 0,0,0,%d,0,0\n", nEndPos - nStartPos + 1);
	fprintf_s(pfJBI, "///TOOL 1\n");
	fprintf_s(pfJBI, "///POSTYPE ROBOT\n");
	fprintf_s(pfJBI, "///RECTAN\n");
	fprintf_s(pfJBI, "///RCONF 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n");
	for (int i = nStartPos; i <= nEndPos; i++)
	{
		CString str;
		str.Format("%d", i);
		str = _T("P000") + str + _T("=1205.010,-290.940,-448.110,-104.3500,35.0000,-180.0000\n");
		fprintf_s(pfJBI, str);
	}
	fprintf_s(pfJBI, "//INST\n");
	fprintf_s(pfJBI, "///DATE 2019/12/25 15:26\n");
	fprintf_s(pfJBI, "///ATTR SC,RW\n");
	if (nExternalAxle > 0)
	{
		fprintf_s(pfJBI, "///GROUP1 RB1,BS1\n");
	}
	else
	{
		fprintf_s(pfJBI, "///GROUP1 RB1\n");
	}
	
	fprintf_s(pfJBI, "NOP\n");
	if (eCabinetModel == YRC1000)
	{
		fprintf_s(pfJBI, "HPVELON\n");
	}
	fprintf_s(pfJBI, "SET I001 0\n");
	for (int j = 0; j < nLoopCount; j++)
	{
		for (int i = nStartPos; i <= nEndPos; i++)
		{
			if (j == 0)//第一次循环
			{
				fprintf_s(pfJBI, "CALL JOB:SLEEP IF I002=1\n");
				CString str;
				str.Format("%d", i);
				str = _T("JUMP *P") + str + _T(" IF I002=1\n");
				fprintf_s(pfJBI, str);
			}
			else if (j == 1)//第二次循环
			{
				CString str;
				str.Format("%d", i);
				str = _T("*P") + str + _T("\n");
				fprintf_s(pfJBI, str);
			}
			CString str;
			str.Format("%d", i);
			CString strBP;
			if (nExternalAxle > 0)
			{
				strBP = _T(" BP0") + str;
			}
			else
			{
				strBP = _T("");
			}
			if (j == 0)
			{
				str = _T("MOVL P0") + str + strBP + _T(" V=I004\n");
			}
			else
			{
				str = _T("MOVL P0") + str + strBP + _T(" V=I0") + str + _T("\n");
			}		
			fprintf_s(pfJBI, str);
			fprintf_s(pfJBI, "INC I001\n");
		}
	}
	fprintf_s(pfJBI, "SET I001 -2\n");
	fprintf_s(pfJBI, "SET I002 -1\n");
	fprintf_s(pfJBI, "END\n");
	fclose(pfJBI);
}

void CalImgRect(CRect &rectShowWholeImage, int nImageWidth, int nImageHeight, int nItem, CDialog *pDlg)
{
	CRect rectDlgItem;
	pDlg->GetDlgItem(nItem)->GetWindowRect(&rectDlgItem);
	pDlg->ScreenToClient(&rectDlgItem);
	pDlg->GetDlgItem(nItem)->GetClientRect(&rectShowWholeImage);
	if ((double)rectDlgItem.Width() / rectDlgItem.Height() > (double)nImageWidth / nImageHeight)
	{
		int nDrawGlobalWidth = (double)nImageWidth / nImageHeight * rectDlgItem.Height();
		int nOffsetX = (rectDlgItem.Width() - nDrawGlobalWidth) / 2;
		rectDlgItem.DeflateRect(nOffsetX, 0);
		rectShowWholeImage.DeflateRect(nOffsetX, 0);
	}
	else
	{
		int nDrawGlobalHeight = (double)nImageHeight / nImageWidth * rectDlgItem.Width();
		int nOffsetY = (rectDlgItem.Height() - nDrawGlobalHeight) / 2;
		rectDlgItem.DeflateRect(0, nOffsetY);
		rectShowWholeImage.DeflateRect(0, nOffsetY);
	}
}

void ShowStepPic(int &nStepNo, int &nPicNo, CString strPicPath, int nItem, CDialog *pDlg)
{
	//限制最小值
	if (nStepNo < 0)
	{
		nStepNo = 0;
	}
	if (nPicNo < 1)
	{
		nPicNo = 1;
	}

	//实际图片路径
	CString strPicName;
	strPicName.Format("%d-%d", nStepNo + 1, nPicNo);
	strPicName = strPicPath + strPicName + _T(".jpg");

	//如该图片不存在
	if (!CheckFileExists(strPicName))
	{
		nPicNo--;
		strPicName.Format("%d-%d", nStepNo + 1, nPicNo);
		strPicName = ROBOT_INSTRUCTION + strPicName + _T(".jpg");
	}
	if (!CheckFileExists(strPicName))
	{
		return;
	}

	//显示
	IplImage *pImageIdentifyResult = cvLoadImage(strPicName);
	DrawImage(pImageIdentifyResult, pDlg, nItem);
}

void CalDeadLine(SYSTEMTIME & tDeadline, SYSTEMTIME tTime, long lDate)
{
	ULARGE_INTEGER fTime1;
	SystemTimeToFileTime(&tTime, (FILETIME*)&fTime1);
	unsigned __int64 dft = 864000000000 * fabs((long double)lDate);
	if (lDate > 0)
	{
		fTime1.QuadPart = fTime1.QuadPart + dft;
	} 
	else
	{
		fTime1.QuadPart = fTime1.QuadPart - dft;
	}	
	FileTimeToSystemTime((FILETIME*)&fTime1, &tDeadline);
}

errno_t XI_fopen_s(FILE **file, CString strFileName, _In_z_ const char * _Mode, int nShFlag)
{
	errno_t err = 0;
	if (nShFlag != _SH_SECURE)
	{
		*file = _fsopen(strFileName, _Mode, nShFlag);
		if (NULL == *file)
		{
			err = ENOENT;
		}
	} 
	else
	{
		err = fopen_s(file, strFileName, _Mode);	
	}	
	if (err != 0)
	{
		CString strMessage;
		strMessage.Format("Error:文件打开失败！错误码：%d", (int)err);
		XiMessageBox(strFileName + strMessage);
	}
	return err;
}

bool CStringToULongLong(unsigned long long & ullNum, CString strNum)
{
	ullNum = 0;
	CString str = strNum;
	str.MakeLower();
	if (str.GetLength() != 18)
	{
		return false;
	}
	if (str[0] != '0' || str[1] != 'x')
	{
		return false;
	}
	for (int i = 2; i < 18; i++)
	{
		if (str[i] >= '0' && str[i] <= '9')
		{
			ullNum = ullNum * 16 + str[i] - '0';
		} 
		else if(str[i] >= 'a' && str[i] <= 'f')
		{
			ullNum = ullNum * 16 + str[i] - 'a' + 10;
		}
		else
		{
			return false;
		}
	}
	return true;
}

bool ULongLongToCString(CString &strNum, unsigned long long ullNum)
{
	CString strCurNum;
	int nCurNum = 0;
	char cCurNum = 'a';
	strNum = _T("");
	for (int i = 0; i < 16; i++)
	{
		nCurNum = ullNum >> (i*4) & 0x000000000000000F;
		if (nCurNum >= 0 && nCurNum <= 9)
		{
			strCurNum.Format("%d", nCurNum);
		} 
		else if(nCurNum >= 10 && nCurNum <= 15)
		{
			cCurNum = 'a' + nCurNum - 10;
			strCurNum.Format("%c", cCurNum);
		}
		else
		{
			return false;
		}
		strNum += strCurNum;
	}
	strNum += "x0";
	strNum.MakeReverse();
	return true;
}

CString SystemTimeToCString(SYSTEMTIME tTime)
{
	CString str;
	str.Format("%4d-%2d-%2d %2d:%2d:%2d", tTime.wYear, tTime.wMonth, tTime.wDay, tTime.wHour, tTime.wMinute, tTime.wSecond);
	return str;//2022-03-04 16:03:00
}

SYSTEMTIME CStringToSystemTime(CString sTime)
{
	SYSTEMTIME tTime;
	CString str;
	str = sTime.Mid(0, 4);
	tTime.wYear = atoi(str);
	str = sTime.Mid(5, 2);
	tTime.wMonth = atoi(str);
	str = sTime.Mid(8, 2);
	tTime.wDay = atoi(str);
	str = sTime.Mid(11, 2);
	tTime.wHour = atoi(str);
	str = sTime.Mid(14, 2);
	tTime.wMinute = atoi(str);
	str = sTime.Mid(17, 2);
	tTime.wSecond = atoi(str);
	return tTime;
}

__int64 TimeDiff(SYSTEMTIME t1, SYSTEMTIME t2)
{
	CTimeSpan			sp;
	int					s1, s2;

	CTime tm1(t1.wYear, t1.wMonth, t1.wDay, 0, 0, 0);
	CTime tm2(t2.wYear, t2.wMonth, t2.wDay, 0, 0, 0);

	sp = tm1 - tm2;

	s1 = t1.wHour * 3600 + t1.wMinute * 60 + t1.wSecond;
	s2 = t2.wHour * 3600 + t2.wMinute * 60 + t2.wSecond;

	return  sp.GetDays() * 86400 + (s1 - s2);
}

IplImage * LoadImage(CString strImgName)
{
	return cvLoadImage((LPCTSTR)strImgName);
}

int SaveImage(IplImage *src, CString strImgName)
{
	return cvSaveImage((LPCTSTR)strImgName, src);
}

int SaveImage(IplImage *src, char *format, ...)
{
	char   cRandomStrName[1000];
	va_list args;
	va_start(args, format);
	vsprintf(cRandomStrName, format, args);
	va_end(args);
	int p[3];
	p[0] = CV_IMWRITE_JPEG_QUALITY;
	p[1] = 10;
	p[2] = 0;
	return cvSaveImage(cRandomStrName, src, p);
}

int SaveImage(IplImage **src, char *format, ...)
{
	char   cRandomStrName[1000];
	va_list args;
	va_start(args, format);
	vsprintf(cRandomStrName, format, args);
	va_end(args);
	int p[3];
	p[0] = CV_IMWRITE_JPEG_QUALITY;
	p[1] = 10;
	p[2] = 0;
	return cvSaveImage(cRandomStrName, *src, p);
}

CvRect ResizeImage(IplImage* img, IplImage* &dst, double dRatio, CvPoint &tCenter)
{
	if (dRatio > 1.0)//放大
	{
		//创建一个四倍大小的图片
		int nWidth = img->width;
		int nHeight = img->height;
		IplImage* src = cvCreateImage(cvSize(2 * nWidth, 2 * nHeight), img->depth, img->nChannels);

		//加载原图
		int nOffsetX = nWidth / 2.0;
		int nOffsetY = nHeight / 2.0;
		cv::Mat image1(img, false);
		cv::Mat image2(src, false);
		cv::Mat roi = image2(cv::Rect(nOffsetX, nOffsetY, nWidth, nHeight));
		image1.copyTo(roi);

		// 设置 img 的 ROI 区域	
		CvRect rect;		
		rect.width = nWidth / dRatio;
		rect.height = nHeight / dRatio;
		rect.x = tCenter.x + nOffsetX - (rect.width / 2.0);
		rect.y = tCenter.y + nOffsetY - (rect.height / 2.0);
		cvSetImageROI(src, rect);

		// 创建等大的新图片
		IplImage *BeforeResize = cvCreateImage(cvSize(rect.width, rect.height), img->depth, img->nChannels);

		// ROI区域内图片复制到新图片
		cvCopy(src, BeforeResize, 0);

		//放大新图片
		CvSize sz;
		sz.width = nWidth;
		sz.height = nHeight;
		dst = cvCreateImage(sz, img->depth, img->nChannels);
		cvResize(BeforeResize, dst, CV_INTER_CUBIC);

		// 重置 src 的 ROI
		cvResetImageROI(src);

		//释放资源
		cvReleaseImage(&BeforeResize);
		cvReleaseImage(&src);

		return rect;
	} 
	else//缩小
	{
		// 创建等大的新图片
		int nWidth = img->width;
		int nHeight = img->height;
		dst = cvCreateImage(cvSize(nWidth, nHeight), img->depth, img->nChannels);

		//获取一个缩小后的图片
		CvSize sz;
		sz.width = img->width * dRatio;
		sz.height = img->height * dRatio;
		IplImage *desc = cvCreateImage(sz, img->depth, img->nChannels);
		cvResize(img, desc, CV_INTER_CUBIC);

		//将缩小后的图片贴到新图片中
		int nOffsetX = nWidth / 2.0 - sz.width;
		int nOffsetY = nHeight / 2.0 - sz.height;
		cv::Mat image1(desc, false);
		cv::Mat image2(dst, false);
		cv::Mat roi = image2(cv::Rect(nOffsetX, nOffsetY, sz.width, sz.height));
		image1.copyTo(roi);

		//释放资源
		cvReleaseImage(&desc);

		CvRect rect;
		rect.x = 0;
		rect.y = 0;
		rect.width = nWidth;
		rect.height = nHeight;
		return rect;
	}
}

void DrawImage(IplImage *pImage, CDialog *pDialog, int nStaticId, bool bDrawCross)
{
	//计算
	CRect rectImage;
	CalImgRect(rectImage, pImage->width, pImage->height, nStaticId, pDialog);

	//显示
	DrawImage(pImage, pDialog, nStaticId, rectImage, bDrawCross);

}

void DrawImage(IplImage* pImage, CDialog* pDialog, int nStaticId, CRect rectImage, bool bDrawCross)
{
	CDC* pDC = pDialog->GetDlgItem(nStaticId)->GetDC();			// 获得显示控件的DC
	HDC hDC = pDC->GetSafeHdc();								// 获取 HDC(设备句柄)来进行绘图操作
	CvvImage cimg;
	cimg.CopyOf(pImage);								// 复制图片
	cimg.DrawToHDC(hDC, &rectImage);					// 将图片绘制到显示控件的指定区域内

	if (bDrawCross)
	{
		CPen newPen;       // 用于创建新画笔   
		CPen* pOldPen;     // 用于存放旧画笔   

		// 创建实心画笔，粗度为1，颜色为白色   
		newPen.CreatePen(PS_SOLID, 1, RGB(255, 0, 255));
		// 选择新画笔，并将旧画笔的指针保存到pOldPen   
		pOldPen = pDC->SelectObject(&newPen);

		// 将当前点移动到绘图控件窗口的左中间位置，以此为起始点   
		pDC->MoveTo(rectImage.left, (rectImage.bottom - rectImage.top) / 2 + rectImage.top);
		pDC->LineTo(rectImage.right, (rectImage.bottom - rectImage.top) / 2 + rectImage.top);

		pDC->MoveTo((rectImage.right - rectImage.left) / 2 + rectImage.left, rectImage.top);
		pDC->LineTo((rectImage.right - rectImage.left) / 2 + rectImage.left, rectImage.bottom);

		// 恢复旧画笔   
		pDC->SelectObject(pOldPen);
		// 删除新画笔   
		newPen.DeleteObject();
	}

	pDialog->ReleaseDC(pDC);
}
void DrawImage_new(IplImage* pImage, CDialog* pDialog, int nStaticId, CRect rectImage, bool bDrawCross)
{
	CRect m_cClientRect;//画布范围

	pDialog->GetDlgItem(nStaticId)->GetClientRect(m_cClientRect);

	CDC m_memDC;//显示缓冲CDC
	CBitmap m_memBitmap;//显示缓冲画布
	CRect rect = m_cClientRect;
	m_memBitmap.DeleteObject();
	m_memDC.DeleteDC();

	CDC* pDC = pDialog->GetDlgItem(nStaticId)->GetDC();			// 获得显示控件的DC

	m_memDC.CreateCompatibleDC(pDC);//创建与目标DC相兼容的内存DC，
	m_memBitmap.CreateCompatibleBitmap(pDC, m_cClientRect.Width(), m_cClientRect.Height());//根据目标DC创建位图
	m_memDC.SelectObject(&m_memBitmap);//把位图选入内存DC
	m_memDC.FillSolidRect(rect, pDC->GetBkColor());

	HDC hDC = m_memDC.GetSafeHdc();								// 获取 HDC(设备句柄)来进行绘图操作
	CvvImage cimg;
	cimg.LoadOf(pImage);								// 复制图片

	cimg.DrawToHDC(hDC, &rectImage);					// 将图片绘制到显示控件的指定区域内

	if (bDrawCross)
	{
		CPen newPen;       // 用于创建新画笔   
		CPen* pOldPen;     // 用于存放旧画笔   

		// 创建实心画笔，粗度为1，颜色为白色   
		newPen.CreatePen(PS_SOLID, 1, RGB(255, 0, 255));
		// 选择新画笔，并将旧画笔的指针保存到pOldPen   
		pOldPen = pDC->SelectObject(&newPen);

		// 将当前点移动到绘图控件窗口的左中间位置，以此为起始点   
		pDC->MoveTo(rectImage.left, (rectImage.bottom - rectImage.top) / 2 + rectImage.top);
		pDC->LineTo(rectImage.right, (rectImage.bottom - rectImage.top) / 2 + rectImage.top);

		pDC->MoveTo((rectImage.right - rectImage.left) / 2 + rectImage.left, rectImage.top);
		pDC->LineTo((rectImage.right - rectImage.left) / 2 + rectImage.left, rectImage.bottom);

		// 恢复旧画笔   
		pDC->SelectObject(pOldPen);
		// 删除新画笔   
		newPen.DeleteObject();
	}
	pDC->BitBlt(m_cClientRect.left, m_cClientRect.top, m_cClientRect.Width(), m_cClientRect.Height(), &m_memDC, 0, 0, SRCCOPY);
	m_memBitmap.DeleteObject();
	m_memDC.DeleteDC();
	pDialog->ReleaseDC(pDC);
}

bool CheckContrarySign(double dNum1, double dNum2)
{
	if ((dNum1 > 0 && dNum2 < 0)
		|| (dNum1 < 0 && dNum2 > 0))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CheckContrarySign(long lNum1, long lNum2)
{
	if ((lNum1 > 0 && lNum2 < 0)
		|| (lNum1 < 0 && lNum2 > 0))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void LoadRobotPara(CString strRobotName, T_KINEMATICS &tKinematics, T_AXISUNIT &tAxisUnit, T_AXISLIMITANGLE &tAxisLimitAngle)
{
	COPini opini;
	opini.SetFileName(DATA_PATH + strRobotName + ROBOT_PARA_INI);
	opini.SetSectionName("Kinematics");
	opini.ReadString("dH1", &tKinematics.dH1);
	opini.ReadString("dA1", &tKinematics.dA1);
	opini.ReadString("dA2", &tKinematics.dA2);
	opini.ReadString("dA3", &tKinematics.dA3);
	opini.ReadString("dD3", &tKinematics.dD3);
	opini.ReadString("dD4", &tKinematics.dD4);
	opini.ReadString("dD5", &tKinematics.dD5);

	double dAngle = 0;
	double dPulse = 0;
	opini.ReadString("dSAngle", &dAngle);
	opini.ReadString("dSPulse", &dPulse);
	tAxisUnit.dSPulse = dAngle / dPulse;
	opini.ReadString("dLAngle", &dAngle);
	opini.ReadString("dLPulse", &dPulse);
	tAxisUnit.dLPulse = dAngle / dPulse;
	opini.ReadString("dUAngle", &dAngle);
	opini.ReadString("dUPulse", &dPulse);
	tAxisUnit.dUPulse = dAngle / dPulse;
	opini.ReadString("dRAngle", &dAngle);
	opini.ReadString("dRPulse", &dPulse);
	tAxisUnit.dRPulse = dAngle / dPulse;
	opini.ReadString("dBAngle", &dAngle);
	opini.ReadString("dBPulse", &dPulse);
	tAxisUnit.dBPulse = dAngle / dPulse;
	opini.ReadString("dTAngle", &dAngle);
	opini.ReadString("dTPulse", &dPulse);
	tAxisUnit.dTPulse = dAngle / dPulse;

	opini.ReadString("dMaxPosSAngle", &tAxisLimitAngle.dMaxPosSAngle);
	opini.ReadString("dMaxNegSAngle", &tAxisLimitAngle.dMaxNegSAngle);
	opini.ReadString("dMaxPosLAngle", &tAxisLimitAngle.dMaxPosLAngle);
	opini.ReadString("dMaxNegLAngle", &tAxisLimitAngle.dMaxNegLAngle);
	opini.ReadString("dMaxPosUAngle", &tAxisLimitAngle.dMaxPosUAngle);
	opini.ReadString("dMaxNegUAngle", &tAxisLimitAngle.dMaxNegUAngle);
	opini.ReadString("dMaxPosRAngle", &tAxisLimitAngle.dMaxPosRAngle);
	opini.ReadString("dMaxNegRAngle", &tAxisLimitAngle.dMaxNegRAngle);
	opini.ReadString("dMaxPosBAngle", &tAxisLimitAngle.dMaxPosBAngle);
	opini.ReadString("dMaxNegBAngle", &tAxisLimitAngle.dMaxNegBAngle);
	opini.ReadString("dMaxPosTAngle", &tAxisLimitAngle.dMaxPosTAngle);
	opini.ReadString("dMaxNegTAngle", &tAxisLimitAngle.dMaxNegTAngle);
}

void WriteAdjustMatchInfo(CString sDepthDataPath, double dCameraDistance, std::vector<T_RECOGNITION_PARA> vtMatchInfo, bool bLineScan)
{
	if (bLineScan)
	{
		WriteAdjustMatchInfoForLineScan(sDepthDataPath, dCameraDistance, vtMatchInfo);
		return;
	}
	CString sRealResultFileName = sDepthDataPath + "\\panoRecognition\\处理结果\\MatchInfo.txt";
	FILE *pfRecognition;
	XI_fopen_s(&pfRecognition, sRealResultFileName, "w");
	fprintf_s(pfRecognition, "模板号：  工件轮廓中心点所在相机号：  工件轮廓中心所在相机位置：  工件轮廓中心点X:   工件轮廓中心点Y:   工件轮廓中心点Z:   顺时针选择角度：   工件编号：   工件厚度：    摞号：  \n");
	for (int nIdx = 0; nIdx < vtMatchInfo.size(); nIdx++)
	{
		fprintf_s(pfRecognition, "%d   %d   %d   %lf   %lf   %lf   %lf   %d   %d\n", vtMatchInfo[nIdx].nTemplateNo, vtMatchInfo[nIdx].nCameraNo,
			vtMatchInfo[nIdx].nCameraPos, (vtMatchInfo[nIdx].dTranslateX - dCameraDistance*(vtMatchInfo[nIdx].nCameraNo)), -vtMatchInfo[nIdx].dTranslateY, vtMatchInfo[nIdx].dPartHeight,
			-vtMatchInfo[nIdx].dAngleRotate, vtMatchInfo[nIdx].nLayerNo, vtMatchInfo[nIdx].nGroupNo);
	}
	fclose(pfRecognition);
}

bool LoadMatchInfo(CString sDepthDataPath, double dCameraDistance, std::vector<T_RECOGNITION_PARA> &vtIdentifyPart, bool bLineScan)
{
	if (bLineScan)
	{
		return LoadMatchInfoForLineScan(sDepthDataPath, dCameraDistance, vtIdentifyPart);
	}
	vtIdentifyPart.clear();
	CString sRealResultFileName = sDepthDataPath + "\\panoRecognition\\处理结果\\MatchInfo.txt";

	if (!CheckFileExists(sRealResultFileName))
	{
		return false;
	}

	char acTmpArr[1000];
	FILE *pfRecognition;
	if (0 != XI_fopen_s(&pfRecognition, sRealResultFileName, "r"))
	{
		return false;
	}
	fgets(acTmpArr, 1000, pfRecognition);

	int nCameraNo = 0;
	int nCameraPos = 0;
	int nTemplateNo = 0;
	double dPartHeight = 0.0;
	double dAngleRotate = 0.0;
	double dTranslateX = 0.0;
	double dTranslateY = 0.0;
	int nLayerNo = 0;
	int nGroupNo = 0;
	T_RECOGNITION_PARA tReconnitionPara;
	while (fscanf_s(pfRecognition, "%d%d%d%lf%lf%lf%lf%d%d", &nTemplateNo, &nCameraNo, &nCameraPos, &dTranslateX, &dTranslateY, &dPartHeight, &dAngleRotate, &nLayerNo, &nGroupNo) > 0)
	{
		tReconnitionPara.nCameraNo = nCameraNo;
		tReconnitionPara.nCameraPos = nCameraPos;
		tReconnitionPara.nTemplateNo = nTemplateNo;
		tReconnitionPara.dPartHeight = dPartHeight;
		tReconnitionPara.dAngleRotate = -dAngleRotate;
		tReconnitionPara.dTranslateX = (dTranslateX + dCameraDistance*nCameraNo);
		tReconnitionPara.dTranslateY = -dTranslateY;
		tReconnitionPara.nLayerNo = nLayerNo;
		tReconnitionPara.nGroupNo = nGroupNo;

		vtIdentifyPart.push_back(tReconnitionPara);
	}
	fclose(pfRecognition);

	return true;
}

void WriteAdjustMatchInfoForLineScan(CString sDepthDataPath, double dCameraDistance, std::vector<T_RECOGNITION_PARA> vtMatchInfo)
{
	CString sRealResultFileName = sDepthDataPath + "\\panoRecognition\\处理结果\\MatchInfo.txt";
	FILE *pfRecognition;
	XI_fopen_s(&pfRecognition, sRealResultFileName, "w");
	fprintf_s(pfRecognition, "模板号：  工件轮廓中心点所在相机号：  工件轮廓中心所在相机位置：  工件轮廓中心点X:   工件轮廓中心点Y:   工件轮廓中心点Z:   顺时针选择角度：   工件编号：   工件厚度：    摞号：  \n");
	for (int nIdx = 0; nIdx < vtMatchInfo.size(); nIdx++)
	{
		fprintf_s(pfRecognition, "%d   %d   %d   %lf   %lf   %lf   %lf   %d   %d\n", vtMatchInfo[nIdx].nTemplateNo, vtMatchInfo[nIdx].nCameraNo,
			vtMatchInfo[nIdx].nCameraPos, vtMatchInfo[nIdx].dTranslateX, vtMatchInfo[nIdx].dTranslateY, vtMatchInfo[nIdx].dPartHeight,
			-vtMatchInfo[nIdx].dAngleRotate, vtMatchInfo[nIdx].nLayerNo, vtMatchInfo[nIdx].nGroupNo);
	}
	fclose(pfRecognition);
}

bool LoadMatchInfoForLineScan(CString sDepthDataPath, double dCameraDistance, std::vector<T_RECOGNITION_PARA> &vtIdentifyPart)
{
	vtIdentifyPart.clear();
	CString sRealResultFileName = sDepthDataPath + "\\panoRecognition\\处理结果\\MatchInfo.txt";

	if (!CheckFileExists(sRealResultFileName))
	{
		return false;
	}

	char acTmpArr[1000];
	FILE *pfRecognition;
	if (0 != XI_fopen_s(&pfRecognition, sRealResultFileName, "r"))
	{
		return false;
	}
	fgets(acTmpArr, 1000, pfRecognition);

	int nCameraNo = 0;
	int nCameraPos = 0;
	int nTemplateNo = 0;
	double dPartHeight = 0.0;
	double dAngleRotate = 0.0;
	double dTranslateX = 0.0;
	double dTranslateY = 0.0;
	int nLayerNo = 0;
	int nGroupNo = 0;
	T_RECOGNITION_PARA tReconnitionPara;
	while (fscanf_s(pfRecognition, "%d%d%d%lf%lf%lf%lf%d%d", &nTemplateNo, &nCameraNo, &nCameraPos, &dTranslateX, &dTranslateY, &dPartHeight, &dAngleRotate, &nLayerNo, &nGroupNo) > 0)
	{
		tReconnitionPara.nCameraNo = nCameraNo;
		tReconnitionPara.nCameraPos = nCameraPos;
		tReconnitionPara.nTemplateNo = nTemplateNo;
		tReconnitionPara.dPartHeight = dPartHeight;
		tReconnitionPara.dAngleRotate = -dAngleRotate;
		tReconnitionPara.dTranslateX = dTranslateX;
		tReconnitionPara.dTranslateY = dTranslateY;
		tReconnitionPara.nLayerNo = nLayerNo;
		tReconnitionPara.nGroupNo = nGroupNo;

		vtIdentifyPart.push_back(tReconnitionPara);
	}
	fclose(pfRecognition);

	return true;
}

double CalcTriangleSideLength(double dSide1Length, double dSide2Length, double dAngle)
{
	double dOppositeSideLength = dSide1Length * dSide1Length + dSide2Length * dSide2Length
		- 2 * dSide1Length * dSide2Length * cos(dAngle * PI / 180.0);
	dOppositeSideLength = sqrt(dOppositeSideLength);
	return dOppositeSideLength;
}

double CalcTriangleDiagonal(double dSide1Length, double dSide2Length, double dOppositeSideLength)
{
	double dAngle = acos((dSide1Length * dSide1Length + dSide2Length * dSide2Length - dOppositeSideLength * dOppositeSideLength)
		/ (2 * dSide1Length * dSide2Length));
	dAngle = dAngle * 180 / PI;
	return dAngle;
}

CString CalCRCData(unsigned long lData)
{
	unsigned long lPony = 0x13;//10011,5-1=4位
	int nCRCBitNum = 4;
	int nDataBitNum = 8;
	unsigned long lRegi = 0x00;

	lData <<= nCRCBitNum;
	for (int i = nCRCBitNum + nDataBitNum - 1; i >= 0; i--)
	{
		if (((lRegi >> nCRCBitNum) & 0x01) == 0x01)
		{
			lRegi = lRegi ^ lPony;
		}
		lRegi <<= 1;
		unsigned int nTmp = (lData >> i) & 0x01;
		lRegi |= nTmp;
	}
	if (((lRegi >> nCRCBitNum) & 0x01) == 0x01)
	{
		lRegi = lRegi ^ lPony;
	}

	CString str;
	ULongLongToCString(str, lRegi);
	return str;
}

bool GetFileNameFromPath(CString sFilePath, CString &sFileName, bool bSuffix /*= true*/)
{
	int nPos = sFilePath.ReverseFind('\\');
	sFileName = sFilePath;
	sFileName.Delete(0, nPos + 1);
	if (!bSuffix)
	{
		nPos = sFileName.ReverseFind('.');
		sFileName = sFileName.Left(nPos);
	}
	return true;
}

bool GetFilePathFromPath(CString sPath, CString &sFilePath)
{
	int nPos = sPath.ReverseFind('\\');
	sFilePath = sPath.Left(nPos+1);
	return true;
}

int CalMinSampleGap(double dSpeed)
{
	int nMinSampleGap = (dSpeed + 200.0) / 1800.0;
	if (nMinSampleGap < 2)
	{
		nMinSampleGap = 2;
	}
	return nMinSampleGap;
}

long long XI_clock()
{
	LARGE_INTEGER lpFrequency, lpPerformanceCount;
	QueryPerformanceFrequency(&lpFrequency);
	QueryPerformanceCounter(&lpPerformanceCount);
	return 1000 * lpPerformanceCount.QuadPart / lpFrequency.QuadPart;
}

void PicMean(std::vector<cv::Mat > vInPutDepth, cv::Mat &OutDepth)
{
	if (vInPutDepth.size() == 0)
	{
		OutDepth = cv::Mat();
		return;
	}
	if (vInPutDepth.size() == 1)
	{
		OutDepth = vInPutDepth[0].clone();
		return;
	}
	double nSize = vInPutDepth.size();
	cv::addWeighted(vInPutDepth[0], 0.5, vInPutDepth[1], 0.5, 0, OutDepth);
	for (size_t i = 2; i < vInPutDepth.size(); i++)
	{
		cv::addWeighted(vInPutDepth[i], (1 / double(i + 1)), OutDepth, (double(i) / double(i + 1)), 0, OutDepth);
	}
	return;
}

void PicMeanRemove0(std::vector<cv::Mat > vInPutDepth, cv::Mat &OutDepth)
{
	if (vInPutDepth.size() == 0)
	{
		OutDepth = cv::Mat();
		return;
	}
	if (vInPutDepth.size() == 1)
	{
		OutDepth = vInPutDepth[0].clone();
		return;
	}
	int nRows = vInPutDepth[0].rows;
	int nCols = vInPutDepth[0].cols;
	for (size_t i = 0; i < vInPutDepth.size(); ++i)
	{
		if (vInPutDepth[i].rows != nRows || vInPutDepth[i].cols != nCols || vInPutDepth[i].type() != CV_16U)
		{
			OutDepth = vInPutDepth[0].clone();
			return;
		}
	}

	OutDepth = cv::Mat::zeros(vInPutDepth[0].size(), vInPutDepth[0].type());
	ushort nUblack = 0;
	ushort Sum = 0;
	for (size_t row = 0; row < nRows; ++row)
	{
		for (size_t col = 0; col < nCols; ++col)
		{
			Sum = 0;
			nUblack = 0;
			for (size_t nNum = 0; nNum < vInPutDepth.size(); nNum++)
			{
				ushort depth = vInPutDepth[nNum].ptr<ushort>(row)[col];
				if (depth != 0)
				{
					Sum += depth;
					nUblack++;
				}
			}
			if (Sum != 0)
			{
				OutDepth.ptr<ushort>(row)[col] = Sum / nUblack;
			}
		}
	}
	return;
}

void PicMax(std::vector<cv::Mat > vInPutDepth, cv::Mat &OutDepth)
{
	if (vInPutDepth.size() == 0)
	{
		OutDepth = cv::Mat();
		return;
	}
	if (vInPutDepth.size() == 1)
	{
		OutDepth = vInPutDepth[0].clone();
		return;
	}
	int nRows = vInPutDepth[0].rows;
	int nCols = vInPutDepth[0].cols;
	for (size_t i = 0; i < vInPutDepth.size(); ++i)
	{
		if (vInPutDepth[i].rows != nRows || vInPutDepth[i].cols != nCols || vInPutDepth[i].type() != CV_16U)
		{
			OutDepth = vInPutDepth[0].clone();
			return;
		}
	}

	OutDepth = cv::Mat::zeros(vInPutDepth[0].size(), vInPutDepth[0].type());
	ushort uMax = 0;
	for (size_t row = 0; row < nRows; ++row)
	{
		for (size_t col = 0; col < nCols; ++col)
		{
			uMax = 0;
			for (size_t nNum = 0; nNum < vInPutDepth.size(); nNum++)
			{
				ushort depth = vInPutDepth[nNum].ptr<ushort>(row)[col];
				uMax = uMax > depth ? uMax : depth;
			}
			OutDepth.ptr<ushort>(row)[col] = uMax;
		}
	}
	return;
}

void Undistort(T_CAMERA_DISTORTION tCameraDistortion, IplImage *pImgGray, IplImage *pResultImgGray)
{
	if (!tCameraDistortion.bEnableUndistort)
	{
		cvCopyImage(pImgGray, pResultImgGray);
		return;
	}
	IplImage *src;
	cv::Mat mSrc(pImgGray, true);
	cv::Mat mDst;
	cv::Mat new_matrix;
	cv::undistort(mSrc, mDst, tCameraDistortion.cvCameraMatrix, tCameraDistortion.cvDistCoeffs, new_matrix);
	src = &IplImage(mDst);
	cvCopyImage(src, pResultImgGray);
	mSrc.release();
	mDst.release();
	mSrc = NULL;
	mDst = NULL;
	src = NULL;
}

// bool PicMean(std::vector<cv::Mat > vInPutDepth, cv::Mat &OutDepth)
// {
// 	//输入图片大小须一致
// 	if (vInPutDepth.size() == 0)
// 	{
// 		//OutDepth = cv::noArray();
// 		return false;
// 	}
// 	if (vInPutDepth.size() == 1)
// 	{
// 		OutDepth = vInPutDepth[0].clone();
// 		return true;
// 	}
// 	double nSize = vInPutDepth.size();
// 	double dRatio = 0;
// 	cv::addWeighted(vInPutDepth[0], 0.5, vInPutDepth[1], 0.5, 0, OutDepth);
// 	for (size_t i = 2; i < vInPutDepth.size(); i++)
// 	{
// 		dRatio = i + 1.0;
// 		cv::addWeighted(vInPutDepth[i], 1 / dRatio, OutDepth, i / dRatio, 0, OutDepth);
// 	}
// 	return true;
// }


void DelayUs(double uDelay)
{
	LARGE_INTEGER litmp;
	LONGLONG QPart1, QPart2;

	double dfMinus, dfFreq, dfTim; //定义变量

								   /*
								   Pointer to a variable that the function sets, in counts per second, to the current performance-counter frequency.
								   If the installed hardware does not support a high-resolution performance counter,
								   the value passed back through this pointer can be zero.

								   */
	QueryPerformanceFrequency(&litmp); //获取当前电脑的时钟频率

	dfFreq = (double)litmp.QuadPart;

	/*
	Pointer to a variable that the function sets, in counts, to the current performance-counter value.
	*/
	QueryPerformanceCounter(&litmp); //获取延时前的CPU执行次数

	QPart1 = litmp.QuadPart;
	do
	{
		QueryPerformanceCounter(&litmp);//获取延时后的CPU执行次数
		QPart2 = litmp.QuadPart;
		dfMinus = (double)(QPart2 - QPart1);//延时后的CPU执行次数 减去  延时前的CPU执行次数
		if (dfMinus < 0)
			break;
		dfTim = dfMinus / dfFreq * 1000000;
		//dfTim &#61; dfMinus/dfFreq;
	} while (dfTim< uDelay); //延时时间小于预定值&#xff0c;继续进行延时

}

void DelayMs(double uDelay)
{
	uDelay = uDelay * 1000.0;
	DelayUs(uDelay);
}

double XI_ContourRound(double r)
{
	return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

CString GetStr(char *format, ...)
{
	va_list args;
	va_start(args, format);
	char s[1000];
	vsprintf_s(s, 1000, format, args);
	va_end(args);
	CString str;
	str.Format("%s", s);
	return str;
}

CString GetStr(CString str)
{
	return str;
}

CString GetStr(T_ROBOT_COORS tCoor, CString strToken /*= ","*/)
{
	CString str;
	str.Format("%lf%s%lf%s%lf%s%lf%s%lf%s%lf%s%lf%s%lf%s%lf",
		tCoor.dX, strToken, tCoor.dY, strToken, tCoor.dZ, strToken,
		tCoor.dRX, strToken, tCoor.dRY, strToken, tCoor.dRZ, strToken, 
		tCoor.dBX, strToken, tCoor.dBY, strToken, tCoor.dBZ, strToken);
	return str;
}

CString GetStr(T_ANGLE_PULSE tPulse, CString strToken)
{
	CString str;
	str.Format("%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d",
		tPulse.nSPulse, strToken, tPulse.nLPulse, strToken, tPulse.nUPulse, strToken,
		tPulse.nRPulse, strToken, tPulse.nBPulse, strToken, tPulse.nTPulse, strToken,
		tPulse.lBXPulse, strToken, tPulse.lBYPulse, strToken, tPulse.lBZPulse, strToken);
	return str;
}

//字符串分割，sToken是多个字符时，每个字符都是分隔符
std::vector<CString> TokenizeCString(CString sSrc, CString sToken)
{
	std::vector<CString> vsRtnVal;
	vsRtnVal.clear();
	CString sTemp;
	sTemp.Empty();
	int nPos = 0;
	while (true)
	{
		sTemp = sSrc.Tokenize(sToken, nPos);
		if (!sTemp.IsEmpty())
		{
			vsRtnVal.push_back(sTemp);
		}
		else
		{
			break;
		}
	}
	return vsRtnVal;
}

BOOL CheckRtnResult(T_RUNNING_RTN_RESULT tRtnResult)
{
	if (tRtnResult.veRtnState.back() == RTN_STATUS_SUCCEED || tRtnResult.veRtnState.back() == RTN_STATUS_CAMPLATE)
	{
		return TRUE;
	}
	return FALSE;
}

BOOL AddRtnData(T_RUNNING_RTN_RESULT &tRtnResult, E_RUNNING_RTN_STATE eRtnState, CString strFunction, int nLine, CString strRtnExplain, int nDataType, void *vData)
{
	tRtnResult.veRtnState.push_back(eRtnState);
	tRtnResult.vsFuntionName.push_back(strFunction);
	tRtnResult.vsRtnExplain.push_back(strRtnExplain);
	tRtnResult.vnRtnDataType.push_back(nDataType);
	tRtnResult.vpRtnData.push_back(vData);
	if (FALSE == CheckRtnResult(tRtnResult))
	{
		WriteLog("[Function]:%s, [Line]:%d, [State]:%x, [Explain]:%s", strFunction, nLine, (int)eRtnState, strRtnExplain);
		return FALSE;
	}
	return TRUE;
}

BOOL AddRtnData(T_RUNNING_RTN_RESULT &tRtnResult, T_RUNNING_RTN_RESULT tRtnData)
{
	tRtnResult.veRtnState.insert(tRtnResult.veRtnState.end(), tRtnData.veRtnState.begin(), tRtnData.veRtnState.end());
	tRtnResult.vsFuntionName.insert(tRtnResult.vsFuntionName.end(), tRtnData.vsFuntionName.begin(), tRtnData.vsFuntionName.end());
	tRtnResult.vsRtnExplain.insert(tRtnResult.vsRtnExplain.end(), tRtnData.vsRtnExplain.begin(), tRtnData.vsRtnExplain.end());
	tRtnResult.vnRtnDataType.insert(tRtnResult.vnRtnDataType.end(), tRtnData.vnRtnDataType.begin(), tRtnData.vnRtnDataType.end());
	tRtnResult.vpRtnData.insert(tRtnResult.vpRtnData.end(), tRtnData.vpRtnData.begin(), tRtnData.vpRtnData.end());
	if (FALSE == CheckRtnResult(tRtnResult))
	{
		return FALSE;
	}
	return TRUE;
}

E_RUNNING_RTN_STATE GetRtnState(T_RUNNING_RTN_RESULT tRtnResult)
{
	if (tRtnResult.veRtnState.empty())
	{
		return RTN_STATUS_VOID;
	}
	return tRtnResult.veRtnState.back();
}

void TransforArray(T_ROBOT_COORS tCoor, double* adArray)
{
	adArray[0] = tCoor.dX;
	adArray[1] = tCoor.dY;
	adArray[2] = tCoor.dZ;
	adArray[3] = tCoor.dRX;
	adArray[4] = tCoor.dRY;
	adArray[5] = tCoor.dRZ;
}

void TransforArray(T_ANGLE_PULSE tPulse, long* alArray)
{
	alArray[0] = tPulse.nSPulse;
	alArray[1] = tPulse.nLPulse;
	alArray[2] = tPulse.nUPulse;
	alArray[3] = tPulse.nRPulse;
	alArray[4] = tPulse.nBPulse;
	alArray[5] = tPulse.nTPulse;
}

void TransforArray(T_ANGLE_PULSE tPulse, double* adArray)
{
	adArray[0] = tPulse.nSPulse;
	adArray[1] = tPulse.nLPulse;
	adArray[2] = tPulse.nUPulse;
	adArray[3] = tPulse.nRPulse;
	adArray[4] = tPulse.nBPulse;
	adArray[5] = tPulse.nTPulse;
}

T_ANGLE_PULSE TransforPulse(long* alArray)
{
	T_ANGLE_PULSE tPulse;
	tPulse.nSPulse = alArray[0];
	tPulse.nLPulse = alArray[1];
	tPulse.nUPulse = alArray[2];
	tPulse.nRPulse = alArray[3];
	tPulse.nBPulse = alArray[4];
	tPulse.nTPulse = alArray[5];
	return tPulse;
}

T_ANGLE_PULSE TransforPulse(double* adArray)
{
	T_ANGLE_PULSE tPulse;
	tPulse.nSPulse = long(adArray[0] + 0.5);
	tPulse.nLPulse = long(adArray[1] + 0.5);
	tPulse.nUPulse = long(adArray[2] + 0.5);
	tPulse.nRPulse = long(adArray[3] + 0.5);
	tPulse.nBPulse = long(adArray[4] + 0.5);
	tPulse.nTPulse = long(adArray[5] + 0.5);
	return tPulse;
}

T_ROBOT_COORS TransforPos(double* adArray)
{
	T_ROBOT_COORS tCoor;
	tCoor.dX = adArray[0];
	tCoor.dY = adArray[1];
	tCoor.dZ = adArray[2];
	tCoor.dRX = adArray[3];
	tCoor.dRY = adArray[4];
	tCoor.dRZ = adArray[5];
	return tCoor;
}

bool JudgeArea(double dAimPos, double dPos1, double dPos2)
{
	if (IsEqual(dAimPos, dPos1, 0.1) || IsEqual(dAimPos, dPos2, 0.1))
	{
		return true;
	}
	if (dPos1 > dPos2)
	{
		if (dAimPos < dPos1 && dAimPos > dPos2)
		{
			return true;
		}
	}
	else
	{
		if (dAimPos < dPos2 && dAimPos > dPos1)
		{
			return true;
		}
	}
	return false;
}

void XI_ReleaseImage(IplImage** img)
{
	if (NULL != *img)
	{
		cvReleaseImage(img);
		*img = NULL;
	}
}

char* UnicodeToUtf8(char* str)
{
	// 1. char转wchar_t
	wchar_t* pwszUnicode;
	int iSize;
	iSize = MultiByteToWideChar(CP_ACP, 0, str, -1, NULL, 0);
	pwszUnicode = (wchar_t*)malloc(iSize * sizeof(wchar_t));
	MultiByteToWideChar(CP_ACP, 0, str, -1, pwszUnicode, iSize);
	// 2. wchar_t转u8char
	int len;
	len = WideCharToMultiByte(CP_UTF8, 0, pwszUnicode, -1, NULL, 0, NULL, NULL);
	char* szUtf8 = (char*)malloc((int)len + 1);
	memset(szUtf8, 0, len + 1);
	WideCharToMultiByte(CP_UTF8, 0, pwszUnicode, -1, szUtf8, len, NULL, NULL);
	return szUtf8;
}

void SaveErrorData(CString strName)
{
	//	 return;  ///离线测试20221231
	//for (int n = 0; n < m_nRobotNum; n++)
	//{
	//	T_ROBOT_COORS tCurrentCoor = GetCurrentPos(m_vpRobotDriver[n]);
	//	T_ANGLE_PULSE tCurrentPulse = GetCurrentPulse(m_vpRobotDriver[n]);
	//	WriteLog("-------->>保存错误数据，自动记录当前机器人坐标：\n%s:\n%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n%d,%d,%d,%d,%d,%d,%d,%d,%d",
	//		m_vpRobotDriver[n]->m_strRobotName,
	//		tCurrentCoor.dX, tCurrentCoor.dY, tCurrentCoor.dZ, tCurrentCoor.dRX, tCurrentCoor.dRY, tCurrentCoor.dRZ, tCurrentCoor.dBX, tCurrentCoor.dBY, tCurrentCoor.dBZ,
	//		tCurrentPulse.nSPulse, tCurrentPulse.nLPulse, tCurrentPulse.nUPulse, tCurrentPulse.nRPulse, tCurrentPulse.nBPulse, tCurrentPulse.nTPulse, tCurrentPulse.lBXPulse, tCurrentPulse.lBYPulse, tCurrentPulse.lBZPulse);
	//}

	//获取限制日期
	long lMaxDate = 100;
	COPini opini;
	opini.SetFileName(DEBUG_INI);
	opini.SetSectionName("Debug");
	opini.ReadString("MaxDateOfSaveErrorData", &lMaxDate);

	//获取时间
	SYSTEMTIME tTime;
	GetLocalTime(&tTime);
	CString strDate;
	strDate.Format("\\%.2d-%.2d", tTime.wMonth, tTime.wDay);
	CString strTime;
	strTime.Format("-%.2d-%.2d-%.2d", tTime.wHour, tTime.wMinute, tTime.wSecond);

	//获取路径
	CString strRootDir = ".\\ERROR";
	CString strDir;
	strDir = strRootDir + strDate;
	strDir += strTime;

	//计算截止日期
	SYSTEMTIME tDeadline;
	CalDeadLine(tDeadline, tTime, -lMaxDate);

	//清除截止日期前的数据
	CFileFind finder;
	BOOL bWorking = finder.FindFile(strRootDir + "\\*.*");
	while (bWorking)
	{
		bWorking = finder.FindNextFile();
		if (finder.IsDirectory() && !finder.IsDots())
		{
			//获取文件上次修改日期
			ULARGE_INTEGER fTime1;
			finder.GetCreationTime((FILETIME*)&fTime1);

			//转化日期
			ULARGE_INTEGER fDeadline;
			SystemTimeToFileTime(&tDeadline, (FILETIME*)&fDeadline);

			//清理旧日期文件
			if (fTime1.QuadPart < fDeadline.QuadPart)
			{
				DelFiles(finder.GetFilePath());
				RemoveDirectory(finder.GetFilePath());
			}
		}
	}

	//复制数据
	strDir += strName;
	CopyFolder(".\\Data", strDir);
	CopyFolder(".\\GraphData", strDir);
	CopyFolder(".\\panoRecognition", strDir);
	CopyFolder(".\\ProPics", strDir);
	CString strLogName = GetLogName();
	CopyFile((LPCTSTR)strLogName, (LPCTSTR)(strDir + "\\Log.txt"), FALSE);

}


void SaveGrooveData(int nLayerNo, int nGroupNo, CString sRobotName, bool bSuccess)
{
	//获取限制日期
	long lMaxDate = 100;
	COPini opini;
	opini.SetFileName(DEBUG_INI);
	opini.SetSectionName("Debug");
	opini.ReadString("MaxDateOfSaveErrorData", &lMaxDate);

	//获取时间
	SYSTEMTIME tTime;
	GetLocalTime(&tTime);
	CString strDate;
	strDate.Format("\\%.2d-%.2d", tTime.wMonth, tTime.wDay);
	CString strTime;
	strTime.Format("-%.2d-%.2d-%.2d", tTime.wHour, tTime.wMinute, tTime.wSecond);

	//获取路径
	CString strRootDir = ".\\ERROR";
	CString strDir;
	strDir = strRootDir + strDate;
	strDir += strTime;

	//计算截止日期
	SYSTEMTIME tDeadline;
	CalDeadLine(tDeadline, tTime, -lMaxDate);

	//清除截止日期前的数据
	CFileFind finder;
	BOOL bWorking = finder.FindFile(strRootDir + "\\*.*");
	while (bWorking)
	{
		bWorking = finder.FindNextFile();
		if (finder.IsDirectory() && !finder.IsDots())
		{
			//获取文件上次修改日期
			ULARGE_INTEGER fTime1;
			finder.GetCreationTime((FILETIME*)&fTime1);

			//转化日期
			ULARGE_INTEGER fDeadline;
			SystemTimeToFileTime(&tDeadline, (FILETIME*)&fDeadline);

			//清理旧日期文件
			if (fTime1.QuadPart < fDeadline.QuadPart)
			{
				DelFiles(finder.GetFilePath());
				RemoveDirectory(finder.GetFilePath());
			}
		}
	}

	//复制数据
	CString strName;
	strName.Format("_%s_Layer%d_GroupNo%d_%s", sRobotName, nLayerNo, nGroupNo, bSuccess ? "true" : "false");
	strDir += strName;
	
	CopyFolder(OUTPUT_PATH + sRobotName + "\\Recognition", strDir);
	CopyFolder(OUTPUT_PATH + sRobotName + "\\Groove", strDir);
	CopyFolder(OUTPUT_PATH + sRobotName + "\\Track", strDir);
	CopyFile((LPCTSTR)("Local_Files\\ExtLib\\Vision\\ConfigFiles\\DoubleSlopesGrooveCloudExtPoint.ini"), (LPCTSTR)(strDir + "\\DoubleSlopesGrooveCloudExtPoint.ini"), FALSE);
	CopyFile((LPCTSTR)("Local_Files\\ExtLib\\Vision\\ConfigFiles\\GroovePnts.ini"), (LPCTSTR)(strDir + "\\GroovePnts.ini"), FALSE);
}

CString GetCurTime()
{
	SYSTEMTIME tm;
	GetLocalTime(&tm);
	CString strSystemTime;
	strSystemTime.Format("%04d-%02d-%02d-%02d-%02d-%02d", tm.wYear, tm.wMonth, tm.wDay, tm.wHour, tm.wMinute, tm.wSecond);
	return strSystemTime;
}

//void RunInteractiveWindow()
//{
//	SHELLEXECUTEINFO ShExecInfo = { 0 };
//	ShExecInfo.cbSize = sizeof(SHELLEXECUTEINFO);
//	ShExecInfo.fMask = SEE_MASK_NOCLOSEPROCESS;
//	ShExecInfo.hwnd = NULL;
//	ShExecInfo.lpVerb = NULL;
//	ShExecInfo.lpFile = ".\\XiRobotCAM_Release_x64_231010\\XiRobotMain.exe"; // exe的路径
//	ShExecInfo.lpParameters = "";
//	ShExecInfo.lpDirectory = NULL;
//	ShExecInfo.nShow = SW_SHOW; // 是否显示
//	ShExecInfo.hInstApp = NULL;
//	ShellExecuteEx(&ShExecInfo);
//	// 等待exe执行
//	WaitForSingleObject(ShExecInfo.hProcess, INFINITE);
//}

void RunInteractiveWindow(CString pointFileName, CString saveFileName, CString openFileName)
{
	SHELLEXECUTEINFO ShExecInfo = { 0 };
	ShExecInfo.cbSize = sizeof(SHELLEXECUTEINFO);
	ShExecInfo.fMask = SEE_MASK_NOCLOSEPROCESS;
	ShExecInfo.hwnd = NULL;
	ShExecInfo.lpVerb = NULL;
	//ShExecInfo.lpFile = ".\\XiRobotCAM_1.4.7\\XiRobotCAM\\XiRobotMain.exe"; // exe的路径
	ShExecInfo.lpFile = ".\\XiRobotCAM\\XiRobotMain.exe"; // exe的路径
	CString paramStr;
	paramStr.Format(" --p_wsl_o %s --p_wsl_s %s --p_wsl_p %s", openFileName, saveFileName, pointFileName);
	ShExecInfo.lpParameters = paramStr;
	ShExecInfo.lpDirectory = NULL;
	ShExecInfo.nShow = SW_SHOW; // 是否显示
	ShExecInfo.hInstApp = NULL;
	ShellExecuteEx(&ShExecInfo);
	// 等待exe执行
	WriteLog("RunInteractiveWindow Wait Start");
	WaitForSingleObject(ShExecInfo.hProcess, INFINITE);
	WriteLog("RunInteractiveWindow Wait End");
}

void RunInteractiveWindow(CString sUnitName)
{

	CString pointFileName = ".\\LineScan\\Gantry\\PointCloud\\Scan5D.txt"; //OUTPUT_PATH + sUnitName + "\\" + POINT_CLOUD_FILE;
	CString saveFileName = OUTPUT_PATH + sUnitName + "\\" + IDENTIFY_RESULT_SAVE;
	CString openFileName = OUTPUT_PATH + sUnitName + "\\" + POINT_CLOUD_IDENTIFY_RESULT;
	//CString openFileName = OUTPUT_PATH + sUnitName + "\\" + POINT_CLOUD_IDENTIFY_RESULT;

	if (IDOK == XUI::MesBox::PopOkCancel("是否打开json焊道数据"))
	//if (IDOK == XiMessageBox("是否打开json焊道数据"))
	{
		openFileName = OUTPUT_PATH + sUnitName + "\\" + POINT_CLOUD_IDENTIFY_RESULT_JSON;
		//openFileName = OUTPUT_PATH + sUnitName + "\\" + IDENTIFY_RESULT_SAVE;
	}
	//已修改
	if (IDOK == XUI::MesBox::PopOkCancel("是否打开删减后焊道数据")/* XiMessageBox("是否打开删减后焊道数据")*/)
	//if (IDOK == XiMessageBox("是否打开删减后焊道数据"))
	{
		openFileName = OUTPUT_PATH + sUnitName + "\\" + IDENTIFY_RESULT_SAVE_JSON;
		//openFileName = OUTPUT_PATH + sUnitName + "\\" + IDENTIFY_RESULT_SAVE;
	}
	/*if (1 == LINESCAN_CAMERA_NUM)*/ // 多相机需要 输入多个文件 或 合并文件 !!!!单机多机需优化
	{
		int nCurUseTable = 0;
		COPini opini2;
		opini2.SetFileName(DATA_PATH + sUnitName + LINE_SCAN_PARAM);
		opini2.SetSectionName("CurUseTableNo");
		opini2.ReadString("CurUseTableNo", &nCurUseTable);

		//线扫
		pointFileName = "LineScan\\GantryLeft\\PointCloud\\Scan5D.txt";
		if (nCurUseTable != 0)
			pointFileName = "LineScan\\GantryRight\\PointCloud\\Scan5D.txt";
		//if (true == g_bRemoveCloud)//是否开启去除点云背景
		//{
			//pointFileName = OUTPUT_PATH + sUnitName + "\\Recognition\\PointCloud_0.txt";
		//}
		//else
		//{
			//pointFileName = OUTPUT_PATH + sUnitName + "\\Recognition\\PointCloud_0.txt";
		//}
	}

	SHELLEXECUTEINFO ShExecInfo = { 0 };
	ShExecInfo.cbSize = sizeof(SHELLEXECUTEINFO);
	ShExecInfo.fMask = SEE_MASK_NOCLOSEPROCESS;
	ShExecInfo.hwnd = NULL;
	ShExecInfo.lpVerb = NULL;
	//ShExecInfo.lpFile = ".\\XiRobotCAM_1.4.7\\XiRobotCAM\\XiRobotMain.exe"; // exe的路径
	ShExecInfo.lpFile = ".\\XiRobotCAM\\XiRobotMain.exe"; // exe的路径
	CString paramStr;
	paramStr.Format(" --p_wsl_o %s --p_wsl_s %s --p_wsl_p %s", openFileName, saveFileName, pointFileName);
	ShExecInfo.lpParameters = paramStr;
	ShExecInfo.lpDirectory = NULL;
	ShExecInfo.nShow = SW_SHOW; // 是否显示
	ShExecInfo.hInstApp = NULL;
	ShellExecuteEx(&ShExecInfo);
	// 等待exe执行
	WaitForSingleObject(ShExecInfo.hProcess, INFINITE);

	RemoveStringInFile(saveFileName, "PointCloudIdentifyReault");
	RemoveStringInFile(saveFileName, "null");
}

bool AutomaticGrouping(CString sUnitName)
{
#if 1
	xi::IPathPlanning* p = xi::CreateIPathPlanningObj();
	p->setConfigPath(xi::IPathPlanning::GENERAL, ".\\LocalFiles\\ExLib\\ALGO\\ConfigFiles\\General.ini");
	p->setConfigPath(xi::IPathPlanning::ONE, ".\\LocalFiles\\ExLib\\ALGO\\ConfigFiles\\One.ini");
	//p->setConfigPath(xi::IPathPlanning::FOUR, ".\\LocalFiles\\ExLib\\ALGO\\ConfigFiles\\Two.ini");
	bool bRst = p->plan();
	xi::ReleaseIPathPlanningObj(p);
	return true;
#else
	std::string input_welds_path = OUTPUT_PATH + sUnitName + RECOGNITION_FOLDER + "PointCloudIdentifyReaultAfter.txt";
	std::string planned_welds_path = OUTPUT_PATH + sUnitName + RECOGNITION_FOLDER + "planned_welds.txt";
	std::string planned_image_path = OUTPUT_PATH + sUnitName + RECOGNITION_FOLDER + "planned_welds.bmp";
	std::string sampling_folder = OUTPUT_PATH + sUnitName + RECOGNITION_FOLDER + "Sampling";
	xi::IPathPlanning* p = xi::CreateIPathPlanningObj();
	p->setRobotNum(4);
	p->setRobotOrientation(xi::IPathPlanning::ROBOT_ORIENTATION::PARALLEL);
	p->setInputWeldsPath(input_welds_path.c_str());
	p->setPlannedWeldsPath(planned_welds_path.c_str());
	p->setPlannedImagePath(planned_image_path.c_str());
	p->setSamplingFolder(sampling_folder.c_str());
	p->setXMax(1100.);
	p->setYMax(1100.);
	p->setMinXSingleDivisionGap(500);
	p->setMinYSingleDivisionGap(500);
	p->setMinLongWeldLength(1500);
	p->setCentersDistance(1704.7);
	bool bRst = p->plan();
	xi::ReleaseIPathPlanningObj(p);
	return true;
#endif // 0
}

void GetMemoryInfo(CString sTipInfo)
{
	HANDLE handle = GetCurrentProcess();
	PROCESS_MEMORY_COUNTERS pmc;
	GetProcessMemoryInfo(handle, &pmc, sizeof(pmc));
	WriteLog("%s内存使用：%ldM/%ldM    %ldM/%ldM", sTipInfo,
		pmc.WorkingSetSize / (1024 * 1024), pmc.PeakWorkingSetSize / (1024 * 1024),
		pmc.PagefileUsage / (1024 * 1024), pmc.PeakPagefileUsage / (1024 * 1024));
}
XI_POINT P2P(CvPoint tPtnSrc)
{
	XI_POINT tPtnDst;
	tPtnDst.x = tPtnSrc.x;
	tPtnDst.y = tPtnSrc.y;
	tPtnDst.z = 0.0;
	return tPtnDst;
}

XI_POINT P2P(T_ROBOT_COORS tPtnSrc)
{
	XI_POINT tPtnDst;
	tPtnDst.x = tPtnSrc.dX;
	tPtnDst.y = tPtnSrc.dY;
	tPtnDst.z = tPtnSrc.dZ;
	return tPtnDst;
}

XI_POINT CreateXI_POINT(double x, double y, double z)
{
	XI_POINT P;
	P.x = x;
	P.y = y;
	P.z = z;
	return P;
}
double GetDouble(char** str, int maxLen)
{
	char* reStr = new char[maxLen];
	memset(reStr, '\0', maxLen);
	double reFloat = 0;
	int i = 0;
	int reStrIndex = 0;
	char nextC;
	for (; i < maxLen; i++) {
		if (**str == '-' || **str == '.' || (**str >= '0' && **str <= '9')) {
			reStr[reStrIndex++] = **str;
			nextC = *(*str + 1);
			if (!(nextC == '-' || nextC == '.' || (nextC >= '0' && nextC <= '9'))) { //检测到数字后面的空格，退出循环
				(*str)++;
				break;
			}
		}
		(*str)++;
	}

	return atof(reStr);
}
void RobotAxisToAlgoAxis(double& x, double& y, double& z)
{
#ifdef SINGLE_ROBOT
	return;
#endif
	double tempx = x, tempy = y;
	y = tempx;
	x = -tempy;
}

void AlgoAxisToRobotAxis(double& x, double& y, double& z)
{
#ifdef SINGLE_ROBOT
	return;
#endif
	double tempx = x, tempy = y;
	y = -tempx;
	x = tempy;
}

void AlgoAxisToRobotAxis(long& x, long& y, long& z)
{
#ifdef SINGLE_ROBOT
	return;
#endif
	long tempx = x, tempy = y;
	y = -tempx;
	x = tempy;
}
//返回值:在同一坐标系下从原始位置到目标位置的坐标变换矩阵
//BasePoints:原始位置四个对应点在坐标系下的坐标
//TargetPoints:目标位置四个对应点在坐标系下的坐标
/*若四个点共面,则矩阵BaseCoordinateMatrix不满秩,其逆矩阵不存在,在计算BaseCoordinateMatrix_1时将得到错误的结果*/
cv::Mat CalculateTransformationMatrix(std::vector<CvPoint3D64f>& BasePoints, std::vector<CvPoint3D64f>& TargetPoints)
{
	if ((BasePoints.size() != 4) || (TargetPoints.size() != 4))
		return cv::Mat();
	cv::Mat BaseCoordinateMatrix = cv::Mat::zeros(4, 4, CV_64F);
	for (int i = 0; i < 4; i++)
	{
		BaseCoordinateMatrix.ptr<double>(0)[i] = BasePoints[i].x;
		BaseCoordinateMatrix.ptr<double>(1)[i] = BasePoints[i].y;
		BaseCoordinateMatrix.ptr<double>(2)[i] = BasePoints[i].z;
		BaseCoordinateMatrix.ptr<double>(3)[i] = 1.0;
	}
	cv::Mat TargetCoordinateMatrix = cv::Mat::zeros(4, 4, CV_64F);
	for (int i = 0; i < 4; i++)
	{
		TargetCoordinateMatrix.ptr<double>(0)[i] = TargetPoints[i].x;
		TargetCoordinateMatrix.ptr<double>(1)[i] = TargetPoints[i].y;
		TargetCoordinateMatrix.ptr<double>(2)[i] = TargetPoints[i].z;
		TargetCoordinateMatrix.ptr<double>(3)[i] = 1.0;
	}
	cv::Mat BaseCoordinateMatrix_1;
	cv::invert(BaseCoordinateMatrix, BaseCoordinateMatrix_1, cv::DECOMP_LU);
	cv::Mat TransformationMatrix = TargetCoordinateMatrix * BaseCoordinateMatrix_1;
	return TransformationMatrix;
}

//返回值:原始位置点变换到目标位置后的结果点在同一坐标系下的坐标
//BasePoints:原始位置点坐标
//TransformationMatrix:坐标变换矩阵
std::vector<CvPoint3D64f> CoordinateTransformate(std::vector<CvPoint3D64f>& BasePoints, cv::Mat TransformationMatrix)
{
	if ((TransformationMatrix.rows != 4) || (TransformationMatrix.cols != 4) || (TransformationMatrix.type() != CV_64F))
		return std::vector<CvPoint3D64f>();
	cv::Mat BaseCoordinateMatrix = cv::Mat::zeros(4, int(BasePoints.size()), CV_64F);
	for (size_t i = 0; i < BasePoints.size(); i++)
	{
		BaseCoordinateMatrix.ptr<double>(0)[i] = BasePoints[i].x;
		BaseCoordinateMatrix.ptr<double>(1)[i] = BasePoints[i].y;
		BaseCoordinateMatrix.ptr<double>(2)[i] = BasePoints[i].z;
		BaseCoordinateMatrix.ptr<double>(3)[i] = 1.0;
	}
	cv::Mat TargetCoordinateMatrix = TransformationMatrix * BaseCoordinateMatrix;
	std::vector<CvPoint3D64f> TargetPoints(BasePoints.size());
	for (size_t i = 0; i < TargetPoints.size(); i++)
	{
		TargetPoints[i].x = TargetCoordinateMatrix.ptr<double>(0)[i];
		TargetPoints[i].y = TargetCoordinateMatrix.ptr<double>(1)[i];
		TargetPoints[i].z = TargetCoordinateMatrix.ptr<double>(2)[i];
	}
	return TargetPoints;
}
std::vector<CvPoint3D64f> CoordinateTransformateNormal(std::vector<CvPoint3D64f>& BasePoints, cv::Mat TransformationMatrix)
{
	std::vector<CvPoint3D64f> TargetPoints = CoordinateTransformate(BasePoints, TransformationMatrix);
	for (size_t i = 0; i < TargetPoints.size(); i++)
	{
		TargetPoints[i].x -= TransformationMatrix.ptr<double>(0)[3];
		TargetPoints[i].y -= TransformationMatrix.ptr<double>(1)[3];
		TargetPoints[i].z -= TransformationMatrix.ptr<double>(2)[3];
	}
	return TargetPoints;
}

bool CalcLineParamRansac(T_LINE_PARA& tBestLineParam, std::vector<XI_POINT> vtPoint, double dSelectedRatio)
{
	T_SPACE_LINE_DIR tLineDir;
	if (vtPoint.size() < 2)
	{
		//已修改
		XUI::MesBox::PopInfo("CalcLineParamRansac输入数据有误");
		//XiMessageBox("CalcLineParamRansac输入数据有误");
		return false;
	}
	double dMinError = 9999999.0;

	int nIdx = 0;
	int sIdx = 0;
	int tIdx = 0;
	int rIdx = 0;

	double dDist = 0.0;
	std::vector<double> vdDist;

	for (nIdx = 0; nIdx < vtPoint.size() - 1; nIdx++)
	{
		for (sIdx = nIdx + 1; sIdx < vtPoint.size(); sIdx++)
		{
			double dError = 0.0;
			vdDist.clear();
			for (rIdx = 0; rIdx < vtPoint.size(); rIdx++)
			{
				dDist = CalDisPointToLine(vtPoint[rIdx], vtPoint[nIdx], vtPoint[sIdx]);
				vdDist.push_back(dDist);
			}
			std::sort(vdDist.begin(), vdDist.end(), dCmpIncFunc);

			for (rIdx = 0; rIdx < vtPoint.size() * dSelectedRatio; rIdx++)
			{
				dError += vdDist[rIdx];
			}

			if (dError < dMinError)
			{
				dMinError = dError;
				tBestLineParam.dPointX = vtPoint[nIdx].x;
				tBestLineParam.dPointY = vtPoint[nIdx].y;
				tBestLineParam.dPointZ = vtPoint[nIdx].z;

				tLineDir = CalLineDir(vtPoint[nIdx], vtPoint[sIdx]);
				tBestLineParam.dDirX = tLineDir.dDirX;
				tBestLineParam.dDirY = tLineDir.dDirY;
				tBestLineParam.dDirZ = tLineDir.dDirZ;
				tBestLineParam.dLineError = dMinError;
			}
		}
	}

	return true;
}

bool dCmpIncFunc(double dFirstValue, double dSecondValue)
{
	return(dFirstValue < dSecondValue);
}

T_SPACE_LINE_DIR CalLineDir(XI_POINT tStartPoint, XI_POINT tEndPoint)
{
	double dLineLength = 0.0;
	T_SPACE_LINE_DIR tLineDir;
	double dX = 0.0, dY = 0.0, dZ = 0.0;

	dX = tEndPoint.x - tStartPoint.x;
	dY = tEndPoint.y - tStartPoint.y;
	dZ = tEndPoint.z - tStartPoint.z;

	dLineLength = sqrt(SQUARE(dX) + SQUARE(dY) + SQUARE(dZ));

	if (fabs(dLineLength) < 0.0001)
	{
		tLineDir.dDirX = 0.0;
		tLineDir.dDirY = 0.0;
		tLineDir.dDirZ = 0.0;
	}
	else
	{
		tLineDir.dDirX = dX / dLineLength;
		tLineDir.dDirY = dY / dLineLength;
		tLineDir.dDirZ = dZ / dLineLength;
	}

	return tLineDir;
}

double CalDisPointToLine(XI_POINT tPoint, XI_POINT tSegmentStartPoint, XI_POINT tSegmentEndPoint)
{
	double p, q, r;
	double m, n;
	double dDis = 0.0;

	p = tSegmentEndPoint.x - tSegmentStartPoint.x;
	q = tSegmentEndPoint.y - tSegmentStartPoint.y;
	r = tSegmentEndPoint.z - tSegmentStartPoint.z;

	m = sqrt((q * (tPoint.z - tSegmentStartPoint.z) - r * (tPoint.y - tSegmentStartPoint.y)) * (q * (tPoint.z - tSegmentStartPoint.z) - r * (tPoint.y - tSegmentStartPoint.y))
		+ (r * (tPoint.x - tSegmentStartPoint.x) - p * (tPoint.z - tSegmentStartPoint.z)) * (r * (tPoint.x - tSegmentStartPoint.x) - p * (tPoint.z - tSegmentStartPoint.z))
		+ (p * (tPoint.y - tSegmentStartPoint.y) - q * (tPoint.x - tSegmentStartPoint.x)) * (p * (tPoint.y - tSegmentStartPoint.y) - q * (tPoint.x - tSegmentStartPoint.x)));
	n = sqrt(p * p + q * q + r * r);

	if (n > 0)
	{
		dDis = m / n;
	}
	else
	{
		dDis = sqrt(SQUARE(tPoint.x - tSegmentStartPoint.x) + SQUARE(tPoint.y - tSegmentStartPoint.y) + SQUARE(tPoint.z - tSegmentStartPoint.z));
	}

	return dDis;
}
//(P-Q) · (B-A) = 0 以及 P = Q + t(B-A)解方程求出t
bool PointtoLineProjection(T_LINE_PARA tPointDir, T_ALGORITHM_POINT tpoint, T_ALGORITHM_POINT& tpointProjection)
{
	// 计算从外点到直线上任意一点的向量
	XI_POINT tOutPointDir = { tpoint.dCoorX - tPointDir.dPointX,tpoint.dCoorY - tPointDir.dPointY,tpoint.dCoorZ - tPointDir.dPointZ };
	XI_POINT tDir = { tPointDir.dDirX,tPointDir.dDirY,tPointDir.dDirZ };
	// 求解投影点的 t 值
	double denom = dot_product(tDir, tDir);
	if (denom == 0) {
		XiMessageBox("输入点数据相同，无法构成直线");
		return false;
	}
	double t = dot_product(tOutPointDir, tDir) / denom;

	// 计算投影点的坐标
	tpointProjection.dCoorX = tPointDir.dPointX + t * tDir.x;
	tpointProjection.dCoorY = tPointDir.dPointY + t * tDir.y;
	tpointProjection.dCoorZ = tPointDir.dPointZ + t * tDir.z;
	return true;
}
bool PointtoLineProjection(T_LINE_PARA tPointDir, XI_POINT tpoint, XI_POINT& tpointProjection)
{
	T_ALGORITHM_POINT tAlgorProj;
	T_ALGORITHM_POINT tAlgorpoint = { tpoint.x,tpoint.y,tpoint.z };
	if (!PointtoLineProjection(tPointDir, tAlgorpoint, tAlgorProj))
	{
		return false;
	}
	tpointProjection.x = tAlgorProj.dCoorX;
	tpointProjection.y = tAlgorProj.dCoorY;
	tpointProjection.z = tAlgorProj.dCoorZ;
	return true;
}
double dot_product(XI_POINT V1, XI_POINT V2) {
	return V1.x * V2.x + V1.y * V2.y + V1.z * V2.z;
}

UINT ThreadSaveImage(void* pParam)
{
	T_SAVE_IMAGE* pSaveStaus = (T_SAVE_IMAGE*)pParam;
	int nNum = SAVE_IMAGE_THREAD_MUN;
	for (int i = 0; i < nNum; i++)
	{
		if (TRUE != pSaveStaus->bSaveImageStaus[i])
		{
			break;
		}
		if (i == nNum - 1)
		{
			WriteLog("%s 存图失败", pSaveStaus->cstrId);
			return false;
		}
	}
	for (int i = 0; i < nNum; i++)
	{
		if (TRUE != pSaveStaus->bSaveImageStaus[i])
		{
			pSaveStaus->bSaveImageStaus[i] = true;
			if (NULL != pSaveStaus->pImage)
			{
				int nParamsArray[3];
				nParamsArray[0] = CV_IMWRITE_JPEG_QUALITY;
				nParamsArray[1] = (int)(0.3 * 100); // 压缩比例
				nParamsArray[2] = 0;
				cvSaveImage((LPCTSTR)pSaveStaus->cstrId, pSaveStaus->pImage, nParamsArray);
			}
			pSaveStaus->bSaveImageStaus[i] = false;
			return true;
		}
	}
	return true;
}

void SaveRobotCoor(FILE* RecordTheoryData, T_ROBOT_COORS tCoor)
{
	/*if (RecordTheoryData == NULL)
	{
		XiMessageBox("数据流为空指针");
		return;
	}*/
	fprintf(RecordTheoryData, "%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf\n",
		tCoor.dX, tCoor.dY, tCoor.dZ, tCoor.dRX,
		tCoor.dRY, tCoor.dRZ, tCoor.dBX, tCoor.dBY, tCoor.dBZ);
	fflush(RecordTheoryData);
}

void SaveRobotCoor(FILE* RecordTheoryData, T_ROBOT_COORS tCoor, int nTrackType)
{
	/*fprintf(RecordTheoryData, "%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %4d\n",
		tCoor.dX, tCoor.dY, tCoor.dZ, tCoor.dRX,
		tCoor.dRY, tCoor.dRZ, tCoor.dBX, tCoor.dBY, tCoor.dBZ, nTrackType);*/
	fprintf(RecordTheoryData, "%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %4d\n",
		tCoor.dX + tCoor.dBX, tCoor.dY + tCoor.dBY, tCoor.dZ, tCoor.dRX,
		tCoor.dRY, tCoor.dRZ, tCoor.dBX, tCoor.dBY, tCoor.dBZ, nTrackType);
	fflush(RecordTheoryData);
}

void SaveRobotCoor(FILE* RecordTheoryData, std::vector<T_ROBOT_COORS> vtCoors)
{
	T_ROBOT_COORS tCoor;
	for (size_t i = 0; i < vtCoors.size(); i++)
	{
		tCoor = vtCoors.at(i);
		/*fprintf(RecordTheoryData, "%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf\n",
			tCoor.dX, tCoor.dY, tCoor.dZ, tCoor.dRX,
			tCoor.dRY, tCoor.dRZ, tCoor.dBX, tCoor.dBY, tCoor.dBZ);*/
		fprintf(RecordTheoryData, "%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf\n",
			tCoor.dX + tCoor.dBX, tCoor.dY + tCoor.dBY, tCoor.dZ, tCoor.dRX,
			tCoor.dRY, tCoor.dRZ, tCoor.dBX, tCoor.dBY, tCoor.dBZ);
		fflush(RecordTheoryData);
	}
}

void SaveRobotCoor(FILE* RecordTheoryData, std::vector<T_ROBOT_COORS> vtCoors, std::vector<int> vtCoorType)
{
	T_ROBOT_COORS tCoor;
	for (size_t i = 0; i < vtCoors.size(); i++)
	{
		tCoor = vtCoors.at(i);
		/*fprintf(RecordTheoryData, "%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %4d\n",
			tCoor.dX, tCoor.dY, tCoor.dZ, tCoor.dRX,
			tCoor.dRY, tCoor.dRZ, tCoor.dBX, tCoor.dBY, tCoor.dBZ, vtCoorType.at(i));*/

		fprintf(RecordTheoryData, "%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %4d\n",
			tCoor.dX + tCoor.dBX, tCoor.dY + tCoor.dBY, tCoor.dZ, tCoor.dRX, 
			tCoor.dRY, tCoor.dRZ, tCoor.dBX, tCoor.dBY, tCoor.dBZ, vtCoorType.at(i));

		fflush(RecordTheoryData);
	}
}

void FlipImage(IplImage* pImg, E_FLIP_MODE eFlipMode)
{
	switch (eFlipMode)
	{
	case E_FLIP_NONE:		break;
	case E_FLIP_VERTICAL:	cvFlip(pImg, pImg, 1);break;
	case E_FLIP_HORIZEN:	cvFlip(pImg, pImg, 0); break;
	case E_FLIP_BOTH:		cvFlip(pImg, pImg, -1); break;
	default:				break;
	}
}

void ResumeImagePoint2D(int nImgWidth, int nImgHeight, CvPoint& tPtn2D, E_FLIP_MODE eFlipMode)
{
	int nCenterX = nImgWidth / 2;
	int nCenterY = nImgHeight / 2;
	switch (eFlipMode)
	{
	case E_FLIP_NONE: break;
	case E_FLIP_VERTICAL:
		tPtn2D.x = nCenterX - (tPtn2D.x - nCenterX);
		break;
	case E_FLIP_HORIZEN:
		tPtn2D.y = nCenterY - (tPtn2D.y - nCenterY);
		break;
	case E_FLIP_BOTH:
		tPtn2D.x = nCenterX - (tPtn2D.x - nCenterX);
		tPtn2D.y = nCenterY - (tPtn2D.y - nCenterY);
		break;
	default: break;
	}
}

void SavePointsData(vector<T_ALGORITHM_POINT> PointsData, string str)
{
	if (PointsData.size() < 1)
	{
		XUI::MesBox::PopError("SaveData函数存储数据有误：{0}", PointsData.size());
		return;
	}
	ofstream CoutData(str.c_str());
	CoutData << PointsData[0].dCoorX << "	" << PointsData[0].dCoorY << "	" << PointsData[0].dCoorZ;
	for (int nred = 0; nred < PointsData.size(); nred++)
	{
		CoutData << endl << PointsData[nred].dCoorX << "	" << PointsData[nred].dCoorY << "	" << PointsData[nred].dCoorZ;
	}
	CoutData.close();
}

void SavePointsData(vector<XI_POINT> PointsData, CString str)
{
	if (PointsData.size() < 1)
	{
		XUI::MesBox::PopError("SaveData函数存储数据有误：{0}", PointsData.size());
		return;
	}
	ofstream CoutData(str.GetBuffer());
	CoutData << PointsData[0].x << "	" << PointsData[0].y << "	" << PointsData[0].z;
	for (int nred = 1; nred < PointsData.size(); nred++)
	{
		CoutData << endl << PointsData[nred].x << "	" << PointsData[nred].y << "	" << PointsData[nred].z;
	}
	CoutData.close();
}

bool LoadPointsData(vector<XI_POINT>& PointsData, CString str)
{
	if (!CheckFileExists(str.GetBuffer()))
	{
		XUI::MesBox::PopOkCancel("{0} 文件不存在", str.GetBuffer());
		return false;
	}
	PointsData.clear();
	XI_POINT tPoint;
	ifstream CintData(str.GetBuffer());
	while (!CintData.eof())
	{
		CintData >> tPoint.x >> tPoint.y >> tPoint.z;
		PointsData.push_back(tPoint);
	}
	CintData.close();
	if (PointsData.size() < 1)
	{
		return false;
}
	return true;
}

bool LoadPointsData(vector<T_ALGORITHM_POINT>& PointsData, CString str)
{
	if (!CheckFileExists(str.GetBuffer()))
	{
		XUI::MesBox::PopOkCancel("{0} 文件不存在", str);
		return false;
	}
	PointsData.clear();
	T_ALGORITHM_POINT tPoint;
	ifstream CintData(str.GetBuffer());
	while (!CintData.eof())
	{
		CintData >> tPoint.dCoorX >> tPoint.dCoorY >> tPoint.dCoorZ;
		PointsData.push_back(tPoint);
	}
	CintData.close();
	if (PointsData.size() < 1)
	{
		return false;
	}
	return true;
}

bool OpenProductLog(FILE** RecordTheoryData, CString sProductDataFileName)
{
	//CString sProductDataFileName;
	//CTime time = CTime::GetCurrentTime();
	//sProductDataFileName.Format("Monitor\\ProductData_%4d.%.2d.%.2d.csv", time.GetYear(), time.GetMonth(), time.GetDay());
	int nFileLineNum = GetFileLineNum(sProductDataFileName);
	*RecordTheoryData = fopen(sProductDataFileName, "a+");
	if (NULL == *RecordTheoryData)
	{
		XiMessageBox("文件打开失败，检查是否没有路径或文件已被手动打开");
		return false;
	}
	if (nFileLineNum == 0)
	{		
		fprintf(*RecordTheoryData, "焊接生产数据记录\n\n");
		fprintf(*RecordTheoryData, "工件编号,记录时间,焊接长度(mm),焊接时长\n");
	}
	return true;
}
bool CloseProductLog(FILE* RecordTheoryData)
{
	if (NULL == RecordTheoryData)
	{
		return true;
	}

	fclose(RecordTheoryData);
	RecordTheoryData = NULL;
	return true;
}
bool SaveProductData(FILE* RecordTheoryData, double dWeldLength, int dWeldTime, int nWorkpieceNo)
{
	CTime cTime;
	cTime = CTime::GetCurrentTime();

	int nCurrentHour = cTime.GetHour();
	int nCurrentMinute = cTime.GetMinute();
	int nCurrentSecond = cTime.GetSecond();

	int nHour = 0;
	int nMinute = 0;
	int nSecond = 0;
	CString strRecordTime;

	nHour = dWeldTime / 3600;
	nSecond = dWeldTime % 3600;
	nMinute = nSecond / 60;//取整
	nSecond = nSecond % 60;//取余z

	strRecordTime.Format("%d-%d-%d %d:%d:%d", cTime.GetYear(), cTime.GetMonth(), cTime.GetDay(), nCurrentHour, nCurrentMinute, nCurrentSecond);

	fprintf(RecordTheoryData, "%2d,%2d:%2d:%2d,%.3lf,%2d:%2d:%2d\n", nWorkpieceNo, nCurrentHour, nCurrentMinute, nCurrentSecond, dWeldLength, nHour, nMinute, nSecond);
	return true;
}

/**************************** 线程相关操作 Start ****************************/
bool WaitAndCheckThreadExit(CWinThread* pWinThread, CString sThreadName) // 阻塞等待线程退出 线程函数返回0成功 其他失败
{
	DWORD nExitCode = 1;
	while (0 != nExitCode)
	{
		BOOL bGetResult = GetExitCodeThread(pWinThread->m_hThread, &nExitCode);
		if ((FALSE == bGetResult) ||
			((RUNNING_STATUS_SUCCESS != nExitCode) && (STILL_ACTIVE != nExitCode)))
		{
			WriteLog("%s 状态异常 GetResult=%d nExitCode=%ld", sThreadName, bGetResult, nExitCode);
			DELETE_POINTER(pWinThread);
			return false;
		}
		Sleep(50);
		DoEvent();
	}
	DELETE_POINTER(pWinThread);
	return (0 == nExitCode) ? true : false;
}

bool WaitAndCheckAllThreadExit(std::vector<CWinThread*>& vpWinThread)
{
	DWORD nCode = 1;
	DWORD nExitCode;
	while (0 != nCode)
	{
		nCode = 0;
		for (int i = 0; i < vpWinThread.size(); i++)
		{
			BOOL bGetResult = GetExitCodeThread(vpWinThread[i]->m_hThread, &nExitCode);
			if ((FALSE == bGetResult) ||
				((0 != nExitCode) && (STILL_ACTIVE != nExitCode)))
			{
				WriteLog("WaitAndCheckAllThreadExit GetResult=%d nExitCode=%ld", bGetResult, nExitCode);
				ClearWinThread(vpWinThread);
				return false;
			}
			nCode += nExitCode;
		}
		Sleep(50);
		DoEvent();
	}
	ClearWinThread(vpWinThread);
	return (0 == nCode) ? true : false;
}

void ClearWinThread(std::vector<CWinThread*>& vpWinThread)
{
	for (int i = 0; i < vpWinThread.size(); i++)
	{
		DELETE_POINTER(vpWinThread[i]);
	}
	vpWinThread.clear();
}

CDialog* g_pMainDlg = nullptr;
void setMainDlg(CDialog* pMainDlg)
{
	g_pMainDlg = pMainDlg;
}

CDialog* getMainDlg()
{
	return g_pMainDlg;
}

std::vector<CString> findSubfolder(CString sFolder)
{
	XiBase::GetSystemPath(sFolder);
	XiBase::AddSlash(sFolder);

	std::vector<CString> vsFileList;
	vsFileList.clear();
	CFileFind finder;
	sFolder += "*.*";
	BOOL bWorking = finder.FindFile(sFolder);
	while (bWorking)
	{
		bWorking = finder.FindNextFile();
		if (finder.IsDirectory() && !finder.IsDots())
		{
			vsFileList.push_back(finder.GetFilePath());
		}
	}
	return vsFileList;
}


bool Matching(E_WORKPIECE_TYPE eWorkPieceType)
{
	CString WeldingLinesSaveRoute;
	bool ifupright;
	CString RealCADTemplateRoute;
	bool if_Add_ExtraWeld;
	bool if_Add_TemplateName = false;
	bool if_Fine_Match = false;

	//读取配置文件参数
	{
		try {
			COPini opini;
			//switch (eWorkPieceType)
			{
			//case E_MODEL:
				// 自动识别匹配，无法处理正确相似件，不可处理圆弧工件，不需要用到模型点云，可处理单板单筋
				opini.SetFileName(".\\TemplateMatching\\config\\WidgetRegistrationDetection.ini");
			//	break;
				//case E_MODEL_ASSIGNMENT:
				//	// 指定识别匹配，可处理正确相似件，不可处理圆弧工件，不需要用到模型点云，可处理单板单筋
				//	opini.SetFileName(".\\TemplateMatching\\config\\RegistrationDivisionRegionCalculatingTemplateTransformMatrixToRealWorkpiece.ini");
				//	break;
				//case E_MODEL_CLOUD:
				//	// 自动识别匹配，无法处理正确相似件，可处理圆弧工件，需要用到模型点云，可处理单板单筋
				//	opini.SetFileName(".\\TemplateMatching\\config\\RegistrationAdaptiveAcquireModelTransformToRealWorkpieceMatrix.ini");
				//	break;
				//case E_MODEL_CLOUD_ASSIGNMENT:
				//	// 指定识别匹配，无法处理正确相似件，可处理圆弧工件，需要用到模型点云，可处理单板单筋
				//	opini.SetFileName(".\\TemplateMatching\\config\\RegistrationDisignationAdaptiveAcquireModelTransformToRealWorkpieceMatrix.ini");
				//	break;
			//default:
			//	break;
			}

			opini.SetSectionName("PARAM");
			opini.ReadString("WeldingLinesSaveRoute", WeldingLinesSaveRoute);
			ifupright = false;//opini.ReadString("ifupright", &ifupright);
			opini.ReadString("RealCADTemplateRoute", RealCADTemplateRoute);
			if_Add_ExtraWeld = false; //opini.ReadString("if_Add_ExtraWeld", &if_Add_ExtraWeld);
			opini.ReadString("if_Add_TemplateName", &if_Add_TemplateName);
			opini.ReadString("if_Fine_Match", &if_Fine_Match);

		}
		catch (std::string str) {
			std::cout << str << std::endl;

			std::cout << "按回车键继续" << std::endl;
			getchar();
			return false;
		}

		std::cout << "读取配置文件结束" << std::endl;
	}
	auto start = std::chrono::high_resolution_clock::now();

	//匹配
	TemplateTransformMatrixWithModelFileRoute* Result = nullptr;
	int target_result_number = 0;

	//switch (eWorkPieceType)
	{
	//case E_MODEL:
		// 自动识别匹配，无法处理正确相似件，不可处理圆弧工件，不需要用到模型点云，可处理单板单筋
		Result = WidgetRegistrationDetection(&target_result_number, ".\\TemplateMatching\\config");
		//break;
		//case E_MODEL_ASSIGNMENT:
		//	// 指定识别匹配，可处理正确相似件，不可处理圆弧工件，不需要用到模型点云，可处理单板单筋
		//	Result = RegistrationDivisionRegionCalculatingTemplateTransformMatrixToRealWorkpiece(&target_result_number, ".\\TemplateMatching\\config");
		//	break;
		//case E_MODEL_CLOUD:
		//	// 自动识别匹配，无法处理正确相似件，可处理圆弧工件，需要用到模型点云，可处理单板单筋
		//	Result = RegistrationAdaptiveAcquireModelTransformToRealWorkpieceMatrix(&target_result_number, ".\\TemplateMatching\\config");
		//	break;
		//case E_MODEL_CLOUD_ASSIGNMENT:
		//	// 指定识别匹配，可处理正确相似件，可处理圆弧工件，需要用到模型点云，可处理单板单筋
		//	Result = RegistrationDisignationAdaptiveAcquireModelTransformToRealWorkpieceMatrix(&target_result_number, ".\\TemplateMatching\\config");
		//	break;
	//default:
	//	break;
	}

	if (Result == nullptr) {
		return false;
	}
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = end - start;

	std::cout << "粗定位用时: " << elapsed.count() << std::endl;

	for (size_t k = 0; k < target_result_number; k++)
	{
		double dCoarseMatrix[4][4];
		double dFineMatrix[4][4];

		// 原始矩阵赋值
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				dCoarseMatrix[i][j] = Result[k].Transform_Matrix[i][j];
			}
		}

		std::cout << "粗匹配矩阵：" << std::endl
			<< dCoarseMatrix[0][0] << "\t" << dCoarseMatrix[0][1] << "\t" << dCoarseMatrix[0][2] << "\t" << dCoarseMatrix[0][3] << std::endl
			<< dCoarseMatrix[1][0] << "\t" << dCoarseMatrix[1][1] << "\t" << dCoarseMatrix[1][2] << "\t" << dCoarseMatrix[1][3] << std::endl
			<< dCoarseMatrix[2][0] << "\t" << dCoarseMatrix[2][1] << "\t" << dCoarseMatrix[2][2] << "\t" << dCoarseMatrix[2][3] << std::endl
			<< dCoarseMatrix[3][0] << "\t" << dCoarseMatrix[3][1] << "\t" << dCoarseMatrix[3][2] << "\t" << dCoarseMatrix[3][3] << std::endl;


		if (!ModelMatching(dCoarseMatrix, Result[k].ModelFileRoute, Result[k].WorkpieceNumber, "ExtractWeldingLinePoints.txt", dFineMatrix, Result[k].if_Fine_Matching)) {
			return false;
		}

		std::cout << "精匹配矩阵：" << std::endl
			<< dFineMatrix[0][0] << "\t" << dFineMatrix[0][1] << "\t" << dFineMatrix[0][2] << "\t" << dFineMatrix[0][3] << std::endl
			<< dFineMatrix[1][0] << "\t" << dFineMatrix[1][1] << "\t" << dFineMatrix[1][2] << "\t" << dFineMatrix[1][3] << std::endl
			<< dFineMatrix[2][0] << "\t" << dFineMatrix[2][1] << "\t" << dFineMatrix[2][2] << "\t" << dFineMatrix[2][3] << std::endl
			<< dFineMatrix[3][0] << "\t" << dFineMatrix[3][1] << "\t" << dFineMatrix[3][2] << "\t" << dFineMatrix[3][3] << std::endl;

		if (!if_Fine_Match)
		{
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					dFineMatrix[i][j] = dCoarseMatrix[i][j];
				}
			}
		}

		// 精确矩阵赋值
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				Result[k].Transform_Matrix[i][j] = dFineMatrix[i][j];
			}
		}
	}
	//std::cout << "3" << std::endl;
	//std::cout << "按回车键继续" << std::endl;
	//getchar();

	std::string filename = "./TemplateMatching/RealTemplate/matrix.txt";
	std::ofstream outfile(filename, std::ios::trunc);

	for (int k = 0; k < target_result_number; k++) {
		outfile << Result[k].ModelFileRoute << std::endl;
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				outfile << Result[k].Transform_Matrix[i][j] << " ";
			}
			outfile << std::endl;
		}
	}
	outfile.close();

	TransformCorrespondingTemplatesToRealWorkpiece(RealCADTemplateRoute.GetBuffer(), Result, target_result_number, ifupright, WeldingLinesSaveRoute.GetBuffer(), if_Add_ExtraWeld, if_Add_TemplateName);

	ReleaseDingHengTemplateTransformMatrixInfomation(&Result, target_result_number);

	//auto end1 = std::chrono::high_resolution_clock::now();
	//std::chrono::duration<double> elapsed1 = end1 - end;

	//std::cout << "精定位用时: " << elapsed1.count() << std::endl;

	auto end1 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed1 = end1 - start;

	std::cout << "总用时: " << elapsed1.count() << std::endl;
	return true;

}

bool ModelMatching(double dCorasrMatrix[4][4], const char* ModelFileName, int nWeldPartNo, std::string SaveRoute_weldline_point, double dFineMatrix[4][4], bool if_Fine_Match)
{
	if (if_Fine_Match)
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				dFineMatrix[i][j] = dCorasrMatrix[i][j];
			}
		}
		std::cout << std::endl << "已覆盖" << std::endl;
		return true;
	}

	if (dCorasrMatrix[0][0] == 1
		&& dCorasrMatrix[0][1] == 0
		&& dCorasrMatrix[0][2] == 0
		&& dCorasrMatrix[0][3] == 0
		&& dCorasrMatrix[1][0] == 0
		&& dCorasrMatrix[1][1] == 1
		&& dCorasrMatrix[1][2] == 0
		&& dCorasrMatrix[1][3] == 0
		&& dCorasrMatrix[2][0] == 0
		&& dCorasrMatrix[2][1] == 0
		&& dCorasrMatrix[2][2] == 1
		&& dCorasrMatrix[2][3] == 0
		)
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				dFineMatrix[i][j] = dCorasrMatrix[i][j];
			}
		}
		std::cout << std::endl << "粗匹配错误，跳过精匹配" << std::endl;
		return true;
	}

	XiAlgorithm* m_XiAlgorithm;
	m_XiAlgorithm = new XiAlgorithm;

	T_ALGORITHM_LINESEGMENT tLineSegment;
	std::vector<T_ALGORITHM_LINESEGMENT> vtAllIdealLineSegments;
	std::vector < std::vector<T_ALGORITHM_LINESEGMENT>> vvtAllIdealLineSegments;
	std::vector<T_ALGORITHM_LINESEGMENT> vtAllTransIdealLineSegments;
	std::vector<T_ALGORITHM_LINESEGMENT> vtIdealLineSegments;
	std::vector <std::vector<T_ALGORITHM_LINESEGMENT>> vvtIdealLineSegments;
	std::vector<T_ALGORITHM_LINESEGMENT> vtRealLineSegments;
	std::vector<VTLINESEGMENT> vvtRealLineSegments;
	std::vector<T_ALGORITHM_POINT> vtRealLineSegmentPoints;
	std::vector<VTPOINT> vvtRealLineSegmentPoints;
	std::vector<VVTPOINT> vvvtRealLineSegmentPoints;

	int nPlateNo = 0;
	int nWeldLineNo = 0;
	int nIsArc = 0;
	int nClockWise = 0;
	double dZSide = 0.0;
	int nIsLeft = 0;
	T_ALGORITHM_POINT tCenterPoint;
	T_ALGORITHM_POINT tStartPoint, tEndPoint;
	T_ALGORITHM_LINE_DIR tStartVerLineDir, tEndVerLineDir;
	int nStartPointType, nEndPointType;
	int nIsDoubleWelding;
	int nStartCornerType;
	int nEndCornerType;
	double nWeldSize;
	double dStartHoleRadiusSize;
	double dEndHoleRadiusSize;
	int nRobotSelect;
	double dTheoryLength;
	double dAngle;

	vtIdealLineSegments.clear();
	FILE* pfInput;/* = fopen(ModelFileName, "r");*///////算法提取理想模型输入  需要在最前端增加一列(富强会增加这个功能) 焊道属于那个立板 nPlateNo

	std::ifstream inputFile(ModelFileName); // 打开要读取的文件
	if (!inputFile.is_open()) return false;

	// 逐行读取文件内容
	std::string line;
	while (std::getline(inputFile, line))
	{
		std::vector<std::string> tokens; // 用于存储分隔后的子字符串

		std::istringstream iss(line);
		std::string token;

		while (iss >> token) // 空格分隔
		{
			tokens.push_back(token);
		}

		nPlateNo = std::stod(tokens[0]);
		nIsArc = std::stod(tokens[1]);
		nClockWise = std::stod(tokens[2]);
		dZSide = std::stod(tokens[3]);
		nIsLeft = std::stod(tokens[4]);
		tCenterPoint.dCoorX = std::stod(tokens[5]);
		tCenterPoint.dCoorY = std::stod(tokens[6]);
		tCenterPoint.dCoorZ = std::stod(tokens[7]);
		tStartPoint.dCoorX = std::stod(tokens[8]);
		tStartPoint.dCoorY = std::stod(tokens[9]);
		tStartPoint.dCoorZ = std::stod(tokens[10]);
		tEndPoint.dCoorX = std::stod(tokens[11]);
		tEndPoint.dCoorY = std::stod(tokens[12]);
		tEndPoint.dCoorZ = std::stod(tokens[13]);
		tStartVerLineDir.dDirX = std::stod(tokens[14]);
		tStartVerLineDir.dDirY = std::stod(tokens[15]);
		tStartVerLineDir.dDirZ = std::stod(tokens[16]);
		tEndVerLineDir.dDirX = std::stod(tokens[17]);
		tEndVerLineDir.dDirY = std::stod(tokens[18]);
		tEndVerLineDir.dDirZ = std::stod(tokens[19]);
		nStartPointType = std::stod(tokens[20]);
		nEndPointType = std::stod(tokens[21]);
		nIsDoubleWelding = std::stod(tokens[22]);
		nStartCornerType = std::stod(tokens[23]);
		nEndCornerType = std::stod(tokens[24]);
		nWeldSize = std::stod(tokens[25]);
		dStartHoleRadiusSize = std::stod(tokens[26]);
		dEndHoleRadiusSize = std::stod(tokens[27]);
		nRobotSelect = std::stod(tokens[28]);
		dTheoryLength = std::stod(tokens[29]);
		dAngle = std::stod(tokens[30]);

		double dIncludeAngle = m_XiAlgorithm->VectorsInnerAngle(tStartVerLineDir.dDirX, tStartVerLineDir.dDirY, tStartVerLineDir.dDirZ, 0.0, 0.0, 1.0);

		tLineSegment.tStartPoint.dCoorX = tStartPoint.dCoorX;
		tLineSegment.tStartPoint.dCoorY = tStartPoint.dCoorY;
		tLineSegment.tStartPoint.dCoorZ = tStartPoint.dCoorZ;
		tLineSegment.tEndPoint.dCoorX = tEndPoint.dCoorX;
		tLineSegment.tEndPoint.dCoorY = tEndPoint.dCoorY;
		tLineSegment.tEndPoint.dCoorZ = tEndPoint.dCoorZ;
		tLineSegment.nPartNo = nPlateNo;
		vtAllIdealLineSegments.push_back(tLineSegment);

		if (fabs(tEndPoint.dCoorZ - tStartPoint.dCoorZ) < 10.0 && nIsArc == 0 && fabs(dIncludeAngle - 45.0) < 5.0)
		{
			vtIdealLineSegments.push_back(tLineSegment);
		}
	}

	inputFile.close();

	//while (EOF != fscanf(pfInput, "%d %d %d %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %d %d %d %lf %lf %lf %d %lf %lf",
	//	&nPlateNo, &nIsArc, &nClockWise, &dZSide, &nIsLeft, &tCenterPoint.dCoorX, &tCenterPoint.dCoorY, &tCenterPoint.dCoorZ, &tStartPoint.dCoorX,
	//	&tStartPoint.dCoorY, &tStartPoint.dCoorZ, &tEndPoint.dCoorX, &tEndPoint.dCoorY, &tEndPoint.dCoorZ, &tStartVerLineDir.dDirX, &tStartVerLineDir.dDirY,
	//	&tStartVerLineDir.dDirZ, &tEndVerLineDir.dDirX, &tEndVerLineDir.dDirY, &tEndVerLineDir.dDirZ, &nStartPointType,
	//	&nEndPointType, &nIsDoubleWelding, &nStartCornerType, &nEndCornerType, &nWeldSize, &dStartHoleRadiusSize, &dEndHoleRadiusSize,
	//	&nRobotSelect, &dTheoryLength,&dAngle))
	//{
	//		double dIncludeAngle = m_XiAlgorithm->VectorsInnerAngle(tStartVerLineDir.dDirX, tStartVerLineDir.dDirY, tStartVerLineDir.dDirZ, 0.0, 0.0, 1.0);
	//		tLineSegment.tStartPoint.dCoorX = tStartPoint.dCoorX;
	//		tLineSegment.tStartPoint.dCoorY = tStartPoint.dCoorY;
	//		tLineSegment.tStartPoint.dCoorZ = tStartPoint.dCoorZ;
	//		tLineSegment.tEndPoint.dCoorX = tEndPoint.dCoorX;
	//		tLineSegment.tEndPoint.dCoorY = tEndPoint.dCoorY;
	//		tLineSegment.tEndPoint.dCoorZ = tEndPoint.dCoorZ;
	//		tLineSegment.nPartNo = nPlateNo;
	//		vtAllIdealLineSegments.push_back(tLineSegment);
	//		if (fabs(tEndPoint.dCoorZ - tStartPoint.dCoorZ) < 10.0 && nIsArc == 0 && fabs(dIncludeAngle - 45.0) < 5.0)
	//		{
	//			vtIdealLineSegments.push_back(tLineSegment);
	//		}
	//}
	//fclose(pfInput);

	int nPartNo = 0;
	int nLastPartNo = 0;
	int nLastWeldLineNo = 0;
	T_ALGORITHM_POINT tPoint;

	pfInput = fopen(SaveRoute_weldline_point.c_str(), "r");/////李伟生成焊道点云坐标信息作为输入
	while (fscanf(pfInput, "%d%d%lf%lf%lf", &nPartNo, &nWeldLineNo, &tPoint.dCoorX, &tPoint.dCoorY, &tPoint.dCoorZ) > 0)
	{
		if (nPartNo == nLastPartNo && nLastWeldLineNo == nWeldLineNo)
		{
			vtRealLineSegmentPoints.push_back(tPoint);
			nLastWeldLineNo = nWeldLineNo;
			nLastPartNo = nPartNo;
		}
		else if (nPartNo == nLastPartNo && nLastWeldLineNo != nWeldLineNo)
		{
			vvtRealLineSegmentPoints.push_back(vtRealLineSegmentPoints);

			vtRealLineSegmentPoints.clear();
			nLastWeldLineNo = nWeldLineNo;
			nLastPartNo = nPartNo;
		}
		else if (nPartNo != nLastPartNo)
		{
			vvtRealLineSegmentPoints.push_back(vtRealLineSegmentPoints);
			vvvtRealLineSegmentPoints.push_back(vvtRealLineSegmentPoints);

			vtRealLineSegmentPoints.clear();
			vvtRealLineSegmentPoints.clear();

			vtRealLineSegmentPoints.push_back(tPoint);
			nLastWeldLineNo = nWeldLineNo;
			nLastPartNo = nPartNo;
		}
	}

	vvtRealLineSegmentPoints.push_back(vtRealLineSegmentPoints);
	vvvtRealLineSegmentPoints.push_back(vvtRealLineSegmentPoints);
	fclose(pfInput);

	std::vector<T_ALGORITHM_LINESEGMENT> vtRealSegment;
	vtRealSegment.clear();
	T_ALGORITHM_LINESEGMENT tRealLineSegment;


	for (nWeldLineNo = 0; nWeldLineNo < vvvtRealLineSegmentPoints[nWeldPartNo].size(); nWeldLineNo++)
	{
		T_ALGORITHM_POINT_2D tStartPoint, tEndPoint;

		tStartPoint.x = tRealLineSegment.tStartPoint.dCoorX = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][0].dCoorX;
		tStartPoint.y = tRealLineSegment.tStartPoint.dCoorY = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][0].dCoorY;
		tRealLineSegment.tStartPoint.dCoorZ = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][0].dCoorZ;

		tEndPoint.x = tRealLineSegment.tEndPoint.dCoorX = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo].size() - 1].dCoorX;
		tEndPoint.y = tRealLineSegment.tEndPoint.dCoorY = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo].size() - 1].dCoorY;
		tRealLineSegment.tEndPoint.dCoorZ = vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo][vvvtRealLineSegmentPoints[nWeldPartNo][nWeldLineNo].size() - 1].dCoorZ;

		double dLength = m_XiAlgorithm->CalDisPointToPoint(tStartPoint, tEndPoint);

		if (fabs(tRealLineSegment.tEndPoint.dCoorZ - tRealLineSegment.tStartPoint.dCoorZ) < dLength / 500.0 * 20.0)
		{
			vtRealSegment.push_back(tRealLineSegment);
		}
	}

	std::string save_file_name_RealSegment = "RealSegment_" + std::to_string(nWeldPartNo) + ".txt";
	std::string save_file_name_TransAllIdealMergeSegment = "TransAllIdealMergeSegment_" + std::to_string(nWeldPartNo) + ".txt";

	const char* strName1 = save_file_name_RealSegment.c_str();
	const char* strName2 = save_file_name_TransAllIdealMergeSegment.c_str();

	FILE* pf1 = fopen(strName1, "w");
	for (int nId = 0; nId < vtRealSegment.size(); nId++)
	{
		fprintf(pf1, "%4d %4d %11.3lf%11.3lf%11.3lf\n", nId, vtRealSegment[nId].nPartNo, vtRealSegment[nId].tStartPoint.dCoorX, vtRealSegment[nId].tStartPoint.dCoorY, vtRealSegment[nId].tStartPoint.dCoorZ);
		fprintf(pf1, "%4d %4d %11.3lf%11.3lf%11.3lf\n", nId, vtRealSegment[nId].nPartNo, vtRealSegment[nId].tEndPoint.dCoorX, vtRealSegment[nId].tEndPoint.dCoorY, vtRealSegment[nId].tEndPoint.dCoorZ);
	}
	fclose(pf1);

	//std::vector<T_ALGORITHM_LINESEGMENT> vtTestTransIdealLineSegments;
	//vtTestTransIdealLineSegments.clear();
	//m_XiAlgorithm->TransLineSegments(vtIdealLineSegments, dCorasrMatrix, vtTestTransIdealLineSegments);///将理想模型数据转到实际点云上 查看结果

	//pf1 = fopen(strName3, "w");
	//for (int nId = 0; nId < vtTestTransIdealLineSegments.size(); nId++)
	//{
	//	fprintf(pf1, "%4d %4d %11.3lf%11.3lf%11.3lf\n", nId, vtTestTransIdealLineSegments[nId].nPartNo, vtTestTransIdealLineSegments[nId].tStartPoint.dCoorX, vtTestTransIdealLineSegments[nId].tStartPoint.dCoorY, vtTestTransIdealLineSegments[nId].tStartPoint.dCoorZ);
	//	fprintf(pf1, "%4d %4d %11.3lf%11.3lf%11.3lf\n", nId, vtTestTransIdealLineSegments[nId].nPartNo, vtTestTransIdealLineSegments[nId].tEndPoint.dCoorX, vtTestTransIdealLineSegments[nId].tEndPoint.dCoorY, vtTestTransIdealLineSegments[nId].tEndPoint.dCoorZ);
	//}
	//fclose(pf1);

	std::vector<T_ALGORITHM_LINESEGMENT> vtIdealTransSegment;
	double dMatchError;

	std::vector<double> Error_Group;
	dMatchError = m_XiAlgorithm->GetCoraseToFineMatch(vtIdealLineSegments, vtRealSegment, dCorasrMatrix, dFineMatrix, vtIdealTransSegment, Error_Group);////调用精匹配函数 （dTransMatrix 李伟粗匹配矩阵） （dFineMatrix 精匹配矩阵）

	m_XiAlgorithm->TransLineSegments(vtAllIdealLineSegments, dFineMatrix, vtAllTransIdealLineSegments);///将理想模型数据转到实际点云上 查看结果

	std::cout << dMatchError << std::endl;
	if (dMatchError > 20.0)
	{
		std::cout << "误差过大请检查" << std::endl;
	}

	std::ofstream outfile;
	outfile.open("Error_Group" + std::to_string(nWeldPartNo) + ".txt");
	for (size_t i = 0; i < Error_Group.size(); i++)
	{
		outfile << Error_Group[i] << std::endl;
	}
	outfile.close();

	int total = 0;
	for (size_t i = 0; i < Error_Group.size(); i++)
	{
		if (Error_Group[i] > 80)
		{
			total++;
		}
	}

	if (total > 4)
	{
		MessageBox(NULL, std::string(ModelFileName + std::string("\n模型与工件立板数量存在误差(模型与工件不符)\n请人工检查模型与工件是否对应")).c_str(), "错误", MB_OK);
	}

	FILE* pf = fopen(strName2, "w");
	for (int nId = 0; nId < vtAllTransIdealLineSegments.size(); nId++)
	{
		fprintf(pf, "%4d %4d %11.3lf%11.3lf%11.3lf\n", nId, vtAllTransIdealLineSegments[nId].nPartNo, vtAllTransIdealLineSegments[nId].tStartPoint.dCoorX, vtAllTransIdealLineSegments[nId].tStartPoint.dCoorY, vtAllTransIdealLineSegments[nId].tStartPoint.dCoorZ);
		fprintf(pf, "%4d %4d %11.3lf%11.3lf%11.3lf\n", nId, vtAllTransIdealLineSegments[nId].nPartNo, vtAllTransIdealLineSegments[nId].tEndPoint.dCoorX, vtAllTransIdealLineSegments[nId].tEndPoint.dCoorY, vtAllTransIdealLineSegments[nId].tEndPoint.dCoorZ);
	}
	fclose(pf);

	return true;
}

bool RemoveStringInFile(CString sFileName, std::string sRemovedString)
{
	//读取文件内容
	std::ifstream inFile(sFileName); // 打开文件
	if (!inFile)
	{
		return false;
	}
	std::ostringstream buffer;
	buffer << inFile.rdbuf(); // 读取整个文件到 buffer
	auto str = buffer.str(); // 文件内容
	inFile.close(); // 关闭文件

	//移除指定字符串
	std::size_t found = str.find(sRemovedString);
	auto length = sRemovedString.length();
	while (found != std::string::npos)
	{
		str.replace(found, length, "");
		found = str.find(sRemovedString);
	}

	//重写文件
	std::ofstream outFile(sFileName); // 打开文件
	if (!inFile)
	{
		return false;
	}
	outFile << str << std::endl;
	outFile.close();
	return true;
}

XUI::MesBox g_cMesBox;

XUI::MesBox& GetNaturalPop()
{
	return g_cMesBox;
}


/**************************** 线程相关操作 End ****************************/