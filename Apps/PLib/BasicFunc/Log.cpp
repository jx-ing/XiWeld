// Log.cpp: implementation of the CLog class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Log.h"
#include "io.h"
#include "Apps/PLib/ExLock/MyCriticalSection.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//OPC日志
CLog g_cOPCLog(OPC_LOG_PATH);
void WriteOPCLog(CString sLog)
{
	g_cOPCLog.Write(sLog);
}
void WriteOPCLog(char* format, ...)
{
	va_list args;
	va_start(args, format);
	char s[1000];
	vsprintf_s(s, 1000, format, args);
	va_end(args);

	g_cOPCLog.Write(s);
}
//////////////////////////////////////////////////////////////////////////////////////////////////

CLog  m_cMainLog(SYSTEM_LOG_PATH);
CMyCriticalSection g_cLogLock;

void WriteLog( CString sLog )
{
	g_cLogLock.Lock();
	m_cMainLog.Write(sLog);
	g_cLogLock.UnLock();
}

void WriteLog( char *format, ... )
{
	g_cLogLock.Lock();
    va_list args;
    va_start(args, format);
    char s[1000];
    vsprintf_s(s, 1000, format, args);
    va_end(args);
    
	m_cMainLog.Write(s);
	g_cLogLock.UnLock();
}

void WritePulseLog(CString sLog, T_ANGLE_PULSE tPulse)
{
	g_cLogLock.Lock();
	m_cMainLog.WritePulseLog(sLog, tPulse);
	g_cLogLock.UnLock();
}

void WritePosLog(CString sLog, T_ROBOT_COORS tPos)
{
	g_cLogLock.Lock();
	m_cMainLog.WritePosLog(sLog, tPos);
	g_cLogLock.UnLock();
}

CString GetLogName()
{
	return m_cMainLog.GetLogName();
}

CLog* ReturnCLog()
{
	return &m_cMainLog;
}

CLog::CLog(CString sFilePath, CString sFileName)
{
	m_sFilePath = sFilePath;
	m_sFileName = sFileName;
    TCHAR szFilePath[MAX_PATH + 1];
    GetModuleFileName(NULL, szFilePath, MAX_PATH);
    (_tcsrchr(szFilePath, _T('\\')))[1] = 0; //删除文件名，只获得路径
    CString m_sLogPath = szFilePath;
    if (sFilePath.GetLength() == 0)
    {
        m_sLogPath += SYSTEM_LOG_PATH;
    }
    else
    {
        m_sLogPath += sFilePath;
    }

	CheckFolder(m_sLogPath);

    // 获取文件个数
    CFileFind finder;
    int fileNum = 0;
    int fileSaveNum = 6;
 	COPini opini;
 	opini.SetFileName(DEBUG_INI);
 	opini.SetSectionName("Debug");
 	opini.ReadString("MaxLogNum", &fileSaveNum);
    CString strFileToFind = m_sLogPath + "*.txt";
    BOOL bWorking = finder.FindFile(strFileToFind);
    while (bWorking)
    {
        bWorking = finder.FindNextFile();
        fileNum ++ ;
    }

    // 删除多余备份日志
    if (fileNum > fileSaveNum)
    {
        for (int i=0; i<fileNum-fileSaveNum; i++)
        {
            if(finder.FindFile(strFileToFind))
            {
                if(finder.FindNextFile())
                {
                    CString strFileToDel = m_sLogPath + finder.GetFileName();
                    DeleteFile(strFileToDel);
                }
            }
        }
    }	

    // 创建今天的日志文件
    CString strYear,strMonth,strDay;
    
    CTime m_time = CTime::GetCurrentTime();
	m_nDay = m_time.GetDay();
    strYear.Format("%4d",m_time.GetYear());
    strMonth.Format("%.2d",m_time.GetMonth());
    strDay.Format("%.2d",m_time.GetDay());
    if (sFileName.GetLength() == 0)
    {
        m_strFileName = m_sLogPath + "log_" + strYear + strMonth + strDay + ".txt";
    }
    else
    {
        m_strFileName = m_sLogPath + sFileName + strYear + strMonth + strDay + ".txt";
    }
    
    if (_access(m_strFileName,0) != 0) // 文件不存在则创建文件
    {
		XI_fopen_s(&m_pFileLog, m_strFileName, "w", _SH_DENYNO);
		fprintf_s(m_pFileLog,"===============================================\n\n");
		fprintf_s(m_pFileLog,"		操作日志:%s\n\n", GetSoftwareVersion().GetBuffer());
		fprintf_s(m_pFileLog,"===============================================\n\n");
        
		fprintf_s(m_pFileLog,"		 "+strYear+"年"+strMonth+"月"+strDay+"日\n\n");
		fprintf_s(m_pFileLog,"===============================================\n\n");
    }
    else // 在底部添加数据
    {
		XI_fopen_s(&m_pFileLog, m_strFileName, "a+", _SH_DENYNO);
    }

	fprintf_s(m_pFileLog,"\n\n********************************\n\n");
    fflush(m_pFileLog);
}

CLog::~CLog()
{
	fprintf_s(m_pFileLog,"********************************\n\n");
    fflush(m_pFileLog);
    fclose(m_pFileLog);
}

void CLog::WritePulseLog(CString sLog, T_ANGLE_PULSE tPulse)
{
	CString sPulseLog;
	sPulseLog.Format("%ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld", tPulse.nSPulse, tPulse.nLPulse, tPulse.nUPulse, tPulse.nRPulse,
		tPulse.nBPulse, tPulse.nTPulse, tPulse.lBXPulse, tPulse.lBYPulse, tPulse.lBZPulse);
	sLog = sLog + sPulseLog;
	Write(sLog);
}

void CLog::WritePosLog(CString sLog, T_ROBOT_COORS tPos)
{
	CString sPosLog;
	sPosLog.Format("%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", tPos.dX, tPos.dY, tPos.dZ, tPos.dRX,
		tPos.dRY, tPos.dRZ, tPos.dBX, tPos.dBY, tPos.dBZ);
	sLog = sLog + sPosLog;
	Write(sLog);
}

BOOL CLog::Write(CString str)
{
	SYSTEMTIME tTime;
	GetLocalTime(&tTime);

	if (tTime.wDay != m_nDay)
	{
		UpdateLogFile();
	}

	CString strTime;
	strTime.Format("%.2d:%.2d:%.2d.%.3d  ",tTime.wHour, tTime.wMinute, tTime.wSecond, tTime.wMilliseconds);
	fprintf_s(m_pFileLog,strTime);
	fprintf_s(m_pFileLog,str);
	fprintf_s(m_pFileLog,"\n\n");
    fflush(m_pFileLog);

    return TRUE; 
}

BOOL CLog::Write( char *format, ... )
{
    va_list args;
    va_start( args, format );
    char s[1000];
	vsprintf_s(s, 1000, format, args);
    va_end(args);
    
    CString str;
    str.Format("%s", s);
    
    return Write(str);
}

CString CLog::GetLogName()
{
	return m_strFileName;
}

BOOL CLog::Read()
{
    ShellExecute(NULL,"open",m_strFileName,"Notepad.exe",NULL,SW_SHOW);//使用默认的程序打开，一般为记事本
    
    //另一种方法，可指定程序打开		
    /*	
    CString str;
    str="C:\\Program Files\\Notepad++\\notepad++.exe  ";
    str += strFilePath;
    
      char *szCmdline = new char[str.GetLength() + 1]; // 给缓存动态分配内存 
      strcpy_s(szCmdline, sizeof(szCmdline)/sizeof(szCmdline[0]), (const char*)str); // 拷贝str到缓存 
      
        PROCESS_INFORMATION pi;
        ZeroMemory(&pi,sizeof(PROCESS_INFORMATION));	
        STARTUPINFO si;
        ZeroMemory (&si, sizeof (STARTUPINFO));     //初始化
        si.cb = sizeof (STARTUPINFO);
        si.wShowWindow=SW_SHOW;   
        si.dwFlags=STARTF_USESHOWWINDOW; 
        ::CreateProcess (NULL, szCmdline, NULL,
        NULL, FALSE, NORMAL_PRIORITY_CLASS, NULL, NULL, &si, &pi);
        delete szCmdline;
    */	
    return TRUE; 
}

void CLog::SetFilePath( CString sFilePath )
{
    
}

void CLog::SetFileName( CString sFileName )
{
    
}

void CLog::UpdateLogFile()
{
	CString sFilePath = m_sFilePath;
	CString sFileName = m_sFileName;
	fprintf_s(m_pFileLog, "********************************\n\n");
	fflush(m_pFileLog);
	fclose(m_pFileLog);


	TCHAR szFilePath[MAX_PATH + 1];
	GetModuleFileName(NULL, szFilePath, MAX_PATH);
	(_tcsrchr(szFilePath, _T('\\')))[1] = 0; //删除文件名，只获得路径
	CString m_sLogPath = szFilePath;
	if (sFilePath.GetLength() == 0)
	{
		m_sLogPath += SYSTEM_LOG_PATH;
	}
	else
	{
		m_sLogPath += sFilePath;
	}

	CheckFolder(sFilePath);
	// 获取文件个数
	CFileFind finder;
	int fileNum = 0;
	int fileSaveNum = 30;
	CString strFileToFind = m_sLogPath + "*.txt";
	BOOL bWorking = finder.FindFile(strFileToFind);
	while (bWorking)
	{
		bWorking = finder.FindNextFile();
		fileNum++;
	}

	// 删除多余备份日志
	if (fileNum > fileSaveNum)
	{
		for (int i = 0; i < fileNum - fileSaveNum; i++)
		{
			if (finder.FindFile(strFileToFind))
			{
				if (finder.FindNextFile())
				{
					CString strFileToDel = m_sLogPath + finder.GetFileName();
					DeleteFile(strFileToDel);
				}
			}
		}
	}

	// 创建今天的日志文件
	CString strYear, strMonth, strDay;

	CTime m_time = CTime::GetCurrentTime();
	m_nDay = m_time.GetDay();
	strYear.Format("%4d", m_time.GetYear());
	strMonth.Format("%.2d", m_time.GetMonth());
	strDay.Format("%.2d", m_time.GetDay());
	if (sFileName.GetLength() == 0)
	{
		m_strFileName = m_sLogPath + "log_" + strYear + strMonth + strDay + ".txt";
	}
	else
	{
		m_strFileName = m_sLogPath + sFileName + strYear + strMonth + strDay + ".txt";
	}

	if (_access(m_strFileName, 0) != 0) // 文件不存在则创建文件
	{
		XI_fopen_s(&m_pFileLog, m_strFileName, "w", _SH_DENYNO);
		fprintf_s(m_pFileLog, "===============================================\n\n");
		fprintf_s(m_pFileLog, "		操作日志:%s\n\n", GetSoftwareVersion().GetBuffer());
		fprintf_s(m_pFileLog, "===============================================\n\n");

		fprintf_s(m_pFileLog, "		 " + strYear + "年" + strMonth + "月" + strDay + "日\n\n");
		fprintf_s(m_pFileLog, "===============================================\n\n");
	}
	else // 在底部添加数据
	{
		XI_fopen_s(&m_pFileLog, m_strFileName, "a+", _SH_DENYNO);
	}

	fprintf_s(m_pFileLog, "\n\n********************************\n\n");
	fflush(m_pFileLog);
}
