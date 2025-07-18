// Log.h: interface for the CLog class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_LOG_H__E67DF3BD_FBFB_4CD7_AEEA_773AFFC3CC22__INCLUDED_)
#define AFX_LOG_H__E67DF3BD_FBFB_4CD7_AEEA_773AFFC3CC22__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000



void WriteLog(CString sLog);
void WriteLog(char *format, ...);
void WritePulseLog(CString sLog, T_ANGLE_PULSE tPulse);
void WritePosLog(CString sLog, T_ROBOT_COORS tPos);

void WriteOPCLog(CString sLog);
void WriteOPCLog(char* format, ...);

CString GetLogName();

class CLog  
{
public:
    CLog(CString sFilePath = "", CString sFileName = "");
    virtual ~CLog();
    BOOL Read();
    BOOL Write(CString str);
    BOOL Write(char *format, ...);
	void WritePulseLog(CString sLog, T_ANGLE_PULSE tPulse);
	void WritePosLog(CString sLog, T_ROBOT_COORS tPos);
	CString GetLogName();

    void SetFilePath(CString sFilePath);
    void SetFileName(CString sFileName);

private:
	void UpdateLogFile();
	CString m_sFilePath, m_sFileName;
    FILE *m_pFileLog;
	CString m_strFileName;
    CString m_sLogPath;
	int m_nDay;
};

CLog* ReturnCLog();

#endif // !defined(AFX_LOG_H__E67DF3BD_FBFB_4CD7_AEEA_773AFFC3CC22__INCLUDED_)
