// OPini.cpp: implementation of the COPini class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include ".\OpenClass\FileOP\ini\opini.h"
#include <fstream>

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

/******************************************************************** 
    filename:   // OPini.cpp 
    file path:     
    file base:  // OPini 
    file ext:   // cpp 
    author:     // alantop 
    purpose:    // 读取INI文件。 
*********************************************************************/   
	

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

COPini::COPini()
{

}

COPini::~COPini()
{

}

  
bool COPini::CheckExists(CString fileName, CString sectionName, CString key)
{
	char ch[255];
	DWORD num = GetPrivateProfileString(sectionName, key, NULL, ch, 255, fileName);
	if (num == FALSE)
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool COPini::CheckExists(CString key)
{
	char ch[255];
	DWORD num = GetPrivateProfileString(m_sectionName, key, NULL, ch, 255, m_fileName);
	if (num == FALSE)
	{
		return false;
	}
	else
	{
		return true;
	}
}

/***************************************************************************** 
Function:       //  
Description:    // 写字符串到INI文件 
Calls:          //  
Called By:      //  
Table Accessed: //  
Table Updated:  //  
Input:          //  
Output:         //  
Return:         // 成功返回真，失败返回假.失败后，可用DWORD GetLastError(VOID) 
                   查询失败原因。 
Others:         //  
author:         // alantop 
******************************************************************************/  
/*
void error(LPSTR lpszFunction)   
{   
    CHAR szBuf[80];   
    DWORD dw = GetLastError();   
    sprintf(szBuf, "%s failed: GetLastError returned %u/n",lpszFunction, dw);   
    MessageBox(NULL, szBuf, "Error", MB_OK);   
    ExitProcess(dw);   
}
*/
DWORD COPini::ReadString(CString fileName,CString sectionName,CString key,char value[])  
{  
	return ::GetPrivateProfileString(sectionName,key,NULL,value,255,fileName);
}

DWORD COPini::ReadString(CString key, bool *value, bool bCheck)
{
	char ch[255];
	int nValue;
	DWORD num = GetPrivateProfileString(m_sectionName, key, NULL, ch, 255, m_fileName);
	nValue = atoi(ch);
	if (nValue == 1)
	{
		*value = true;
	}
	else
	{
		*value = false;
	}
	CheckRead(key, num, bCheck);
	return num;
}

DWORD COPini::ReadString(CString key, unsigned long *value)
{
	char ch[255];
	DWORD num = GetPrivateProfileString(m_sectionName, key, NULL, ch, 255, m_fileName);
	*value = atol(ch);
	CheckRead(key, num);
	return num;
}

DWORD COPini::ReadString(CString key, long long *value)
{
	char ch[255];
	DWORD num = GetPrivateProfileString(m_sectionName, key, NULL, ch, 255, m_fileName);
	*value = atol(ch);
	CheckRead(key, num);
	return num;
}

DWORD COPini::ReadString(CString key1, CString key2, T_ANGLE_PULSE &tPulse, T_ANGLE_PULSE tIsRead)
{
	DWORD bRtn = TRUE;
	if (tIsRead.nSPulse > 0)
	{
		bRtn = bRtn && ReadString(key1 + "S" + key2, &tPulse.nSPulse);
	}
	if (tIsRead.nLPulse > 0)
	{
		bRtn = bRtn && ReadString(key1 + "L" + key2, &tPulse.nLPulse);
	}
	if (tIsRead.nUPulse > 0)
	{
		bRtn = bRtn && ReadString(key1 + "U" + key2, &tPulse.nUPulse);
	}
	if (tIsRead.nRPulse > 0)
	{
		bRtn = bRtn && ReadString(key1 + "R" + key2, &tPulse.nRPulse);
	}
	if (tIsRead.nBPulse > 0)
	{
		bRtn = bRtn && ReadString(key1 + "B" + key2, &tPulse.nBPulse);
	}
	if (tIsRead.nTPulse > 0)
	{
		bRtn = bRtn && ReadString(key1 + "T" + key2, &tPulse.nTPulse);
	}
	if (tIsRead.lBXPulse > 0)
	{
		bRtn = bRtn && ReadString(key1 + "BX" + key2, &tPulse.lBXPulse);
	}
	if (tIsRead.lBYPulse > 0)
	{
		bRtn = bRtn && ReadString(key1 + "BY" + key2, &tPulse.lBYPulse);
	}
	if (tIsRead.lBZPulse > 0)
	{
		bRtn = bRtn && ReadString(key1 + "BZ" + key2, &tPulse.lBZPulse);
	}

	return bRtn;
}

DWORD COPini::ReadString(CString key1, CString key2, T_ROBOT_COORS &tCoord, T_ROBOT_COORS tIsRead)
{
	DWORD bRtn = TRUE;
	if (tIsRead.dX > 0)
	{
		bRtn = bRtn && ReadString(key1 + "X" + key2, &tCoord.dX);
	}
	if (tIsRead.dY > 0)
	{
		bRtn = bRtn && ReadString(key1 + "Y" + key2, &tCoord.dY);
	}
	if (tIsRead.dZ > 0)
	{
		bRtn = bRtn && ReadString(key1 + "Z" + key2, &tCoord.dZ);
	}
	if (tIsRead.dRX > 0)
	{
		bRtn = bRtn && ReadString(key1 + "RX" + key2, &tCoord.dRX);
	}
	if (tIsRead.dRY > 0)
	{
		bRtn = bRtn && ReadString(key1 + "RY" + key2, &tCoord.dRY);
	}
	if (tIsRead.dRZ > 0)
	{
		bRtn = bRtn && ReadString(key1 + "RZ" + key2, &tCoord.dRZ);
	}
	if (tIsRead.dBX > 0)
	{
		bRtn = bRtn && ReadString(key1 + "BX" + key2, &tCoord.dBX);
	}
	if (tIsRead.dBY > 0)
	{
		bRtn = bRtn && ReadString(key1 + "BY" + key2, &tCoord.dBY);
	}
	if (tIsRead.dBZ > 0)
	{
		bRtn = bRtn && ReadString(key1 + "BZ" + key2, &tCoord.dBZ);
	}

	return bRtn;
}

DWORD COPini::ReadString(bool bCheck, CString key, long *value)
{
	char ch[255];
	DWORD num = GetPrivateProfileString(m_sectionName, key, NULL, ch, 255, m_fileName);
	if (num)
	{
		*value = atol(ch);
	}
	CheckRead(key, num, bCheck);

	return num;
}

void COPini::CheckRead(CString key, DWORD nReturnValue, bool bCheck)
{
	if (!bCheck)
	{
		return;
	}
	if (nReturnValue == FALSE)
	{
		CString str;
		str = "Error:" + m_fileName + "文件" + m_sectionName + "项" + key + "参数加载失败！";
		XiMessageBox(str);
	}
}

void COPini::CheckFileEncodeType(CString fileName)
{
    unsigned char headBuf[3] = { 0 };
    TextCodeType type = TextUnkonw;

    wchar_t *FilePath;
    FilePath = fileName.AllocSysString();
    FILE  *file;
    errno_t  error = _wfopen_s(&file, FilePath, L"r+");

    if (error != 0)
    {
        CString str;
        str = "Error:" + m_fileName + "文件打开失败！";
		XiMessageBox(str);
        return;
    }

    fseek(file, 0, SEEK_SET);
    fread(headBuf, 3, 1, file);

    if (headBuf[0] == 0xEF && headBuf[1] == 0xBB && headBuf[2] == 0xBF)     //utf8-bom 文件开头：FF BB BF
    {
        type = TextUTF8;
        fseek(file, 0L, SEEK_END);
        long len = ftell(file);
        unsigned char *Buf = new unsigned char[len];      
        fseek(file, 0L, SEEK_SET);
        int nNo = 0;
        for (int i = 0;i < len; i++)
        {
            fread(&(Buf[i]), 1, 1, file);
            if (Buf[i] == '\n')
            {
                nNo++;
            }
        }    
        fclose(file);

        FILE  *file;
        _wfopen_s(&file, FilePath, L"w+");
        fwrite(&(Buf[3]),1, len - nNo - 3, file);
        delete[] Buf;
    }
    else if (headBuf[0] == 0xFF && headBuf[1] == 0xFE)      //小端Unicode  文件开头：FF FE 
    {
        type = TextUNICODE;
        CString str;
        str = "Error:" + m_fileName + "文件格式错误！";
		XiMessageBox(str);
    }
    else if (headBuf[0] == 0xFE && headBuf[1] == 0xFF)  //大端Unicode  文件开头：FE FF
    {
        type = TextUNICODE_BIG;
        CString str;
        str = "Error:" + m_fileName + "文件格式错误！";
		XiMessageBox(str);
    }
    else
    {
        type = TextANSI;    //ansi或者unf8 无bom
    }

    fclose(file);
}

BOOL COPini::WriteString(CString fileName,CString sectionName,CString key,CString value)
{  
	return ::WritePrivateProfileString(sectionName,key,value,fileName);  
}   


BOOL COPini::WriteString(CString key, bool value)
{
	CString str;
	int nValue = 0;
	if (value)
	{
		nValue = 1;
	}
	str.Format("%ld", nValue);
	return ::WritePrivateProfileString(m_sectionName, key, str, m_fileName);
}

BOOL COPini::WriteString(CString key, long value)
{
	CString str;
	str.Format("%ld", value);
	return ::WritePrivateProfileString(m_sectionName, key, str, m_fileName);
}

BOOL COPini::WriteString(CString key1, CString key2, T_ANGLE_PULSE tPulse, T_ANGLE_PULSE tIsRead)
{
	BOOL bRtn = TRUE;
	if (tIsRead.nSPulse > 0)
	{
		bRtn = bRtn && WriteString(key1 + "S" + key2, tPulse.nSPulse);
	}
	if (tIsRead.nLPulse > 0)
	{
		bRtn = bRtn && WriteString(key1 + "L" + key2, tPulse.nLPulse);
	}
	if (tIsRead.nUPulse > 0)
	{
		bRtn = bRtn && WriteString(key1 + "U" + key2, tPulse.nUPulse);
	}
	if (tIsRead.nRPulse > 0)
	{
		bRtn = bRtn && WriteString(key1 + "R" + key2, tPulse.nRPulse);
	}
	if (tIsRead.nBPulse > 0)
	{
		bRtn = bRtn && WriteString(key1 + "B" + key2, tPulse.nBPulse);
	}
	if (tIsRead.nTPulse > 0)
	{
		bRtn = bRtn && WriteString(key1 + "T" + key2, tPulse.nTPulse);
	}
	if (tIsRead.lBXPulse > 0)
	{
		bRtn = bRtn && WriteString(key1 + "BX" + key2, tPulse.lBXPulse);
	}
	if (tIsRead.lBYPulse > 0)
	{
		bRtn = bRtn && WriteString(key1 + "BY" + key2, tPulse.lBYPulse);
	}
	if (tIsRead.lBZPulse > 0)
	{
		bRtn = bRtn && WriteString(key1 + "BZ" + key2, tPulse.lBZPulse);
	}
	return bRtn;
}

BOOL COPini::WriteString(CString key1, CString key2, T_ROBOT_COORS tCoord, T_ROBOT_COORS tIsRead)
{
	BOOL bRtn = TRUE;
	if (tIsRead.dX > 0)
	{
		bRtn = bRtn && WriteString(key1 + "X" + key2, tCoord.dX);
	}
	if (tIsRead.dY > 0)
	{
		bRtn = bRtn && WriteString(key1 + "Y" + key2, tCoord.dY);
	}
	if (tIsRead.dZ > 0)
	{
		bRtn = bRtn && WriteString(key1 + "Z" + key2, tCoord.dZ);
	}
	if (tIsRead.dRX > 0)
	{
		bRtn = bRtn && WriteString(key1 + "RX" + key2, tCoord.dRX);
	}
	if (tIsRead.dRY > 0)
	{
		bRtn = bRtn && WriteString(key1 + "RY" + key2, tCoord.dRY);
	}
	if (tIsRead.dRZ > 0)
	{
		bRtn = bRtn && WriteString(key1 + "RZ" + key2, tCoord.dRZ);
	}
	if (tIsRead.dBX > 0)
	{
		bRtn = bRtn && WriteString(key1 + "BX" + key2, tCoord.dBX);
	}
	if (tIsRead.dBY > 0)
	{
		bRtn = bRtn && WriteString(key1 + "BY" + key2, tCoord.dBY);
	}
	if (tIsRead.dBZ > 0)
	{
		bRtn = bRtn && WriteString(key1 + "BZ" + key2, tCoord.dBZ);
	}
	return bRtn;
}

BOOL COPini::SetFileName(CString fileName)
{
	m_fileName = fileName;
	if (!CheckFileExists(fileName))
	{
		XiMessageBox("Error:" + fileName + "文件不存在，请退出程序，添加文件后重启！");
	}
    CheckFileEncodeType(m_fileName);
	return TRUE;
}

BOOL COPini::SetFileName(bool bCheck, CString fileName)
{
	m_fileName = fileName;
	if (!bCheck)
	{
		return TRUE;
	}
	if (!CheckFileExists(fileName))
	{
		XiMessageBox("Error:" + fileName + "文件不存在，请退出程序，添加文件后重启！");
	}
	CheckFileEncodeType(m_fileName);
	return TRUE;
}

BOOL COPini::SetSectionName(CString sectionName)
{
	m_sectionName = sectionName;
	return TRUE;
}

DWORD COPini::ReadString(CString key,char value[])
{
	DWORD num = ::GetPrivateProfileString(m_sectionName, key, NULL, value, 255, m_fileName);
	CheckRead(key, num);
	return num;
}

BOOL  COPini::WriteString(CString key,CString value)
{
	return ::WritePrivateProfileString(m_sectionName,key,value,m_fileName);
} 

DWORD COPini::ReadString (CString key,double *value)
{
	char ch[255];
	DWORD num = GetPrivateProfileString(m_sectionName,key,NULL,ch,255,m_fileName);
	*value = atof(ch);
	CheckRead(key, num);
	return num;
}

DWORD COPini::ReadString( CString key,float *value )
{
    char ch[255];
    DWORD num = GetPrivateProfileString(m_sectionName,key,NULL,ch,255,m_fileName);
    *value = (float)atof(ch);
	CheckRead(key, num);
    return num;
}

BOOL  COPini::WriteString(CString key,double value, unsigned int unDecimalDigits)
{
	CString str1;
	CString str2;
	str2.Format("%d", unDecimalDigits);
	str2 = "%." + str2 + "lf";
	str1.Format(str2, value);
	return ::WritePrivateProfileString(m_sectionName,key,str1,m_fileName);
} 


DWORD COPini::ReadString (CString key,int *value)
{
	char ch[255];
	DWORD num = GetPrivateProfileString(m_sectionName,key,NULL,ch,255,m_fileName);
	*value = atoi(ch);
	CheckRead(key, num);
	return num;
}

DWORD COPini::ReadString( CString key,long *value )
{
    char ch[255];
    DWORD num = GetPrivateProfileString(m_sectionName,key,NULL,ch,255,m_fileName);
    *value = atol(ch);
	CheckRead(key, num);
    return num;
}

DWORD COPini::ReadString( CString key,CString &value )
{
    char ch[255];
    DWORD num = GetPrivateProfileString(m_sectionName,key,NULL,ch,255,m_fileName);
    value.Format("%s", ch);
	CheckRead(key, num);
    return num;
}

DWORD COPini::ReadString(CString key, CString* value)
{
	char ch[255];
	DWORD num = GetPrivateProfileString(m_sectionName, key, NULL, ch, 255, m_fileName);
	value->Format("%s", ch);
	CheckRead(key, num);
	return num;
}

BOOL  COPini::WriteString(CString key,int value)
{
	CString str;
	str.Format("%ld",value);
	return ::WritePrivateProfileString(m_sectionName,key,str,m_fileName);
} 

DWORD COPini::ReadString(bool bCheck, CString key, bool *value)
{
	char ch[255];
	int nValue;
	DWORD num = GetPrivateProfileString(m_sectionName, key, NULL, ch, 255, m_fileName);
	nValue = atoi(ch);
	if (num)
	{
		if (nValue == 1)
		{
			*value = true;
		}
		else
		{
			*value = false;
		}
	}
	CheckRead(key, num, bCheck);
	return num;
}

DWORD COPini::ReadString(bool bCheck, CString key, CString &value)
{
	char ch[255];
	DWORD num = GetPrivateProfileString(m_sectionName, key, NULL, ch, 255, m_fileName);
	if (num)
	{
		value.Format("%s", ch);
	}
	CheckRead(key, num, bCheck);
	return num;
}
DWORD COPini::ReadString(bool bCheck, CString key, CString *value)
{
	char ch[255];
	DWORD num = GetPrivateProfileString(m_sectionName, key, NULL, ch, 255, m_fileName);
	if (num)
	{
		(*value).Format("%s", ch);
	}
	CheckRead(key, num, bCheck);

	return num;
}
DWORD COPini::ReadString(bool bCheck, CString key, double *value)
{
	char ch[255];
	DWORD num = GetPrivateProfileString(m_sectionName, key, NULL, ch, 255, m_fileName);
	if (num)
	{
		*value = atof(ch);
	}
	CheckRead(key, num, bCheck);
	return num;
}
DWORD COPini::ReadString(bool bCheck, CString key, int *value)
{
	char ch[255];
	DWORD num = GetPrivateProfileString(m_sectionName, key, NULL, ch, 255, m_fileName);
	if (num)
	{
		*value = atoi(ch);
	}
	CheckRead(key, num, bCheck);

	return num;
}
DWORD COPini::ReadAddString(CString key, bool *value, bool init_value)
{
	if (FALSE == ReadString(false, key, value))
	{
		*value = init_value;
		WriteString(key, init_value);
		return FALSE;
	}
	return TRUE;
}

DWORD COPini::ReadAddString(CString key, int *value, int init_value)
{
	if (FALSE == ReadString(false, key, value))
	{
		*value = init_value;
		WriteString(key, init_value);
		return FALSE;
	}
	return TRUE;
}

DWORD COPini::ReadAddString(CString key, int* value, long init_value)
{
	if (FALSE == ReadString(false, key, value))
	{
		*value = init_value;
		WriteString(key, init_value);
		return FALSE;
	}
	return TRUE;
}

DWORD COPini::ReadAddString(CString key, CString &value, CString init_value)
{
	if (FALSE == ReadString(false, key, value))
	{
		value = init_value;
		WriteString(key, init_value);
		return FALSE;
	}
	return TRUE;
}

DWORD COPini::ReadAddString(CString key, double *value, double init_value)
{
	if (FALSE == ReadString(false, key, value))
	{
		*value = init_value;
		WriteString(key, init_value);
		return FALSE;
	}
	return TRUE;
}