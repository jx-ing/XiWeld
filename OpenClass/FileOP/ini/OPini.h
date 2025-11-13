// OPini.h: interface for the COPini class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_OPINI_H__7449A144_FBE5_4D4B_BD76_520FC78545A4__INCLUDED_)
#define AFX_OPINI_H__7449A144_FBE5_4D4B_BD76_520FC78545A4__INCLUDED_

#include ".\Apps\PLib\BasicFunc\Const.h"

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

typedef enum tagTextCodeType
{
    TextUnkonw = -1,
    TextANSI = 0,
    TextUTF8 = 1,
    TextUNICODE = 2,
    TextUNICODE_BIG = 3
}TextCodeType;

class COPini
{
public:
    COPini();
    virtual ~COPini();

	bool  CheckExists(CString fileName, CString sectionName, CString key);
	bool  CheckExists(CString key);

	BOOL  SetFileName(CString fileName);
	BOOL  SetFileName(bool bCheck, CString fileName);
	BOOL  SetSectionName(CString sectionName);

	BOOL  WriteString(CString key,CString value);
	BOOL  WriteString(CString key,double value, unsigned int unDecimalDigits = 6);
	BOOL  WriteString(CString key,int value);
	BOOL  WriteString(CString key,long value);
	BOOL  WriteString(CString key,bool value);
	BOOL  WriteString(CString key1, CString key2, T_ANGLE_PULSE tPulse, T_ANGLE_PULSE tIsRead = T_ANGLE_PULSE(1, 1, 1, 1, 1, 1, 1, 1, 1));
	BOOL  WriteString(CString key1, CString key2, T_ROBOT_COORS tCoord, T_ROBOT_COORS tIsRead = T_ROBOT_COORS(1, 1, 1, 1, 1, 1, 1, 1, 1));

	DWORD ReadString (CString key,char value[]);
    DWORD ReadString (CString key,double *value);
    DWORD ReadString (CString key,float *value);
    DWORD ReadString (CString key,int *value);
    DWORD ReadString (CString key,long *value);
	DWORD ReadString (CString key,long long *value);
	DWORD ReadString (CString key,unsigned long *value);
    DWORD ReadString (CString key,CString &value);
    DWORD ReadString (CString key,CString *value);
	DWORD ReadString (CString key,bool *value, bool bCheck = true);
	DWORD ReadString (CString key1, CString key2, T_ANGLE_PULSE &tPulse, T_ANGLE_PULSE tIsRead = T_ANGLE_PULSE(1, 1, 1, 1, 1, 1, 1, 1, 1));
	DWORD ReadString (CString key1, CString key2, T_ROBOT_COORS &tCoord, T_ROBOT_COORS tIsRead = T_ROBOT_COORS(1, 1, 1, 1, 1, 1, 1, 1, 1));

	DWORD ReadString(bool bCheck, CString key, bool *value);
	DWORD ReadString(bool bCheck, CString key, int *value);
	DWORD ReadString(bool bCheck, CString key, long *value);
	DWORD ReadString(bool bCheck, CString key, double *value);
	DWORD ReadString(bool bCheck, CString key, CString &value);
	DWORD ReadString(bool bCheck, CString key, CString *value);

	DWORD ReadAddString(CString key, bool *value, bool init_value);//init_value:Ä¬ÈÏÖµ
	DWORD ReadAddString(CString key, int* value, int init_value);
	DWORD ReadAddString(CString key, int* value, long init_value);
	DWORD ReadAddString(CString key, double *value, double init_value);
	DWORD ReadAddString(CString key, CString &value, CString init_value);

//private:
    CString m_fileName;
    CString m_sectionName;
    static BOOL  WriteString(CString fileName, CString sectionName, CString key, CString value);
    static DWORD ReadString(CString fileName, CString sectionName, CString key, char value[]);
    void CheckRead(CString key, DWORD nReturnValue, bool bCheck = true);
    void CheckFileEncodeType(CString fileName);
};

#endif // !defined(AFX_OPINI_H__7449A144_FBE5_4D4B_BD76_520FC78545A4__INCLUDED_)
