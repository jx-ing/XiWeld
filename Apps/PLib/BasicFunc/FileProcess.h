#pragma once
#include "afxwin.h"
#include <vector>
#include ".\OpenClass\FileOP\ini\opini.h"

typedef struct
{
	CString strKeyName;
	CString strKeyVal;
}T_INI_KEY;

typedef struct
{
	CString strSectionName;
	std::vector<T_INI_KEY> vtKey;
}T_INI_SECTION;

typedef struct
{
	CString strFileName;
	std::vector<T_INI_SECTION> vtSection;
}T_INI_FILE_INFO;

//std::vector<CString> TokenizeCString(CString sSrc, CString sToken);

std::vector<CString> GetTotalFilePath(CString strFileName, CString strSuffix = "*", bool bPath = true);
std::vector<CString> GetTotalFilePath(std::vector<CString> strFileName, CString strSuffix = "*");//�ݹ��㷨

std::vector<CString> DeleteTotalFilePathFolder(std::vector<CString> vstrFileName);
std::vector<CString> ScreenTotalFilePathSuffix(std::vector<CString> vstrFileName, std::vector<CString> vstrSuffix);
std::vector<CString> ScreenTotalFilePathSuffix(std::vector<CString> vstrFileName, CString strSuffix);

T_INI_FILE_INFO GetIniFileInfo(CString strFilePath);
T_INI_KEY GetKeyVal(CString strData);
T_INI_SECTION GetSectionVal(std::vector<CString> vtrData);
T_INI_FILE_INFO ReplaceIniFile(T_INI_FILE_INFO tNewFile, T_INI_FILE_INFO tOldFile);
void SaveIniFile(T_INI_FILE_INFO tFile);

//CString����
CString DeleteContinue(CString str, char cContinue);//ɾ�������ַ�
CString DeletePrefix(CString str, CString strPrefix);//ɾ��strPrefix֮ǰ�ַ�
CString DeleteSuffix(CString str);//ɾ����׺
CString DeletePath(CString str);//ɾ��·��
CString GetPath(CString str);//�õ�·��
CString DeleteDrackets(CString str);//ɾ������������
CString DeleteChinese(CString str, bool bDeleteAfter = false);//ɾ������
CString Replace(CString str, char cOld, char cNew);//�滻��cNew=NULLΪɾ��
int StringSimilarity(CString str1, CString str2, int nMode = 1);//nMode = 1��ǰ����nMode = 2�Ӻ���ǰ
