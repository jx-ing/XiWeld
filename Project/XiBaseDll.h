#pragma once

#include <VECTOR>
#include <map>
#include <iostream>
#include <fstream>
#include <atlconv.h>

#define MAX_ALL_SECTION_SIZE 1024 * 1024
#define MAX_ALL_KEY_SIZE 1024 * 1024
#define MAX_VALUE_SIZE 1024 * 1024

//�������ؼ��ʡ�ֵ
#define INI_DATA std::map<CString, std::map<CString, CString>>

//�����ַ�����·�����ļ����ļ��в���
namespace XiBaseDll
{
	/************************************************************************/
	/*                                �ַ���                                */
	/************************************************************************/
	//�ַ����ָsToken�Ƕ���ַ�ʱ��ÿ���ַ����Ƿָ���
	std::vector<CString> TokenizeCString(CString sSrc, CString sToken);

	//Utf8תGBK
	CString Utf8ToGBK(const CString &sUtf8);

	//Utf8תGBK
	CString GBKToUtf8(const CString &sGBK);

	//�����ַ�����ָ�����ַ�
	CString ClearOtherCharacter(CString sSrc, CString sCharacter);

	//�����ַ����ڵĴ�Сд��ĸ�����ֺ��»���
	CString OnlyLettersNumbersUnderscores(CString sSrc);

	//�ж��ַ����Ƿ��ǲ�������
	bool CheckCStringType_bool(CString sData);

	//�ж��ַ����Ƿ�����������
	bool CheckCStringType_long(CString sData);

	//�ж��ַ����Ƿ��Ǹ�������
	bool CheckCStringType_double(CString sData);

	//�ж��ַ����Ƿ����޷�����������
	bool CheckCStringType_unsignedlong(CString sData);

	//����16�������ַ���������unsigned long long������
	bool CStringToULongLong(unsigned long long &ullNum, CString strNum);

	bool ULongLongToCString(CString &strNum, unsigned long long ullNum);

	/************************************************************************/
	/*                                 ·��                                 */
	/************************************************************************/
	//��ȡ��ǰ���̵�·��
	CString GetModuleFilePath();

	//������·���л�ȡ�ļ���
	bool GetFileNameFromPath(CString sFilePath, CString &sFileName, bool bSuffix);

	//������·���л�ȡ·��
	bool GetFilePathFromPath(CString sFilePath, CString &sFileName);

	//������·����ȥ����׺��
	bool GetFilePathFromPathWithoutSuffix(CString sFilePath, CString &sFileName);

	//ѡ���ļ�����
	CString OpenFileDlg(CDialog *pFatherDlg, CString sFileType, CString sDefaultPath);
	bool OpenFileDlg(CDialog *pFatherDlg, CString sFileType, CString sDefaultPath, std::vector< CString> &vstrFileList);

	//ѡ���ļ���
	bool OpenFolder(CDialog *pFatherDlg, CString sDefaultPath, CString sTitle, CString &sPath);

	/************************************************************************/
	/*                               �����ļ�                               */
	/************************************************************************/
	//����ȫ��ini����
	INI_DATA LoadIni(CString sFileName);

	//��ȡһ������
	bool ReadIni(const CString &sFileName, const CString &sSection, const CString &sKey, CString &sValue);

	//д��һ������
	bool WriteIni(const CString &sFileName, const CString &sSection, const CString &sKey, const CString &sValue);

	//ɾ��һ������
	bool DelIniKey(const CString &sFileName, const CString &sSection, const CString &sKey);

	//ɾ��һ����
	bool DelIniSection(const CString &sFileName, const CString &sSection);

	/************************************************************************/
	/*                               ͨ���ļ�                               */
	/************************************************************************/
	//����ļ��Ƿ���ڣ���ѡ�񴴽����ļ��в�����Ҳ�ᴴ����
	bool CheckFileExists(CString sFileName, bool bCreate = false);

	//��ָ��·����Ѱ��ָ����չ����ȫ���ļ�
	bool FindFile(std::vector<CString> &vstrFileList, CString strPath, CString strFileType, bool bIncludeSubfolder = true, bool bReturnPathOrName = true);
	bool FindFile(std::vector<CString> &vstrFileList, CString strPath, std::vector<CString> vstrFileType, bool bIncludeSubfolder = true, bool bReturnPathOrName = true);
	
	//��ȫ���ļ���ȡ,Ĭ��ʹ��fopen_s�����б����ܣ�nShFlag��Ϊ_SH_DENYNO�൱��fopen
	errno_t XI_fopen_s(FILE **file, CString strFileName, _In_z_ const char * _Mode, int nShFlag = _SH_SECURE);

	/************************************************************************/
	/*                                �ļ���                                */
	/************************************************************************/
	//����ļ����Ƿ���ڣ��������򴴽�һ��
	long CheckFolder(CString strPath, bool bCreate = true);//1:�ļ��д��ڣ�0:�ļ��в����ڵ��Ѵ���/����Ҫ����������(����):����

	//���������ļ��е�ָ��·����
	long CopyFolder(CString strSrcFolderName, CString strDstPath, bool bCopySubfolder = true);

	//�����ļ����е��ļ�
	void DelFiles(CString directory_path);
}

static int CALLBACK BrowseCallbackProc(HWND hwnd, UINT uMsg, LPARAM lParam, LPARAM lpData);

