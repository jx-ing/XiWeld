// HttpClient.cpp
#include "stdafx.h"
#include ".\OpenClass\COMM\Http\HttpTools.h"

#define  BUFFER_SIZE       1024

#define  NORMAL_CONNECT             INTERNET_FLAG_KEEP_CONNECTION
#define  SECURE_CONNECT                NORMAL_CONNECT | INTERNET_FLAG_SECURE
#define  NORMAL_REQUEST             INTERNET_FLAG_RELOAD | INTERNET_FLAG_DONT_CACHE 
#define  SECURE_REQUEST             NORMAL_REQUEST | INTERNET_FLAG_SECURE | INTERNET_FLAG_IGNORE_CERT_CN_INVALID

CHttpClient::CHttpClient(LPCTSTR strAgent)
{
	m_pSession = new CInternetSession(strAgent);
	m_pConnection = NULL;
	m_pFile = NULL;
}


CHttpClient::~CHttpClient(void)
{
	Clear();
	if (NULL != m_pSession)
	{
		m_pSession->Close();
		delete m_pSession;
		m_pSession = NULL;
	}
}

void CHttpClient::Clear()
{
	if (NULL != m_pFile)
	{
		m_pFile->Close();
		delete m_pFile;
		m_pFile = NULL;
	}

	if (NULL != m_pConnection)
	{
		m_pConnection->Close();
		delete m_pConnection;
		m_pConnection = NULL;
	}
}


bool CHttpClient::DownloadSaveFiles(char* url, char* strSaveFile) {//下载文件并保存为新文件名
	bool ret = false;
	CInternetSession Sess("lpload");
	Sess.SetOption(INTERNET_OPTION_CONNECT_TIMEOUT, 2000); //2秒的连接超时
	Sess.SetOption(INTERNET_OPTION_SEND_TIMEOUT, 2000); //2秒的发送超时
	Sess.SetOption(INTERNET_OPTION_RECEIVE_TIMEOUT, 2000); //2秒的接收超时
	Sess.SetOption(INTERNET_OPTION_DATA_SEND_TIMEOUT, 2000); //2秒的发送超时
	Sess.SetOption(INTERNET_OPTION_DATA_RECEIVE_TIMEOUT, 2000); //2秒的接收超时
	DWORD dwFlag = INTERNET_FLAG_TRANSFER_BINARY | INTERNET_FLAG_DONT_CACHE | INTERNET_FLAG_RELOAD;

	CHttpFile* cFile = NULL;
	char* pBuf = NULL;
	int        nBufLen = 0;
	do {
		try {
			cFile = (CHttpFile*)Sess.OpenURL(url, 1, dwFlag);
			DWORD dwStatusCode;
			cFile->QueryInfoStatusCode(dwStatusCode);
			if (dwStatusCode == HTTP_STATUS_OK) {
				pBuf = (char*)malloc(8 * BUFFER_SIZE + 8);
				ZeroMemory(pBuf, 8 * BUFFER_SIZE + 8);
				CFile file(strSaveFile, CFile::modeCreate | CFile::modeWrite);
				while (1) {
					//每次下载8K
					int n = cFile->Read(pBuf, 8*BUFFER_SIZE);

					file.Write(pBuf, n);

					//接收完成退出循环
					if (n <= 0) break;
				}

				file.Close();
				ret = true;
			}
		}
		catch (...) {
			break;//
		}
	} while (0);

	//释放缓存
	if (pBuf) {
		free(pBuf);
		pBuf = NULL;
		nBufLen = 0;
	}

	//关闭下载连接
	if (cFile) {
		cFile->Close();
		Sess.Close();
		delete cFile;
	}
	return ret;
}

int CHttpClient::ExecuteRequest(int strMethod, LPCTSTR strUrl, CString strPostData, CString &strResponse)
{
	int result = FAILURE;
	//WCHAR* wPostData = strPostData.GetBuffer();
	CString strServer;
	CString strObject;
	DWORD dwServiceType;
	INTERNET_PORT nPort;


	AfxParseURL(strUrl, dwServiceType, strServer, strObject, nPort);
	if (AFX_INET_SERVICE_HTTP != dwServiceType && AFX_INET_SERVICE_HTTPS != dwServiceType)
	{
		return FAILURE;
	}

	try
	{
		m_pConnection = m_pSession->GetHttpConnection(strServer,
			dwServiceType == AFX_INET_SERVICE_HTTP ? NORMAL_CONNECT : SECURE_CONNECT,
			nPort);
		m_pFile = m_pConnection->OpenRequest(strMethod, strObject,
			NULL, 1, NULL, NULL,
			(dwServiceType == AFX_INET_SERVICE_HTTP ? NORMAL_REQUEST : SECURE_REQUEST));

		/*设置请求相关参数*/
		m_pFile->AddRequestHeaders("Accept: */*,application/json");//accept请求报头域，表示客户端接受哪些类型的信息
		m_pFile->AddRequestHeaders("Accept-Charset:UTF8");
		m_pFile->AddRequestHeaders("Accept-Language: zh-cn;q=0.8,en;q=0.6,ja;q=0.4");
		m_pFile->AddRequestHeaders("Content-Type:application/json");//content为实体报头域，格式及编码

		//m_pFile->SendRequest(NULL, 0, (LPVOID)(LPCTSTR)strPostData, strPostData == NULL ? 0 : _tcslen(strPostData));

		/*请求body内容先转为UTF-8编码，与服务端保持一致,cword为要发送内容*/
		char* cword; //ANSI指针
		if (strPostData.GetLength() > 0) {
			DWORD  num = strPostData.GetLength();

			// 				
			// 				
			// 				WideCharToMultiByte(CP_UTF8, 0, strPostData, -1, NULL, 0, NULL, NULL);//計算這個UNICODE实际由几个UTF-8字組成
			cword = (char*)calloc(num+1, sizeof(char));   //申请空间
			if (cword == NULL)                          //是否申请
			{
				free(cword);
			}
			memset(cword, 0, num * sizeof(char));     //初始化
			//			WideCharToMultiByte(CP_UTF8, 0, strPostData, -1, cword, num, NULL, NULL);
			strcpy(cword, strPostData);
			printf("content长度为%zd\n", strlen(cword));
			m_pFile->SendRequest(NULL, 0, cword, strlen(cword));//发送请求
			free(cword);
		}
		else {
			m_pFile->SendRequest(NULL, 0, NULL, 0);//发送请求
		}


		DWORD dwRet;
		m_pFile->QueryInfoStatusCode(dwRet);//查询执行状态
		printf("HTTP_STATUS_code:%d\n", dwRet);
		if (dwRet == HTTP_STATUS_OK) {//http请求执行失败
			result = SUCCESS;
		}

		/*保存http响应*/
		char szChars[BUFFER_SIZE + 1] = { 0 };
		string strRawResponse = "";
		UINT nReaded = 0;
		while ((nReaded = m_pFile->Read((void*)szChars, BUFFER_SIZE)) > 0)
		{
			szChars[nReaded] = '\0';
			strRawResponse += szChars;
			memset(szChars, 0, BUFFER_SIZE + 1);
		}

		/*utf8转unicode*/
		int unicodeLen = MultiByteToWideChar(CP_UTF8, 0, strRawResponse.c_str(), -1, NULL, 0);
		WCHAR* pUnicode = new WCHAR[unicodeLen + 1];
		memset(pUnicode, 0, (unicodeLen + 1) * sizeof(wchar_t));
		MultiByteToWideChar(CP_UTF8, 0, strRawResponse.c_str(), -1, pUnicode, unicodeLen);
		strResponse = pUnicode;//最终响应结果
		//TRACE(strResponse + L"");
		delete[]pUnicode;
		pUnicode = NULL;

		Clear();
	}
	catch (CInternetException* e)
	{
		Clear();
		DWORD dwErrorCode = e->m_dwError;
		e->Delete();

		DWORD dwError = GetLastError();

		printf("dwError = %d", dwError);

		strResponse = L"CInternetException\n";

		if (ERROR_INTERNET_TIMEOUT == dwErrorCode)
		{
			return OUTTIME;
		}
		else
		{
			return FAILURE;
		}
	}
	return result;
}

int CHttpClient::HttpGet(LPCTSTR strUrl, CString strPostData, CString &strResponse)
{
	return ExecuteRequest(CHttpConnection::HTTP_VERB_GET, strUrl, strPostData, strResponse);
}

int CHttpClient::HttpPost(LPCTSTR strUrl, CString strPostData, CString &strResponse)
{
	return ExecuteRequest(CHttpConnection::HTTP_VERB_POST, strUrl, strPostData, strResponse);
}
int CHttpClient::HttpPost(LPCTSTR strUrl, CString strPostData, CString& strResponse, map<CString, CString> vmQueryParam)
{
	CString strQueryParam;
	for (auto iter = vmQueryParam.begin(); iter != vmQueryParam.end(); iter++)
	{
		strQueryParam = strQueryParam + iter->first + "=" + iter->second + "&";
	}
	if (!strQueryParam.IsEmpty())
	{
		strQueryParam = "?" + strQueryParam;
		strQueryParam.Left(strQueryParam.GetLength() - 1);
		strQueryParam = strUrl + strQueryParam;
	//	XiMessageBoxOk(NULL, strQueryParam);
		return HttpPost(strQueryParam, strPostData, strResponse);
	}
	return HttpPost(strUrl, strPostData, strResponse);
}
int CHttpClient::HttpPut(LPCTSTR strUrl, CString strPostData, CString &strResponse)
{
	return ExecuteRequest(CHttpConnection::HTTP_VERB_PUT, strUrl, strPostData, strResponse);
}
