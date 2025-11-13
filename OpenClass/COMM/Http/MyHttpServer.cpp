#include "stdafx.h"
#include "MyHttpServer.h"

static int s_signo;
static void signal_handler(int signo) {
	s_signo = signo;
}

static const char *s_root_dir = ".";
static const char *s_listening_address = "http://127.0.0.1:8017";
static const char *s_enable_hexdump = "no";
static const char *s_ssi_pattern = "#.html";




CHttpService::CHttpService()
{
	mg_mgr_init(&m_Mgr);
}

CHttpService::~CHttpService(void)
{
	CloseHttpServer();
}

bool CHttpService::OpenHttpServer(const char *acListeningAddress /*= "http://127.0.0.1:8017"*/, const  char *EnableHexdump /*= "no"*/, mg_event_handler_t fn /*= NULL*/, void *fn_data /*= NULL*/)
{
	if ((c = mg_http_listen(&m_Mgr, acListeningAddress, fn, fn_data)) == NULL)
	{
		printf("Http·þÎñÆ÷¿ªÆôÊ§°Ü£¬¼àÌýµØÖ·£º%s", acListeningAddress);
		exit(EXIT_FAILURE);
		return false;
	}
	if (mg_casecmp(s_enable_hexdump, "yes") == 0) c->is_hexdumping = 1;
	HANDLE hThreadConnectSocketServer;
	UINT threadConnectSocketServerId;
	hThreadConnectSocketServer = (HANDLE)_beginthreadex(NULL, 0, ThreadHttpServer, this, 0, &threadConnectSocketServerId);

	return true;
}

bool CHttpService::CloseHttpServer()
{
	m_nExit = 1;
	long long lTime = XI_clock();
	while (m_nExit != 2)
	{
		Sleep(100);
		if ((XI_clock() - lTime) > 2000)
		{
			return false;
		}
	}
	return true;
}

std::string GetHeaderData(mg_http_message *hm, const  char *name)
{
	for (int n = 0; n < MG_MAX_HTTP_HEADERS; n++)
	{
		char *acName;
		acName = new char[hm->headers[n].name.len+1];
		_memccpy(acName, hm->headers[n].name.ptr, hm->headers[n].name.len, hm->headers[n].name.len);
		acName[hm->headers[n].name.len] = '\0';
		if (0 == _strcmpi(acName, name))
		{
			char *acVal;
			acVal = new char[hm->headers[n].value.len + 1];
			_memccpy(acVal, hm->headers[n].value.ptr, hm->headers[n].value.len, hm->headers[n].value.len);
			acVal[hm->headers[n].value.len] = '\0';
			std::string str = acVal;
			delete acVal;
			return str;
		}
	}
	return "null";
}


UINT WINAPI CHttpService::ThreadHttpServer(void *pParam)
{
	CHttpService *pMyParam = ((CHttpService*)pParam);
	pMyParam->HttpServer();
	return 0;
}

void CHttpService::HttpServer()
{
	while (0 == m_nExit) mg_mgr_poll(&m_Mgr, 1000);
	mg_mgr_free(&m_Mgr);
	m_nExit = 2;
}