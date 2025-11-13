#pragma once
#include "mongoose.h"
#include "json.h"
#include "string.h"

class CHttpService
{
public:
	CHttpService();
	virtual ~CHttpService(void);

	bool OpenHttpServer(const char *acListeningAddress = "http://127.0.0.1:8017", const char *EnableHexdump = "no", mg_event_handler_t fn = NULL, void *fn_data = NULL);
	bool CloseHttpServer();
	
	

private:
	struct mg_mgr m_Mgr;
	struct mg_connection *c;

	static UINT WINAPI ThreadHttpServer(void *pParam);
	void HttpServer();
	BOOL m_nExit = 0;
};

std::string GetHeaderData(mg_http_message *hm, const  char *name);


