// CXiGrooveRobot.cpp : Defines the class behaviors for the application.
//

#include "stdafx.h"
#include ".\Project\XiGrooveRobot.h"
#include "XiGrooveRobotMainDlg.h"
#include ".\OpenClass\FileOP\ini\opini.h"
#include <VersionHelpers.h>
#include <ShellScalingAPI.h>
#pragma comment(lib, "Shcore.lib")    

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//#define SKINSPACE _T("/SPATH:")

/////////////////////////////////////////////////////////////////////////////
// CXiGrooveRobotApp

BEGIN_MESSAGE_MAP(CXiGrooveRobotApp, CWinApp)
	//{{AFX_MSG_MAP(CXiGrooveRobotApp)
		// NOTE - the ClassWizard will add and remove mapping macros here.
		//    DO NOT EDIT what you see in these blocks of generated code!
	//}}AFX_MSG
//	ON_COMMAND(ID_HELP, CWinApp::OnHelp)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CXiGrooveRobotApp construction

CXiGrooveRobotApp::CXiGrooveRobotApp()
{
	// TODO: add construction code here,
	// Place all significant initialization in InitInstance
}

/////////////////////////////////////////////////////////////////////////////
// The one and only CXiGrooveRobotApp object

CXiGrooveRobotApp theApp;



/////////////////////////////////////////////////////////////////////////////
// CXiGrooveRobotApp initialization

BOOL CXiGrooveRobotApp::InitInstance()
{
	//不加这个库的话，程序会提醒SetProcessDpiAwareness无法解析的外部符号
	SetProcessDpiAwareness(PROCESS_SYSTEM_DPI_AWARE); //在程序刚开始运行的地方加入该语句

	AfxEnableControlContainer();
	
	AfxOleInit();

	// Standard initialization
	// If you are not using these features and wish to reduce the size
	//  of your final executable, you should remove from the following
	//  the specific initialization routines you do not need.
#if _MSC_VER <= 1200 // MFC 6.0 or earlier
#ifdef _AFXDLL
	Enable3dControls();			// Call this when using MFC in a shared DLL
#else
	Enable3dControlsStatic();	// Call this when linking to MFC statically
#endif
#endif

	//已修改
	COPini opini;
	opini.SetFileName(DEBUG_INI);
	opini.SetSectionName("Debug");
	CString str;
	opini.ReadAddString("Language", str, NULL);
	str = Utf8ToGBK(str);
	XUI::Languge::GetInstance().loadAllLanguge();
	XUI::Languge::GetInstance().chioceLanguge(str.GetBuffer());//Русский язык
	

	HANDLE   m_hMutex=CreateMutex(NULL,TRUE,m_pszAppName);   
	if(GetLastError()==ERROR_ALREADY_EXISTS)   
	{   
		XiMessageBox("已经在运行了!");
		return   FALSE;
	} 
	
	CXiGrooveRobotMainDlg dlg;
	m_pMainWnd = &dlg;
	int nResponse = dlg.DoModal();
	if (nResponse == IDOK)
	{
		// TODO: Place code here to handle when the dialog is
		//  dismissed with OK
	}
	else if (nResponse == IDCANCEL)
	{
		// TODO: Place code here to handle when the dialog is
		//  dismissed with Cancel
	}

	// Since the dialog has been closed, return FALSE so that we exit the
	//  application, rather than start the application's message pump.
	return FALSE;
}

int CXiGrooveRobotApp::ExitInstance() 
{
	// TODO: Add your specialized code here and/or call the base class
//	skinppExitSkin();
	return CWinApp::ExitInstance();
}
