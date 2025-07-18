// CXiGrooveRobot.h : main header file for the CXiGrooveRobot application
//

#if !defined(AFX_XIGROOVEROBOT_H__AF688AFD_3F31_420E_8928_2CD02B9862E4__INCLUDED_)
#define AFX_XIGROOVEROBOT_H__AF688AFD_3F31_420E_8928_2CD02B9862E4__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef __AFXWIN_H__
	#error include 'stdafx.h' before including this file for PCH
#endif

#include "res/resource.h"		// main symbols

/////////////////////////////////////////////////////////////////////////////
// CXiGrooveRobotApp:
// See XiGrooveRobot.cpp for the implementation of this class
//

class CXiGrooveRobotApp : public CWinApp
{
public:
	CXiGrooveRobotApp();

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CXiGrooveRobotApp)
	public:
	virtual BOOL InitInstance();
	virtual int ExitInstance();
	//}}AFX_VIRTUAL

// Implementation

	//{{AFX_MSG(CXiGrooveRobotApp)
		// NOTE - the ClassWizard will add and remove member functions here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

private:

private:
};


/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_XIGROOVEROBOT_H__AF688AFD_3F31_420E_8928_2CD02B9862E4__INCLUDED_)
