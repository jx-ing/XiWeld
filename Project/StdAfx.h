// stdafx.h : include file for standard system include files,
//  or project specific include files that are used frequently, but
//      are changed infrequently
//

#if !defined(AFX_STDAFX_H__018652C0_6AA9_4C4C_8AE9_E53914191F53__INCLUDED_)
#define AFX_STDAFX_H__018652C0_6AA9_4C4C_8AE9_E53914191F53__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define VC_EXTRALEAN		// Exclude rarely-used stuff from Windows headers

#pragma warning (disable:4996 4244)

#include "Apps/PLib/BasicFunc/targetver.h"

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // 某些 CString 构造函数将是显式的

	// 关闭 MFC 对某些常见但经常可放心忽略的警告消息的隐藏
#define _AFX_ALL_WARNINGS

#include <afxwin.h>         // MFC 核心组件和标准组件
#include <afxext.h>         // MFC 扩展
#include <afxdisp.h>        // MFC 自动化类
#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>           // MFC 对 Internet Explorer 4 公共控件的支持
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>             // MFC 对 Windows 公共控件的支持
#endif // _AFX_NO_AFXCMN_SUPPORT
#include <afxcontrolbars.h>     // 功能区和控件条的 MFC 支持

//设置界面风格
#if defined _M_IX86
#pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_IA64
#pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='ia64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif

#define IDX 0

	//#include <CommCtrl.h>
	//#pragma comment(lib,"comctl32.lib")
	//#pragma comment(linker,"/manifestdependency:\"type='Win32' name='microsoft.windows.common-controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
//#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.
#include "Apps/PLib/BasicFunc/BaseStruct.h"
#include "OpenClass/FileOP/ini/OPini.h"
#include "Apps/PLib/BasicFunc/Log.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string.h>
#include <fstream>
#include <sstream>
//#include <OpenNI.h>
#include <opencv2/core/core.hpp>
//#include <Eigen/Core>
//#include <Eigen/Geometry>
//#include <Eigen/SVD>
#include <afxcontrolbars.h>
#define CAPTIMES 6
#endif // !defined(AFX_STDAFX_H__018652C0_6AA9_4C4C_8AE9_E53914191F53__INCLUDED_)


