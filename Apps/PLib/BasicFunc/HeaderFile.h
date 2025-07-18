#pragma once

//ϵͳ
#include <MAP>
#include "afxwin.h"
#include "windows.h"
#include <vector>
#include <math.h>
#include <FSTREAM>
#include <iomanip>
#include <afxsock.h>

//ͨ��
#include ".\Apps\PLib\BasicFunc\Const.h"
#include ".\OpenClass\FileOP\ini\opini.h"
#include "Log.h"
#include "FullScreen.h"
#include ".\OpenClass\FileOP\Excel\OpExcel.h"
#include ".\Apps\PLib\ExLock\MyCriticalSection.h"
#include ".\Apps\PLib\Others\ShowTrack.h"

//�㷨���
#include "CommonAlgorithm.h"
#include "XiPlaneTypes.h "
#include "AbsCoorTransLib.h"
#include "CommonAlgorithm.h"
#include "GeometryFeature.h"
#include "XiPlaneTypes.h"

//�Ӿ����
#include "Dll_XiCV_Image_Processing.h"
#include "FindCornner.h"

//�������
#include ".\Apps\PLib\LeisaiCtrl\ServoMotorDriver.h"
#include ".\Apps\PLib\DahengCam\DHGigeImageCapture.h"
#include ".\Apps\PLib\KinectCam\KinectControl.h"

//�������
#include ".\Apps\PLib\YaskawaRobot\RobotDriverAdaptor.h"
#include ".\Apps\PLib\CtrlUnit\CContralUnit.h"

//����
#include ".\Project\DHCameraCtrlDlg.h"
#include ".\Project\RobotCtrlDlg.h"

//�궨��
#ifndef MESSAGE_BOX
#define MESSAGE_BOX(data) {CString __str;\
__str.Format("Function:[%s] Line:[%d]	\n----%s",__FUNCTION__, __LINE__, data);\
XiMessageBoxOk(__str);\
}
#endif

#ifndef WRITE_LOG
#define WRITE_LOG(data) {CString __str;\
__str.Format("Function:[%s], Line:[%d]	\ndata: %s",__FUNCTION__, __LINE__, data);\
WriteLog(__str);}
#endif // !WRITE_LOG

#ifndef CHECK_BOOL_RTN
#define CHECK_BOOL_RTN(data, str) if(data != TRUE)\
{\
WRITE_LOG(str);\
return FALSE;\
}
#endif

#ifndef CHECK_BOOL_BOX
#define CHECK_BOOL_BOX(data, str) if(data != TRUE)\
{\
CString __str;\
__str.Format("Function:[%s] Line:[%d]	\n----%s",__FUNCTION__, __LINE__, data);\
XiMessageBoxOk(__str);\
return FALSE;\
}
#endif

#ifndef CHECK_INT_RTN
#define CHECK_INT_RTN(data, str) if(data != 0)\
{\
WRITE_LOG(str);\
return data;\
}
#endif